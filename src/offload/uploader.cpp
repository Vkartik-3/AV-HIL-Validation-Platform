/*
==============================================================================
SensorForge - Sealed-segment offload (implementation)
==============================================================================
*/

#include "sensorforge/offload/uploader.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

namespace sensorforge::offload {

namespace fs = std::filesystem;

const char * to_string(UploadStatus s)
{
  switch (s) {
    case UploadStatus::kPending: return "pending";
    case UploadStatus::kUploaded: return "uploaded";
    case UploadStatus::kFailed: return "failed";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// FilesystemDestination
// ---------------------------------------------------------------------------
bool FilesystemDestination::put(
  const std::string & local_path, const std::string & object_name)
{
  ++put_calls_;
  if (!available_.load()) {
    return false;
  }
  std::error_code ec;
  fs::create_directories(dir_, ec);

  const std::string final_path = (fs::path(dir_) / object_name).string();
  const std::string tmp_path = final_path + ".partial";

  // copy -> fsync -> atomic rename. A reader of dir_ never sees a partial
  // object: it exists under .partial until the rename publishes it whole.
  {
    std::ifstream in(local_path, std::ios::binary);
    if (!in) {
      return false;
    }
    std::ofstream out(tmp_path, std::ios::binary | std::ios::trunc);
    if (!out) {
      return false;
    }
    out << in.rdbuf();
    out.flush();
    if (!out.good()) {
      return false;
    }
  }
  const int fd = ::open(tmp_path.c_str(), O_RDONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
  fs::rename(tmp_path, final_path, ec);
  if (ec) {
    fs::remove(tmp_path, ec);
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Uploader
// ---------------------------------------------------------------------------
Uploader::Uploader(
  std::shared_ptr<Destination> dest, std::string wal_dir, UploaderConfig cfg)
: dest_(std::move(dest)), wal_dir_(std::move(wal_dir)), cfg_(std::move(cfg))
{
  if (cfg_.manifest_path.empty()) {
    cfg_.manifest_path = (fs::path(wal_dir_) / "offload_manifest.txt").string();
  }
  load_manifest();
}

Uploader::~Uploader()
{
  stop();
}

std::string Uploader::object_name_for(uint32_t segment_id)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "segment-%06u.sflog", segment_id);
  return buf;
}

void Uploader::load_manifest()
{
  std::ifstream in(cfg_.manifest_path);
  if (!in) {
    return;
  }
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    uint32_t id = 0;
    std::string state;
    if (ls >> id >> state) {
      manifest_[id] = (state == "uploaded") ? UploadStatus::kUploaded : UploadStatus::kPending;
    }
  }
}

bool Uploader::persist_manifest()
{
  // Durable: temp -> fsync -> atomic rename, same discipline as an upload.
  const std::string tmp = cfg_.manifest_path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out) {
      return false;
    }
    for (const auto & [id, st] : manifest_) {
      out << id << " " << to_string(st) << "\n";
    }
    out.flush();
    if (!out.good()) {
      return false;
    }
  }
  const int fd = ::open(tmp.c_str(), O_RDONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
  std::error_code ec;
  fs::rename(tmp, cfg_.manifest_path, ec);
  return !ec;
}

size_t Uploader::recover_pending()
{
  // The DIRECTORY is authoritative, not the manifest: a manifest that is stale,
  // truncated or missing must never cause a completed segment to be forgotten.
  size_t n = 0;
  std::error_code ec;
  if (!fs::exists(wal_dir_, ec)) {
    return 0;
  }
  std::vector<std::pair<uint32_t, std::string>> found;
  for (const auto & e : fs::directory_iterator(wal_dir_, ec)) {
    if (!e.is_regular_file()) {
      continue;
    }
    const std::string fn = e.path().filename().string();
    uint32_t id = 0;
    if (std::sscanf(fn.c_str(), "segment-%u.sflog", &id) == 1 && id != 0) {
      found.emplace_back(id, e.path().string());
    }
  }
  std::sort(found.begin(), found.end());

  std::lock_guard<std::mutex> lk(mtx_);
  // The highest-numbered segment may still be the live one being appended to;
  // only sealed (i.e. not-highest) segments are safe to offload on recovery.
  const size_t sealed_count = found.empty() ? 0 : found.size() - 1;
  for (size_t i = 0; i < sealed_count; ++i) {
    const auto & [id, path] = found[i];
    const auto it = manifest_.find(id);
    if (it != manifest_.end() && it->second == UploadStatus::kUploaded) {
      continue;
    }
    manifest_[id] = UploadStatus::kPending;
    if (queue_.size() < cfg_.max_queue) {
      queue_.push_back(Item{path, id, 0});
      ++n;
    }
  }
  recovered_.store(n);
  cv_.notify_all();
  return n;
}

void Uploader::start()
{
  if (running_.load()) {
    return;
  }
  recover_pending();
  {
    std::lock_guard<std::mutex> lk(mtx_);
    stop_ = false;
  }
  running_.store(true);
  worker_ = std::thread([this]() {worker();});
}

void Uploader::stop()
{
  if (!running_.load()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  running_.store(false);
  std::lock_guard<std::mutex> lk(mtx_);
  persist_manifest();
}

bool Uploader::enqueue(const std::string & path, uint32_t segment_id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  const auto it = manifest_.find(segment_id);
  if (it != manifest_.end() && it->second == UploadStatus::kUploaded) {
    ++duplicates_;
    return false;   // idempotent: already done
  }
  if (queue_.size() >= cfg_.max_queue) {
    // Bounded: reject the newest rather than grow. Recording continues; the
    // segment stays on local disk and recover_pending() will pick it up.
    ++rejected_;
    return false;
  }
  manifest_[segment_id] = UploadStatus::kPending;
  queue_.push_back(Item{path, segment_id, 0});
  ++enqueued_;
  cv_.notify_all();
  return true;
}

void Uploader::worker()
{
  uint32_t backoff_ms = cfg_.base_backoff_ms;
  for (;;) {
    Item item;
    {
      std::unique_lock<std::mutex> lk(mtx_);
      cv_.wait(lk, [this]() {return stop_ || !queue_.empty();});
      if (stop_ && queue_.empty()) {
        return;
      }
      if (queue_.empty()) {
        continue;
      }
      item = queue_.front();
      queue_.pop_front();
    }

    const bool ok = dest_->put(item.path, object_name_for(item.segment_id));

    std::unique_lock<std::mutex> lk(mtx_);
    if (ok) {
      manifest_[item.segment_id] = UploadStatus::kUploaded;
      ++uploaded_;
      healthy_ = true;
      backoff_ms = cfg_.base_backoff_ms;
      persist_manifest();
      lk.unlock();
      if (cfg_.delete_after_upload) {
        // Only after an acknowledged upload, and only if asked.
        std::error_code ec;
        fs::remove(item.path, ec);
      }
      continue;
    }

    ++failed_attempts_;
    healthy_ = false;
    ++item.attempts;
    if (cfg_.max_attempts != 0 && item.attempts >= cfg_.max_attempts) {
      manifest_[item.segment_id] = UploadStatus::kFailed;
      persist_manifest();
      continue;
    }
    // Requeue at the FRONT so ordering is preserved across retries.
    queue_.push_front(item);
    const uint32_t wait_ms = std::min(backoff_ms, cfg_.max_backoff_ms);
    backoff_ms = std::min<uint32_t>(backoff_ms * 2, cfg_.max_backoff_ms);
    cv_.wait_for(lk, std::chrono::milliseconds(wait_ms), [this]() {return stop_;});
    if (stop_) {
      return;
    }
  }
}

bool Uploader::wait_drained(uint32_t timeout_ms)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (queue_.empty()) {
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::lock_guard<std::mutex> lk(mtx_);
  return queue_.empty();
}

bool Uploader::is_uploaded(uint32_t segment_id) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  const auto it = manifest_.find(segment_id);
  return it != manifest_.end() && it->second == UploadStatus::kUploaded;
}

UploaderStats Uploader::stats() const
{
  UploaderStats s;
  s.enqueued = enqueued_.load();
  s.uploaded = uploaded_.load();
  s.failed_attempts = failed_attempts_.load();
  s.rejected_queue_full = rejected_.load();
  s.duplicates_skipped = duplicates_.load();
  s.recovered_on_start = recovered_.load();
  std::lock_guard<std::mutex> lk(mtx_);
  s.backlog = queue_.size();
  s.destination_healthy = healthy_;
  return s;
}

}  // namespace sensorforge::offload
