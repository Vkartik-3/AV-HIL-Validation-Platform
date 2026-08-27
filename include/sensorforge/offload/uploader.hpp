/*
==============================================================================
SensorForge - Sealed-segment offload
Part of the SensorForge AV HIL validation platform.

Offload operates on SEALED WAL SEGMENTS, which is what makes this tractable:
a sealed segment is immutable, self-describing, CRC-verified per record, and
named by a monotonic id. Re-uploading one is therefore idempotent by
construction, and resume needs nothing more than a record of which ids are done.

Guarantees:
  - Recording is NEVER blocked by offload. The writer's segment-sealed hook only
    enqueues a path; all I/O happens on the uploader's own thread.
  - The queue is BOUNDED. When full, the oldest pending entry is retained and
    the newest is rejected and counted -- a full queue must not grow memory.
  - A local segment is NEVER deleted before the destination acknowledges, and
    deletion is opt-in even then.
  - The manifest is durable (write temp -> fsync -> atomic rename) so a restart
    recovers the pending set.

FilesystemDestination is the primary implementation and performs
copy -> fsync -> atomic rename, so a reader of the destination directory never
observes a partially written object. No cloud SDK dependency exists or is
wanted here.
==============================================================================
*/

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace sensorforge::offload {

enum class UploadStatus { kPending, kUploaded, kFailed };

const char * to_string(UploadStatus s);

/// A destination an uploader can write sealed segments to.
class Destination
{
public:
  virtual ~Destination() = default;
  /// Transfer @p local_path so it appears atomically under @p object_name.
  /// Must be idempotent: re-uploading the same object must not duplicate it.
  virtual bool put(const std::string & local_path, const std::string & object_name) = 0;
  virtual std::string name() const = 0;
};

/**
 * @brief Writes into a directory with copy -> fsync -> atomic rename.
 *
 * A temp file is written and fsynced, then renamed into place. rename(2) within
 * a filesystem is atomic, so a concurrent reader sees either no object or the
 * complete one -- never a partial upload.
 */
class FilesystemDestination : public Destination
{
public:
  explicit FilesystemDestination(std::string dir)
  : dir_(std::move(dir)) {}

  bool put(const std::string & local_path, const std::string & object_name) override;
  std::string name() const override {return "filesystem:" + dir_;}

  /// Test hook: when set, put() fails without touching the destination.
  void set_available(bool a) {available_.store(a);}
  bool available() const {return available_.load();}
  uint64_t put_calls() const {return put_calls_.load();}

private:
  std::string dir_;
  std::atomic<bool> available_{true};
  std::atomic<uint64_t> put_calls_{0};
};

struct UploaderConfig
{
  size_t max_queue = 256;
  uint32_t base_backoff_ms = 50;
  uint32_t max_backoff_ms = 5000;
  uint32_t max_attempts = 0;      // 0 = retry indefinitely with capped backoff
  bool delete_after_upload = false;
  std::string manifest_path;      // defaults to <wal_dir>/offload_manifest.txt
};

struct UploaderStats
{
  uint64_t enqueued = 0;
  uint64_t uploaded = 0;
  uint64_t failed_attempts = 0;
  uint64_t rejected_queue_full = 0;
  uint64_t duplicates_skipped = 0;
  uint64_t recovered_on_start = 0;
  size_t backlog = 0;
  bool destination_healthy = true;
};

class Uploader
{
public:
  Uploader(std::shared_ptr<Destination> dest, std::string wal_dir, UploaderConfig cfg = {});
  ~Uploader();

  Uploader(const Uploader &) = delete;
  Uploader & operator=(const Uploader &) = delete;

  void start();
  void stop();

  /// Enqueue a sealed segment. Non-blocking; safe from the WAL writer's thread.
  bool enqueue(const std::string & path, uint32_t segment_id);

  /// Re-enqueue everything the manifest does not mark uploaded. Called by
  /// start(); exposed for tests.
  size_t recover_pending();

  /// Block until the backlog drains or @p timeout_ms elapses. Test helper.
  bool wait_drained(uint32_t timeout_ms);

  UploaderStats stats() const;
  bool is_uploaded(uint32_t segment_id) const;

private:
  struct Item
  {
    std::string path;
    uint32_t segment_id = 0;
    uint32_t attempts = 0;
  };

  void worker();
  void load_manifest();
  bool persist_manifest();
  static std::string object_name_for(uint32_t segment_id);

  std::shared_ptr<Destination> dest_;
  std::string wal_dir_;
  UploaderConfig cfg_;

  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<Item> queue_;
  std::map<uint32_t, UploadStatus> manifest_;
  bool stop_ = false;
  bool healthy_ = true;

  std::thread worker_;
  std::atomic<bool> running_{false};

  std::atomic<uint64_t> enqueued_{0};
  std::atomic<uint64_t> uploaded_{0};
  std::atomic<uint64_t> failed_attempts_{0};
  std::atomic<uint64_t> rejected_{0};
  std::atomic<uint64_t> duplicates_{0};
  std::atomic<uint64_t> recovered_{0};
};

}  // namespace sensorforge::offload
