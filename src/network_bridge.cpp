/*
==============================================================================
MIT License

Copyright (c) 2024 Ethan M Brown

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
==============================================================================
*/

#include "network_bridge/network_bridge.hpp"

#include <zstd.h>
#include <span>
#include <fstream>
#include <bit>

#include <rclcpp/serialization.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>

#include "network_bridge/subscription_manager_tf.hpp"
#include "network_interfaces/network_interface_base.hpp"

NetworkBridge::NetworkBridge(const std::string & node_name)
: Node(node_name),
  loader_("network_bridge", "network_bridge::NetworkInterface") {}

NetworkBridge::~NetworkBridge()
{
  shutdown();
}

void NetworkBridge::initialize()
{
  load_parameters();
  load_network_interface();
  network_interface_->open();
}

void NetworkBridge::shutdown()
{
  RCLCPP_INFO(this->get_logger(), "NetworkBridge: Shuting down");
  if (network_interface_) {
    network_interface_->close();
  }
  network_interface_.reset();

  network_check_timer_.reset();
  sub_mgrs_.clear();
  timers_.clear();
  publishers_.clear();

  if (fault_engine_) {
    fault_engine_->flush();
    fault_engine_.reset();
  }

  if (wal_writer_) {
    wal_writer_->close();
    wal_writer_.reset();
  }
}

void NetworkBridge::load_parameters()
{
  this->declare_parameter(
    "network_interface",
    std::string("network_bridge::UdpInterface"));
  this->get_parameter("network_interface", network_interface_name_);

  bool publish_stale_data;
  this->declare_parameter("publish_stale_data", false);
  this->get_parameter("publish_stale_data", publish_stale_data);
  // Defaults
  this->declare_parameter("default_rate", 5.0);
  this->declare_parameter("default_zstd_level", 3);

  float default_rate;
  int default_zstd_level;
  this->get_parameter("default_rate", default_rate);
  this->get_parameter("default_zstd_level", default_zstd_level);

  // Optional WAL replay recording. Empty (default) disables it.
  std::string wal_record_dir;
  this->declare_parameter("wal_record_dir", "");
  this->get_parameter("wal_record_dir", wal_record_dir);
  if (!wal_record_dir.empty()) {
    wal_writer_ = std::make_unique<sensorforge::replay::WalWriter>(wal_record_dir);
    RCLCPP_INFO(
      this->get_logger(), "Recording WAL replay log to %s", wal_record_dir.c_str());
  }

  // Optional transport-layer fault injection (Extension K). A single
  // param-configured rule; scenarios with multiple faults drive the scenario
  // runner instead. fault_type = none (default) disables injection.
  std::string fault_type;
  this->declare_parameter("fault_type", "none");
  this->get_parameter("fault_type", fault_type);
  if (fault_type != "none" && !fault_type.empty()) {
    namespace fx = sensorforge::faults;
    fx::FaultRule rule;
    rule.kind = fx::fault_kind_from_string(fault_type);
    rule.start_s = this->declare_parameter<double>("fault_start_s", 0.0);
    rule.duration_s = this->declare_parameter<double>("fault_duration_s", 1e9);
    rule.delay_ms = this->declare_parameter<double>("fault_delay_ms", 0.0);
    rule.drop_rate = this->declare_parameter<double>("fault_drop_rate", 0.0);
    rule.bandwidth_kbps = this->declare_parameter<double>("fault_bandwidth_kbps", 0.0);
    if (rule.kind != fx::FaultKind::kNone) {
      fault_engine_ = std::make_unique<fx::FaultEngine>(
        [this](const std::vector<uint8_t> & bytes) {
          if (network_interface_) {network_interface_->write(bytes);}
        });
      fault_engine_->add_rule(rule);
      RCLCPP_INFO(this->get_logger(), "Transport fault injection enabled: %s", fault_type.c_str());
    }
  }

  this->declare_parameter("publish_namespace", "");
  this->get_parameter("publish_namespace", publish_namespace_);

  if (!publish_namespace_.empty()) {
    if (publish_namespace_.front() != '/') {
      publish_namespace_.insert(0, "/");
    }
    if (publish_namespace_.back() == '/') {
      publish_namespace_.pop_back();
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Topics will be published under the namespace %s",
      publish_namespace_.c_str());
  }

  std::string subscribe_namespace;
  this->declare_parameter("subscribe_namespace", "");
  this->get_parameter("subscribe_namespace", subscribe_namespace);

  if (!subscribe_namespace.empty()) {
    if (subscribe_namespace.front() != '/') {
      subscribe_namespace.insert(0, "/");
    }
    if (subscribe_namespace.back() == '/') {
      subscribe_namespace.pop_back();
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Topics will be subscribed to under the namespace %s",
      subscribe_namespace.c_str());
  }

  // Load topics information
  this->declare_parameter<std::vector<std::string>>(
    "topics",
    std::vector<std::string>{});

  std::vector<std::string> topics;
  this->get_parameter("topics", topics);

  for (const auto & topic : topics) {
    std::string rate_param_name = topic + ".rate";
    std::string zstd_level_param_name = topic + ".zstd_level";
    std::string is_tf_param_name = topic + ".is_tf";
    bool is_tf = (topic == "/tf") || (topic == "tf") || (topic == "/tf_static") ||
      (topic == "tf_static");
    bool is_static_tf = is_tf && ((topic == "/tf_static") || (topic == "tf_static"));
    float rate = 1;
    int zstd_level = 3;


    this->declare_parameter<int>(zstd_level_param_name, default_zstd_level);
    // Add this parameter to force the tf nature if needed
    this->declare_parameter<bool>(is_tf_param_name, is_tf);
    this->declare_parameter<double>(rate_param_name, default_rate);
    this->get_parameter(is_tf_param_name, is_tf);
    this->get_parameter(rate_param_name, rate);
    this->get_parameter(zstd_level_param_name, zstd_level);

    if (is_tf) {
      // Add this parameter to force the static tf nature if needed
      std::string is_static_tf_param_name = topic + ".is_static_tf";
      this->declare_parameter<bool>(is_static_tf_param_name, is_static_tf);
      std::string tf_include_param_name = topic + ".include";
      std::string tf_exclude_param_name = topic + ".exclude";
      std::vector<std::string> tf_include, tf_exclude;
      this->declare_parameter(tf_include_param_name, tf_include);
      this->declare_parameter(tf_exclude_param_name, tf_exclude);

      this->get_parameter(is_static_tf_param_name, is_static_tf);
      this->get_parameter(tf_include_param_name, tf_include);
      this->get_parameter(tf_exclude_param_name, tf_exclude);

      std::shared_ptr<SubscriptionManagerTF> manager(new SubscriptionManagerTF(
          shared_from_this(), topic, subscribe_namespace,
          zstd_level, publish_stale_data, is_static_tf));
      if (!tf_include.empty()) {
        manager->set_include_pattern(tf_include);
      }
      if (!tf_exclude.empty()) {
        manager->set_exclude_pattern(tf_exclude);
      }
      manager->setup_subscription();
      sub_mgrs_.push_back(std::static_pointer_cast<SubscriptionManager>(manager));

      // TODO: specialize this
      int ms = static_cast<int>(1000.0 / rate);
      auto timer = this->create_wall_timer(
        std::chrono::milliseconds(ms),
        [this, manager]() {
          send_data(manager);
        });

      timers_.push_back(timer);
      RCLCPP_INFO(
        this->get_logger(),
        "TF Topic: %s, Rate: %f Hz", topic.c_str(), rate);
    } else {
      auto manager = std::make_shared<SubscriptionManager>(
        shared_from_this(), topic, subscribe_namespace,
        zstd_level, publish_stale_data);
      manager->setup_subscription();
      sub_mgrs_.push_back(manager);

      int ms = static_cast<int>(1000.0 / rate);
      auto timer = this->create_wall_timer(
        std::chrono::milliseconds(ms),
        [this, manager]() {
          send_data(manager);
        });

      timers_.push_back(timer);
      RCLCPP_INFO(
        this->get_logger(),
        "Topic: %s, Rate: %f Hz", topic.c_str(), rate);
    }

  }

  network_check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&NetworkBridge::check_network_health, this));

}

void NetworkBridge::check_network_health()
{
  if (!network_interface_) {
    initialize();
    return;
  }
  if (network_interface_->has_failed()) {
    RCLCPP_INFO(this->get_logger(), "Network interface has failed. Resetting");
    network_interface_->close();
    network_interface_->open();
    return;
  }
}

void NetworkBridge::load_network_interface()
{
  try {
    network_interface_ = loader_.createSharedInstance(network_interface_name_);

    network_interface_->initialize(
      shared_from_this(),
      std::bind(
        &NetworkBridge::receive_data,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded network interface: %s", network_interface_name_.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Failed to load network interface: %s", ex.what());
    rclcpp::shutdown();
    exit(1);
  }
}

void NetworkBridge::receive_data(std::span<const uint8_t> data)
{
  if (!rclcpp::ok()) {
    return;
  }

  auto now = std::chrono::system_clock::now();

  // 1. Validate the SensorForge frame before touching the payload. This
  //    enforces magic/version/header_size/bounds/CRC and per-link monotonic
  //    sequence & timestamp (stream key 0 = the single inbound link).
  namespace sfp = sensorforge::protocol;
  sfp::FrameHeader fh;
  const sfp::FrameError err =
    frame_decoder_.decode(data.data(), data.size(), /*stream_key=*/0, fh);
  if (err != sfp::FrameError::kOk) {
    ++frame_reject_count_;
    if (err == sfp::FrameError::kHeaderCrcMismatch ||
      err == sfp::FrameError::kPayloadCrcMismatch)
    {
      ++crc_failure_count_;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "Rejected frame: %s (rejects=%llu, crc_failures=%llu)",
      std::string(sfp::to_string(err)).c_str(),
      static_cast<unsigned long long>(frame_reject_count_),
      static_cast<unsigned long long>(crc_failure_count_));
    return;
  }

  std::span<const uint8_t> frame_payload(sfp::payload_ptr(data.data()), fh.payload_size);

  // 2. Decompress the (framed) payload. Fail closed on error.
  std::vector<uint8_t> decompressed_data;
  try {
    decompress(frame_payload, decompressed_data);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Decompression Failed: %s", e.what());
    return;
  }

  // 3. Parse the inner header (topic '\0' type '\0').
  std::string topic;
  std::string type;
  parse_header(decompressed_data, topic, type);

  if (topic.empty() || type.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Malformed header!");
    return;
  }

  size_t header_length = topic.size() + 1 + type.size() + 1;
  if (header_length > decompressed_data.size()) {
    RCLCPP_ERROR(this->get_logger(), "Inner payload shorter than header!");
    return;
  }

  std::span<const uint8_t> payload(
    decompressed_data.data() + header_length,
    decompressed_data.size() - header_length);

  // 4. Transmission delay is derived from the frame timestamp (nanoseconds).
  const double now_s = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  const double sent_s = static_cast<double>(fh.timestamp_ns) / 1e9;
  const double delay = now_s - sent_s;
  RCLCPP_DEBUG(
    this->get_logger(),
    "Received frame seq=%llu %lu bytes on topic %s with type %s",
    static_cast<unsigned long long>(fh.sequence),
    data.size(), topic.c_str(), type.c_str());
  RCLCPP_DEBUG(
    this->get_logger(),
    "Decompressed data size: %lu", decompressed_data.size());
  RCLCPP_DEBUG(this->get_logger(), "Delay: %f ms", delay * 1000);

  if (publishers_.find(topic) == publishers_.end()) {
    // Create a QoS configuration with reliability and durability settings
    rclcpp::QoS qos(10);

    // Set QoS to Reliable
    qos.reliable();

    // Set QoS to Transient Local Durability
    qos.transient_local();
    publishers_[topic] = this->create_generic_publisher(
      publish_namespace_ + topic, type, qos);
    RCLCPP_INFO(
      this->get_logger(), "Created publisher on %s type %s",
      (publish_namespace_ + topic).c_str(), type.c_str());
  }

  rclcpp::SerializedMessage msg(payload.size());
  std::copy(
    payload.begin(), payload.end(),
    msg.get_rcl_serialized_message().buffer);

  msg.get_rcl_serialized_message().buffer_length = payload.size();
  if (rclcpp::ok()) {
    publishers_[topic]->publish(msg);
  }

  auto end = std::chrono::system_clock::now();
  RCLCPP_DEBUG(
    this->get_logger(),
    "Receive time: %f ms",
    std::chrono::duration<double, std::milli>(end - now).count());
}

void NetworkBridge::send_data(std::shared_ptr<SubscriptionManager> manager)
{
  manager->check_subscription();
  if (!manager->has_data()) {
    return;
  }
  if (!network_interface_->is_ready()) {
    return;
  }

  bool is_data_valid = false;
  const std::vector<uint8_t> & data = manager->get_data(is_data_valid);
  if (data.empty() || !is_data_valid) { // This should not happen given the test above
    RCLCPP_WARN(
      this->get_logger(),
      "SubscriptionManager %s has no data", manager->topic_.c_str());
    return;
  }

  auto now = std::chrono::system_clock::now();
  const std::string & topic = manager->topic_;
  const std::string & type = manager->msg_type_;

  auto header = create_header(topic, type);

  // Form inner message: [topic '\0' type '\0' | serialized ROS2 message]
  std::vector<uint8_t> message;
  message.reserve(header.size() + data.size());
  message.insert(message.end(), header.begin(), header.end());
  message.insert(message.end(), data.begin(), data.end());

  // Compress inner message
  std::vector<uint8_t> compressed_data;
  try {
    compress(message, compressed_data, manager->zstd_compression_level_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Compression Failed: %s", e.what());
    return;
  }

  // Wrap the compressed payload in a SensorForge frame: magic/version/seq/
  // timestamp + CRC32C over header and payload. The bridge forwards arbitrary
  // topics, so the sensor_type is kControl; dedicated sensor publishers use
  // their specific type.
  namespace sfp = sensorforge::protocol;
  const uint64_t timestamp_ns = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());

  std::vector<uint8_t> frame;
  try {
    frame = sfp::encode_frame(
      sfp::SensorType::kControl, tx_sequence_++, timestamp_ns,
      sfp::kFlagCompressed, compressed_data.data(), compressed_data.size());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Frame encode failed: %s", e.what());
    return;
  }

  // Optionally record the frame payload to the WAL replay log (append-only).
  if (wal_writer_) {
    wal_writer_->append(
      timestamp_ns, sfp::SensorType::kControl, tx_sequence_ - 1,
      compressed_data.data(), compressed_data.size());
  }

  // Send framed data. When transport fault injection is enabled, route through
  // the fault engine (single call site); it decides drop/delay/corrupt/etc and
  // performs the actual network write via its callback.
  if (fault_engine_) {
    const double t_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - node_start_).count();
    fault_engine_->process(frame, t_s, topic);
  } else {
    network_interface_->write(frame);
  }
  auto end = std::chrono::system_clock::now();
  RCLCPP_DEBUG(
    this->get_logger(),
    "Send time: %f ms",
    std::chrono::duration<double, std::milli>(end - now).count());
}

std::vector<uint8_t> NetworkBridge::create_header(
  const std::string & topic,
  const std::string & msg_type)
{
  // Inner header carried inside the frame payload: topic '\0' msg_type '\0'.
  // The timestamp now lives in the SensorForge frame header (timestamp_ns),
  // so it is no longer emitted here.
  std::vector<uint8_t> header;
  header.reserve(topic.size() + 1 + msg_type.size() + 1);

  header.insert(header.end(), topic.begin(), topic.end());
  header.push_back('\0');

  header.insert(header.end(), msg_type.begin(), msg_type.end());
  header.push_back('\0');
  return header;
}

void NetworkBridge::parse_header(
  const std::vector<uint8_t> & header,
  std::string & topic, std::string & msg_type)
{
  // Minimum usable inner header: 1 char topic + null + 1 char type + null.
  if (header.size() < 4) {
    RCLCPP_ERROR(this->get_logger(), "Malformed inner header!");
    return;
  }

  // Bounded scan for the two null terminators so a corrupt/short payload that
  // slipped past the frame CRC cannot run reinterpret_cast off the end.
  const char * base = reinterpret_cast<const char *>(header.data());
  size_t topic_end = 0;
  while (topic_end < header.size() && base[topic_end] != '\0') {
    ++topic_end;
  }
  if (topic_end >= header.size() - 1) {
    RCLCPP_ERROR(this->get_logger(), "Malformed inner header: no type field!");
    return;
  }
  topic.assign(base, topic_end);

  size_t type_start = topic_end + 1;
  size_t type_end = type_start;
  while (type_end < header.size() && base[type_end] != '\0') {
    ++type_end;
  }
  if (type_end >= header.size()) {
    RCLCPP_ERROR(this->get_logger(), "Malformed inner header: unterminated type!");
    return;
  }
  msg_type.assign(base + type_start, type_end - type_start);
}

void NetworkBridge::compress(
  std::vector<uint8_t> const & data,
  std::vector<uint8_t> & compressed_data,
  int zstd_compression_level)
{
  size_t compressedCapacity = ZSTD_compressBound(data.size());

  // Resize the output buffer to the capacity needed
  compressed_data.resize(compressedCapacity);

  // Compress the data
  size_t compressedSize = ZSTD_compress(
    compressed_data.data(), compressedCapacity, data.data(), data.size(),
    zstd_compression_level);

  // Check for errors
  if (ZSTD_isError(compressedSize)) {
    throw std::runtime_error(ZSTD_getErrorName(compressedSize));
  }

  // Resize compressed_data to actual compressed size
  compressed_data.resize(compressedSize);
}

void NetworkBridge::decompress(
  std::span<const uint8_t> compressed_data,
  std::vector<uint8_t> & data)
{
  // Find the size of the original uncompressed data
  size_t decompressed_size = ZSTD_getFrameContentSize(
    compressed_data.data(), compressed_data.size());

  // Check if the size is known and valid
  if (decompressed_size == ZSTD_CONTENTSIZE_ERROR) {
    throw std::runtime_error("Not compressed by Zstd");
  } else if (decompressed_size == ZSTD_CONTENTSIZE_UNKNOWN) {
    throw std::runtime_error("Original size unknown");
  }

  // Resize the output buffer to the size of the uncompressed data
  data.resize(decompressed_size);

  // Decompress the data
  size_t decompressed_result = ZSTD_decompress(
    data.data(), decompressed_size, compressed_data.data(),
    compressed_data.size());

  // Check for errors during decompression
  if (ZSTD_isError(decompressed_result)) {
    throw std::runtime_error(ZSTD_getErrorName(decompressed_result));
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Randomized name to avoid conflicts
  std::string node_name = "network_bridge" + std::to_string(::getpid());

  auto node = std::make_shared<NetworkBridge>(node_name);
  node->initialize();

  rclcpp::spin(node);
  node->shutdown();
  node.reset();

  rclcpp::shutdown();
  return 0;
}
