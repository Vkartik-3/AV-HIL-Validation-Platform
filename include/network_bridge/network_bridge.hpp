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

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "network_bridge/subscription_manager.hpp"
#include "network_interfaces/network_interface_base.hpp"
#include "sensorforge/protocol/frame_codec.hpp"
#include "sensorforge/replay/wal_writer.hpp"

/**
 * @class NetworkBridge
 * @brief A class that provides bridging of ROS2 topics over a network interface.
 *
 * The `NetworkBridge` class is derived from the `rclcpp::Node` class and provides functionality for sending and receiving telemetry data over network.
 * It handles the setup of a network interface, parsing and creating headers, compressing and decompressing data, and error handling.
 * The class also manages the ROS 2 subscriptions, timers, and publishers associated with the communication.
 */
class NetworkBridge : public rclcpp::Node
{
public:
  /**
   * @brief Constructs a NetworkBridge object.
   *
   * This constructor initializes a NetworkBridge object with the specified node name.
   *
   * @param node_name The name of the ROS 2 node.
   */
  explicit NetworkBridge(const std::string & node_name);

  ~NetworkBridge() override;

  /**
   * @brief Loads parameters, loads network interface, and opens the network interface.
   *
   * It should be called before any other functions are called on the object.
   */
  virtual void initialize();

  /**
   * @brief Destroy the objects created by the bridge
   *
   * It should be called once spinning is over, and before the shared pointer is reset, to
   * make sure the garbage collector can run
   */
  virtual void shutdown();

protected:
  /**
   * @brief Loads default parameters and creates subsciption managers for each topic.
   */
  virtual void load_parameters();

  /**
   * @brief Loads the network interface as a dynamic plugin and initializes it.
   */
  virtual void load_network_interface();

  /**
   * @brief Callback function for handling received data.
   *
   * @param data The data to be received, represented as a span.
   */
  virtual void receive_data(std::span<const uint8_t> data);

  /**
   * @brief Sends data to the network interface.
   *
   * @param manager A shared pointer to the SubscriptionManager object.
   */
  virtual void send_data(std::shared_ptr<SubscriptionManager> manager);


  void check_network_health();
  /**
   * @brief Creates the inner (per-message) header carried inside a frame payload.
   *
   * Layout: topic '\0' msg_type '\0'. The wall-clock timestamp that used to
   * live here now lives in the SensorForge frame header (timestamp_ns), so it
   * is no longer part of the inner header.
   *
   * @param topic The topic of the message.
   * @param msg_type The type of the message.
   */
  virtual std::vector<uint8_t> create_header(
    const std::string & topic, const std::string & msg_type);

  /**
   * @brief Parses the inner header (topic '\0' msg_type '\0') of a message.
   *
   * @param header The decompressed inner payload to parse.
   * @param topic [out] The topic of the message.
   * @param msg_type [out] The type of the message.
   */
  virtual void parse_header(
    const std::vector<uint8_t> & header, std::string & topic,
    std::string & msg_type);

  /**
   * Compresses the given data using Zstandard compression algorithm.
   *
   * @param data The input data to be compressed.
   * @param compressed_data [out] The vector to store the compressed data.
   * @param zstd_compression_level The compression level to be used (default: 3).
   */
  virtual void compress(
    std::vector<uint8_t> const & data, std::vector<uint8_t> & compressed_data,
    int zstd_compression_level = 3);

  /**
   * @brief Decompresses the given compressed data.
   *
   * This function takes a span of compressed data and decompresses it,
   * storing the result in the provided data vector
   *
   * @param compressed_data The compressed data to be decompressed.
   * @param data [out] The vector to store the decompressed data.
   */
  virtual void decompress(
    std::span<const uint8_t> compressed_data, std::vector<uint8_t> & data);

  /**
   * @brief A class template that provides a plugin loader for network interfaces.
   *
   * @tparam InterfaceT The interface type that the loaded plugins must implement.
   */
  pluginlib::ClassLoader<network_bridge::NetworkInterface> loader_;

  /**
   * @brief The name of the network interface plugin.
   */
  std::string network_interface_name_;

  /**
   * @brief A shared pointer to an instance of the `network_bridge::NetworkInterface` class.
   */
  std::shared_ptr<network_bridge::NetworkInterface> network_interface_;

  /**
   * @brief A vector of the SubscriptionManager's for each topic.
   *
   * These are stored to keep them from going out of scope, but are not used directly.
   */
  std::vector<std::shared_ptr<SubscriptionManager>> sub_mgrs_;

  /**
   * @brief A vector of timers for sending each received topic over network.
   *
   * These are stored to keep them from going out of scope.
   *
   * @see rclcpp::TimerBase
   */
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;

  /**
   * @brief A time to check the network status, especially useful for tcp interface
   *
   * These are stored to keep them from going out of scope, but are not used directly.
   *
   * @see rclcpp::TimerBase
   */
  rclcpp::TimerBase::SharedPtr network_check_timer_;
  /**
   * @brief A map that stores the publisher object against the topic name.
   */
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;

  /**
   * @brief the namespace for the publishers.
   */
  std::string publish_namespace_;

  /**
   * @brief Monotonic per-link frame sequence counter (transmit side).
   *
   * The bridge multiplexes many ROS2 topics over a single transport link, so
   * the "stream" for framing purposes is the link itself: one monotonically
   * increasing sequence per NetworkBridge instance. (Dedicated sensor
   * publishers in the HIL layer each own their own per-sensor sequence.)
   */
  uint64_t tx_sequence_ = 0;

  /**
   * @brief Stateful frame decoder enforcing monotonic sequence/timestamp on
   *        the single inbound link (stream key 0).
   */
  sensorforge::protocol::FrameDecoder frame_decoder_;

  /// Count of frames rejected by validation (bad magic/version/CRC/etc).
  uint64_t frame_reject_count_ = 0;
  /// Subset of rejects attributable to a CRC mismatch (header or payload).
  uint64_t crc_failure_count_ = 0;

  /**
   * @brief Optional WAL recorder. When the `wal_record_dir` parameter is set,
   *        every outbound frame's payload is appended to a segmented replay log
   *        so the session can be deterministically replayed later.
   */
  std::unique_ptr<sensorforge::replay::WalWriter> wal_writer_;
};
