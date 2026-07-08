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

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include "sensorforge/core/spsc_ring.hpp"
#include "sensorforge/core/sensor_frame.hpp"
#include "sensorforge/core/backpressure_policy.hpp"
#include "sensorforge/protocol/frame.hpp"

/**
 * @class SubscriptionManager
 * @brief Manages and stores data of subscriptions to a specific topic.
 *
 * The SubscriptionManager class is responsible for managing and storing data of subscriptions to a specific topic.
 * It provides methods to retrieve the stored data and set up subscriptions for the topic.
 */
class SubscriptionManager
{
public:
  /**
   * @brief Constructs a SubscriptionManager object.
   *
   * This constructor initializes a SubscriptionManager object with the given parameters.
   *
   * @param node A pointer to the rclcpp::Node object.
   * @param topic The topic to subscribe to.
   * @param zstd_compression_level The compression level for Zstandard compression (default: 3).
   * @param namespace The namespace for the subscription.
   * @param publish_stale_data Flag indicating whether to publish stale data (default: false).
   */
  SubscriptionManager(
    const rclcpp::Node::SharedPtr & node, const std::string & topic,
    const std::string & subscribe_namespace, int zstd_compression_level = 3,
    bool publish_stale_data = false);

  virtual ~SubscriptionManager();

  /**
   * @brief Retrieves the data stored in the subscription manager.
   *
   * Set is_valid to false if no data has been received or if the data is stale and
   * the flag publish_stale_data_ is false,
   *
   * @return a const reference to the internal data buffer
   */
  virtual const std::vector<uint8_t> & get_data(bool & is_valid);

  /**
   * @brief Check if data is available
   *
   * @return a boolean flag indicating if the data is valid
   */
  virtual bool has_data() const;

  /**
   * @brief Check if the subscription has been successful, or try to set it up
   *
   */
  virtual void check_subscription();

  /**
   * @brief Sets up a subscription for a given topic.
   *
   * This function sets up a subscription for the specified topic.
   * It takes a string parameter representing the topic to subscribe to.
   * This function is called automatically in the constructor and get_data() method.
   * It fails if the topic does not exist or if there are no publishers on this topic.
   */
  virtual void setup_subscription();

  /**
   * @brief Create the subscriber
   *
   * This function creates the actual subscriber after setup-subscription has
   * handled the qos and other params. Can be overloaded by specialized
   * subscribers
   */
  virtual void create_subscription(
    const std::string & topic,
    const std::string & msg_type, const rclcpp::QoS & qos);


  virtual bool is_stale() const;

  /**
   * @brief Callback function for handling serialized messages.
   *
   * This function is called when a serialized message is received by the subscription manager.
   * It pushes the data from serialized_msg into the SPSC ring under this
   * stream's backpressure policy.
   *
   * @param serialized_msg A shared pointer to the serialized message.
   */
  void callback(
    const std::shared_ptr<const rclcpp::SerializedMessage> & serialized_msg);

  /**
   * @brief Pointer to the ROS 2 node.
   */
  const rclcpp::Node::SharedPtr node_;

public:
  /**
   * @brief The type of message stored in the subscription manager.
   */
  std::string msg_type_;

  /**
   * @brief The topic name for the subscription.
   */
  std::string topic_;

  /**
   * @brief The namespace for the subscription.
   */
  std::string subscribe_namespace_;

  /**
   * @brief The compression level used for Zstandard compression (1->22).
   */
  int zstd_compression_level_;

  /// Compile-time ring capacity per stream (power of two).
  static constexpr size_t kRingCapacity = 1024;

  /// Set the sensor type (selects the default backpressure policy). Defaults
  /// to kControl for the generic bridge; dedicated sensor publishers override.
  void set_sensor_type(sensorforge::protocol::SensorType t)
  {
    sensor_type_ = t;
    policy_ = sensorforge::core::default_policy_for(t);
  }

  /// Producer/consumer counters for metrics (approximate, lock-free).
  uint64_t enqueued_count() const {return enqueued_count_;}
  uint64_t dropped_count() const {return dropped_count_;}
  uint64_t overwritten_count() const {return overwritten_count_;}
  size_t ring_occupancy() const {return ring_.size_approx();}

protected:
  bool topic_found_;
  /**
   * @brief Flag indicating whether a message has ever been received.
   */
  bool received_msg_;

  /**
   * @brief Flag indicating whether the data is stale (already accessed via get_data()).
   */
  bool is_stale_;

  /**
   * @brief Flag indicating whether to publish stale data.
   */
  bool publish_stale_data_;

  /**
   * @brief The ROS2 generalized subscriber object.
   */
  rclcpp::GenericSubscription::SharedPtr subscriber;

  /**
   * @brief Lock-free SPSC ring holding captured frames.
   *
   * Producer: the ROS2 subscription callback thread (callback()).
   * Consumer: the send-timer thread (get_data()).
   * Replaces the previous single-slot data_ buffer.
   */
  sensorforge::core::SPSCRing<sensorforge::core::SensorFrame, kRingCapacity> ring_;

  /**
   * @brief The most recently popped frame. get_data() returns a reference into
   *        this so the public API (const std::vector<uint8_t> &) is preserved,
   *        and it doubles as the "last value" resent when publish_stale_data_.
   */
  sensorforge::core::SensorFrame current_frame_;

  /// Per-stream capture sequence counter (producer side).
  uint64_t capture_sequence_ = 0;

  /// Backpressure policy + sensor classification for this stream.
  sensorforge::protocol::SensorType sensor_type_ =
    sensorforge::protocol::SensorType::kControl;
  sensorforge::core::BackpressurePolicy policy_ =
    sensorforge::core::BackpressurePolicy::kDropNewest;

  /// Metrics counters (single-writer each; read approximately elsewhere).
  uint64_t enqueued_count_ = 0;
  uint64_t dropped_count_ = 0;
  uint64_t overwritten_count_ = 0;
};
