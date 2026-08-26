// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Patrick Chen
// =============================================================================
//
// ChROSBridge: the simulation-side engine of Chrono::ROS.
//
// Owns the IPC channel and the chrono_ros_node subprocess; fetches runtime
// type schemas, validates the wire encoding per type at initialization, and
// pumps serialized messages in both directions. Contains zero ROS code - the
// ROS side lives entirely in the subprocess.
//
// Thread model: NOT thread-safe; call everything from the simulation thread.
// Subscription callbacks fire synchronously inside ProcessIncoming() (i.e.
// inside ChROSManager::Update()).
//
// =============================================================================

#ifndef CH_ROS_BRIDGE_H
#define CH_ROS_BRIDGE_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/ChROSQoS.h"
#include "chrono_ros/ChROSSubscription.h"
#include "chrono_ros/core/ChROSFrame.h"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace chrono {
namespace ros {

namespace core {
class Channel;
}
class ChROSManager;
class ChROSNodeProcess;

/// @addtogroup ros_core
/// @{

class CH_ROS_API ChROSBridge : public std::enable_shared_from_this<ChROSBridge> {
  public:
    /// Construct a bridge for a node of the given name; usually created by
    /// ChROSManager rather than directly.
    explicit ChROSBridge(const std::string& node_name = "chrono_ros_node");
    /// Runs Shutdown().
    ~ChROSBridge();

    ChROSBridge(const ChROSBridge&) = delete;
    ChROSBridge& operator=(const ChROSBridge&) = delete;

    // --- configuration (before Initialize)

    /// Shared-memory ring capacities. The sim->node direction carries bulk
    /// sensor data (default 128 MiB); node->sim carries commands (default
    /// 32 MiB). Raise sim_to_node if a single message exceeds it (the error
    /// at Publish() names this knob). In Docker, /dev/shm must exceed the sum.
    void SetChannelCapacity(size_t sim_to_node_bytes, size_t node_to_sim_bytes);

    // --- lifecycle (driven by ChROSManager; callable directly in bare setups)

    /// Create the IPC channel, launch chrono_ros_node, and wait for its
    /// handshake. Throws std::runtime_error with an actionable message on any
    /// failure (ROS not sourced, executable missing, version mismatch, ...).
    void Initialize();

    /// True once Initialize() has completed successfully.
    bool IsInitialized() const { return m_initialized; }

    /// True while the bridge subprocess is alive.
    bool IsNodeAlive() const;

    /// Graceful teardown: protocol Shutdown, subprocess termination, channel
    /// release. Idempotent; also runs on destruction.
    void Shutdown();

    // --- topic API

    /// Create a publisher for any installed message type, addressed by type
    /// name (e.g. "sensor_msgs/msg/NavSatFix"). Blocks for the schema /
    /// validation / advertise round trips (milliseconds; once per type for
    /// the schema work). Throws on unknown type, invalid topic, or a wire-
    /// format validation failure.
    std::shared_ptr<ChROSPublisher> CreatePublisher(const std::string& topic,
                                                    const std::string& type_name,
                                                    const ChROSQoS& qos = ChROSQoS());

    /// Create a subscription; 'callback' fires inside ProcessIncoming() on
    /// the simulation thread.
    std::shared_ptr<ChROSSubscription> CreateSubscription(const std::string& topic,
                                                          const std::string& type_name,
                                                          std::shared_ptr<ChROSSubscriptionCallback> callback,
                                                          const ChROSQoS& qos = ChROSQoS());

    /// C++ convenience overload (not exposed to Python; subclass
    /// ChROSSubscriptionCallback there).
    std::shared_ptr<ChROSSubscription> CreateSubscription(const std::string& topic,
                                                          const std::string& type_name,
                                                          std::function<void(const ChROSMessageView&)> callback,
                                                          const ChROSQoS& qos = ChROSQoS());

    /// Human-readable field layout of any installed message type (fetches and
    /// caches its schema).
    std::string DescribeType(const std::string& type_name);

    // --- pumping (ChROSManager::Update calls these every step)

    /// Drain inbound frames: deliver received messages to subscription
    /// callbacks and refresh matched-endpoint counts.
    void ProcessIncoming();

    /// Record the current simulation time (stamped into outbound frames).
    void SetSimTime(double time_seconds);

    // --- diagnostics

    /// Outbound messages dropped because the IPC ring was full.
    uint64_t GetDroppedOutboundCount() const { return m_dropped_outbound; }

  private:
    friend class ChROSPublisher;
    friend class ChROSSubscription;

    struct SubscriptionRecord;

    // handle backends
    bool PublishSerialized(uint32_t channel_id, const ChROSMessage& message,
                           const std::shared_ptr<const core::Schema>& expected_schema);
    void Unadvertise(uint32_t channel_id);
    size_t GetMatchedCount(uint32_t channel_id) const;
    bool WaitForMessageOn(ChROSSubscription& subscription, double timeout_seconds);

    // initialization machinery
    std::shared_ptr<const core::Schema> GetOrFetchSchema(const std::string& type_name);
    void ValidateTypeOnce(const std::string& type_name, const std::shared_ptr<const core::Schema>& schema);
    uint32_t Advertise(const std::string& topic,
                       const std::string& type_name,
                       const ChROSQoS& qos,
                       bool subscribe);

    /// Pump frames until one of 'kind' (matching channel_id when nonzero)
    /// arrives; other frames are dispatched normally. Throws on timeout or
    /// node death.
    core::FrameHeader AwaitFrame(uint16_t kind,
                                 uint32_t channel_id,
                                 std::vector<uint8_t>& payload,
                                 double timeout_seconds,
                                 const char* what);

    void DispatchFrame(const core::FrameHeader& header, std::vector<uint8_t>& payload);
    void RequireInitialized(const char* what) const;

    std::string m_node_name;
    size_t m_capacity_sim_to_node;
    size_t m_capacity_node_to_sim;

    bool m_initialized = false;
    std::unique_ptr<core::Channel> m_channel;
    std::unique_ptr<ChROSNodeProcess> m_process;

    std::map<std::string, std::shared_ptr<const core::Schema>> m_schemas;
    std::set<std::string> m_validated_types;

    uint32_t m_next_channel_id = 1;
    std::map<uint32_t, std::weak_ptr<ChROSSubscription>> m_subscriptions;
    std::map<uint32_t, size_t> m_matched_counts;

    uint64_t m_sim_time_ns = 0;
    uint64_t m_dropped_outbound = 0;
    std::vector<uint8_t> m_serialize_scratch;
    std::vector<uint8_t> m_receive_scratch;
};

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif
