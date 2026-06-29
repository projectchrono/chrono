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
// Subscription-side handle and callback base class. Created by
// ChROSBridge::CreateSubscription(); the ROS subscription lives in the bridge
// subprocess, which forwards serialized messages back to the simulation.
//
// Callbacks are invoked synchronously inside ChROSManager::Update() on the
// simulation thread (never from a background thread) - deterministic in sim
// time and safe for Python directors.
//
// =============================================================================

#ifndef CH_ROS_SUBSCRIPTION_H
#define CH_ROS_SUBSCRIPTION_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSQoS.h"

#include <cstdint>
#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSBridge;

/// @addtogroup ros_core
/// @{

/// Base class for subscription callbacks. Subclass (in C++ or Python) and
/// override OnMessage. C++ users may instead pass a lambda to the
/// std::function overload of ChROSBridge::CreateSubscription.
class CH_ROS_API ChROSSubscriptionCallback {
  public:
    virtual ~ChROSSubscriptionCallback() = default;

    /// Called once per received message, on the simulation thread, inside
    /// ChROSManager::Update().
    virtual void OnMessage(const ChROSMessageView& message) = 0;
};

class CH_ROS_API ChROSSubscription {
  public:
    /// Number of ROS publishers currently matched to this topic.
    size_t GetPublisherCount() const;

    /// Total messages delivered to the callback so far.
    uint64_t GetReceivedCount() const { return m_received_count; }

    /// Block until at least one message is delivered to this subscription (or
    /// the wall-clock timeout expires; <= 0 polls once). Other channels keep
    /// being serviced while waiting. Enables user-built lock-step loops; see
    /// the time-synchronization notes in the module documentation.
    bool WaitForMessage(double timeout_seconds);

    /// Topic this subscription listens on.
    const std::string& GetTopic() const { return m_topic; }
    /// ROS message type name of this subscription.
    const std::string& GetTypeName() const { return m_type_name; }
    /// Pretty-print this type's field layout (for discovery/debugging).
    std::string DescribeType() const;

    /// Unadvertises the subscription in the bridge subprocess.
    ~ChROSSubscription();
    ChROSSubscription(const ChROSSubscription&) = delete;
    ChROSSubscription& operator=(const ChROSSubscription&) = delete;

  private:
    friend class ChROSBridge;
    ChROSSubscription(std::shared_ptr<ChROSBridge> bridge,
                      uint32_t channel_id,
                      std::shared_ptr<const core::Schema> schema,
                      std::string topic,
                      std::string type_name,
                      std::shared_ptr<ChROSSubscriptionCallback> callback);

    std::shared_ptr<ChROSBridge> m_bridge;  ///< handles keep the bridge alive
    uint32_t m_channel_id;
    std::shared_ptr<const core::Schema> m_schema;
    std::string m_topic;
    std::string m_type_name;
    std::shared_ptr<ChROSSubscriptionCallback> m_callback;
    uint64_t m_received_count = 0;
};

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif
