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
// Handle to a topic published by the simulation. Created by
// ChROSBridge::CreatePublisher(); the corresponding ROS publisher lives in
// the bridge subprocess.
//
// =============================================================================

#ifndef CH_ROS_PUBLISHER_H
#define CH_ROS_PUBLISHER_H

#include "chrono_ros/ChApiROS.h"
#include <cstdint>
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSQoS.h"

#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSBridge;

/// @addtogroup ros_core
/// @{

class CH_ROS_API ChROSPublisher {
  public:
    /// New empty message of this publisher's type.
    ChROSMessage NewMessage() const;

    /// Serialize and queue the message for publication. Returns false if the
    /// IPC channel is momentarily full (the frame is dropped and counted;
    /// appropriate for sensor streams - low-rate critical topics should treat
    /// false as an error).
    bool Publish(const ChROSMessage& message);

    /// Number of ROS subscriptions currently matched to this topic (updated
    /// asynchronously). Big-data handlers use this to skip extraction when
    /// nobody is listening.
    size_t GetSubscriptionCount() const;

    /// Topic this publisher writes to.
    const std::string& GetTopic() const { return m_topic; }
    /// ROS message type name of this publisher.
    const std::string& GetTypeName() const { return m_type_name; }

    /// Pretty-print this type's field layout (for discovery/debugging).
    std::string DescribeType() const;

    /// Unadvertises the topic in the bridge subprocess.
    ~ChROSPublisher();
    ChROSPublisher(const ChROSPublisher&) = delete;
    ChROSPublisher& operator=(const ChROSPublisher&) = delete;

  private:
    friend class ChROSBridge;
    ChROSPublisher(std::shared_ptr<ChROSBridge> bridge,
                   uint32_t channel_id,
                   std::shared_ptr<const core::Schema> schema,
                   std::string topic,
                   std::string type_name);

    std::shared_ptr<ChROSBridge> m_bridge;  ///< handles keep the bridge alive
    uint32_t m_channel_id;
    std::shared_ptr<const core::Schema> m_schema;
    std::string m_topic;
    std::string m_type_name;
};

/// @} ros_core

}  // namespace ros
}  // namespace chrono

#endif
