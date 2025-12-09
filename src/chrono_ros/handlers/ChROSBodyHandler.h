// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Handler responsible for publishing information about a ChBody
//
// =============================================================================

#ifndef CH_ROS_BODY_HANDLER_H
#define CH_ROS_BODY_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono/physics/ChBody.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

#include <string>
#include <cstring>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// Data structure for body handler communication between processes.
/// Plain C++ struct with no ROS dependencies for IPC serialization.
struct ChROSBodyData {
    char body_name[64];
    char topic_prefix[128];
    
    double pos_x, pos_y, pos_z;
    double rot_w, rot_x, rot_y, rot_z;
    
    double lin_vel_x, lin_vel_y, lin_vel_z;
    double ang_vel_x, ang_vel_y, ang_vel_z;
    
    double lin_acc_x, lin_acc_y, lin_acc_z;
    double ang_acc_x, ang_acc_y, ang_acc_z;
};

/// Publishes pose, twist, and acceleration information for a ChBody.
/// Creates three publishers: <topic>/pose, <topic>/twist, <topic>/accel
class CH_ROS_API ChROSBodyHandler : public ChROSHandler {
  public:
    ChROSBodyHandler(double update_rate, std::shared_ptr<ChBody> body, const std::string& topic = "~/");

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    
    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::BODY_DATA; }

    /// Extract body state for IPC (called in main process, no ROS calls)
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

    /// Publish data to ROS from serialized bytes (called in subprocess)
    virtual void PublishFromSerialized(const std::vector<uint8_t>& data, 
                                       std::shared_ptr<ChROSInterface> interface) override;

  private:
    std::shared_ptr<ChBody> m_body;
    const std::string m_topic;
    
    geometry_msgs::msg::PoseStamped m_pose_msg;
    geometry_msgs::msg::TwistStamped m_twist_msg;
    geometry_msgs::msg::AccelStamped m_accel_msg;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_twist_publisher;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr m_accel_publisher;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
