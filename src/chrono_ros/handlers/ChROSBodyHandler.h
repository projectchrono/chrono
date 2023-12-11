// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
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

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// This handler is responsible for publishing state information about a ChBody. This handler creates three publishers
/// for the pose, twist (linear/angular velocity), and accel (linear/angular acceleration) topics.
class ChROSBodyHandler : public ChROSHandler {
  public:
    /// Constructor.
    /// The based topic is concatenated before the individual topic names. This handler will publish to the topics:
    ///     <topic>/pose
    ///     <topic>/twist
    ///     <topic>/accel
    /// where <topic> is the passed in topic argument. If the no topic is passed, the topics will be:
    ///     ~/pose
    ///     ~/twist
    ///     ~/accel
    ChROSBodyHandler(double update_rate, std::shared_ptr<ChBody> body, const std::string& topic = "~/");

    /// Initializes the handler. This creates the three publishers for the pose, twist, and accel topics.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// @brief Simply calls PublishPose, PublishTwist, and PublishAccel.
    /// @param time The time at which the current state of the simulation is.
    virtual void Tick(double time) override;

  private:
    /// Publishes the pose of the body at the given time.
    void PublishPose(double time);
    /// Publishes the twist of the body at the given time.
    void PublishTwist(double time);
    /// Publishes the accel of the body at the given time.
    void PublishAccel(double time);

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
