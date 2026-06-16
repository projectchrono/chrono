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
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Handler that publishes the state of a ChBody.
//
// =============================================================================

#ifndef CH_ROS_BODY_HANDLER_H
#define CH_ROS_BODY_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono/physics/ChBody.h"

#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSPublisher;

/// @addtogroup ros_handlers
/// @{

/// Publishes the state of a ChBody on three topics derived from a base topic:
///     <topic>/pose   geometry_msgs/msg/PoseStamped   (position + orientation)
///     <topic>/twist  geometry_msgs/msg/TwistStamped  (linear + angular velocity)
///     <topic>/accel  geometry_msgs/msg/AccelStamped  (linear + angular accel.)
/// With the default base topic "~/" these resolve to ~/pose, ~/twist, ~/accel.
/// Schema-driven equivalent of the Chrono 9.0 handler; the call interface is
/// unchanged.
class CH_ROS_API ChROSBodyHandler : public ChROSHandler {
  public:
    /// @param update_rate publish rate (Hz, sim time); 0 = every step.
    /// @param body the body whose state is published.
    /// @param topic base topic; "/pose", "/twist", "/accel" are appended.
    ChROSBodyHandler(double update_rate, std::shared_ptr<ChBody> body, const std::string& topic = "~/");

    /// Creates the pose, twist, and accel publishers.
    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    /// Publishes pose, twist, and accel for the current state.
    virtual void Tick(double time) override;

  private:
    void PublishPose(double time);
    void PublishTwist(double time);
    void PublishAccel(double time);

    std::shared_ptr<ChBody> m_body;
    const std::string m_topic;

    std::shared_ptr<ChROSPublisher> m_pose_publisher;
    std::shared_ptr<ChROSPublisher> m_twist_publisher;
    std::shared_ptr<ChROSPublisher> m_accel_publisher;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
