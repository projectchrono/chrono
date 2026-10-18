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
// Clock publisher handler.
//
// =============================================================================

#ifndef CH_ROS_CLOCK_HANDLER_H
#define CH_ROS_CLOCK_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSPublisher;

/// @addtogroup ros_handlers
/// @{

/// Publishes rosgraph_msgs/msg/Clock at each simulation step (by default), so
/// that ROS nodes consuming /clock advance on simulation time rather than wall
/// time.
class CH_ROS_API ChROSClockHandler : public ChROSHandler {
  public:
    /// The update rate defaults to 0 (tick on every simulation step). The topic
    /// defaults to "/clock", which is what ROS time sources expect.
    ChROSClockHandler(double update_rate = 0, const std::string& topic_name = "/clock");

    /// Creates the single /clock publisher.
    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    /// Publishes the current simulation time as a Clock message.
    virtual void Tick(double time) override;

  private:
    const std::string m_topic_name;
    std::shared_ptr<ChROSPublisher> m_publisher;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
