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
// Clock publisher handler
//
// =============================================================================

#ifndef CH_ROS_CLOCK_HANDLER
#define CH_ROS_CLOCK_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "rosgraph_msgs/msg/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// Publishes rosgraph_msgs::msg::Clock messages at each timestep (by default) of the simulation. This is useful if you
/// want run the ROS at a corresponding to simulation time versus wall time.
class CH_ROS_API ChROSClockHandler : public ChROSHandler {
  public:
    /// Constructor for the ChROSClockHandler. The update rate defaults to 0 (i.e. to be updated at each timestep).
    /// topic_name defaults to "/clock" which overrides the ros time update handler. The chrono time will then
    /// correspond to ros time.
    ChROSClockHandler(double update_rate = 0, const std::string& topic_name = "/clock");

    /// Initializes this handler. Will create a single publisher for the clock information under the "/clock" topic.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Tick this hander. Will create the clock message and publish it. By default, Tick is called at each sim update.
    virtual void Tick(double time) override;

  private:
    const std::string m_topic_name;  ///< The topic name to publish the clock information to
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr
        m_publisher;  ///< The publisher which data is published through
};

///@} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif