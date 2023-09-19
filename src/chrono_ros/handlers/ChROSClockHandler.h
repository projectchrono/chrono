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

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "rclcpp/publisher.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_vehicle_handlers
/// @{

/// Publishes rosgraph_msgs::msg::Clock messages at each timestep (by default) of the simulation. This is useful if you want run the ROS at a frequency corresponding to simulation time versus wall time.
class CH_ROS_API ChROSClockHandler : public ChROSHandler {
  public:
    /// Constructor for the ChROSClockHandler. Frequency defaults to 0 (i.e. to be updated at each timestep).
    ChROSClockHandler(uint64_t frequency = 0);

    /// Initializes this handler. Will create a single publisher for the clock information under the "/clock" topic.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Tick this hander. Will create the clock message and publish it. By default, Tick is called at each sim update.
    virtual void Tick(double time) override;

  private:
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_publisher; ///< The publisher which data is published through
};

///@} ros_vehicle_handlers

}  // namespace ros
}  // namespace chrono

#endif