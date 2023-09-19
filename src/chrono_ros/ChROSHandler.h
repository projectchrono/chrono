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
// Base class for all ros handlers
//
// =============================================================================

#ifndef CH_ROS_HANDLER_H
#define CH_ROS_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSInterface.h"

#include <string>
#include <memory>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// Base class for a ROS handler. A specific handler should inherit from here.
/// A handler is essentially a wrapper around a ROS subscriber/publisher/service/action.
/// Logic regarding message generation/parsing and sending/receiving is implemented here.
/// A derived class is implemented to define logic specific to a desired output.
class CH_ROS_API ChROSHandler {
  public:
    /// Constructor for the ChROSHandler
    /// @param frequency Frequency with which the handler should tick relative to the simulation clock. NOTE: A
    ///         frequency of 0 indicates tick should be called on each update of the simulation.
    explicit ChROSHandler(uint64_t frequency);

    /// Initializes the handler. Must be implemented by derived classes. This is called after rclcpp::init().
    ///         Here the underlying ROS objects (e.g. publisher, subscription) should be created.
    /// @param interface The interface to the ROS node
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) = 0;

    /// Updates the internal clock and checks if a tick should occur. If yes, Tick() is called.
    /// @param time the current simulation time
    /// @param step the step size used since the last Update call
    virtual void Update(double time, double step) final;

    /// Get the frequency which this handler operates at
    uint64_t GetFrequency() const { return m_frequency; }

  protected:
    /// Derived class must implement this function.
    /// @param time the current simulation time
    virtual void Tick(double time) = 0;

  private:
    const uint64_t m_frequency;  ///< Hz

    double m_time_elapsed_since_last_tick;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif