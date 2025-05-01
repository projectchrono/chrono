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
// Authors: Radu Serban
// =============================================================================
//
// Interactive driver for a suspension test rig.
// Independent of keyboard event handler.
//
// =============================================================================

#ifndef CH_STR_INTERACTIVE_DRIVER_H
#define CH_STR_INTERACTIVE_DRIVER_H

#include <string>

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_test_rig
/// @{

/// Interactive driver for a suspension test rig.
/// This class implements support for keyboard control of a suspension test rig, but is independent of a particular
/// implementation of a keyboard event handler.
class CH_VEHICLE_API ChSuspensionTestRigInteractiveDriver : public ChSuspensionTestRigDriver {
  public:
    ChSuspensionTestRigInteractiveDriver();
    ~ChSuspensionTestRigInteractiveDriver() {}

    /// Get the currently actuated axle.
    int GetActiveAxle() const { return m_crt_axle; }

    /// Get the left post displacement for current actuated axle.
    double GetLeft();

    /// Get the right post displacement for current actuated axle.
    double GetRight();

    /// Change current actuated axle to next (cycle as needed).
    void NextAxle();

    /// Change current actuated axle to previous (cycle as needed).
    void PreviousAxle();

    /// Increment steering input.
    void IncrementSteering();

    /// Decrement steering input.
    void DecrementSteering();

    /// Increment displacement of left post for current actuated axle.
    void IncrementLeft();

    /// Decrement displacement of left post for current actuated axle.
    void DecrementLeft();

    /// Increment displacement of right post for current actuated axle.
    void IncrementRight();

    /// Decrement displacement of right post for current actuated axle.
    void DecrementRight();

    /// Set the time response for post displacement control.
    /// This value represents the time (in seconds) for changing the displacement input from 0 to 1 (or 0 to -1).
    /// Default: 1/100.
    void SetDisplacementDelta(double delta) { m_displ_delta = delta; }

    /// Set the time response for steering control.
    /// This value represents the time (in seconds) for changing the steering input from 0 to 1 (or 0 to -1).
    /// Default: 1/250.
    void SetSteeringDelta(double delta) { m_steering_delta = delta; }

  private:
    int m_crt_axle;  ///< current actuated axle

    double m_steering_delta;
    double m_displ_delta;
};

/// @} vehicle_wheeled_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
