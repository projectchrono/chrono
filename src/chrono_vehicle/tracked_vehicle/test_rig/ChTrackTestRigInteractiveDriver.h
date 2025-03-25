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
// Interactive driver for a track test rig.
// Independent of keyboard event handler.
//
// =============================================================================

#ifndef CH_TTR_INTERACTIVE_DRIVER_H
#define CH_TTR_INTERACTIVE_DRIVER_H

#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigDriver.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_test_rig
/// @{

/// Interactive driver for a track test rig.
/// This class implements support for keyboard control of a track test rig, but is independent of a particular
/// implementation of a keyboard event handler.
class CH_VEHICLE_API ChTrackTestRigInteractiveDriver : public ChTrackTestRigDriver {
  public:
    ChTrackTestRigInteractiveDriver();

    ~ChTrackTestRigInteractiveDriver() {}

    /// Get the displacement for current actuated post.
    double GetPost();

    /// Change current actuated post to next (cycle as needed).
    void NextPost();

    /// Change current actuated post to previous (cycle as needed).
    void PreviousPost();

    /// Increment displacement of current actuated post.
    void IncreasePost();

    /// Decrement displacement of current actuated post.
    void DecreasePost();

    /// Increase Throttle
    void IncreaseThrottle();

    /// Decrease Throttle
    void DecreaseThrottle();

    /// Set the time response for post displacement control.
    /// This value represents the time (in seconds) for changing the displacement input from 0 to 1 (or 0 to -1).
    /// Default: 1/250.
    void SetDisplacementDelta(double delta) { m_displ_delta = delta; }

    /// Set the time response for throttle control.
    /// This value represents the time (in seconds) for changing the throttle input from 0 to 1.
    /// Default: 1/50.
    void SetThrottleDelta(double delta) { m_throttle_delta = delta; }

  private:
    virtual std::string GetInfoMessage() const override { return m_msg; }

    double m_throttle_delta;
    double m_displ_delta;
    int m_current_post;
    std::string m_msg;
};

/// @} vehicle_tracked_test_rig

}  // end namespace vehicle
}  // end namespace chrono

#endif
