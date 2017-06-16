// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Simple brake created with constant torque opposing sprocket rotation.
// It just uses a speed-dependent torque.
//
// =============================================================================

#ifndef CH_TRACK_BRAKE_SIMPLE_H
#define CH_TRACK_BRAKE_SIMPLE_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkBrake.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackBrake.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_brake
/// @{

/// Simple brake created with constant torque opposing sprocket rotation.
class CH_VEHICLE_API ChTrackBrakeSimple : public ChTrackBrake {
  public:
    ChTrackBrakeSimple(const std::string& name  ///< [in] name of the subsystem
                       );

    virtual ~ChTrackBrakeSimple() {}

    /// Initialize the brake by providing the sprocket's revolute link.
    virtual void Initialize(std::shared_ptr<ChLinkLockRevolute> hub) override;

    /// Update the brake subsystem for the given braking driver input.
    ///   braking = 0 : completely free,
    ///   braking = 1 : provide maximum braking torque
    virtual void Synchronize(double braking) override;

    /// Get the current brake torque.
    virtual double GetBrakeTorque() override { return m_braking * GetMaxBrakingTorque(); }

    /// Get the current brake angular speed (between disc and caliper) [rad/s].
    double GetBrakeSpeed() { return m_brake->GetRelWvel().Length(); }

  protected:
    /// Get the max braking torque (for braking = 1)
    virtual double GetMaxBrakingTorque() = 0;

    double m_braking;
    std::shared_ptr<ChLinkBrake> m_brake;
};

/// @} vehicle_tracked_brake

}  // end namespace vehicle
}  // end namespace chrono

#endif
