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
// Authors: Radu Serban
// =============================================================================
//
// Brake for tracked vehicles modeled using a clutch between two shafts.
//
// =============================================================================

#ifndef CH_TRACK_BRAKE_SHAFTS_H
#define CH_TRACK_BRAKE_SHAFTS_H

#include "chrono/physics/ChShaftsClutch.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackBrake.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_brake
/// @{

/// Brake for tracked vehicles modeled using a clutch between two shafts.
class CH_VEHICLE_API ChTrackBrakeShafts : public ChTrackBrake {
  public:
    ChTrackBrakeShafts(const std::string& name);

    virtual ~ChTrackBrakeShafts();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackBrakeShafts"; }

    /// Construct the brake by providing the chassis and associated sprocket.
    virtual void Construct(std::shared_ptr<ChChassis> chassis,   ///< associated chassis subsystem
                           std::shared_ptr<ChSprocket> sprocket  ///< associated sprocket subsystem
                           ) override;

    /// Update the brake subsystem for the given braking driver input.
    /// The input value is in the range [0,1].<br>
    /// <pre>
    ///   braking = 0 indicates no braking
    ///   braking = 1 indicates that the subsystem should provide maximum braking torque
    /// </pre>
    virtual void Synchronize(double time, double braking) override;

    /// Get the current brake torque.
    virtual double GetBrakeTorque() override { return m_braking * GetMaxBrakingTorque(); }

  protected:
    /// Get the brake shaft inertia
    virtual double GetShaftInertia() = 0;

    /// Get the max braking torque (for braking = 1)
    virtual double GetMaxBrakingTorque() = 0;

    double m_braking;                          ///< current braking input
    std::shared_ptr<ChShaft> m_shaft;          ///< shaft attached to "fixed" body
    std::shared_ptr<ChShaftsClutch> m_clutch;  ///< clutch between the brake shaft and axle shaft
};

/// @} vehicle_tracked_brake

}  // end namespace vehicle
}  // end namespace chrono

#endif
