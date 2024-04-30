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
// Simple brake created with constant torque opposing wheel rotation.
// It just uses a speed-dependent torque, so it fits in ODEs because it does not
// use NSC set valued constraints (the drawback is that it cannot simulate
// sticking brakes).
//
// =============================================================================

#ifndef CH_BRAKESIMPLE_H
#define CH_BRAKESIMPLE_H

#include "chrono/physics/ChLinkLockBrake.h"

#include "chrono_vehicle/wheeled_vehicle/ChBrake.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_brake
/// @{

/// Template for a simple brake subsystem.
/// Simple brake created with constant torque opposing wheel rotation.
/// It just uses a speed-dependent torque, so it fits in ODEs because it does not
/// use NSC set valued constraints (the drawback is that it cannot simulate
/// sticking brakes).
class CH_VEHICLE_API ChBrakeSimple : public ChBrake {
  public:
    ChBrakeSimple(const std::string& name);

    virtual ~ChBrakeSimple();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "BrakeSimple"; }

    /// Initialize the brake by associating it to an existing suspension subsystem.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,        ///< associated chassis subsystem
                            std::shared_ptr<ChSuspension> suspension,  ///< associated suspension subsystem
                            VehicleSide side                           ///< brake mounted on left/right side
                            ) override;

    /// Update the brake subsystem for the given braking driver input.
    /// The input value is in the range [0,1].<br>
    /// <pre>
    ///   braking = 0 indicates no braking
    ///   braking = 1 indicates that the subsystem should provide maximum braking torque
    /// </pre>
    virtual void Synchronize(double time, double braking) override;

    /// Get the current brake torque.
    virtual double GetBrakeTorque() override { return m_modulation * GetMaxBrakingTorque(); }

    /// Get the current brake angular speed, relative between disc and caliper [rad/s]
    double GetBrakeSpeed() { return m_brake->GetRelativeAngVel().Length(); }

  protected:
    /// Get the max braking torque (for modulation =1)
    virtual double GetMaxBrakingTorque() = 0;

    double m_modulation;                        ///< current braking input
    std::shared_ptr<ChLinkLockBrake> m_brake;   ///< underlying brake component
    std::shared_ptr<ChLinkLockRevolute> m_hub;  ///< associated spindle revolute joint
    bool m_locked;                              ///< is brake locked?
};

/// @} vehicle_wheeled_brake

}  // end namespace vehicle
}  // end namespace chrono

#endif
