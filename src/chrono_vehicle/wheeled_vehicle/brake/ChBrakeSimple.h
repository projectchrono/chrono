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

#include "chrono/physics/ChLinkBrake.h"

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

    virtual ~ChBrakeSimple() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "BrakeSimple"; }

    /// Initialize the brake by associating it to an existing suspension subsystem.
    virtual void Initialize(std::shared_ptr<ChSuspension> suspension,  ///< associated suspension subsystem
                            VehicleSide side                           ///< brake mounted on left/right side
                            ) override;

    /// Update the brake subsystem: set the brake modulation, in 0..1 range:
    /// <pre>
    /// when = 0 it is completely free,
    /// when = 1 it should provide the max braking torque
    /// </pre>
    virtual void Synchronize(double modulation) override;

    /// Get the current brake torque, as a result of simulation,
    /// so it might change from time to time
    virtual double GetBrakeTorque() override { return m_modulation * GetMaxBrakingTorque(); }

    /// Get the current brake angular speed, relative between disc and caliper [rad/s]
    double GetBrakeSpeed() { return m_brake->GetRelWvel().Length(); }

  protected:
    /// Get the max braking torque (for modulation =1)
    virtual double GetMaxBrakingTorque() = 0;

    double m_modulation;
    std::shared_ptr<ChLinkBrake> m_brake;
};

/// @} vehicle_wheeled_brake

}  // end namespace vehicle
}  // end namespace chrono

#endif
