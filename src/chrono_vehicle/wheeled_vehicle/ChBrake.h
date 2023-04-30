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
// Base class for a wheel brake.
//
// =============================================================================

#ifndef CH_BRAKE_H
#define CH_BRAKE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_brake
/// @{

/// Base class for a brake subsystem.
class CH_VEHICLE_API ChBrake : public ChPart {
  public:
    virtual ~ChBrake() {}

    /// Initialize the brake by associating it to an existing suspension subsystem.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,        ///< associated chassis subsystem
                            std::shared_ptr<ChSuspension> suspension,  ///< associated suspension subsystem
                            VehicleSide side                           ///< brake mounted on left/right side
    );

    /// Update the brake subsystem: set the brake modulation.
    /// The input value is in the range [0,1].<br>
    /// <pre>
    ///   modulation = 0 indicates no braking
    ///   modulation = 1 indicates that the subsystem should provide maximum braking torque
    /// </pre>
    virtual void Synchronize(double modulation) = 0;

    /// Enable/disable ability of locking (default: false).
    /// Note that a derived class may or may not support this option.
    void EnableLocking(bool val) { m_can_lock = val; }

    /// Get the current brake torque.
    virtual double GetBrakeTorque() = 0;

  protected:
    ChBrake(const std::string& name);

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    bool m_can_lock;  ///< can the brake lock?
};

/// Vector of handles to brake subsystems.
typedef std::vector<std::shared_ptr<ChBrake>> ChBrakeList;

/// @} vehicle_wheeled_brake

}  // end namespace vehicle
}  // end namespace chrono

#endif
