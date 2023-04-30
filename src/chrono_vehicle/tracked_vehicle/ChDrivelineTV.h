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
// Base class for a tracked vehicle driveline.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_DRIVELINE_TV_H
#define CH_DRIVELINE_TV_H

#include "chrono_vehicle/ChDriveline.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_driveline
/// @{

/// Base class for a tracked vehicle driveline.
class CH_VEHICLE_API ChDrivelineTV : public ChDriveline {
  public:
    virtual ~ChDrivelineTV() {}

    /// Get the motor torque to be applied to the specified sprocket.
    virtual double GetSprocketTorque(VehicleSide side) const = 0;

    /// Get the angular speed of the specified sprocket.
    virtual double GetSprocketSpeed(VehicleSide side) const = 0;

    /// Turn gyration mode on/off.
    virtual void SetGyrationMode(bool mode) { m_gyration_mode = mode; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the sprockets of the two track assembly subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,           ///< associated chassis subsystem
                            std::shared_ptr<ChTrackAssembly> track_left,  ///< left track assembly
                            std::shared_ptr<ChTrackAssembly> track_right  ///< right track assembly
                            ) = 0;

    /// Lock/unlock the differential (if available).
    virtual void LockDifferential(bool lock);

  protected:
    ChDrivelineTV(const std::string& name);

    virtual void CombineDriverInputs(const DriverInputs& driver_inputs, double& braking_left, double& braking_right);

    bool m_gyration_mode;  ///< flag indicating if in gyration mode (turn in place)

    friend class ChTrackedVehicle;
};

/// @} vehicle_tracked_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
