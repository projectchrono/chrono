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
    ChDrivelineTV(const std::string& name);

    virtual ~ChDrivelineTV() {}

    /// Get the motor torque to be applied to the specified sprocket.
    virtual double GetSprocketTorque(VehicleSide side) const = 0;

    /// Get the angular speed of the specified sprocket.
    virtual double GetSprocketSpeed(VehicleSide side) const = 0;

    /// Turn gyration mode on/off.
    virtual void SetGyrationMode(bool mode) { m_gyration_mode = mode; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the sprockets of the two track assembly subsystems.
    virtual void Initialize(std::shared_ptr<ChBody> chassis,              ///< handle to the chassis body
                            std::shared_ptr<ChTrackAssembly> track_left,  ///< handle to the left track assembly
                            std::shared_ptr<ChTrackAssembly> track_right  ///< handle to the right track assembly
                            ) = 0;

    /// Update the driveline subsystem.
    /// The motor torque represents the input to the driveline subsystem from the powertrain system. The default
    /// implementation applies this torque to the driveline's driveshaft. A derived class must also process the steering
    /// input to send appropriate torques to the sprockets of the two track assemblies.
    virtual void Synchronize(double steering, double torque);

  protected:
    bool m_gyration_mode;  ///< flag indicating if in gyration mode (turn in place)
};

/// @} vehicle_tracked_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
