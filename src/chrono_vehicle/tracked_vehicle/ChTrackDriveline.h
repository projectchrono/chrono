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

#ifndef CH_TRACK_DRIVELINE_H
#define CH_TRACK_DRIVELINE_H

#include "chrono/physics/ChShaft.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_driveline
/// @{

/// Base class for a tracked vehicle driveline.
class CH_VEHICLE_API ChTrackDriveline : public ChPart {
  public:
    ChTrackDriveline(const std::string& name  ///< [in] name of the subsystem
                     );

    virtual ~ChTrackDriveline() {}

    /// Get a handle to the driveshaft.
    /// Return a shared pointer to the shaft that connects this driveline to a
    /// powertrain system (i.e., right after the transmission box).
    std::shared_ptr<ChShaft> GetDriveshaft() const { return m_driveshaft; }

    /// Get the angular speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to
    /// the powertrain system. The default implementation returns the driveline's
    /// driveshaft speed.
    virtual double GetDriveshaftSpeed() const { return m_driveshaft->GetPos_dt(); }

    /// Get the motor torque to be applied to the specified sprocket.
    virtual double GetSprocketTorque(VehicleSide side) const = 0;

    /// Get the angular speed of the specified sprocket.
    virtual double GetSprocketSpeed(VehicleSide side) const = 0;

    /// Turn gyration mode on/off.
    virtual void SetGyrationMode(bool mode) { m_gyration_mode = mode; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the sprockets of the
    /// two track assembly subsystems.
    virtual void Initialize(std::shared_ptr<ChBody> chassis,              ///< handle to the chassis body
                            std::shared_ptr<ChTrackAssembly> track_left,  ///< handle to the left track assembly
                            std::shared_ptr<ChTrackAssembly> track_right  ///< handle to the right track assembly
                            ) = 0;

    /// Update the driveline subsystem.
    /// The motor torque represents the input to the driveline subsystem from the
    /// powertrain system. The default implementation applies this torque to the
    /// driveline's driveshaft.
    /// A derived class must also process the steering input to send appropriate
    /// torques to the sprockets of the two track assemblies.
    virtual void Synchronize(double steering, double torque) { m_driveshaft->SetAppliedTorque(torque); }

  protected:
    bool m_gyration_mode;                   ///< flag indicating if in gyration mode (turn in place)
    std::shared_ptr<ChShaft> m_driveshaft;  ///< handle to the shaft connection to the powertrain
};

/// @} vehicle_tracked_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
