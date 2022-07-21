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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a wheeled vehicle driveline.
//
// =============================================================================

#ifndef CH_DRIVELINE_WV_H
#define CH_DRIVELINE_WV_H

#include "chrono_vehicle/ChDriveline.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/ChAxle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// Base class for a wheeled vehicle driveline subsystem.
class CH_VEHICLE_API ChDrivelineWV : public ChDriveline {
  public:
    virtual ~ChDrivelineWV() {}

    /// Return the number of driven axles.
    virtual int GetNumDrivenAxles() const = 0;

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the specified axle subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,   ///< associated chassis subsystem
                            const ChAxleList& axles,              ///< list of all vehicle axle subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) = 0;

    /// Update the driveline subsystem.
    /// The motor torque represents the input to the driveline subsystem from the powertrain system.
    /// The default implementation applies this torque to the driveline's driveshaft.
    virtual void Synchronize(double time,                            ///< [in] current time
                             const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                             double torque                           ///< [in] motor torque
    );

    /// Lock/unlock the differential on the specified axle.
    /// By convention, axles are counted front to back, starting with index 0 for the front-most axle.
    virtual void LockAxleDifferential(int axle, bool lock);

    /// Lock/unlock the specified central differential.
    /// By convention, central differentials are counted from front to back, starting with index 0.
    virtual void LockCentralDifferential(int which, bool lock);

    /// Get the indexes of the vehicle's axles driven by this driveline subsystem.
    const std::vector<int>& GetDrivenAxleIndexes() const { return m_driven_axles; }

    /// Get the motor torque to be applied to the specified spindle.
    virtual double GetSpindleTorque(int axle, VehicleSide side) const = 0;

    /// Disconnect driveline from driven wheels.
    virtual void Disconnect() = 0;

  protected:
    ChDrivelineWV(const std::string& name);

    std::vector<int> m_driven_axles;  ///< indexes of the driven vehicle axles
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
