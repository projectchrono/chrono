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
// Base class for a vehicle driveline.
//
// =============================================================================

#ifndef CH_DRIVELINE_H
#define CH_DRIVELINE_H

#include "chrono/core/ChVector.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// Base class for a driveline subsystem.
class CH_VEHICLE_API ChDriveline : public ChPart {
  public:
    ChDriveline(const std::string& name  ///< [in] name of the subsystem
                );

    virtual ~ChDriveline() {}

    /// Return the number of driven axles.
    virtual int GetNumDrivenAxles() const = 0;

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the axles of the
    /// specified suspension subsystems.
    virtual void Initialize(std::shared_ptr<ChBody> chassis,      ///< handle to the chassis body
                            const ChSuspensionList& suspensions,  ///< list of all vehicle suspension subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) = 0;

    /// Get a handle to the driveshaft.
    /// Return a shared pointer to the shaft that connects this driveline to a
    /// powertrain system (i.e., right after the transmission box).
    std::shared_ptr<ChShaft> GetDriveshaft() const { return m_driveshaft; }

    /// Get the angular speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to
    /// the powertrain system. The default implementation returns the driveline's
    /// driveshaft speed.
    virtual double GetDriveshaftSpeed() const { return m_driveshaft->GetPos_dt(); }

    /// Update the driveline subsystem: apply the specified motor torque.
    /// This represents the input to the driveline subsystem from the powertrain
    /// system. The default implementation applies this torque to the driveline's
    /// driveshaft.
    virtual void Synchronize(double torque) { m_driveshaft->SetAppliedTorque(torque); }

    /// Get the indexes of the vehicle's axles driven by this driveline subsystem.
    const std::vector<int>& GetDrivenAxleIndexes() const { return m_driven_axles; }

    /// Get the motor torque to be applied to the specified wheel.
    virtual double GetWheelTorque(const WheelID& wheel_id) const = 0;

  protected:
    std::shared_ptr<ChShaft> m_driveshaft;  ///< handle to the shaft connection to the powertrain

    std::vector<int> m_driven_axles;  ///< indexes of the driven vehicle axles
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
