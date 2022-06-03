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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Simple driveline model for a single axle open differential.
//
// =============================================================================

#ifndef CH_GENERIC_SIMPLE_DRIVELINE_H
#define CH_GENERIC_SIMPLE_DRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Simple driveline model for the generic vehicle (purely kinematic).
class CH_MODELS_API Generic_SimpleDriveline : public ChDrivelineWV {
  public:
    Generic_SimpleDriveline(const std::string& name);
    virtual ~Generic_SimpleDriveline() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "Generic_SimpleDriveline"; }

    /// Return the number of driven axles.
    virtual int GetNumDrivenAxles() const final override { return 1; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the specified axles.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,   ///< associated chassis subsystem
                            const ChAxleList& axles,              ///< list of all vehicle axle subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) override;

    /// Update the driveline subsystem: apply the specified motor torque.
    /// This represents the input to the driveline subsystem from the powertrain
    /// system.
    virtual void Synchronize(double time,                        ///< [in] current time
                             const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                             double torque                       ///< [in] motor torque
                             ) override;

    /// Get the motor torque to be applied to the specified spindle.
    virtual double GetSpindleTorque(int axle, VehicleSide side) const override;

    /// Disconnect driveline from driven wheels.
    virtual void Disconnect() override;

  private:
    static const double m_conicalgear_ratio;

    bool m_connected;
    std::shared_ptr<ChShaft> m_driven_left;
    std::shared_ptr<ChShaft> m_driven_right;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
