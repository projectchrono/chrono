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
// Gator simple driveline model.
//
// =============================================================================

#ifndef GATOR_SIMPLEDRIVELINE_H
#define GATOR_SIMPLEDRIVELINE_H

#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace gator {

/// @addtogroup vehicle_models_gator
/// @{

/// Simple Gator driveline subsystem (purely kinematic).
class CH_MODELS_API Gator_SimpleDriveline : public ChDrivelineWV {
  public:
    Gator_SimpleDriveline(const std::string& name);

    ~Gator_SimpleDriveline() {}

    virtual std::string GetTemplateName() const override { return "GatorCustomDriveline"; }

    virtual int GetNumDrivenAxles() const override { return 1; }  // RWD

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the specified axle subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,   ///< associated chassis subsystem
                            const ChAxleList& axles,              ///< list of all vehicle axle subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) override;

    /// Update the driveline subsystem: apply the specified motor torque.
    /// This represents the input to the driveline subsystem from the powertrain system.
    virtual void Synchronize(double time,                        ///< [in] current time
                             const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                             double torque                       ///< [in] motor torque
                             ) override;

    /// Get the motor torque to be applied to the specified spindle.
    virtual double GetSpindleTorque(int axle, VehicleSide side) const override;

    /// Disconnect driveline from driven wheels.
    virtual void Disconnect() override;

  private:
    bool m_connected;
    std::shared_ptr<ChShaft> m_left;
    std::shared_ptr<ChShaft> m_right;

    static const double m_diff_bias;
};

/// @} vehicle_models_gator

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono

#endif
