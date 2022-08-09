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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Simple driveline model. This template can be used to model a XWD driveline.
// Number of axles can be 1 to X.
// It uses a constant torque split depending on the number of axles driven and a
// simple model for Torsen limited-slip differentials.
//
// =============================================================================

#ifndef CH_SIMPLE_DRIVELINE_XWD_H
#define CH_SIMPLE_DRIVELINE_XWD_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// Simple driveline model. This template can be used to model a XWD driveline.
/// Number of axles can be 1 to X.
/// It uses a constant torque split depending on the number of axles driven and a
/// simple model for Torsen limited-slip differentials.
class CH_VEHICLE_API ChSimpleDrivelineXWD : public ChDrivelineWV {
  public:
    ChSimpleDrivelineXWD(const std::string& name);

    virtual ~ChSimpleDrivelineXWD() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SimpleDrivelineXWD"; }

    /// Return the number of driven axles.
    virtual int GetNumDrivenAxles() const override { return static_cast<int>(m_shaft_left.size()); }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the specified axle subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,   ///< associated chassis subsystem
                            const ChAxleList& axles,              ///< list of all vehicle axle subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) override;

    /// Update the driveline subsystem: apply the specified motor torque.
    /// This represents the input to the driveline subsystem from the powertrain system.
    virtual void Synchronize(double time,                            ///< [in] current time
                             const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                             double torque                           ///< [in] motor torque
                             ) override;

    /// Get the motor torque to be applied to the specified spindle.
    virtual double GetSpindleTorque(int axle, VehicleSide side) const override;

    /// Disconnect driveline from driven wheels.
    virtual void Disconnect() override;

  protected:
    /// Return the torque bias ratio every axlewise differential.
    /// This is a simple model of a Torsen limited-slip differential.
    virtual double GetDifferentialMaxBias() const = 0;

  private:
    bool m_connected;

    std::vector<std::shared_ptr<ChShaft> > m_shaft_left;
    std::vector<std::shared_ptr<ChShaft> > m_shaft_right;
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
