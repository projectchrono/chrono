// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Automatic transmission model template based on a simple gear-shifting model.
//
// =============================================================================

#ifndef CH_SIMPLEMAP_AUTOMATIC_TRANSMISSION_H
#define CH_SIMPLEMAP_AUTOMATIC_TRANSMISSION_H

#include <utility>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTransmission.h"

#include "chrono/motion_functions/ChFunction_Recorder.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Template for an automatic transmission model based on a simple gear-shifting model.
/// This transmission has no torque converter.
/// It accepts a single reverse gear and any number of forward gears.
/// In automatic mode, gear shifting is done based on specified ideal shift points.
class CH_VEHICLE_API ChAutomaticTransmissionSimpleMap : public ChAutomaticTransmission {
  public:
    virtual ~ChAutomaticTransmissionSimpleMap() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "AutomaticTransmissionSimpleMap"; }

    /// Return true if a torque converter model is included.
    /// A ChAutomaticTransmissionSimpleMap does not model the torque converter.
    virtual bool HasTorqueConverter() const override { return false; }

    /// Return the value of slippage in the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterSlippage() const override { return 0; }

    /// Return the input torque to the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterInputTorque() const override { return 0; }

    /// Return the output torque from the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterOutputTorque() const override { return 0; }

    /// Return the torque converter output shaft speed.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterOutputSpeed() const override { return 0; }

    /// Return the transmission output torque on the driveshaft.
    /// This is the torque that is passed to the driveline subsystem, thus providing the interface between the
    /// powertrain and vehicle systems.
    virtual double GetOutputDriveshaftTorque() const override { return m_driveshaft_torque; }

    /// Return the transmission output speed of the motorshaft.
    /// This represents the output from the transmision subsystem that is passed to the engine subsystem.
    virtual double GetOutputMotorshaftSpeed() const override { return m_motorshaft_speed; }

  protected:
    ChAutomaticTransmissionSimpleMap(const std::string& name);

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify the min and max engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) = 0;

  private:
    /// Initialize the transmission system.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Update the state of this transmission system at the current time.
    /// The powertrain system is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_torque,           ///< input engine torque
                             double driveshaft_speed             ///< input driveline speed
                             ) override;

    /// Advance the state of this transmission system by the specified time step.
    /// This function does nothing for this simplified model.
    virtual void Advance(double step) override {}

    double m_motorshaft_speed;   ///< current motorshaft speed (transmission output)
    double m_driveshaft_torque;  ///< current driveshaft torque (transmission output)

    std::vector<std::pair<double, double>> m_shift_points;  ///< ideal shift points
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
