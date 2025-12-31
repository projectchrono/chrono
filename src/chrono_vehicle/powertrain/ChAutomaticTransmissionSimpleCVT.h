// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Automatic transmission model template to represent a CVT (Continuous Variable
// Transmission).
//
// =============================================================================

#ifndef CH_AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H
#define CH_AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H

#include <utility>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTransmission.h"

#include "chrono/functions/ChFunctionInterp.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChAutomaticTransmissionSimpleCVT : public ChAutomaticTransmission {
  public:
    virtual ~ChAutomaticTransmissionSimpleCVT() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "AutomaticTransmissionSimpleCVT"; }

    /// Return true if a torque converter model is included.
    /// A ChAutomaticTransmissionSimpleCVT does not model the torque converter.
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
    /// This represents the output from the transmission subsystem that is passed to the engine subsystem.
    virtual double GetOutputMotorshaftSpeed() const override { return m_motorshaft_speed; }

    /// Set the operation range
    void SetOperationRange(double driveshaft_speed_start,   // (rad/sec), begin gear variation
                           double ratio_start,              // () smallest possible gear ratio
                           double driveshaft_speed_end,     // (rad/sec), end gear variation
                           double ratio_end,                // () greatest possible gear ratio
                           double gearbox_efficiency = 1.0  // optional efficiency setting
    );

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

  protected:
    ChAutomaticTransmissionSimpleCVT(const std::string& name);

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

    virtual void PopulateComponentList() override;

    double m_motorshaft_speed;   ///< current motorshaft speed (transmission output)
    double m_driveshaft_torque;  ///< current driveshaft torque (transmission output)

    std::vector<std::pair<double, double>> m_shift_points;  ///< ideal shift points
    ChFunctionInterp m_cvt_gear_ratios;                         // function object for calculation of the current gear ratio
    double m_efficiency{1.0};  // allows to consider gearbox efficiency (usually 0.80 ... 0.86)
};

}  // namespace vehicle
}  // namespace chrono

#endif  // CH_AUTOMATIC_TRANSMISSION_SIMPLE_CVT_H
