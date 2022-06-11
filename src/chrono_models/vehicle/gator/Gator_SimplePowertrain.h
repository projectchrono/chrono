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
// Simple powertrain model for the Gator vehicle.
// - linear speed-torque curve
// - no torque converter
// - no transmission box (single forward gear, single reverse gear)
//
// =============================================================================

#ifndef GATOR_SIMPLEPOWERTRAIN_H
#define GATOR_SIMPLEPOWERTRAIN_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace gator {

/// @addtogroup vehicle_models_gator
/// @{

/// Simple Gator powertrain subsystem (DC motor linear torque-speed characteristic).
class CH_MODELS_API Gator_SimplePowertrain : public ChPowertrain {
  public:
    Gator_SimplePowertrain(const std::string& name);
    ~Gator_SimplePowertrain() {}

    virtual std::string GetTemplateName() const override { return "SimplePowertrain"; }
    virtual double GetMotorSpeed() const override { return m_motorSpeed; }
    virtual double GetMotorTorque() const override { return m_motorTorque; }
    virtual double GetTorqueConverterSlippage() const override { return 0; }
    virtual double GetTorqueConverterInputTorque() const override { return 0; }
    virtual double GetTorqueConverterOutputTorque() const override { return 0; }
    virtual double GetTorqueConverterOutputSpeed() const override { return 0; }
    virtual double GetOutputTorque() const override { return m_shaftTorque; }

  private:
    virtual void Synchronize(double time, const DriverInputs& driver_inputs, double shaft_speed) override;
    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    double m_motorSpeed;
    double m_motorTorque;
    double m_shaftTorque;

    static const double m_fwd_gear_ratio;  // forward gear ratio (single gear transmission)
    static const double m_rev_gear_ratio;  // reverse gear ratio
    static const double m_max_torque;      // maximum motor torque
    static const double m_max_speed;       // maximum motor speed
};

/// @} vehicle_models_gator

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono

#endif
