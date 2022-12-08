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
// Authors: Radu Serban, Mike Taylor, Asher Elmquist
// =============================================================================
//
// Simple powertrain model template.
// - based on torque-speed engine maps
// - both power and torque limited
// - no torque converter
// - simple gear-shifting model (in automatic mode)
//
// =============================================================================

#ifndef CH_SIMPLEMAP_POWERTRAIN_H
#define CH_SIMPLEMAP_POWERTRAIN_H

#include <utility>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"

#include "chrono/motion_functions/ChFunction_Recorder.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Template for simple powertrain model based on speed-torque engine maps.
/// This model has no torque converter and can have either a manual or an automatic transmission.
/// It accepts a single reverse gear and any number of forward gears.
/// In automatic mode, gear shifting is done based on specified ideal shift points.
class CH_VEHICLE_API ChSimpleMapPowertrain : public ChPowertrain {
  public:
    // Construct the powertrain model, by default with an automatic transmission.
    ChSimpleMapPowertrain(const std::string& name);

    virtual ~ChSimpleMapPowertrain() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "SimpleMapPowertrain"; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_motor_speed; }

    /// Return the current total engine torque.
    virtual double GetMotorTorque() const override { return m_motor_torque; }

    /// Return the current engine resistance torque.
    double GetMotorResistance() const;

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

    /// Return the output torque from the powertrain.
    /// This is the torque that is passed to a vehicle system, thus providing the
    /// interface between the powertrain and vehicle co-simulation modules.
    virtual double GetOutputTorque() const override { return m_shaft_torque; }

  protected:
    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() = 0;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps, using the
    /// ChFunction_Recorder::AddPoint() function.
    virtual void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                                     ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                                     ) = 0;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and maximum engine speed for shifting (down
    /// and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) = 0;

    /// Set coefficients for motor resistance.
    /// The motor resistance is calculated as: resistance_torque = c0 + c1 * m_motor_speed.
    /// This default implementation sets both coefficients to zero (no motor resistance).
    virtual void SetMotorResistanceCoefficients(double& c0, double& c1) {}

  private:
    /// Initialize the powertrain system.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Update the state of this powertrain system at the current time.
    /// The powertrain system is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,                        ///< [in] current time
                             const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                             double shaft_speed                  ///< [in] driveshaft speed
                             ) override;

    /// Advance the state of this powertrain system by the specified time step.
    /// This function does nothing for this simplified powertrain model.
    virtual void Advance(double step) override {}

    double m_motor_speed;   ///< current engine speed
    double m_motor_torque;  ///< current engine torque
    double m_shaft_torque;  ///< current driveshaft torque

    std::vector<std::pair<double, double>> m_shift_points;  ///< ideal shift points

    ChFunction_Recorder m_zero_throttle_map;  ///< engine map at zero throttle
    ChFunction_Recorder m_full_throttle_map;  ///< engine map at full throttle

    double m_motor_resistance_c0;  ///< coefficient for motor resistance
    double m_motor_resistance_c1;  ///< coefficient for motor resistance
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
