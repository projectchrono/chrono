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

#include <vector>
#include <utility>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"

#include "chrono/motion_functions/ChFunction_Recorder.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Simple powertrain model, based on speed-torque engine maps.
/// This model has no torque converter and can have either a manual or an automatic transmission.
/// It accepts a single reverse gear and any number of forward gears.
/// In automatic mode, gear shifting is done based on specified ideal shift points.
class CH_VEHICLE_API ChSimpleMapPowertrain : public ChPowertrain {
  public:
    // Construct the powertrain model, by default with an automatic transmission.
    ChSimpleMapPowertrain();

    virtual ~ChSimpleMapPowertrain() {}

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_motor_speed; }

    /// Return the current engine torque.
    virtual double GetMotorTorque() const override { return m_motor_torque; }

    /// Return the value of slippage in the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterSlippage() const override { return 0; }

    /// Return the input torque to the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterInputTorque() const override { return 0; }

    /// Return the output torque from the torque converter.
    /// This simplified model does not have a torque converter.
    virtual double GetTorqueConverterOutputTorque() const override { return 0; }

    /// Return the current transmission gear.
    /// Returns 0 if in REVERSE, or the current gear if in FORWARD mode.
    virtual int GetCurrentTransmissionGear() const override;

    /// Return the output torque from the powertrain.
    /// This is the torque that is passed to a vehicle system, thus providing the
    /// interface between the powertrain and vehicle co-simulation modules.
    virtual double GetOutputTorque() const override { return m_shaft_torque; }

    /// Use this function to set the mode of the transmission box.
    virtual void SetDriveMode(ChPowertrain::DriveMode mode) override;

    /// Enable/disable automatic transmission.
    /// If enabled, gear selection is based on the provided ideal shift points.
    void EnableAutomaticTransmission(bool automatic);

    /// Set the current forward gear (1, 2, ...).
    /// This function has no effect if the transmission is set to automatic or if
    /// currently not in FORWARD drive mode.
    void SetForwardGear(int igear);

    /// Initialize the powertrain system.
    virtual void Initialize(std::shared_ptr<ChBody> chassis,     ///< [in] chassis o the associated vehicle
                            std::shared_ptr<ChShaft> driveshaft  ///< [in] shaft connection to the vehicle driveline
                            ) override;

    /// Update the state of this powertrain system at the current time.
    /// The powertrain system is provided the current driver throttle input, a
    /// value in the range [0,1], and the current angular speed of the transmission
    /// shaft (from the driveline).
    virtual void Synchronize(double time,        ///< [in] current time
                             double throttle,    ///< [in] current throttle input [0,1]
                             double shaft_speed  ///< [in] current angular speed of the transmission shaft
                             ) override;

    /// Advance the state of this powertrain system by the specified time step.
    /// This function does nothing for this simplified powertrain model.
    virtual void Advance(double step) override {}

  protected:
    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() = 0;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps,
    /// using the ChFunction_Recorder::AddPoint() function.
    virtual void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< [out] engine map at zero throttle
                                     ChFunction_Recorder& mapF   ///< [out] engine map at full throttle
                                     ) = 0;

    /// Set the gears, i.e. the transmission ratios of the various gears.
    /// A concrete class must populate the vector of forward gear ratios, ordered as 1st, 2nd, etc.
    /// and provide a value for the single reverse gear ratio.
    virtual void SetGearRatios(std::vector<double>& fwd_gear_ratios,  ///< [out] list of forward gear ratios
                               double& reverse_gear_ratio             ///< [out] single reverse gear ratio
                               ) = 0;

    /// Set the ideal shift points for automatic gear shifting.
    /// For each forward gear, specify a pair (min, max) with the minimum and
    /// maximum engine speed for shifting (down and up, respectively).
    virtual void SetShiftPoints(
        std::vector<std::pair<double, double>>& shift_bands  ///< [out] down-shift/up-shift points
        ) = 0;

  private:
    bool m_automatic;    ///< manual or automatic transmission
    bool m_initialized;  ///< set to 'true' when the powertrain is initialized

    double m_motor_speed;   ///< current engine speed
    double m_motor_torque;  ///< current engine torque
    double m_shaft_torque;  ///< current driveshaft torque

    double m_rev_gear_ratio;                                ///< reverse gear ratio
    std::vector<double> m_gear_ratios;                      ///< set of forward gear ratios
    std::vector<std::pair<double, double>> m_shift_points;  ///< ideal shift points

    int m_current_gear;           ///< current selected forward gear (0, 1, 2, ...)
    double m_current_gear_ratio;  ///< current transmission gear ratio

    ChFunction_Recorder m_zero_throttle_map;  ///< engine map at zero throttle
    ChFunction_Recorder m_full_throttle_map;  ///< engine map at full throttle

    /// If in automatic transmission mode, select the forward gear based on ideal shift points.
    void CheckShift();
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
