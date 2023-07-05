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
// Base class for a vehicle transmission.
//
// =============================================================================

#ifndef CH_TRANSMISSION_H
#define CH_TRANSMISSION_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

class ChAutomaticTransmission;
class ChManualTransmission;

/// Base class for a transmission subsystem.
class CH_VEHICLE_API ChTransmission : public ChPart {
  public:
    /// Transmission type.
    enum class Type {
        AUTOMATIC,  ///< automatic transmission (torque converter + gearbox)
        MANUAL      ///< manual transmission (clutch + gearbox)
    };

    virtual ~ChTransmission() {}

    /// Get transmission type.
    virtual Type GetType() const = 0;

    /// Return true if automatic transmission.
    bool IsAutomatic() const { return GetType() == Type::AUTOMATIC; }

    /// Return true if manual transmission.
    bool IsManual() const { return GetType() == Type::MANUAL; }

    /// Return the current transmission gear.
    /// A return value of -1 indicates reverse, 0 indicates neutral and a positive value indicates a forward gear.
    int GetCurrentGear() const { return m_current_gear; }

    /// Return the highest available gear.
    int GetMaxGear() const { return((int) m_gear_ratios.size() - 1); }

    /// Shift to the specified gear.
    /// Reverse gear is -1, neutral is 0, forward gears are 1, 2, ..., Gmax.
    void SetGear(int gear);

    /// Return the transmission output torque on the driveshaft.
    /// This is the torque that is passed to the driveline subsystem, thus providing the interface between the
    /// powertrain and vehicle systems.
    virtual double GetOutputDriveshaftTorque() const = 0;

    /// Return the transmission output speed of the motorshaft.
    /// This represents the output from the transmision subsystem that is passed to the engine subsystem.
    virtual double GetOutputMotorshaftSpeed() const = 0;

    /// Shift up.
    virtual void ShiftUp() = 0;

    /// Shift down.
    virtual void ShiftDown() = 0;

  protected:
    ChTransmission(const std::string& name = "");

    /// Return this object as an automatic transmission.
    virtual ChAutomaticTransmission* asAutomatic() { return nullptr; }

    /// Return this object as a manual transmission.
    virtual ChManualTransmission* asManual() { return nullptr; }

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Set the transmission gear ratios (one or more forward gear ratios and a single reverse gear ratio).
    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) = 0;

    /// Perform any action required on a gear shift (the new gear and gear ratio are available).
    virtual void OnGearShift() {}

    /// Perform any action required on placing the transmission in neutral.
    virtual void OnNeutralShift() {}

    /// Initialize this transmission system by attaching it to an existing vehicle chassis and connecting the provided
    /// engine and driveline subsystems. A derived class override must first call this base class implementation.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis);

    /// Update the transmission system at the current time.
    /// The motorshaft torque represents the input to the transmission from the engine system.
    /// The driveshaft speed represents the input to the transmission from the driveline system.
    /// This default implementation sets the driveshaft speed and the motorshaft torque from the provided values.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_torque,           ///< input engine torque
                             double driveshaft_speed             ///< input driveline speed
                             ) = 0;

    /// Advance the state of this powertrain system by the specified time step.
    virtual void Advance(double step) {}


    std::vector<double> m_gear_ratios;  ///< gear ratios (0: reverse, 1+: forward)
    int m_current_gear;                 ///< current transmission gear (0: reverse, 1+: forward)
    double m_current_gear_ratio;        ///< current gear ratio (positive for forward, negative for reverse)

    friend class ChPowertrainAssembly;
    friend class ChVehicleVisualSystemIrrlicht;
    friend class ChInteractiveDriverIRR;
    friend class ChVehicleVisualSystemVSG;
    friend class ChVehicleKeyboardHandlerVSG;
    friend class ChVehicleGuiComponentVSG;
};

// -----------------------------------------------------------------------------

class CH_VEHICLE_API ChAutomaticTransmission : public ChTransmission {
  public:
    /// Driving modes.
    enum class DriveMode {
        FORWARD,  ///< vehicle moving forward
        NEUTRAL,  ///< vehicle in neutral
        REVERSE   ///< vehicle moving backward
    };

    /// Transmission shift mode.
    enum class ShiftMode {
        AUTOMATIC,  ///< automatic transmission
        MANUAL      ///< manual (manumatic) transmission
    };

    /// Get transmission type.
    virtual Type GetType() const override final { return Type::AUTOMATIC; }

    /// Set the drive mode.
    void SetDriveMode(DriveMode mode);

    /// Return the current drive mode.
    DriveMode GetDriveMode() const { return m_drive_mode; }

    /// Return true if a torque converter model is included.
    virtual bool HasTorqueConverter() const = 0;

    /// Return the value of slippage in the torque converter.
    virtual double GetTorqueConverterSlippage() const = 0;

    /// Return the input torque to the torque converter.
    virtual double GetTorqueConverterInputTorque() const = 0;

    /// Return the output torque from the torque converter.
    virtual double GetTorqueConverterOutputTorque() const = 0;

    /// Return the torque converter output shaft speed.
    virtual double GetTorqueConverterOutputSpeed() const = 0;

    /// Set the transmission shift mode (automatic or manual).
    /// Note that a derived transmission class may ignore this is the selected mode is not supported.
    void SetShiftMode(ShiftMode mode) { m_shift_mode = mode; }

    /// Get the current transmission shift mode.
    ShiftMode GetShiftMode() const { return m_shift_mode; }

    void ShiftUp() override;

    void ShiftDown() override;

  protected:
    ChAutomaticTransmission(const std::string& name);

    DriveMode m_drive_mode;  ///< drive mode (neutral, forward, or reverse)
    ShiftMode m_shift_mode;  ///< transmission shift mode (automatic or manual)

  private:
    /// Return this object as an automatic transmission.
    virtual ChAutomaticTransmission* asAutomatic() override final { return this; }
};

// -----------------------------------------------------------------------------

class CH_VEHICLE_API ChManualTransmission : public ChTransmission {
  public:
    /// Get transmission type.
    virtual Type GetType() const override final { return Type::MANUAL; }

    /// Return true if a clutch model is included.
    virtual bool HasClutch() const = 0;

    /// Shifts up a gear, including from reverse to neutral to 1st.
    void ShiftUp() override;

    /// Shifts down a gear, including from 1st to neutral and to reverse.
    void ShiftDown() override;

  protected:
    ChManualTransmission(const std::string& name);

  private:
    /// Return this object as an automatic transmission.
    virtual ChManualTransmission* asManual() override final { return this; }
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
