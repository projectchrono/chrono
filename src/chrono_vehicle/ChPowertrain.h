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
// Base class for a vehicle powertrain.
//
// =============================================================================

#ifndef CH_POWERTRAIN_H
#define CH_POWERTRAIN_H

#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChShaft.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChDriveline.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Base class for a powertrain system.
class CH_VEHICLE_API ChPowertrain : public ChPart {
  public:
    /// Driving modes.
    enum DriveMode {
        FORWARD,  ///< vehicle moving forward
        NEUTRAL,  ///< vehicle in neutral
        REVERSE   ///< vehicle moving backward
    };

    ChPowertrain(const std::string& name);

    virtual ~ChPowertrain() {}

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const = 0;

    /// Return the current engine torque.
    virtual double GetMotorTorque() const = 0;

    /// Return the value of slippage in the torque converter.
    virtual double GetTorqueConverterSlippage() const = 0;

    /// Return the input torque to the torque converter.
    virtual double GetTorqueConverterInputTorque() const = 0;

    /// Return the output torque from the torque converter.
    virtual double GetTorqueConverterOutputTorque() const = 0;

    /// Return the torque converter output shaft speed.
    virtual double GetTorqueConverterOutputSpeed() const = 0;

    /// Return the current transmission gear.
    virtual int GetCurrentTransmissionGear() const = 0;

    /// Return the output torque from the powertrain.
    /// This is the torque that is passed to a vehicle system, thus providing the
    /// interface between the powertrain and vehicle co-simulation modules.
    virtual double GetOutputTorque() const = 0;

    /// Return the current mode of the transmission.
    DriveMode GetDriveMode() { return m_drive_mode; }

    /// Set the mode of the transmission.
    virtual void SetDriveMode(DriveMode mmode) = 0;

  protected:
    // Note: Users should not directly call these functions. The vehicle system (whether wheeled or tracked)
    // intermediates calls to these functions.

    /// Initialize this powertrain system by attaching it to an existing vehicle chassis and associating it with an
    /// existing driveline subsystem. A derived class override must first call this base class version.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,     ///< [in] chassis of the associated vehicle
                            std::shared_ptr<ChDriveline> driveline  ///< [in] driveline of the associated vehicle
    );

    /// Synchronize the state of this powertrain system at the current time.
    /// The powertrain system is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,     ///< [in] current time
                             double throttle  ///< [in] current throttle input [0,1]
                             ) = 0;

    /// Advance the state of this powertrain system by the specified time step.
    virtual void Advance(double step) {}

    DriveMode m_drive_mode;                    ///< drive mode (neutral, forward, or reverse)
    std::shared_ptr<ChDriveline> m_driveline;  ///< associated driveline subsystem

    friend class ChWheeledVehicle;
    friend class ChTrackedVehicle;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
