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
// Authors:
// =============================================================================
//
// Track driveline model template based on ChShaft objects.
//
// =============================================================================

#ifndef CH_TRACK_DRIVELINE_BDS_H
#define CH_TRACK_DRIVELINE_BDS_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/tracked_vehicle/ChDrivelineTV.h"

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsGearboxAngled.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsClutch.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_driveline
/// @{

/// Track driveline model template based on ChShaft objects.
class CH_VEHICLE_API ChTrackDrivelineBDS : public ChDrivelineTV {
  public:
    ChTrackDrivelineBDS(const std::string& name  ///< [in] name of the subsystem
                        );

    virtual ~ChTrackDrivelineBDS();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "TrackDrivelineBDS"; }

    /// Set the direction of the motor block.
    /// This direction is a unit vector, relative to the chassis frame (for the
    /// ISO coordinate system, this is [1, 0, 0] for a longitudinal engine and
    /// [0, 1, 0] for a transversal engine).
    void SetMotorBlockDirection(const ChVector<>& dir) { m_dir_motor_block = dir; }

    /// Set the direction of the wheel axles.
    /// This direction is a unit vector, relative to the chassis frame. It must be
    /// specified for the design configuration (for the ISO vehicle coordinate
    /// system, this is typically [0, 1, 0]).
    void SetAxleDirection(const ChVector<>& dir) { m_dir_axle = dir; }

    /// Lock/unlock the differential (if available).
    virtual void LockDifferential(bool lock) override;

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the sprockets of the two track assembly subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,           ///< associated chassis subsystem
                            std::shared_ptr<ChTrackAssembly> track_left,  ///< left track assembly
                            std::shared_ptr<ChTrackAssembly> track_right  ///< right track assembly
                            ) override;

    /// Update the driveline subsystem.
    /// The motor torque represents the input to the driveline subsystem from the powertrain system.
    /// Apply the provided torque to the driveshaft.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double driveshaft_torque            ///< input transmission torque
                             ) override;

    /// Get the motor torque to be applied to the specified sprocket.
    virtual double GetSprocketTorque(VehicleSide side) const override;

    /// Get the angular speed of the specified sprocket.
    virtual double GetSprocketSpeed(VehicleSide side) const override;

    /// Disconnect driveline from driven sprockets.
    virtual void Disconnect() override;

    /// Return the output driveline speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to the transmission subsystem.
    virtual double GetOutputDriveshaftSpeed() const override { return m_driveshaft->GetPos_dt(); }

  protected:
    /// Return the inertia of the driveshaft.
    virtual double GetDriveshaftInertia() const = 0;
    /// Return the inertia of the differential box.
    virtual double GetDifferentialBoxInertia() const = 0;

    /// Return the gear ratio for the conical gear.
    virtual double GetConicalGearRatio() const = 0;

    /// Return the limit for the differential locking torque.
    virtual double GetDifferentialLockingLimit() const = 0;

  private:
    virtual void CombineDriverInputs(const DriverInputs& driver_inputs,
                                     double& braking_left,
                                     double& braking_right) override;

    std::shared_ptr<ChShaft> m_driveshaft;                 ///< shaft connection to the transmission
    std::shared_ptr<ChShaftsGearboxAngled> m_conicalgear;  ///< conical gear
    std::shared_ptr<ChShaft> m_differentialbox;            ///< differential casing
    std::shared_ptr<ChShaftsPlanetary> m_differential;     ///< planetary differential
    std::shared_ptr<ChShaftsClutch> m_clutch;              ///< clutch for locking differential

    ChVector<> m_dir_motor_block;
    ChVector<> m_dir_axle;
};

/// @} vehicle_tracked_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
