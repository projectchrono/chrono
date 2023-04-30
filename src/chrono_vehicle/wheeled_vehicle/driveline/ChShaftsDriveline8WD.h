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
// Authors: Alessandro Tasora, Radu Serban, Rainer Gericke
// =============================================================================
//
// 8WD driveline model template based on ChShaft objects.
// it is based on ChDriveline4WD, we use
//  4 axle differentials
//  2 interaxle differentials
//  1 interaxlegroup differential
//
//  all differentials can be locked individually or together
//
// =============================================================================

#ifndef CH_SHAFTS_DRIVELINE_8WD_H
#define CH_SHAFTS_DRIVELINE_8WD_H

#include <array>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsGearboxAngled.h"
#include "chrono/physics/ChShaftsPlanetary.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// 4WD driveline model template based on ChShaft objects.
class CH_VEHICLE_API ChShaftsDriveline8WD : public ChDrivelineWV {
  public:
    ChShaftsDriveline8WD(const std::string& name);

    virtual ~ChShaftsDriveline8WD();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ShaftsDriveline8WD"; }

    /// Set the direction of the motor block.
    /// This direction is a unit vector, relative to the chassis frame (for the ISO coordinate system, this is [1, 0, 0]
    /// for a longitudinal engine and [0, 1, 0] for a transversal engine).
    void SetMotorBlockDirection(const ChVector<>& dir) { m_dir_motor_block = dir; }

    /// Set the direction of the wheel axles.
    /// This direction is a unit vector, relative to the chassis frame. It must be specified for the design
    /// configuration (for the ISO vehicle coordinate system, this is typically [0, 1, 0]).
    void SetAxleDirection(const ChVector<>& dir) { m_dir_axle = dir; }

    /// Lock/unlock the differential on the specified axle.
    /// By convention, axles are counted front to back, starting with index 0 for the front-most axle.
    /// Pass axle=-1 to simultaneously lock/unlock both axle differentials. Differential locking is implemented through
    /// a friction torque between the output shafts of the differential. The locking effect is limited by a maximum
    /// locking torque.
    virtual void LockAxleDifferential(int axle, bool lock) override;

    /// Lock/unlock the specified central or group differential.
    /// Pass which=0 to lock/unlock the group 0 differential, which=1 to lock/unlock the group 1 differential, and
    /// which=2 to lock/unlock the centrtal differential.  Pass which=-1 to lock/unlock all 3 differential
    /// simultaneously. Differential locking is implemented through a friction torque between the output shafts of the
    /// differential. The locking effect is limited by a maximum locking torque.
    virtual void LockCentralDifferential(int which, bool lock) override;

    /// Return the number of driven axles.
    /// A ChShaftsDriveline8WD driveline connects to two axles.
    virtual int GetNumDrivenAxles() const final override { return 4; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the specified axle subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis,   ///< associated chassis subsystem
                            const ChAxleList& axles,              ///< list of all vehicle axle subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) override;

    /// Update the driveline subsystem.
    /// The motor torque represents the input to the driveline subsystem from the powertrain system.
    /// Apply the provided torque to the driveshaft.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double driveshaft_torque            ///< input transmission torque
                             ) override;

    /// Get the motor torque to be applied to the specified spindle.
    virtual double GetSpindleTorque(int axle, VehicleSide side) const override;

    /// Disconnect driveline from driven wheels.
    virtual void Disconnect() override;

    /// Return the output driveline speed of the driveshaft.
    /// This represents the output from the driveline subsystem that is passed to the transmission subsystem.
    virtual double GetOutputDriveshaftSpeed() const override { return m_driveshaft->GetPos_dt(); }

  protected:
    /// Return the inertia of the driveshaft.
    virtual double GetDriveshaftInertia() const = 0;
    /// Return the inertia of the group differential input shaft.
    virtual double GetGroupDiffInputShaftInertia() const = 0;
    /// Return the inertia of the axle differential input shaft.
    virtual double GetAxleDiffInputShaftInertia() const = 0;
    /// Return the inertia of the axle differential casing.
    virtual double GetAxleDiffBoxInertia() const = 0;

    /// Return the gear ratio for the axle differential conical gear.
    virtual double GetAxleDiffConicalGearRatio() const = 0;

    /// Return the limit for the central differential locking torque.
    virtual double GetCentralDifferentialLockingLimit() const = 0;
    /// Return the limit for the group differential locking limit.
    virtual double GetGroupDifferentialLockingLimit() const = 0;
    /// Return the limit for the axle differential locking torque.
    virtual double GetAxleDifferentialLockingLimit() const = 0;

  private:
    std::shared_ptr<ChShaft> m_driveshaft;  ///< shaft connection to the transmission

    // Central differential
    std::shared_ptr<ChShaftsPlanetary> m_central_differential;  ///< central differential
    std::shared_ptr<ChShaftsClutch> m_central_clutch;           ///< clutch for locking central differential

    // Group differentials
    std::array<std::shared_ptr<ChShaft>, 2> m_GD_inshaft;                 ///< input shafts
    std::array<std::shared_ptr<ChShaftsPlanetary>, 2> m_GD_differential;  ///< differentials
    std::array<std::shared_ptr<ChShaftsClutch>, 2> m_GD_clutch;           ///< differential locking clutches

    // Axle differentials
    std::array<std::shared_ptr<ChShaft>, 4> m_AD_inshaft;                    ///< input shafts
    std::array<std::shared_ptr<ChShaftsGearboxAngled>, 4> m_AD_conicalgear;  ///< conical gears
    std::array<std::shared_ptr<ChShaftsPlanetary>, 4> m_AD_differential;     ///< differentials
    std::array<std::shared_ptr<ChShaft>, 4> m_AD_differentialbox;            ///< differential casings
    std::array<std::shared_ptr<ChShaftsClutch>, 4> m_AD_clutch;              ///< differential locking clutches

    ChVector<> m_dir_motor_block;
    ChVector<> m_dir_axle;
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
