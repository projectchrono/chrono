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

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"

#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsGearboxAngled.h"
#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsTorque.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// 4WD driveline model template based on ChShaft objects.
class CH_VEHICLE_API ChShaftsDriveline8WD : public ChDrivelineWV {
  public:
    ChShaftsDriveline8WD(const std::string& name);

    virtual ~ChShaftsDriveline8WD() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ShaftsDriveline8WD"; }

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

    /// Lock/unlock the differential on the specified axle.
    /// By convention, axles are counted front to back, starting with index 0 for the front-most axle.
    /// Pass axle=-1 to simultaneously lock/unlock both axle differentials.
    /// Differential locking is implemented through a friction torque between the output shafts
    /// of the differential. The locking effect is limited by a maximum locking torque.
    virtual void LockAxleDifferential(int axle, bool lock) override;

    /// Lock/unlock the central differential.
    /// Differential locking is implemented through a friction torque between the output shafts
    /// of the differential. The locking effect is limited by a maximum locking torque.
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

    /// Get the motor torque to be applied to the specified spindle.
    virtual double GetSpindleTorque(int axle, VehicleSide side) const override;

  protected:
    /// Return the inertia of the driveshaft.
    virtual double GetDriveshaftInertia() const = 0;
    /// Return the inertia of the differential box.
    virtual double GetCentralDifferentialBoxInertia() const = 0;
    /// Return the inertia of the front driveshaft.
    virtual double GetToFrontDiffShaftInertia() const = 0;
    /// Return the inertia of the rear driveshaft.
    virtual double GetToRearDiffShaftInertia() const = 0;
    /// Return the inertia of the rear differential box.
    virtual double GetRearDifferentialBoxInertia() const = 0;
    /// Return the inertia of the front differential box.
    virtual double GetFrontDifferentialBoxInertia() const = 0;

    /// Return the gear ratio for the rear conical gear.
    virtual double GetRearConicalGearRatio() const = 0;
    /// Return the gear ratio for the front conical gear.
    virtual double GetFrontConicalGearRatio() const = 0;

    /// Return the limit for the axle differential locking torque.
    virtual double GetAxleDifferentialLockingLimit() const = 0;

    /// Return the limit for the central differential locking torque.
    virtual double GetCentralDifferentialLockingLimit() const = 0;

  private:
    std::shared_ptr<ChShaftsPlanetary> m_central_differential;  ///< central differential
    std::shared_ptr<ChShaftsClutch> m_central_clutch;           ///< clutch for locking central differential

    std::shared_ptr<ChShaftsPlanetary> m_frontgroup_differential;  ///< frontgroup differential
    std::shared_ptr<ChShaftsClutch> m_frontgroup_clutch;           ///< clutch for locking frontgroup differential

    std::shared_ptr<ChShaftsPlanetary> m_reargroup_differential;  ///< reargroup differential
    std::shared_ptr<ChShaftsClutch> m_reargroup_clutch;           ///< clutch for locking reargroup differential

    std::shared_ptr<ChShaft> m_frontgroup_shaft;  ///< shaft from center diff to frontgroup diff
    std::shared_ptr<ChShaft> m_reargroup_shaft;   ///< shaft from center diff to reargroup diff

    std::shared_ptr<ChShaft> m_front1_shaft;  ///< shaft to front1 axle (#0)
    std::shared_ptr<ChShaft> m_front2_shaft;  ///< shaft to front2 axle (#1)
    std::shared_ptr<ChShaft> m_rear1_shaft;   ///< shaft to rear1 axle (#2)
    std::shared_ptr<ChShaft> m_rear2_shaft;   ///< shaft to rear2 axle (#3)

    std::shared_ptr<ChShaftsGearboxAngled> m_frontgroup_conicalgear;  ///< frontgroup conic gear
    std::shared_ptr<ChShaft> m_frontgroup_differentialbox;            ///< frontgroup differential casing

    std::shared_ptr<ChShaftsGearboxAngled> m_reargroup_conicalgear;  ///< reargroup conic gear
    std::shared_ptr<ChShaft> m_reargroup_differentialbox;            ///< group differential casing

    std::shared_ptr<ChShaftsGearboxAngled> m_rear1_conicalgear;  ///< rear1 conic gear
    std::shared_ptr<ChShaftsPlanetary> m_rear1_differential;     ///< rear1 differential
    std::shared_ptr<ChShaft> m_rear1_differentialbox;            ///< rear1 differential casing
    std::shared_ptr<ChShaftsClutch> m_rear1_clutch;              ///< clutch for locking rear1 differential

    std::shared_ptr<ChShaftsGearboxAngled> m_rear2_conicalgear;  ///< rear2 conic gear
    std::shared_ptr<ChShaftsPlanetary> m_rear2_differential;     ///< rear2 differential
    std::shared_ptr<ChShaft> m_rear2_differentialbox;            ///< rear2 differential casing
    std::shared_ptr<ChShaftsClutch> m_rear2_clutch;              ///< clutch for locking rear2 differential

    std::shared_ptr<ChShaftsGearboxAngled> m_front1_conicalgear;  ///< front1 conic gear
    std::shared_ptr<ChShaftsPlanetary> m_front1_differential;     ///< front1 differential
    std::shared_ptr<ChShaft> m_front1_differentialbox;            ///< front1 differential casing
    std::shared_ptr<ChShaftsClutch> m_front1_clutch;              ///< clutch for locking front1 differential

    std::shared_ptr<ChShaftsGearboxAngled> m_front2_conicalgear;  ///< front2 conic gear
    std::shared_ptr<ChShaftsPlanetary> m_front2_differential;     ///< front2 differential
    std::shared_ptr<ChShaft> m_front2_differentialbox;            ///< front2 differential casing
    std::shared_ptr<ChShaftsClutch> m_front2_clutch;              ///< clutch for locking front2 differential

    ChVector<> m_dir_motor_block;
    ChVector<> m_dir_axle;

    void CreateShafts(std::shared_ptr<ChChassis> chassis);
    void CreateDifferentials(std::shared_ptr<ChChassis> chassis, const ChAxleList& axles);
    void CreateClutches(std::shared_ptr<ChChassis> chassis, const ChAxleList& axles);
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
