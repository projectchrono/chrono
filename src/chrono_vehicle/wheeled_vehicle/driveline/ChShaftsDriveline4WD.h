// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// 4WD driveline model template based on ChShaft objects.
//
// =============================================================================

#ifndef CH_SHAFTS_DRIVELINE_4WD_H
#define CH_SHAFTS_DRIVELINE_4WD_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChDriveline.h"

#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsGearboxAngled.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsMotor.h"
#include "chrono/physics/ChShaftsTorque.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_driveline
/// @{

/// 4WD driveline model template based on ChShaft objects.
class CH_VEHICLE_API ChShaftsDriveline4WD : public ChDriveline {
  public:
    ChShaftsDriveline4WD(const std::string& name);

    virtual ~ChShaftsDriveline4WD() {}

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

    /// Return the number of driven axles.
    /// A ChShaftsDriveline4WD driveline connects to two axles.
    virtual int GetNumDrivenAxles() const final override { return 2; }

    /// Initialize the driveline subsystem.
    /// This function connects this driveline subsystem to the axles of the
    /// specified suspension subsystems.
    virtual void Initialize(std::shared_ptr<ChBody> chassis,      ///< handle to the chassis body
                            const ChSuspensionList& suspensions,  ///< list of all vehicle suspension subsystems
                            const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles
                            ) override;

    /// Get the motor torque to be applied to the specified wheel.
    virtual double GetWheelTorque(const WheelID& wheel_id) const override;

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
    /// Return the gear ratio for the rear differential.
    virtual double GetRearDifferentialRatio() const = 0;
    /// Return the gear ratio for the front differential.
    virtual double GetFrontDifferentialRatio() const = 0;
    /// Return the gear ratio for the central differential.
    virtual double GetCentralDifferentialRatio() const = 0;

  private:
    std::shared_ptr<ChShaftsPlanetary> m_central_differential;
    std::shared_ptr<ChShaft> m_front_shaft;
    std::shared_ptr<ChShaft> m_rear_shaft;
    std::shared_ptr<ChShaftsGearboxAngled> m_rear_conicalgear;
    std::shared_ptr<ChShaftsPlanetary> m_rear_differential;
    std::shared_ptr<ChShaft> m_rear_differentialbox;
    std::shared_ptr<ChShaftsGearboxAngled> m_front_conicalgear;
    std::shared_ptr<ChShaftsPlanetary> m_front_differential;
    std::shared_ptr<ChShaft> m_front_differentialbox;

    ChVector<> m_dir_motor_block;
    ChVector<> m_dir_axle;
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
