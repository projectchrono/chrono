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
// 2WD driveline model template based on ChShaft objects. This template can be
// used to model either a FWD or a RWD driveline.
//
// =============================================================================

#ifndef CH_SHAFTS_DRIVELINE_2WD_H
#define CH_SHAFTS_DRIVELINE_2WD_H

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

/// 2WD driveline model template based on ChShaft objects. This template can be
/// used to model either a FWD or a RWD driveline.
class CH_VEHICLE_API ChShaftsDriveline2WD : public ChDriveline {
  public:
    ChShaftsDriveline2WD(const std::string& name);

    virtual ~ChShaftsDriveline2WD() {}

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
    /// A ChShaftsDriveline2WD driveline connects to a single axle.
    virtual int GetNumDrivenAxles() const final override { return 1; }

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
    virtual double GetDifferentialBoxInertia() const = 0;

    /// Return the gear ratio for the conical gear.
    virtual double GetConicalGearRatio() const = 0;

    /// Return the gear ratio for the differential.
    virtual double GetDifferentialRatio() const = 0;

  private:
    std::shared_ptr<ChShaftsGearboxAngled> m_conicalgear;
    std::shared_ptr<ChShaft> m_differentialbox;
    std::shared_ptr<ChShaftsPlanetary> m_differential;

    ChVector<> m_dir_motor_block;
    ChVector<> m_dir_axle;
};

/// @} vehicle_wheeled_driveline

}  // end namespace vehicle
}  // end namespace chrono

#endif
