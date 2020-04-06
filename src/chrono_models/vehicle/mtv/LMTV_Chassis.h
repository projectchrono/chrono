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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// LMTV cargo truck (2.5 tons) chassis subsystem.
//
// =============================================================================

#ifndef LMTV_CHASSIS_H
#define LMTV_CHASSIS_H

#include <string>

#include "chrono_vehicle/chassis/ChTorsionChassis.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace mtv {

/// @addtogroup vehicle_models_mtv
/// @{

/// LMTV chassis subsystem.
class CH_MODELS_API LMTV_Chassis : public ChTorsionChassis {
  public:
    LMTV_Chassis(const std::string& name,
                 bool fixed = false,
                 ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);
    ~LMTV_Chassis() {}

    /// Return the mass of the front chassis body.
    virtual double GetMass() const override { return m_mass; }

    /// Return the mass of the rear chassis body.
    virtual double GetRearMass() const override { return m_rear_mass; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetInertia() const override { return m_inertia; }

    /// Return the inertia tensor of the chassis body.
    virtual const ChMatrix33<>& GetRearInertia() const override { return m_rear_inertia; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetLocalPosCOM() const override { return m_COM_loc; }

    /// Get the location of the center of mass in the chassis frame.
    virtual const ChVector<>& GetRearLocalPosCOM() const override { return m_rear_COM_loc; }

    /// Get the local driver position and orientation.
    /// This is a coordinate system relative to the chassis reference frame.
    virtual ChCoordsys<> GetLocalDriverCoordsys() const override { return m_driverCsys; }

    /// Get the location of the torsion revolute joint.
    virtual const ChVector<>& GetTorsionJointLocalPos() const override { return m_torsion_joint_pos; }

    /// Get the torsion stiffness of the chassis
    virtual const double GetTorsionStiffness() const override { return m_torsion_stiffness; }

  protected:
    virtual void CreateContactMaterials(ChContactMethod contact_method) override;

    ChMatrix33<> m_inertia;
    ChMatrix33<> m_rear_inertia;

    static const double m_mass;
    static const ChVector<> m_inertiaXX;
    static const ChVector<> m_inertiaXY;
    static const ChVector<> m_COM_loc;
    static const ChCoordsys<> m_driverCsys;

    static const double m_rear_mass;
    static const ChVector<> m_rear_inertiaXX;
    static const ChVector<> m_rear_inertiaXY;
    static const ChVector<> m_rear_COM_loc;

    static const ChVector<> m_torsion_joint_pos;
    static const double m_torsion_stiffness;
};

/// @} vehicle_models_mtv

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
