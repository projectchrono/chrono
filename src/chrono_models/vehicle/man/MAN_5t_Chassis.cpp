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
// MAN 5t chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/man/MAN_5t_Chassis.h"

namespace chrono {
namespace vehicle {
namespace man {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double MAN_5t_Chassis::m_mass = 7085;
// const ChVector<> MAN_5t_Chassis::m_inertiaXX(222.8, 944.1, 1053.5);
const ChVector<> MAN_5t_Chassis::m_inertiaXX(3441, 28485, 29395);
const ChVector<> MAN_5t_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> MAN_5t_Chassis::m_COM_loc(-1.748, 0, 0.744);
const ChCoordsys<> MAN_5t_Chassis::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MAN_5t_Chassis::MAN_5t_Chassis(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    //// TODO:
    //// A more appropriate contact shape from primitives
    ChRigidChassisGeometry::BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0),
                                          ChVector<>(1.0, 0.5, 0.2));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "MAN_Kat1/meshes/MAN_5t_chassis.obj";

    m_geometry.m_has_collision = (chassis_collision_type != ChassisCollisionType::NONE);
    switch (chassis_collision_type) {
        case ChassisCollisionType::PRIMITIVES:
            box1.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        case ChassisCollisionType::MESH: {
            ChRigidChassisGeometry::ConvexHullsShape hull("MAN_Kat1/meshes/MAN_5t_chassis_col.obj", 0);
            m_geometry.m_coll_hulls.push_back(hull);
            break;
        }
        default:
            break;
    }
}

void MAN_5t_Chassis::CreateContactMaterials(ChContactMethod contact_method) {
    // Create the contact materials.
    // In this model, we use a single material with default properties.
    MaterialInfo minfo;
    m_geometry.m_materials.push_back(minfo.CreateMaterial(contact_method));
}

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono
