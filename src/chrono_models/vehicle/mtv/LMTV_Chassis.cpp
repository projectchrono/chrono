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
// LMTV 2.5t chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mtv/LMTV_Chassis.h"

namespace chrono {
namespace vehicle {
namespace mtv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double LMTV_Chassis::m_mass = 3946;
// const ChVector<> LMTV_Chassis::m_inertiaXX(222.8, 944.1, 1053.5);
const ChVector<> LMTV_Chassis::m_inertiaXX(3.2282e3, 5.2323e3, 4.4980e3);
const ChVector<> LMTV_Chassis::m_inertiaXY(0, -0.4027e3, 0);
const ChVector<> LMTV_Chassis::m_COM_loc(-0.6972, 0, 0.6672);
const ChCoordsys<> LMTV_Chassis::m_driverCsys(ChVector<>(0.4, 0.7, 1.18), ChQuaternion<>(1, 0, 0, 0));

const double LMTV_Chassis::m_rear_mass = 1938.333;
const ChVector<> LMTV_Chassis::m_rear_inertiaXX(2.4461e3, 2.4605e3, 3.2300e3);
const ChVector<> LMTV_Chassis::m_rear_inertiaXY(0, -0.1055e3, 0);
const ChVector<> LMTV_Chassis::m_rear_COM_loc(-3.1919, 0, 0.8404);

const ChVector<> LMTV_Chassis::m_torsion_joint_pos(-3.1919, 0, 0.8404);
const double LMTV_Chassis::m_torsion_stiffness = 7085;  // 7.085;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LMTV_Chassis::LMTV_Chassis(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type)
    : ChTorsionChassis(name, fixed) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    m_rear_inertia(0, 0) = m_rear_inertiaXX.x();
    m_rear_inertia(1, 1) = m_rear_inertiaXX.y();
    m_rear_inertia(2, 2) = m_rear_inertiaXX.z();

    m_rear_inertia(0, 1) = m_rear_inertiaXY.x();
    m_rear_inertia(0, 2) = m_rear_inertiaXY.y();
    m_rear_inertia(1, 2) = m_rear_inertiaXY.z();
    m_rear_inertia(1, 0) = m_rear_inertiaXY.x();
    m_rear_inertia(2, 0) = m_rear_inertiaXY.y();
    m_rear_inertia(2, 1) = m_rear_inertiaXY.z();

    //// TODO:
    //// A more appropriate contact shape from primitives
    //// Add collision shapes for rear body

    double widthFrame = 0.905;
    double heightFrame = 0.2;
    double lx_front = 1.0 - m_torsion_joint_pos.x();
    ChVector<> frontBoxPos((1.0 + m_torsion_joint_pos.x()) / 2, 0, m_torsion_joint_pos.z());
    BoxShape boxFront(frontBoxPos, ChQuaternion<>(1, 0, 0, 0), ChVector<>(lx_front, widthFrame, heightFrame));

    double lx_rear = m_torsion_joint_pos.x() + 4.9;
    ChVector<> rearBoxPos((-4.9 + m_torsion_joint_pos.x()) / 2, 0, m_torsion_joint_pos.z());
    BoxShape boxRear(rearBoxPos, ChQuaternion<>(1, 0, 0, 0), ChVector<>(lx_rear, widthFrame, heightFrame));

    m_has_primitives = true;
    m_vis_boxes.push_back(boxFront);

    m_has_rear_primitives = true;
    m_rear_vis_boxes.push_back(boxRear);

    m_has_mesh = true;
    m_vis_mesh_file = "mtv/meshes/m1078_front.obj";

    m_has_rear_mesh = true;
    m_vis_rear_mesh_file = "mtv/meshes/m1078_rear.obj";

    m_has_collision = (chassis_collision_type != ChassisCollisionType::NONE);
    switch (chassis_collision_type) {
        case ChassisCollisionType::MESH:
            // For now, fall back to using primitive collision shapes
        case ChassisCollisionType::PRIMITIVES:
            boxFront.m_matID = 0;
            boxRear.m_matID = 0;
            m_coll_boxes.push_back(boxFront);
            break;
        default:
            break;
    }
}

void LMTV_Chassis::CreateContactMaterials(ChContactMethod contact_method) {
    // Create the contact materials.
    // In this model, we use a single material with default properties.
    MaterialInfo minfo;
    m_materials.push_back(minfo.CreateMaterial(contact_method));
}

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono
