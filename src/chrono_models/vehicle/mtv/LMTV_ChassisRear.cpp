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
// LMTV cargo truck (2.5 tons) chassis subsystems.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mtv/LMTV_ChassisRear.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// Static variables

const double LMTV_ChassisRear::m_body_mass = 1938.333;
const ChVector<> LMTV_ChassisRear::m_body_inertiaXX(2.4461e3, 2.4605e3, 3.2300e3);
const ChVector<> LMTV_ChassisRear::m_body_inertiaXY(0, -0.1055e3, 0);
const ChVector<> LMTV_ChassisRear::m_body_COM_loc(-3.1919, 0, 0.8404);
const ChVector<> LMTV_ChassisRear::m_connector_loc(-1.85, 0, 0.45);

const double LMTV_ChassisConnector::m_torsion_stiffness = 7085;

// -----------------------------------------------------------------------------

LMTV_ChassisRear::LMTV_ChassisRear(const std::string& name, CollisionType chassis_collision_type)
    : ChRigidChassisRear(name) {
    m_body_inertia(0, 0) = m_body_inertiaXX.x();
    m_body_inertia(1, 1) = m_body_inertiaXX.y();
    m_body_inertia(2, 2) = m_body_inertiaXX.z();

    m_body_inertia(0, 1) = m_body_inertiaXY.x();
    m_body_inertia(0, 2) = m_body_inertiaXY.y();
    m_body_inertia(1, 2) = m_body_inertiaXY.z();
    m_body_inertia(1, 0) = m_body_inertiaXY.x();
    m_body_inertia(2, 0) = m_body_inertiaXY.y();
    m_body_inertia(2, 1) = m_body_inertiaXY.z();

    //// TODO:
    //// A more appropriate contact shape from primitives
    //// Add collision shapes for rear body

    //    double joint_pos_x = -3.1919;
    //    double joint_pos_z = 0.8404;
    double joint_pos_x = m_connector_loc.x();
    double joint_pos_z = m_connector_loc.z();
    double widthFrame = 0.905;
    double heightFrame = 0.2;

    ChVector<> rearBoxPos((-4.9 + joint_pos_x) / 2, 0, joint_pos_z);
    ChVehicleGeometry::BoxShape box(rearBoxPos, ChQuaternion<>(1, 0, 0, 0),
                                    ChVector<>(joint_pos_x + 4.7, widthFrame, heightFrame));
    ChVehicleGeometry::CylinderShape cyl_torsion(m_connector_loc, Q_from_AngZ(CH_C_PI_2), 0.1, 0.2);

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box);
    m_geometry.m_vis_cylinders.push_back(cyl_torsion);

    m_geometry.m_has_colors = true;
    m_geometry.m_color_boxes = ChColor(0.4f, 0.2f, 0.2f);
    m_geometry.m_color_cylinders = ChColor(0.4f, 0.2f, 0.2f);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "mtv/meshes/m1078_rear.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::HULLS:
            // For now, fall back to using primitive collision shapes
        case CollisionType::PRIMITIVES:
            box.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box);
            break;
        default:
            break;
    }
}

void LMTV_ChassisRear::CreateContactMaterials(ChContactMethod contact_method) {
    // This model uses a single material with default properties.
    MaterialInfo minfo;
    m_geometry.m_materials.push_back(minfo.CreateMaterial(contact_method));
}

// -----------------------------------------------------------------------------

LMTV_ChassisConnector::LMTV_ChassisConnector(const std::string& name) : ChChassisConnectorTorsion(name) {}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
