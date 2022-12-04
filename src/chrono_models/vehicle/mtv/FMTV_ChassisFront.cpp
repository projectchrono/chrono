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
// FMTV front chassis subsystem (common for MTV and LMTV trucks)
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/mtv/FMTV_ChassisFront.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// Static variables

const double FMTV_ChassisFront::m_body_mass = 3946;
const ChVector<> FMTV_ChassisFront::m_body_inertiaXX(3.2282e3, 5.2323e3, 4.4980e3);
const ChVector<> FMTV_ChassisFront::m_body_inertiaXY(0, -0.4027e3, 0);
const ChVector<> FMTV_ChassisFront::m_body_COM_loc(-0.6972, 0, 0.6672);
const ChVector<> FMTV_ChassisFront::m_connector_loc(-1.85, 0, 0.45);
const ChCoordsys<> FMTV_ChassisFront::m_driverCsys(ChVector<>(0.4, 0.7, 1.18), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------

FMTV_ChassisFront::FMTV_ChassisFront(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    // In this model, we use a single material with default properties.
    ChContactMaterialData minfo;
    m_geometry.m_materials.push_back(minfo);

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

    double joint_pos_x = -3.1919;
    double joint_pos_z = 0.8404;
    double widthFrame = 0.905;
    double heightFrame = 0.2;
    ChVector<> frontBoxPos((1.0 + joint_pos_x) / 2, 0, joint_pos_z);
    ChVehicleGeometry::BoxShape box(frontBoxPos, ChQuaternion<>(1, 0, 0, 0),
                                    ChVector<>(1.0 - joint_pos_x, widthFrame, heightFrame));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "mtv/meshes/m1078_front.obj";

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

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
