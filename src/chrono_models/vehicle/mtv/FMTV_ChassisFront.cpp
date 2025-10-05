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

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/mtv/FMTV_ChassisFront.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

// Static variables

const double FMTV_ChassisFront::m_body_mass = 3946;
const ChVector3d FMTV_ChassisFront::m_body_inertiaXX(3.2282e3, 5.2323e3, 4.4980e3);
const ChVector3d FMTV_ChassisFront::m_body_inertiaXY(0, -0.4027e3, 0);
const ChVector3d FMTV_ChassisFront::m_body_COM_loc(-0.6972, 0, 0.6672);
const ChVector3d FMTV_ChassisFront::m_connector_loc(-1.85, 0, 0.45);
const ChCoordsys<> FMTV_ChassisFront::m_driverCsys(ChVector3d(0.4, 0.7, 1.18), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------

FMTV_ChassisFront::FMTV_ChassisFront(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    // In this model, we use a single material with default properties.
    ChContactMaterialData minfo;
    m_geometry.materials.push_back(minfo);

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
    ChVector3d frontBoxPos((1.0 + joint_pos_x) / 2, 0, joint_pos_z);
    utils::ChBodyGeometry::BoxShape box(frontBoxPos, ChQuaternion<>(1, 0, 0, 0),
                                    ChVector3d(1.0 - joint_pos_x, widthFrame, heightFrame));

    m_geometry.vis_boxes.push_back(box);

    m_geometry.vis_model_file = GetVehicleDataFile("mtv/meshes/m1078_front.obj");

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box.matID = 0;
            m_geometry.coll_boxes.push_back(box);
            break;
        case CollisionType::HULLS: {
            utils::ChBodyGeometry::ConvexHullsShape hull(GetVehicleDataFile("mtv/meshes/m1078_front_col.obj"), 0);
            m_geometry.coll_hulls.push_back(hull);
            break;
        }
        case CollisionType::MESH: {
            utils::ChBodyGeometry::TrimeshShape trimesh(
                VNULL, QUNIT, GetVehicleDataFile("mtv/meshes/m1078_front_col.obj"), 1.0, 0.005, 0);
            m_geometry.coll_meshes.push_back(trimesh);
            break;
        }
        default:
            break;
    }
}

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono
