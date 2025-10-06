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
// Authors: Radu Serban
// =============================================================================
//
// UAZBUS chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/uaz/UAZBUS_Chassis.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double UAZBUS_Chassis::m_body_mass = 2321.0;
const ChVector3d UAZBUS_Chassis::m_body_inertiaXX(785.0, 2612.0, 2761.0);
const ChVector3d UAZBUS_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d UAZBUS_Chassis::m_body_COM_loc(-1.204, 0.0, 0.3);
const ChVector3d UAZBUS_Chassis::m_connector_rear_loc(-3.5, 0, -0.05);
const ChCoordsys<> UAZBUS_Chassis::m_driverCsys(ChVector3d(0.0, 0.7, 0.5), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_Chassis::UAZBUS_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
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
    utils::ChBodyGeometry::BoxShape box1(ChVector3d(-1.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0),
                                         ChVector3d(1.6, 1.0, 0.2));

    m_geometry.vis_boxes.push_back(box1);

    m_geometry.vis_model_file = GetVehicleDataFile("uaz/uazbus_chassis.obj");

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.matID = 0;
            m_geometry.coll_boxes.push_back(box1);
            break;
        case CollisionType::HULLS: {
            utils::ChBodyGeometry::ConvexHullsShape hull(GetVehicleDataFile("uaz/uazbus_chassis_col.obj"), 0);
            m_geometry.coll_hulls.push_back(hull);
            break;
        }
        case CollisionType::MESH: {
            utils::ChBodyGeometry::TrimeshShape trimesh(
                VNULL, QUNIT, GetVehicleDataFile("uaz/uazbus_chassis_col.obj"), 1.0, 0.005, 0);
            m_geometry.coll_meshes.push_back(trimesh);
            break;
        }
        default:
            break;
    }
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
