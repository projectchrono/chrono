// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Duro chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/duro/Duro_Chassis.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Duro_Chassis::m_body_mass = 4900;
const ChVector3d Duro_Chassis::m_body_inertiaXX(2629, 13484, 13502);
const ChVector3d Duro_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d Duro_Chassis::m_body_COM_loc(-2.2529, 0, 0.6586);
const ChVector3d Duro_Chassis::m_connector_rear_loc(-3.88 - 1.37, 0, -0.3);
const ChCoordsys<> Duro_Chassis::m_driverCsys(ChVector3d(-0.52, 0.7, 1.4), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------

Duro_Chassis::Duro_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    // In this model, we use a single contact material.
    ChContactMaterialData minfo;
    minfo.mu = 1.0f;
    minfo.cr = 0.5f;
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
    utils::ChBodyGeometry::BoxShape box1(ChVector3d(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0),
                                         ChVector3d(2.0, 1.0, 0.2));
    utils::ChBodyGeometry::BoxShape box2(ChVector3d(0.0, 0.0, 0.3), ChQuaternion<>(1, 0, 0, 0),
                                         ChVector3d(1.0, 0.5, 0.2));

    m_geometry.vis_boxes.push_back(box1);
    m_geometry.vis_boxes.push_back(box2);

    ////m_geometry.vis_model_file = GetVehicleDataFile("Duro/Duro_chassis.obj");

    switch (chassis_collision_type) {
        default:
        case CollisionType::PRIMITIVES:
            box1.matID = 0;
            m_geometry.coll_boxes.push_back(box1);
            break;
        ////case CollisionType::HULLS: {
        ////    utils::ChBodyGeometry::ConvexHullsShape hull(GetVehicleDataFile("Duro/Duro_chassis_col.obj"), 0);
        ////    m_geometry.coll_hulls.push_back(hull);
        ////    break;
        ////}
        ////case CollisionType::MESH: {
        ////    utils::ChBodyGeometry::TrimeshShape trimesh(ChVector3d(), GetVehicleDataFile("Duro/Duro_chassis_col.obj"),
        ////                                                0.005, 0);
        ////    m_geometry.coll_meshes.push_back(trimesh);
        ////    break;
        ////}
    }
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
