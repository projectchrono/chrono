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
// BMW E90 (330i of 2006) chassis subsystem.
// masses / CoG / inertia in test configuration incl. driver and instrumentation
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/bmw/BMW_E90_Chassis.h"

namespace chrono {
namespace vehicle {
namespace bmw {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double BMW_E90_Chassis::m_body_mass = 1442.1;
const ChVector3d BMW_E90_Chassis::m_body_inertiaXX(572, 2487, 2768);
const ChVector3d BMW_E90_Chassis::m_body_inertiaXY(0, 0, 42);
const ChVector3d BMW_E90_Chassis::m_body_COM_loc(-1.371, -0.001, 0.206232);
const ChVector3d BMW_E90_Chassis::m_connector_rear_loc(-2.974, 0, 0.0);
const ChCoordsys<> BMW_E90_Chassis::m_driverCsys(ChVector3d(-1.1, 0.7, 0.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BMW_E90_Chassis::BMW_E90_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
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
    utils::ChBodyGeometry::BoxShape box1(ChVector3d(-1, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector3d(1, 1.0, 0.2));

    m_geometry.vis_boxes.push_back(box1);

    m_geometry.vis_model_file = GetVehicleDataFile("bmw/chassis/bmw_e90_chassis.obj");

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.matID = 0;
            m_geometry.coll_boxes.push_back(box1);
            break;
        case CollisionType::HULLS: {
            utils::ChBodyGeometry::ConvexHullsShape hull(GetVehicleDataFile("bmw/chassis/bmw_e90_chassis_col.obj"),
                                                         0);
            m_geometry.coll_hulls.push_back(hull);
            break;
        }
        case CollisionType::MESH: {
            utils::ChBodyGeometry::TrimeshShape trimesh(
                VNULL, QUNIT, GetVehicleDataFile("bmw/chassis/bmw_e90_chassis_col.obj"), 1.0, 0.005, 0);
            m_geometry.coll_meshes.push_back(trimesh);
            break;
        }
        default:
            break;
    }
}

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono
