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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Chassis subsystem for the articulated vehicle.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_tractor_Chassis.h"

#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Kraz_tractor_Chassis::m_body_mass = 10000.0;
const ChVector3d Kraz_tractor_Chassis::m_body_inertiaXX(3441, 28485, 29395);
const ChVector3d Kraz_tractor_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d Kraz_tractor_Chassis::m_body_COM_loc(-2.0, 0, 0.6);
const ChVector3d Kraz_tractor_Chassis::m_connector_loc(-4.64, 0, 0.82);
const ChCoordsys<> Kraz_tractor_Chassis::m_driverCsys(ChVector3d(-1.5, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Kraz_tractor_Chassis::Kraz_tractor_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name) {
    // In this model, we use a single contact material.
    ChContactMaterialData minfo;
    minfo.mu = 1.0f;
    minfo.cr = 0.1f;
    minfo.Y = 5e5f;
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

    m_geometry.vis_mesh_file = vehicle::GetDataFile("longhaul/meshes/SemiTractorBody.obj");

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.matID = 0;
            m_geometry.coll_boxes.push_back(box1);
            break;
        case CollisionType::HULLS: {
            utils::ChBodyGeometry::ConvexHullsShape hull(vehicle::GetDataFile("longhaul/meshes/SemiTractorCab_col.obj"),
                                                         0);
            m_geometry.coll_hulls.push_back(hull);
            break;
        }
        case CollisionType::MESH: {
            utils::ChBodyGeometry::TrimeshShape trimesh(
                ChVector3d(), vehicle::GetDataFile("longhaul/meshes/SemiTractorCab_col.obj"), 0.005, 0);
            m_geometry.coll_meshes.push_back(trimesh);
            break;
        }
        default:
            break;
    }
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
