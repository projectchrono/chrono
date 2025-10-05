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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// FEDA chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/feda/FEDA_Chassis.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
/* VIPER config
const double FEDA_Chassis::m_body_mass = 4450;
const ChVector3d FEDA_Chassis::m_body_inertiaXX(2420.0, 8200.0, 7100.0);
const ChVector3d FEDA_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d FEDA_Chassis::m_body_COM_loc(-1.591564, 0.0889, 0.57);
*/
// configuration as tested on proving ground
const double FEDA_Chassis::m_body_mass = 5672.87;
const ChVector3d FEDA_Chassis::m_body_inertiaXX(5.74E+03, 7.66E+03, 9.87E+03);
const ChVector3d FEDA_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d FEDA_Chassis::m_body_COM_loc(-(1.0 - 0.4162) * 3.302, 0.00889, 0.61);

const ChCoordsys<> FEDA_Chassis::m_driverCsys(ChVector3d(-1.35, 0.52, 1.01), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_Chassis::FEDA_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
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
    utils::ChBodyGeometry::BoxShape box1(ChVector3d(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0),
                                         ChVector3d(1.0, 0.5, 0.2));

    m_geometry.vis_boxes.push_back(box1);

    m_geometry.vis_model_file = GetVehicleDataFile("feda/meshes/feda_chassis.obj");

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.matID = 0;
            m_geometry.coll_boxes.push_back(box1);
            break;
        case CollisionType::HULLS: {
            utils::ChBodyGeometry::ConvexHullsShape hull(GetVehicleDataFile("feda/meshes/feda_chassis_col.obj"), 0);
            m_geometry.coll_hulls.push_back(hull);
            break;
        }
        case CollisionType::MESH: {
            utils::ChBodyGeometry::TrimeshShape mesh(
                VNULL, QUNIT, GetVehicleDataFile("feda/meshes/feda_chassis_col.obj"), 1.0, 0.1, 0);
            m_geometry.coll_meshes.push_back(mesh);
            break;
        }
        default:
            break;
    }
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
