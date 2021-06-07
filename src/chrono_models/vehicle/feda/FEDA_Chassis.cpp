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

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/feda/FEDA_Chassis.h"

namespace chrono {
namespace vehicle {
namespace feda {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
/* VIPER config
const double FEDA_Chassis::m_mass = 4450;
const ChVector<> FEDA_Chassis::m_inertiaXX(2420.0, 8200.0, 7100.0);
const ChVector<> FEDA_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> FEDA_Chassis::m_COM_loc(-1.591564, 0.0889, 0.57);
*/
// configuration as tested on proving ground
const double FEDA_Chassis::m_mass = 5672.87;
const ChVector<> FEDA_Chassis::m_inertiaXX(5.74E+03, 7.66E+03, 9.87E+03);
const ChVector<> FEDA_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> FEDA_Chassis::m_COM_loc(-(1.0 - 0.4162) * 3.302, 0.00889, 0.61);

const ChCoordsys<> FEDA_Chassis::m_driverCsys(ChVector<>(-1.35, 0.52, 1.01), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
FEDA_Chassis::FEDA_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
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
    ChVehicleGeometry::BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0),
                                          ChVector<>(1.0, 0.5, 0.2));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "feda/meshes/feda_chassis.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        case CollisionType::MESH: {
            ChVehicleGeometry::TrimeshShape mesh(ChVector<>(0,0,0), "feda/meshes/feda_chassis_coll_mesh.obj", 0.1, 0);
            m_geometry.m_coll_meshes.push_back(mesh);
            break;
        }
        ////case CollisionType::HULLS: {
        ////    ChVehicleGeometry::ConvexHullsShape hull("feda/meshes/feda_chassis_coll_hulls.obj", 0);
        ////    m_geometry.m_coll_hulls.push_back(hull);
        ////    break;
        ////}
        default:
            break;
    }
}

void FEDA_Chassis::CreateContactMaterials(ChContactMethod contact_method) {
    // Create the contact materials.
    // In this model, we use a single material with default properties.
    MaterialInfo minfo;
    m_geometry.m_materials.push_back(minfo.CreateMaterial(contact_method));
}

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono
