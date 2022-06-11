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

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/uaz/UAZBUS_Chassis.h"

namespace chrono {
namespace vehicle {
namespace uaz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double UAZBUS_Chassis::m_body_mass = 2321.0;
const ChVector<> UAZBUS_Chassis::m_body_inertiaXX(785.0, 2612.0, 2761.0);
const ChVector<> UAZBUS_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector<> UAZBUS_Chassis::m_body_COM_loc(-1.204, 0.0, 0.3);
const ChVector<> UAZBUS_Chassis::m_connector_rear_loc(-3.5, 0, -0.05);
const ChCoordsys<> UAZBUS_Chassis::m_driverCsys(ChVector<>(0.0, 0.7, 0.5), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
UAZBUS_Chassis::UAZBUS_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
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
    ChVehicleGeometry::BoxShape box1(ChVector<>(-1.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(1.6, 1.0, 0.2));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);

    m_geometry.m_has_obj = true;
    m_geometry.m_vis_mesh_file = "uaz/uazbus_chassis.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        case CollisionType::HULLS: {
            ChVehicleGeometry::ConvexHullsShape hull("uaz/uazbus_chassis_simple.obj", 0);
            m_geometry.m_coll_hulls.push_back(hull);
            break;
        }
        default:
            break;
    }
}

void UAZBUS_Chassis::CreateContactMaterials(ChContactMethod contact_method) {
    // Create the contact materials.
    // In this model, we use a single material with default properties.
    MaterialInfo minfo;
    m_geometry.m_materials.push_back(minfo.CreateMaterial(contact_method));
}

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono
