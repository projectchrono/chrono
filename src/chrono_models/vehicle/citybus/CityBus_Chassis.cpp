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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl
// =============================================================================
//
// CityBus chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/citybus/CityBus_Chassis.h"

namespace chrono {
namespace vehicle {
namespace citybus {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double CityBus_Chassis::m_body_mass = 13000;
const ChVector<> CityBus_Chassis::m_body_inertiaXX(13.5e3, 13.5e3, 115.1e3);
const ChVector<> CityBus_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector<> CityBus_Chassis::m_body_COM_loc(-4, 0, 0.86);
const ChCoordsys<> CityBus_Chassis::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
CityBus_Chassis::CityBus_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
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
    ChVehicleGeometry::BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(1.0, 0.5, 0.2));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "citybus/CityBus_Vis.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        case CollisionType::HULLS: {
            ChVehicleGeometry::ConvexHullsShape hull("citybus/CityBus_Col.obj", 0);
            m_geometry.m_coll_hulls.push_back(hull);
            break;
        }
        default:
            break;
    }
}

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono
