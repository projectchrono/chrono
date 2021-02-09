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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// RCCar chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/rccar/RCCar_Chassis.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace rccar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
static const double in2m = 0.0254;
static const double lb2kg = 0.453592;

const double RCCar_Chassis::m_mass = lb2kg * 8;
const ChVector<> RCCar_Chassis::m_inertiaXX(0.0717, 0.00516, 0.00674);
const ChVector<> RCCar_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> RCCar_Chassis::m_COM_loc(in2m * -8.1365, 0.0, in2m * -1.5);
const ChCoordsys<> RCCar_Chassis::m_driverCsys(ChVector<>(0.0, 0.0, 0.0), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RCCar_Chassis::RCCar_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : chrono::vehicle::ChRigidChassis(name, fixed) {
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
    ChVehicleGeometry::BoxShape box1(in2m * ChVector<>(-8.936, 0, 1.0), ChQuaternion<>(1, 0, 0, 0),
                                     in2m * ChVector<>(18.5, 5.0, 1.2));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);

    m_geometry.m_has_mesh = true;
    // m_vis_mesh_name = "RCCar_chassis_POV_geom";
    // m_vis_mesh_file = "rccar/RCCar_chassis.obj";
    m_geometry.m_vis_mesh_file = "hmmwv/HMMWV_chassis.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        // case CollisionType::HULLS:
        //     m_coll_mesh_names.push_back("rccar/RCCar_chassis_simple.obj");
        //     break;
        default:
            break;
    }
}

}  // namespace rccar
}  // namespace vehicle
}  // namespace chrono
