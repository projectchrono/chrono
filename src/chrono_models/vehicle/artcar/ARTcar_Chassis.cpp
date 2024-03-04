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
// ARTcar chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/artcar/ARTcar_Chassis.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace chrono {
namespace vehicle {
namespace artcar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
static const double in2m = 0.0254;
static const double lb2kg = 0.453592;

const double ARTcar_Chassis::m_body_mass = 4.96866766;
const ChVector<> ARTcar_Chassis::m_body_inertiaXX(0.05389410, 0.24738708, 0.28435979);
const ChVector<> ARTcar_Chassis::m_body_inertiaXY(-0.00027434, -0.00228453, -0.00294115);
const ChVector<> ARTcar_Chassis::m_body_COM_loc(-.1336, -.0014, -.048);
const ChCoordsys<> ARTcar_Chassis::m_driverCsys(ChVector<>(0.0, 0.0, 0.0), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ARTcar_Chassis::ARTcar_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : chrono::vehicle::ChRigidChassis(name, fixed) {
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
    ChVehicleGeometry::BoxShape box1(ChVector<>(-.13, 0, -.0368), ChQuaternion<>(1, 0, 0, 0),
                                     ChVector<>(.38, .15, .01));

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);

    m_geometry.m_has_mesh = false; 

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            m_geometry.m_coll_boxes.push_back(box1);
            break;
        default:
            break;
    }
}

}  // namespace artcar
}  // namespace vehicle
}  // namespace chrono
