// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// M113 chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_Chassis.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_Chassis::m_mass = 5489.24;
const ChVector<> M113_Chassis::m_inertiaXX(1786.92, 10449.67, 10721.22);
const ChVector<> M113_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> M113_Chassis::m_COM_loc(-2.006, 0, 0.406);
const ChCoordsys<> M113_Chassis::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Chassis::M113_Chassis(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    m_inertia.SetElement(0, 0, m_inertiaXX.x());
    m_inertia.SetElement(1, 1, m_inertiaXX.y());
    m_inertia.SetElement(2, 2, m_inertiaXX.z());

    m_inertia.SetElement(0, 1, m_inertiaXY.x());
    m_inertia.SetElement(0, 2, m_inertiaXY.y());
    m_inertia.SetElement(1, 2, m_inertiaXY.z());
    m_inertia.SetElement(1, 0, m_inertiaXY.x());
    m_inertia.SetElement(2, 0, m_inertiaXY.y());
    m_inertia.SetElement(2, 1, m_inertiaXY.z());

    //// TODO:
    //// A more appropriate contact shape from primitives
    BoxShape box1(ChVector<>(-2.0, 0.0, -0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(4.0, 1.0, 0.3));

    m_has_primitives = true;
    m_vis_boxes.push_back(box1);

    m_has_mesh = true;
    m_vis_mesh_name = "Chassis_POV_geom";
    m_vis_mesh_file = "M113/Chassis.obj";

    m_has_collision = (chassis_collision_type != ChassisCollisionType::NONE);
    switch (chassis_collision_type) {
        case ChassisCollisionType::PRIMITIVES:
            m_coll_boxes.push_back(box1);
            break;
        case ChassisCollisionType::MESH:
            m_coll_mesh_names.push_back("M113/Chassis_Hulls.obj");
            break;
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
