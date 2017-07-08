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
// HMMWV chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Chassis.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_Chassis::m_mass = 2086.52;
const ChVector<> HMMWV_Chassis::m_inertiaXX(1078.52, 2955.66, 3570.20);
const ChVector<> HMMWV_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> HMMWV_Chassis::m_COM_loc(0.056, 0, 0.523);
const ChCoordsys<> HMMWV_Chassis::m_driverCsys(ChVector<>(0.87, -0.27, 1.05), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Chassis::HMMWV_Chassis(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type)
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
    BoxShape box1(ChVector<>(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(2.0, 1.0, 0.2));

    m_has_primitives = true;
    m_vis_boxes.push_back(box1);

    m_has_mesh = true;
    m_vis_mesh_name = "hmmwv_chassis_POV_geom";
    m_vis_mesh_file = "hmmwv/hmmwv_chassis.obj";

    m_has_collision = (chassis_collision_type != ChassisCollisionType::NONE);
    switch (chassis_collision_type) {
        case ChassisCollisionType::PRIMITIVES:
            m_coll_boxes.push_back(box1);
            break;
        case ChassisCollisionType::MESH:
            m_coll_mesh_names.push_back("hmmwv/hmmwv_chassis_simple.obj");
            break;
    }
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
