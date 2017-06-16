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
// M113 chassis subsystem.
//
// =============================================================================

#include <cmath>

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113a/M113a_Chassis.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113a_Chassis::m_mass = 7000.72;
const ChVector<> M113a_Chassis::m_inertiaXX(12519.33, 24431.14, 25790.20);
const ChVector<> M113a_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> M113a_Chassis::m_COM_loc(-1.991, 0, 0.537);
const ChCoordsys<> M113a_Chassis::m_driverCsys(ChVector<>(-1.730, 0.0, 0.508), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_Chassis::M113a_Chassis(const std::string& name, bool fixed, ChassisCollisionType chassis_collision_type)
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

    // Belly shape (all dimensions in cm)
    // width: 170
    // points in x-z transversal plane: (-417.0 -14.3), (4.1, -14.3), (21.4, 34.3)
    // thickness: 20
    double width = 1.70;
    double Ax = -4.17;
    double Az = -0.143;
    double Bx = 0.041;
    double Bz = -0.143;
    double Cx = 0.214;
    double Cz = 0.343;
    double thickness = 0.2;

    ChVector<> dims1((Bx - Ax), width, thickness);
    ChVector<> loc1(0.5 * (Ax + Bx), 0.0, Az + 0.5 * thickness);
    ChQuaternion<> rot1(1, 0, 0, 0);
    BoxShape box1(loc1, rot1, dims1);

    double alpha = std::atan2(Cz - Bz, Cx - Bx);  // pitch angle of front box

    ChVector<> dims2((Cx - Bx) / std::cos(alpha), width, thickness);
    ChVector<> loc2(0.5 * (Bx + Cx) - 0.5 * thickness * std::sin(alpha), 0.0,
        0.5 * (Bz + Cz) + 0.5 * thickness * std::cos(alpha));
    ChQuaternion<> rot2 = Q_from_AngY(-alpha);
    BoxShape box2(loc2, rot2, dims2);

    m_has_primitives = true;
    m_vis_boxes.push_back(box1);
    m_vis_boxes.push_back(box2);

    m_has_mesh = true;
    m_vis_mesh_name = "Chassis_POV_geom";
    m_vis_mesh_file = "M113/Chassis.obj";

    m_has_collision = (chassis_collision_type != ChassisCollisionType::NONE);
    switch (chassis_collision_type) {
    case ChassisCollisionType::PRIMITIVES:
        m_coll_boxes.push_back(box1);
        m_coll_boxes.push_back(box2);
        break;
    case ChassisCollisionType::MESH:
        m_coll_mesh_names.push_back("M113/Chassis_Hulls.obj");
        break;
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
