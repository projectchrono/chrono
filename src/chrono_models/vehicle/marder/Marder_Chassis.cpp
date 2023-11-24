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
// Authors: Rainer Gericke
// =============================================================================
//
// Marder chassis subsystem.
//
// =============================================================================

#include <cmath>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/marder/Marder_Chassis.h"

namespace chrono {
namespace vehicle {
namespace marder {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Marder_Chassis::m_body_mass = 25000.00;
const ChVector<> Marder_Chassis::m_body_inertiaXX(13653.38542, 98182.70833, 101954.4271);
const ChVector<> Marder_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector<> Marder_Chassis::m_body_COM_loc(-2.7958, 0, 0.0);
const ChCoordsys<> Marder_Chassis::m_driverCsys(ChVector<>(0.0, 0.5, 0.0), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Marder_Chassis::Marder_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
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

    // Belly shape (all dimensions in cm)
    //   width: 170
    //   points in x-z transversal plane: (-417.0 -14.3), (4.1, -14.3), (21.4, 34.3)
    //   thickness: 20
    double width = 2.045;
    double Ax = -5.45;
    double Az = -0.4;
    double Bx = 0.041;
    double Bz = -0.243;
    double Cx = 0.214;
    double Cz = 0.343;
    double thickness = 0.2;

    ChVector<> dims1((Bx - Ax), width, thickness);
    ChVector<> loc1(0.5 * (Ax + Bx), 0.0, Az + 0.5 * thickness);
    ChQuaternion<> rot1(1, 0, 0, 0);
    ChVehicleGeometry::BoxShape box1(loc1, rot1, dims1);

    double alpha = std::atan2(Cz - Bz, Cx - Bx);  // pitch angle of front box

    ChVector<> dims2((Cx - Bx) / std::cos(alpha), width, thickness);
    ChVector<> loc2(0.5 * (Bx + Cx) - 0.5 * thickness * std::sin(alpha), 0.0,
                    0.5 * (Bz + Cz) + 0.5 * thickness * std::cos(alpha));
    ChQuaternion<> rot2 = Q_from_AngY(-alpha);
    ChVehicleGeometry::BoxShape box2(loc2, rot2, dims2);

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box1);
    m_geometry.m_vis_boxes.push_back(box2);

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "Marder/Hull.obj";

    m_geometry.m_has_collision = (chassis_collision_type != CollisionType::NONE);
    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.m_matID = 0;
            box2.m_matID = 0;
            m_geometry.m_coll_boxes.push_back(box1);
            m_geometry.m_coll_boxes.push_back(box2);
            break;
        case CollisionType::HULLS: {
            ChVehicleGeometry::ConvexHullsShape hull("M113/Chassis_Hulls.obj", 0);
            m_geometry.m_coll_hulls.push_back(hull);
            break;
        }
        default:
            break;
    }
}

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono
