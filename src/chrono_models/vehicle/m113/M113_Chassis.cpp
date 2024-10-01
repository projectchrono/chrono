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

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113/M113_Chassis.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_Chassis::m_body_mass = 7819.24;
const ChVector3d M113_Chassis::m_body_inertiaXX(13983.08, 27287.61, 28805.57);
const ChVector3d M113_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d M113_Chassis::m_body_COM_loc(-2.006, 0, 0.406);
const ChCoordsys<> M113_Chassis::m_driverCsys(ChVector3d(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Chassis::M113_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
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

    // Belly shape (all dimensions in cm)
    //   width: 170
    //   points in x-z transversal plane: (-417.0 -14.3), (4.1, -14.3), (21.4, 34.3)
    //   thickness: 20
    double width = 1.70;
    double Ax = -4.17;
    double Az = -0.143;
    double Bx = 0.041;
    double Bz = -0.143;
    double Cx = 0.214;
    double Cz = 0.343;
    double thickness = 0.2;

    ChVector3d dims1((Bx - Ax), width, thickness);
    ChVector3d loc1(0.5 * (Ax + Bx), 0.0, Az + 0.5 * thickness);
    ChQuaternion<> rot1(1, 0, 0, 0);
    utils::ChBodyGeometry::BoxShape box1(loc1, rot1, dims1);

    double alpha = std::atan2(Cz - Bz, Cx - Bx);  // pitch angle of front box

    ChVector3d dims2((Cx - Bx) / std::cos(alpha), width, thickness);
    ChVector3d loc2(0.5 * (Bx + Cx) - 0.5 * thickness * std::sin(alpha), 0.0,
                    0.5 * (Bz + Cz) + 0.5 * thickness * std::cos(alpha));
    ChQuaternion<> rot2 = QuatFromAngleY(-alpha);
    utils::ChBodyGeometry::BoxShape box2(loc2, rot2, dims2);

    m_geometry.vis_boxes.push_back(box1);
    m_geometry.vis_boxes.push_back(box2);

    m_geometry.vis_mesh_file = vehicle::GetDataFile("M113/meshes/Chassis.obj");

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.matID = 0;
            box2.matID = 0;
            m_geometry.coll_boxes.push_back(box1);
            m_geometry.coll_boxes.push_back(box2);
            break;
        case CollisionType::HULLS: {
            utils::ChBodyGeometry::ConvexHullsShape hull(vehicle::GetDataFile("M113/meshes/Chassis_Hulls.obj"), 0);
            m_geometry.coll_hulls.push_back(hull);
            break;
        }
        default:
            break;
    }
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
