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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Trailer chassis and connector for the Kraz trailer vehicle model.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_trailer_Chassis.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// Static variables
const double Kraz_trailer_Chassis::m_body_mass = 20000.0;
const ChVector3d Kraz_trailer_Chassis::m_body_inertiaXX(23904, 322240, 320011);
const ChVector3d Kraz_trailer_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d Kraz_trailer_Chassis::m_body_COM_loc(-6, 0, 0.8);
const ChVector3d Kraz_trailer_Chassis::m_connector_loc(-0.04, 0, 0.82);

Kraz_trailer_Chassis::Kraz_trailer_Chassis(const std::string& name, CollisionType chassis_collision_type)
    : ChRigidChassisRear(name) {
    // In this model, we use a single contact material.
    ChContactMaterialData minfo;
    minfo.mu = 1.0f;
    minfo.cr = 0.1f;
    minfo.Y = 5e5f;
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

    utils::ChBodyGeometry::BoxShape box(ChVector3d(-5, 0, 1.4 + 0.7), ChQuaternion<>(1, 0, 0, 0),
                                    ChVector3d(13.62, 2.55, 2.8));
    utils::ChBodyGeometry::SphereShape sphere(m_body_COM_loc, 0.1);

    m_geometry.vis_boxes.push_back(box);
    m_geometry.vis_spheres.push_back(sphere);

    m_geometry.color_boxes = ChColor(0.3f, 0.2f, 0.2f);
    m_geometry.color_cylinders = ChColor(0.3f, 0.2f, 0.2f);

    // Collision shape is always a box
    box.matID = 0;
    m_geometry.coll_boxes.push_back(box);
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
