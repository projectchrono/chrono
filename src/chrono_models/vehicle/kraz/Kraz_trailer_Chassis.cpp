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
const double Kraz_trailer_Chassis::m_mass = 20000.0;
const ChVector<> Kraz_trailer_Chassis::m_inertiaXX(23904, 322240, 320011);
const ChVector<> Kraz_trailer_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> Kraz_trailer_Chassis::m_COM_loc(-6, 0, 0.8);
const ChVector<> Kraz_trailer_Chassis::m_connector_loc(-0.04, 0, 0.82);

Kraz_trailer_Chassis::Kraz_trailer_Chassis(const std::string& name) : ChRigidChassisRear(name) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    ChVehicleGeometry::BoxShape box(ChVector<>(-5, 0, 1.4 + 0.7), ChQuaternion<>(1, 0, 0, 0),
                                    ChVector<>(13.62, 2.55, 2.8));
    ChVehicleGeometry::SphereShape sphere(m_COM_loc, 0.1);

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box);
    m_geometry.m_vis_spheres.push_back(sphere);

    m_geometry.m_has_colors = true;
    m_geometry.m_color_boxes = ChColor(0.3f, 0.2f, 0.2f);
    m_geometry.m_color_cylinders = ChColor(0.3f, 0.2f, 0.2f);
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
