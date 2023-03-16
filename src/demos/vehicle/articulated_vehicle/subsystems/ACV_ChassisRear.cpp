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
// Rear chassis subsystem for the articulated vehicle.
//
// =============================================================================

#include "subsystems/ACV_ChassisRear.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double ACV_ChassisRear::m_body_mass = 2000.0;
const ChVector<> ACV_ChassisRear::m_body_inertiaXX(100, 400, 500);
const ChVector<> ACV_ChassisRear::m_body_inertiaXY(0, 0, 0);
const ChVector<> ACV_ChassisRear::m_body_COM_loc(0, 0, 0.4);
const ChVector<> ACV_ChassisRear::m_connector_loc(1.0, 0, 0.1);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ACV_ChassisRear::ACV_ChassisRear(const std::string& name) : ChRigidChassisRear(name) {
    m_body_inertia(0, 0) = m_body_inertiaXX.x();
    m_body_inertia(1, 1) = m_body_inertiaXX.y();
    m_body_inertia(2, 2) = m_body_inertiaXX.z();

    m_body_inertia(0, 1) = m_body_inertiaXY.x();
    m_body_inertia(0, 2) = m_body_inertiaXY.y();
    m_body_inertia(1, 2) = m_body_inertiaXY.z();
    m_body_inertia(1, 0) = m_body_inertiaXY.x();
    m_body_inertia(2, 0) = m_body_inertiaXY.y();
    m_body_inertia(2, 1) = m_body_inertiaXY.z();

    ChVehicleGeometry::BoxShape box(ChVector<>(0.1, 0, 0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(1.2, 1.0, 0.2));
    ChVehicleGeometry::CylinderShape cyl_axle(ChVector<>(-0.5, 0, 0), ChVector<>(0, 1, 0), 0.05, 2);
    ChVehicleGeometry::CylinderShape cyl_articulation(m_connector_loc, ChVector<>(0, 0, 1), 0.1, 0.22);

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box);
    m_geometry.m_vis_cylinders.push_back(cyl_axle);
    m_geometry.m_vis_cylinders.push_back(cyl_articulation);

    m_geometry.m_has_colors = true;
    m_geometry.m_color_boxes = ChColor(0.4f, 0.2f, 0.2f);
    m_geometry.m_color_cylinders = ChColor(0.4f, 0.2f, 0.2f);
}
