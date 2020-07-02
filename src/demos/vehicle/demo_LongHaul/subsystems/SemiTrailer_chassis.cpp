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
// Trailer chassis and connector for the tractor-trailer vehicle model.
//
// =============================================================================

#include "subsystems/SemiTrailer_chassis.h"

using namespace chrono;
using namespace chrono::vehicle;

// Static variables
const double SemiTrailer_chassis::m_mass = 20000.0;
const ChVector<> SemiTrailer_chassis::m_inertiaXX(23904, 322240, 320011);
const ChVector<> SemiTrailer_chassis::m_inertiaXY(0, 0, 0);
const ChVector<> SemiTrailer_chassis::m_COM_loc(-6, 0, 0.8);
const ChVector<> SemiTrailer_chassis::m_connector_loc(-0.04, 0, 0.8);

SemiTrailer_chassis::SemiTrailer_chassis(const std::string& name) : ChRigidChassisRear(name) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    ChRigidChassisGeometry::BoxShape box(ChVector<>(-5, 0, 1.4 + 0.7), ChQuaternion<>(1, 0, 0, 0),
                                         ChVector<>(13.62, 2.55, 2.8));
    ChRigidChassisGeometry::SphereShape sphere(m_COM_loc, 0.1);

    m_geometry.m_has_primitives = true;
    m_geometry.m_vis_boxes.push_back(box);
    m_geometry.m_vis_spheres.push_back(sphere);
    m_geometry.m_color = ChColor(0.3f, 0.2f, 0.2f);
}
