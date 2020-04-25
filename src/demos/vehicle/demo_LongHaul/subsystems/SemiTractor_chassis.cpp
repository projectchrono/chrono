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
// Chassis subsystem for the articulated vehicle.
//
// =============================================================================

#include "subsystems/SemiTractor_chassis.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double SemiTractor_chassis::m_mass = 10000.0;
const ChVector<> SemiTractor_chassis::m_inertiaXX(3441, 28485, 29395);
const ChVector<> SemiTractor_chassis::m_inertiaXY(0, 0, 0);
const ChVector<> SemiTractor_chassis::m_COM_loc(-2.0, 0, 0.6);
const ChCoordsys<> SemiTractor_chassis::m_driverCsys(ChVector<>(-1.5, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

const ChVector<> SemiTractor_chassis::m_5th_wheel_loc(-4.64, 0, 0.82);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SemiTractor_chassis::SemiTractor_chassis(const std::string& name) : ChRigidChassis(name) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    m_has_mesh = true;
    m_vis_mesh_file = "longhaul/meshes/SemiTractorBody.obj";
}
