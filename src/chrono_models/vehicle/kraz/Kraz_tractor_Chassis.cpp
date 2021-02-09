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
// Chassis subsystem for the articulated vehicle.
//
// =============================================================================

#include "chrono_models/vehicle/kraz/Kraz_tractor_Chassis.h"

namespace chrono {
namespace vehicle {
namespace kraz {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Kraz_tractor_Chassis::m_mass = 10000.0;
const ChVector<> Kraz_tractor_Chassis::m_inertiaXX(3441, 28485, 29395);
const ChVector<> Kraz_tractor_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> Kraz_tractor_Chassis::m_COM_loc(-2.0, 0, 0.6);
const ChVector<> Kraz_tractor_Chassis::m_connector_loc(-4.64, 0, 0.82);
const ChCoordsys<> Kraz_tractor_Chassis::m_driverCsys(ChVector<>(-1.5, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Kraz_tractor_Chassis::Kraz_tractor_Chassis(const std::string& name) : ChRigidChassis(name) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();

    m_geometry.m_has_mesh = true;
    m_geometry.m_vis_mesh_file = "longhaul/meshes/SemiTractorBody.obj";
}

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono
