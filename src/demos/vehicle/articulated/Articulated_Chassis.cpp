// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Chassis subsystem for the articulated vehicle.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "articulated/Articulated_Chassis.h"

using namespace chrono;
using namespace chrono::vehicle;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Articulated_Chassis::m_mass = 3500;
const ChVector<> Articulated_Chassis::m_inertia(125.8, 497.4, 531.4);
const ChVector<> Articulated_Chassis::m_COM_loc(-0.2, 0, 0.8);
const ChCoordsys<> Articulated_Chassis::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Articulated_Chassis::Articulated_Chassis(const std::string& name) : ChChassis(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Articulated_Chassis::AddVisualizationAssets(VisualizationType vis) {
    ChChassis::AddVisualizationAssets(vis);
}
