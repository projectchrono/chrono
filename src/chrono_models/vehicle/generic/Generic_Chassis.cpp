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
// Chassis subsystem for the generic vehicle.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/generic/Generic_Chassis.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double Generic_Chassis::m_mass = 1399.13;
const ChVector<> Generic_Chassis::m_inertiaXX(222.8, 944.1, 1053.5);
const ChVector<> Generic_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> Generic_Chassis::m_COM_loc(1.5-1.763, 0, 0.440 - 0.450);
const ChCoordsys<> Generic_Chassis::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_Chassis::Generic_Chassis(const std::string& name, bool fixed) : ChRigidChassis(name, fixed) {
    m_inertia(0, 0) = m_inertiaXX.x();
    m_inertia(1, 1) = m_inertiaXX.y();
    m_inertia(2, 2) = m_inertiaXX.z();

    m_inertia(0, 1) = m_inertiaXY.x();
    m_inertia(0, 2) = m_inertiaXY.y();
    m_inertia(1, 2) = m_inertiaXY.z();
    m_inertia(1, 0) = m_inertiaXY.x();
    m_inertia(2, 0) = m_inertiaXY.y();
    m_inertia(2, 1) = m_inertiaXY.z();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_Chassis::AddVisualizationAssets(VisualizationType vis) {
    ChChassis::AddVisualizationAssets(vis);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
