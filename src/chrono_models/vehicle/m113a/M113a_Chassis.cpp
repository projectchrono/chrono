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
// M113 chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/m113a/M113a_Chassis.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113a_Chassis::m_mass = 7000.72;
const ChVector<> M113a_Chassis::m_inertiaXX(12519.33, 24431.14, 25790.20);
const ChVector<> M113a_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> M113a_Chassis::m_COM_loc(-1.991, 0, 0.537);
const ChCoordsys<> M113a_Chassis::m_driverCsys(ChVector<>(-1.730, 0.0, 0.508), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_Chassis::M113a_Chassis(const std::string& name, bool fixed) : ChRigidChassis(name, fixed) {
    m_inertia.SetElement(0, 0, m_inertiaXX.x());
    m_inertia.SetElement(1, 1, m_inertiaXX.y());
    m_inertia.SetElement(2, 2, m_inertiaXX.z());

    m_inertia.SetElement(0, 1, m_inertiaXY.x());
    m_inertia.SetElement(0, 2, m_inertiaXY.y());
    m_inertia.SetElement(1, 2, m_inertiaXY.z());
    m_inertia.SetElement(1, 0, m_inertiaXY.x());
    m_inertia.SetElement(2, 0, m_inertiaXY.y());
    m_inertia.SetElement(2, 1, m_inertiaXY.z());

    m_has_mesh = true;
    m_vis_mesh_name = "Chassis_POV_geom";
    m_vis_mesh_file = "M113/Chassis.obj";
}

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
