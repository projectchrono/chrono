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

const std::string M113a_Chassis::m_meshName = "Chassis_POV_geom";
const std::string M113a_Chassis::m_meshFile = "M113/Chassis.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113a_Chassis::M113a_Chassis(const std::string& name, bool fixed) : ChChassis(name, fixed) {
    m_inertia.SetElement(0, 0, m_inertiaXX.x());
    m_inertia.SetElement(1, 1, m_inertiaXX.y());
    m_inertia.SetElement(2, 2, m_inertiaXX.z());

    m_inertia.SetElement(0, 1, m_inertiaXY.x());
    m_inertia.SetElement(0, 2, m_inertiaXY.y());
    m_inertia.SetElement(1, 2, m_inertiaXY.z());
    m_inertia.SetElement(1, 0, m_inertiaXY.x());
    m_inertia.SetElement(2, 0, m_inertiaXY.y());
    m_inertia.SetElement(2, 1, m_inertiaXY.z());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113a_Chassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        m_body->AddAsset(trimesh_shape);
    } else {
        ChChassis::AddVisualizationAssets(vis);
    }
}


}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono
