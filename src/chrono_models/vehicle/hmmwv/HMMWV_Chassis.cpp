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
// HMMWV chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Chassis.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV_Chassis::m_mass = 2086.52;
const ChVector<> HMMWV_Chassis::m_inertiaXX(1078.52, 2955.66, 3570.20);
const ChVector<> HMMWV_Chassis::m_inertiaXY(0, 0, 0);
const ChVector<> HMMWV_Chassis::m_COM_loc(0.056, 0, 0.523);
const ChCoordsys<> HMMWV_Chassis::m_driverCsys(ChVector<>(0.87, -0.27, 1.05), ChQuaternion<>(1, 0, 0, 0));

const std::string HMMWV_Chassis::m_meshName = "hmmwv_chassis_POV_geom";
const std::string HMMWV_Chassis::m_meshFile = "hmmwv/hmmwv_chassis.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Chassis::HMMWV_Chassis(const std::string& name, bool fixed) : ChChassis(name, fixed) {
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
void HMMWV_Chassis::AddVisualizationAssets(VisualizationType vis) {
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

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
