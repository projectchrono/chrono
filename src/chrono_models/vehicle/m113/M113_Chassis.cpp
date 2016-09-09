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

#include "chrono_models/vehicle/m113/M113_Chassis.h"

namespace chrono {
namespace vehicle {
namespace m113 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double M113_Chassis::m_mass = 5489.24;
const ChVector<> M113_Chassis::m_inertia(1786.92, 10449.67, 10721.22);
const ChVector<> M113_Chassis::m_COM_loc(-2.006, 0, 0.406);
const ChCoordsys<> M113_Chassis::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

const std::string M113_Chassis::m_meshName = "Chassis_POV_geom";
const std::string M113_Chassis::m_meshFile = "M113/Chassis.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
M113_Chassis::M113_Chassis(const std::string& name, bool fixed) : ChChassis(name, fixed) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void M113_Chassis::AddVisualizationAssets(VisualizationType vis) {
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
