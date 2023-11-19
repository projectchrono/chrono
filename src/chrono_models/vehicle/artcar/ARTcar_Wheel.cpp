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
// Authors: Radu Serban, Justin Madsen, Jayne Henry
// =============================================================================
//
// ARTcar wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/artcar/ARTcar_Wheel.h"

using namespace chrono::vehicle;
using namespace chrono;

namespace chrono {
namespace vehicle {
namespace artcar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254; 
static const double lb2kg = 0.453592;

const double ARTcar_Wheel::m_mass = .07;
const ChVector<> ARTcar_Wheel::m_inertia(.00007, 0.000136, .00007);

const double ARTcar_Wheel::m_radius = .103/2;
const double ARTcar_Wheel::m_width = .0855;

const std::string ARTcar_WheelLeft::m_meshName = "wheel_L_POV_geom";
// const std::string ARTcar_WheelLeft::m_meshFile = "artcar/wheel_L.obj";
const std::string ARTcar_WheelLeft::m_meshFile = "artcar/wheel_L.obj";

const std::string ARTcar_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string ARTcar_WheelRight::m_meshFile = "artcar/wheel_R.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ARTcar_Wheel::ARTcar_Wheel(const std::string& name) : ChWheel(name) {}

ARTcar_WheelLeft::ARTcar_WheelLeft(const std::string& name) : ARTcar_Wheel(name) {}

ARTcar_WheelRight::ARTcar_WheelRight(const std::string& name) : ARTcar_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ARTcar_Wheel::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(GetMeshFile(), false, false);
        m_trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(GetMeshName());
        m_trimesh_shape->SetMutable(false);
        m_spindle->AddVisualShape(m_trimesh_shape);
    } else {
        ChWheel::AddVisualizationAssets(vis);
    }
}

void ARTcar_Wheel::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_spindle, m_trimesh_shape);
    ChWheel::RemoveVisualizationAssets();
}

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono
