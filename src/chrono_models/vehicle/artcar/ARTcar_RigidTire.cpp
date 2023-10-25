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
// Authors: Radu Serban, Jayne Henry
// =============================================================================
//
// ARTcar rigid tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/artcar/ARTcar_RigidTire.h"

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

const double ARTcar_RigidTire::m_radius = .170/2;
const double ARTcar_RigidTire::m_width = .0855;

const double ARTcar_RigidTire::m_mass = .200;
const ChVector<> ARTcar_RigidTire::m_inertia(.0008, 0.001, .0008);

const std::string ARTcar_RigidTire::m_meshName = "ARTcar_tire_POV_geom";
//const std::string ARTcar_RigidTire::m_meshFile = "artcar/tire.obj";

const std::string ARTcar_RigidTire::m_meshFile_left = "artcar/tire_left.obj";
const std::string ARTcar_RigidTire::m_meshFile_right = "artcar/tire_right.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

void ARTcar_RigidTire::CreateContactMaterial(ChContactMethod contact_method) {
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.1f;
    minfo.Y = 2e7f;
    m_material = minfo.CreateMaterial(contact_method);
}

ARTcar_RigidTire::ARTcar_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ARTcar_RigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void ARTcar_RigidTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChRigidTire::RemoveVisualizationAssets();
}

}  // end namespace artcar
}  // namespace vehicle
}  // namespace chrono
