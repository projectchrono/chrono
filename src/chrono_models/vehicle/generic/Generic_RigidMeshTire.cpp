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
// Generic wheel subsystem (mesh contact)
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_RigidMeshTire.h"

#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Generic_RigidMeshTire::m_radius = 0.4699;
const double Generic_RigidMeshTire::m_width = 0.254;

const double Generic_RigidMeshTire::m_mass = 37.6;
const ChVector<> Generic_RigidMeshTire::m_inertia(3.84, 6.69, 3.84);

const std::string Generic_RigidMeshTire::m_meshFile("generic/tire/generic_tire_coarse.obj");

// -----------------------------------------------------------------------------

Generic_RigidMeshTire::Generic_RigidMeshTire(const std::string& name) : ChRigidTire(name) {
    SetContactFrictionCoefficient(0.9f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(2e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);

    // Contact and visualization meshes
    double sweep_radius = 0.005;
    SetMeshFilename(vehicle::GetDataFile(m_meshFile), sweep_radius);
}

// -----------------------------------------------------------------------------

void Generic_RigidMeshTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(vehicle::GetDataFile(m_meshFile),   // left side
                                               vehicle::GetDataFile(m_meshFile));  // right side
    } else {
        ChRigidTire::AddVisualizationAssets(vis);
    }
}

void Generic_RigidMeshTire::RemoveVisualizationAssets() {
    ChRigidTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
