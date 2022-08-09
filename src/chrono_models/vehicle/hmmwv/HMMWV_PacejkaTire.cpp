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
// HMMWV Pacejka 2002 tire subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_PacejkaTire.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_PacejkaTire::m_mass = 37.6;
const ChVector<> HMMWV_PacejkaTire::m_inertia(3.84, 6.69, 3.84);

const std::string HMMWV_PacejkaTire::m_pacTireFile = "hmmwv/tire/HMMWV_pacejka.tir";

const std::string HMMWV_PacejkaTire::m_meshFile_left = "hmmwv/hmmwv_tire_left.obj";
const std::string HMMWV_PacejkaTire::m_meshFile_right = "hmmwv/hmmwv_tire_right.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_PacejkaTire::HMMWV_PacejkaTire(const std::string& name)
    : ChPacejkaTire(name, vehicle::GetDataFile(m_pacTireFile)) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_PacejkaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChPacejkaTire::AddVisualizationAssets(vis);
    }
}

void HMMWV_PacejkaTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChPacejkaTire::RemoveVisualizationAssets();
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
