// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor, Rainer Gericke
// =============================================================================
//
// Pac89 tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/MFTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
MFTire::MFTire(const std::string& filename)
    : ChMFTire(""), m_mass(0), m_has_mesh(false), m_has_vert_table(false), m_has_bott_table(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

MFTire::MFTire(const rapidjson::Document& d)
    : ChMFTire(""), m_mass(0), m_has_mesh(false), m_has_vert_table(false), m_has_bott_table(false) {
    Create(d);
}

void MFTire::Create(const rapidjson::Document& d) {  // Invoke base class method.
    ChPart::Create(d);

}

void MFTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChMFTire::AddVisualizationAssets(vis);
    }
}

void MFTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChMFTire::RemoveVisualizationAssets();
}

}  // namespace vehicle
}  // namespace chrono

