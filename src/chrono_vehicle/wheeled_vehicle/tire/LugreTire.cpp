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
// LuGre tire constructed with data from file (JSON format).
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/wheeled_vehicle/tire/LugreTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
LugreTire::LugreTire(const std::string& filename) : ChLugreTire(""), m_discLocs(NULL), m_has_mesh(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    Create(d);

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

LugreTire::LugreTire(const rapidjson::Document& d) : ChLugreTire(""), m_discLocs(NULL), m_has_mesh(false) {
    Create(d);
}

LugreTire::~LugreTire() {
    delete m_discLocs;
}

void LugreTire::Create(const rapidjson::Document& d) {
    // Invoke base class method.
    ChPart::Create(d);

    // Read tire radius, mass, and inertia
    m_radius = d["Radius"].GetDouble();
    m_mass = d["Mass"].GetDouble();
    m_inertia = ReadVectorJSON(d["Inertia"]);

    // Read disc locations
    m_numDiscs = d["Disc Locations"].Size();
    m_discLocs = new double[m_numDiscs];
    for (int i = 0; i < m_numDiscs; i++) {
        m_discLocs[i] = d["Disc Locations"][SizeType(i)].GetDouble();
    }

    // Read normal stiffness and damping
    m_normalStiffness = d["Normal Stiffness"].GetDouble();
    m_normalDamping = d["Normal Damping"].GetDouble();

    // Read LuGre model parameters
    m_sigma0[0] = d["Lugre Parameters"]["sigma0"][0u].GetDouble();  // longitudinal
    m_sigma0[1] = d["Lugre Parameters"]["sigma0"][1u].GetDouble();  // lateral

    m_sigma1[0] = d["Lugre Parameters"]["sigma1"][0u].GetDouble();  // longitudinal
    m_sigma1[1] = d["Lugre Parameters"]["sigma1"][1u].GetDouble();  // lateral

    m_sigma2[0] = d["Lugre Parameters"]["sigma2"][0u].GetDouble();  // longitudinal
    m_sigma2[1] = d["Lugre Parameters"]["sigma2"][1u].GetDouble();  // lateral

    m_Fc[0] = d["Lugre Parameters"]["Fc"][0u].GetDouble();  // longitudinal
    m_Fc[1] = d["Lugre Parameters"]["Fc"][1u].GetDouble();  // lateral

    m_Fs[0] = d["Lugre Parameters"]["Fs"][0u].GetDouble();  // longitudinal
    m_Fs[1] = d["Lugre Parameters"]["Fs"][1u].GetDouble();  // lateral

    m_vs[0] = d["Lugre Parameters"]["vs"][0u].GetDouble();  // longitudinal
    m_vs[1] = d["Lugre Parameters"]["vs"][1u].GetDouble();  // lateral

    // Check how to visualize this tire.
    if (d.HasMember("Visualization")) {
        if (d["Visualization"].HasMember("Mesh Filename Left") && d["Visualization"].HasMember("Mesh Filename Right")) {
            m_meshFile_left = d["Visualization"]["Mesh Filename Left"].GetString();
            m_meshFile_right = d["Visualization"]["Mesh Filename Right"].GetString();
            m_has_mesh = true;
        }
    }
}

double LugreTire::GetNormalStiffnessForce(double depth) const {
    return m_normalStiffness * depth;
}

double LugreTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_normalDamping * velocity;
}

// -----------------------------------------------------------------------------
void LugreTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH && m_has_mesh) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    }
    else {
        ChLugreTire::AddVisualizationAssets(vis);
    }
}

void LugreTire::RemoveVisualizationAssets() {
    ChLugreTire::RemoveVisualizationAssets();
    RemoveVisualizationMesh(m_trimesh_shape);
}

}  // end namespace vehicle
}  // end namespace chrono
