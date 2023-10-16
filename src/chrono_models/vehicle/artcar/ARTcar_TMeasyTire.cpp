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
// Authors: Rainer Gericke, Asher Elmquist
// =============================================================================
//
// ARTcar TMeasy tire subsystem
//
// =============================================================================

#include <algorithm>
#include <cmath>

#include "chrono_models/vehicle/artcar/ARTcar_TMeasyTire.h"
#include "chrono_vehicle/ChVehicleModelData.h"

namespace chrono {
namespace vehicle {
namespace artcar {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const std::string ARTcar_TMeasyTire::m_meshFile_left = "artcar/tire_left.obj";
const std::string ARTcar_TMeasyTire::m_meshFile_right = "artcar/tire_right.obj";

const double ARTcar_TMeasyTire::m_mass = .200;
const ChVector<> ARTcar_TMeasyTire::m_inertia(.0008, 0.001, .0008);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ARTcar_TMeasyTire::ARTcar_TMeasyTire(const std::string& name) : ChTMeasyTire(name) {
    SetTMeasyParams();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ARTcar_TMeasyTire::SetTMeasyParams() {
    double h = (.170 - .103) / 2;
    double w = .0855;
    double r = h / w;
    double rimdia = .103;

    double load = 10 * 6.044 * 9.81 / 4;  // from vehicle mass / 4 * gravity for Kg->N

    GuessTruck80Par(load,   // tire load [N]
                    w,      // tire width [m]
                    r,      // aspect ratio []
                    rimdia  // rim diameter [m]
    );
}

void ARTcar_TMeasyTire::GenerateCharacteristicPlots(const std::string& dirname) {
    // Write a plot file (gnuplot) to check the tire characteristics.
    // Inside gnuplot use the command load 'filename'
    std::string filename = dirname + "/tire_" + GetName() + ".gpl";
    WritePlots(filename, "tire");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ARTcar_TMeasyTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        m_trimesh_shape = AddVisualizationMesh(m_meshFile_left,    // left side
                                               m_meshFile_right);  // right side
    } else {
        ChTMeasyTire::AddVisualizationAssets(vis);
    }
}

void ARTcar_TMeasyTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_trimesh_shape);
    ChTMeasyTire::RemoveVisualizationAssets();
}

}  // end namespace artcar
}  // end namespace vehicle
}  // end namespace chrono
