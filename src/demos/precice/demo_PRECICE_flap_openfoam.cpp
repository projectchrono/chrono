// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Demonstration for the Chrono MBS preCICE adapter, using a flap mechanism
// set up for co-simulation with OpenFOAM.
//
// =============================================================================

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::ch_precice;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

void GenerateData();

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Debug pause to allow attaching a debugger before MPI initialization
#ifdef _DEBUG
    int foo;
    cout << "Enter something to continue..." << endl;
    cin >> foo;
#endif

    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    ////GenerateData();

    // Problem settings
    std::string precice_config_filename = GetChronoDataFile("precice/flap_openfoam/precise_config.xml");
    bool verbose = true;

    // Set up Chrono MBS participant
    ChPreciceAdapterMbs participant(GetChronoDataFile("precice/flap_openfoam/solid_chrono/mbs_participant.yaml"), verbose);
    participant.RegisterParticipant(precice_config_filename);
    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();

    return 0;
}

// -----------------------------------------------------------------------------

//                  upper y
//          o---o---o---o---o---o---o
//          |.......................|
// lower x  o.......................o   upper x
//          |.......................|
//          o---o---o---o---o---o---o
//                  lower y
//
void GenerateData() {
    // Mesh configuration
    constexpr int vertical_refinement = 3;
    constexpr int horizontal_refinement = 6;
    // Rotation center is at (0,0)
    constexpr double length = 0.2;
    constexpr double height = 0.02;
    // Derived quantities
    constexpr int n_vertical_nodes = vertical_refinement * 2 + 1;
    constexpr int n_horizontal_nodes = horizontal_refinement * 2 + 1;
    // Subtract shared nodes at each rigid body corner
    constexpr int n_nodes = (n_vertical_nodes + n_horizontal_nodes - 2) * 2;
    constexpr double delta_y = height / (n_vertical_nodes - 1);
    constexpr double delta_x = length / (n_horizontal_nodes - 1);

    std::vector<ChVector3d> vertices(n_nodes);

    // x planes
    for (int i = 0; i < n_vertical_nodes; ++i) {
        // lower x plane
        vertices[i] = ChVector3d(0.0, -height * 0.5 + delta_y * i, 0.0);
        // upper x plane
        vertices[n_vertical_nodes + i] = ChVector3d(length, vertices[i].y(), 0.0);
    }

    // y planes
    const unsigned int of = 2 * n_vertical_nodes;  // static offset
    // Lower and upper bounds are already included due to positive/negative x-planes
    const unsigned int n_remaining_nodes = n_horizontal_nodes - 2;
    for (unsigned int i = 0; i < n_remaining_nodes; ++i) {
        // lower y plane
        vertices[of + i] = ChVector3d(delta_x * (i + 1), -height * 0.5, 0.0);
        // upper y plane
        vertices[of + n_remaining_nodes + i] = ChVector3d(vertices[of + i].x(), -vertices[of + i].y(), 0.0);
    }

    // Mass and inertia
    constexpr double density = 10000;
    constexpr double mass = length * height * density;
    constexpr double inertia_moment = (1. / 12) * mass * (4 * length * length + height * height);
    std::cout << mass << " ... " << inertia_moment << endl;

    // Write vertices
    cout << vertices.size() << endl;
    for (const auto& v : vertices)
        cout << v << endl;
}