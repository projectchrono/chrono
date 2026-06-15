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
// This is an adaptation of the preCICE tutorial example for coupling a
// multibody system to OpenFOAM (see https://precice.org/quickstart.html),
// modified to use a Chrono MBS preCICE participant for the solid phase.
//
// =============================================================================

#include "chrono/utils/ChForceFunctors.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

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
    ////#ifdef _DEBUG
    ////    int foo;
    ////    cout << "Enter something to continue..." << endl;
    ////    cin >> foo;
    ////#endif

    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Ensure the program is run from the correct directory
    auto crt_path = std::filesystem::current_path().string();
    cout << "Working directory: " << crt_path << endl;
    auto run_script = std::filesystem::path(crt_path + "/run.sh");
    if (!exists(run_script)) {
        cerr << "Incorrect working directory. Run through the 'run.sh' script in the data/precice/flap_openfoam/solid_chrono/ directory." << endl;
        return 1;
    }

    // Set path to Chrono data (based on assumed working directory)
    SetChronoDataPath("../../../");

    // Set path to preCICE configuration file (based on assumed working directory)
    std::string precice_config_filename = "../precice_config.xml";

    // Enable verbose terminal output
    bool verbose = true;

    // Create the Chrono MBS participant
    ChPreciceAdapterMbs participant("./mbs_participant.yaml", verbose);

    // Access RSDA in MBS model
    auto& sys = participant.GetSystem();
    auto rsda = std::dynamic_pointer_cast<ChLinkRSDA>(sys.SearchLink("rsda"));
    ChAssertAlways(rsda);
    auto functor = std::dynamic_pointer_cast<utils::LinearSpringTorque>(rsda->GetTorqueFunctor());
    ChAssertAlways(functor);

    // Add callback for pre-step operations
    class SpringCoefficientCallback : public ChPreciceAdapterMbs::BeforeStepDynamicsCallback {
      public:
        SpringCoefficientCallback(std::shared_ptr<ChLinkRSDA> rsda) : rsda(rsda) {}
        virtual void OnStepDynamics(double time, double step) override {
            constexpr double spring_constant = 25;
            constexpr double stiffening_factor = 8;
            constexpr double switch_time = 1.5;
            auto functor = std::static_pointer_cast<utils::LinearSpringTorque>(rsda->GetTorqueFunctor());
            functor->SetSpringCoefficient((time < switch_time) ? spring_constant : spring_constant * stiffening_factor);
        }
        std::shared_ptr<ChLinkRSDA> rsda;
    };
    participant.RegisterBeforeStepDynamicsCallback(chrono_types::make_shared<SpringCoefficientCallback>(rsda));

    // Create and set output directory
    std::string out_dir = "results";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    participant.SetOutputDir(out_dir);

    // Enable simulation output (if available and configured)
    participant.EnableOutput(true);

    // Register preCICE participant, initialize and run simulation
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