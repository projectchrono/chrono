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
// Two versions of the problem are available: a 2D flap with a revolute joint
// and a 3D flap with a spherical joint. Select one with the -2D or -3D command
// line argument; without a recognized argument, the demo asks interactively.
//
// =============================================================================

#include <algorithm>
#include <cctype>

#include "chrono/utils/ChForceFunctors.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

using namespace chrono;
using namespace chrono::ch_precice;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

enum class FlapModel { FLAP_2D, FLAP_3D };

FlapModel SelectModel(int argc, char* argv[]);
void GenerateData(FlapModel model);

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Debug pause to allow attaching a debugger before MPI initialization
    ////#ifdef _DEBUG
    ////    int foo;
    ////    cout << "Enter something to continue..." << endl;
    ////    cin >> foo;
    ////#endif

    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Select the 2D or 3D version of the problem
    auto model = SelectModel(argc, argv);
    std::string case_name = (model == FlapModel::FLAP_2D) ? "flap_openfoam_2D" : "flap_openfoam_3D";
    cout << "Problem: " << case_name << endl;

    // Ensure the program is run from the correct directory
    auto crt_path = std::filesystem::current_path();
    cout << "Working directory: " << crt_path.string() << endl;
    bool valid_dir = std::filesystem::exists(crt_path / "run.sh") &&  //
                     crt_path.filename() == "solid_chrono" &&         //
                     crt_path.parent_path().filename() == case_name;
    if (!valid_dir) {
        cerr << "Incorrect working directory. Run through the 'run.sh' script in the data/precice/" << case_name << "/solid_chrono/ directory." << endl;
        return 1;
    }

    // Set path to Chrono data (based on assumed working directory)
    SetChronoDataPath("../../../");

    // Set paths to preCICE configuration file and MBS configuration file (based on assumed working directory)
    std::string precice_config_filename = "../precice_config.xml";
    std::string participant_config_filename = "./mbs_participant.yaml";

    // Enable verbose terminal output
    bool verbose = true;

    // Create the Chrono MBS participant
    ChPreciceAdapterMbs participant(precice_config_filename, participant_config_filename, verbose);

    // Collect all RSDAs in the MBS model (one in the 2D model, two in the 3D model)
    auto& sys = participant.GetSystem();
    std::vector<std::shared_ptr<ChLinkRSDA>> rsdas;
    for (const auto& link : sys.GetLinks()) {
        if (auto rsda = std::dynamic_pointer_cast<ChLinkRSDA>(link)) {
            ChAssertAlways(std::dynamic_pointer_cast<utils::LinearSpringTorque>(rsda->GetTorqueFunctor()));
            rsdas.push_back(rsda);
        }
    }
    ChAssertAlways(!rsdas.empty());

    // Add callback for pre-step operations
    class SpringCoefficientCallback : public ChPreciceAdapterMbs::BeforeStepDynamicsCallback {
      public:
        SpringCoefficientCallback(const std::vector<std::shared_ptr<ChLinkRSDA>>& rsdas) : rsdas(rsdas) {}
        virtual void OnStepDynamics(double time, double step) override {
            constexpr double spring_constant = 25;
            constexpr double stiffening_factor = 8;
            constexpr double switch_time = 1.5;
            for (const auto& rsda : rsdas) {
                auto functor = std::static_pointer_cast<utils::LinearSpringTorque>(rsda->GetTorqueFunctor());
                functor->SetSpringCoefficient((time < switch_time) ? spring_constant : spring_constant * stiffening_factor);
            }
        }
        std::vector<std::shared_ptr<ChLinkRSDA>> rsdas;
    };
    participant.RegisterBeforeStepDynamicsCallback(chrono_types::make_shared<SpringCoefficientCallback>(rsdas));

    // Create and set output directory
    std::string out_dir = "results";
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    participant.SetOutputDir(out_dir);

    // Enable simulation output (if available and configured)
    participant.EnableOutput(true);

    // Create preCICE participant, initialize and run simulation
    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();

    return 0;
}

// -----------------------------------------------------------------------------

// Select the 2D or 3D model from the first command line argument (-2D or -3D).
// Without a recognized argument, ask the user interactively.
FlapModel SelectModel(int argc, char* argv[]) {
    if (argc > 1) {
        std::string arg = argv[1];
        std::transform(arg.begin(), arg.end(), arg.begin(), [](unsigned char c) { return (char)std::toupper(c); });
        if (arg == "-2D")
            return FlapModel::FLAP_2D;
        if (arg == "-3D")
            return FlapModel::FLAP_3D;
        cout << "Unrecognized argument '" << argv[1] << "' (use -2D or -3D)" << endl;
    }

    int which = 1;
    cout << "Options:" << endl;
    cout << "  1. 2D flap (revolute joint)" << endl;
    cout << "  2. 3D flap (spherical joint)" << endl;
    cout << "\nSelect problem: ";
    cin >> which;
    cout << endl;
    ChClampValue(which, 1, 2);

    return (which == 1) ? FlapModel::FLAP_2D : FlapModel::FLAP_3D;
}

// -----------------------------------------------------------------------------

// Utility function to generate the coupling points on the flap surface (flap.txt) and the flap mass properties.
//
// 2D model: points on the flap outline in the (x-y) plane.
//
//                  upper y
//          o---o---o---o---o---o---o
//          |.......................|
// lower x  o.......................o   upper x
//          |.......................|
//          o---o---o---o---o---o---o
//                  lower y
//
// 3D model: grid nodes on the 6 faces of the flap plate (spanning the z direction).
//
void GenerateData(FlapModel model) {
    // Mesh configuration
    constexpr int vertical_refinement = 3;
    constexpr int horizontal_refinement = 6;
    constexpr int span_refinement = 6;
    // Rotation center is at (0,0,0)
    constexpr double length = 0.2;
    constexpr double height = 0.02;
    constexpr double width = 0.2;  // plate span (3D model only)
    // Derived quantities
    constexpr int n_vertical_nodes = vertical_refinement * 2 + 1;
    constexpr int n_horizontal_nodes = horizontal_refinement * 2 + 1;
    constexpr int n_span_nodes = span_refinement * 2 + 1;
    constexpr double delta_y = height / (n_vertical_nodes - 1);
    constexpr double delta_x = length / (n_horizontal_nodes - 1);
    constexpr double delta_z = width / (n_span_nodes - 1);

    std::vector<ChVector3d> vertices;

    if (model == FlapModel::FLAP_2D) {
        // Subtract shared nodes at each rigid body corner
        constexpr int n_nodes = (n_vertical_nodes + n_horizontal_nodes - 2) * 2;
        vertices.resize(n_nodes);

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
    } else {
        // Keep the nodes of an (x,y,z) grid that lie on the plate surface
        for (int i = 0; i < n_horizontal_nodes; ++i) {
            for (int j = 0; j < n_vertical_nodes; ++j) {
                for (int k = 0; k < n_span_nodes; ++k) {
                    bool on_surface = i == 0 || i == n_horizontal_nodes - 1 ||  //
                                      j == 0 || j == n_vertical_nodes - 1 ||    //
                                      k == 0 || k == n_span_nodes - 1;
                    if (on_surface)
                        vertices.push_back(ChVector3d(delta_x * i, -height * 0.5 + delta_y * j, -width * 0.5 + delta_z * k));
                }
            }
        }
    }

    // Mass and inertia
    constexpr double density = 10000;
    if (model == FlapModel::FLAP_2D) {
        // Mass per unit depth and moment of inertia about the rotation center
        constexpr double mass = length * height * density;
        constexpr double inertia_moment = (1. / 12) * mass * (4 * length * length + height * height);
        std::cout << mass << " ... " << inertia_moment << endl;
    } else {
        // Mass and principal moments of inertia about the center of mass
        constexpr double mass = length * height * width * density;
        constexpr double inertia_xx = (1. / 12) * mass * (height * height + width * width);
        constexpr double inertia_yy = (1. / 12) * mass * (length * length + width * width);
        constexpr double inertia_zz = (1. / 12) * mass * (length * length + height * height);
        std::cout << mass << " ... " << inertia_xx << "  " << inertia_yy << "  " << inertia_zz << endl;
    }

    // Write vertices
    cout << vertices.size() << endl;
    for (const auto& v : vertices)
        cout << v << endl;
}
