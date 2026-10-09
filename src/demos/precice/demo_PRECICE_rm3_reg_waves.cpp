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
// Demonstration for Chrono preCICE adapters: the RM3 two-body point absorber in
// regular waves, coupling a Chrono multibody solid phase to a Chrono::TDPF fluid
// phase. This is the co-simulation counterpart of demo_FSI-TDPF_rm3_reg_waves.
//
// Unlike demo_PRECICE_sphere_drop, the coupling interface here carries more than
// one FSI body: the float and the reaction plate are both hydrodynamic bodies,
// exchanged as two vertices of the same coupling mesh. The multibody model also
// includes a body with no hydrodynamics, to show that the coupling interface need
// not cover the entire model.
//
// This program must be run twice, in separate processes, once per participant:
//   demo_PRECICE_rm3_reg_waves -p Solid
//   demo_PRECICE_rm3_reg_waves -p Fluid_TDPF
//
// =============================================================================

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

#ifdef CHRONO_FSI_TDPF
    #include "chrono_precice/ChPreciceAdapterTdpf.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::ch_precice;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

void RunParticipantMBS(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output);
void RunParticipantTDPF(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output);

// =============================================================================

int main(int argc, char* argv[]) {
#ifdef _DEBUG
    // Debug pause to allow attaching a debugger before MPI initialization
    int foo;
    cout << "Enter something to continue..." << endl;
    cin >> foo;
#endif

    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Enable verbose terminal output
    bool verbose = true;
    bool visualize = true;
    bool output = true;

    // Default preCICE configuration file
    std::string precice_config_filename = GetChronoDataFile("precice/rm3_reg_waves/precice_config_AM_explicit.xml");

    // Get the participant type from the command line arguments
    std::string help =
        "Specify the participant type, one of:\n"         //
        " 'Solid'      - Chrono multibody solid phase\n"  //
        " 'Fluid_TDPF' - Chrono::TDPF fluid solver\n";    //

    ChCLI cli(argv[0], help);
    cli.AddOption<std::string>("", "p,participant_type", "participant type (Solid, Fluid_TDPF)");
    cli.AddOption<std::string>("", "c,config_file", "preCICE configuration file", precice_config_filename);

    if (!cli.Parse(argc, argv, true))
        return 1;

    std::string type;
    try {
        type = cli.GetAsType<std::string>("participant_type");
    } catch (std::domain_error&) {
        cli.Help();
        return 1;
    }

    precice_config_filename = cli.Get("config_file").as<std::string>();

    // Set root output directory
    std::string out_dir = GetChronoOutputPath() + "PRECICE_RM3_Reg_Waves/";
    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    // Run the specified preCICE participant
    if (type == "Solid")
        RunParticipantMBS(precice_config_filename, out_dir, verbose, visualize, output);
    else if (type == "Fluid_TDPF")
        RunParticipantTDPF(precice_config_filename, out_dir, verbose, visualize, output);
    else
        cerr << "Unrecognized participant. Use 'Solid' or 'Fluid_TDPF'" << endl;

    return 0;
}

// =============================================================================

void RunParticipantMBS(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output) {
    ChPreciceAdapterMbs participant(precice_config_filename, GetChronoDataFile("precice/rm3_reg_waves/solid_chrono/mbs_participant.yaml"), verbose);

    auto mbs_out_dir = out_dir + "mbs";
    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(mbs_out_dir))) {
            std::cout << "Error creating directory " << mbs_out_dir << std::endl;
            throw std::runtime_error("Error creating MBS output directory");
        }
        participant.SetOutputDir(mbs_out_dir);
    }

    participant.EnableOutput(output);
    participant.EnableVisualization(visualize);
    participant.EnforceRealtime(visualize);

    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();
}

// =============================================================================

void RunParticipantTDPF(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output) {
#ifdef CHRONO_FSI_TDPF
    ChPreciceAdapterTdpf participant(precice_config_filename, GetChronoDataFile("precice/rm3_reg_waves/fluid_tdpf/tdpf_participant.yaml"), verbose);

    auto tdpf_out_dir = out_dir + "tdpf";
    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(tdpf_out_dir))) {
            std::cout << "Error creating directory " << tdpf_out_dir << std::endl;
            throw std::runtime_error("Error creating TDPF output directory");
        }
        participant.SetOutputDir(tdpf_out_dir);
    }

    participant.EnableOutput(output);
    participant.EnableVisualization(visualize);

    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();
#else
    cerr << "Chrono was not configured with FSI-TDPF support!" << endl;
    throw std::runtime_error("Chrono was not configured with FSI-TDPF support");
#endif
}
