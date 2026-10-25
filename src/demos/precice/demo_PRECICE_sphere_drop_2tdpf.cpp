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
// Demonstration for Chrono preCICE adapters: co-simulation with more than two
// participants. A sphere dropped in fluid is simulated with:
// - one Chrono MBS solid participant ('Solid'), and
// - two Chrono::TDPF fluid participants ('Fluid1' and 'Fluid2').
//
// All participants use hydrodynamic data with half the fluid density, so that
// each fluid participant applies half of the hydrodynamic loads. Denoting by M
// the sphere's infinite-frequency added mass, the solid participant holds a
// baseline added mass of M/2 (read from that same file), and the two fluid
// participants send added mass increments of M/6 (Fluid1) and M/3 (Fluid2). The
// solid participant sums the forces received from both fluid participants and
// applies its baseline added mass plus both increments. This reproduces the
// sphere drop demo with a single TDPF participant and dynamic added mass
// (demo_PRECICE_sphere_drop with the precice_config_AMdyn_explicit.xml
// configuration).
//
// A TDPF participant normally sends zero added mass increments, since its added
// mass is the constant infinite-frequency added mass, held by the solid
// participant as its baseline. To illustrate non-zero increments from several
// sources, the fluid participants of this demo derive from ChPreciceAdapterTdpf
// and override OnWriteDataAM to send a fraction of their infinite-frequency
// added mass (M/2) as increment: 1/3 for Fluid1 (M/6) and 2/3 for Fluid2 (M/3).
//
// This demo is meant to illustrate the setup of a multi-participant
// co-simulation; see the preCICE configuration file for the naming rules it
// follows.
//
// Run the three participants as separate processes, e.g.:
//   demo_PRECICE_sphere_drop_2tdpf -p Solid
//   demo_PRECICE_sphere_drop_2tdpf -p Fluid1
//   demo_PRECICE_sphere_drop_2tdpf -p Fluid2
//
// =============================================================================

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"
#include "chrono_precice/ChPreciceAdapterTdpf.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::ch_precice;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

// TDPF participant that sends a fraction of its infinite-frequency added mass as added mass increment.
// (The base class ChPreciceAdapterTdpf sends zero increments.)
class ParticipantTDPF : public ChPreciceAdapterTdpf {
  public:
    ParticipantTDPF(const std::string& precice_config_filename, const std::string& participant_filename, double fraction, bool verbose)
        : ChPreciceAdapterTdpf(precice_config_filename, participant_filename, verbose), m_fraction(fraction) {}

    virtual void OnWriteDataAM(std::vector<ChMatrix66d>& blocks) override {
        // Let the base class check consistency (it leaves the increments at zero)
        ChPreciceAdapterTdpf::OnWriteDataAM(blocks);

        // Send the specified fraction of the self blocks of the infinite-frequency added mass
        auto am_blocks = GetFluidSystemTDPF()->GetInfiniteFrequencyAddedMass();
        for (size_t i = 0; i < blocks.size(); i++)
            blocks[i] = m_fraction * am_blocks[i];
    }

  private:
    double m_fraction;  ///< fraction of the infinite-frequency added mass sent as increment
};

// -----------------------------------------------------------------------------

void RunParticipantMBS(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output);
void RunParticipantTDPF(const std::string& precice_config_filename,
                        const std::string& participant_filename,
                        double fraction,
                        const std::string& out_dir,
                        bool verbose,
                        bool visualize,
                        bool output);

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
    bool output = true;

    // Run-time visualization (solid participant only; the two fluid participants are identical)
    bool visualize_solid = true;
    bool visualize_fluid = false;

    // Default preCICE configuration file
    std::string precice_config_filename = GetChronoDataFile("precice/sphere_drop_2tdpf/precice_config_explicit.xml");

    // Get the participant type from the command line arguments
    std::string help =
        "Specify the participant type, one of:\n"          //
        " 'Solid'  - Chrono multibody solid phase\n"       //
        " 'Fluid1' - first Chrono::TDPF fluid solver\n"    //
        " 'Fluid2' - second Chrono::TDPF fluid solver\n";  //

    ChCLI cli(argv[0], help);
    cli.AddOption<std::string>("", "p,participant_type", "participant type (Solid, Fluid1, Fluid2)");
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
    std::string out_dir = GetChronoOutputPath() + "PRECICE_Sphere_Drop_2TDPF/";
    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    // Run the specified preCICE participant.
    // Each fluid participant has an infinite-frequency added mass of M/2 (fluid density halved) and sends a fraction of
    // it as added mass increment: 1/3 for Fluid1 (M/6) and 2/3 for Fluid2 (M/3).
    if (type == "Solid")
        RunParticipantMBS(precice_config_filename, out_dir, verbose, visualize_solid, output);
    else if (type == "Fluid1")
        RunParticipantTDPF(precice_config_filename, GetChronoDataFile("precice/sphere_drop_2tdpf/fluid_tdpf/tdpf_participant_1.yaml"), 1.0 / 3,
                           out_dir + "tdpf1", verbose, visualize_fluid, output);
    else if (type == "Fluid2")
        RunParticipantTDPF(precice_config_filename, GetChronoDataFile("precice/sphere_drop_2tdpf/fluid_tdpf/tdpf_participant_2.yaml"), 2.0 / 3,
                           out_dir + "tdpf2", verbose, visualize_fluid, output);
    else
        cerr << "Unrecognized participant. Use 'Solid', 'Fluid1', or 'Fluid2'" << endl;

    return 0;
}

// =============================================================================

void RunParticipantMBS(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output) {
    ChPreciceAdapterMbs participant(precice_config_filename, GetChronoDataFile("precice/sphere_drop_2tdpf/solid_chrono/mbs_participant.yaml"), verbose);

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

void RunParticipantTDPF(const std::string& precice_config_filename,
                        const std::string& participant_filename,
                        double fraction,
                        const std::string& out_dir,
                        bool verbose,
                        bool visualize,
                        bool output) {
    ParticipantTDPF participant(precice_config_filename, participant_filename, fraction, verbose);

    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            throw std::runtime_error("Error creating TDPF output directory");
        }
        participant.SetOutputDir(out_dir);
    }

    participant.EnableOutput(output);
    participant.EnableVisualization(visualize);

    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();
}
