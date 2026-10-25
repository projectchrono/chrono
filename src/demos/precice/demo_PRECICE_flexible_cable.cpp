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
// Demonstration for Chrono preCICE adapters: a flexible cable (FEA mesh with ANCF
// cable elements), anchored at the bottom of a container, impacted by a collapsing
// granular (CRM) or water (CFD) column.
// - the solid participant is a Chrono MBS with an FEA mesh
// - the fluid participant is a Chrono::FSI-SPH solver
//
// The coupled simulation is the preCICE counterpart of demo_FSI-SPH_Flexible_Cable
// (single cable, no rigid bodies) and of the corresponding YAML models in
// demo_YAML_fsi. The two participants must be run in separate processes.
//
// =============================================================================

#include <fstream>

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAxyz.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

#ifdef CHRONO_FSI_SPH
    #include "chrono_precice/ChPreciceAdapterSph.h"
#endif

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::ch_precice;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

// Callback to record the trajectory of the top cable node (in the same format as demo_FSI-SPH_Flexible_Cable)
class CableTipOutput : public ChPreciceAdapterMbs::AfterStepDynamicsCallback {
  public:
    CableTipOutput(std::shared_ptr<fea::ChNodeFEAxyz> node, const std::string& filename) : m_node(node), m_init_pos(node->GetPos()), m_ofile(filename, std::ios::trunc) {}

    virtual void OnStepDynamics(double time, double step) override {
        auto pos = m_node->GetPos();
        m_ofile << time + step << "\t" << pos.x() << "\t" << pos.y() << "\t" << pos.z() << "\t" << (pos - m_init_pos).Length() << "\n";
    }

  private:
    std::shared_ptr<fea::ChNodeFEAxyz> m_node;
    ChVector3d m_init_pos;
    std::ofstream m_ofile;
};

// -----------------------------------------------------------------------------

// Create the output directory for a participant (also used for run-time visualization snapshots, if enabled).
std::string CreateParticipantOutputDirectory(const std::string& out_dir, const std::string& participant) {
    auto dir = out_dir + "/" + participant;
    if (!CreateOutputDirectory(std::filesystem::path(dir))) {
        cerr << "Error creating directory " << dir << endl;
        throw std::runtime_error("Error creating participant output directory");
    }
    return dir;
}

void RunParticipantMBS(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output) {
    ChPreciceAdapterMbs participant(precice_config_filename, GetChronoDataFile("precice/flexible_cable/solid_chrono/mbs_participant.yaml"), verbose);
    participant.SetOutputDir(CreateParticipantOutputDirectory(out_dir, "solid"));

    if (output) {
        // Record the trajectory of the top cable node
        auto mesh = participant.GetSystem().GetMeshes().front();
        auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(mesh->GetNode(0));
        participant.RegisterAfterStepDynamicsCallback(chrono_types::make_shared<CableTipOutput>(node, out_dir + "/results.txt"));
    }

    participant.EnableVisualization(visualize);
    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();
}

void RunParticipantSPH(const std::string& precice_config_filename, const std::string& out_dir, const std::string& fluid, bool verbose, bool visualize) {
#ifdef CHRONO_FSI_SPH
    auto participant_file = "precice/flexible_cable/fluid_sph/sph_participant_" + fluid + ".yaml";
    ChPreciceAdapterSph participant(precice_config_filename, GetChronoDataFile(participant_file), verbose);
    participant.SetOutputDir(CreateParticipantOutputDirectory(out_dir, "fluid"));
    participant.EnableVisualization(visualize);
    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();
#else
    cerr << "Chrono was not configured with FSI-SPH support!" << endl;
    throw std::runtime_error("Chrono was not configured with FSI-SPH support");
#endif
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
#ifdef _DEBUG
    int foo;
    cout << "Enter something to continue..." << endl;
    cin >> foo;
#endif

    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    std::string help =
        "Specify the participant type, one of:\n"        //
        " 'Solid'     - Chrono multibody solid phase\n"  //
        " 'Fluid_SPH' - Chrono::SPH fluid solver\n";     //

    ChCLI cli(argv[0], help);
    cli.AddOption<std::string>("", "p,participant_type", "participant type (Solid, Fluid_SPH)");
    cli.AddOption<std::string>("", "f,fluid", "fluid phase problem (CRM, CFD)", "CRM");
    cli.AddOption<std::string>("", "c,config_file", "preCICE configuration file (default: depends on fluid problem)", "");
    cli.AddOption<bool>("", "quiet", "Disable verbose terminal output");
    cli.AddOption<bool>("", "no_vis", "Disable run-time visualization");
    cli.AddOption<bool>("", "no_output", "Disable output of the cable tip trajectory");

    if (!cli.Parse(argc, argv, true))
        return 1;

    std::string type;
    try {
        type = cli.GetAsType<std::string>("participant_type");
    } catch (std::domain_error&) {
        cli.Help();
        return 1;
    }

    auto fluid = ChToLower(cli.GetAsType<std::string>("fluid"));
    if (fluid != "crm" && fluid != "cfd") {
        cerr << "Unrecognized fluid problem. Use 'CRM' or 'CFD'" << endl;
        return 1;
    }

    auto precice_config_filename = cli.GetAsType<std::string>("config_file");
    if (precice_config_filename.empty())
        precice_config_filename = GetChronoDataFile("precice/flexible_cable/precice_config_" + fluid + ".xml");

    bool verbose = !cli.GetAsType<bool>("quiet");
    bool visualize = !cli.GetAsType<bool>("no_vis");
    bool output = !cli.GetAsType<bool>("no_output");

    // Output directory (each participant uses a subdirectory for its own output and snapshots)
    std::string out_dir = GetChronoOutputPath() + "PRECICE_Flexible_Cable/" + ChToUpper(fluid);
    if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
        cerr << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (type == "Solid")
        RunParticipantMBS(precice_config_filename, out_dir, verbose, visualize, output);
    else if (type == "Fluid_SPH")
        RunParticipantSPH(precice_config_filename, out_dir, fluid, verbose, visualize);
    else
        cerr << "Unrecognized participant. Use 'Solid' or 'Fluid_SPH'" << endl;

    return 0;
}
