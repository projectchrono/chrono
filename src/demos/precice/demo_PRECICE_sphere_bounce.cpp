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
// Program for...
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

void RunParticipantMBS(const std::string& precice_config_filename, bool verbose);
void RunParticipantCFD(const std::string& precice_config_filename, bool verbose);

// =============================================================================

int main(int argc, char* argv[]) {
    // Debug pause to allow attaching a debugger before MPI initialization
#ifdef _DEBUG
    int foo;
    cout << "Enter something to continue..." << endl;
    cin >> foo;
#endif

    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Problem settings
    std::string precice_config_filename = GetChronoDataFile("precice/mbs/sphere/buoyancy.xml");
    bool verbose = false;

    // Get the participant type (MBS or CFD)
    ChCLI cli(argv[0], "");
    cli.AddOption<std::string>("", "participant", "participant type (MBS or CFD)");
    if (!cli.Parse(argc, argv, true))
        return 1;
    auto type = cli.GetAsType<std::string>("participant");
    if (type == "MBS")
        RunParticipantMBS(precice_config_filename, verbose);
    else if (type == "CFD")
        RunParticipantCFD(precice_config_filename, verbose);
    else
        cerr << "Unrecognized participant. Use 'MBS' or 'CFD'" << endl;

    return 0;
}

// =============================================================================

void RunParticipantMBS(const std::string& precice_config_filename, bool verbose) {
    ChPreciceAdapterMbs participant(GetChronoDataFile("precice/mbs/sphere/mbs.yaml"), verbose);
    participant.RegisterSolver(precice_config_filename);
    participant.InitializeSimulation();
    participant.SimulationLoop();
    participant.FinalizeSimulation();
}

// =============================================================================

class ParticipantCFD : public ChPreciceAdapter {
  public:
    ParticipantCFD(bool verbose);

    virtual void InitializeParticipant() override;
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void WriteData() override;

  private:
    std::string mesh_name;
    std::string read_data_name;
    std::string write_data_name;
    double radius = 0.2;
    std::vector<double> positions;
    std::vector<double> forces;
};

ParticipantCFD::ParticipantCFD(bool verbose) : ChPreciceAdapter() {
    SetVerbose(verbose);
    ConstructSolver(GetChronoDataFile("precice/mbs/sphere/cfd.yaml"));
}

void ParticipantCFD::InitializeParticipant() {
    ChPreciceAdapter::InitializeParticipant();

    // Check that the participant has the expected number of interfaces (1 mesh)
    auto mesh_names = GetMeshNames();
    ChAssertAlways(mesh_names.size() == 1);

    // Check the dimension of the one and only mesh
    mesh_name = mesh_names[0];
    int mesh_dim = GetMeshDimensions(mesh_name);
    ChAssertAlways(mesh_dim == 3);

    // Create the coupling mesh
    std::vector<double> vertices(3, 0.0);
    RegisterMesh(mesh_name, vertices);

    read_data_name = "positions";
    write_data_name = "forces";

    // Size and initialize input and output data
    positions.resize(3, 0.0);
    forces.resize(3, 0.0);
}

void ParticipantCFD::ReadData() {
    ChPreciceAdapter::ReadData();
    positions = m_coupling_meshes[mesh_name].data[read_data_name].values;
    if (m_verbose) {
        cout << m_prefix2 << "positions:  " << positions[0] << " " << positions[1] << " " << positions[2] << endl;
    }
}

double ParticipantCFD::GetSolverTimeStep(double max_time_step) const {
    return std::min(max_time_step, 1e-2);
}

void ParticipantCFD::AdvanceParticipant(double time, double time_step) {
    ChPreciceAdapter::AdvanceParticipant(time, time_step);

    // Calculate volume of displaced fluid assuming surface at z=0
    double h = radius - positions[2];
    if (h < 0)
        forces[2] = 0;
    else {
        h = std::min(h, 2 * radius);
        double rho = 1000;
        double g = 9.81;
        double V = CH_PI * h * h * (radius - CH_1_3 * h);
        forces[2] = rho * g * V;
        if (m_verbose)
            cout << m_prefix2 << "h = " << h << "  BUYOANCY force = " << forces[2] << endl;
    }
}

void ParticipantCFD::WriteData() {
    m_coupling_meshes[mesh_name].data[write_data_name].values = forces;
    ChPreciceAdapter::WriteData();
    if (m_verbose) {
        cout << m_prefix2 << "forces: " << forces[0] << " " << forces[1] << " " << forces[2] << endl;
    }
}

// -----------------------------------------------------------------------------

void RunParticipantCFD(const std::string& precice_config_filename, bool verbose) {
    ParticipantCFD participant(verbose);

    participant.RegisterSolver(precice_config_filename);
    participant.InitializeSimulation();
    participant.SimulationLoop();
    participant.FinalizeSimulation();
}
