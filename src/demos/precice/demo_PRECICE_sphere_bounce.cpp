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
    bool verbose = true;

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
    participant.RegisterParticipant(precice_config_filename);
    participant.InitializeSimulation();
    participant.RunSimulation();
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
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> force;

    double radius = 0.2;  // sphere radius
    double cd = 0.47;     // sphere drag coefficient
    double rho = 1000;    // water density
    double g = 9.81;      // gravitational acceleration
};

ParticipantCFD::ParticipantCFD(bool verbose) : ChPreciceAdapter() {
    SetVerbose(verbose);
    ConfigureParticipant(GetChronoDataFile("precice/mbs/sphere/cfd.yaml"));
}

void ParticipantCFD::InitializeParticipant() {
    ChPreciceAdapter::InitializeParticipant();

    // Check that the participant has the expected number of interfaces (1 mesh)
    auto mesh_names = GetCouplingMeshNames();
    ChAssertAlways(mesh_names.size() == 1);

    // Check the dimension of the one and only mesh
    mesh_name = mesh_names[0];
    int mesh_dim = GetCouplingMeshDimensions(mesh_name);
    ChAssertAlways(mesh_dim == 3);

    // Create the coupling mesh
    std::vector<double> vertices(3, 0.0);
    RegisterMesh(mesh_name, vertices);

    // Size and initialize input and output data
    position.resize(3, 0.0);
    velocity.resize(3, 0.0);
    force.resize(3, 0.0);
}

void ParticipantCFD::ReadData() {
    ChPreciceAdapter::ReadData();
    position = m_coupling_meshes[mesh_name].data["positions"].values;
    velocity = m_coupling_meshes[mesh_name].data["velocities"].values;
    if (m_verbose) {
        cout << m_prefix2 << "position:  " << position[0] << " " << position[1] << " " << position[2] << endl;
        cout << m_prefix2 << "velocity:  " << velocity[0] << " " << velocity[1] << " " << velocity[2] << endl;
    }
}

double ParticipantCFD::GetSolverTimeStep(double max_time_step) const {
    return std::min(max_time_step, 1e-2);
}

void ParticipantCFD::AdvanceParticipant(double time, double time_step) {
    ChPreciceAdapter::AdvanceParticipant(time, time_step);

    // Calculate fluid forces for current sphere positin and velocity
    double h = radius - position[2];
    double v = velocity[2];
    force[2] = 0;
    if (h > 0) {
        h = std::min(h, 2 * radius);

        // Calculate volume of displaced fluid (assuming surface at z=0) and buoyancy force
        double volume = CH_PI * h * h * (radius - CH_1_3 * h);
        double frc_b = rho * g * volume;

        // Calculate dynamic viscous drag force
        double frc_d = 0.5 * rho * v * v * cd * (4 * CH_PI * radius * radius);

        // Set total force
        force[2] = (v < 0) ? frc_b + frc_d : frc_b - frc_d;

        if (m_verbose)
            cout << m_prefix2 << "h = " << h << "  buoyancy = " << frc_b << "  drag = " << frc_d << endl;
    }
}

void ParticipantCFD::WriteData() {
    m_coupling_meshes[mesh_name].data["forces"].values = force;
    ChPreciceAdapter::WriteData();
    if (m_verbose) {
        cout << m_prefix2 << "force: " << force[0] << " " << force[1] << " " << force[2] << endl;
    }
}

// -----------------------------------------------------------------------------

void RunParticipantCFD(const std::string& precice_config_filename, bool verbose) {
    ParticipantCFD participant(verbose);

    participant.RegisterParticipant(precice_config_filename);
    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();
}
