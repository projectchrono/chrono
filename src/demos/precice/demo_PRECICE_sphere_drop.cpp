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
// Demonstration for Chrono preCICE adapters: simulation of a sphere dropped in
// fluid. The fluid phase preCICE participant can be one of:
// (a) a mock-up fluid solver that only applies buoyancy and drag forces
// (b) a Chrono::SPH solver
//
// =============================================================================

#include <fstream>

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_precice/ChPreciceAdapterMbs.h"

#ifdef CHRONO_FSI_SPH
    #include "chrono_precice/ChPreciceAdapterSph.h"
#endif

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
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
void RunParticipantCFD(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output);
void RunParticipantSPH(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output);

// =============================================================================

int main(int argc, char* argv[]) {
#ifdef _DEBUG
    // Debug pause to allow attaching a debugger before MPI initialization
    int foo;
    cout << "Enter something to continue..." << endl;
    cin >> foo;
#endif

    cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Problem settings
    std::string precice_config_filename = GetChronoDataFile("precice/sphere_drop/precice_config_explicit.xml");

    // Enable verbose terminal output
    bool verbose = true;
    bool visualize = true;
    bool output = true;

    // Set root output directory
    std::string out_dir = GetChronoOutputPath() + "PRECICE_Sphere_Drop/";
    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }

    // Get the participant type from the command line arguments
    std::string help =
        "Specify the participant type, one of:\n"                                        //
        " 'Solid'      - Chrono multibody solid phase\n"                                 //
        " 'Fluid_SPH'  - Chrono::SPH fluid solver\n"                                     //
        " 'Fluid_BUOY' - mock-up fluid solver that applies buoyancy and drag forces\n";  //

    ChCLI cli(argv[0], help);
    cli.AddOption<std::string>("", "p,participant_type", "participant type (Solid, Fluid_BUOY, Fluid_SPH)");

    if (!cli.Parse(argc, argv, true))
        return 1;

    std::string type;
    try {
        type = cli.GetAsType<std::string>("participant_type");
    } catch (std::domain_error&) {
        cli.Help();
        return 1;
    }

    // Run the specified preCICE participant
    if (type == "Solid")
        RunParticipantMBS(precice_config_filename, out_dir, verbose, visualize, output);
    else if (type == "Fluid_BUOY")
        RunParticipantCFD(precice_config_filename, out_dir, verbose, visualize, output);
    else if (type == "Fluid_SPH")
        RunParticipantSPH(precice_config_filename, out_dir, verbose, visualize, output);
    else
        cerr << "Unrecognized participant. Use 'Solid', 'Fluid_BUOY', or 'Fluid_SPH'" << endl;

    return 0;
}

// =============================================================================

void RunParticipantMBS(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output) {
    ChPreciceAdapterMbs participant(GetChronoDataFile("precice/sphere_drop/solid_chrono/mbs_participant.yaml"), verbose);

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

    participant.RegisterParticipant(precice_config_filename);
    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();
}

// =============================================================================

void RunParticipantSPH(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output) {
#ifdef CHRONO_FSI_SPH
    ChPreciceAdapterSph participant(GetChronoDataFile("precice/sphere_drop/fluid_sph/sph_participant.yaml"), verbose);

    auto sph_out_dir = out_dir + "sph";
    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(sph_out_dir))) {
            std::cout << "Error creating directory " << sph_out_dir << std::endl;
            throw std::runtime_error("Error creating SPH output directory");
        }
        participant.SetOutputDir(sph_out_dir);
    }
#else
    cerr << "Chrono was not configured with FSI-SPH support!" << endl;
    throw("Chrono was not configured with FSI-SPH support");
#endif

    participant.EnableOutput(output);
    participant.EnableVisualization(visualize);

    participant.RegisterParticipant(precice_config_filename);
    participant.InitializeSimulation();

    //// DEBUG - advance SPH participant with no data exchange
    ////auto h = participant.GetSolverTimeStep(1000);
    ////double t = 0;
    ////for (int i = 0; i < 10000; i++) {
    ////    participant.AdvanceParticipant(t, h);
    ////    t += h;
    ////}

    participant.RunSimulation();
    participant.FinalizeSimulation();
}

// =============================================================================

class ParticipantCFD : public ChPreciceAdapter {
  public:
    ParticipantCFD(bool verbose);
    ~ParticipantCFD();

    virtual void InitializeParticipant() override;
    virtual void WriteCheckpoint(double time) override {}
    virtual void ReadCheckpoint(double time) override {}
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void WriteData() override;
    virtual void WriteOutput(int frame, double time) override {}

    void PlotResults();

  private:
    std::string mesh_name;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> force;

    std::ofstream file;  // output file stream

    double radius = 0.12;  // sphere radius
    double cd = 0.47;      // sphere drag coefficient
    double rho = 1000;     // water density
    double g = 9.81;       // gravitational acceleration
};

ParticipantCFD::ParticipantCFD(bool verbose) : ChPreciceAdapter() {
    SetVerbose(verbose);
    ReadParticipantConfigurationYAML(GetChronoDataFile("precice/sphere_drop/fluid_buoyancy/cfd_participant.yaml"));
}

ParticipantCFD::~ParticipantCFD() {
    if (file.is_open())
        file.close();
}

void ParticipantCFD::InitializeParticipant() {
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
    // Calculate fluid forces for current sphere position and velocity
    double h = radius - position[2];
    force[2] = 0;
    if (h > 0) {
        h = std::min(h, 2 * radius);

        // Calculate volume of displaced fluid (assuming surface at z=0) and buoyancy force
        double volume = CH_PI * h * h * (radius - CH_1_3 * h);
        double frc_b = rho * g * volume;

        // Calculate dynamic viscous drag force
        double v = velocity[2];
        double frc_d = 0.5 * cd * rho * v * v * (CH_PI * radius * radius);

        // Set total force
        force[2] = (v < 0) ? frc_b + frc_d : frc_b - frc_d;

        if (m_verbose)
            cout << m_prefix2 << "h = " << h << "  buoyancy = " << frc_b << "  drag = " << frc_d << endl;
    }

    // Write output
    if (m_output) {
        if (!file.is_open())
            file.open(m_output_dir + "/cfd_results.txt");
        file << time << " " << h << " " << velocity[2] << " " << force[2] << std::endl;
    }
}

void ParticipantCFD::WriteData() {
    m_coupling_meshes[mesh_name].data["forces"].values = force;
    if (m_verbose) {
        cout << m_prefix2 << "force: " << force[0] << " " << force[1] << " " << force[2] << endl;
    }

    ChPreciceAdapter::WriteData();
}

void ParticipantCFD::PlotResults() {
    if (!m_output)
        return;

#ifdef CHRONO_POSTPROCESS
    if (file.is_open())
        file.close();

    std::string out_filename = m_output_dir + "/cfd_results.txt";
    std::string gpl_filename = m_output_dir + "/cfd_plot.gpl";

    postprocess::ChGnuPlot gplot(gpl_filename);
    gplot.SetCanvasSize(800, 600);
    gplot.SetGrid(false, 1, ChColor(0.8f, 0.8f, 0.8f));
    gplot.SetLabelX("time (s)");
    gplot.SetLabelY("depth (m) and vel (m/s)");
    gplot.SetLabelY2("force (N)");
    gplot.SetCommand("set ytics 0.25 nomirror");
    gplot.SetCommand("set y2tics 50.0 nomirror");
    gplot.SetTitle("Bouncing sphere");
    gplot.Plot(out_filename, 1, 2, "depth", " axes x1y1 with lines lt rgb '#FF5500' lw 2");
    gplot.Plot(out_filename, 1, 3, "vel", " axes x1y1 with lines lt rgb '#0055FF' lw 2");
    gplot.Plot(out_filename, 1, 4, "force", " axes x1y2 with lines lt rgb '#000000' lw 2");
#endif
}

// -----------------------------------------------------------------------------

void RunParticipantCFD(const std::string& precice_config_filename, const std::string& out_dir, bool verbose, bool visualize, bool output) {
    ParticipantCFD participant(verbose);

    auto cfd_out_dir = out_dir + "cfd";
    if (output) {
        if (!CreateOutputDirectory(std::filesystem::path(cfd_out_dir))) {
            std::cout << "Error creating directory " << cfd_out_dir << std::endl;
            throw std::runtime_error("Error creating CFD output directory");
        }
        participant.SetOutputDir(cfd_out_dir);
    }

    participant.EnableOutput(output);
    participant.EnableVisualization(visualize);

    participant.RegisterParticipant(precice_config_filename);
    participant.InitializeSimulation();
    participant.RunSimulation();
    participant.FinalizeSimulation();

    participant.PlotResults();
}
