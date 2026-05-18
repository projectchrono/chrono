// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo for the preCICE coupling adapter, using a mock-up solver that exchanges
// data with preCICE but does not perform any actual computations.
//
// =============================================================================

#include <numeric>

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_precice/ChPreciceAdapter.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::ch_precice;

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

// -----------------------------------------------------------------------------

class TestAdapter : public ChPreciceAdapter {
  public:
    TestAdapter(const std::string& yaml_input, bool verbose = false);

    virtual void InitializeParticipant() override;
    virtual void WriteCheckpoint(double time) override;
    virtual void ReadCheckpoint(double time) override;
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void WriteData() override;

  private:
    int id;
    std::string mesh_name;

    double solver_time;
    double solver_state;
    double checkpoint_state;
    std::vector<double> solver_scalar_output;
    std::vector<double> solver_vector_output;
};

TestAdapter::TestAdapter(const std::string& yaml_input, bool verbose) : ChPreciceAdapter(), solver_time(0), solver_state(0) {
    SetVerbose(verbose);
    ConfigureParticipant(yaml_input);
}

void TestAdapter::InitializeParticipant() {
    ChPreciceAdapter::InitializeParticipant();

    // Set a unique id for different instantiations of this adapter (to differentiate their behavior)
    id = (m_participant_name == "Solver1") ? 1 : 2;

    // Check that the participant has the expected number of interfaces (1 mesh)
    auto mesh_names = GetCouplingMeshNames();
    ChAssertAlways(mesh_names.size() == 1);

    // Get dimension of the one and only mesh
    mesh_name = mesh_names[0];
    int mesh_dim = GetCouplingMeshDimensions(mesh_name);

    // Create a mock-up mesh for testing
    int num_vertices = 4;
    std::vector<double> vertices;
    switch (mesh_dim) {
        case 2: {
            cout << "Creating 2D mesh with " << num_vertices << " vertices" << endl;
            std::vector<ChVector2d> vertices_2d(num_vertices);
            for (int i = 0; i < num_vertices; ++i)
                vertices_2d[i] = ChVector2d(i + 1.0);
            vertices = ChPreciceAdapter::SetVerticesToData(vertices_2d);
            break;
        }
        case 3: {
            cout << "Creating 3D mesh with " << num_vertices << " vertices" << endl;
            std::vector<ChVector3d> vertices_3d(num_vertices);
            for (int i = 0; i < num_vertices; ++i)
                vertices_3d[i] = ChVector3d(i + 1.0);
            vertices = ChPreciceAdapter::SetVerticesToData(vertices_3d);
            break;
        }
        default:
            cerr << "Error: [SetInterfaces] Unsupported mesh dimension: " << mesh_dim << endl;
            throw std::runtime_error("[SetInterfaces] Unsupported mesh dimension");
    }

    // Register the mesh in the adapter, given its vertices, and resize interface read/write data
    RegisterMesh(mesh_name, vertices);

    // Size and initialize mock-up output (write) data
    solver_scalar_output.resize(num_vertices);
    solver_vector_output.resize(num_vertices * mesh_dim);
    std::iota(solver_scalar_output.begin(), solver_scalar_output.end(), solver_time);
    std::iota(solver_vector_output.begin(), solver_vector_output.end(), solver_time);
}

void TestAdapter::WriteCheckpoint(double time) {
    ChPreciceAdapter::WriteCheckpoint(time);
    checkpoint_state = solver_state;
}

void TestAdapter::ReadCheckpoint(double time) {
    ChPreciceAdapter::ReadCheckpoint(time);
    solver_state = checkpoint_state;
    solver_time = time;
}

void TestAdapter::ReadData() {
    ChPreciceAdapter::ReadData();
}

double TestAdapter::GetSolverTimeStep(double max_time_step) const {
    return (id == 1 ? 2e-1 : 1e-1);
}

void TestAdapter::AdvanceParticipant(double time, double time_step) {
    ChPreciceAdapter::AdvanceParticipant(time, time_step);
    ChAssertAlways(time == solver_time);

    // Dummy dynamics
    solver_state += (10 * id) * time_step;
    solver_time += time_step;

    // Update output (write) data
    std::iota(solver_scalar_output.begin(), solver_scalar_output.end(), solver_time);
    std::iota(solver_vector_output.begin(), solver_vector_output.end(), solver_time);
}

void TestAdapter::WriteData() {
    for (auto& [mesh_name, mesh_info] : m_coupling_meshes) {
        for (const auto& data_name : m_data_write[mesh_name]) {
            auto data_dim = GetCouplingDataDimensions(mesh_name, data_name);
            switch (data_dim) {
                case 1:
                    mesh_info.data[data_name].values = solver_scalar_output;
                    break;
                case 3:
                    mesh_info.data[data_name].values = solver_vector_output;
                    break;
                default:
                    cerr << "Error: [WriteData] Unsupported data dimension: " << data_dim << endl;
                    throw std::runtime_error("[WriteData] Unsupported mesh dimension");
            }


            auto& data_info = mesh_info.data[data_name];
        }
    }

    ChPreciceAdapter::WriteData();
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Debug pause to allow attaching a debugger before MPI initialization
    ////#ifdef _DEBUG
    ////    int foo;
    ////    cout << "Enter something to continue..." << endl;
    ////    cin >> foo;
    ////#endif
  
  std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Get input file name from command line arguments
    ChCLI cli(argv[0], "");
    cli.AddOption<std::string>("", "f,input_filename", "preCICE adapter configuration file (YAML)");
    if (!cli.Parse(argc, argv, true))
        return 1;
    auto input_filename = cli.GetAsType<std::string>("input_filename");
    cout << "YAML adapter configuration file: " << input_filename << endl;

    // Create the participant and register it with preCICE
    std::string precice_config_filename = GetChronoDataFile("precice/test/test.xml");
    TestAdapter participant(input_filename, true);

    // Register solver with preCICE
    participant.RegisterParticipant(precice_config_filename);

    // Initialize simulation
    participant.InitializeSimulation();

    // Perform simulation loop
    participant.RunSimulation();

    // Finalize the preCICE coupling
    participant.FinalizeSimulation();

    return 0;
}
