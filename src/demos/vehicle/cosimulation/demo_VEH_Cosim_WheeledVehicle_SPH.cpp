// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Demo for Polaris mwheeled vehicle cosimulation on SPH terrain.
// The vehicle (specified through JSON files, for the vehicle, engine,
// and transmission), is co-simulated with a terrain node and a number of rigid
// tire nodes equal to the number of wheels.
//
// Global reference frame: Z up, X towards the front, and Y pointing to the left
//
// =============================================================================

#include <iostream>
#include <string>
#include <limits>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#include "chrono_vehicle/cosim/mbs/ChVehicleCosimWheeledVehicleNode.h"
#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeRigid.h"
#include "chrono_vehicle/cosim/terrain/ChVehicleCosimTerrainNodeGranularSPH.h"

using std::cout;
using std::cin;
using std::endl;

using namespace chrono;
using namespace chrono::vehicle;

std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);

// =============================================================================

class DriverWrapper : public ChPathFollowerDriver {
  public:
    DriverWrapper(ChVehicle& vehicle,                   // associated vehicle
                  std::shared_ptr<ChBezierCurve> path,  // Bezier curve with target path
                  const std::string& path_name,         // name of the path curve
                  double target_speed,                  // constant target speed
                  double delay = 0.5                    // delay in applying driver inputs
                  )
        : ChPathFollowerDriver(vehicle, path, path_name, target_speed), m_delay(delay) {}

    virtual void Advance(double step) {
        ChPathFollowerDriver::Advance(step);
        double t = m_vehicle.GetChTime();

        // Override ChPathFollowerDriver calculated driver inputs in initial transision phase.
        // Zero inputs for m_delay, then increase linearly to desired throttle.
        if (t < m_delay) {
            m_steering = 0;
            m_throttle = 0;
            m_braking = 1;
        } else {
            ChClampValue(m_throttle, m_throttle, 0.2 * (t - m_delay) / m_delay);
        }
        m_throttle *= 0.7;
    }

  private:
    double m_delay;
};

// =============================================================================

int main(int argc, char** argv) {
    // Initialize MPI.
    int num_procs;
    int rank;
    int name_len;
    char procname[MPI_MAX_PROCESSOR_NAME];

    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &num_procs);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Get_processor_name(procname, &name_len);

#ifdef _DEBUG
    if (rank == 0) {
        int foo;
        cout << "Enter something to continue..." << endl;
        cin >> foo;
    }
    MPI_Barrier(MPI_COMM_WORLD);
#endif

    if (num_procs != 6) {
        if (rank == 0)
            std::cout << "\n\n4-wheel vehicle cosimulation code must be run on exactly 6 ranks!\n\n" << std::endl;
        MPI_Abort(MPI_COMM_WORLD, 1);
        return 1;
    }

    // Simulation parameters
    double step_size = 5e-4;
    double sim_time = 20.0;

    double output_fps = 100;
    double render_fps = 100;

    bool output = false;
    bool renderRT = true;
    bool renderPP = false;
    bool writeRT = true;
    bool verbose = false;

    std::string path_specfile = "terrain/sph/S-lane_RMS/path.txt";
    std::string terrain_specfile = "cosim/terrain/granular_sph_markers.json";

    double terrain_length = 60;
    double terrain_width = 8;
    ChVector<> init_loc(4.0, 0, 0.25);

    double target_speed = 4.0;
    std::string vehicle_specfile = "Polaris/Polaris.json";
    std::string tire_specfile = "Polaris/Polaris_RigidMeshTire.json";
    std::string engine_specfile = "Polaris/Polaris_EngineSimpleMap.json";
    std::string transmission_specfile = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";

    // Prepare output directory.
    std::string out_dir = GetChronoOutputPath() + "WHEELED_VEHICLE_SPH_COSIM";
    if (rank == 0) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            cout << "Error creating directory " << out_dir << endl;
            MPI_Abort(MPI_COMM_WORLD, 1);
            return 1;
        }
    }
    MPI_Barrier(MPI_COMM_WORLD);

    // Number of simulation steps between miscellaneous events.
    int sim_steps = (int)std::ceil(sim_time / step_size);
    int output_steps = (int)std::ceil(1 / (output_fps * step_size));

    // Initialize co-simulation framework (specify 4 tire nodes).
    cosim::InitializeFramework(4);

    // Peek in spec file and check terrain type
    auto terrain_type = ChVehicleCosimTerrainNodeChrono::GetTypeFromSpecfile(vehicle::GetDataFile(terrain_specfile));
    if (terrain_type != ChVehicleCosimTerrainNodeChrono::Type::GRANULAR_SPH) {
        MPI_Finalize();
        return 1;
    }

    // Create the node (vehicle, terrain, or tire node, depending on rank).
    ChVehicleCosimBaseNode* node = nullptr;

    // VEHICLE node
    if (rank == MBS_NODE_RANK) {
        if (verbose)
            cout << "[Vehicle node] rank = " << rank << " running on: " << procname << endl;

        ChVehicleCosimWheeledVehicleNode* vehicle;
        vehicle = new ChVehicleCosimWheeledVehicleNode(vehicle::GetDataFile(vehicle_specfile),
                                                       vehicle::GetDataFile(engine_specfile),
                                                       vehicle::GetDataFile(transmission_specfile));
        vehicle->SetVerbose(verbose);
        vehicle->SetInitialLocation(init_loc);
        vehicle->SetInitialYaw(0);
        vehicle->SetStepSize(step_size);
        vehicle->SetNumThreads(1);
        vehicle->SetOutDir(out_dir);
        if (verbose)
            cout << "[Vehicle node] output directory: " << vehicle->GetOutDirName() << endl;

        if (renderRT)
            vehicle->EnableRuntimeVisualization(render_fps, writeRT);
        if (renderPP)
            vehicle->EnablePostprocessVisualization(render_fps);
        vehicle->SetCameraPosition(ChVector<>(20, 6, 2));

        node = vehicle;
    }

    // TERRAIN node
    if (rank == TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Terrain node] rank = " << rank << " running on: " << procname << endl;

        auto terrain = new ChVehicleCosimTerrainNodeGranularSPH(vehicle::GetDataFile(terrain_specfile));
        terrain->SetDimensions(terrain_length, terrain_width);
        terrain->SetVerbose(verbose);
        terrain->SetStepSize(step_size);
        terrain->SetOutDir(out_dir);
        if (verbose)
            cout << "[Terrain node] output directory: " << terrain->GetOutDirName() << endl;

        if (renderRT)
            terrain->EnableRuntimeVisualization(render_fps, writeRT);
        if (renderPP)
            terrain->EnablePostprocessVisualization(render_fps);
        terrain->SetCameraPosition(ChVector<>(4, 6, 1.5));

        node = terrain;
    }

    // TIRE nodes
    if (rank > TERRAIN_NODE_RANK) {
        if (verbose)
            cout << "[Tire node   ] rank = " << rank << " running on: " << procname << endl;

        auto tire = new ChVehicleCosimTireNodeRigid(rank - 2, vehicle::GetDataFile(tire_specfile));
        tire->SetVerbose(verbose);
        tire->SetStepSize(step_size);
        tire->SetNumThreads(1);
        tire->SetOutDir(out_dir);
        node = tire;
    }

    // Initialize systems
    // (perform initial inter-node data exchange)
    node->Initialize();

    // Defer creation and initialization of the driver system until after the vehicle is initialized on the MBS node.
    if (rank == MBS_NODE_RANK) {
        auto vehicle = static_cast<ChVehicleCosimWheeledVehicleNode*>(node);
        auto path = CreatePath(vehicle::GetDataFile(path_specfile));
        double x_max = path->getPoint(path->getNumPoints() - 2).x() - 3.0;
        auto driver = chrono_types::make_shared<DriverWrapper>(*vehicle->GetVehicle(), path, "path", target_speed, 0.5);
        driver->GetSteeringController().SetLookAheadDistance(2.0);
        driver->GetSteeringController().SetGains(1.0, 0, 0);
        driver->GetSpeedController().SetGains(0.6, 0.05, 0);
        driver->Initialize();

        vehicle->SetDriver(driver);
    }

    // Perform co-simulation
    // (perform synchronization inter-node data exchange)
    int output_frame = 0;

    double t_start = MPI_Wtime();
    for (int is = 0; is < sim_steps; is++) {
        double time = is * step_size;

        if (verbose && rank == 0)
            cout << is << " ---------------------------- " << endl;
        MPI_Barrier(MPI_COMM_WORLD);

        node->Synchronize(is, time);
        node->Advance(step_size);
        if (verbose)
            cout << "Node" << rank << " sim time = " << node->GetStepExecutionTime() << "  ["
                 << node->GetTotalExecutionTime() << "]" << endl;

        if (output && is % output_steps == 0) {
            node->OutputData(output_frame);
            node->OutputVisualizationData(output_frame);
            output_frame++;
        }
    }
    double t_total = MPI_Wtime() - t_start;

    cout << "Node" << rank << " sim time: " << node->GetTotalExecutionTime() << " total time: " << t_total << endl;

    // Cleanup.
    delete node;
    MPI_Finalize();
    return 0;
}

// =============================================================================

std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file) {
    // Open input file
    std::ifstream ifile(path_file);
    std::string line;

    // Read number of knots and type of curve
    size_t numPoints;
    size_t numCols;

    std::getline(ifile, line);
    std::istringstream iss(line);
    iss >> numPoints >> numCols;

    assert(numCols == 3);

    // Read path points
    std::vector<ChVector<>> points;

    for (size_t i = 0; i < numPoints; i++) {
        double x, y, z;
        std::getline(ifile, line);
        std::istringstream jss(line);
        jss >> x >> y >> z;
        points.push_back(ChVector<>(x, y, z));
    }

    // Include point beyond SPH patch
    {
        auto np = points.size();
        points.push_back(2.0 * points[np - 1] - points[np - 2]);
    }

    // Raise all path points
    for (auto& p : points)
        p.z() += 0.1;

    ifile.close();

    return std::shared_ptr<ChBezierCurve>(new ChBezierCurve(points));
}
