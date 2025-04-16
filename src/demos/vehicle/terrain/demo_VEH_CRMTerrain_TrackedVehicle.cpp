// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Polaris wheeled vehicle on CRM terrain (initialized from particle data files)
//
// =============================================================================

#include <cstdio>
#include <string>
#include <stdexcept>
#include <iomanip>
#include <thread>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

std::shared_ptr<TrackedVehicle> CreateVehicle(const ChCoordsys<>& init_pos);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);
void CreateFSITracks(std::shared_ptr<TrackedVehicle> vehicle, CRMTerrain& terrain);

// ===================================================================================================================

int main(int argc, char* argv[]) {
    // Set model and simulation parameters
    std::string terrain_dir = "terrain/sph/S-lane_RMS";

    double density = 1700;
    double cohesion = 5e3;
    double friction = 0.8;
    double youngs_modulus = 1e6;
    double poisson_ratio = 0.3;

    double target_speed = 7.0;
    double tend = 30;
    double step_size = 5e-4;
    ChVector3d active_box_dim(0.3, 0.4, 0.4);

    bool render = true;       // use run-time visualization
    double render_fps = 200;  // rendering FPS

    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;  // render wheel BCE markers

    bool chase_cam = true;  // chase-cam or fixed camera

    bool verbose = true;

    // Set SPH spacing
    double initial_spacing = 0.02;

    // Create vehicle
    cout << "Create vehicle..." << endl;
    ChVector3d veh_init_pos(5.0, 0, 0.7);
    auto vehicle = CreateVehicle(ChCoordsys<>(veh_init_pos, QUNIT));
    auto sysMBS = vehicle->GetSystem();
    SetChronoSolver(*sysMBS, ChSolver::Type::BARZILAIBORWEIN, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Set collision system
    sysMBS->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the CRM terrain system
    CRMTerrain terrain(*sysMBS, initial_spacing);
    ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();
    terrain.SetVerbose(verbose);
    terrain.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    terrain.SetStepSizeCFD(step_size);

    // Register the vehicle with the CRM terrain
    terrain.RegisterVehicle(vehicle.get());

    // Set SPH parameters and soil material properties
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = cohesion;
    terrain.SetElasticSPH(mat_props);

    // Set SPH solver parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    sph_params.boundary_type = BoundaryType::HOLMES;
    terrain.SetSPHParameters(sph_params);

    // Set output level from SPH simulation
    terrain.SetOutputLevel(OutputLevel::STATE);

    // Add track shoes as FSI bodies
    CreateFSITracks(vehicle, terrain);
    terrain.SetActiveDomain(active_box_dim);

    cout << "Create terrain..." << endl;
    // Construct flat rectangular CRM terrain
    ////terrain.Construct(ChVector3d(10, 4, 0.5), ChVector3d(5, 0, -0.5), true);
    // Construct the terrain using SPH particles and BCE markers from files
    terrain.Construct(vehicle::GetDataFile(terrain_dir + "/sph_particles.txt"),
                      vehicle::GetDataFile(terrain_dir + "/bce_markers.txt"), VNULL);

    // Initialize the terrain system
    terrain.Initialize();

    const auto& aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << terrain.GetNumSPHParticles() << endl;
    cout << "  Bndry BCE markers: " << terrain.GetNumBoundaryBCEMarkers() << endl;
    cout << "  SPH AABB:          " << aabb.min << "   " << aabb.max << endl;

    // Create driver
    cout << "Create path..." << endl;
    auto path = CreatePath(terrain_dir + "/path.txt");
    double x_max = path->GetPoint(path->GetNumPoints() - 2).x() - 3.0;
    ChPathFollowerDriver driver(*vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(2.0);
    driver.GetSteeringController().SetGains(1.0, 0, 0);
    driver.GetSpeedController().SetGains(0.6, 0.05, 0);
    driver.Initialize();

    // Create run-time visualization
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto col_callback = chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f),
                                                                                    aabb.min.z(), aabb.max.z());

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetSPHColorCallback(col_callback);

        // Tracked vehicle VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<ChTrackedVehicleVisualSystemVSG>();
        visVSG->AttachVehicle(vehicle.get());
        visVSG->AttachPlugin(visFSI);
        visVSG->SetWindowTitle("Tracked vehicle on CRM deformable terrain");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, 8, 0.5), ChVector3d(0, -1, 0));
        visVSG->SetLightIntensity(0.9f);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Simulation loop
    DriverInputs driver_inputs = {0, 0, 0};
    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;

    if (x_max < veh_init_pos.x())
        x_max = veh_init_pos.x() + 0.25;

    cout << "Start simulation..." << endl;
    TerrainForces shoe_forces_left(vehicle->GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(vehicle->GetNumTrackShoes(RIGHT));

    while (time < tend) {
        const auto& veh_loc = vehicle->GetPos();

        // Stop before end of patch
        if (veh_loc.x() > x_max)
            break;

        // Set current driver inputs
        driver_inputs = driver.GetInputs();

        if (time < 0.5) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            ChClampValue(driver_inputs.m_throttle, driver_inputs.m_throttle, (time - 0.5) / 0.5);
        }

        // Run-time visualization
        if (render && time >= render_frame / render_fps) {
            if (chase_cam) {
                ChVector3d cam_loc = veh_loc + ChVector3d(-6, 6, 0.5);
                ChVector3d cam_point = veh_loc;
                vis->UpdateCamera(cam_loc, cam_point);
            }
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }
        if (!render) {
            std::cout << time << "  " << terrain.GetRtfCFD() << "  " << terrain.GetRtfMBD() << std::endl;
        }

        // Synchronize systems
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance system state
        // Note: CRMTerrain::Advance also performs the vehicle dynamics
        driver.Advance(step_size);
        terrain.Advance(step_size);

        time += step_size;
        sim_frame++;
    }

    return 0;
}

// ===================================================================================================================

std::shared_ptr<TrackedVehicle> CreateVehicle(const ChCoordsys<>& init_pos) {
    std::string vehicle_json = "M113/vehicle/M113_Vehicle_SinglePin.json";
    std::string engine_json = "M113/powertrain/M113_EngineSimple.json";
    std::string transmission_json = "M113/powertrain/M113_AutomaticTransmissionSimpleMap.json";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<TrackedVehicle>(vehicle::GetDataFile(vehicle_json), ChContactMethod::NSC);
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);

    vehicle->SetChassisVisualizationType(VisualizationType::NONE);
    vehicle->SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetRollerVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    return vehicle;
}

std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file) {
    // Open input file
    std::ifstream ifile(vehicle::GetDataFile(path_file));
    std::string line;

    // Read number of knots and type of curve
    size_t numPoints;
    size_t numCols;

    std::getline(ifile, line);
    std::istringstream iss(line);
    iss >> numPoints >> numCols;

    assert(numCols == 3);

    // Read path points
    std::vector<ChVector3d> points;

    for (size_t i = 0; i < numPoints; i++) {
        double x, y, z;
        std::getline(ifile, line);
        std::istringstream jss(line);
        jss >> x >> y >> z;
        points.push_back(ChVector3d(x, y, z));
    }

    // Include point beyond CRM patch
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

void CreateFSITracks(std::shared_ptr<TrackedVehicle> vehicle, CRMTerrain& terrain) {
    auto& sysFSI = terrain.GetSystemFSI();
    auto& sysSPH = sysFSI.GetFluidSystemSPH();

    // GetCollision shapes for a track shoe (will use only collision boxes)
    auto track_geometry = vehicle->GetTrackShoe(VehicleSide::LEFT, 0)->GetGroundContactGeometry();

    // Consider only collision boxes that are large enough
    utils::ChBodyGeometry geometry;
    auto min_length = 2 * (sysSPH.GetNumBCELayers() - 1) * sysSPH.GetInitialSpacing();
    for (const auto& box : track_geometry.coll_boxes) {
        if (box.dims.x() > min_length && box.dims.y() > min_length && box.dims.z() < min_length) {
            geometry.coll_boxes.push_back(utils::ChBodyGeometry::BoxShape(box.pos, box.rot, box.dims));
        }
    }

    cout << "Consider " << geometry.coll_boxes.size() << " collision boxes out of " << track_geometry.coll_boxes.size()
         << endl;

    // Add an FSI body and associated BCE markers for each track shoe
    size_t num_track_BCE = 0;

    auto nshoes_left = vehicle->GetNumTrackShoes(VehicleSide::LEFT);
    for (size_t i = 0; i < nshoes_left; i++) {
        auto shoe_body = vehicle->GetTrackShoe(VehicleSide::LEFT, i)->GetShoeBody();
        num_track_BCE += terrain.AddRigidBody(shoe_body, geometry, false);
    }

    auto nshoes_right = vehicle->GetNumTrackShoes(VehicleSide::RIGHT);
    for (size_t i = 0; i < nshoes_right; i++) {
        auto shoe_body = vehicle->GetTrackShoe(VehicleSide::RIGHT, i)->GetShoeBody();
        num_track_BCE += terrain.AddRigidBody(shoe_body, geometry, false);
    }

    cout << "Added " << num_track_BCE << " BCE markers on track shoes" << endl;
}
