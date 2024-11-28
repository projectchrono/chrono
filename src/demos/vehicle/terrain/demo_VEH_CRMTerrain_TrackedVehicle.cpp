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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/tracked_vehicle/vehicle/TrackedVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "demos/SetChronoSolver.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// ===================================================================================================================

std::shared_ptr<TrackedVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);
void CreateTrackBCEMarkers(std::shared_ptr<TrackedVehicle> vehicle, CRMTerrain& terrain);

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
    double active_box_hdim = 0.4;

    bool render = true;       // use run-time visualization
    double render_fps = 200;  // rendering FPS

    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;  // render wheel BCE markers

    bool chase_cam = true;  // chase-cam or fixed camera

    bool verbose = true;

    // Create the Chrono system and associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    SetChronoSolver(sys, ChSolver::Type::BARZILAIBORWEIN, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Create vehicle
    cout << "Create vehicle..." << endl;
    ChVector3d veh_init_pos(5.0, 0, 0.7);
    auto vehicle = CreateVehicle(sys, ChCoordsys<>(veh_init_pos, QUNIT));

    // Create the CRM terrain system
    double initial_spacing = 0.02;
    CRMTerrain terrain(sys, initial_spacing);
    terrain.SetVerbose(verbose);
    ChFsiSystemSPH& sysFSI = terrain.GetSystemFSI();
    ChFluidSystemSPH& sysSPH = sysFSI.GetFluidSystemSPH();

    // Set SPH parameters and soil material properties
    const ChVector3d gravity(0, 0, -9.81);
    sysFSI.SetGravitationalAcceleration(gravity);
    sys.SetGravitationalAcceleration(gravity);

    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);

    ChFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = cohesion;

    sysSPH.SetElasticSPH(mat_props);

    ChFluidSystemSPH::SPHParameters sph_params;
    sph_params.sph_method = SPHMethod::WCSPH;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1;
    sph_params.kernel_threshold = 0.8;
    sph_params.artificial_viscosity = 0.5;
    sph_params.consistent_gradient_discretization = false;
    sph_params.consistent_laplacian_discretization = false;
    sph_params.viscosity_type = ViscosityType::ARTIFICIAL_BILATERAL;
    sph_params.boundary_type = BoundaryType::HOLMES;
    sysSPH.SetSPHParameters(sph_params);

    sysSPH.SetActiveDomain(ChVector3d(active_box_hdim));

    sysSPH.SetOutputLevel(OutputLevel::STATE);

    // Add track shoes as FSI bodies
    CreateTrackBCEMarkers(vehicle, terrain);

    // Construct the terrain using SPH particles and BCE markers from files
    cout << "Create terrain..." << endl;
    terrain.Construct(vehicle::GetDataFile(terrain_dir + "/sph_particles.txt"),
                      vehicle::GetDataFile(terrain_dir + "/bce_markers.txt"), VNULL);

    // Initialize the terrain system
    terrain.Initialize();

    const auto& aabb = terrain.GetSPHBoundingBox();
    cout << "  SPH particles:     " << sysSPH.GetNumFluidMarkers() << endl;
    cout << "  Bndry BCE markers: " << sysSPH.GetNumBoundaryMarkers() << endl;
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
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif
#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        visFSI->SetTitle("Tracked vehicle on CRM deformable terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(0, 8, 0.5), ChVector3d(0, -1, 0));
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f),
                                                                                           aabb.min.z(), aabb.max.z()));
        visFSI->AttachSystem(&sys);
        visFSI->Initialize();
    }

    // Simulation loop
    DriverInputs driver_inputs = {0, 0, 0};
    double time = 0;
    int sim_frame = 0;
    int render_frame = 0;

    double timer_CFD = 0;
    double timer_MBS = 0;
    double timer_FSI = 0;
    double timer_step = 0;

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
                visFSI->UpdateCamera(cam_loc, cam_point);
            }
            if (!visFSI->Render())
                break;
            render_frame++;
        }
        if (!render) {
            cout << sysFSI.GetSimTime() << "  " << sysFSI.GetRtf() << endl;
        }

        // Synchronize sy^stems
        driver.Synchronize(time);
        vehicle->Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);

        // Advance system state
        driver.Advance(step_size);
        sysFSI.DoStepDynamics(step_size);

        timer_CFD += sysFSI.GetTimerCFD();
        timer_MBS += sysFSI.GetTimerMBS();
        timer_FSI += sysFSI.GetTimerFSI();
        timer_step += sysFSI.GetTimerStep();
        if (sim_frame == 2000) {
            cout << "Cummulative timers at time: " << time << endl;
            cout << "   timer CFD:  " << timer_CFD << endl;
            cout << "   timer MBS:  " << timer_MBS << endl;
            cout << "   timer FSI:  " << timer_FSI << endl;
            cout << "   timer step: " << timer_step << endl;
        }

        time += step_size;
        sim_frame++;
    }

    return 0;
}

// ===================================================================================================================

std::shared_ptr<TrackedVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos) {
    std::string vehicle_json = "M113/vehicle/M113_Vehicle_SinglePin.json";
    std::string engine_json = "M113/powertrain/M113_EngineSimple.json";
    std::string transmission_json = "M113/powertrain/M113_AutomaticTransmissionSimpleMap.json";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<TrackedVehicle>(&sys, vehicle::GetDataFile(vehicle_json));
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

void CreateTrackBCEMarkers(std::shared_ptr<TrackedVehicle> vehicle, CRMTerrain& terrain) {
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
