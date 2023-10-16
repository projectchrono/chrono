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

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::vehicle;

using std::cout;
using std::cin;
using std::endl;

// ===================================================================================================================

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// CRM terrain patch type
enum class PatchType {RECTANGULAR, MARKER_DATA, HEIGHT_MAP};
PatchType patch_type = PatchType::HEIGHT_MAP;

// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos);
std::shared_ptr<ChBezierCurve> CreatePath(const std::string& path_file);
void CreateWheelBCEMarkers(std::shared_ptr<WheeledVehicle> vehicle, ChSystemFsi& sysFSI);

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

    bool visualization = true;             // run-time visualization
    double visualizationFPS = 0;           // frames rendered per second (0: every frame)
    bool visualization_sph = true;         // render SPH particles
    bool visualization_bndry_bce = false;  // render boundary BCE markers
    bool visualization_rigid_bce = false;  // render wheel BCE markers
    bool chase_cam = true;                 // chase-cam or fixed camera

    bool verbose = true;

    // Set SPH spacing
    double spacing = (patch_type == PatchType::MARKER_DATA) ? 0.02 : 0.04;

    // Create the Chrono system
    ChSystemNSC sys;

    // Create the CRM terrain system
    CRMTerrain terrain(sys, spacing);
    terrain.SetVerbose(verbose);
    ChSystemFsi& sysFSI = terrain.GetSystemFSI();

    // Set SPH parameters and soil material properties
    const ChVector<> gravity(0, 0, -9.81);
    sysFSI.Set_G_acc(gravity);
    sys.Set_G_acc(gravity);

    ChSystemFsi::ElasticMaterialProperties mat_props;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = poisson_ratio;
    mat_props.stress = 0;  // default
    mat_props.viscosity_alpha = 0.5;
    mat_props.viscosity_beta = 0.0;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = friction;
    mat_props.mu_fric_2 = friction;
    mat_props.average_diam = 0.005;
    mat_props.friction_angle = CH_C_PI / 10;  // default
    mat_props.dilation_angle = CH_C_PI / 10;  // default
    mat_props.cohesion_coeff = 0;             // default
    mat_props.kernel_threshold = 0.8;

    sysFSI.SetElasticSPH(mat_props);
    sysFSI.SetDensity(density);
    sysFSI.SetCohesionForce(cohesion);

    sysFSI.SetActiveDomain(ChVector<>(active_box_hdim));
    sysFSI.SetDiscreType(false, false);
    sysFSI.SetWallBC(BceVersion::ORIGINAL);
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);
    sysFSI.SetStepSize(step_size);

    sysFSI.SetOutputLength(0);

    // Construct the terrain
    cout << "Create terrain..." << endl;
    switch (patch_type) {
        case PatchType::RECTANGULAR:
            // Create a rectangular terrain patch
            terrain.Construct(10.0, 3.0,            // length X width
                              0.25,                 // depth
                              3,                    // number BCE layers
                              ChVector<>(5, 0, 0),  // patch center
                              0.0,                  // patch orientation angle
                              true                  // create side boundaries
            );
            break;
        case PatchType::MARKER_DATA:
            // Create a patch using SPH particles and BCE markers from files
            terrain.Construct(vehicle::GetDataFile(terrain_dir + "/sph_particles.txt"),  // SPH marker locations
                              vehicle::GetDataFile(terrain_dir + "/bce_markers.txt")     // BCE marker locations
            );
            break;
        case PatchType::HEIGHT_MAP:
            // Create a patch from a heigh field map image
            terrain.Construct(vehicle::GetDataFile("terrain/height_maps/bump64.bmp"),  // height map image file
                              10.0, 3.0,                                               // length (X) and width (Y)
                              {0, 0.3},                                                // height range
                              0.25,                                                    // depth
                              3,                                                       // number of BCE layers
                              ChVector<>(5, 0, 0),                                     // patch center
                              0.0,                                                     // patch yaw rotation
                              false                                                    // side walls?
            );
            break;
    }

    // Create vehicle
    cout << "Create vehicle..." << endl;
    ChVector<> veh_init_pos(3.0, 0, 0.25);
    auto vehicle = CreateVehicle(sys, ChCoordsys<>(veh_init_pos, QUNIT));

    // Create the wheel BCE markers
    CreateWheelBCEMarkers(vehicle, sysFSI);

    // Initialize the terrain system
    terrain.Initialize();

    ChVector<> aabb_min, aabb_max;
    terrain.GetAABB(aabb_min, aabb_max);
    cout << "  SPH particles:     " << sysFSI.GetNumFluidMarkers() << endl;
    cout << "  Bndry BCE markers: " << sysFSI.GetNumBoundaryMarkers() << endl;
    cout << "  AABB:              " << aabb_min << "   " << aabb_max << endl;

    // Set maximum vehicle X location (based on CRM patch size) 
    double x_max = aabb_max.x() - 3.0;

    // Create driver
    cout << "Create path..." << endl;
    auto path = CreatePath(terrain_dir + "/path.txt");
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

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (visualization) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI, verbose);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI, verbose);
#endif
                break;
            }
        }

        visFSI->SetTitle("Wheeled vehicle on CRM deformable terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector<>(0, 8, 1.5), ChVector<>(0, -1, 0));
        visFSI->SetCameraMoveScale(0.2f);
        visFSI->EnableFluidMarkers(visualization_sph);
        visFSI->EnableBoundaryMarkers(visualization_bndry_bce);
        visFSI->EnableRigidBodyMarkers(visualization_rigid_bce);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<HeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), aabb_min.z(), aabb_max.z()));
        visFSI->AttachSystem(&sys);
        visFSI->Initialize();
    }

    // Simulation loop
    DriverInputs driver_inputs = {0, 0, 0};
    int render_steps = (visualizationFPS > 0) ? (int)std::round((1.0 / visualizationFPS) / step_size) : 1;
    double t = 0;
    int frame = 0;

    cout << "Start simulation..." << endl;

    while (t < tend) {
        const auto& veh_loc = vehicle->GetPos();

        // Set current driver inputs
        driver_inputs = driver.GetInputs();

        // Ramp up throttle to value requested by the cruise controller
        if (t < 0.5) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        } else {
            ChClampValue(driver_inputs.m_throttle, driver_inputs.m_throttle, (t - 0.5) / 0.5);
        }

        // Stop vehicle before reaching end of terrain patch
        if (veh_loc.x() > x_max) {
            driver_inputs.m_throttle = 0;
            driver_inputs.m_braking = 1;
        }

        // Run-time visualization
        if (visualization && frame % render_steps == 0) {
            if (chase_cam) {
                ChVector<> cam_loc = veh_loc + ChVector<>(-6, 6, 1.5);
                ChVector<> cam_point = veh_loc;
                visFSI->UpdateCamera(cam_loc, cam_point);
            }
            if (!visFSI->Render())
                break;
        }
        if (!visualization) {
            std::cout << sysFSI.GetSimTime() << "  " << sysFSI.GetRTF() << std::endl;
        }

        // Synchronize systems
        driver.Synchronize(t);
        vehicle->Synchronize(t, driver_inputs, terrain);

        // Advance system state
        driver.Advance(step_size);
        sysFSI.DoStepDynamics_FSI();
        t += step_size;

        frame++;
    }

    return 0;
}

// ===================================================================================================================

std::shared_ptr<WheeledVehicle> CreateVehicle(ChSystem& sys, const ChCoordsys<>& init_pos) {
    std::string vehicle_json = "Polaris/Polaris.json";
    std::string engine_json = "Polaris/Polaris_EngineSimpleMap.json";
    std::string transmission_json = "Polaris/Polaris_AutomaticTransmissionSimpleMap.json";
    std::string tire_json = "Polaris/Polaris_RigidTire.json";

    // Create and initialize the vehicle
    auto vehicle = chrono_types::make_shared<WheeledVehicle>(&sys, vehicle::GetDataFile(vehicle_json));
    vehicle->Initialize(init_pos);
    vehicle->GetChassis()->SetFixed(false);
    vehicle->SetChassisVisualizationType(VisualizationType::MESH);
    vehicle->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle->SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(vehicle::GetDataFile(engine_json));
    auto transmission = ReadTransmissionJSON(vehicle::GetDataFile(transmission_json));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle->InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(vehicle::GetDataFile(tire_json));
            vehicle->InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

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
    std::vector<ChVector<>> points;

    for (size_t i = 0; i < numPoints; i++) {
        double x, y, z;
        std::getline(ifile, line);
        std::istringstream jss(line);
        jss >> x >> y >> z;
        points.push_back(ChVector<>(x, y, z));
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

void CreateWheelBCEMarkers(std::shared_ptr<WheeledVehicle> vehicle, ChSystemFsi& sysFSI) {
    // Create BCE markers for a tire
    std::string tire_coll_obj = "Polaris/meshes/Polaris_tire_collision.obj";

    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(vehicle::GetDataFile(tire_coll_obj));
    std::vector<ChVector<>> point_cloud;
    sysFSI.CreateMeshPoints(trimesh, sysFSI.GetInitialSpacing(), point_cloud);

    // Create and initialize the tires
    for (auto& axle : vehicle->GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            sysFSI.AddFsiBody(wheel->GetSpindle());
            sysFSI.AddPointsBCE(wheel->GetSpindle(), point_cloud, ChFrame<>(), true);
        }
    }
}
