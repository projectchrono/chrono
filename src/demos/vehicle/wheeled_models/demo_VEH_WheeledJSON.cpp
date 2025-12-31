// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
// Main driver function for a vehicle specified through JSON files.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/vehicle/WheeledVehicleJSON.h"
#include "demos/SetChronoSolver.h"

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Trailer model selection (use only with HMMWV, Sedan, or UAZ)
bool add_trailer = false;
auto trailer_model = UT_Model();

// JSON files for terrain
std::string rigidterrain_file("terrain/RigidPlane.json");
////std::string rigidterrain_file("terrain/RigidMesh.json");
////std::string rigidterrain_file("terrain/RigidHeightMap.json");
////std::string rigidterrain_file("terrain/RigidSlope10.json");
////std::string rigidterrain_file("terrain/RigidSlope20.json");

// Initial vehicle position and orientation (adjust for selected terrain)
ChVector3d initLoc(0, 0, 0.5);
double initYaw = 20 * CH_DEG_TO_RAD;

// Contact method
ChContactMethod contact_method = ChContactMethod::SMC;

// Render frequency
double render_fps = 50;

// End time (used only if no run-time visualization)
double t_end = 20;

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Select vehicle model (see WheeledVehicleJSON.h)
    auto models = WheeledVehicleJSON::List();

    int num_models = (int)models.size();
    int which = 0;
    std::cout << "Options:\n";
    for (int i = 0; i < num_models; i++)
        std::cout << i + 1 << "  " << models[i].second << std::endl;
    std::cout << "\nSelect vehicle: ";
    std::cin >> which;
    std::cout << std::endl;
    ChClampValue(which, 1, num_models);

    const auto& vehicle_model = models[which - 1].first;

    // Create the vehicle system
    WheeledVehicle vehicle(GetVehicleDataFile(vehicle_model->VehicleJSON()), contact_method);
    vehicle.Initialize(ChCoordsys<>(initLoc, QuatFromAngleZ(initYaw)));
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetChassisRearVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSubchassisVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto engine = ReadEngineJSON(GetVehicleDataFile(vehicle_model->EngineJSON()));
    auto transmission = ReadTransmissionJSON(GetVehicleDataFile(vehicle_model->TransmissionJSON()));
    auto powertrain = chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (unsigned int i = 0; i < vehicle.GetNumberAxles(); i++) {
        for (auto& wheel : vehicle.GetAxle(i)->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile(vehicle_model->TireJSON(i)));
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    // Containing system
    auto sys = vehicle.GetSystem();

    // Create the trailer system (build into same ChSystem)
    std::shared_ptr<WheeledTrailer> trailer;
    if (add_trailer) {
        trailer = chrono_types::make_shared<WheeledTrailer>(sys, GetVehicleDataFile(trailer_model.TrailerJSON()));
        trailer->Initialize(vehicle.GetChassis());
        trailer->SetChassisVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
        trailer->SetWheelVisualizationType(VisualizationType::NONE);
        for (auto& axle : trailer->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile(trailer_model.TireJSON()));
                trailer->InitializeTire(tire, wheel, VisualizationType::PRIMITIVES);
            }
        }
    }

    // Associate a collision system
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the terrain
    RigidTerrain terrain(sys, GetVehicleDataFile(rigidterrain_file));
    terrain.Initialize();

    // Set solver and integrator
    double step_size = 2e-3;
    auto solver_type = ChSolver::Type::BARZILAIBORWEIN;
    auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    if (vehicle.HasBushings()) {
        solver_type = ChSolver::Type::MINRES;
        step_size = 2e-4;
    }
    SetChronoSolver(*sys, solver_type, integrator_type);

    // Create the interactive VSG driver system
    ChInteractiveDriver driver(vehicle);
    driver.SetSteeringDelta(0.02);
    driver.SetThrottleDelta(0.02);
    driver.SetBrakingDelta(0.06);
    driver.Initialize();

    // Create the vehicle run-time visualization interface and the interactive driver
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::string title = "Vehicle demo - JSON specification - " + vehicle_model->ModelName();
    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle(title);
            vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), vehicle_model->CameraDistance(), 0.5);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&vehicle);
            vis_irr->AttachDriver(&driver);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle(title);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->AttachDriver(&driver);
            vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, 1.75), vehicle_model->CameraDistance(), 0.5);
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->EnableSkyBox();
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Initialize output directories
    const std::string out_dir = GetChronoOutputPath() + "WHEELED_JSON";
    const std::string veh_dir = out_dir + "/" + vehicle_model->ModelName();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(veh_dir))) {
        std::cout << "Error creating directory " << veh_dir << std::endl;
        return 1;
    }

    // Generate JSON information with available output channels
    std::string out_json = vehicle.ExportComponentList();
    std::cout << out_json << std::endl;
    vehicle.ExportComponentList(veh_dir + "/component_list.json");

    vehicle.LogSubsystemTypes();

    // Optionally, enable output from selected vehicle subsystems
    ////vehicle.SetSuspensionOutput(0, true);
    ////vehicle.SetSuspensionOutput(1, true);
    ////vehicle.SetOutput(ChOutput::Type::ASCII, ChOutput::Mode::FRAMES, veh_dir, "output", 0.1);

    // Simulation loop
    vehicle.EnableRealtime(true);

    int sim_frame = 0;
    int render_frame = 0;
    while (true) {
        double time = sys->GetChTime();

        if (vis) {
            if (!vis->Run())
                break;

            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();

                render_frame++;
            }
        } else if (time > t_end) {
            break;
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        if (add_trailer)
            trailer->Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        if (add_trailer)
            trailer->Advance(step_size);
        terrain.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        sim_frame++;
    }

    return 0;
}
