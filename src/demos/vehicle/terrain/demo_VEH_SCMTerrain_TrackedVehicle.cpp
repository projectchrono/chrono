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
// Demonstration program for M113 vehicle on SCM terrain.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_models/vehicle/m113/M113.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/driver/ChInteractiveDriverVSG.h"
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle position
ChVector<> initLoc(-5, 0, 1.1);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Terrain dimensions
double terrain_length = 20.0;  // size in X direction
double terrain_width = 4.0;    // size in Y direction
double delta = 0.05;          // SCM grid spacing

// Simulation step size
double step_size = 5e-4;

// Use PardisoMKL
bool use_mkl = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(-2.0, 0.0, 0.0);

// Output directories
const std::string out_dir = GetChronoOutputPath() + "M113_DEF_SOIL";
const std::string img_dir = out_dir + "/IMG";

// Visualization output
bool img_output = false;

// =============================================================================

// Simple powertrain model
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// =============================================================================

// Forward declarations
void AddFixedObstacles(ChSystem* system);
void AddMovingObstacles(ChSystem* system);

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    M113 m113;
    m113.SetContactMethod(ChContactMethod::SMC);
    m113.SetTrackShoeType(TrackShoeType::SINGLE_PIN);
    m113.SetBrakeType(BrakeType::SIMPLE);
    m113.SetDrivelineType(DrivelineTypeTV::BDS);
    m113.SetEngineType(EngineModelType::SHAFTS);
    m113.SetTransmissionType(TransmissionModelType::SHAFTS);
    m113.SetChassisCollisionType(CollisionType::NONE);

    // Control steering type (enable crossdrive capability)
    ////vehicle.GetDriveline()->SetGyrationMode(true);

    // Initialize the vehicle at the specified position
    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();

    // Set visualization type for vehicle components.
    m113.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetSprocketVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetIdlerVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetIdlerWheelVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetRoadWheelVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Control internal collisions and contact monitoring.

    // Disable contact for all tracked vehicle parts
    ////m113.GetVehicle().SetCollide(TrackedCollisionFlag::NONE);

    // Monitor internal contacts for the left sprocket, left idler, and first shoe on the left track.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Collect contact information
    ////m113.GetVehicle().SetContactCollection(true);

    ChSystem* system = m113.GetSystem();

    // ----------------------
    // Create the SCM terrain
    // ----------------------

    SCMTerrain terrain(system);
    terrain.SetSoilParameters(2e7,   // Bekker Kphi
                              0,     // Bekker Kc
                              1.1,   // Bekker n exponent
                              0,     // Mohr cohesive limit (Pa)
                              20,    // Mohr friction limit (degrees)
                              0.01,  // Janosi shear coefficient (m)
                              2e8,   // Elastic stiffness (Pa/m), before plastic yield
                              3e4    // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );

    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    ////terrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);

    terrain.Initialize(terrain_length, terrain_width, delta);

    // Add obstacles
    AddFixedObstacles(system);
    ////AddMovingObstacles(system);

    // ------------------------------------------------------------------------------
    // Create the vehicle run-time visualization interface and the interactive driver
    // ------------------------------------------------------------------------------

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1

    std::shared_ptr<ChVehicleVisualSystem> vis;
    std::shared_ptr<ChDriver> driver;

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Tracked vehicle on SCM deformable terrain");
            vis_irr->SetChaseCamera(trackPoint, 4.0, 1.0);
            vis_irr->SetChaseCameraPosition(ChVector<>(-3, 4, 1.5));
            vis_irr->SetChaseCameraMultipliers(1e-4, 10);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional();
            vis_irr->AddSkyBox();
            vis_irr->AddLogo();
            vis_irr->AttachVehicle(&m113.GetVehicle());

            // Create the interactive Irrlicht driver system
            auto driver_irr = chrono_types::make_shared<ChInteractiveDriverIRR>(*vis_irr);
            driver_irr->SetSteeringDelta(render_step_size / steering_time);
            driver_irr->SetThrottleDelta(render_step_size / throttle_time);
            driver_irr->SetBrakingDelta(render_step_size / braking_time);
            driver_irr->SetInputDataFile(vehicle::GetDataFile("M113/driver/Acceleration.txt"));
            driver_irr->SetInputMode(ChInteractiveDriverIRR::InputMode::DATAFILE);
            driver_irr->Initialize();

            vis = vis_irr;
            driver = driver_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChTrackedVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Tracked vehicle on SCM deformable terrain");
            vis_vsg->SetWindowSize(ChVector2<int>(1000, 800));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 100));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraAngleDeg(40);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetChaseCamera(trackPoint, 7.0, 2.0);
            vis_vsg->SetChaseCameraPosition(ChVector<>(-3, 4, 1.5));
            vis_vsg->SetChaseCameraMultipliers(1e-4, 10);
            vis_vsg->AttachVehicle(&m113.GetVehicle());
            vis_vsg->AddGuiColorbar("Sinkage (m)", 0.0, 0.1);
            vis_vsg->Initialize();

            // Create the interactive VSG driver system
            auto driver_vsg = chrono_types::make_shared<ChInteractiveDriverVSG>(*vis_vsg);
            driver_vsg->SetSteeringDelta(render_step_size / steering_time);
            driver_vsg->SetThrottleDelta(render_step_size / throttle_time);
            driver_vsg->SetBrakingDelta(render_step_size / braking_time);
            driver_vsg->SetInputDataFile(vehicle::GetDataFile("M113/driver/Acceleration.txt"));
            driver_vsg->SetInputMode(ChInteractiveDriverVSG::InputMode::DATAFILE);
            driver_vsg->Initialize();

            vis = vis_vsg;
            driver = driver_vsg;
#endif
            break;
        }
    }

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // -------------------
    // Simulation settings
    // -------------------

    system->SetNumThreads(std::min(8, ChOMP::GetNumProcs()));

#ifndef CHRONO_PARDISO_MKL
    // Do not use PardisoMKL if not available
    use_mkl = false;
#endif

    // Solver and integrator settings
    if (use_mkl) {
#ifdef CHRONO_PARDISO_MKL
        GetLog() << "Using PardisoMKL solver\n";
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        system->SetSolver(mkl_solver);

        system->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(system->GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxiters(50);
        integrator->SetAbsTolerances(5e-05, 1.8e00);
        integrator->SetMode(ChTimestepperHHT::POSITION);
        integrator->SetModifiedNewton(false);
        integrator->SetScaling(true);
        integrator->SetVerbose(true);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverBB>();
        solver->SetMaxIterations(120);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
        m113.GetSystem()->SetSolver(solver);

        m113.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
        m113.GetSystem()->SetMinBounceSpeed(2.0);
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(m113.GetVehicle().GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(m113.GetVehicle().GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(m113.GetVehicle().GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(m113.GetVehicle().GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    // Execution time
    double total_timing = 0;

    while (vis->Run()) {
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (img_output) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                vis->WriteImageToFile(filename);
                render_frame++;
            }
        }

        // Collect output data from modules
        DriverInputs driver_inputs = driver->GetInputs();
        m113.GetVehicle().GetTrackShoeStates(LEFT, shoe_states_left);
        m113.GetVehicle().GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = m113.GetVehicle().GetChTime();
        driver->Synchronize(time);
        terrain.Synchronize(time);
        m113.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        m113.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        // Execution time
        double step_timing = system->GetTimerStep();
        total_timing += step_timing;
        ////std::cout << step_number << " " << step_timing << " " << total_timing << std::endl;
    }

    m113.GetVehicle().WriteContacts("M113_contacts.out");

    return 0;
}

// =============================================================================
void AddFixedObstacles(ChSystem* system) {
    double radius = 2;
    double length = 10;

    auto obstacle = std::shared_ptr<ChBody>(system->NewBody());
    obstacle->SetPos(ChVector<>(0, 0, -1.8));
    obstacle->SetBodyFixed(true);
    obstacle->SetCollide(true);

    // Visualization
    auto cyl_shape = chrono_types::make_shared<ChCylinderShape>(radius, length);
    cyl_shape->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"));
    obstacle->AddVisualShape(cyl_shape, ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)));

    auto box_shape = chrono_types::make_shared<ChBoxShape>(terrain_length, 2*length, 0.1);
    box_shape->SetColor(ChColor(0.2f, 0.2f, 0.2f));
    obstacle->AddVisualShape(box_shape, ChFrame<>(ChVector<>(0, 0, 1.5), QUNIT));

    // Contact
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    obstacle->GetCollisionModel()->ClearModel();
    obstacle->GetCollisionModel()->AddCylinder(obst_mat, radius, length, VNULL, Q_from_AngX(CH_C_PI_2));
    obstacle->GetCollisionModel()->BuildModel();

    system->AddBody(obstacle);
}

void AddMovingObstacles(ChSystem* system) {
    double radius = 0.2;
    double mass = 100;
    ChVector<> pos(-4, 0, 0.6);
    ChQuaternion<> rot(1, 0, 0, 0);
    ChVector<> init_vel(0, 0, 0);
    ChVector<> init_ang_vel(0, 30, 0);

    // Create a material
    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    material->SetRestitution(0.1f);
    material->SetFriction(0.4f);

    // Create a ball
    auto ball = std::shared_ptr<ChBody>(system->NewBody());

    ball->SetMass(mass);
    ball->SetPos(pos);
    ball->SetRot(rot);
    ball->SetPos_dt(init_vel);
    ball->SetWvel_loc(init_ang_vel);
    ball->SetBodyFixed(false);
    ball->SetCollide(true);

    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(material, radius);
    ball->GetCollisionModel()->BuildModel();

    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));

    auto sphere = chrono_types::make_shared<ChSphereShape>(radius);
    sphere->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddVisualShape(sphere);

    system->AddBody(ball);
}
