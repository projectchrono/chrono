// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
//
// =============================================================================

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/DeformableTerrain.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

#include "models/vehicle/m113/M113_SimplePowertrain.h"
#include "models/vehicle/m113/M113_Vehicle.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace m113;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Initial vehicle position
ChVector<> initLoc(-5, 0, 1.1);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Terrain dimensions
double terrainHeight = 0;
double terrainLength = 20.0;  // size in X direction
double terrainWidth = 4.0;    // size in Y direction
int divLength = 640;
int divWidth = 128;

// Simulation step size
double step_size = 1e-2;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(-2.0, 0.0, 0.0);

// Output directories
const std::string out_dir = "../M113_DEF_SOIL";
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
    // --------------------------
    // Construct the M113 vehicle
    // --------------------------
    M113_Vehicle vehicle(false, ChMaterialSurfaceBase::DEM);

    // Set visualization type for vehicle components (default: PRIMITIVES).
    ////vehicle.SetRoadWheelVisType(NONE);
    ////vehicle.SetTrackShoeVisType(NONE);

    // Control steering type (enable crossdrive capability).
    ////vehicle.GetDriveline()->SetGyrationMode(true);

    // Solver settings.
    ////vehicle.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
    vehicle.GetSystem()->SetIterLCPmaxItersSpeed(50);
    vehicle.GetSystem()->SetIterLCPmaxItersStab(50);
    ////vehicle.GetSystem()->SetTol(0);
    ////vehicle.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    ////vehicle.GetSystem()->SetMinBounceSpeed(2.0);
    ////vehicle.GetSystem()->SetIterLCPomega(0.8);
    ////vehicle.GetSystem()->SetIterLCPsharpnessLambda(1.0);

    // Initialize the vehicle at the specified position.
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // Control internal collisions and contact monitoring.
    ////vehicle.SetCollide(TrackCollide::NONE);
    ////vehicle.MonitorContacts(TrackCollide::SPROCKET_LEFT | TrackCollide::SHOES_LEFT | TrackCollide::IDLER_LEFT);
    ////vehicle.SetContactCollection(true);

    // ------------------
    // Create the terrain
    // ------------------

    DeformableTerrain terrain(vehicle.GetSystem());
    terrain.SetPlane(ChCoordsys<>(VNULL, Q_from_AngX(CH_C_PI_2)));
    terrain.SetSoilParametersSCM(2e7,   // Bekker Kphi
                                 0,     // Bekker Kc
                                 1.1,   // Bekker n exponent
                                 0,     // Mohr cohesive limit (Pa)
                                 20,    // Mohr friction limit (degrees)
                                 0.01,  // Janosi shear coefficient (m)
                                 2e8    // Elastic stiffness (Pa/m), before plastic yeld
                                 );
    ////terrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 80, 16);
    terrain.SetPlotType(vehicle::DeformableTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    ////terrain.SetPlotType(vehicle::DeformableTerrain::PLOT_SINKAGE, 0, 0.15);
    terrain.Initialize(terrainHeight, terrainLength, terrainWidth, divLength, divWidth);

    AddFixedObstacles(vehicle.GetSystem());
    ////AddMovingObstacles(vehicle.GetSystem());

    // ----------------------------
    // Create the powertrain system
    // ----------------------------

    M113_SimplePowertrain powertrain;
    powertrain.Initialize(vehicle.GetChassis(), vehicle.GetDriveshaft());

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChTrackedVehicleIrrApp app(&vehicle, &powertrain, L"M113 Vehicle Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    app.SetChaseCameraPosition(ChVector<>(-3, 4, 1.5));
    app.SetChaseCameraMultipliers(1e-4, 10);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // ------------------------
    // Create the driver system
    // ------------------------

    ChIrrGuiDriver driver(app);

    // Set the time response for keyboard inputs.
    double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // Set file with driver input time series
    driver.SetInputDataFile(vehicle::GetDataFile("M113/driver/Acceleration.txt"));
    driver.SetInputMode(ChIrrGuiDriver::DATAFILE);

    driver.Initialize();

    // -----------------
    // Initialize output
    // -----------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (img_output) {
        if (ChFileutils::MakeDirectory(img_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // ---------------
    // Simulation loop
    // ---------------

    vehicle.GetSystem()->SetupInitial();


    // Inter-module communication data
    BodyStates shoe_states_left(vehicle.GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(vehicle.GetNumTrackShoes(RIGHT));
    TrackShoeForces shoe_forces_left(vehicle.GetNumTrackShoes(LEFT));
    TrackShoeForces shoe_forces_right(vehicle.GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;
    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (img_output && step_number > 0) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                app.WriteImageToFile(filename);
            }

            render_frame++;
        }

        // Collect output data from modules
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();
        double powertrain_torque = powertrain.GetOutputTorque();
        double driveshaft_speed = vehicle.GetDriveshaftSpeed();
        vehicle.GetTrackShoeStates(LEFT, shoe_states_left);
        vehicle.GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, shoe_forces_left, shoe_forces_right);
        app.Synchronize("", steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        terrain.Advance(step);
        powertrain.Advance(step);
        vehicle.Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;
    }

    vehicle.WriteContacts("M113_contacts.out");

    return 0;
}

// =============================================================================
void AddFixedObstacles(ChSystem* system) {
    double radius = 2;
    double length = 10;

    float friction_coefficient = 0.9f;
    float restitution_coefficient = 0.01f;
    float young_modulus = 2e7f;
    float poisson_ratio = 0.3f;

    auto obstacle = std::shared_ptr<ChBody>(system->NewBody());
    obstacle->SetPos(ChVector<>(0, 0, -1.8));
    obstacle->SetBodyFixed(true);
    obstacle->SetCollide(true);

    // Visualization
    auto shape = std::make_shared<ChCylinderShape>();
    shape->GetCylinderGeometry().p1 = ChVector<>(0, -length * 0.5, 0);
    shape->GetCylinderGeometry().p2 = ChVector<>(0, length * 0.5, 0);
    shape->GetCylinderGeometry().rad = radius;
    obstacle->AddAsset(shape);

    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(1, 1, 1));
    obstacle->AddAsset(color);

    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(vehicle::GetDataFile("terrain/textures/tile4.jpg"));
    texture->SetTextureScale(10, 10);
    obstacle->AddAsset(texture);

    // Contact
    obstacle->GetCollisionModel()->ClearModel();
    obstacle->GetCollisionModel()->AddCylinder(radius, radius, length * 0.5);
    obstacle->GetCollisionModel()->BuildModel();

    switch (obstacle->GetContactMethod()) {
        case ChMaterialSurfaceBase::DVI:
            obstacle->GetMaterialSurface()->SetFriction(friction_coefficient);
            obstacle->GetMaterialSurface()->SetRestitution(restitution_coefficient);
            break;
        case ChMaterialSurfaceBase::DEM:
            obstacle->GetMaterialSurfaceDEM()->SetFriction(friction_coefficient);
            obstacle->GetMaterialSurfaceDEM()->SetRestitution(restitution_coefficient);
            obstacle->GetMaterialSurfaceDEM()->SetYoungModulus(young_modulus);
            obstacle->GetMaterialSurfaceDEM()->SetPoissonRatio(poisson_ratio);
            break;
    }

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
    auto material = std::make_shared<ChMaterialSurfaceDEM>();
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
    ball->SetMaterialSurface(material);

    ball->SetCollide(true);

    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(radius);
    ball->GetCollisionModel()->BuildModel();

    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));

    auto sphere = std::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = radius;
    ball->AddAsset(sphere);

    auto mtexture = std::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    ball->AddAsset(mtexture);

    system->AddBody(ball);
}