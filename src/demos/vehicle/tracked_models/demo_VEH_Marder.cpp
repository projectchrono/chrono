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
// Authors: Radu Serban / Rainer Gericke
// =============================================================================
//
// Demonstration program for Marder vehicle on rigid terrain.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/marder/Marder.h"

#ifdef CHRONO_PARDISO_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::marder;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================
// Initial vehicle position
ChVector<> initLoc(-40, 0, 0.9);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 5e-4;

// Use HHT + MKL
bool use_mkl = false;

// Time interval between two render frames
double render_step_size = 1.0 / 120;  // FPS = 120

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 0.0);

// Output directories
const std::string out_dir = GetChronoOutputPath() + "Marder";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string img_dir = out_dir + "/IMG";

// Output
bool povray_output = false;
bool img_output = false;
bool dbg_output = false;

// =============================================================================

// Forward declarations
void AddFixedObstacles(ChSystem* system);
void AddFallingObjects(ChSystem* system);

// =============================================================================
int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    CollisionType chassis_collision_type = CollisionType::NONE;
    ////TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
    ////DrivelineTypeTV driveline_type = DrivelineTypeTV::SIMPLE;
    BrakeType brake_type = BrakeType::SIMPLE;
    PowertrainModelType powertrain_type = PowertrainModelType::SIMPLE_CVT;

    Marder marder;
    marder.SetContactMethod(contact_method);
    ////marder.SetTrackShoeType(shoe_type);
    ////marder.SetDrivelineType(driveline_type);
    marder.SetBrakeType(brake_type);
    marder.SetPowertrainType(powertrain_type);
    marder.SetChassisCollisionType(chassis_collision_type);

    ////marder.SetChassisFixed(true);
    ////marder.CreateTrack(false);

    // Control steering type (enable crossdrive capability)
    ////marder.GetDriveline()->SetGyrationMode(true);

    // Change collision detection system
    ////marder.SetCollisionSystemType(collision::ChCollisionSystemType::CHRONO);

    // Change collision shape for road wheels, idlers, and rollers (true: cylinder; false: cylshell)
    ////marder.SetWheelCollisionType(false, false, false);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    marder.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    marder.Initialize();

    // Set visualization type for vehicle components.
    VisualizationType track_vis = VisualizationType::MESH;
    marder.SetChassisVisualizationType(VisualizationType::MESH);
    marder.SetSprocketVisualizationType(track_vis);
    marder.SetIdlerVisualizationType(track_vis);
    marder.SetRollerVisualizationType(track_vis);
    marder.SetSuspensionVisualizationType(track_vis);
    marder.SetIdlerWheelVisualizationType(track_vis);
    marder.SetRoadWheelVisualizationType(track_vis);
    marder.SetTrackShoeVisualizationType(VisualizationType::PRIMITIVES);

    // Disable gravity in this simulation
    ////marder.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Change (SMC) contact force model
    ////if (contact_method == ChContactMethod::SMC) {
    ////static_cast<ChSystemSMC*>(m113.GetSystem())->SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);
    ////}

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////marder.GetVehicle().SetCollide(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////marder.GetVehicle().SetCollide(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////marder.GetVehicle().SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////marder.GetVehicle().SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////marder.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor contacts involving one of the sprockets.
    marder.GetVehicle().MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Monitor only contacts involving the chassis.
    ////marder.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Render contact normals and/or contact forces.
    marder.GetVehicle().SetRenderContactNormals(true);
    ////marder.GetVehicle().SetRenderContactForces(true, 1e-4);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////marder.GetVehicle().SetContactCollection(true);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(marder.GetSystem());
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.75f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // --------------------------------
    // Add fixed and/or falling objects
    // --------------------------------

    ////AddFixedObstacles(vehicle.GetSystem());
    ////AddFallingObjects(vehicle.GetSystem());

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    auto vis = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Marder Vehicle Demo");
    vis->SetChaseCamera(trackPoint, 10.0, 0.5);
    ////vis->SetChaseCameraPosition(vehicle.GetPos() + ChVector<>(0, 2, 0));
    vis->SetChaseCameraMultipliers(1e-4, 10);
    vis->Initialize();
    vis->AddLightDirectional();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&marder.GetVehicle());

    // ------------------------
    // Create the driver system
    // ------------------------

    ChIrrGuiDriver driver(*vis);

    // Set the time response for keyboard inputs.
    double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);
    driver.SetGains(2, 5, 5);

    driver.Initialize();

    // -----------------
    // Initialize output
    // -----------------

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    if (img_output) {
        if (!filesystem::create_directory(filesystem::path(img_dir))) {
            std::cout << "Error creating directory " << img_dir << std::endl;
            return 1;
        }
    }

    // Set up vehicle output
    ////marder.GetVehicle().SetChassisOutput(true);
    ////marder.GetVehicle().SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    ////marder.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    ////marder.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    switch (contact_method) {
        case ChContactMethod::NSC:
            std::cout << "Use NSC" << std::endl;
            // Cannot use HHT + MKL with NSC contact
            use_mkl = false;
            break;
        case ChContactMethod::SMC:
            std::cout << "Use SMC" << std::endl;
            break;
    }

#ifndef CHRONO_PARDISO_MKL
    use_mkl = false;
#endif

    if (use_mkl) {
#ifdef CHRONO_PARDISO_MKL
        std::cout << "Solver: PardisoMKL" << std::endl;
        std::cout << "Integrator: HHT" << std::endl;

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        marder.GetSystem()->SetSolver(mkl_solver);

        marder.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(marder.GetSystem()->GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxiters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
        integrator->SetMode(ChTimestepperHHT::ACCELERATION);
        integrator->SetStepControl(false);
        integrator->SetModifiedNewton(false);
        integrator->SetScaling(true);
        ////integrator->SetVerbose(true);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverBB>();
        solver->SetMaxIterations(120);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
        marder.GetSystem()->SetSolver(solver);

        marder.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
        marder.GetSystem()->SetMinBounceSpeed(2.0);
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    BodyStates shoe_states_left(marder.GetVehicle().GetNumTrackShoes(LEFT));
    BodyStates shoe_states_right(marder.GetVehicle().GetNumTrackShoes(RIGHT));
    TerrainForces shoe_forces_left(marder.GetVehicle().GetNumTrackShoes(LEFT));
    TerrainForces shoe_forces_right(marder.GetVehicle().GetNumTrackShoes(RIGHT));

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    marder.GetVehicle().EnableRealtime(true);
    while (vis->Run()) {
        // Debugging output
        if (dbg_output) {
            auto track_L = marder.GetVehicle().GetTrackAssembly(LEFT);
            auto track_R = marder.GetVehicle().GetTrackAssembly(RIGHT);
            cout << "Time: " << marder.GetSystem()->GetChTime() << endl;
            cout << "      Num. contacts: " << marder.GetSystem()->GetNcontacts() << endl;
            const ChFrameMoving<>& c_ref = marder.GetChassisBody()->GetFrame_REF_to_abs();
            const ChVector<>& c_pos = marder.GetVehicle().GetPos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector<>& i_pos_abs = track_L->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_L->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            {
                const ChVector<>& i_pos_abs = track_R->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_R->GetSprocket()->GetGearBody()->GetPos();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      R idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      R sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            cout << "      L suspensions (arm angles):";
            for (size_t i = 0; i < track_L->GetNumTrackSuspensions(); i++) {
                cout << " " << track_L->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):";
            for (size_t i = 0; i < track_R->GetNumTrackSuspensions(); i++) {
                cout << " " << track_R->GetTrackSuspension(i)->GetCarrierAngle();
            }
            cout << endl;
        }

        if (step_number % render_steps == 0) {
            // Render scene
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(marder.GetSystem(), filename);
            }
            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                vis->WriteImageToFile(filename);
            }
            render_frame++;
        }

        // Collect output data from modules
        DriverInputs driver_inputs = driver.GetInputs();
        marder.GetVehicle().GetTrackShoeStates(LEFT, shoe_states_left);
        marder.GetVehicle().GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = marder.GetVehicle().GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        marder.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        vis->Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        marder.Advance(step_size);
        vis->Advance(step_size);

        // Report if the chassis experienced a collision
        if (marder.GetVehicle().IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    marder.GetVehicle().WriteContacts("Marder_contacts.out");

    return 0;
}

// =============================================================================
void AddFixedObstacles(ChSystem* system) {
    double radius = 2.2;
    double length = 6;

    auto obstacle = std::shared_ptr<ChBody>(system->NewBody());
    obstacle->SetPos(ChVector<>(10, 0, -1.8));
    obstacle->SetBodyFixed(true);
    obstacle->SetCollide(true);

    // Visualization
    auto shape = chrono_types::make_shared<ChCylinderShape>();
    shape->GetCylinderGeometry().p1 = ChVector<>(0, -length * 0.5, 0);
    shape->GetCylinderGeometry().p2 = ChVector<>(0, length * 0.5, 0);
    shape->GetCylinderGeometry().rad = radius;
    shape->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"));
    obstacle->AddVisualShape(shape);

    // Contact
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    obstacle->GetCollisionModel()->ClearModel();
    obstacle->GetCollisionModel()->AddCylinder(obst_mat, radius, radius, length * 0.5);
    obstacle->GetCollisionModel()->BuildModel();

    system->AddBody(obstacle);
}

// =============================================================================
void AddFallingObjects(ChSystem* system) {
    double radius = 0.1;
    double mass = 10;

    auto ball = std::shared_ptr<ChBody>(system->NewBody());
    ball->SetMass(mass);
    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector<>(1, 1, 1));
    ball->SetPos(initLoc + ChVector<>(-3, 0, 2));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetPos_dt(ChVector<>(3, 0, 0));
    ball->SetBodyFixed(false);

    MaterialInfo minfo;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    ball->SetCollide(true);
    ball->GetCollisionModel()->ClearModel();
    ball->GetCollisionModel()->AddSphere(obst_mat, radius);
    ball->GetCollisionModel()->BuildModel();

    auto sphere = chrono_types::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = radius;
    sphere->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddVisualShape(sphere);

    system->AddBody(ball);
}
