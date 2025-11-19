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
#include "chrono/output/ChOutputASCII.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"

#include "chrono_models/vehicle/marder/Marder.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vehicle/tracked_vehicle/ChTrackedVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
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

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Initial vehicle position
ChVector3d initLoc(-40, 0, 0.9);

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
ChVector3d trackPoint(0.0, 0.0, 0.0);

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
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // --------------------------
    // Construct the M113 vehicle
    // --------------------------

    ChContactMethod contact_method = ChContactMethod::SMC;
    CollisionType chassis_collision_type = CollisionType::NONE;
    ////TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
    ////DrivelineTypeTV driveline_type = DrivelineTypeTV::SIMPLE;
    BrakeType brake_type = BrakeType::SIMPLE;
    EngineModelType engine_type = EngineModelType::SIMPLE;
    TransmissionModelType transmission_type = TransmissionModelType::AUTOMATIC_SIMPLE_MAP;

    Marder marder;
    marder.SetContactMethod(contact_method);
    ////marder.SetTrackShoeType(shoe_type);
    ////marder.SetDrivelineType(driveline_type);
    marder.SetBrakeType(brake_type);
    marder.SetEngineType(engine_type);
    marder.SetTransmissionType(transmission_type);
    marder.SetChassisCollisionType(chassis_collision_type);

    ////marder.SetChassisFixed(true);
    ////marder.CreateTrack(false);

    // Control steering type (enable crossdrive capability)
    ////marder.GetDriveline()->SetGyrationMode(true);

    // Change collision detection system
    ////marder.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

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

    auto& vehicle = marder.GetVehicle();

    // Disable gravity in this simulation
    ////marder.GetSystem()->SetGravitationalAcceleration(ChVector3d(0, 0, 0));

    // Change (SMC) contact force model
    ////if (contact_method == ChContactMethod::SMC) {
    ////static_cast<ChSystemSMC*>(m113.GetSystem())->SetContactForceModel(ChSystemSMC::ContactForceModel::PlainCoulomb);
    ////}

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////vehicle.EnableCollision(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////vehicle.EnableCollision(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////vehicle.SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////vehicle.SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor contacts involving one of the sprockets.
    vehicle.MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Monitor only contacts involving the chassis.
    ////vehicle.MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Render contact normals and/or contact forces.
    vehicle.SetRenderContactNormals(true);
    ////vehicle.SetRenderContactForces(true, 1e-4);

    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////vehicle.SetContactCollection(true);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(marder.GetSystem());
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.75f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(GetVehicleDataFile("terrain/textures/grass.jpg"), 20, 20);
    terrain.Initialize();

    // --------------------------------
    // Add fixed and/or falling objects
    // --------------------------------

    ////AddFixedObstacles(vehicle.GetSystem());
    ////AddFallingObjects(vehicle.GetSystem());

    // ------------------------
    // Create the driver system
    // ------------------------

    ChInteractiveDriver driver(vehicle);
    double steering_time = 0.5;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);
    driver.SetGains(2, 5, 5);
    driver.Initialize();

    // -----------------------------------------
    // Create the vehicle run-time visualization
    // -----------------------------------------

    std::shared_ptr<ChVehicleVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            // Create the vehicle Irrlicht interface
            auto vis_irr = chrono_types::make_shared<ChTrackedVehicleVisualSystemIrrlicht>();
            vis_irr->SetWindowTitle("Marder Vehicle Demo");
            vis_irr->SetChaseCamera(trackPoint, 10.0, 0.5);
            vis_irr->SetChaseCameraMultipliers(1e-4, 10);
            vis_irr->Initialize();
            vis_irr->AddLightDirectional(60, -90);
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
            // Create the vehicle VSG interface
            auto vis_vsg = chrono_types::make_shared<ChTrackedVehicleVisualSystemVSG>();
            vis_vsg->SetWindowTitle("Marder Vehicle Demo");
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetChaseCamera(trackPoint, 12.0, 0.75);
            vis_vsg->AttachVehicle(&vehicle);
            vis_vsg->AttachDriver(&driver);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // -----------------
    // Initialize output
    // -----------------

    const std::string out_dir = GetChronoOutputPath() + "Marder";
    const std::string pov_dir = out_dir + "/POVRAY";
    const std::string img_dir = out_dir + "/IMG";

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
    ////vehicle.SetChassisOutput(true);
    ////vehicle.SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    ////vehicle.SetOutput(ChOutput::Type::ASCII, ChOutput::Mode::FRAMES, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    ////vehicle.ExportComponentList(out_dir + "/component_list.json");

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
        integrator->SetMaxIters(50);
        integrator->SetAbsTolerances(1e-4, 1e2);
        integrator->SetStepControl(false);
        integrator->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_ITERATION);
        ////integrator->SetVerbose(true);
#endif
    } else {
        auto solver = chrono_types::make_shared<ChSolverBB>();
        solver->SetMaxIterations(120);
        solver->SetOmega(0.8);
        solver->SetSharpnessLambda(1.0);
        marder.GetSystem()->SetSolver(solver);

        marder.GetSystem()->SetMaxPenetrationRecoverySpeed(1.5);
    }

    // ---------------
    // Simulation loop
    // ---------------

    std::cout << "\n============ Vehicle subsystems ============" << std::endl;
    vehicle.LogSubsystemTypes();

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter
    int step_number = 0;
    int render_frame = 0;

    vehicle.EnableRealtime(true);
    while (vis->Run()) {
        // Debugging output
        if (dbg_output) {
            auto track_L = vehicle.GetTrackAssembly(LEFT);
            auto track_R = vehicle.GetTrackAssembly(RIGHT);
            cout << "Time: " << marder.GetSystem()->GetChTime() << endl;
            cout << "      Num. contacts: " << marder.GetSystem()->GetNumContacts() << endl;
            const ChFrameMoving<>& c_ref = marder.GetChassisBody()->GetFrameRefToAbs();
            const ChVector3d& c_pos = vehicle.GetPos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector3d& i_pos_abs = track_L->GetIdler()->GetWheelBody()->GetPos();
                const ChVector3d& s_pos_abs = track_L->GetSprocket()->GetGearBody()->GetPos();
                ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
            }
            {
                const ChVector3d& i_pos_abs = track_R->GetIdler()->GetWheelBody()->GetPos();
                const ChVector3d& s_pos_abs = track_R->GetSprocket()->GetGearBody()->GetPos();
                ChVector3d i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector3d s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
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

            // Zero-pad frame numbers in file names for postprocessing
            if (povray_output) {
                std::ostringstream filename;
                filename << pov_dir << "/data_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".dat";
                utils::WriteVisualizationAssets(marder.GetSystem(), filename.str());
            }
            if (img_output && step_number > 200) {
                std::ostringstream filename;
                filename << img_dir << "/img_" << std::setw(4) << std::setfill('0') << render_frame + 1 << ".jpg";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }

        // Current driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        double time = vehicle.GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        marder.Synchronize(time, driver_inputs);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        marder.Advance(step_size);
        vis->Advance(step_size);

        // Report if the chassis experienced a collision
        if (vehicle.IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Increment frame number
        step_number++;
    }

    vehicle.WriteContacts("Marder_contacts.out");

    return 0;
}

// =============================================================================
void AddFixedObstacles(ChSystem* system) {
    double radius = 2.2;
    double length = 6;

    auto obstacle = chrono_types::make_shared<ChBody>();
    obstacle->SetPos(ChVector3d(10, 0, -1.8));
    obstacle->SetFixed(true);
    obstacle->EnableCollision(true);

    // Visualization
    auto vis_shape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, length);
    vis_shape->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 10, 10);
    obstacle->AddVisualShape(vis_shape, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    // Contact
    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(obst_mat, radius, length);
    obstacle->AddCollisionShape(ct_shape, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    system->AddBody(obstacle);
}

// =============================================================================
void AddFallingObjects(ChSystem* system) {
    double radius = 0.1;
    double mass = 10;

    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetMass(mass);
    ball->SetInertiaXX(0.4 * mass * radius * radius * ChVector3d(1, 1, 1));
    ball->SetPos(initLoc + ChVector3d(-3, 0, 2));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetPosDt(ChVector3d(3, 0, 0));
    ball->SetFixed(false);

    ChContactMaterialData minfo;
    auto obst_mat = minfo.CreateMaterial(system->GetContactMethod());

    ball->EnableCollision(true);
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeSphere>(obst_mat, radius);
    ball->AddCollisionShape(ct_shape);

    auto vis_shape = chrono_types::make_shared<ChVisualShapeSphere>(radius);
    vis_shape->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddVisualShape(vis_shape);

    system->AddBody(ball);
}
