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
// Demonstration program for M113 vehicle on rigid terrain.
//
// =============================================================================

#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/solver/ChSolverBB.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/output/ChVehicleOutputASCII.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

#include "chrono_models/vehicle/m113/M113.h"

#ifdef CHRONO_PARDISO_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::m113;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================
// Initial vehicle position
ChVector<> initLoc(-40, 0, 0.8);

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
const std::string out_dir = GetChronoOutputPath() + "M113";
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

    bool fix_chassis = false;
    bool create_track = true;

    ChContactMethod contact_method = ChContactMethod::SMC;
    CollisionType chassis_collision_type = CollisionType::NONE;
    TrackShoeType shoe_type = TrackShoeType::SINGLE_PIN;
    BrakeType brake_type = BrakeType::SIMPLE;
    DrivelineTypeTV driveline_type = DrivelineTypeTV::BDS;
    PowertrainModelType powertrain_type = PowertrainModelType::SHAFTS;

    //// TODO
    //// When using SMC, a double-pin shoe type requires MKL or MUMPS.
    //// However, there appear to still be redundant constraints in the double-pin assembly
    //// resulting in solver failures with MKL and MUMPS (rank-deficient matrix).
    if (shoe_type == TrackShoeType::DOUBLE_PIN)
        contact_method = ChContactMethod::NSC;

    M113 m113;
    m113.SetContactMethod(contact_method);
    m113.SetTrackShoeType(shoe_type);
    m113.SetDrivelineType(driveline_type);
    m113.SetBrakeType(brake_type);
    m113.SetPowertrainType(powertrain_type);
    m113.SetChassisCollisionType(chassis_collision_type);

    m113.SetChassisFixed(fix_chassis);
    m113.CreateTrack(create_track);

    // Disable gravity in this simulation
    ////m113.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Control steering type (enable crossdrive capability)
    ////m113.GetDriveline()->SetGyrationMode(true);

    // ------------------------------------------------
    // Initialize the vehicle at the specified position
    // ------------------------------------------------
    m113.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    m113.Initialize();

    // Set visualization type for vehicle components.
    VisualizationType track_vis =
        (shoe_type == TrackShoeType::SINGLE_PIN) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;
    m113.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
    m113.SetSprocketVisualizationType(track_vis);
    m113.SetIdlerVisualizationType(track_vis);
    m113.SetRoadWheelAssemblyVisualizationType(track_vis);
    m113.SetRoadWheelVisualizationType(track_vis);
    m113.SetTrackShoeVisualizationType(track_vis);

    // --------------------------------------------------
    // Control internal collisions and contact monitoring
    // --------------------------------------------------

    // Enable contact on all tracked vehicle parts, except the left sprocket
    ////m113.GetVehicle().SetCollide(TrackedCollisionFlag::ALL & (~TrackedCollisionFlag::SPROCKET_LEFT));

    // Disable contact for all tracked vehicle parts
    ////m113.GetVehicle().SetCollide(TrackedCollisionFlag::NONE);

    // Disable all contacts for vehicle chassis (if chassis collision was defined)
    ////m113.GetVehicle().SetChassisCollide(false);

    // Disable only contact between chassis and track shoes (if chassis collision was defined)
    ////m113.GetVehicle().SetChassisVehicleCollide(false);

    // Monitor internal contacts for the chassis, left sprocket, left idler, and first shoe on the left track.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS | TrackedCollisionFlag::SPROCKET_LEFT |
    ////                        TrackedCollisionFlag::SHOES_LEFT | TrackedCollisionFlag::IDLER_LEFT);

    // Monitor only contacts involving the chassis.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::CHASSIS);

    // Monitor contacts involving one of the sprockets.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::SPROCKET_LEFT | TrackedCollisionFlag::SPROCKET_RIGHT);

    // Monitor only contacts involving the left idler.
    ////m113.GetVehicle().MonitorContacts(TrackedCollisionFlag::IDLER_LEFT);
    
    // Collect contact information.
    // If enabled, number of contacts and local contact point locations are collected for all
    // monitored parts.  Data can be written to a file by invoking ChTrackedVehicle::WriteContacts().
    ////m113.GetVehicle().SetContactCollection(true);

    // Demonstration of user callback for specifying contact between track shoe and
    // idlers and/or road wheels and/or ground.
    // This particular implementation uses a simple SMC-like contact force (normal only).
    class MyCustomContact : public ChTrackCustomContact {
      public:
        virtual bool OverridesIdlerContact() const override { return false; }
        virtual bool OverridesWheelContact() const override { return true; }
        virtual bool OverridesGroundContact() const override { return false; }

        virtual void ComputeIdlerContactForce(const collision::ChCollisionInfo& cinfo,
                                              std::shared_ptr<ChBody> wheelBody,
                                              std::shared_ptr<ChBody> shoeBody,
                                              ChVector<>& forceShoe) override {
            ComputeContactForce(cinfo, wheelBody, shoeBody, forceShoe);
        };

        virtual void ComputeWheelContactForce(const collision::ChCollisionInfo& cinfo,
                                              std::shared_ptr<ChBody> wheelBody,
                                              std::shared_ptr<ChBody> shoeBody,
                                              ChVector<>& forceShoe) override {
            ComputeContactForce(cinfo, wheelBody, shoeBody, forceShoe);
        };

        virtual void ComputeGroundContactForce(const collision::ChCollisionInfo& cinfo,
                                               std::shared_ptr<ChBody> groundBody,
                                               std::shared_ptr<ChBody> shoeBody,
                                               ChVector<>& forceShoe) override {
            ComputeContactForce(cinfo, groundBody, shoeBody, forceShoe);
        };

      private:
        void ComputeContactForce(const collision::ChCollisionInfo& cinfo,
                                 std::shared_ptr<ChBody> other,
                                 std::shared_ptr<ChBody> shoe,
                                 ChVector<>& forceShoe) {
            ////std::cout << other->GetName() << " " << shoe->GetName() << std::endl;

            if (cinfo.distance >= 0) {
                forceShoe = VNULL;
                return;
            }

            // Create a fictitious SMC composite contact material
            // (do not use the shape materials, so that this can work with both an SMC and NSC system)
            ChMaterialCompositeSMC mat;
            mat.E_eff = 2e7f;
            mat.cr_eff = 0.2f;

            auto delta = -cinfo.distance;
            auto normal_dir = cinfo.vN;
            auto p1 = cinfo.vpA;
            auto p2 = cinfo.vpB;
            auto objA = cinfo.modelA->GetContactable();
            auto objB = cinfo.modelB->GetContactable();
            auto vel1 = objA->GetContactPointSpeed(p1);
            auto vel2 = objB->GetContactPointSpeed(p2);

            ChVector<> relvel = vel2 - vel1;
            double relvel_n_mag = relvel.Dot(normal_dir);

            double eff_radius = 0.1;
            double eff_mass = objA->GetContactableMass() * objB->GetContactableMass() /
                              (objA->GetContactableMass() + objB->GetContactableMass());
            double Sn = 2 * mat.E_eff * std::sqrt(eff_radius * delta);
            double loge = std::log(mat.cr_eff);
            double beta = loge / std::sqrt(loge * loge + CH_C_PI * CH_C_PI);
            double kn = (2.0 / 3) * Sn;
            double gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);

            double forceN = kn * delta - gn * relvel_n_mag;
            forceShoe = (forceN < 0) ? VNULL : forceN * normal_dir;
        }
    };

    // Enable custom contact force calculation for road wheel - track shoe collisions.
    // If enabled, the underlying Chrono contact processing does not compute any forces.
    ////m113.GetVehicle().EnableCustomContact(chrono_types::make_shared<MyCustomContact>());

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(m113.GetSystem());
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.2f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);
    auto patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), terrainLength, terrainWidth);
    patch->SetColor(ChColor(0.5f, 0.8f, 0.5f));
    patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
    terrain.Initialize();

    // --------------------------------
    // Add fixed and/or falling objects
    // --------------------------------

    ////AddFixedObstacles(m113.GetSystem());
    ////AddFallingObjects(m113.GetSystem());

    // ---------------------------------------
    // Create the vehicle Irrlicht application
    // ---------------------------------------

    ChTrackedVehicleIrrApp app(&m113.GetVehicle(), L"M113 Vehicle Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);
    ////app.SetChaseCameraPosition(m113.GetVehicle().GetVehiclePos() + ChVector<>(0, 2, 0));
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
    m113.GetVehicle().SetChassisOutput(true);
    m113.GetVehicle().SetTrackAssemblyOutput(VehicleSide::LEFT, true);
    m113.GetVehicle().SetOutput(ChVehicleOutput::ASCII, out_dir, "output", 0.1);

    // Generate JSON information with available output channels
    m113.GetVehicle().ExportComponentList(out_dir + "/component_list.json");

    // ------------------------------
    // Solver and integrator settings
    // ------------------------------

    // Cannot use HHT + MKL with NSC contact
    if (contact_method == ChContactMethod::NSC) {
        use_mkl = false;
    }

#ifndef CHRONO_PARDISO_MKL
    // Cannot use HHT + PardisoMKL if Chrono::PardisoMKL not available
    use_mkl = false;
#endif

    if (use_mkl) {
#ifdef CHRONO_PARDISO_MKL
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
        mkl_solver->LockSparsityPattern(true);
        m113.GetSystem()->SetSolver(mkl_solver);

        m113.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(m113.GetSystem()->GetTimestepper());
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

    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        // Debugging output
        if (dbg_output) {
            auto track_L = m113.GetVehicle().GetTrackAssembly(LEFT);
            auto track_R = m113.GetVehicle().GetTrackAssembly(RIGHT);
            cout << "Time: " << m113.GetSystem()->GetChTime() << endl;
            cout << "      Num. contacts: " << m113.GetSystem()->GetNcontacts() << endl;
            const ChFrameMoving<>& c_ref = m113.GetChassisBody()->GetFrame_REF_to_abs();
            const ChVector<>& c_pos = m113.GetVehicle().GetVehiclePos();
            cout << "      chassis:    " << c_pos.x() << "  " << c_pos.y() << "  " << c_pos.z() << endl;
            {
                const ChVector<>& i_pos_abs = track_L->GetIdler()->GetWheelBody()->GetPos();
                const ChVector<>& s_pos_abs = track_L->GetSprocket()->GetGearBody()->GetPos();
                const ChVector<>& s_omg_rel = track_L->GetSprocket()->GetGearBody()->GetWvel_loc();
                auto s_appl_trq = track_L->GetSprocket()->GetAxle()->GetAppliedTorque();
                ChVector<> i_pos_rel = c_ref.TransformPointParentToLocal(i_pos_abs);
                ChVector<> s_pos_rel = c_ref.TransformPointParentToLocal(s_pos_abs);
                cout << "      L idler:    " << i_pos_rel.x() << "  " << i_pos_rel.y() << "  " << i_pos_rel.z() << endl;
                cout << "      L sprocket: " << s_pos_rel.x() << "  " << s_pos_rel.y() << "  " << s_pos_rel.z() << endl;
                cout << "      L sprocket omg: " << s_omg_rel << endl;
                cout << "      L sprocket trq: " << s_appl_trq << endl;
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
            for (size_t i = 0; i < track_L->GetNumRoadWheelAssemblies(); i++) {
                cout << " " << track_L->GetRoadWheelAssembly(i)->GetCarrierAngle();
            }
            cout << endl;
            cout << "      R suspensions (arm angles):";
            for (size_t i = 0; i < track_R->GetNumRoadWheelAssemblies(); i++) {
                cout << " " << track_R->GetRoadWheelAssembly(i)->GetCarrierAngle();
            }
            cout << endl;
        }

        if (step_number % render_steps == 0) {
            // Render scene
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(m113.GetSystem(), filename);
            }
            if (img_output && step_number > 200) {
                char filename[100];
                sprintf(filename, "%s/img_%03d.jpg", img_dir.c_str(), render_frame + 1);
                app.WriteImageToFile(filename);
            }
            render_frame++;
        }

        // Collect output data from modules
        ChDriver::Inputs driver_inputs = driver.GetInputs();
        m113.GetVehicle().GetTrackShoeStates(LEFT, shoe_states_left);
        m113.GetVehicle().GetTrackShoeStates(RIGHT, shoe_states_right);

        // Update modules (process inputs from other modules)
        double time = m113.GetVehicle().GetChTime();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        m113.Synchronize(time, driver_inputs, shoe_forces_left, shoe_forces_right);
        app.Synchronize("", driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        m113.Advance(step_size);
        app.Advance(step_size);

        // Report if the chassis experienced a collision
        if (m113.GetVehicle().IsPartInContact(TrackedCollisionFlag::CHASSIS)) {
            std::cout << time << "  chassis contact" << std::endl;
        }

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    m113.GetVehicle().WriteContacts(out_dir + "/M113_contacts.out");

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
    obstacle->AddAsset(shape);

    auto color = chrono_types::make_shared<ChColorAsset>();
    color->SetColor(ChColor(1, 1, 1));
    obstacle->AddAsset(color);

    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(vehicle::GetDataFile("terrain/textures/tile4.jpg"));
    texture->SetTextureScale(10, 10);
    obstacle->AddAsset(texture);

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
    ball->AddAsset(sphere);

    auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/bluewhite.png"));
    ball->AddAsset(mtexture);

    system->AddBody(ball);
}