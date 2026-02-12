// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Demonstration of using checkpoints with Chrono::Vehicle simulations.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <cmath>

#include "chrono/physics/ChSystemSMC.h"

#include "chrono/assets/ChVisualShapeCone.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChFilters.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/utils/ChVehiclePath.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"

#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
using namespace chrono::postprocess;
#endif

#include "chrono_thirdparty/filesystem/path.h"

#include "demos/vehicle/WheeledVehicleModels.h"
#include "demos/SetChronoSolver.h"

using std::cout;
using std::cerr;
using std::endl;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Terrain type
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;

// Render frequency
double render_fps = 50;

// Maximum simulation time
double t_end = 20.0;

// =============================================================================

RigidTerrain CreateTerrain(ChSystem* sys, const std::vector<ChVector3d>& targets, double height = 0) {
    // Terrain patch dimensions
    double x_size = 200;
    double y_size = 200;

    // Terrain patch reference frame
    ChCoordsysd csys(ChVector3d(0, 0, height), QUNIT);

    // Create the terrain
    RigidTerrain terrain(sys);

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(sys->GetContactMethod());

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, csys, x_size, y_size);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, csys, GetVehicleDataFile("terrain/height_maps/test64.bmp"), x_size,
                                     y_size, 0, 4);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, csys, GetVehicleDataFile("terrain/meshes/test.obj"));
            break;
    }
    patch->SetTexture(GetChronoDataFile("textures/checker2.png"), 20, 20);
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // Add visualization for target locations
    auto target_shape = chrono_types::make_shared<ChVisualShapeCone>(0.5, 2.0);
    target_shape->SetColor(ChColor(1, 0, 0));
    for (const auto target : targets) {
        patch->GetGroundBody()->GetVisualModel()->AddShape(
            target_shape, ChFramed(target + ChVector3d(0, 0, 1.1), QuatFromAngleX(CH_PI)));
    }

    return terrain;
}

// =============================================================================

void SimulateSingle(std::shared_ptr<WheeledVehicleModel> vehicle_model,
                    const ChFrame<> target_pose,
                    double target_speed,
                    const std::string& dir) {
    cout << endl;
    cout << "Simulate single vehicle: " << vehicle_model->ModelName() << endl;
    cout << endl;

    // Create the vehicle path
    double path_length = 40;
    ChVector3d path_dir = target_pose.GetRotMat().GetAxisX();
    ChVector3d path_start = target_pose.GetPos() - path_length * path_dir;
    ChVector3d path_end = target_pose.GetPos() + 20.0 * path_dir;
    auto path = StraightLinePath(path_start + ChVector3d(0, 0, 0.1), path_end + ChVector3d(0, 0, 0.1));

    // Contact method
    ChContactMethod contact_method = ChContactMethod::SMC;

    // Create the vehicle model
    auto initial_pos = path_start + ChVector3d(0, 0, 0.5);
    auto initial_rot = target_pose.GetRot();
    vehicle_model->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    vehicle_model->Create(contact_method, ChCoordsysd(initial_pos, initial_rot), false);
    auto& vehicle = vehicle_model->GetVehicle();
    auto sys = vehicle.GetSystem();

    // Generate log with vehicle subsystems
    std::ofstream ofile(dir + "/template_list.json");
    vehicle.LogSubsystemTypes(ofile);
    ofile.close();

    // Generate JSON information with available output channels
    vehicle.ExportComponentList(dir + "/component_list.json");

    // Create the terrain
    RigidTerrain terrain = CreateTerrain(sys, {target_pose.GetPos()});

    // Create the driver system
    ChPathFollowerDriver driver(vehicle, path, "my_path", target_speed);
    driver.GetSteeringController().SetLookAheadDistance(5.0);
    driver.GetSteeringController().SetGains(0.5, 0, 0);
    driver.GetSpeedController().SetGains(0.4, 0, 0);
    driver.Initialize();

    // Create the vehicle run-time visualization
    std::shared_ptr<ChVehicleVisualSystem> vis;

#ifdef CHRONO_VSG
    auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis_vsg->SetWindowTitle(vehicle_model->ModelName());
    vis_vsg->AttachVehicle(&vehicle);
    vis_vsg->AttachDriver(&driver);
    vis_vsg->SetChaseCamera(vehicle_model->TrackPoint(), vehicle_model->CameraDistance(),
                            vehicle_model->CameraHeight());
    vis_vsg->SetWindowSize(1280, 800);
    vis_vsg->EnableSkyBox();
    vis_vsg->SetCameraAngleDeg(40);
    vis_vsg->SetLightIntensity(1.0f);
    vis_vsg->SetLightDirection(1.8 * CH_PI_2, CH_PI_4);
    vis_vsg->EnableShadows();
    vis_vsg->Initialize();

    vis = vis_vsg;
#endif

    // Set solver and integrator
    double step_size = 2e-3;
    auto solver_type = ChSolver::Type::BARZILAIBORWEIN;
    auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    SetChronoSolver(*sys, solver_type, integrator_type);

    // Simulation loop
    vehicle.EnableRealtime(true);

    int render_frame = 0;
    while (true) {
        double time = sys->GetChTime();
        double dist_to_target = std::abs(Vdot(target_pose.GetPos() - vehicle.GetPos(), path_dir));

        cout << "\r" << std::setprecision(5) << dist_to_target;

        if (dist_to_target < 0.05) {
            cout << "\rReached end pose. delta = " << dist_to_target << endl;
            break;
        }
        if (time > t_end) {
            cout << "\rStopped at end time." << endl;
            break;
        }

        if (vis) {
            if (!vis->Run())
                break;

            if (time >= render_frame / render_fps) {
                vis->BeginScene();
                vis->Render();
                vis->EndScene();

                render_frame++;
            }
        }

        // Synchronize subsystems
        DriverInputs driver_inputs = driver.GetInputs();
        driver.Synchronize(time);
        terrain.Synchronize(time);
        vehicle_model->Synchronize(time, driver_inputs, terrain);
        if (vis)
            vis->Synchronize(time, driver_inputs);

        // Advance simulation
        driver.Advance(step_size);
        terrain.Advance(step_size);
        vehicle_model->Advance(step_size);
        if (vis)
            vis->Advance(step_size);
    }

    cout << endl;
    cout << "Target pose:\n" << target_pose << endl;
    cout << "Target speed: " << target_speed << endl;
    cout << endl;
    cout << "Final pose:\n" << vehicle.GetTransform() << endl;
    cout << "Final speed:  " << vehicle.GetSpeed() << endl;
    cout << endl;

    // Checkpoint final vehicle and driver state
    cout << "Num. position-level states: " << sys->GetNumCoordsPosLevel() << endl;
    cout << "Num. velocity-level states: " << sys->GetNumCoordsVelLevel() << endl;
    cout << endl;
    cout << "Output vehicle checkpoint file: " << dir + "/vehicle_checkpoint.txt" << endl;
    cout << "Output tire checkpoint file:  " << dir + "/tire_X_checkpoint.txt" << endl;
    cout << "Output driver checkpoint file:  " << dir + "/driver_checkpoint.txt" << endl;
    cout << endl;

    vehicle.ExportCheckpoint(ChCheckpoint::Format::ASCII, dir + "/vehicle_checkpoint.txt");
    driver.ExportCheckpoint(ChCheckpoint::Format::ASCII, dir + "/driver_checkpoint.txt");
    int tire_id = 0;
    for (const auto& a : vehicle.GetAxles()) {
        for (const auto& w : a->GetWheels()) {
            if (w->GetTire()) {
                w->GetTire()->ExportCheckpoint(ChCheckpoint::Format::ASCII,
                                               dir + "/tire_" + std::to_string(tire_id++) + "_checkpoint.txt");
            }
        }
    }
}

// =============================================================================

void SimulateBoth(std::shared_ptr<WheeledVehicleModel> vehicle_model_1,
                  std::shared_ptr<WheeledVehicleModel> vehicle_model_2,
                  const std::string& dir_1,
                  const std::string& dir_2) {
    cout << endl;
    cout << "Simulate two vehicle: " << vehicle_model_1->ModelName() << " and " << vehicle_model_2->ModelName() << endl;
    cout << endl;

    // Contact method
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(-9.81 * ChWorldFrame::Vertical());

    // Create the vehicle models
    vehicle_model_1->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    vehicle_model_1->Create(&sys, ChCoordsysd());
    vehicle_model_2->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    vehicle_model_2->Create(&sys, ChCoordsysd());

    auto& vehicle_1 = vehicle_model_1->GetVehicle();
    auto& vehicle_2 = vehicle_model_2->GetVehicle();

    ////vehicle_1.GetChassisBody()->SetFixed(true);
    ////vehicle_2.GetChassisBody()->SetFixed(true);

    // Initialize vehicles and tires from checkpoint files
    {
        vehicle_1.ImportCheckpoint(ChCheckpoint::Format::ASCII, dir_1 + "/vehicle_checkpoint.txt");
        int tire_id = 0;
        for (const auto& a : vehicle_1.GetAxles()) {
            for (const auto& w : a->GetWheels()) {
                if (w->GetTire()) {
                    w->GetTire()->ExportCheckpoint(ChCheckpoint::Format::ASCII,
                                                   dir_1 + "/tire_" + std::to_string(tire_id++) + "_checkpoint.txt");
                }
            }
        }
    }
    {
        vehicle_2.ImportCheckpoint(ChCheckpoint::Format::ASCII, dir_2 + "/vehicle_checkpoint.txt");
        int tire_id = 0;
        for (const auto& a : vehicle_2.GetAxles()) {
            for (const auto& w : a->GetWheels()) {
                if (w->GetTire()) {
                    w->GetTire()->ExportCheckpoint(ChCheckpoint::Format::ASCII,
                                                   dir_2 + "/tire_" + std::to_string(tire_id++) + "_checkpoint.txt");
                }
            }
        }
    }

    // Create the driver systems and initialize from checkpint files
    ChDriver driver_1(vehicle_1);
    ChDriver driver_2(vehicle_2);
    driver_1.ImportCheckpoint(ChCheckpoint::Format::ASCII, dir_1 + "/driver_checkpoint.txt");
    driver_2.ImportCheckpoint(ChCheckpoint::Format::ASCII, dir_2 + "/driver_checkpoint.txt");

    // Create the terrain
    ChVector3d pos_1 = vehicle_1.GetPos();
    ChVector3d pos_2 = vehicle_2.GetPos();
    pos_1.z() = 0;
    pos_2.z() = 0;
    cout << "Position 1: " << pos_1 << endl;
    cout << "Position 2: " << pos_2 << endl;
    RigidTerrain terrain = CreateTerrain(&sys, {pos_1, pos_2});

    // Set solver and integrator
    double step_size = 2e-3;
    auto solver_type = ChSolver::Type::BARZILAIBORWEIN;
    auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    SetChronoSolver(sys, solver_type, integrator_type);

    // Create the vehicle run-time visualization
    std::shared_ptr<ChVehicleVisualSystem> vis;

#ifdef CHRONO_VSG
    auto vis_vsg = chrono_types::make_shared<ChWheeledVehicleVisualSystemVSG>();
    vis_vsg->SetWindowTitle(vehicle_model_1->ModelName() + " and " + vehicle_model_2->ModelName());
    vis_vsg->AttachVehicle(&vehicle_1);
    vis_vsg->SetChaseCamera(ChVector3d(0.0, 0.0, .75), 6.0, 0.5);
    vis_vsg->SetChaseCameraState(utils::ChChaseCamera::Track);
    vis_vsg->SetChaseCameraPosition(pos_1 + ChVector3d(-35, 0, 2.0));
    vis_vsg->SetWindowSize(1280, 800);
    vis_vsg->EnableSkyBox();
    vis_vsg->SetCameraAngleDeg(40);
    vis_vsg->SetLightIntensity(1.0f);
    vis_vsg->SetLightDirection(1.8 * CH_PI_2, CH_PI_4);
    vis_vsg->EnableShadows();
    vis_vsg->Initialize();

    vis = vis_vsg;
#endif

    // Simulation loop
    vehicle_1.EnableRealtime(true);
    vehicle_2.EnableRealtime(true);

    int render_frame = 0;
    double time0 = sys.GetChTime();
    while (true) {
        double time = sys.GetChTime();

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
            cout << "\rStopped at end time." << endl;
            break;
        }

        // Driver inputs
        // overwrite steering inputs
        DriverInputs driver_inputs_1 = driver_1.GetInputs();
        DriverInputs driver_inputs_2 = driver_2.GetInputs();

        double tau = time - time0;
        double si_1 = driver_inputs_1.m_steering;
        double si_2 = driver_inputs_2.m_steering;
        double sf_1 = -0.5;
        double sf_2 = +0.5;
        double steering_1 = si_1;
        double steering_2 = si_2;
        if (tau > 1.0) {
            steering_1 = sf_1;
            steering_2 = sf_2;
        } else if (tau > 0.5) {
            steering_1 = (sf_1 - si_1) * (tau - 0.5) / 0.5 + si_1;
            steering_2 = (sf_2 - si_2) * (tau - 0.5) / 0.5 + si_2;
        }
        driver_inputs_1.m_steering = steering_1;
        driver_inputs_2.m_steering = steering_2;

        // Update modules (process inputs from other modules)
        driver_1.Synchronize(time);
        driver_2.Synchronize(time);
        vehicle_1.Synchronize(time, driver_inputs_1, terrain);
        vehicle_2.Synchronize(time, driver_inputs_2, terrain);
        terrain.Synchronize(time);
        if (vis)
            vis->Synchronize(time, driver_inputs_1);

        // Advance simulation for one timestep for all modules.
        driver_1.Advance(step_size);
        driver_2.Advance(step_size);
        vehicle_1.Advance(step_size);
        vehicle_2.Advance(step_size);
        terrain.Advance(step_size);
        if (vis)
            vis->Advance(step_size);

        // Advance state of entire system (containing both vehicles)
        sys.DoStepDynamics(step_size);
    }
}

// =============================================================================

int main(int argc, char* argv[]) {
    cout << "Copyright (c) 2025 projectchrono.org\nChrono version: " << CHRONO_VERSION << endl;

    // Vehicle models
    auto v1 = chrono_types::make_shared<HMMWV_Model>();
    double speed1 = 10;
    ChFramed pose1(ChVector3d(50, 50, 0), QUNIT);

    auto v2 = chrono_types::make_shared<Gator_Model>();
    double speed2 = 5;
    ChFramed pose2(ChVector3d(46, 46, 0), QuatFromAngleZ(CH_PI_4));

    // Create output directories
    std::string out_dir = GetChronoOutputPath() + "VEHICLE_CHECKPOINT";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    auto dir1 = out_dir + "/" + v1->ModelName();
    if (!filesystem::create_directory(filesystem::path(dir1))) {
        cout << "Error creating directory " << dir1 << endl;
        return 1;
    }

    auto dir2 = out_dir + "/" + v2->ModelName();
    if (!filesystem::create_directory(filesystem::path(dir2))) {
        cout << "Error creating directory " << dir2 << endl;
        return 1;
    }

    // Simulate first vehicle to its end pose and save final checkpoint
    SimulateSingle(v1, pose1, speed1, dir1);

    // Simulate second vehicle to its end pose and save final checkpoint
    SimulateSingle(v2, pose2, speed2, dir2);

    // Initialize the two vehicles from checkpoints and simulate simultaneously
    SimulateBoth(v1, v2, dir1, dir2);

    return 0;
}
