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
// Authors: Radu Serban
// =============================================================================
//
// Demo for the URDF -> Chrono parser
//
// =============================================================================

////#include <float.h>
////unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/assets/ChBoxShape.h"

#include "chrono_parsers/ChParserURDF.h"
#include "chrono_parsers/ChRobotActuation.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::parsers;

// -----------------------------------------------------------------------------

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// RoboSimian locomotion mode
enum class LocomotionMode { WALK, SCULL, INCHWORM, DRIVE };
LocomotionMode mode = LocomotionMode::INCHWORM;

// -----------------------------------------------------------------------------

std::shared_ptr<ChBody> CreateTerrain(ChSystem& sys, double length, double width, double height, double offset) {
    float friction = 0.8f;
    float Y = 1e7f;
    float cr = 0.0f;

    auto ground_mat = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());
    ground_mat->SetFriction(friction);
    ground_mat->SetRestitution(cr);

    if (sys.GetContactMethod() == ChContactMethod::SMC) {
        std::static_pointer_cast<ChMaterialSurfaceSMC>(ground_mat)->SetYoungModulus(Y);
    }

    auto ground = std::shared_ptr<ChBody>(sys.NewBody());
    ground->SetBodyFixed(true);
    ground->SetPos(ChVector<>(offset, 0, height - 0.1));
    ground->SetCollide(true);

    ground->GetCollisionModel()->ClearModel();
    ground->GetCollisionModel()->AddBox(ground_mat, length, width, 0.2);
    ground->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>(length, width, 0.2);
    box->SetTexture(GetChronoDataFile("textures/checker2.png"), (float)length, (float)width);
    ground->AddVisualShape(box);

    sys.AddBody(ground);

    return ground;
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create a Chrono system
    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, 0, -9.8));

    // Create parser instance
    ChParserURDF robot(GetChronoDataFile("robot/robosimian/rs.urdf"));

    // Set root body pose
    robot.SetRootInitPose(ChFrame<>(ChVector<>(0, 0, 1.5), QUNIT));

    // Make all eligible joints as actuated (POSITION type) and
    // overwrite wheel motors with SPEED actuation.
    robot.SetAllJointsActuationType(ChParserURDF::ActuationType::POSITION);
    robot.SetJointActuationType("limb1_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb2_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb3_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb4_joint8", ChParserURDF::ActuationType::SPEED);

    // Use convex hull for the sled collision shape
    robot.SetBodyMeshCollisionType("sled", ChParserURDF::MeshCollisionType::CONVEX_HULL);

    // Optional: visualize collision shapes
    ////robot.EnableCollisionVisualization();

    // Report parsed elements
    robot.PrintModelBodies();
    robot.PrintModelJoints();

    // Create the Chrono model
    robot.PopulateSystem(sys);

    // Get selected bodies of the robot
    auto torso = robot.GetChBody("torso");
    auto sled = robot.GetChBody("sled");
    auto limb1_wheel = robot.GetChBody("limb1_link8");
    auto limb2_wheel = robot.GetChBody("limb2_link8");
    auto limb3_wheel = robot.GetChBody("limb3_link8");
    auto limb4_wheel = robot.GetChBody("limb4_link8");

    // Enable collsion and set contact material for selected bodies of the robot
    sled->SetCollide(true);
    limb1_wheel->SetCollide(true);
    limb2_wheel->SetCollide(true);
    limb3_wheel->SetCollide(true);
    limb4_wheel->SetCollide(true);

    ChContactMaterialData mat;
    mat.mu = 0.8f;
    mat.cr = 0.0f;
    mat.Y = 1e7f;
    auto cmat = mat.CreateMaterial(sys.GetContactMethod());
    sled->GetCollisionModel()->SetAllShapesMaterial(cmat);
    limb1_wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);
    limb2_wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);
    limb3_wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);
    limb4_wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);

    // Fix root body
    robot.GetRootChBody()->SetBodyFixed(true);

    // Read the list of actuated motors, cache the motor links, and set their actuation function
    int num_motors = 32;
    std::ifstream ifs(GetChronoDataFile("robot/robosimian/actuation/motor_names.txt"));
    std::vector<std::shared_ptr<ChLinkMotor>> motors(num_motors);
    std::vector<std::shared_ptr<ChFunction_Setpoint>> motor_functions(num_motors);
    for (int i = 0; i < num_motors; i++) {
        std::string name;
        ifs >> name;
        motors[i] = robot.GetChMotor(name);
        motor_functions[i] = chrono_types::make_shared<ChFunction_Setpoint>();
        motors[i]->SetMotorFunction(motor_functions[i]);
    }

    // Actuation input files
    std::string start_filename;
    std::string cycle_filename;
    std::string stop_filename;

    switch (mode) {
        case LocomotionMode::WALK:
            cycle_filename = GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt");
            break;
        case LocomotionMode::SCULL:
            start_filename = GetChronoDataFile("robot/robosimian/actuation/sculling_start.txt");
            cycle_filename = GetChronoDataFile("robot/robosimian/actuation/sculling_cycle2.txt");
            stop_filename = GetChronoDataFile("robot/robosimian/actuation/sculling_stop.txt");
            break;
        case LocomotionMode::INCHWORM:
            start_filename = GetChronoDataFile("robot/robosimian/actuation/inchworming_start.txt");
            cycle_filename = GetChronoDataFile("robot/robosimian/actuation/inchworming_cycle.txt");
            stop_filename = GetChronoDataFile("robot/robosimian/actuation/inchworming_stop.txt");
            break;
        case LocomotionMode::DRIVE:
            start_filename = GetChronoDataFile("robot/robosimian/actuation/driving_start.txt");
            cycle_filename = GetChronoDataFile("robot/robosimian/actuation/driving_cycle.txt");
            stop_filename = GetChronoDataFile("robot/robosimian/actuation/driving_stop.txt");
            break;
    }

    // Create a robot motor actuation object
    ChRobotActuation actuator(32,              // number motors
                              start_filename,  // start input file
                              cycle_filename,  // cycle input file
                              stop_filename,   // stop input file
                              true             // repeat cycle
    );
    double duration_pose = 1.0;          // time interval to assume initial pose
    double duration_settle_robot = 0.5;  // time interval to allow robot settling on terrain
    actuator.SetTimeOffsets(duration_pose, duration_settle_robot);
    actuator.SetVerbose(true);

    // Create the visualization window
    std::shared_ptr<ChVisualSystem> vis;
    auto camera_lookat = torso->GetPos();
    auto camera_loc = camera_lookat + ChVector<>(3, 3, 0);
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->SetWindowSize(1200, 800);
            vis_irr->SetWindowTitle("RoboSimian URDF demo");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(camera_loc, camera_lookat);
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetWindowTitle("RoboSimian URDF demo");
            vis_vsg->AddCamera(camera_loc, camera_lookat);
            vis_vsg->SetWindowSize(ChVector2<int>(1200, 800));
            vis_vsg->SetWindowPosition(ChVector2<int>(400, 100));
            vis_vsg->SetClearColor(ChColor(0.455f, 0.525f, 0.640f));
            vis_vsg->SetUseSkyBox(false);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->SetWireFrameMode(false);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Solver settings
    sys.SetSolverMaxIterations(200);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);

    // Simulation loop
    double step_size = 5e-4;
    ChRealtimeStepTimer real_timer;
    bool terrain_created = false;

    while (vis->Run()) {
        double time = sys.GetChTime();

        // Create the terrain
        if (!terrain_created && sys.GetChTime() > duration_pose) {
            // Set terrain height
            double z = limb1_wheel->GetPos().z() - 0.15;

            // Rigid terrain parameters
            double length = 8;
            double width = 2;

            // Create terrain
            auto ground = CreateTerrain(sys, length, width, z, length / 4);
            vis->BindItem(ground);

            // Release robot
            torso->SetBodyFixed(false);

            terrain_created = true;
        }

        // Update camera location
        camera_lookat = ChVector<>(torso->GetPos().x(), torso->GetPos().y(), camera_lookat.z());
        camera_loc = camera_lookat + ChVector<>(3, 3, 0);

        vis->BeginScene();
        vis->UpdateCamera(camera_loc, camera_lookat);
        vis->Render();
        vis->EndScene();

        // Update actuator and get current actuations
        actuator.Update(time);
        const auto& actuations = actuator.GetActuation();

        ////std::cout << time << "   " << actuator.GetCurrentPhase() << std::endl;
        ////int k = 0;
        ////for (int i = 0; i < 4; i++) {
        ////    for (int j = 0; j < 8; j++) {
        ////        std::cout << actuations[k++] << " ";
        ////    }
        ////    std::cout << std::endl;
        ////}
        ////std::cout << std::endl;

        // Apply motor actuations
        for (int i = 0; i < num_motors; i++)
            motor_functions[i]->SetSetpoint(-actuations[i], time);

        // Advance system dynamics
        sys.DoStepDynamics(step_size);
        real_timer.Spin(step_size);
    }

    return 0;
}
