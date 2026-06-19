// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Demo: integrating ROS with ChParserURDF. A RoboSimian robot is parsed from
// URDF, actuated through a walking cycle, and its model + transforms are
// published to ROS so it can be visualized in RViz/Foxglove:
//
//   /robot_description  (std_msgs/String, latched)  - the URDF, for the robot_state model
//   /tf                 (tf2_msgs/TFMessage)        - per-link transforms
//   /clock              (rosgraph_msgs/Clock)
//
// A Chrono VSG/Irrlicht window is also opened (as in 9.0/10.0); on a headless
// host it degrades to running without the window.
//
// =============================================================================

#include "chrono/core/ChTypes.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/functions/ChFunctionSetpoint.h"

#include "chrono_parsers/ChParserURDF.h"
#include "chrono_parsers/ChRobotActuation.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include <fstream>
#include <string>
#include <vector>

using namespace chrono;
using namespace chrono::ros;
using namespace chrono::parsers;

// Run-time visualization system (IRRLICHT or VSG).
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// =============================================================================

void CreateTerrain(ChSystem& sys, std::shared_ptr<ChBody> ground, double length, double width, double height, double offset) {
    auto ground_mat = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());
    ground_mat->SetFriction(0.8f);
    ground_mat->SetRestitution(0.0f);
    if (sys.GetContactMethod() == ChContactMethod::SMC)
        std::static_pointer_cast<ChContactMaterialSMC>(ground_mat)->SetYoungModulus(1e7f);

    ground->SetFixed(true);
    ground->SetPos(ChVector3d(offset, 0, height - 0.1));
    ground->EnableCollision(true);

    auto ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(ground_mat, length, width, 0.2);
    ground->AddCollisionShape(ct_shape);

    auto box = chrono_types::make_shared<ChVisualShapeBox>(length, width, 0.2);
    box->SetTexture(GetChronoDataFile("textures/checker2.png"), (float)length, (float)width);
    ground->AddVisualShape(box);
    sys.GetCollisionSystem()->BindItem(ground);
}

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl << std::endl;

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration({0, 0, -9.81});
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.GetSolver()->AsIterative()->SetMaxIterations(200);

    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetName("floor");
    sys.AddBody(floor);

    // Parse the RoboSimian URDF and build the Chrono model.
    const std::string robot_urdf = GetChronoDataFile("robot/robosimian/rs.urdf");
    ChParserURDF robot(robot_urdf);
    robot.SetRootInitPose(ChFrame<>(ChVector3d(0, 0, 1.5), QUNIT));
    robot.SetAllJointsActuationType(ChParserURDF::ActuationType::POSITION);
    robot.SetJointActuationType("limb1_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb2_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb3_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetJointActuationType("limb4_joint8", ChParserURDF::ActuationType::SPEED);
    robot.SetBodyMeshCollisionType("sled", ChParserURDF::MeshCollisionType::CONVEX_HULL);
    robot.EnableCollisionVisualization();
    robot.PopulateSystem(sys);

    auto torso = robot.GetChBody("torso");
    auto sled = robot.GetChBody("sled");
    auto limb1_wheel = robot.GetChBody("limb1_link8");
    auto limb2_wheel = robot.GetChBody("limb2_link8");
    auto limb3_wheel = robot.GetChBody("limb3_link8");
    auto limb4_wheel = robot.GetChBody("limb4_link8");

    sled->EnableCollision(true);
    limb1_wheel->EnableCollision(true);
    limb2_wheel->EnableCollision(true);
    limb3_wheel->EnableCollision(true);
    limb4_wheel->EnableCollision(true);

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

    robot.GetRootChBody()->SetFixed(true);

    // Read the actuated motors and attach a setpoint function to each.
    std::ifstream ifs(GetChronoDataFile("robot/robosimian/actuation/motor_names.txt"));
    std::string name;
    std::vector<std::string> motor_names;
    while (std::getline(ifs, name)) {
        auto motor = robot.GetChMotor(name);
        motor->SetMotorFunction(chrono_types::make_shared<ChFunctionSetpoint>());
        motor_names.push_back(name);
    }

    // Drive the motors from a pre-recorded walking cycle (stands in for an
    // external controller node).
    const std::string cycle_filename = GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt");
    ChRobotActuation actuator(motor_names.size(), "", cycle_filename, "", true);
    double duration_pose = 1.0;
    double duration_settle_robot = 0.5;
    actuator.SetTimeOffsets(duration_pose, duration_settle_robot);
    actuator.SetVerbose(true);

    // ------------ ROS

    auto ros_manager = chrono_types::make_shared<ChROSManager>();
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSClockHandler>());

    // tf: a few explicit body->body transforms plus every link of the URDF tree.
    auto tf_handler = chrono_types::make_shared<ChROSTFHandler>(100);
    tf_handler->AddTransform(floor, floor->GetName(), torso, torso->GetName());
    tf_handler->AddTransform(torso, torso->GetName(), robot.GetChBody("limb1_link1"), "limb1_link1");
    tf_handler->AddTransform(torso, torso->GetName(), robot.GetChBody("limb2_link1"), "limb2_link1");
    tf_handler->AddTransform(torso, torso->GetName(), robot.GetChBody("limb3_link1"), "limb3_link1");
    tf_handler->AddTransform(torso, torso->GetName(), robot.GetChBody("limb4_link1"), "limb4_link1");
    tf_handler->AddURDF(robot);
    ros_manager->RegisterHandler(tf_handler);

    // The robot model (URDF string) on /robot_description for RViz.
    ros_manager->RegisterHandler(chrono_types::make_shared<ChROSRobotModelHandler>(robot));

    ros_manager->Initialize();

    // ------------ Visualization (degrades to no window when headless)

    std::shared_ptr<ChVisualSystem> vis;
    auto camera_lookat = torso->GetPos();
    auto camera_loc = camera_lookat + ChVector3d(3, 3, 0);
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
            vis_irr->SetWindowTitle("ROS RoboSimian URDF Demo");
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
            vis_vsg->SetBackgroundColor(ChColor(0.455f, 0.525f, 0.640f));
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->Initialize();
            vis = vis_vsg;
#endif
            break;
        }
    }

    // ------------ Simulation loop

    double time = 0;
    double step_size = 5e-4;
    double time_end = 1000;
    bool terrain_created = false;

    ChRealtimeStepTimer realtime_timer;
    while (time < time_end) {
        if (vis) {
            if (!vis->Run())
                break;
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
        }

        time = sys.GetChTime();

        // Drop the terrain in once the robot has assumed its initial pose.
        if (!terrain_created && time > duration_pose) {
            double z = limb1_wheel->GetPos().z() - 0.15;
            CreateTerrain(sys, floor, 8, 2, z, 2);
            if (vis)
                vis->BindItem(floor);
            torso->SetFixed(false);
            terrain_created = true;
        }

        actuator.Update(time);
        auto actuations = actuator.GetActuation();
        for (size_t i = 0; i < actuations.size(); i++) {
            auto motor = robot.GetChMotor(motor_names[i]);
            std::dynamic_pointer_cast<ChFunctionSetpoint>(motor->GetMotorFunction())->SetSetpoint(-actuations[i], time);
        }

        if (!ros_manager->Update(time, step_size)) {
            std::cerr << "Chrono::ROS bridge node stopped; ending simulation." << std::endl;
            break;
        }

        sys.DoStepDynamics(step_size);
        realtime_timer.Spin(step_size);
    }

    return 0;
}
