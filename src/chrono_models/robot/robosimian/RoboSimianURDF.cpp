// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// RoboSimian model constructed from URDF specification.
//
// =============================================================================

#include "chrono_models/robot/robosimian/RoboSimianURDF.h"

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::utils;

namespace chrono {
namespace robosimian {

RoboSimianURDF::RoboSimianURDF()
    : m_verbose(false),
      m_duration_pose(1.0),
      m_duration_hold(0.5),
      m_num_motors(32),
      m_locomotion_mode(LocomotionMode::DRIVE),
      m_out_dir("."),
      m_checkpoint_file("/checkpoint.txt") {}

void RoboSimianURDF::Construct(ChSystem& sys) {
    m_sys = &sys;

    // Create parser instance
    m_robot = chrono_types::make_unique<ChParserURDF>(GetChronoDataFile("robot/robosimian/rs.urdf"));

    // Set root body pose
    m_robot->SetRootInitPose(ChFrame<>(ChVector3d(0, 0, 1.5), QUNIT));

    // Make all eligible joints as actuated (POSITION type) and
    // overwrite wheel motors with SPEED actuation.
    m_robot->SetAllJointsActuationType(ChParserURDF::ActuationType::POSITION);
    m_robot->SetJointActuationType("limb1_joint8", ChParserURDF::ActuationType::SPEED);
    m_robot->SetJointActuationType("limb2_joint8", ChParserURDF::ActuationType::SPEED);
    m_robot->SetJointActuationType("limb3_joint8", ChParserURDF::ActuationType::SPEED);
    m_robot->SetJointActuationType("limb4_joint8", ChParserURDF::ActuationType::SPEED);

    // Use convex hull for the sled collision shape
    m_robot->SetBodyMeshCollisionType("sled", ChParserURDF::MeshCollisionType::CONVEX_HULL);

    // Optional: visualize collision shapes
    ////m_robot->EnableCollisionVisualization();

    // Report parsed elements
    if (m_verbose) {
        m_robot->PrintModelBodies();
        m_robot->PrintModelJoints();
    }

    // Create the Chrono model
    m_robot->PopulateSystem(sys);

    // Get selected bodies of the robot
    m_wheels.resize(4);

    m_root = m_robot->GetRootChBody();
    m_torso = m_robot->GetChBody("torso");
    m_sled = m_robot->GetChBody("sled");
    m_wheels[0] = m_robot->GetChBody("limb1_link8");
    m_wheels[1] = m_robot->GetChBody("limb2_link8");
    m_wheels[2] = m_robot->GetChBody("limb3_link8");
    m_wheels[3] = m_robot->GetChBody("limb4_link8");

    m_sled_geometry = m_robot->GetCollisionGeometry("sled");
    m_wheel_geometry = m_robot->GetCollisionGeometry("limb1_link8");
    m_wheel_radius = m_wheel_geometry->CalculateAABB().Size().x() / 2;

    // Enable collision and set contact material for selected bodies of the robot
    ChContactMaterialData mat;
    mat.mu = 0.8f;
    mat.cr = 0.0f;
    mat.Y = 1e7f;
    auto cmat = mat.CreateMaterial(sys.GetContactMethod());

    m_sled->EnableCollision(true);
    m_sled->GetCollisionModel()->SetAllShapesMaterial(cmat);

    for (auto& wheel : m_wheels) {
        wheel->EnableCollision(true);
        wheel->GetCollisionModel()->SetAllShapesMaterial(cmat);
    }

    // Read the list of actuated motors, cache the motor links, and set their actuation function
    std::ifstream ifs(GetChronoDataFile("robot/robosimian/actuation/motor_names.txt"));
    m_motor_functions.resize(m_num_motors);
    for (int i = 0; i < m_num_motors; i++) {
        std::string name;
        ifs >> name;
        auto motor = m_robot->GetChMotor(name);
        m_motor_functions[i] = chrono_types::make_shared<ChFunctionSetpoint>();
        motor->SetMotorFunction(m_motor_functions[i]);
    }

    // Actuation input files
    std::string start_filename;
    std::string cycle_filename;
    std::string stop_filename;

    switch (m_locomotion_mode) {
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
    m_robot_actuation = chrono_types::make_unique<models::ChRobotActuation>(32,              // number motors
                                                                            start_filename,  // start input file
                                                                            cycle_filename,  // cycle input file
                                                                            stop_filename,   // stop input file
                                                                            true             // repeat cycle
    );
    m_robot_actuation->SetTimeOffsets(m_duration_pose, m_duration_hold);
    m_robot_actuation->SetVerbose(m_verbose);

    // Set checkpoint file name
    m_checkpoint_file = "/checkpoint." + LocomotionModeAsString(m_locomotion_mode) + ".txt";
}

void RoboSimianURDF::UpdateActuation(double time) {
    m_robot_actuation->Update(time);
    const auto& actuations = m_robot_actuation->GetActuation();
    for (int i = 0; i < m_num_motors; i++)
        m_motor_functions[i]->SetSetpoint(-actuations[i], time);
}

void RoboSimianURDF::SetLocomotionPhase(models::ChRobotActuation::Phase phase) {
    m_robot_actuation->SetPhase(phase);
}

void RoboSimianURDF::SimulateToStartPose(double step_size, std::shared_ptr<ChVisualSystem> vis) {
    FixTorso(true);

    ChVector3d camera_lookat = m_torso->GetPos();
    ChVector3d camera_loc = camera_lookat + ChVector3d(3, 3, 0);
    if (vis)
        vis->UpdateCamera(camera_loc, camera_lookat);

    while (true) {
        double time = m_sys->GetChTime();

        if (vis) {
            if (!vis->Run())
                break;
            vis->Render();
        }

        // Stop when reaching start pose, record lowest point, and write checkpoint
        if (m_sys->GetChTime() > m_duration_pose) {
            wheel_bottom = m_wheels[0]->GetPos().z() - m_wheel_radius;
            m_checkpoint = chrono_types::make_unique<ChCheckpointASCII>(ChCheckpoint::Type::SYSTEM);
            m_checkpoint->Save(m_sys);
            m_checkpoint->WriteFile(m_out_dir + "/" + m_checkpoint_file);
            break;
        }

        // Update actuation
        UpdateActuation(time);

        // Advance dynamics
        m_sys->DoStepDynamics(step_size);
    }
}

void RoboSimianURDF::LoadCheckpoint() {
    if (!m_checkpoint)
        m_checkpoint = chrono_types::make_unique<ChCheckpointASCII>(ChCheckpoint::Type::SYSTEM);
    m_checkpoint->ReadFile(m_out_dir + "/" + m_checkpoint_file);
    m_checkpoint->Load(m_sys);
}

}  // end namespace robosimian
}  // namespace chrono
