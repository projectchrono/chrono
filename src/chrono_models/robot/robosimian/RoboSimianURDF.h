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
// Author: Radu Serban
// =============================================================================
//
// RoboSimian model constructed from URDF specification.
//
// For a description of this robot, see:
//  Satzinger B.W., Lau C., Byl M., Byl K. (2016)
//  Experimental Results for Dexterous Quadruped Locomotion Planning with RoboSimian.
//  In: Hsieh M., Khatib O., Kumar V. (eds) Experimental Robotics.
//  Springer Tracts in Advanced Robotics, vol 109. Springer, Cham.
//  https://doi.org/10.1007/978-3-319-23778-7_3
//
// =============================================================================

#ifndef ROBOSIMIAN_URDF_H
#define ROBOSIMIAN_URDF_H

#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/input_output/ChCheckpointASCII.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_parsers/urdf/ChParserURDF.h"

#include "chrono_models/robot/ChRobotActuation.h"
#include "chrono_models/robot/robosimian/RoboSimian.h"

namespace chrono {
namespace robosimian {

/// @addtogroup robot_models_robosimian
/// @{

/// RoboSimian robot model constructed from URDF specification file.
/// The robot model consists of a chassis (torso) with an attached sled and four limbs (legs).
class CH_MODELS_API RoboSimianURDF {
  public:
    RoboSimianURDF();

    void SetVerbose(bool val) { m_verbose = val; }

    void SetOutputDirectory(const std::string& dir) { m_out_dir = dir; }

    /// Set initial robot pose.
    void SetInitialPose(const ChFramed& frame) { m_init_pose = frame; }

    /// Set locomotion mode.
    void SetLocomotionMode(LocomotionMode mode) { m_locomotion_mode = mode; }

    /// Set time interval to assume start pose.
    void SetDurationStartPose(double dt) { m_duration_pose = dt; }

    /// Set time interval to hold the start pose.
    void SetDurationHold(double dt) { m_duration_hold = dt; }

    /// Get the locomotion mode.
    LocomotionMode GetLocomotionMode() const { return m_locomotion_mode; }

    /// Construct the RoboSimian robot in the specified Chrono system.
    void Construct(ChSystem& sys);

    /// Get the robot torso body;
    std::shared_ptr<ChBody> GetTorsoBody() const { return m_torso; }

    /// Get the robot sled body.
    std::shared_ptr<ChBody> GetSledBody() const { return m_sled; }

    /// Get a vector with all robot wheel bodies.
    std::vector<std::shared_ptr<ChBody>> GetWheelBodies() const { return m_wheels; }

    /// Get the position of the specified wheel.
    const ChVector3d& GetWheelPos(int i) const { return m_wheels[i]->GetPos(); }

    /// Get an approximate wheel radius.
    /// This is the X-dimension of the AABB of the wheel collision model.
    double GetWheelRadius() const { return m_wheel_radius; }

    /// Get the sled collision geometry.
    std::shared_ptr<utils::ChBodyGeometry> GetSledGeometry() const { return m_sled_geometry; }

    /// Get the wheel collision geometry.
    std::shared_ptr<utils::ChBodyGeometry> GetWheelGeometry() const { return m_wheel_geometry; }

    void FixTorso(bool fixed) const { m_root->SetFixed(fixed); }

    void SimulateToStartPose(double step_size, std::shared_ptr<ChVisualSystem> vis = nullptr);

    /// Initialize the robot with data from the checkpoint file.
    /// The robot must be constructed and a checkpoint file must exist (e.g., created by SimulateToStartPose()).
    void LoadCheckpoint();

    /// Update robot actuation at the current time.
    /// This function sets the values for all motor functions.
    void UpdateActuation(double time);

    /// Set the current locomotion phase.
    void SetLocomotionPhase(models::ChRobotActuation::Phase phase);

  private:
    ChSystem* m_sys;

    bool m_verbose;

    ChFramed m_init_pose;
    double m_duration_pose;
    double m_duration_hold;

    std::unique_ptr<parsers::ChParserURDF> m_robot;
    std::shared_ptr<ChBodyAuxRef> m_root;

    std::shared_ptr<ChBody> m_torso;
    std::shared_ptr<ChBody> m_sled;
    std::vector<std::shared_ptr<ChBody>> m_wheels;
    double m_wheel_radius;
    double wheel_bottom;

    std::shared_ptr<utils::ChBodyGeometry> m_sled_geometry;
    std::shared_ptr<utils::ChBodyGeometry> m_wheel_geometry;

    int m_num_motors;
    LocomotionMode m_locomotion_mode;
    std::unique_ptr<models::ChRobotActuation> m_robot_actuation;
    std::vector<std::shared_ptr<ChFunctionSetpoint>> m_motor_functions;

    std::string m_out_dir;
    std::string m_checkpoint_file;
    std::unique_ptr<ChCheckpointASCII> m_checkpoint;
};

/// @} robot_models_robosimian

}  // namespace robosimian
}  // namespace chrono

#endif
