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
// A driver model that uses a path steering controller and a speed controller.
// The steering controller adjusts the steering input to follow the prescribed
// path.  The output from the speed controller is used to adjust throttle and
// braking inputs in order to maintain the prescribed vehicle speed.
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChMathematics.h"
#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

namespace chrono {
namespace vehicle {

ChClosedLoopDriver::ChClosedLoopDriver(ChVehicle& vehicle, const std::string& path_name, double target_speed)
    : ChDriver(vehicle),
      m_target_speed(target_speed),
      m_pathName(path_name),
      m_color(0.8f, 0.8f, 0.0f),
      m_throttle_threshold(0.2) {
    m_speedPID = chrono_types::make_unique<ChSpeedController>();
}

ChClosedLoopDriver::ChClosedLoopDriver(ChVehicle& vehicle,
                                       const std::string& speed_filename,
                                       const std::string& path_name,
                                       double target_speed)
    : ChDriver(vehicle),
      m_target_speed(target_speed),
      m_pathName(path_name),
      m_color(0.8f, 0.8f, 0.0f),
      m_throttle_threshold(0.2) {
    m_speedPID = chrono_types::make_unique<ChSpeedController>(speed_filename);
}

void ChClosedLoopDriver::Initialize() {
    // Create a fixed body to carry a visualization asset for the path
    auto road = chrono_types::make_shared<ChBody>();
    road->SetBodyFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto bezier_curve = m_steeringPID->GetPath();
    auto num_points = static_cast<unsigned int>(bezier_curve->getNumPoints());
    auto path_asset = chrono_types::make_shared<ChVisualShapeLine>();
    path_asset->SetLineGeometry(chrono_types::make_shared<geometry::ChLineBezier>(bezier_curve));
    path_asset->SetColor(m_color);
    path_asset->SetName(m_pathName);
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));

    road->AddVisualShape(path_asset);
}

void ChClosedLoopDriver::Reset() {
    m_speedPID->Reset(m_vehicle.GetRefFrame());
    m_steeringPID->Reset(m_vehicle.GetRefFrame());
}

void ChClosedLoopDriver::Advance(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID->Advance(m_vehicle.GetRefFrame(), m_target_speed, m_vehicle.GetChTime(), step);
    ChClampValue(out_speed, -1.0, 1.0);

    if (out_speed > 0) {
        // Vehicle moving too slow
        m_braking = 0;
        m_throttle = out_speed;
    } else if (m_throttle > m_throttle_threshold) {
        // Vehicle moving too fast: reduce throttle
        m_braking = 0;
        m_throttle = 1 + out_speed;
    } else {
        // Vehicle moving too fast: apply brakes
        m_braking = -out_speed;
        m_throttle = 0;
    }

    // Set the steering value based on the output from the steering controller.
    double out_steering = m_steeringPID->Advance(m_vehicle.GetRefFrame(), m_vehicle.GetChTime(), step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

void ChClosedLoopDriver::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_steeringPID->GetPath(), m_pathName, out_dir, 0.04, ChColor(0.8f, 0.5f, 0.0f));
}

//========================================= Version with default PID steering controller ====================

ChPathFollowerDriver::ChPathFollowerDriver(ChVehicle& vehicle,
                                           std::shared_ptr<ChBezierCurve> path,
                                           const std::string& path_name,
                                           double target_speed)
    : ChClosedLoopDriver(vehicle, path_name, target_speed) {
    m_steeringPID = chrono_types::make_unique<ChPathSteeringController>(path);
    Reset();
}

ChPathFollowerDriver::ChPathFollowerDriver(ChVehicle& vehicle,
                                           const std::string& steering_filename,
                                           const std::string& speed_filename,
                                           std::shared_ptr<ChBezierCurve> path,
                                           const std::string& path_name,
                                           double target_speed)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed) {
    m_steeringPID = chrono_types::make_unique<ChPathSteeringController>(steering_filename, path);
    Reset();
}

ChPathSteeringController& ChPathFollowerDriver::GetSteeringController() const {
    return dynamic_cast<ChPathSteeringController&>(*m_steeringPID);
}

//========================================= Version with eXTended steering controller ======================

ChPathFollowerDriverXT::ChPathFollowerDriverXT(ChVehicle& vehicle,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double maxWheelTurnAngle)
    : ChClosedLoopDriver(vehicle, path_name, target_speed) {
    m_steeringPID = chrono_types::make_unique<ChPathSteeringControllerXT>(path, maxWheelTurnAngle);
    Reset();
}

ChPathFollowerDriverXT::ChPathFollowerDriverXT(ChVehicle& vehicle,
                                               const std::string& steering_filename,
                                               const std::string& speed_filename,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double maxWheelTurnAngle)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed) {
    m_steeringPID = chrono_types::make_unique<ChPathSteeringControllerXT>(steering_filename, path, maxWheelTurnAngle);
    Reset();
}

ChPathSteeringControllerXT& ChPathFollowerDriverXT::GetSteeringController() const {
    return dynamic_cast<ChPathSteeringControllerXT&>(*m_steeringPID);
}

//========================================= Version with Simple Realistic steering controller ======================

ChPathFollowerDriverSR::ChPathFollowerDriverSR(ChVehicle& vehicle,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double maxWheelTurnAngle,
                                               double axle_space)
    : ChClosedLoopDriver(vehicle, path_name, target_speed) {
    m_steeringPID = chrono_types::make_unique<ChPathSteeringControllerSR>(path, maxWheelTurnAngle, axle_space);
    Reset();
}

ChPathFollowerDriverSR::ChPathFollowerDriverSR(ChVehicle& vehicle,
                                               const std::string& steering_filename,
                                               const std::string& speed_filename,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double maxWheelTurnAngle,
                                               double axle_space)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed) {
    m_steeringPID =
        chrono_types::make_unique<ChPathSteeringControllerSR>(steering_filename, path, maxWheelTurnAngle, axle_space);
    Reset();
}

ChPathSteeringControllerSR& ChPathFollowerDriverSR::GetSteeringController() const {
    return dynamic_cast<ChPathSteeringControllerSR&>(*m_steeringPID);
}

//========================================= Version with Stanley steering controller ======================

ChPathFollowerDriverStanley::ChPathFollowerDriverStanley(ChVehicle& vehicle,
                                                         std::shared_ptr<ChBezierCurve> path,
                                                         const std::string& path_name,
                                                         double target_speed,
                                                         double maxWheelTurnAngle)
    : ChClosedLoopDriver(vehicle, path_name, target_speed) {
    m_steeringPID = chrono_types::make_unique<ChPathSteeringControllerStanley>(path, maxWheelTurnAngle);
    Reset();
}

ChPathFollowerDriverStanley::ChPathFollowerDriverStanley(ChVehicle& vehicle,
                                                         const std::string& steering_filename,
                                                         const std::string& speed_filename,
                                                         std::shared_ptr<ChBezierCurve> path,
                                                         const std::string& path_name,
                                                         double target_speed,
                                                         double maxWheelTurnAngle)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed) {
    m_steeringPID =
        chrono_types::make_unique<ChPathSteeringControllerStanley>(steering_filename, path, maxWheelTurnAngle);
    Reset();
}

ChPathSteeringControllerStanley& ChPathFollowerDriverStanley::GetSteeringController() const {
    return dynamic_cast<ChPathSteeringControllerStanley&>(*m_steeringPID);
}

}  // end namespace vehicle
}  // end namespace chrono
