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
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

namespace chrono {
namespace vehicle {

ChPathFollowerDriver::ChPathFollowerDriver(ChVehicle& vehicle,
                                           std::shared_ptr<ChBezierCurve> path,
                                           const std::string& path_name,
                                           double target_speed,
                                           bool isClosedPath)
    : ChDriver(vehicle),
      m_steeringPID(path, isClosedPath),
      m_pathName(path_name),
      m_target_speed(target_speed),
      m_throttle_threshold(0.2) {
    Create();
}

ChPathFollowerDriver::ChPathFollowerDriver(ChVehicle& vehicle,
                                           const std::string& steering_filename,
                                           const std::string& speed_filename,
                                           std::shared_ptr<ChBezierCurve> path,
                                           const std::string& path_name,
                                           double target_speed,
                                           bool isClosedPath)
    : ChDriver(vehicle),
      m_steeringPID(steering_filename, path, isClosedPath),
      m_speedPID(speed_filename),
      m_pathName(path_name),
      m_target_speed(target_speed),
      m_throttle_threshold(0.2) {
    Create();
}

void ChPathFollowerDriver::Create() {
    // Reset the steering and speed controllers
    m_steeringPID.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);

    // Create a fixed body to carry a visualization asset for the path
    auto road = std::shared_ptr<ChBody>(m_vehicle.GetSystem()->NewBody());
    road->SetBodyFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto bezier_curve = m_steeringPID.GetPath();
    auto num_points = static_cast<unsigned int>(bezier_curve->getNumPoints());
    auto path_asset = std::make_shared<ChLineShape>();
    path_asset->SetLineGeometry(std::make_shared<geometry::ChLineBezier>(bezier_curve));
    path_asset->SetColor(ChColor(0.0f, 0.8f, 0.0f));
    path_asset->SetName(m_pathName);
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
    road->AddAsset(path_asset);
}

void ChPathFollowerDriver::Reset() {
    m_steeringPID.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);
}

void ChPathFollowerDriver::Advance(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID.Advance(m_vehicle, m_target_speed, step);
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
    double out_steering = m_steeringPID.Advance(m_vehicle, step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

void ChPathFollowerDriver::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_steeringPID.GetPath(), m_pathName, out_dir, 0.04, ChColor(0.8f, 0.5f, 0.0f));
}

//========================================= Version with eXTended steering controller ======================

ChPathFollowerDriverXT::ChPathFollowerDriverXT(ChVehicle& vehicle,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               bool isClosedPath,
                                               double maxWheelTurnAngle)
    : ChDriver(vehicle),
      m_steeringXT(path, isClosedPath, maxWheelTurnAngle),
      m_pathName(path_name),
      m_target_speed(target_speed),
      m_throttle_threshold(0.2) {
    Create();
}

ChPathFollowerDriverXT::ChPathFollowerDriverXT(ChVehicle& vehicle,
                                               const std::string& steering_filename,
                                               const std::string& speed_filename,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               bool isClosedPath,
                                               double maxWheelTurnAngle)
    : ChDriver(vehicle),
      m_steeringXT(steering_filename, path, isClosedPath, maxWheelTurnAngle),
      m_speedPID(speed_filename),
      m_pathName(path_name),
      m_target_speed(target_speed),
      m_throttle_threshold(0.2) {
    Create();
}

void ChPathFollowerDriverXT::Create() {
    // Reset the steering and speed controllers
    m_steeringXT.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);

    // Create a fixed body to carry a visualization asset for the path
    auto road = std::shared_ptr<ChBody>(m_vehicle.GetSystem()->NewBody());
    road->SetBodyFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto bezier_curve = m_steeringXT.GetPath();
    auto num_points = static_cast<unsigned int>(bezier_curve->getNumPoints());
    auto path_asset = std::make_shared<ChLineShape>();
    path_asset->SetLineGeometry(std::make_shared<geometry::ChLineBezier>(bezier_curve));
    path_asset->SetColor(ChColor(0.0f, 0.8f, 0.0f));
    path_asset->SetName(m_pathName);
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
    road->AddAsset(path_asset);
}

void ChPathFollowerDriverXT::Reset() {
    m_steeringXT.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);
}

void ChPathFollowerDriverXT::Advance(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID.Advance(m_vehicle, m_target_speed, step);
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
    double out_steering = m_steeringXT.Advance(m_vehicle, step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

void ChPathFollowerDriverXT::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_steeringXT.GetPath(), m_pathName, out_dir, 0.04, ChColor(0.8f, 0.5f, 0.0f));
}

//========================================= Version with Simple Realistic steering controller ======================

ChPathFollowerDriverSR::ChPathFollowerDriverSR(ChVehicle& vehicle,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               bool isClosedPath,
                                               double maxWheelTurnAngle,
                                               double axle_space)
    : ChDriver(vehicle),
      m_steeringSR(path, isClosedPath, maxWheelTurnAngle, axle_space),
      m_pathName(path_name),
      m_target_speed(target_speed),
      m_throttle_threshold(0.2) {
    Create();
}

ChPathFollowerDriverSR::ChPathFollowerDriverSR(ChVehicle& vehicle,
                                               const std::string& steering_filename,
                                               const std::string& speed_filename,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               bool isClosedPath,
                                               double maxWheelTurnAngle,
                                               double axle_space)
    : ChDriver(vehicle),
      m_steeringSR(steering_filename, path, isClosedPath, maxWheelTurnAngle, axle_space),
      m_speedPID(speed_filename),
      m_pathName(path_name),
      m_target_speed(target_speed),
      m_throttle_threshold(0.2) {
    Create();
}

void ChPathFollowerDriverSR::Create() {
    // Reset the steering and speed controllers
    m_steeringSR.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);

    // Create a fixed body to carry a visualization asset for the path
    auto road = std::shared_ptr<ChBody>(m_vehicle.GetSystem()->NewBody());
    road->SetBodyFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto bezier_curve = m_steeringSR.GetPath();
    auto num_points = static_cast<unsigned int>(bezier_curve->getNumPoints());
    auto path_asset = std::make_shared<ChLineShape>();
    path_asset->SetLineGeometry(std::make_shared<geometry::ChLineBezier>(bezier_curve));
    path_asset->SetColor(ChColor(0.0f, 0.8f, 0.0f));
    path_asset->SetName(m_pathName);
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));
    road->AddAsset(path_asset);
}

void ChPathFollowerDriverSR::Reset() {
    m_steeringSR.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);
}

void ChPathFollowerDriverSR::Advance(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID.Advance(m_vehicle, m_target_speed, step);
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
    double out_steering = m_steeringSR.Advance(m_vehicle, step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;
}

void ChPathFollowerDriverSR::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_steeringSR.GetPath(), m_pathName, out_dir, 0.04, ChColor(0.8f, 0.5f, 0.0f));
}

}  // end namespace vehicle
}  // end namespace chrono
