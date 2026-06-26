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
// Authors: Radu Serban, Rqaqiner Gericke
// =============================================================================
//
// A driver model that uses a path steering controller and a speed controller.
// The steering controller adjusts the steering input to follow the prescribed
// path.  The output from the speed controller is used to adjust throttle and
// braking inputs in order to maintain the prescribed vehicle speed.
//
// =============================================================================

#include <algorithm>

#include "chrono/assets/ChVisualShapeLine.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/input_output/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {

ChClosedLoopDriver::ChClosedLoopDriver(ChVehicle& vehicle, const std::string& path_name, double target_speed, double zero_duration, double ramp_duration)
    : ChDriver(vehicle),
      m_target_speed(target_speed),
      m_pathName(path_name),
      m_color(0.8f, 0.8f, 0.0f),
      m_throttle_threshold(0.2),
      m_zero_duration(zero_duration),
      m_ramp_duration(ramp_duration) {}

ChClosedLoopDriver::ChClosedLoopDriver(ChVehicle& vehicle,
                                       const std::string& speed_filename,
                                       const std::string& path_name,
                                       double target_speed,
                                       double zero_duration,
                                       double ramp_duration)
    : ChDriver(vehicle),
      m_target_speed(target_speed),
      m_pathName(path_name),
      m_color(0.8f, 0.8f, 0.0f),
      m_throttle_threshold(0.2),
      m_zero_duration(zero_duration),
      m_ramp_duration(ramp_duration) {}

void ChClosedLoopDriver::Initialize() {
    // Create a fixed body to carry a visualization asset for the path
    auto road = chrono_types::make_shared<ChBody>();
    road->SetFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto bezier_curve = m_steering_controller->GetPath();
    auto num_points = static_cast<unsigned int>(bezier_curve->GetNumPoints());
    auto path_asset = chrono_types::make_shared<ChVisualShapeLine>();
    path_asset->SetLineGeometry(chrono_types::make_shared<ChLineBezier>(bezier_curve));
    path_asset->SetColor(m_color);
    path_asset->SetName(m_pathName);
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));

    road->AddVisualShape(path_asset);
    if (m_vehicle.GetSystem()->GetVisualSystem()) {
        m_vehicle.GetSystem()->GetVisualSystem()->BindItem(road);
    }
}

void ChClosedLoopDriver::Reset() {
    m_speed_controller->Reset(m_vehicle.GetRefFrame());
    m_steering_controller->Reset(m_vehicle.GetRefFrame());
}

void ChClosedLoopDriver::Advance(double step) {
    double t = m_vehicle.GetChTime();
    if (t < m_zero_duration) {
        m_steering = 0;
        m_throttle = 0;
        m_braking = 1;
        return;
    }

    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speed_controller->Advance(m_vehicle.GetRefFrame(), m_target_speed, t, step);
    ChClampValue(out_speed, -1.0, 1.0);

    if (m_target_speed * out_speed > 0) {
        // Vehicle moving too slow
        m_braking = 0;
        m_throttle = std::abs(out_speed);
    } else if (m_throttle > m_throttle_threshold) {
        // Vehicle moving too fast: reduce throttle
        m_braking = 0;
        m_throttle = 1 - std::abs(out_speed);
    } else {
        // Vehicle moving too fast: apply brakes
        m_braking = std::abs(out_speed);
        m_throttle = 0;
    }

    // Set the steering value based on the output from the steering controller.
    double out_steering = m_steering_controller->Advance(m_vehicle.GetRefFrame(), t, step);
    ChClampValue(out_steering, -1.0, 1.0);
    m_steering = out_steering;

    // During the ramp-up period, only apply a fraction of controller outputs
    if (m_ramp_duration > 0) {
        t = t - m_zero_duration;
        double alpha = std::min(t / m_ramp_duration, 1.0);
        m_throttle *= alpha;
        m_steering *= alpha;
    }
}

void ChClosedLoopDriver::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_steering_controller->GetPath(), m_pathName, out_dir, 0.04, ChColor(0.8f, 0.5f, 0.0f));
}

//========================================= Version with PID steering controller ====================

ChPathFollowerDriver::ChPathFollowerDriver(ChVehicle& vehicle,
                                           std::shared_ptr<ChBezierCurve> path,
                                           const std::string& path_name,
                                           double target_speed,
                                           double zero_duration,
                                           double ramp_duration)
    : ChClosedLoopDriver(vehicle, path_name, target_speed, zero_duration, ramp_duration) {
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerPID>(path);
    Reset();
}

ChPathFollowerDriver::ChPathFollowerDriver(ChVehicle& vehicle,
                                           const std::string& steering_filename,
                                           const std::string& speed_filename,
                                           std::shared_ptr<ChBezierCurve> path,
                                           const std::string& path_name,
                                           double target_speed,
                                           double zero_duration,
                                           double ramp_duration)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed, zero_duration, ramp_duration) {
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerPID>(steering_filename, path);
    Reset();
}

ChSpeedControllerPID& ChPathFollowerDriver::GetSpeedController() const {
    return static_cast<ChSpeedControllerPID&>(*m_speed_controller);
}

ChPathSteeringControllerPID& ChPathFollowerDriver::GetSteeringController() const {
    return static_cast<ChPathSteeringControllerPID&>(*m_steering_controller);
}

//========================================= Version with eXTended steering controller ======================

ChPathFollowerDriverXT::ChPathFollowerDriverXT(ChVehicle& vehicle,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double maxWheelTurnAngle,
                                               double zero_duration,
                                               double ramp_duration)
    : ChClosedLoopDriver(vehicle, path_name, target_speed, zero_duration, ramp_duration) {
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerXT>(path, maxWheelTurnAngle);
    Reset();
}

ChPathFollowerDriverXT::ChPathFollowerDriverXT(ChVehicle& vehicle,
                                               const std::string& steering_filename,
                                               const std::string& speed_filename,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double maxWheelTurnAngle,
                                               double zero_duration,
                                               double ramp_duration)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed, zero_duration, ramp_duration) {
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerXT>(steering_filename, path, maxWheelTurnAngle);
    Reset();
}

ChSpeedControllerPID& ChPathFollowerDriverXT::GetSpeedController() const {
    return static_cast<ChSpeedControllerPID&>(*m_speed_controller);
}

ChPathSteeringControllerXT& ChPathFollowerDriverXT::GetSteeringController() const {
    return static_cast<ChPathSteeringControllerXT&>(*m_steering_controller);
}

//========================================= Version with Simple Realistic steering controller ======================

ChPathFollowerDriverSR::ChPathFollowerDriverSR(ChVehicle& vehicle,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double maxWheelTurnAngle,
                                               double axle_space,
                                               double zero_duration,
                                               double ramp_duration)
    : ChClosedLoopDriver(vehicle, path_name, target_speed, zero_duration, ramp_duration) {
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerSR>(path, maxWheelTurnAngle, axle_space);
    Reset();
}

ChPathFollowerDriverSR::ChPathFollowerDriverSR(ChVehicle& vehicle,
                                               const std::string& steering_filename,
                                               const std::string& speed_filename,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double maxWheelTurnAngle,
                                               double axle_space,
                                               double zero_duration,
                                               double ramp_duration)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed, zero_duration, ramp_duration) {
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerSR>(steering_filename, path, maxWheelTurnAngle, axle_space);
    Reset();
}

ChSpeedControllerPID& ChPathFollowerDriverSR::GetSpeedController() const {
    return static_cast<ChSpeedControllerPID&>(*m_speed_controller);
}

ChPathSteeringControllerSR& ChPathFollowerDriverSR::GetSteeringController() const {
    return static_cast<ChPathSteeringControllerSR&>(*m_steering_controller);
}

//========================================= Version with Stanley steering controller ======================

ChPathFollowerDriverStanley::ChPathFollowerDriverStanley(ChVehicle& vehicle,
                                                         std::shared_ptr<ChBezierCurve> path,
                                                         const std::string& path_name,
                                                         double target_speed,
                                                         double maxWheelTurnAngle,
                                                         double zero_duration,
                                                         double ramp_duration)
    : ChClosedLoopDriver(vehicle, path_name, target_speed, zero_duration, ramp_duration) {
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerStanley>(path, maxWheelTurnAngle);
    Reset();
}

ChPathFollowerDriverStanley::ChPathFollowerDriverStanley(ChVehicle& vehicle,
                                                         const std::string& steering_filename,
                                                         const std::string& speed_filename,
                                                         std::shared_ptr<ChBezierCurve> path,
                                                         const std::string& path_name,
                                                         double target_speed,
                                                         double maxWheelTurnAngle,
                                                         double zero_duration,
                                                         double ramp_duration)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed, zero_duration, ramp_duration) {
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerStanley>(steering_filename, path, maxWheelTurnAngle);
    Reset();
}

ChSpeedControllerPID& ChPathFollowerDriverStanley::GetSpeedController() const {
    return static_cast<ChSpeedControllerPID&>(*m_speed_controller);
}

ChPathSteeringControllerStanley& ChPathFollowerDriverStanley::GetSteeringController() const {
    return static_cast<ChPathSteeringControllerStanley&>(*m_steering_controller);
}

//========================================= Pure Pursuit ======================

ChPathFollowerDriverPP::ChPathFollowerDriverPP(chrono::vehicle::ChVehicle& vehicle,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double zero_duration,
                                               double ramp_duration)
    : ChClosedLoopDriver(vehicle, path_name, target_speed, zero_duration, ramp_duration) {
    ChWheeledVehicle* veh = dynamic_cast<ChWheeledVehicle*>(&vehicle);
    if (veh == nullptr) {
        std::cout << "The Pure Pursuit Controller can actually not be used with tracked vehicles!" << std::endl;
        exit(99);
    }
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerPP>(path, veh->GetMaxSteeringAngle(), veh->GetWheelbase());
    Reset();
}

ChPathFollowerDriverPP::ChPathFollowerDriverPP(ChVehicle& vehicle,
                                               const std::string& steering_filename,
                                               const std::string& speed_filename,
                                               std::shared_ptr<ChBezierCurve> path,
                                               const std::string& path_name,
                                               double target_speed,
                                               double zero_duration,
                                               double ramp_duration)
    : ChClosedLoopDriver(vehicle, speed_filename, path_name, target_speed, zero_duration, ramp_duration) {
    ChWheeledVehicle* veh = dynamic_cast<ChWheeledVehicle*>(&vehicle);
    if (veh == nullptr) {
        std::cout << "The Pure Pursuit Controller can actually not be used with tracked vehicles!" << std::endl;
        exit(99);
    }
    m_speed_controller = chrono_types::make_unique<ChSpeedControllerPID>();
    m_steering_controller = chrono_types::make_unique<ChPathSteeringControllerPP>(steering_filename, path, veh->GetMaxSteeringAngle(), veh->GetWheelbase());
    Reset();
}

ChSpeedControllerPID& ChPathFollowerDriverPP::GetSpeedController() const {
    return static_cast<ChSpeedControllerPID&>(*m_speed_controller);
}

ChPathSteeringControllerPP& ChPathFollowerDriverPP::GetSteeringController() const {
    return static_cast<ChPathSteeringControllerPP&>(*m_steering_controller);
}

}  // end namespace vehicle
}  // end namespace chrono
