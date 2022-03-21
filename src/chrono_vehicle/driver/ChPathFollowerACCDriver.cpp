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
// Authors: Mike Taylor, Radu Serban
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
#include "chrono/assets/ChLineShape.h"
#include "chrono/geometry/ChLineBezier.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

namespace chrono {
namespace vehicle {

ChPathFollowerACCDriver::ChPathFollowerACCDriver(ChVehicle& vehicle,
                                                 std::shared_ptr<ChBezierCurve> path,
                                                 const std::string& path_name,
                                                 double target_speed,
                                                 double target_following_time,
                                                 double target_min_distance,
                                                 double current_distance,
                                                 bool isClosedPath)
    : ChDriver(vehicle),
      m_steeringPID(path, isClosedPath),
      m_target_speed(target_speed),
      m_target_following_time(target_following_time),
      m_target_min_distance(target_min_distance),
      m_current_distance(current_distance),
      m_pathName(path_name),
      m_throttle_threshold(0.2) {
    Create();
}

ChPathFollowerACCDriver::ChPathFollowerACCDriver(ChVehicle& vehicle,
                                                 const std::string& steering_filename,
                                                 const std::string& speed_filename,
                                                 std::shared_ptr<ChBezierCurve> path,
                                                 const std::string& path_name,
                                                 double target_speed,
                                                 double target_following_time,
                                                 double target_min_distance,
                                                 double current_distance,
                                                 bool isClosedPath)
    : ChDriver(vehicle),
      m_steeringPID(steering_filename, path, isClosedPath),
      m_speedPID(speed_filename),
      m_target_speed(target_speed),
      m_target_following_time(target_following_time),
      m_target_min_distance(target_min_distance),
      m_current_distance(current_distance),
      m_pathName(path_name),
      m_throttle_threshold(0.2) {
    Create();
}

void ChPathFollowerACCDriver::Create() {
    // Reset the steering and speed controllers
    m_steeringPID.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);

    // Create a fixed body to carry a visualization asset for the path
    auto road = std::shared_ptr<ChBody>(m_vehicle.GetSystem()->NewBody());
    road->SetBodyFixed(true);
    m_vehicle.GetSystem()->AddBody(road);

    auto bezier_curve = m_steeringPID.GetPath();
    auto num_points = static_cast<unsigned int>(bezier_curve->getNumPoints());
    auto path_asset = chrono_types::make_shared<ChLineShape>();
    path_asset->SetLineGeometry(chrono_types::make_shared<geometry::ChLineBezier>(bezier_curve));
    path_asset->SetColor(ChColor(0.8f, 0.8f, 0.0f));
    path_asset->SetName(m_pathName);
    path_asset->SetNumRenderPoints(std::max<unsigned int>(2 * num_points, 400));

    road->AddVisualShape(path_asset);
}

void ChPathFollowerACCDriver::Reset() {
    m_steeringPID.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);
}

void ChPathFollowerACCDriver::Advance(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID.Advance(m_vehicle, m_target_speed, m_target_following_time, m_target_min_distance,
                                          m_current_distance, step);
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

void ChPathFollowerACCDriver::ExportPathPovray(const std::string& out_dir) {
    utils::WriteCurvePovray(*m_steeringPID.GetPath(), m_pathName, out_dir, 0.04, ChColor(0.8f, 0.5f, 0.0f));
}

}  // end namespace vehicle
}  // end namespace chrono
