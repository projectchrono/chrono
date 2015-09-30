// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
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
//
// =============================================================================

#include "chrono/core/ChMathematics.h"

#include "generic/Generic_PathFollowerDriver.h"

using namespace chrono;

Generic_PathFollowerDriver::Generic_PathFollowerDriver(ChVehicle& vehicle, ChBezierCurve* path)
    : m_vehicle(vehicle), m_steeringPID(path), m_target_speed(0) {
    m_steeringPID.Reset(vehicle);
    m_speedPID.Reset(vehicle);
}

Generic_PathFollowerDriver::Generic_PathFollowerDriver(ChVehicle& vehicle,
                                                       const std::string& steering_filename,
                                                       const std::string& speed_filename,
                                                       ChBezierCurve* path)
    : m_vehicle(vehicle), m_steeringPID(steering_filename, path), m_speedPID(speed_filename), m_target_speed(0) {
    m_steeringPID.Reset(vehicle);
    m_speedPID.Reset(vehicle);
}

void Generic_PathFollowerDriver::Reset() {
    m_steeringPID.Reset(m_vehicle);
    m_speedPID.Reset(m_vehicle);
}

void Generic_PathFollowerDriver::Advance(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID.Advance(m_vehicle, m_target_speed, step);
    ChClampValue(out_speed, -1.0, 1.0);

    if (out_speed > 0) {
        // Vehicle moving too slow
        m_braking = 0;
        m_throttle = out_speed;
    } else if (m_throttle > 0.3) {
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
