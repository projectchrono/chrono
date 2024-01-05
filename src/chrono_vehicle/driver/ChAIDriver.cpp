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
// Vehicle driver based on controls from some Autonomy Stack / AI.
// Currently, it is assumed that the AI provides a desired longitudinal
// acceleration and wheel angles for the front and rear axles. The underlying
// assumption is of Ackermann steering of a bicycle model.
//
// =============================================================================

#include "chrono_vehicle/driver/ChAIDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

namespace chrono {
namespace vehicle {

ChAIDriver::ChAIDriver(ChVehicle& vehicle)
    : ChDriver(vehicle), m_target_speed(0), m_last_time(0), m_last_speed(0), m_throttle_threshold(0.2) {
    m_speedPID.Reset(m_vehicle.GetRefFrame());
}

void ChAIDriver::SetSpeedControllerGains(double Kp, double Ki, double Kd) {
    m_speedPID.SetGains(Kp, Ki, Kd);
}

void ChAIDriver::Synchronize(double time) {
    std::cout << "ERROR! Incorrect version of Synchronize for ChAIDriver." << std::endl;
}

void ChAIDriver::Synchronize(double time, double long_acc, double front_axle_angle, double rear_axle_angle) {
    // Calculate target speed
    if (time <= m_last_time) {
        m_last_time = 0;
    }
    m_target_speed = m_last_speed + long_acc * (time - m_last_time);
    m_last_time = time;
    m_last_speed = m_target_speed;

    // Defer to derived class to calculate vehicle steering input
    m_steering = CalculateSteering(front_axle_angle, rear_axle_angle);
}

void ChAIDriver::Advance(double step) {
    // Set the throttle and braking values based on the output from the speed controller.
    double out_speed = m_speedPID.Advance(m_vehicle.GetRefFrame(), m_target_speed, m_vehicle.GetChTime(), step);
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
}

}  // end namespace vehicle
}  // end namespace chrono
