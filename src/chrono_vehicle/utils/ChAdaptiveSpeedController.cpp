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
// Utility class implementing PID adaptive cruise speed controllers. The base
// class implements the basic functionality to control the error between the
// location of a sentinel point (a point at a look-ahead distance in front of the
// vehicle) and the current target point.
// Derived classes differ in how they specify the target point.  This can be the
// closest point to the sentinel point on a predefined curve path (currently
// using a ChBezierCurve) or from some other external sources (e.g. interfacing
// with a camera sensor).
//
// An object of this type can be used within a Chrono::Vehicle driver model to
// provide the steering output.
//
// =============================================================================

#include <cstdio>

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/utils/ChAdaptiveSpeedController.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the class ChAdaptiveSpeedController
// -----------------------------------------------------------------------------
ChAdaptiveSpeedController::ChAdaptiveSpeedController()
    : m_speed(0), m_err(0), m_errd(0), m_erri(0), m_csv(nullptr), m_collect(false) {
    // Default PID controller gains all zero (no control).
    SetGains(0, 0, 0);
}

ChAdaptiveSpeedController::ChAdaptiveSpeedController(const std::string& filename)
    : m_speed(0), m_err(0), m_errd(0), m_erri(0), m_csv(nullptr), m_collect(false) {
    Document d;
    ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    m_Kp = d["Gains"]["Kp"].GetDouble();
    m_Ki = d["Gains"]["Ki"].GetDouble();
    m_Kd = d["Gains"]["Kd"].GetDouble();

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ChAdaptiveSpeedController::~ChAdaptiveSpeedController() {
    delete m_csv;
}

void ChAdaptiveSpeedController::Reset(const ChFrameMoving<>& ref_frame) {
    m_speed = Vdot(ref_frame.GetPos_dt(), ref_frame.GetA().Get_A_Xaxis());
    m_err = 0;
    m_erri = 0;
    m_errd = 0;
}

double ChAdaptiveSpeedController::Advance(const ChFrameMoving<>& ref_frame,
                                          double target_speed,
                                          double target_following_time,
                                          double target_min_distance,
                                          double current_distance,
                                          double time,
                                          double step) {
    // Current vehicle speed.
    m_speed = Vdot(ref_frame.GetPos_dt(), ref_frame.GetA().Get_A_Xaxis());

    double desired_gap = target_speed / target_following_time;
    double distance = current_distance;
    if (desired_gap > target_min_distance) {
        // Scale the distance so it goes to 0 at distance = target_min_distance
        // & desired_gap at distance = desired_gap
        distance = (desired_gap * distance - target_min_distance) / (desired_gap - target_min_distance);
    }
    double equivalent_speed = distance / target_following_time;

    // Clamp the equivalent_speed between 0 & the target speed.  That way when the vehicle
    // in front is far away, the controller behaves like a regular cruise controller
    equivalent_speed = equivalent_speed < 0 ? 0 : equivalent_speed;
    equivalent_speed = equivalent_speed > target_speed ? target_speed : equivalent_speed;

    // If data collection is enabled, append current target and sentinel locations.
    if (m_collect) {
        *m_csv << time << target_speed << m_speed << std::endl;
    }

    // Calculate current error.
    double err = equivalent_speed - m_speed;

    // Estimate error derivative (backward FD approximation).
    m_errd = (err - m_err) / step;

    // Calculate current error integral (trapezoidal rule).
    m_erri += (err + m_err) * step / 2;

    // Cache new error
    m_err = err;

    // Return PID output (steering value)
    if (distance > target_min_distance)
        return m_Kp * m_err + m_Ki * m_erri + m_Kd * m_errd;
    else {
        // Apply full brakes to bring the vehicle to a complete stop if the
        // current distance is less than the min distance
        // With the normal PID, the vehicle may not "completely" stop
        return -1;
    }
}

void ChAdaptiveSpeedController::StartDataCollection() {
    // Return now if currently collecting data.
    if (m_collect)
        return;
    // Create the CSV_writer object if needed (first call to this function).
    if (!m_csv) {
        m_csv = new utils::CSV_writer("\t");
        m_csv->stream().setf(std::ios::scientific | std::ios::showpos);
        m_csv->stream().precision(6);
    }
    // Enable data collection.
    m_collect = true;
}

void ChAdaptiveSpeedController::StopDataCollection() {
    // Suspend data collection.
    m_collect = false;
}

void ChAdaptiveSpeedController::WriteOutputFile(const std::string& filename) {
    // Do nothing if data collection was never enabled.
    if (m_csv)
        m_csv->write_to_file(filename);
}

}  // end namespace vehicle
}  // end namespace chrono
