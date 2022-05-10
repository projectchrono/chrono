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
// Utility classes implementing PID speed controllers.
//
// An object of this type can be used within a Chrono::Vehicle driver model to
// provide the throttle/braking outputs.
//
// =============================================================================

#include <cstdio>

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/utils/ChSpeedController.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

using namespace rapidjson;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the class ChSpeedController
// -----------------------------------------------------------------------------
ChSpeedController::ChSpeedController() : m_speed(0), m_err(0), m_errd(0), m_erri(0), m_csv(nullptr), m_collect(false) {
    // Default PID controller gains all zero (no control).
    SetGains(0, 0, 0);
}

ChSpeedController::ChSpeedController(const std::string& filename)
    : m_speed(0), m_err(0), m_errd(0), m_erri(0), m_csv(nullptr), m_collect(false) {
    Document d; ReadFileJSON(filename, d);
    if (d.IsNull())
        return;

    m_Kp = d["Gains"]["Kp"].GetDouble();
    m_Ki = d["Gains"]["Ki"].GetDouble();
    m_Kd = d["Gains"]["Kd"].GetDouble();

    GetLog() << "Loaded JSON: " << filename.c_str() << "\n";
}

ChSpeedController::~ChSpeedController() {
    delete m_csv;
}

void ChSpeedController::Reset(const ChVehicle& vehicle) {
    m_speed = vehicle.GetSpeed();
    m_err = 0;
    m_erri = 0;
    m_errd = 0;
}

double ChSpeedController::Advance(const ChVehicle& vehicle, double target_speed, double step) {
    // Current vehicle speed.
    m_speed = vehicle.GetSpeed();

    // If data collection is enabled, append current target and sentinel locations.
    if (m_collect) {
        *m_csv << vehicle.GetChTime() << target_speed << m_speed << std::endl;
    }

    // Calculate current error.
    double err = target_speed - m_speed;

    // Estimate error derivative (backward FD approximation).
    m_errd = (err - m_err) / step;

    // Calculate current error integral (trapezoidal rule).
    m_erri += (err + m_err) * step / 2;

    // Cache new error
    m_err = err;

    // Return PID output (steering value)
    return m_Kp * m_err + m_Ki * m_erri + m_Kd * m_errd;
}

void ChSpeedController::StartDataCollection() {
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

void ChSpeedController::StopDataCollection() {
    // Suspend data collection.
    m_collect = false;
}

void ChSpeedController::WriteOutputFile(const std::string& filename) {
    // Do nothing if data collection was never enabled.
    if (m_csv)
        m_csv->write_to_file(filename);
}

}  // end namespace vehicle
}  // end namespace chrono
