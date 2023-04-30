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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle driver. A driver object must be able to report the
// current values of the inputs (throttle, steering, braking). To set these
// values, a concrete driver class can implement the virtual method Synchronize()
// which will be invoked at each time step.
//
// =============================================================================

#include <iostream>
#include <sstream>
#include <fstream>

#include "chrono/core/ChMathematics.h"
#include "chrono_vehicle/ChDriver.h"

namespace chrono {
namespace vehicle {

ChDriver::ChDriver(ChVehicle& vehicle)
    : m_vehicle(vehicle), m_throttle(0), m_steering(0), m_braking(0), m_clutch(0), m_log_filename("") {
}

// Get current driver inputs
DriverInputs ChDriver::GetInputs() const {
    return { m_steering, m_throttle, m_braking, m_clutch };
}

// Initialize output file for recording deriver inputs.
bool ChDriver::LogInit(const std::string& filename) {
    m_log_filename = filename;

    std::ofstream ofile(filename.c_str(), std::ios::out);
    if (!ofile)
        return false;

    ofile << "Time\tSteering\tThrottle\tBraking\tClutch\n";
    ofile.close();
    return true;
}

// Record the current driver inputs to the log file.
bool ChDriver::Log(double time) {
    if (m_log_filename.empty())
        return false;

    std::ofstream ofile(m_log_filename.c_str(), std::ios::app);
    if (!ofile)
        return false;

    ofile << time << "\t" << m_steering << "\t" << m_throttle << "\t" << m_braking << "\t" << m_clutch << std::endl;
    ofile.close();
    return true;
}

// Set driver inputs and clamp the specified value to the appropriate interval
void ChDriver::SetSteering(double steering) {
    m_steering = ChClamp(steering, -1.0, 1.0);
}
void ChDriver::SetThrottle(double throttle) {
    m_throttle = ChClamp(throttle, 0.0, 1.0);
}
void ChDriver::SetBraking(double braking) {
    m_braking = ChClamp(braking, 0.0, 1.0);
}
void ChDriver::SetClutch(double clutch) {
    m_clutch = ChClamp(clutch, 0.0, 1.0);
}


}  // end namespace vehicle
}  // end namespace chrono
