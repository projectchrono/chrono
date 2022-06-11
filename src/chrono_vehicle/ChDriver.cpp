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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDriver::ChDriver(ChVehicle& vehicle)
    : m_vehicle(vehicle), m_throttle(0), m_steering(0), m_braking(0), m_log_filename("") {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DriverInputs ChDriver::GetInputs() const {
    return { m_steering, m_throttle, m_braking };
}

// -----------------------------------------------------------------------------
// Initialize output file for recording deriver inputs.
// -----------------------------------------------------------------------------
bool ChDriver::LogInit(const std::string& filename) {
    m_log_filename = filename;

    std::ofstream ofile(filename.c_str(), std::ios::out);
    if (!ofile)
        return false;

    ofile << "Time\tSteering\tThrottle\tBraking\n";
    ofile.close();
    return true;
}

// -----------------------------------------------------------------------------
// Record the current driver inputs to the log file.
// -----------------------------------------------------------------------------
bool ChDriver::Log(double time) {
    if (m_log_filename.empty())
        return false;

    std::ofstream ofile(m_log_filename.c_str(), std::ios::app);
    if (!ofile)
        return false;

    ofile << time << "\t" << m_steering << "\t" << m_throttle << "\t" << m_braking << std::endl;
    ofile.close();
    return true;
}

// -----------------------------------------------------------------------------
// Clamp a specified input value to appropriate interval.
// -----------------------------------------------------------------------------
void ChDriver::SetSteering(double val, double min_val, double max_val) {
    m_steering = ChClamp(val, min_val, max_val);
}

void ChDriver::SetThrottle(double val, double min_val, double max_val) {
    m_throttle = ChClamp(val, min_val, max_val);
}

void ChDriver::SetBraking(double val, double min_val, double max_val) {
    m_braking = ChClamp(val, min_val, max_val);
}

}  // end namespace vehicle
}  // end namespace chrono
