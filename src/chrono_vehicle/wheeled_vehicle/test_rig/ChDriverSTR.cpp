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
// Base class for a suspension test rig driver.A driver system must be able to
// report the current values of the inputs (steering, left post, right post).
// To set these values, a concrete driver class can implement the virtual method
// Synchronize() which will be invoked at each time step.
//
// =============================================================================

#include <iostream>
#include <sstream>
#include <fstream>

#include "chrono/core/ChMathematics.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDriverSTR.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDriverSTR::ChDriverSTR()
    : m_displLeft(0), m_displRight(0), m_steering(0), m_delay(0), m_log_filename("") {
}

// -----------------------------------------------------------------------------
// Initialize output file for recording deriver inputs.
// -----------------------------------------------------------------------------
bool ChDriverSTR::LogInit(const std::string& filename) {
    m_log_filename = filename;

    std::ofstream ofile(filename.c_str(), std::ios::out);
    if (!ofile)
        return false;

    ofile << "Time\tDisplLeft\tDisplRight\tSteering\n";
    ofile.close();
    return true;
}

// -----------------------------------------------------------------------------
// Record the current driver inputs to the log file.
// -----------------------------------------------------------------------------
bool ChDriverSTR::Log(double time) {
    if (m_log_filename.empty())
        return false;

    std::ofstream ofile(m_log_filename.c_str(), std::ios::app);
    if (!ofile)
        return false;

    ofile << time << "\t" << m_displLeft << "\t" << m_displRight << "\t" << m_steering << std::endl;
    ofile.close();
    return true;
}

// -----------------------------------------------------------------------------
// Clamp a specified input value to appropriate interval.
// -----------------------------------------------------------------------------
void ChDriverSTR::SetDisplacementLeft(double val, double min_val, double max_val) {
    m_displLeft = ChClamp(val, min_val, max_val);
}

void ChDriverSTR::SetDisplacementRight(double val, double min_val, double max_val) {
    m_displRight = ChClamp(val, min_val, max_val);
}

void ChDriverSTR::SetSteering(double val, double min_val, double max_val) {
    m_steering = ChClamp(val, min_val, max_val);
}

}  // end namespace vehicle
}  // end namespace chrono
