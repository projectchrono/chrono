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
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDriver.h"

namespace chrono {
namespace vehicle {

ChSuspensionTestRigDriver::ChSuspensionTestRigDriver() : m_time(0), m_steering(0), m_delay(0), m_log_filename("") {}

void ChSuspensionTestRigDriver::Synchronize(double time) {
    m_time = time;
}

bool ChSuspensionTestRigDriver::Started() const {
    return m_time > m_delay;
}

void ChSuspensionTestRigDriver::Initialize(int naxles) {
    m_naxles = naxles;
    m_displLeft.resize(naxles, 0.0);
    m_displRight.resize(naxles, 0.0);
    m_displSpeedLeft.resize(naxles, 0.0);
    m_displSpeedRight.resize(naxles, 0.0);
}

// Initialize output file for recording driver inputs.
bool ChSuspensionTestRigDriver::LogInit(const std::string& filename) {
    m_log_filename = filename;

    std::ofstream ofile(filename.c_str(), std::ios::out);
    if (!ofile)
        return false;

    ofile << "Time\tDisplLeft\tDisplRight\tSteering\n";
    ofile.close();
    return true;
}

// Record the current driver inputs to the log file.
bool ChSuspensionTestRigDriver::Log(double time) {
    if (m_log_filename.empty())
        return false;

    std::ofstream ofile(m_log_filename.c_str(), std::ios::app);
    if (!ofile)
        return false;

    ofile << time << "\t" << m_steering << "\t";
    for (int ia = 0; ia < m_naxles; ia++) {
        ofile << m_displLeft[ia] << "\t" << m_displRight[ia] << "\t";
    }
    ofile << std::endl;

    ofile.close();
    return true;
}

// Clamp a specified input value to appropriate interval.
void ChSuspensionTestRigDriver::SetDisplacementLeft(int axle, double val, double min_val, double max_val) {
    m_displLeft[axle] = ChClamp(val, min_val, max_val);
}

void ChSuspensionTestRigDriver::SetDisplacementRight(int axle, double val, double min_val, double max_val) {
    m_displRight[axle] = ChClamp(val, min_val, max_val);
}

void ChSuspensionTestRigDriver::SetSteering(double val, double min_val, double max_val) {
    m_steering = ChClamp(val, min_val, max_val);
}

}  // end namespace vehicle
}  // end namespace chrono
