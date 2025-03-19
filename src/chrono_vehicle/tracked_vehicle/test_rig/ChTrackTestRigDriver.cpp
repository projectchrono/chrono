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
// Base class for a track test rig driver. A driver system must be able to
// report the current values of the inputs (throttle and post displacements).
//
// =============================================================================

#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>

#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigDriver.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackTestRigDriver::ChTrackTestRigDriver() : m_throttle(0), m_delay(0), m_log_filename("") {}

void ChTrackTestRigDriver::Initialize(size_t num_posts, const std::vector<double>& locations) {
    m_nposts = num_posts;
    m_displ.resize(num_posts, 0.0);
    m_displSpeed.resize(num_posts, 0.0);
    m_locations = locations;
}

void ChTrackTestRigDriver::Synchronize(double time) {
    m_time = time;
}

bool ChTrackTestRigDriver::Started() const {
    return m_time > m_delay;
}

// -----------------------------------------------------------------------------
// Initialize output file for recording deriver inputs.
// -----------------------------------------------------------------------------
bool ChTrackTestRigDriver::LogInit(const std::string& filename) {
    m_log_filename = filename;

    std::ofstream ofile(filename, std::ios::out);
    if (!ofile)
        return false;

    ofile << "Time\t";
    for (int i = 0; i < m_displ.size(); i++)
        ofile << "Displ" << i << "\t";
    ofile << "Throttle\n";
    ofile.close();

    return true;
}

// -----------------------------------------------------------------------------
// Record the current driver inputs to the log file.
// -----------------------------------------------------------------------------
bool ChTrackTestRigDriver::Log(double time) {
    if (m_log_filename.empty())
        return false;

    std::ofstream ofile(m_log_filename, std::ios::app);
    if (!ofile)
        return false;

    ofile << time << "\t";
    for (int i = 0; i < m_displ.size(); i++)
        ofile << m_displ[i] << "\t";
    ofile << m_throttle << std::endl;
    ofile.close();

    return true;
}

// -----------------------------------------------------------------------------
// Clamp a specified input value to appropriate interval.
// -----------------------------------------------------------------------------
void ChTrackTestRigDriver::SetDisplacement(int index, double val, double min_val, double max_val) {
    m_displ[index] = ChClamp(val, min_val, max_val);
}

void ChTrackTestRigDriver::SetThrottle(double val, double min_val, double max_val) {
    m_throttle = ChClamp(val, min_val, max_val);
}

}  // end namespace vehicle
}  // end namespace chrono
