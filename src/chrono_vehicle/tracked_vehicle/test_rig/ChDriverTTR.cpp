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

#include "chrono_vehicle/tracked_vehicle/test_rig/ChDriverTTR.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include "chrono/core/ChMathematics.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDriverTTR::ChDriverTTR() : m_throttle(0), m_delay(0), m_log_filename("") {}

void ChDriverTTR::Initialize(size_t num_posts) {
    m_displ.resize(num_posts);
}

// -----------------------------------------------------------------------------
// Initialize output file for recording deriver inputs.
// -----------------------------------------------------------------------------
bool ChDriverTTR::LogInit(const std::string& filename) {
    m_log_filename = filename;

    std::ofstream ofile(filename.c_str(), std::ios::out);
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
bool ChDriverTTR::Log(double time) {
    if (m_log_filename.empty())
        return false;

    std::ofstream ofile(m_log_filename.c_str(), std::ios::app);
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
void ChDriverTTR::SetDisplacement(int index, double val, double min_val, double max_val) {
    m_displ[index] = ChClamp(val, min_val, max_val);
}

void ChDriverTTR::SetThrottle(double val, double min_val, double max_val) {
    m_throttle = ChClamp(val, min_val, max_val);
}

}  // end namespace vehicle
}  // end namespace chrono
