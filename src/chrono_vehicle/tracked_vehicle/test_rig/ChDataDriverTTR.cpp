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
// A driver model based on user inputs provided as time series. If provided as a
// text file, each line in the file must contain the following values:
//   time displ_post1 displ_post_2 ... throttle
// It is assumed that the time values are unique.
// If the time values are not sorted, this must be specified at construction.
// Inputs for post displacements and throttle are assumed to be in [-1,1].
// Driver inputs at intermediate times are obtained through linear interpolation.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/test_rig/ChDataDriverTTR.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace chrono {
namespace vehicle {

ChDataDriverTTR::ChDataDriverTTR(const std::string& filename, bool sorted) : m_filename(filename), m_sorted(sorted) {}

void ChDataDriverTTR::Initialize(size_t num_posts) {
    ChDriverTTR::Initialize(num_posts);

    std::ifstream ifile(m_filename.c_str());
    std::string line;

    double time;
    std::vector<double> displ(num_posts);
    double throttle;

    while (std::getline(ifile, line)) {
        std::istringstream iss(line);

        iss >> time;
        for (int i = 0; i < num_posts; i++)
            iss >> displ[i];
        iss >> throttle;

        if (iss.fail())
            break;

        m_data.push_back(Entry(time, displ, throttle));
    }

    ifile.close();

    if (!m_sorted)
        std::sort(m_data.begin(), m_data.end(), ChDataDriverTTR::compare);
}

static void Zero(std::vector<double> &vec) {
    for (auto& x : vec) {
        x = 0;
    }
}

void ChDataDriverTTR::Synchronize(double time) {
    if (time < m_delay) {
        Zero(m_displ);
        m_throttle = 0;
        return;
    }

    time -= m_delay;

    if (time <= m_data[0].m_time) {
        m_displ = m_data[0].m_displ;
        m_throttle = m_data[0].m_throttle;
        return;
    } else if (time >= m_data.back().m_time) {
        m_displ = m_data.back().m_displ;
        m_throttle = m_data.back().m_throttle;
        return;
    }

    std::vector<Entry>::iterator right =
        std::lower_bound(m_data.begin(), m_data.end(), Entry(time), ChDataDriverTTR::compare);

    std::vector<Entry>::iterator left = right - 1;

    double tbar = (time - left->m_time) / (right->m_time - left->m_time);

    for (size_t i = 0; i < m_displ.size(); i++) {
        m_displ[i] = left->m_displ[i] + tbar * (right->m_displ[i] - left->m_displ[i]);
    }        
    m_throttle = left->m_throttle + tbar * (right->m_throttle - left->m_throttle);
}

}  // end namespace vehicle
}  // end namespace chrono
