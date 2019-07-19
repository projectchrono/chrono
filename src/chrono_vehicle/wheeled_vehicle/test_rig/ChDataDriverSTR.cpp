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
// text file, each line in the file must contain 4 values:
//   time left_post right_post steering
// It is assumed that the time values are unique.
// If the time values are not sorted, this must be specified at construction.
// Inputs for post_left, post_right, and steering are assumed to be in [-1,1].
// Driver inputs at intermediate times are obtained through linear interpolation.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChDataDriverSTR.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDataDriverSTR::ChDataDriverSTR(const std::string& filename, bool sorted) {
    std::ifstream ifile(filename.c_str());
    std::string line;

    while (std::getline(ifile, line)) {
        std::istringstream iss(line);

        double time, left, right, steering;

        iss >> time >> left >> right >> steering;

        if (iss.fail())
            break;

        m_data.push_back(Entry(time, left, right, steering));
    }

    ifile.close();

    if (!sorted)
        std::sort(m_data.begin(), m_data.end(), ChDataDriverSTR::compare);
}

ChDataDriverSTR::ChDataDriverSTR(const std::vector<Entry>& data, bool sorted) : m_data(data) {
    if (!sorted)
        std::sort(m_data.begin(), m_data.end(), ChDataDriverSTR::compare);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDataDriverSTR::Synchronize(double time) {
    if (time < m_delay) {
        m_displLeft = 0;
        m_displRight = 0;
        m_steering = 0;
        return;
    }

    time -= m_delay;

    if (time <= m_data[0].m_time) {
        m_displLeft = m_data[0].m_displLeft;
        m_displRight = m_data[0].m_displRight;
        m_steering = m_data[0].m_steering;
        return;
    } else if (time >= m_data.back().m_time) {
        m_displLeft = m_data.back().m_displLeft;
        m_displRight = m_data.back().m_displRight;
        m_steering = m_data.back().m_steering;
        return;
    }

    std::vector<Entry>::iterator right =
        std::lower_bound(m_data.begin(), m_data.end(), Entry(time, 0, 0, 0), ChDataDriverSTR::compare);

    std::vector<Entry>::iterator left = right - 1;

    double tbar = (time - left->m_time) / (right->m_time - left->m_time);

    m_displLeft = left->m_displLeft + tbar * (right->m_displLeft - left->m_displLeft);
    m_displRight = left->m_displRight + tbar * (right->m_displRight - left->m_displRight);
    m_steering = left->m_steering + tbar * (right->m_steering - left->m_steering);
}

}  // end namespace vehicle
}  // end namespace chrono
