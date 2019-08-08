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
// A driver model based on a user-specified road profile.  Post displacements
// are computed based on current road height at each post location and a given
// translation speed of the road profile. The road profile input data is assumed
// to contain (x,z) pairs, with x locations in increasing order.
//
// =============================================================================

#include "chrono_vehicle/tracked_vehicle/test_rig/ChRoadDriverTTR.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace chrono {
namespace vehicle {

ChRoadDriverTTR::ChRoadDriverTTR(const std::string& filename, double speed) : m_filename(filename), m_speed(speed) {}

void ChRoadDriverTTR::Initialize(size_t num_posts, const std::vector<double>& locations) {
    ChDriverTTR::Initialize(num_posts, locations);

    std::ifstream ifile(m_filename.c_str());

    std::string line;
    double x;
    double z;
    while (std::getline(ifile, line)) {
        std::istringstream iss(line);
        iss >> x;
        iss >> z;
        if (iss.fail())
            break;
        m_data.push_back(std::make_pair(x, z));
    }

    ifile.close();
}

static void Zero(std::vector<double>& vec) {
    for (auto& x : vec) {
        x = 0;
    }
}

void ChRoadDriverTTR::Synchronize(double time) {
    Zero(m_displ);
    m_throttle = 0;
    
    if (time < m_delay) {
        return;
    }

    time -= m_delay;

    for (size_t i = 0; i < m_displ.size(); i++) {
        double x = time * m_speed + m_locations[i];
        if (x <= m_data.front().first || x >= m_data.back().first)
            continue;
        auto right = std::lower_bound(m_data.begin(), m_data.end(), std::make_pair(x, -1.0));
        auto left = right - 1;
        double xbar = (x - left->first) / (right->first - left->first);
        m_displ[i] = left->second + xbar * (right->second - left->second);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
