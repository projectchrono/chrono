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

#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigRoadDriver.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace chrono {
namespace vehicle {

ChTrackTestRigRoadDriver::ChTrackTestRigRoadDriver(const std::string& filename, double speed)
    : m_filename(filename), m_speed(speed), m_ended(false), m_curve_road(nullptr) {}

ChTrackTestRigRoadDriver::~ChTrackTestRigRoadDriver() {
    delete m_curve_road;
}

void ChTrackTestRigRoadDriver::Initialize(size_t num_posts, const std::vector<double>& locations) {
    ChTrackTestRigDriver::Initialize(num_posts, locations);

    std::vector<double> vx;  // road profile x
    std::vector<double> vz;  // road profile z

    std::ifstream ifile(m_filename.c_str());
    std::string line;
    while (std::getline(ifile, line)) {
        std::istringstream iss(line);
        double x, z;
        iss >> x;
        iss >> z;
        if (iss.fail())
            break;
        vx.push_back(x);
        vz.push_back(z);
    }
    ifile.close();

    // Create spline
    m_curve_road = new ChCubicSpline(vx, vz);

    // Cache min/max x values
    m_min = vx.front();
    m_max = vx.back();
}

static void Zero(std::vector<double>& vec) {
    for (auto& x : vec) {
        x = 0;
    }
}

void ChTrackTestRigRoadDriver::Synchronize(double time) {
    ChTrackTestRigDriver::Synchronize(time);

    Zero(m_displ);
    m_throttle = 0;

    if (time < m_delay) {
        return;
    }

    time -= m_delay;

    m_ended = true;
    for (size_t i = 0; i < m_displ.size(); i++) {
        double x = time * m_speed + m_locations[i];
        if (x < m_max) {
            // Not done as long as at least one post didn't reach the end of data
            m_ended = false;
        }
        if (x <= m_min || x >= m_max)
            continue;
        double dummy;
        m_curve_road->Evaluate(x, m_displ[i], m_displSpeed[i], dummy);
    }
}

bool ChTrackTestRigRoadDriver::Ended() const {
    return m_ended;
}

}  // end namespace vehicle
}  // end namespace chrono
