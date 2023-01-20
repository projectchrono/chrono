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
// Driver inputs at intermediate times are obtained through cubic spline
// interpolation.
//
// =============================================================================

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstring>

#include "chrono/core/ChMathematics.h"
#include "chrono_vehicle/tracked_vehicle/test_rig/ChTrackTestRigDataDriver.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

// Definition of driver inputs at a given time.
struct DataEntryTTR {
    DataEntryTTR() : m_time(0), m_throttle(0) {}
    DataEntryTTR(double time, const std::vector<double>& displ, double throttle)
        : m_time(time), m_displ(displ), m_throttle(throttle) {}
    double m_time;
    double m_throttle;
    std::vector<double> m_displ;
};

static bool compare(const DataEntryTTR& a, const DataEntryTTR& b) {
    return a.m_time < b.m_time;
}

// -----------------------------------------------------------------------------

ChTrackTestRigDataDriver::ChTrackTestRigDataDriver(const std::string& filename)
    : m_filename(filename), m_ended(false), m_curve_throttle(nullptr) {}

ChTrackTestRigDataDriver::~ChTrackTestRigDataDriver() {
    delete m_curve_throttle;
}

void ChTrackTestRigDataDriver::Initialize(size_t num_posts, const std::vector<double>& locations) {
    ChTrackTestRigDriver::Initialize(num_posts, locations);
    m_curve_displ.resize(num_posts, nullptr);

    std::vector<DataEntryTTR> data;  // data table (for sorting)
    double time;
    std::vector<double> displ(num_posts);
    double throttle;

    // Read data from file
    std::ifstream ifile(m_filename.c_str());
    if (!ifile.is_open()) {
        std::cout << "failed to open " << m_filename << "    " << std::strerror(errno) << std::endl;
        return;
    }
    std::string line;
    while (std::getline(ifile, line)) {
        std::istringstream iss(line);
        iss >> time;
        for (int i = 0; i < num_posts; i++)
            iss >> displ[i];
        iss >> throttle;
        if (iss.fail())
            break;
        data.push_back(DataEntryTTR(time, displ, throttle));
    }
    ifile.close();

    // Ensure data is sorted
    std::sort(data.begin(), data.end(), compare);

    // Create cubic splines
    std::vector<double> t;                          // time points
    std::vector<std::vector<double>> d(num_posts);  // displacement input values
    std::vector<double> r;                          // throttle values

    t.push_back(data.begin()->m_time - 1);
    for (int i = 0; i < num_posts; i++)
        d[i].push_back(0);
    r.push_back(0);

    for (auto& entry : data) {
        t.push_back(entry.m_time);
        for (int i = 0; i < num_posts; i++)
            d[i].push_back(entry.m_displ[i]);
        r.push_back(entry.m_throttle);
    }

    for (int i = 0; i < num_posts; i++)
        m_curve_displ[i] = new ChCubicSpline(t, d[i]);
    m_curve_throttle = new ChCubicSpline(t, r);

    // Cache the last data entry
    m_last_time = data.back().m_time;
    m_last_displ = data.back().m_displ;
    m_last_throttle = data.back().m_throttle;
}

static void Zero(std::vector<double>& vec) {
    for (auto& x : vec) {
        x = 0;
    }
}

void ChTrackTestRigDataDriver::Synchronize(double time) {
    ChTrackTestRigDriver::Synchronize(time);

    if (time < m_delay) {
        Zero(m_displ);
        Zero(m_displSpeed);
        m_throttle = 0;
        return;
    }

    time -= m_delay;

    if (time > m_last_time) {
        m_ended = true;
        m_displ = m_last_displ;
        m_throttle = m_last_throttle;
        Zero(m_displSpeed);
        return;
    }

    double dummy;
    for (size_t i = 0; i < m_displ.size(); i++) {
        m_curve_displ[i]->Evaluate(time, m_displ[i], m_displSpeed[i], dummy);
    }
    m_curve_throttle->Evaluate(time, m_throttle, dummy, dummy);
    ChClampValue(m_throttle, 0.0, 1.0);
}

bool ChTrackTestRigDataDriver::Ended() const {
    return m_ended;
}

}  // end namespace vehicle
}  // end namespace chrono
