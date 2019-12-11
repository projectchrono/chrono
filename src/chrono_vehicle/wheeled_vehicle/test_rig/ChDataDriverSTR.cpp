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
static bool compare(const ChDataDriverSTR::Entry& a, const ChDataDriverSTR::Entry& b) {
    return a.m_time < b.m_time;
}

ChDataDriverSTR::Entry& ChDataDriverSTR::Entry::operator=(const ChDataDriverSTR::Entry& other) {
    if (this != &other) {
        m_time = other.m_time;
        m_displLeft = other.m_displLeft;
        m_displRight = other.m_displRight;
        m_steering = other.m_steering;
    }
    return *this;
}

// -----------------------------------------------------------------------------
ChDataDriverSTR::ChDataDriverSTR(const std::string& filename, bool sorted)
    : m_ended(false), m_curve_left(nullptr), m_curve_right(nullptr), m_curve_steering(nullptr) {
    std::vector<Entry> mdata;
    std::ifstream ifile(filename.c_str());
    std::string line;
    while (std::getline(ifile, line)) {
        std::istringstream iss(line);
        double time, left, right, steering;
        iss >> time >> left >> right >> steering;
        if (iss.fail())
            break;
        mdata.push_back(Entry(time, left, right, steering));
    }
    ifile.close();

    Process(mdata, sorted);
}

ChDataDriverSTR::ChDataDriverSTR(const std::vector<Entry>& data, bool sorted)
    : m_ended(false), m_curve_left(nullptr), m_curve_right(nullptr), m_curve_steering(nullptr) {
    // Make a copy of data (may need to be sorted)
    std::vector<Entry> mdata = data;
    Process(mdata, sorted);
}

ChDataDriverSTR::~ChDataDriverSTR() {
    delete m_curve_left;
    delete m_curve_right;
    delete m_curve_steering;
}

void ChDataDriverSTR::Process(std::vector<Entry>& data, bool sorted) {
    if (!sorted)
        std::sort(data.begin(), data.end(), compare);

    std::vector<double> t;  // time points
    std::vector<double> l;  // left input values
    std::vector<double> r;  // right input values
    std::vector<double> s;  // steering values

    t.push_back(data.begin()->m_time - 1);
    l.push_back(0);
    r.push_back(0);
    s.push_back(0);

    for (auto& entry : data) {
        t.push_back(entry.m_time);
        l.push_back(entry.m_displLeft);
        r.push_back(entry.m_displRight);
        s.push_back(entry.m_steering);
    }

    m_curve_left = new ChCubicSpline(t, l);
    m_curve_right = new ChCubicSpline(t, r);
    m_curve_steering = new ChCubicSpline(t, s);

    m_last = data.back();  // cache the last data entry
}

// -----------------------------------------------------------------------------

void ChDataDriverSTR::Synchronize(double time) {
    ChDriverSTR::Synchronize(time);

    if (time < m_delay) {
        m_displLeft = 0;
        m_displRight = 0;
        m_steering = 0;
        m_displSpeedLeft = 0;
        m_displSpeedRight = 0;
        return;
    }

    time -= m_delay;

    if (time > m_last.m_time) {
        m_ended = true;
        m_displLeft = m_last.m_displLeft;
        m_displRight = m_last.m_displRight;
        m_steering = m_last.m_steering;
        m_displSpeedLeft = 0;
        m_displSpeedRight = 0;
        return;
    }

    double dummy;
    m_curve_left->Evaluate(time, m_displLeft, m_displSpeedLeft, dummy);
    m_curve_right->Evaluate(time, m_displRight, m_displSpeedRight, dummy);
    m_curve_steering->Evaluate(time, m_steering, dummy, dummy);
}

// -----------------------------------------------------------------------------
bool ChDataDriverSTR::Ended() const {
    return m_ended;
}

}  // end namespace vehicle
}  // end namespace chrono
