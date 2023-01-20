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
// Driver inputs at intermediate times are obtained through cubic spline
// interpolation.
//
// =============================================================================

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "chrono_vehicle/wheeled_vehicle/test_rig/ChSuspensionTestRigDataDriver.h"
#include "chrono/core/ChTypes.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
/// Definition of driver inputs at a given time.
struct DataEntrySTR {
    DataEntrySTR() : m_time(0), m_steering(0) {}
    DataEntrySTR(double time, double steering, std::vector<double> displLeft, std::vector<double> displRight)
        : m_time(time), m_displLeft(displLeft), m_displRight(displRight), m_steering(steering) {}
    DataEntrySTR& operator=(const DataEntrySTR& other) {
        if (this != &other) {
            m_time = other.m_time;
            m_displLeft = other.m_displLeft;
            m_displRight = other.m_displRight;
            m_steering = other.m_steering;
        }
        return *this;
    }
    double m_time;
    double m_steering;
    std::vector<double> m_displLeft;
    std::vector<double> m_displRight;
};

static bool compare(const DataEntrySTR& a, const DataEntrySTR& b) {
    return a.m_time < b.m_time;
}

// -----------------------------------------------------------------------------
ChSuspensionTestRigDataDriver::ChSuspensionTestRigDataDriver(const std::string& filename) : m_filename(filename), m_ended(false) {}

void ChSuspensionTestRigDataDriver::Initialize(int naxles) {
    // Invoke base method
    ChSuspensionTestRigDriver::Initialize(naxles);

    // Read data from file
    std::vector<DataEntrySTR> data;  // data table (for sorting)

    std::ifstream ifile(m_filename.c_str());
    std::string line;
    while (std::getline(ifile, line)) {
        std::istringstream iss(line);
        double time, steering;
        std::vector<double> left(naxles);
        std::vector<double> right(naxles);
        iss >> time >> steering;
        for (int ia = 0; ia < naxles; ia++)
            iss >> left[ia] >> right[ia];
        if (iss.fail())
            break;
        data.push_back(DataEntrySTR(time, steering, left, right));
    }
    ifile.close();

    // Ensure data is sorted
    std::sort(data.begin(), data.end(), compare);

    // Create cubic splines
    std::vector<double> t;                       // time points
    std::vector<double> s;                       // steering values
    std::vector<std::vector<double>> l(naxles);  // left input values
    std::vector<std::vector<double>> r(naxles);  // right input values

    t.push_back(data.begin()->m_time - 1);
    s.push_back(0);
    for (int ia = 0; ia < naxles; ia++) {
        l[ia].push_back(0);
        r[ia].push_back(0);
    }

    for (auto& entry : data) {
        t.push_back(entry.m_time);
        s.push_back(entry.m_steering);
        for (int ia = 0; ia < naxles; ia++) {
            l[ia].push_back(entry.m_displLeft[ia]);
            r[ia].push_back(entry.m_displRight[ia]);
        }
    }

    m_curve_steering = chrono_types::make_unique<ChCubicSpline>(t, s);
    for (int ia = 0; ia < m_naxles; ia++) {
        m_curve_left.push_back(chrono_types::make_unique<ChCubicSpline>(t, l[ia]));
        m_curve_right.push_back(chrono_types::make_unique<ChCubicSpline>(t, r[ia]));
    }

    // Cache the last data entry
    m_last_time = data.back().m_time;
    m_last_steering = data.back().m_steering;
    m_last_displLeft.resize(naxles);
    m_last_displRight.resize(naxles);
    for (int ia = 0; ia < m_naxles; ia++) {
        m_last_displLeft[ia] = data.back().m_displLeft[ia];
        m_last_displRight[ia] = data.back().m_displRight[ia];
    }
}

// -----------------------------------------------------------------------------

void ChSuspensionTestRigDataDriver::Synchronize(double time) {
    ChSuspensionTestRigDriver::Synchronize(time);

    if (time < m_delay) {
        m_steering = 0;
        std::fill(m_displLeft.begin(), m_displLeft.end(), 0.0);
        std::fill(m_displRight.begin(), m_displRight.end(), 0.0);
        std::fill(m_displSpeedLeft.begin(), m_displSpeedLeft.end(), 0.0);
        std::fill(m_displSpeedRight.begin(), m_displSpeedRight.end(), 0.0);
        return;
    }

    time -= m_delay;

    if (time > m_last_time) {
        m_ended = true;
        m_displLeft = m_last_displLeft;
        m_displRight = m_last_displRight;
        m_steering = m_last_steering;
        std::fill(m_displSpeedLeft.begin(), m_displSpeedLeft.end(), 0.0);
        std::fill(m_displSpeedRight.begin(), m_displSpeedRight.end(), 0.0);
        return;
    }

    double dummy;
    m_curve_steering->Evaluate(time, m_steering, dummy, dummy);
    for (int ia = 0; ia < m_naxles; ia++) {
        m_curve_left[ia]->Evaluate(time, m_displLeft[ia], m_displSpeedLeft[ia], dummy);
        m_curve_right[ia]->Evaluate(time, m_displRight[ia], m_displSpeedRight[ia], dummy);
    }
}

// -----------------------------------------------------------------------------
bool ChSuspensionTestRigDataDriver::Ended() const {
    return m_ended;
}

}  // end namespace vehicle
}  // end namespace chrono
