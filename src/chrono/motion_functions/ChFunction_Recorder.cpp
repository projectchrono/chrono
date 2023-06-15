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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cmath>
#include <limits>

#include "chrono/motion_functions/ChFunction_Recorder.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_Recorder)

ChFunction_Recorder::ChFunction_Recorder(const ChFunction_Recorder& other) {
    m_points = other.m_points;
    m_last = m_points.end();
}

void ChFunction_Recorder::Estimate_x_range(double& xmin, double& xmax) const {
    if (m_points.empty()) {
        xmin = 0.0;
        xmax = 1.2;
        return;
    }

    xmin = m_points.front().x;
    xmax = m_points.back().x;
    if (xmin == xmax)
        xmax = xmin + 0.5;
}

void ChFunction_Recorder::AddPoint(double mx, double my, double mw) {
    for (auto iter = m_points.rbegin(); iter != m_points.rend(); ++iter) {
        double dist = mx - iter->x;
        if (std::abs(dist) < std::numeric_limits<double>::epsilon()) {
            // Overwrite current iterator
            iter->x = mx;
            iter->y = my;
            iter->w = mw;
            return;
        } else if (dist > 0) {
            // Insert before current iterator
            ChRecPoint rec(mx, my, mw);
            m_points.insert(iter.base(), rec);
            return;
        }
    }

    // Insert in front of list
    m_points.push_front(ChRecPoint(mx, my, mw));
}

double Interpolate_y(double x, const ChRecPoint& p1, const ChRecPoint& p2) {
    return ((x - p1.x) * p2.y + (p2.x - x) * p1.y) / (p2.x - p1.x);
}

double ChFunction_Recorder::Get_y(double x) const {
    if (m_points.empty()) {
        return 0;
    }

    if (x <= m_points.front().x) {
        return m_points.front().y;
    }

    if (x >= m_points.back().x) {
        return m_points.back().y;
    }

    // At this point we are guaranteed that there are at least two records.

    if (m_last == m_points.end()) {
        m_last = m_points.begin();
    }

    if (x > m_last->x) {
        // Search to the right
        for (auto iter = m_last; iter != m_points.end(); ++iter) {
            if (x <= iter->x) {
                return Interpolate_y(x, *m_last, *iter);
            }
            m_last = iter;
        }
    } else {
        // Search to the left
        for (auto iter = m_last; iter != m_points.begin();) {
            --iter;
            if (x >= iter->x) {
                return Interpolate_y(x, *iter, *m_last);
            }
            m_last = iter;
        }
    }

    return 0;
}

double ChFunction_Recorder::Get_y_dx(double x) const {
    //// TODO:  can we do better?
    return ChFunction::Get_y_dx(x);
}

double ChFunction_Recorder::Get_y_dxdx(double x) const {
    //// TODO: can we do better?
    return ChFunction::Get_y_dxdx(x);
}

void ChFunction_Recorder::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunction_Recorder>();
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data: copy to vector and store
    std::vector<ChRecPoint> tmpvect{std::begin(m_points), std::end(m_points)};
    marchive << CHNVP(tmpvect);
}

void ChFunction_Recorder::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunction_Recorder>();
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data: load vector of points and copy to list
    std::vector<ChRecPoint> tmpvect;
    marchive >> CHNVP(tmpvect);
    m_points.clear();
    std::copy(tmpvect.begin(), tmpvect.end(), std::back_inserter(m_points));
}

}  // end namespace chrono
