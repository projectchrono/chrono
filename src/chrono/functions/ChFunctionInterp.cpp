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

#include "chrono/functions/ChFunctionInterp.h"

namespace chrono {

CH_FACTORY_REGISTER(ChFunctionInterp)

ChFunctionInterp::ChFunctionInterp(const ChFunctionInterp& other) {
    m_table = other.m_table;
    m_last_greater = m_table.end();
}

void ChFunctionInterp::AddPoint(double x, double y, bool overwrite_if_existing) {
    std::pair<std::map<double, double>::iterator, bool> ret = m_table.emplace(x, y);

    if (!ret.second) {
        // no insertion took place, so the point already exists
        if (overwrite_if_existing) {
            ret.first->second = y;
        } else {
            throw std::invalid_argument("Point already exists and overwrite flag was not set.");
        }
    }
}

double ChFunctionInterp::GetVal(double x) const {
    if (m_table.empty()) {
        return 0.0;
    }

    if (x <= m_table.begin()->first) {
        // if the extrapolation is not allowed, the derivative will be zero
        return m_table.begin()->second - GetDer(x) * (m_table.begin()->first - x);
    }

    if (x >= m_table.rbegin()->first) {
        // if the extrapolation is not allowed, the derivative will be zero
        return m_table.rbegin()->second + GetDer(x) * (x - m_table.rbegin()->first);
    }

    if (m_last_greater == m_table.end()) {
        m_last_greater = std::next(m_table.begin());
    }

    // there shouldn't be any case in which this happens, but if it does add a check if m_last_greater==m_table.begin
    // and then set it to ++m_table.begin()
    assert(m_last_greater != m_table.begin());

    //// Find the pair of points for which 'x' is in between
    // - the point is surely bigger than m_table.begin()->first and smaller than m_table.rbegin()->first

    // if m_last_greater is not valid anymore make a full search
    if (!(x > std::prev(m_last_greater)->first && x <= m_last_greater->first)) {
        m_last_greater = m_table.upper_bound(x);
    }

    double x_prev = std::prev(m_last_greater)->first;
    double y_prev = std::prev(m_last_greater)->second;
    double val = y_prev + (m_last_greater->second - y_prev) * (x - x_prev) / (m_last_greater->first - x_prev);

    return val;
}

double ChFunctionInterp::GetDer(double x) const {
    if (m_table.empty()) {
        return 0.0;
    }

    if (x <= m_table.begin()->first) {
        if (m_extrapolate && m_table.size() > 1) {
            double der_first = (std::next(m_table.begin())->second - m_table.begin()->second) /
                               (std::next(m_table.begin())->first - m_table.begin()->first);
            return der_first;
        } else {
            return 0.0;
        }
    }

    if (x >= m_table.rbegin()->first) {
        if (m_extrapolate && m_table.size() > 1) {
            double der_last = (m_table.rbegin()->second - std::next(m_table.rbegin())->second) /
                              (m_table.rbegin()->first - std::next(m_table.rbegin())->first);
            return der_last;
        } else {
            return 0.0;
        }
    }

    if (m_last_greater == m_table.end()) {
        m_last_greater = std::next(m_table.begin());
    }

    // there shouldn't be any case in which this happens, but if it does add a check if m_last_greater==m_table.begin
    // and then set it to ++m_table.begin()
    assert(m_last_greater != m_table.begin());

    //// Find the pair of points for which 'x' is in between
    // - the point is surely bigger than m_table.begin()->first and smaller than m_table.rbegin()->first

    // if m_last_greater is not valid anymore make a full search
    if (!(x >= std::prev(m_last_greater)->first && x < m_last_greater->first)) {
        m_last_greater = m_table.upper_bound(x);
    }

    double der = (m_last_greater->second - std::prev(m_last_greater)->second) /
                 (m_last_greater->first - std::prev(m_last_greater)->first);

    return der;
}

double ChFunctionInterp::GetDer2(double x) const {
    //// TODO: can we do better?
    return ChFunction::GetDer2(x);
}

double ChFunctionInterp::GetMax() const {
    using pType = std::pair<double, double>;
    auto ptr = std::max_element(m_table.begin(), m_table.end(),
                                [](const pType& p1, const pType& p2) { return p1.second < p2.second; });
    return ptr->second;
}

double ChFunctionInterp::GetMin() const {
    using pType = std::pair<double, double>;
    auto ptr = std::min_element(m_table.begin(), m_table.end(),
                                [](const pType& p1, const pType& p2) { return p1.second < p2.second; });
    return ptr->second;
}

void ChFunctionInterp::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionInterp>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data: copy to vector and store
    archive_out << CHNVP(m_table);
    archive_out << CHNVP(m_extrapolate);
}

void ChFunctionInterp::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionInterp>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data: load vector of points and copy to list
    archive_in >> CHNVP(m_table);
    archive_in >> CHNVP(m_extrapolate);

    m_last_greater = m_table.end();
}

}  // end namespace chrono
