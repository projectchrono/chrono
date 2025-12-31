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

#include "chrono/functions/ChFunctionPoly345.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPoly345)

ChFunctionPoly345::ChFunctionPoly345(double height, double width) : m_height(height) {
    SetWidth(width);
}

ChFunctionPoly345::ChFunctionPoly345(const ChFunctionPoly345& other) {
    m_height = other.m_height;
    m_width = other.m_width;
}

double ChFunctionPoly345::GetVal(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= m_width)
        return m_height;
    double a = x / m_width;
    ret = m_height * (10 * std::pow(a, 3) - 15 * std::pow(a, 4) + 6 * std::pow(a, 5));
    return ret;
}

double ChFunctionPoly345::GetDer(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= m_width)
        return 0;
    double a = x / m_width;
    ret = m_height * (1 / m_width) * (30 * std::pow(a, 2) - 60 * std::pow(a, 3) + 30 * std::pow(a, 4));
    return ret;
}

double ChFunctionPoly345::GetDer2(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= m_width)
        return 0;
    double a = x / m_width;
    ret = m_height * (1 / (m_width * m_width)) * (60 * a - 180 * std::pow(a, 2) + 120 * std::pow(a, 3));
    return ret;
}

double ChFunctionPoly345::GetDer3(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= m_width)
        return 0;
    double a = x / m_width;
    ret = m_height / std::pow(m_width, 3) * (60 - 360 * a + 360 * std::pow(a, 2));
    return ret;
}

void ChFunctionPoly345::SetWidth(double width) {
    if (width <= 0)
        throw std::invalid_argument("Invalid width. Must be positive.");
    m_width = width;
}

void ChFunctionPoly345::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionPoly345>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_height);
    archive_out << CHNVP(m_width);
}

void ChFunctionPoly345::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionPoly345>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_height);
    archive_in >> CHNVP(m_width);
}

}  // end namespace chrono
