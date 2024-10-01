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
// Authors: Dario Fusai
// =============================================================================

#include <cmath>

#include "chrono/functions/ChFunctionCycloidal.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionCycloidal)

ChFunctionCycloidal::ChFunctionCycloidal(double height, double width) : m_height(height) {
    SetWidth(width);
}

ChFunctionCycloidal::ChFunctionCycloidal(const ChFunctionCycloidal& other) {
    m_height = other.m_height;
    m_width = other.m_width;
}

double ChFunctionCycloidal::GetVal(double x) const {
    if (x <= 0)
        return 0;
    else if (x >= m_width)
        return m_height;
    double ret = m_height * (x / m_width - std::sin(CH_2PI * x / m_width) / CH_2PI);
    return ret;
}

double ChFunctionCycloidal::GetDer(double x) const {
    if (x <= 0)
        return 0;
    else if (x >= m_width)
        return 0;
    double ret = m_height / m_width * (1 - std::cos(CH_2PI * x / m_width));
    return ret;
}

double ChFunctionCycloidal::GetDer2(double x) const {
    if (x <= 0)
        return 0;
    else if (x >= m_width)
        return 0;
    double ret = CH_2PI * m_height / m_width / m_width * std::sin(CH_2PI * x / m_width);
    return ret;
}

double ChFunctionCycloidal::GetDer3(double x) const {
    if (x <= 0)
        return 0;
    else if (x >= m_width)
        return 0;
    double ret = std::pow(CH_2PI, 2) * m_height / std::pow(m_width, 3) * std::cos(CH_2PI * x / m_width);
    return ret;
}

void ChFunctionCycloidal::SetWidth(double width) {
    if (width <= 0)
        throw std::invalid_argument("Width must be positive.");

    m_width = width;
}

void ChFunctionCycloidal::ArchiveOut(ChArchiveOut& archive_out) {
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_height);
    archive_out << CHNVP(m_width);
}

void ChFunctionCycloidal::ArchiveIn(ChArchiveIn& archive_in) {
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_height);
    archive_in >> CHNVP(m_width);
}

}  // namespace chrono
