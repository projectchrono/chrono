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

#include "chrono/functions/ChFunctionPoly23.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionPoly23)

ChFunctionPoly23::ChFunctionPoly23(const ChFunctionPoly23& other) {
    m_ampl = other.m_ampl;
    m_x_start = other.m_x_start;
    m_x_end = other.m_x_end;
}

double ChFunctionPoly23::GetVal(double x) const {
    double ret;
    double A = (m_x_end - m_x_start);
    if (x < m_x_start)
        return 0;
    if (x > m_x_end)
        return m_ampl;
    else {
        ret = m_ampl * ((3 * (std::pow(((x - m_x_start) / A), 2))) - 2 * (std::pow(((x - m_x_start) / A), 3)));
    }
    return ret;
}

double ChFunctionPoly23::GetDer(double x) const {
    double ret;
    double A = (m_x_end - m_x_start);
    if ((x < m_x_start) || (x > m_x_end))
        ret = 0;
    else {
        ret = m_ampl * (6 * ((x - m_x_start) / std::pow(A, 2)) - 6 * (std::pow((x - m_x_start), 2) / std::pow(A, 3)));
    }
    return ret;
}

double ChFunctionPoly23::GetDer2(double x) const {
    double ret;
    double A = (m_x_end - m_x_start);
    if ((x < m_x_start) || (x > m_x_end))
        ret = 0;
    else {
        ret = m_ampl * (6 * (1 / std::pow(A, 2)) - 12 * ((x - m_x_start) / std::pow(A, 3)));
    }
    return ret;
}

void ChFunctionPoly23::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionPoly23>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_ampl);
    archive_out << CHNVP(m_x_start);
    archive_out << CHNVP(m_x_end);
}

void ChFunctionPoly23::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionPoly23>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_ampl);
    archive_in >> CHNVP(m_x_start);
    archive_in >> CHNVP(m_x_end);
}

}  // namespace chrono
