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

#include "chrono/functions/ChFunctionFillet3.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionFillet3)

ChFunctionFillet3::ChFunctionFillet3(const ChFunctionFillet3& other) {
    m_width = other.m_width;
    m_val_start = other.m_val_start;
    m_val_end = other.m_val_end;
    m_der_start = other.m_der_start;
    m_der_end = other.m_der_end;
    c1 = other.c1;
    c2 = other.c2;
    c3 = other.c3;
    c4 = other.c4;
}

double ChFunctionFillet3::GetVal(double x) const {
    double ret = 0;
    if (x <= 0)
        return m_val_start;
    if (x >= m_width)
        return m_val_end;
    ret = c1 * std::pow(x, 3) + c2 * std::pow(x, 2) + c3 * x + c4;
    return ret;
}

double ChFunctionFillet3::GetDer(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= m_width)
        return 0;
    ret = 3 * c1 * std::pow(x, 2) + 2 * c2 * x + c3;
    return ret;
}

double ChFunctionFillet3::GetDer2(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= m_width)
        return 0;
    ret = 6 * c1 * x + 2 * c2;
    return ret;
}

void ChFunctionFillet3::SetWidth(double width) {
    if (width < 0)
        throw std::invalid_argument("Width cannot be negative");

    m_width = width;
}

void ChFunctionFillet3::Setup() {
    ChMatrixDynamic<> ma(4, 4);
    ChMatrixDynamic<> mb(4, 1);
    ChMatrixDynamic<> mx(4, 1);

    mb(0, 0) = m_val_start;
    mb(1, 0) = m_val_end;
    mb(2, 0) = m_der_start;
    mb(3, 0) = m_der_end;

    ma(0, 3) = 1.0;

    ma(1, 0) = std::pow(m_width, 3);
    ma(1, 1) = std::pow(m_width, 2);
    ma(1, 2) = m_width;
    ma(1, 3) = 1.0;

    ma(2, 2) = 1.0;

    ma(3, 0) = 3 * std::pow(m_width, 2);
    ma(3, 1) = 2 * m_width;
    ma(3, 2) = 1.0;

    mx = ma.colPivHouseholderQr().solve(mb);

    c1 = mx(0, 0);
    c2 = mx(1, 0);
    c3 = mx(2, 0);
    c4 = mx(3, 0);
}

void ChFunctionFillet3::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionFillet3>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_width);
    archive_out << CHNVP(m_val_start);
    archive_out << CHNVP(m_val_end);
    archive_out << CHNVP(m_der_start);
    archive_out << CHNVP(m_der_end);
}

void ChFunctionFillet3::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionFillet3>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_width);
    archive_in >> CHNVP(m_val_start);
    archive_in >> CHNVP(m_val_end);
    archive_in >> CHNVP(m_der_start);
    archive_in >> CHNVP(m_der_end);
    Setup();
}

}  // namespace chrono
