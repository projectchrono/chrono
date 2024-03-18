// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/functions/ChFunctionSineStep.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionSineStep)

ChFunctionSineStep::ChFunctionSineStep(const ChVector2d& p1, const ChVector2d& p2) : m_p1(p1), m_p2(p2) {
    assert(m_p1.x() != m_p2.x());
    m_dp = p2 - p1;
}

ChFunctionSineStep::ChFunctionSineStep(const ChFunctionSineStep& other) {
    m_p1 = other.m_p1;
    m_p2 = other.m_p2;
    m_dp = other.m_dp;
}

void ChFunctionSineStep::SetFirstPoint(const ChVector2d& p1) {
    m_p1 = p1;
    assert(m_p1.x() != m_p2.x());
    m_dp = m_p2 - m_p1;
}

void ChFunctionSineStep::SetSecondPoint(const ChVector2d& p2) {
    m_p2 = p2;
    assert(m_p1.x() != m_p2.x());
    m_dp = m_p2 - m_p1;
}

double ChFunctionSineStep::GetVal(double x) const {
    if (x <= m_p1.x())
        return m_p1.y();

    if (x >= m_p2.x())
        return m_p2.y();

    double xx = (x - m_p1.x()) / m_dp.x();
    double y = m_p1.y() + m_dp.y() * (xx - std::sin(CH_2PI * xx) / CH_2PI);

    return y;
}

double ChFunctionSineStep::GetDer(double x) const {
    if (x <= m_p1.x())
        return 0;

    if (x >= m_p2.x())
        return 0;

    double xx = (x - m_p1.x()) / m_dp.x();
    double yd = (m_dp.y() / m_dp.x()) * (1 - std::cos(CH_2PI * xx));

    return yd;
}

double ChFunctionSineStep::GetDer2(double x) const {
    if (x <= m_p1.x())
        return 0;

    if (x >= m_p2.x())
        return 0;

    double xx = (x - m_p1.x()) / m_dp.x();
    double ydd = CH_2PI * m_dp.y() / (m_dp.x() * m_dp.x()) * std::sin(CH_2PI * xx);

    return ydd;
}

void ChFunctionSineStep::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChFunctionSineStep>();
    ChFunction::ArchiveOut(archive_out);
    archive_out << CHNVP(m_p1);
    archive_out << CHNVP(m_p2);
    archive_out << CHNVP(m_dp);
}

void ChFunctionSineStep::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChFunctionSineStep>();
    ChFunction::ArchiveIn(archive_in);
    archive_in >> CHNVP(m_p1);
    archive_in >> CHNVP(m_p2);
    archive_in >> CHNVP(m_dp);
}

double ChFunctionSineStep::Eval(double x, double x1, double y1, double x2, double y2) {
    if (x <= x1)
        return y1;

    if (x >= x2)
        return y2;

    double xx = (x - x1) / (x2 - x1);
    double y = y1 + (y2 - y1) * (xx - std::sin(CH_2PI * xx) / CH_2PI);

    return y;
}

}  // end namespace chrono
