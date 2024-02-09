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

#include "chrono/motion_functions/ChFunctionSineStep.h"

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

void ChFunctionSineStep::SetP1(const ChVector2d& p1) {
    m_p1 = p1;
    assert(m_p1.x() != m_p2.x());
    m_dp = m_p2 - m_p1;
}

void ChFunctionSineStep::SetP2(const ChVector2d& p2) {
    m_p2 = p2;
    assert(m_p1.x() != m_p2.x());
    m_dp = m_p2 - m_p1;
}

double ChFunctionSineStep::Get_y(double x) const {
    if (x <= m_p1.x())
        return m_p1.y();

    if (x >= m_p2.x())
        return m_p2.y();

    double xx = (x - m_p1.x()) / m_dp.x();
    double y = m_p1.y() + m_dp.y() * (xx - std::sin(CH_C_2PI * xx) / CH_C_2PI);

    return y;
}

double ChFunctionSineStep::Get_y_dx(double x) const {
    if (x <= m_p1.x())
        return 0;

    if (x >= m_p2.x())
        return 0;

    double xx = (x - m_p1.x()) / m_dp.x();
    double yd = (m_dp.y() / m_dp.x()) * (1 - std::cos(CH_C_2PI * xx));

    return yd;
}

double ChFunctionSineStep::Get_y_dxdx(double x) const {
    if (x <= m_p1.x())
        return 0;

    if (x >= m_p2.x())
        return 0;

    double xx = (x - m_p1.x()) / m_dp.x();
    double ydd = CH_C_2PI * m_dp.y() / (m_dp.x() * m_dp.x()) * std::sin(CH_C_2PI * xx);

    return ydd;
}

void ChFunctionSineStep::ArchiveOut(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChFunctionSineStep>();
    ChFunction::ArchiveOut(marchive);
    marchive << CHNVP(m_p1);
    marchive << CHNVP(m_p2);
    marchive << CHNVP(m_dp);
}

void ChFunctionSineStep::ArchiveIn(ChArchiveIn& marchive) {
    /*int version =*/marchive.VersionRead<ChFunctionSineStep>();
    ChFunction::ArchiveIn(marchive);
    marchive >> CHNVP(m_p1);
    marchive >> CHNVP(m_p2);
    marchive >> CHNVP(m_dp);
}

double ChFunctionSineStep::Eval(double x, double x1, double y1, double x2, double y2) {
    if (x <= x1)
        return y1;

    if (x >= x2)
        return y2;

    double xx = (x - x1) / (x2 - x1);
    double y = y1 + (y2 - y1) * (xx - std::sin(CH_C_2PI * xx) / CH_C_2PI);

    return y;
}

}  // end namespace chrono
