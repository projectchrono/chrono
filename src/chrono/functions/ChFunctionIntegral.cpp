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

#include "chrono/functions/ChFunctionIntegral.h"
#include "chrono/functions/ChFunctionConst.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionIntegral)

ChFunctionIntegral::ChFunctionIntegral()
    : m_integration_order(1), m_offset(0), m_x_start(0), m_x_end(1), m_num_samples(2000) {
    m_integrand_fun = chrono_types::make_shared<ChFunctionConst>();  // default
    m_cumintegral.resize(m_num_samples);
}

ChFunctionIntegral::ChFunctionIntegral(const ChFunctionIntegral& other) {
    m_integrand_fun = std::shared_ptr<ChFunction>(other.m_integrand_fun->Clone());
    m_integration_order = other.m_integration_order;
    m_offset = other.m_offset;
    m_x_start = other.m_x_start;
    m_x_end = other.m_x_end;
    m_num_samples = other.m_num_samples;
    m_cumintegral = other.m_cumintegral;
}

void ChFunctionIntegral::Setup() {
    double mstep = (m_x_end - m_x_start) / ((double)(m_num_samples - 1));
    double x_a, x_b, y_a, y_b, F_b;

    double F_sum = this->GetOffsetVal();

    m_cumintegral(0) = this->GetOffsetVal();

    for (unsigned int i = 1; i < this->m_num_samples; i++) {
        x_b = m_x_start + ((double)i) * (mstep);
        x_a = x_b - mstep;
        y_a = this->m_integrand_fun->GetVal(x_a);
        y_b = this->m_integrand_fun->GetVal(x_b);
        // trapezoidal rule..
        F_b = F_sum + mstep * (y_a + y_b) * 0.5;
        m_cumintegral(i) = F_b;
        F_sum = F_b;
    }
}

void ChFunctionIntegral::SetNumSamples(int m_samples) {
    m_num_samples = m_samples;
    m_cumintegral.setZero(m_num_samples, 1);
}

void ChFunctionIntegral::SetInterval(double xstart, double xend) {
    if (xend < xstart)
        throw std::invalid_argument("Invalid integration limits");

    m_x_start = xstart;
    m_x_end = xend;
}

double ChFunctionIntegral::GetVal(double x) const {
    if ((x < m_x_start) || (x > m_x_end))
        return 0.0;

    unsigned int i_a, i_b;
    double position = (double)(m_num_samples - 1) * ((x - m_x_start) / (m_x_end - m_x_start));
    i_a = (unsigned int)(floor(position));
    i_b = i_a + 1;

    if (i_b > m_num_samples - 1)
        return m_cumintegral(m_num_samples - 1);

    if (i_a < 0)
        return m_cumintegral(0);

    double weightB = position - (double)i_a;
    double weightA = 1 - weightB;

    return (weightA * (m_cumintegral(i_a)) + weightB * (m_cumintegral(i_b)));
}

void ChFunctionIntegral::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionIntegral>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_integrand_fun);
    archive_out << CHNVP(m_integration_order);
    archive_out << CHNVP(m_offset);
    archive_out << CHNVP(m_x_start);
    archive_out << CHNVP(m_x_end);
    archive_out << CHNVP(m_num_samples);
}

void ChFunctionIntegral::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionIntegral>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_integrand_fun);
    archive_in >> CHNVP(m_integration_order);
    archive_in >> CHNVP(m_offset);
    archive_in >> CHNVP(m_x_start);
    archive_in >> CHNVP(m_x_end);
    archive_in >> CHNVP(m_num_samples);
    m_cumintegral.setZero(m_num_samples);
    Setup();
}

}  // end namespace chrono
