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

#include "chrono/functions/ChFunctionConstAcc.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionConstAcc)

ChFunctionConstAcc::ChFunctionConstAcc() : m_displacement(1), m_accel1_end(0.5), m_accel2_start(0.5), m_duration(1) {
    if (m_accel1_end > m_accel2_start)
        throw std::invalid_argument("First acceleration ramp must end before starting the second.");
}

ChFunctionConstAcc::ChFunctionConstAcc(double displacement,
                                       double acceleration1_end,
                                       double acceleration2_start,
                                       double duration)
    : m_displacement(displacement) {
    SetDuration(duration);
    SetAccelerationPoints(acceleration1_end, acceleration2_start);
}

ChFunctionConstAcc::ChFunctionConstAcc(const ChFunctionConstAcc& other) {
    m_displacement = other.m_displacement;
    m_accel1_end = other.m_accel1_end;
    m_accel2_start = other.m_accel2_start;
    m_duration = other.m_duration;
}

double ChFunctionConstAcc::GetVal(double x) const {
    double ret = 0;
    if (x <= 0)
        return 0;
    if (x >= m_duration)
        return m_displacement;
    double ev = m_accel1_end * m_duration;
    double ew = m_accel2_start * m_duration;
    double A = 2 * m_displacement / ((ev) * (m_duration - ev + ew));
    double B = 2 * m_displacement / ((m_duration - ew) * (m_duration - ev + ew));
    if ((x > 0) && (x < ev)) {
        ret = 0.5 * A * x * x;
    }
    if ((x >= ev) && (x <= ew)) {
        ret = A * ev * (x - ev * 0.5);
    }
    if ((x > ew) && (x < m_duration)) {
        ret = A * ev * (x - ev * 0.5) - B * 0.5 * std::pow((x - ew), 2);
    }
    return ret;
}

double ChFunctionConstAcc::GetDer(double x) const {
    double ret = 0;
    double ev = m_accel1_end * m_duration;
    double ew = m_accel2_start * m_duration;
    double A = 2 * m_displacement / ((ev) * (m_duration - ev + ew));
    double B = 2 * m_displacement / ((m_duration - ew) * (m_duration - ev + ew));
    if ((x > 0) && (x < ev)) {
        ret = A * x;
    }
    if ((x >= ev) && (x <= ew)) {
        ret = A * ev;
    }
    if ((x > ew) && (x < m_duration)) {
        ret = A * ev - B * (x - ew);
    }
    return ret;
}

double ChFunctionConstAcc::GetDer2(double x) const {
    double ret = 0;
    double ev = m_accel1_end * m_duration;
    double ew = m_accel2_start * m_duration;
    double A = 2 * m_displacement / ((ev) * (m_duration - ev + ew));
    double B = 2 * m_displacement / ((m_duration - ew) * (m_duration - ev + ew));
    if ((x > 0) && (x < ev)) {
        ret = A;
    }
    if ((x >= ev) && (x <= ew)) {
        ret = 0;
    }
    if ((x > ew) && (x < m_duration)) {
        ret = -B;
    }
    return ret;
}

void ChFunctionConstAcc::SetDuration(double duration) {
    if (duration < 0)
        throw std::invalid_argument("Duration should be greater than 0.");

    m_duration = duration;
}

void ChFunctionConstAcc::SetFirstAccelerationEnd(double acceleration1_end) {
    if (acceleration1_end > 1 || acceleration1_end < 0)
        throw std::invalid_argument("Acceleration starts and ends should be between 0 and 1.");

    m_accel1_end = acceleration1_end;
    if (m_accel1_end > m_accel2_start)
        m_accel1_end = m_accel2_start;
}

void ChFunctionConstAcc::SetSecondAccelerationStart(double acceleration2_start) {
    if (acceleration2_start > 1 || acceleration2_start < 0)
        throw std::invalid_argument("Acceleration starts and ends should be between 0 and 1.");

    m_accel2_start = acceleration2_start;
    if (m_accel2_start < m_accel1_end)
        m_accel2_start = m_accel1_end;
}

void ChFunctionConstAcc::SetAccelerationPoints(double acceleration1_end, double acceleration2_start) {
    if (acceleration1_end > acceleration2_start)
        throw std::invalid_argument("First acceleration ramp must end before starting the second.");

    if (acceleration1_end < 0 || acceleration2_start < 0 || acceleration1_end > 1 || acceleration2_start > 1)
        throw std::invalid_argument("Acceleration starts and ends should be between 0 and 1.");

    m_accel1_end = 0.0;
    m_accel2_start = 1.0;
    SetFirstAccelerationEnd(acceleration1_end);
    SetSecondAccelerationStart(acceleration2_start);
}

double ChFunctionConstAcc::GetPositiveAccelerationCoeff() const {
    return 2 * (m_duration * m_duration) /
           (m_accel1_end * m_duration * (m_duration - m_accel1_end * m_duration + m_accel2_start * m_duration));
}

double ChFunctionConstAcc::GetNegativeAccelerationCoeff() const {
    return 2 * (m_duration * m_duration) /
           ((m_duration - m_accel2_start * m_duration) *
            (m_duration - m_accel1_end * m_duration + m_accel2_start * m_duration));
}

double ChFunctionConstAcc::GetVelocityCoefficient() const {
    return 2 * (m_duration) / (m_duration - m_accel1_end * m_duration + m_accel2_start * m_duration);
}

void ChFunctionConstAcc::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChFunctionConstAcc>();
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_displacement);
    archive_out << CHNVP(m_duration);
    archive_out << CHNVP(m_accel2_start);
    archive_out << CHNVP(m_accel1_end);
}

void ChFunctionConstAcc::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChFunctionConstAcc>();
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_displacement);
    archive_in >> CHNVP(m_duration);
    archive_in >> CHNVP(m_accel2_start);
    archive_in >> CHNVP(m_accel1_end);
}

}  // end namespace chrono
