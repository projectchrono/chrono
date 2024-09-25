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

#include "chrono/functions/ChFunctionConstJerk.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionConstJerk)

ChFunctionConstJerk::ChFunctionConstJerk(double q0, double q1, double v0, double v1, double T, double Ta, double Tj) {
    Setup(q0, q1, v0, v1, T, Ta, Tj);
}

ChFunctionConstJerk::ChFunctionConstJerk(double q0, double q1, double vmax, double amax, double jmax) {
    Setup(q0, q1, vmax, amax, jmax);
}

ChFunctionConstJerk::ChFunctionConstJerk(bool& feasible,
                                         double q0,
                                         double q1,
                                         double v0,
                                         double v1,
                                         double vmax,
                                         double amax,
                                         double jmax) {
    if (v0 == 0 && v1 == 0) {  // fallback to simplified case
        feasible = true;
        Setup(q0, q1, vmax, amax, jmax);
    } else
        Setup(feasible, q0, q1, v0, v1, vmax, amax, jmax);  // general case
}

ChFunctionConstJerk::ChFunctionConstJerk(const ChFunctionConstJerk& other) {
    m_q0 = other.m_q0;
    m_q1 = other.m_q1;
    m_v0 = other.m_v0;
    m_v1 = other.m_v1;
    m_vmax_lim = other.m_vmax_lim;
    m_amax_lim = other.m_amax_lim;
    m_jmax_lim = other.m_jmax_lim;
    m_Ta = other.m_Ta;
    m_Tv = other.m_Tv;
    m_Td = other.m_Td;
    m_Tj1 = other.m_Tj1;
    m_Tj2 = other.m_Tj2;
    m_T = other.m_T;
    m_amax_reached = other.m_amax_reached;
    m_amin_reached = other.m_amin_reached;
    m_vmax_reached = other.m_vmax_reached;
}

void ChFunctionConstJerk::Setup(double q0, double q1, double v0, double v1, double T, double Ta, double Tj) {
    // Adjust motion law sign
    m_sign = (q1 > q0) ? 1 : -1;

    m_q0 = m_sign * q0;
    m_q1 = m_sign * q1;
    m_v0 = m_sign * v0;
    m_v1 = m_sign * v1;
    m_T = T;
    m_Ta = Ta;
    m_Td = Ta;
    m_Tj1 = Tj;
    m_Tj2 = Tj;

    double h = m_q1 - m_q0;
    m_Tv = m_T - m_Ta - m_Td;
    m_vmax_lim = h / (m_T - m_Ta);
    m_amax_lim = h / (m_T - m_Ta) / (m_Ta - m_Tj1);
    m_jmax_lim = h / (m_T - m_Ta) / (m_Ta - m_Tj1) / m_Tj1;
    m_vmax_reached = m_vmax_lim;
    m_amax_reached = m_amax_lim;
    m_amin_reached = -m_amax_lim;
}

void ChFunctionConstJerk::Setup(double q0, double q1, double vmax, double amax, double jmax) {
    // Adjust motion law sign
    m_sign = (q1 > q0) ? 1 : -1;

    m_q0 = m_sign * q0;
    m_q1 = m_sign * q1;
    m_v0 = 0;
    m_v1 = 0;
    m_vmax_lim = vmax;
    m_amax_lim = amax;
    m_jmax_lim = jmax;

    // Initial useful data
    double h = m_q1 - m_q0;                  // total displacement
    double amax2 = m_amax_lim * m_amax_lim;  // amax squared
    double Tj = 0;

    if (m_vmax_lim * m_jmax_lim >= amax2) {
        Tj = m_amax_lim / m_jmax_lim;
        m_Tj1 = Tj;
        m_Tj2 = Tj;
        m_Ta = Tj + m_vmax_lim / m_amax_lim;
    } else {
        Tj = std::sqrt(m_vmax_lim / m_jmax_lim);
        m_Tj1 = Tj;
        m_Tj2 = Tj;
        m_Ta = 2 * Tj;
    }

    m_Tv = h / m_vmax_lim - m_Ta;

    // No constant speed tract
    if (m_Tv <= 0) {
        m_Tv = 0;  // adjust

        if (h >= 2 * std::pow(m_amax_lim, 3) / std::pow(m_jmax_lim, 2)) {
            Tj = m_amax_lim / m_jmax_lim;
            m_Tj1 = Tj;
            m_Tj2 = Tj;
            m_Ta = 0.5 * Tj + std::sqrt(std::pow(0.5 * Tj, 2) + h / m_amax_lim);
        } else {
            Tj = std::pow(h / (2 * m_jmax_lim), 1. / 3.);
            m_Tj1 = Tj;
            m_Tj2 = Tj;
            m_Ta = 2 * Tj;
        }
    }

    // Total actuation time
    m_Td = m_Ta;
    m_T = m_Ta + m_Tv + m_Td;

    // Finally, compute max acceleration and velocity effectively reached
    double alim = m_jmax_lim * Tj;
    m_amax_reached = alim;
    m_amin_reached = -m_amax_reached;
    m_vmax_reached = (m_Ta - Tj) * alim;
}

void ChFunctionConstJerk::Setup(bool& feasible,
                                double q0,
                                double q1,
                                double v0,
                                double v1,
                                double vmax,
                                double amax,
                                double jmax) {
    // Adjust motion law sign
    m_sign = (q1 > q0) ? 1 : -1;

    m_q0 = m_sign * q0;
    m_q1 = m_sign * q1;
    m_v0 = m_sign * v0;
    m_v1 = m_sign * v1;
    m_vmax_lim = vmax;
    m_amax_lim = amax;
    m_jmax_lim = jmax;

    // Initial useful data
    double h = m_q1 - m_q0;                  // total displacement
    double amax2 = m_amax_lim * m_amax_lim;  // amax squared

    // Check motion law feasability --------------------
    double Tj_star1 = std::sqrt(std::abs(m_v1 - m_v0) / m_jmax_lim);
    double Tj_star2 = m_amax_lim / m_jmax_lim;
    double Tj_star = std::min(Tj_star1, Tj_star2);

    feasible = false;  // assume not feasible for now
    if (Tj_star < Tj_star2) {
        double tmp = Tj_star * (m_v0 + m_v1);
        if (h > tmp)
            feasible = true;
    } else {  // Tj_star == Tj_star1
        double tmp = 0.5 * (m_v0 + m_v1) * (Tj_star + std::abs(m_v1 - m_v0) / m_amax_lim);
        if (h > tmp)
            feasible = true;
    }

    // If not feasible, return now.
    if (feasible == false) {
        m_Tj1 = 0;
        m_Tj2 = 0;
        m_Ta = 0;
        m_Tv = 0;
        m_Td = 0;
        m_T = 0;
        m_amax_reached = 0;
        m_amin_reached = 0;
        m_vmax_lim = 0;
        return;
    }

    // Case 1: vlim = vmax -----------------------------
    // For now, assume vmax and amax are reached

    // acceleration time intervals
    if ((m_vmax_lim - m_v0) * m_jmax_lim < amax2) {  // amax is not reached
        m_Tj1 = std::sqrt((m_vmax_lim - m_v0) / m_jmax_lim);
        m_Ta = 2 * m_Tj1;
    } else {  // amax is reached
        m_Tj1 = amax / jmax;
        m_Ta = m_Tj1 + (m_vmax_lim - m_v0) / m_amax_lim;
    }

    // deceleration time intervals
    if ((m_vmax_lim - m_v1) * m_jmax_lim < amax2) {  // amin is not reached
        m_Tj2 = std::sqrt((m_vmax_lim - m_v1) / m_jmax_lim);
        m_Td = 2 * m_Tj2;
    } else {  // amin is reached
        m_Tj2 = m_amax_lim / m_jmax_lim;
        m_Td = m_Tj2 + (m_vmax_lim - m_v1) / m_amax_lim;
    }

    m_Tv = h / m_vmax_lim - 0.5 * m_Ta * (1 + m_v0 / m_vmax_lim) -
           0.5 * m_Td * (1 + m_v1 / m_vmax_lim);  // constant velocity time

    // Case 2: vlim < vmax -----------------------------
    // If Tv > 0, vmax is effectively reached; otherwise, recompute times

    if (m_Tv <= 0) {                          // no constant speed tract
        m_Tv = 0;                             // adjust
        double Tj = m_amax_lim / m_jmax_lim;  // Tj1 = Tj2 == Tj
        m_Tj1 = Tj;
        m_Tj2 = Tj;

        double delta = std::pow(m_amax_lim, 4) / std::pow(m_jmax_lim, 2) + 2 * (std::pow(m_v0, 2) + std::pow(m_v1, 2)) +
                       m_amax_lim * (4 * h - 2 * m_amax_lim / m_jmax_lim * (m_v0 + m_v1));
        m_Ta = (amax2 / m_jmax_lim - 2 * m_v0 + std::sqrt(delta)) / (2 * m_amax_lim);
        m_Td = (amax2 / m_jmax_lim - 2 * m_v1 + std::sqrt(delta)) / (2 * m_amax_lim);
    }

    if (m_Ta < 0 || m_Td < 0) {
        if (m_Ta < 0) {
            m_Ta = 0;  // adjust
            m_Td = 2 * h / (m_v1 + m_v0);
            m_Tj2 = (m_jmax_lim * h -
                     std::sqrt(m_jmax_lim * (m_jmax_lim * std::pow(h, 2) + std::pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)))) /
                    (m_jmax_lim * (m_v1 + m_v0));
        } else if (m_Td < 0) {
            m_Td = 0;  // adjust
            m_Ta = 2 * h / (v1 + v0);
            m_Tj1 = (m_jmax_lim * h -
                     std::sqrt(m_jmax_lim * (m_jmax_lim * std::pow(h, 2) - std::pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)))) /
                    (m_jmax_lim * (m_v1 + m_v0));
        }
    } else {
        if (m_Ta < 2 * m_Tj1 || m_Td < 2 * m_Tj1) {  // = Tj
            // TODO: iterative refining (?)
        }
    }

    // Total actuation time
    m_T = m_Ta + m_Tv + m_Td;

    // Finally, compute max acceleration and velocity effectively reached
    m_amax_reached = m_jmax_lim * m_Tj1;
    m_amin_reached = -m_jmax_lim * m_Tj2;
    m_vmax_reached = m_v0 + (m_Ta - m_Tj1) * m_amax_reached;  // = m_v1 - (m_Td - m_Tj2) * m_amin_reached
}

double ChFunctionConstJerk::GetVal(double x) const {
    double y = 0;

    // Acceleration phase
    if (x >= 0 && x < m_Tj1)
        y = m_q0 + m_v0 * x + m_jmax_lim * std::pow(x, 3) / 6.;
    else if (x >= m_Tj1 && x < m_Ta - m_Tj1)
        y = m_q0 + m_v0 * x + m_amax_reached / 6. * (3 * std::pow(x, 2) - 3 * m_Tj1 * x + std::pow(m_Tj1, 2));
    else if (x >= m_Ta - m_Tj1 && x < m_Ta)
        y = m_q0 + (m_vmax_reached + m_v0) * m_Ta / 2. - m_vmax_reached * (m_Ta - x) +
            m_jmax_lim * std::pow((m_Ta - x), 3) / 6.;  // +jmax = -jmin
    // Constant velocity phase
    else if (x >= m_Ta && x < m_Ta + m_Tv)
        y = m_q0 + (m_vmax_reached + m_v0) * m_Ta / 2. + m_vmax_reached * (x - m_Ta);
    // Deceleration phase
    else if (x >= m_T - m_Td && x < m_T - m_Td + m_Tj2)
        y = m_q1 - (m_vmax_reached + m_v1) * m_Td / 2. + m_vmax_reached * (x - m_T + m_Td) -
            m_jmax_lim * std::pow((x - m_T + m_Td), 3) / 6.;
    else if (x >= m_T - m_Td + m_Tj2 && x < m_T - m_Tj2)
        y = m_q1 - (m_vmax_reached + m_v1) * m_Td / 2. + m_vmax_reached * (x - m_T + m_Td) +
            m_amin_reached / 6. *
                (3 * std::pow((x - m_T + m_Td), 2) - 3 * m_Tj2 * (x - m_T + m_Td) + std::pow(m_Tj2, 2));
    else if (x >= m_T - m_Tj2 && x < m_T)
        y = m_q1 - m_v1 * (m_T - x) - m_jmax_lim * std::pow((m_T - x), 3) / 6.;
    else
        y = m_q1;  // clamp for x > motion time

    return m_sign * y;
}

double ChFunctionConstJerk::GetDer(double x) const {
    double y_dx = 0;

    // Acceleration phase
    if (x >= 0 && x < m_Tj1)
        y_dx = m_v0 + m_jmax_lim * std::pow(x, 2) / 2.;
    else if (x >= m_Tj1 && x < m_Ta - m_Tj1)
        y_dx = m_v0 + m_amax_reached * (x - m_Tj1 / 2.);
    else if (x >= m_Ta - m_Tj1 && x < m_Ta)
        y_dx = m_vmax_reached - m_jmax_lim * std::pow(m_Ta - x, 2) / 2.;  // -jmax = +jmin
    // Constant velocity phase
    else if (x >= m_Ta && x < m_Ta + m_Tv)
        y_dx = m_vmax_reached;
    // Deceleration phase
    else if (x >= m_T - m_Td && x < m_T - m_Td + m_Tj2)
        y_dx = m_vmax_reached - m_jmax_lim * std::pow(x - m_T + m_Td, 2) / 2.;
    else if (x >= m_T - m_Td + m_Tj2 && x < m_T - m_Tj2)
        y_dx = m_vmax_reached + m_amin_reached * (x - m_T + m_Td - m_Tj2 / 2.);
    else if (x >= m_T - m_Tj2 && x < m_T)
        y_dx = m_v1 + m_jmax_lim * std::pow(m_T - x, 2) / 2.;
    else
        y_dx = m_v1;  // clamp for x > motion time

    return m_sign * y_dx;
}

double ChFunctionConstJerk::GetDer2(double x) const {
    double y_dxdx = 0;

    // Acceleration phase
    if (x >= 0 && x < m_Tj1)
        y_dxdx = m_jmax_lim * x;
    else if (x >= m_Tj1 && x < m_Ta - m_Tj1)
        y_dxdx = m_amax_reached;  // = m_jmax_lim * m_Tj1
    else if (x >= m_Ta - m_Tj1 && x < m_Ta)
        y_dxdx = m_jmax_lim * (m_Ta - x);  // -jmax = +jmin
    // Constant velocity phase
    else if (x >= m_Ta && x < m_Ta + m_Tv)
        y_dxdx = 0;
    // Deceleration phase
    else if (x >= m_T - m_Td && x < m_T - m_Td + m_Tj2)
        y_dxdx = -m_jmax_lim * (x - m_T + m_Td);
    else if (x >= m_T - m_Td + m_Tj2 && x < m_T - m_Tj2)
        y_dxdx = m_amin_reached;  // = -m_jmax_lim * m_Tj2
    else if (x >= m_T - m_Tj2 && x < m_T)
        y_dxdx = -m_jmax_lim * (m_T - x);
    else
        y_dxdx = 0;  // clamp for x > motion time

    return m_sign * y_dxdx;
}

double ChFunctionConstJerk::GetDer3(double x) const {
    double y_dxdxdx = 0;

    // Acceleration phase
    if (x >= 0 && x < m_Tj1)
        y_dxdxdx = m_jmax_lim;
    else if (x >= m_Tj1 && x < m_Ta - m_Tj1)
        y_dxdxdx = 0;
    else if (x >= m_Ta - m_Tj1 && x < m_Ta)
        y_dxdxdx = -m_jmax_lim;
    // Constant velocity phase
    else if (x >= m_Ta && x < m_Ta + m_Tv)
        y_dxdxdx = 0;
    // Deceleration phase
    else if (x >= m_T - m_Td && x < m_T - m_Td + m_Tj2)
        y_dxdxdx = -m_jmax_lim;
    else if (x >= m_T - m_Td + m_Tj2 && x < m_T - m_Tj2)
        y_dxdxdx = 0;
    else if (x >= m_T - m_Tj2 && x < m_T)
        y_dxdxdx = m_jmax_lim;
    else
        y_dxdxdx = 0;  // clamp for x > motion time

    return m_sign * y_dxdxdx;
}

void ChFunctionConstJerk::GetBoundaryConditions(double& q0, double& q1, double& v0, double& v1) {
    q0 = m_q0;
    q1 = m_q1;
    v0 = m_v0;
    v1 = m_v1;
}

void ChFunctionConstJerk::GetImposedLimits(double& vmax, double& amax, double& jmax) {
    vmax = m_vmax_lim;
    amax = m_amax_lim;
    jmax = m_jmax_lim;
}

void ChFunctionConstJerk::GetTimes(double& T, double& Ta, double& Tv, double& Td, double& Tj1, double& Tj2) {
    T = m_T;
    Ta = m_Ta;
    Tv = m_Tv;
    Td = m_Td;
    Tj1 = m_Tj1;
    Tj2 = m_Tj2;
}

void ChFunctionConstJerk::GetReachedLimits(double& vlim, double& alim_a, double& alim_d) {
    vlim = m_vmax_reached;
    alim_a = m_amax_reached;
    alim_d = m_amin_reached;
}

void ChFunctionConstJerk::ArchiveOut(ChArchiveOut& archive_out) {
    // serialize parent class
    ChFunction::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_q0);
    archive_out << CHNVP(m_q1);
    archive_out << CHNVP(m_v0);
    archive_out << CHNVP(m_v1);
    archive_out << CHNVP(m_vmax_lim);
    archive_out << CHNVP(m_amax_lim);
    archive_out << CHNVP(m_jmax_lim);
    archive_out << CHNVP(m_Ta);
    archive_out << CHNVP(m_Tv);
    archive_out << CHNVP(m_Td);
    archive_out << CHNVP(m_Tj1);
    archive_out << CHNVP(m_Tj2);
    archive_out << CHNVP(m_T);
    archive_out << CHNVP(m_amax_reached);
    archive_out << CHNVP(m_amin_reached);
    archive_out << CHNVP(m_vmax_reached);
    archive_out << CHNVP(m_sign);
}

void ChFunctionConstJerk::ArchiveIn(ChArchiveIn& archive_in) {
    // deserialize parent class
    ChFunction::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_q0);
    archive_in >> CHNVP(m_q1);
    archive_in >> CHNVP(m_v0);
    archive_in >> CHNVP(m_v1);
    archive_in >> CHNVP(m_vmax_lim);
    archive_in >> CHNVP(m_amax_lim);
    archive_in >> CHNVP(m_jmax_lim);
    archive_in >> CHNVP(m_Ta);
    archive_in >> CHNVP(m_Tv);
    archive_in >> CHNVP(m_Td);
    archive_in >> CHNVP(m_Tj1);
    archive_in >> CHNVP(m_Tj2);
    archive_in >> CHNVP(m_T);
    archive_in >> CHNVP(m_amax_reached);
    archive_in >> CHNVP(m_amin_reached);
    archive_in >> CHNVP(m_vmax_reached);
    archive_in >> CHNVP(m_sign);
}

}  // end namespace chrono
