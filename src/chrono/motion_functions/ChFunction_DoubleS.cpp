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

#include "chrono/motion_functions/ChFunction_DoubleS.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunction_DoubleS)


/// Simplified case Double-S, with imposed boundary conditions and times.
/// Eg. for q1 > q0, it produces a motion profile characterized by
/// - time: [0, Tj, Ta-Tj, Ta, Ta+Tv, Ta+Tv+Tj, T-Tj, T]
/// - jerk: [+jmax, 0, -jmax, 0, -jmax, 0, +jmax]
/// - acceleration: [+lin, const, -lin, zero, -lin, const, +lin]
/// 
/// NB: Ta = (0..1/2) * T; Tj = (0..1/2) * Ta
ChFunction_DoubleS::ChFunction_DoubleS(
    double q0,      ///< start position
    double q1,      ///< end position
    double v0,      ///< start velocity
    double v1,      ///< end velocity
    double T,       ///< total motion time
    double Ta,      ///< acceleration time (corresponds to first accel trapezoid) -> NB: Ta = (0..1/2) * T
    double Tj       ///< jerk time (corresponds to first jerk square wave) -> NB: Tj = (0..1/2) * Ta
)
{
    Setup_Data(q0, q1, v0, v1, T, Ta, Tj);
}

/// Simplified case Double-S:
/// under
/// - imposed boundary positions
/// - (assumed) zero boundary velocities
/// - (assumed) zero boundary accelerations
/// - symmetric kinematic constraints on max |velocity|, |acceleration|, |jerk|
/// minimize total motion time.
ChFunction_DoubleS::ChFunction_DoubleS(
    double q0,      ///< start position
    double q1,      ///< end position
    double vmax,    ///< kinematic constraint: (abs) max allowed velocity
    double amax,    ///< kinematic constraint: (abs) max allowed acceleration
    double jmax     ///< kinematic constraint: (abs) max allowed jerk
)
{
    Setup_Data(q0, q1, vmax, amax, jmax);
}

/// General case Double-S:
/// under
/// - imposed boundary positions
/// - imposed boundary velocities
/// - (assumed) zero boundary accelerations
/// - symmetric kinematic constraints on max |velocity|, |acceleration|, |jerk|
/// attempt to minimize total motion time.
/// 
/// NB: if desired motion law is not feasible, everything is set to zero (try to relax constraints).
ChFunction_DoubleS::ChFunction_DoubleS(
    bool& feasible,     ///< output: will be set to true if desired motlaw is feasible, false otherwise
    double q0,          ///< start position
    double q1,          ///< end position
    double v0,          ///< start velocity
    double v1,          ///< end velocity
    double vmax,        ///< kinematic constraint: (abs) max allowed velocity
    double amax,        ///< kinematic constraint: (abs) max allowed acceleration
    double jmax         ///< kinematic constraint: (abs) max allowed jerk
)
{
    if (v0 == 0 && v1 == 0) { // fallback to simplified case
        feasible = true;
        Setup_Data(q0, q1, vmax, amax, jmax);
    }
    else
        Setup_Data(feasible, q0, q1, v0, v1, vmax, amax, jmax); // general case
}

ChFunction_DoubleS::ChFunction_DoubleS(const ChFunction_DoubleS& other) {
    m_q0 = other.m_q0;
    m_q1 = other.m_q1;
    m_v0 = other.m_v0;
    m_v1 = other.m_v1;
    m_vmax = other.m_vmax;
    m_amax = other.m_amax;
    m_jmax = other.m_jmax;
    m_Ta = other.m_Ta;
    m_Tv = other.m_Tv;
    m_Td = other.m_Td;
    m_Tj1 = other.m_Tj1;
    m_Tj2 = other.m_Tj2;
    m_T = other.m_T;
    m_alim_a = other.m_alim_a;
    m_alim_d = other.m_alim_d;
    m_vlim = other.m_vlim;
}

/// Setup internal data of Double-S, imposed times case.
void ChFunction_DoubleS::Setup_Data(double q0, double q1, double v0, double v1, double T, double Ta, double Tj) {
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
    m_vmax = h / (m_T - m_Ta);
    m_amax = h / (m_T - m_Ta) / (m_Ta - m_Tj1);
    m_jmax = h / (m_T - m_Ta) / (m_Ta - m_Tj1) / m_Tj1;
    m_vlim = m_vmax;
    m_alim_a = m_amax;
    m_alim_d = -m_amax;
}

/// Setup internal data of Double-S, simplified case for minimization of motion time with v0 = v1 = 0.
void ChFunction_DoubleS::Setup_Data(double q0, double q1, double vmax, double amax, double jmax) {
    // Adjust motion law sign
    m_sign = (q1 > q0) ? 1 : -1;

    m_q0 = m_sign * q0;
    m_q1 = m_sign * q1;
    m_v0 = 0;
    m_v1 = 0;
    m_vmax = vmax;
    m_amax = amax;
    m_jmax = jmax;

    // Initial useful data
    double h = m_q1 - m_q0; // total displacement
    double amax2 = m_amax * m_amax; // amax squared
    double Tj = 0;

    if (m_vmax * m_jmax >= amax2) {
        Tj = m_amax / m_jmax;
        m_Tj1 = Tj;
        m_Tj2 = Tj;
        m_Ta = Tj + m_vmax / m_amax;
    }
    else {
        Tj = sqrt(m_vmax / m_jmax);
        m_Tj1 = Tj;
        m_Tj2 = Tj;
        m_Ta = 2 * Tj;
    }

    m_Tv = h / m_vmax - m_Ta;

    // No constant speed tract
    if (m_Tv <= 0) {
        m_Tv = 0; // adjust

        if (h >= 2 * pow(m_amax, 3) / pow(m_jmax, 2)) {
            Tj = m_amax / m_jmax;
            m_Tj1 = Tj;
            m_Tj2 = Tj;
            m_Ta = 0.5 * Tj + sqrt(pow(0.5 * Tj, 2) + h / m_amax);
        }
        else {
            Tj = pow(h / (2 * m_jmax), 1. / 3.);
            m_Tj1 = Tj;
            m_Tj2 = Tj;
            m_Ta = 2 * Tj;
        }
    }

    // Total actuation time
    m_Td = m_Ta;
    m_T = m_Ta + m_Tv + m_Td;

    // Finally, compute max acceleration and velocity effectively reached
    double alim = m_jmax * Tj;
    m_alim_a = alim;
    m_alim_d = -m_alim_a;
    m_vlim = (m_Ta - Tj) * alim;
}

/// Setup internal data of Double-S, general case for minimization of motion time.
void ChFunction_DoubleS::Setup_Data(bool& feasible, double q0, double q1, double v0, double v1, double vmax, double amax, double jmax) {
    // Adjust motion law sign
    m_sign = (q1 > q0) ? 1 : -1;

    m_q0 = m_sign * q0;
    m_q1 = m_sign * q1;
    m_v0 = m_sign * v0;
    m_v1 = m_sign * v1;
    m_vmax = vmax;
    m_amax = amax;
    m_jmax = jmax;

    // Initial useful data
    double h = m_q1 - m_q0; // total displacement
    double amax2 = m_amax * m_amax; // amax squared

    // Check motion law feasability --------------------
    double Tj_star1 = sqrt(std::abs(m_v1 - m_v0) / m_jmax);
    double Tj_star2 = m_amax / m_jmax;
    double Tj_star = std::min(Tj_star1, Tj_star2);

    feasible = false; // assume not feasible for now
    if (Tj_star < Tj_star2) {
        double tmp = Tj_star * (m_v0 + m_v1);
        if (h > tmp)
            feasible = true;
    }
    else { // Tj_star == Tj_star1
        double tmp = 0.5 * (m_v0 + m_v1) * (Tj_star + std::abs(m_v1 - m_v0) / m_amax);
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
        m_alim_a = 0;
        m_alim_d = 0;
        m_vmax = 0;
        return;
    }

    // Case 1: vlim = vmax -----------------------------
    // For now, assume vmax and amax are reached

    // acceleration time intervals
    if ((m_vmax - m_v0) * m_jmax < amax2) { // amax is not reached
        m_Tj1 = sqrt((m_vmax - m_v0) / m_jmax);
        m_Ta = 2 * m_Tj1;
    }
    else { // amax is reached
        m_Tj1 = amax / jmax;
        m_Ta = m_Tj1 + (m_vmax - m_v0) / m_amax;
    }

    // deceleration time intervals
    if ((m_vmax - m_v1) * m_jmax < amax2) { // amin is not reached
        m_Tj2 = sqrt((m_vmax - m_v1) / m_jmax);
        m_Td = 2 * m_Tj2;
    }
    else { // amin is reached
        m_Tj2 = m_amax / m_jmax;
        m_Td = m_Tj2 + (m_vmax - m_v1) / m_amax;
    }

    m_Tv = h / m_vmax - 0.5 * m_Ta * (1 + m_v0 / m_vmax) - 0.5 * m_Td * (1 + m_v1 / m_vmax); // constant velocity time

    // Case 2: vlim < vmax -----------------------------
    // If Tv > 0, vmax is effectively reached; otherwise, recompute times

    if (m_Tv <= 0) { // no constant speed tract
        m_Tv = 0; // adjust
        double Tj = m_amax / m_jmax; // Tj1 = Tj2 == Tj
        m_Tj1 = Tj;
        m_Tj2 = Tj;

        double delta = pow(m_amax, 4) / pow(m_jmax, 2)
            + 2 * (pow(m_v0, 2) + pow(m_v1, 2))
            + m_amax * (4 * h - 2 * m_amax / m_jmax * (m_v0 + m_v1));
        m_Ta = (amax2 / m_jmax - 2 * m_v0 + sqrt(delta)) / (2 * m_amax);
        m_Td = (amax2 / m_jmax - 2 * m_v1 + sqrt(delta)) / (2 * m_amax);
    }

    if (m_Ta < 0 || m_Td < 0) {
        if (m_Ta < 0) {
            m_Ta = 0; // adjust
            m_Td = 2 * h / (m_v1 + m_v0);
            m_Tj2 = (m_jmax * h - sqrt(m_jmax * (m_jmax * pow(h, 2) + pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)))) / (m_jmax * (m_v1 + m_v0));
        }
        else if (m_Td < 0) {
            m_Td = 0; // adjust
            m_Ta = 2 * h / (v1 + v0);
            m_Tj1 = (m_jmax * h - sqrt(m_jmax * (m_jmax * pow(h, 2) - pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)))) / (m_jmax * (m_v1 + m_v0));
        }
    }
    else {
        if (m_Ta < 2 * m_Tj1 || m_Td < 2 * m_Tj1) { // = Tj
            GetLog() << "ChFunction_DoubleS corner case: decrease amax. \n";
            // TODO: iterative refining (?)
        }
    }

    // Total actuation time
    m_T = m_Ta + m_Tv + m_Td;

    // Finally, compute max acceleration and velocity effectively reached
    m_alim_a = m_jmax * m_Tj1;
    m_alim_d = -m_jmax * m_Tj2;
    m_vlim = m_v0 + (m_Ta - m_Tj1) * m_alim_a; // = m_v1 - (m_Td - m_Tj2) * m_alim_d
}

/// Position: return the y value of the function, at position x.
double ChFunction_DoubleS::Get_y(double x) const {
    double y = 0;

    // Acceleration phase
    if (x >= 0 && x < m_Tj1)
        y = m_q0 + m_v0 * x + m_jmax * pow(x, 3) / 6.;
    else if (x >= m_Tj1 && x < m_Ta - m_Tj1)
        y = m_q0 + m_v0 * x + m_alim_a / 6. * (3 * pow(x, 2) - 3 * m_Tj1 * x + pow(m_Tj1, 2));
    else if (x >= m_Ta - m_Tj1 && x < m_Ta)
        y = m_q0 + (m_vlim + m_v0) * m_Ta / 2. - m_vlim * (m_Ta - x) + m_jmax * pow((m_Ta - x), 3) / 6.; // +jmax = -jmin
    // Constant velocity phase
    else if (x >= m_Ta && x < m_Ta + m_Tv)
        y = m_q0 + (m_vlim + m_v0) * m_Ta / 2. + m_vlim * (x - m_Ta);
    // Deceleration phase
    else if (x >= m_T - m_Td && x < m_T - m_Td + m_Tj2)
        y = m_q1 - (m_vlim + m_v1) * m_Td / 2. + m_vlim * (x - m_T + m_Td) - m_jmax * pow((x - m_T + m_Td), 3) / 6.;
    else if (x >= m_T - m_Td + m_Tj2 && x < m_T - m_Tj2)
        y = m_q1 - (m_vlim + m_v1) * m_Td / 2. + m_vlim * (x - m_T + m_Td) + m_alim_d / 6. * (3 * pow((x - m_T + m_Td), 2) - 3 * m_Tj2 * (x - m_T + m_Td) + pow(m_Tj2, 2));
    else if (x >= m_T - m_Tj2 && x < m_T)
        y = m_q1 - m_v1 * (m_T - x) - m_jmax * pow((m_T - x), 3) / 6.;
    else
        y = m_q1; // clamp for x > motion time

    return m_sign * y;
}

/// Velocity: return the dy/dx derivative of the function, at position x.
double ChFunction_DoubleS::Get_y_dx(double x) const {
    double y_dx = 0;

    // Acceleration phase
    if (x >= 0 && x < m_Tj1)
        y_dx = m_v0 + m_jmax * pow(x, 2) / 2.;
    else if (x >= m_Tj1 && x < m_Ta - m_Tj1)
        y_dx = m_v0 + m_alim_a * (x - m_Tj1 / 2.);
    else if (x >= m_Ta - m_Tj1 && x < m_Ta)
        y_dx = m_vlim - m_jmax * pow(m_Ta - x, 2) / 2.; // -jmax = +jmin
    // Constant velocity phase
    else if (x >= m_Ta && x < m_Ta + m_Tv)
        y_dx = m_vlim;
    // Deceleration phase
    else if (x >= m_T - m_Td && x < m_T - m_Td + m_Tj2)
        y_dx = m_vlim - m_jmax * pow(x - m_T + m_Td, 2) / 2.;
    else if (x >= m_T - m_Td + m_Tj2 && x < m_T - m_Tj2)
        y_dx = m_vlim + m_alim_d * (x - m_T + m_Td - m_Tj2 / 2.);
    else if (x >= m_T - m_Tj2 && x < m_T)
        y_dx = m_v1 + m_jmax * pow(m_T - x, 2) / 2.;
    else
        y_dx = m_v1; // clamp for x > motion time

    return m_sign * y_dx;
}

/// Acceleration: return the ddy/dxdx double derivative of the function, at position x.
double ChFunction_DoubleS::Get_y_dxdx(double x) const {
    double y_dxdx = 0;

    // Acceleration phase
    if (x >= 0 && x < m_Tj1)
        y_dxdx = m_jmax * x;
    else if (x >= m_Tj1 && x < m_Ta - m_Tj1)
        y_dxdx = m_alim_a; // = m_jmax * m_Tj1
    else if (x >= m_Ta - m_Tj1 && x < m_Ta)
        y_dxdx = m_jmax * (m_Ta - x); // -jmax = +jmin
    // Constant velocity phase
    else if (x >= m_Ta && x < m_Ta + m_Tv)
        y_dxdx = 0;
    // Deceleration phase
    else if (x >= m_T - m_Td && x < m_T - m_Td + m_Tj2)
        y_dxdx = -m_jmax * (x - m_T + m_Td);
    else if (x >= m_T - m_Td + m_Tj2 && x < m_T - m_Tj2)
        y_dxdx = m_alim_d; // = -m_jmax * m_Tj2
    else if (x >= m_T - m_Tj2 && x < m_T)
        y_dxdx = -m_jmax * (m_T - x);
    else
        y_dxdx = 0; // clamp for x > motion time

    return m_sign * y_dxdx;
}

/// Jerk: return the dddy/dxdxdx triple derivative of the function, at position x.
double ChFunction_DoubleS::Get_y_dxdxdx(double x) const {
    double y_dxdxdx = 0;

    // Acceleration phase
    if (x >= 0 && x < m_Tj1)
        y_dxdxdx = m_jmax;
    else if (x >= m_Tj1 && x < m_Ta - m_Tj1)
        y_dxdxdx = 0;
    else if (x >= m_Ta - m_Tj1 && x < m_Ta)
        y_dxdxdx = -m_jmax;
    // Constant velocity phase
    else if (x >= m_Ta && x < m_Ta + m_Tv)
        y_dxdxdx = 0;
    // Deceleration phase
    else if (x >= m_T - m_Td && x < m_T - m_Td + m_Tj2)
        y_dxdxdx = -m_jmax;
    else if (x >= m_T - m_Td + m_Tj2 && x < m_T - m_Tj2)
        y_dxdxdx = 0;
    else if (x >= m_T - m_Tj2 && x < m_T)
        y_dxdxdx = m_jmax;
    else
        y_dxdxdx = 0; // clamp for x > motion time

    return m_sign * y_dxdxdx;
}

/// Get boundary conditions.
void ChFunction_DoubleS::Get_Bounds(double& q0, double& q1, double& v0, double& v1) {
    q0 = m_q0;
    q1 = m_q1;
    v0 = m_v0;
    v1 = m_v1;
}

/// Get kinematic constraints.
void ChFunction_DoubleS::Get_Constraints(double& vmax, double& amax, double& jmax) {
    vmax = m_vmax;
    amax = m_amax;
    jmax = m_jmax;
}

/// Get internal motion times.
void ChFunction_DoubleS::Get_Times(double& T, double& Ta, double& Tv, double& Td, double& Tj1, double& Tj2) {
    T = m_T;
    Ta = m_Ta;
    Tv = m_Tv;
    Td = m_Td;
    Tj1 = m_Tj1;
    Tj2 = m_Tj2;
}

/// Get maximum velocity/acceleration/deceleration effectively reached during motion law.
void ChFunction_DoubleS::Get_Limits(double& vlim, double& alim_a, double& alim_d) {
    vlim = m_vlim;
    alim_a = m_alim_a;
    alim_d = m_alim_d;
}

/// Method to allow serialization of transient data to archives.
void ChFunction_DoubleS::ArchiveOut(ChArchiveOut& marchive) {
    // serialize parent class
    ChFunction::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(m_q0);
    marchive << CHNVP(m_q1);
    marchive << CHNVP(m_v0);
    marchive << CHNVP(m_v1);
    marchive << CHNVP(m_vmax);
    marchive << CHNVP(m_amax);
    marchive << CHNVP(m_jmax);
    marchive << CHNVP(m_Ta);
    marchive << CHNVP(m_Tv);
    marchive << CHNVP(m_Td);
    marchive << CHNVP(m_Tj1);
    marchive << CHNVP(m_Tj2);
    marchive << CHNVP(m_T);
    marchive << CHNVP(m_alim_a);
    marchive << CHNVP(m_alim_d);
    marchive << CHNVP(m_vlim);
    marchive << CHNVP(m_sign);

}

/// Method to allow de-serialization of transient data from archives.
void ChFunction_DoubleS::ArchiveIn(ChArchiveIn& marchive) {
    // deserialize parent class
    ChFunction::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_q0);
    marchive >> CHNVP(m_q1);
    marchive >> CHNVP(m_v0);
    marchive >> CHNVP(m_v1);
    marchive >> CHNVP(m_vmax);
    marchive >> CHNVP(m_amax);
    marchive >> CHNVP(m_jmax);
    marchive >> CHNVP(m_Ta);
    marchive >> CHNVP(m_Tv);
    marchive >> CHNVP(m_Td);
    marchive >> CHNVP(m_Tj1);
    marchive >> CHNVP(m_Tj2);
    marchive >> CHNVP(m_T);
    marchive >> CHNVP(m_alim_a);
    marchive >> CHNVP(m_alim_d);
    marchive >> CHNVP(m_vlim);
    marchive >> CHNVP(m_sign);
}


} // end namespace chrono