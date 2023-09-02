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

#ifndef CHFUNCT_DOUBLES_H
#define CHFUNCT_DOUBLES_H

#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{


/// Double-S ramp function composed by seven tracts with constant jerk
/// (aka. 'seven segments', 'constant jerk', 'trapezoidal acceleration').

class ChApi ChFunction_DoubleS : public ChFunction {
public:
    /// Default constructor.
    ChFunction_DoubleS() {}

    /// Simplified case Double-S, with imposed boundary conditions and times.
    /// Eg. for q1 > q0, it produces a motion profile characterized by
    /// - time: [0, Tj, Ta-Tj, Ta, Ta+Tv, Ta+Tv+Tj, T-Tj, T]
    /// - jerk: [+jmax, 0, -jmax, 0, -jmax, 0, +jmax]
    /// - acceleration: [+lin, const, -lin, zero, -lin, const, +lin]
    /// 
    /// NB: Ta = (0..1/2) * T; Tj = (0..1/2) * Ta
    ChFunction_DoubleS(
        double q0,      ///< start position
        double q1,      ///< end position
        double v0,      ///< start velocity
        double v1,      ///< end velocity
        double T,       ///< total motion time
        double Ta,      ///< acceleration time (corresponds to first accel trapezoid) -> NB: Ta = (0..1/2) * T
        double Tj       ///< jerk time (corresponds to first jerk square wave) -> NB: Tj = (0..1/2) * Ta
    );

    /// Simplified case Double-S:
    /// under
    /// - imposed boundary positions
    /// - (assumed) zero boundary velocities
    /// - (assumed) zero boundary accelerations
    /// - symmetric kinematic constraints on max |velocity|, |acceleration|, |jerk|
    /// minimizes total motion time.
    ChFunction_DoubleS(
        double q0,      ///< start position
        double q1,      ///< end position
        double vmax,    ///< kinematic constraint: (abs) max allowed velocity
        double amax,    ///< kinematic constraint: (abs) max allowed acceleration
        double jmax     ///< kinematic constraint: (abs) max allowed jerk
    );

    /// General case Double-S:
    /// under
    /// - imposed boundary positions
    /// - imposed boundary velocities
    /// - (assumed) zero boundary accelerations
    /// - symmetric kinematic constraints on max |velocity|, |acceleration|, |jerk|
    /// attempts to minimize total motion time.
    /// 
    /// NB: if desired motion law is not feasible, everything is set to zero (try to relax constraints).
    ChFunction_DoubleS(
        bool& feasible,     ///< output: will be set to true if desired motlaw is feasible, false otherwise
        double q0,          ///< start position
        double q1,          ///< end position
        double v0,          ///< start velocity
        double v1,          ///< end velocity
        double vmax,        ///< kinematic constraint: (abs) max allowed velocity
        double amax,        ///< kinematic constraint: (abs) max allowed acceleration
        double jmax         ///< kinematic constraint: (abs) max allowed jerk
    );

    ChFunction_DoubleS(const ChFunction_DoubleS& other);

    ~ChFunction_DoubleS() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_DoubleS* Clone() const override { return new ChFunction_DoubleS(*this); }

    /// Get type of ChFunction.
    virtual FunctionType Get_Type() const override { return FUNCT_DOUBLES; }

    /// Setup internal data of Double-S, imposed times case.
    void Setup_Data(double q0, double q1, double v0, double v1, double T, double Ta, double Tj);

    /// Setup internal data of Double-S, simplified case for minimization of motion time with v0 = v1 = 0.
    void Setup_Data(double q0, double q1, double vmax, double amax, double jmax);

    /// Setup internal data of Double-S, general case for minimization of motion time.
    void Setup_Data(bool& feasible, double q0, double q1, double v0, double v1, double vmax, double amax, double jmax);

    /// Position: return the y value of the function, at position x.
    virtual double Get_y(double x) const override;

    /// Velocity: return the dy/dx derivative of the function, at position x.
    virtual double Get_y_dx(double x) const override;

    /// Acceleration: return the ddy/dxdx double derivative of the function, at position x.
    virtual double Get_y_dxdx(double x) const override;

    /// Jerk: return the dddy/dxdxdx triple derivative of the function, at position x.
    virtual double Get_y_dxdxdx(double x) const override;

    /// Get total displacement.
    double Get_h() const { return m_q1 - m_q0; }

    /// Get total motion time.
    double Get_end() const { return m_T; }

    /// Get boundary conditions.
    void Get_Bounds(double& q0, double& q1, double& v0, double& v1);

    /// Get kinematic constraints.
    void Get_Constraints(double& vmax, double& amax, double& jmax);

    /// Get internal motion times.
    void Get_Times(double& T, double& Ta, double& Tv, double& Td, double& Tj1, double& Tj2);

    /// Get maximum velocity/acceleration/deceleration effectively reached during motion law.
    void Get_Limits(double& vlim, double& alim_a, double& alim_d);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

private:
    double m_q0 = 0, m_q1 = 0, m_v0 = 0, m_v1 = 0;              ///< boundary conditions (assumed a0 = a1 = 0)
    double m_vmax = 0, m_amax = 0, m_jmax = 0;                  ///< kinematic constraints
    double m_Ta = 0, m_Tv = 0, m_Td = 0, m_Tj1 = 0, m_Tj2 = 0;  ///< internal motion law times (Ta: accel time, Tv: constant speed time, Td: decel time, Tj1/Tj2: jerk times)
    double m_T = 0;                                             ///< total motion time
    double m_alim_a = 0, m_alim_d = 0, m_vlim = 0;              ///< max acceleration/deceleration/velocity effectively reached
    int m_sign = 1;                                             ///< adjust motlaw sign: +1 if q1 > q0, -1 if q1 < q0
};


/// @} chrono_functions

} // end namespace chrono

#endif
