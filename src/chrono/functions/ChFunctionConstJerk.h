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

#ifndef CHFUNCT_CONSTJERK_H
#define CHFUNCT_CONSTJERK_H

#include "chrono/functions/ChFunctionBase.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Ramp function composed by seven segments with constant jerk.
/// (aka. 'seven segments', 'double-S', 'trapezoidal acceleration').

class ChApi ChFunctionConstJerk : public ChFunction {
  public:
    /// Default constructor.
    ChFunctionConstJerk() {}

    /// Simplified case constant jerk, with imposed boundary conditions and times.
    /// Eg. for q1 > q0, it produces a motion profile characterized by
    /// - time: [0, Tj, Ta-Tj, Ta, Ta+Tv, Ta+Tv+Tj, T-Tj, T]
    /// - jerk: [+jmax, 0, -jmax, 0, -jmax, 0, +jmax]
    /// - acceleration: [+lin, const, -lin, zero, -lin, const, +lin]
    ///
    /// NB: Ta = (0..1/2) * T; Tj = (0..1/2) * Ta
    ChFunctionConstJerk(
        double q0,  ///< start position
        double q1,  ///< end position
        double v0,  ///< start velocity
        double v1,  ///< end velocity
        double T,   ///< total motion time
        double Ta,  ///< acceleration time (corresponds to first accel trapezoid) -> NB: Ta = (0..1/2) * T
        double Tj   ///< jerk time (corresponds to first jerk square wave) -> NB: Tj = (0..1/2) * Ta
    );

    /// Simplified case constant jerk.
    /// Under
    /// - imposed boundary positions
    /// - (assumed) zero boundary velocities
    /// - (assumed) zero boundary accelerations
    /// - symmetric kinematic constraints on max |velocity|, |acceleration|, |jerk|
    /// minimizes total motion time.
    ChFunctionConstJerk(double q0,    ///< start position
                        double q1,    ///< end position
                        double vmax,  ///< kinematic constraint: (abs) max allowed velocity
                        double amax,  ///< kinematic constraint: (abs) max allowed acceleration
                        double jmax   ///< kinematic constraint: (abs) max allowed jerk
    );

    /// General case constant jerk.
    /// Under
    /// - imposed boundary positions
    /// - imposed boundary velocities
    /// - (assumed) zero boundary accelerations
    /// - symmetric kinematic constraints on max |velocity|, |acceleration|, |jerk|
    /// attempts to minimize total motion time.
    ///
    /// NB: if desired motion law is not feasible, everything is set to zero (try to relax constraints).
    ChFunctionConstJerk(bool& feasible,  ///< output: will be set to true if desired motlaw is feasible, false otherwise
                        double q0,       ///< start position
                        double q1,       ///< end position
                        double v0,       ///< start velocity
                        double v1,       ///< end velocity
                        double vmax,     ///< kinematic constraint: (abs) max allowed velocity
                        double amax,     ///< kinematic constraint: (abs) max allowed acceleration
                        double jmax      ///< kinematic constraint: (abs) max allowed jerk
    );

    ChFunctionConstJerk(const ChFunctionConstJerk& other);

    ~ChFunctionConstJerk() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunctionConstJerk* Clone() const override { return new ChFunctionConstJerk(*this); }

    /// Get type of ChFunction.
    virtual Type GetType() const override { return ChFunction::Type::CONSTJERK; }

    /// Setup internal data of constant jerk, imposed times case.
    void Setup(double q0, double q1, double v0, double v1, double T, double Ta, double Tj);

    /// Setup internal data of constant jerk, simplified case for minimization of motion time with v0 = v1 = 0.
    void Setup(double q0, double q1, double vmax, double amax, double jmax);

    /// Setup internal data of constant jerk, general case for minimization of motion time.
    void Setup(bool& feasible, double q0, double q1, double v0, double v1, double vmax, double amax, double jmax);

    /// Position: return the y value of the function, at position x.
    virtual double GetVal(double x) const override;

    /// Velocity: return the dy/dx derivative of the function, at position x.
    virtual double GetDer(double x) const override;

    /// Acceleration: return the ddy/dxdx double derivative of the function, at position x.
    virtual double GetDer2(double x) const override;

    /// Jerk: return the dddy/dxdxdx triple derivative of the function, at position x.
    virtual double GetDer3(double x) const override;

    /// Get total displacement.
    double GetDisplacement() const { return m_q1 - m_q0; }

    /// Get total motion time.
    double GetDuration() const { return m_T; }

    /// Get boundary conditions.
    void GetBoundaryConditions(double& q0, double& q1, double& v0, double& v1);

    /// Get imposed limits on velocity, acceleration and jerk.
    void GetImposedLimits(double& vmax, double& amax, double& jmax);

    /// Get internal motion times.
    void GetTimes(double& T, double& Ta, double& Tv, double& Td, double& Tj1, double& Tj2);

    /// Get maximum velocity/acceleration/deceleration effectively reached during motion law.
    void GetReachedLimits(double& vlim, double& alim_a, double& alim_d);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double m_q0 = 0, m_q1 = 0, m_v0 = 0, m_v1 = 0;  ///< boundary conditions (assumed a0 = a1 = 0)
    double m_vmax_lim = 0;                          ///< max allowed velocity
    double m_amax_lim = 0;                          ///< max allowed acceleration
    double m_jmax_lim = 0;                          ///< max allowed jerk
    double m_Ta = 0;                                ///< accel time
    double m_Tv = 0;                                ///< constant speed time
    double m_Td = 0;                                ///< decel time
    double m_Tj1 = 0;                               ///< jerk time (first part)
    double m_Tj2 = 0;                               ///< jerk time (second part)
    double m_T = 0;                                 ///< total motion time
    double m_amax_reached = 0;                      ///< max acceleration effectively reached
    double m_amin_reached = 0;                      ///< min acceleration effectively reached
    double m_vmax_reached = 0;                      ///< max velocity effectively reached
    int m_sign = 1;                                 ///< adjust motlaw sign: +1 if q1 > q0, -1 if q1 < q0
};

/// @} chrono_functions

}  // end namespace chrono

#endif
