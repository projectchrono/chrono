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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHFUNCTIONPOSITION_SETPOINT_H
#define CHFUNCTIONPOSITION_SETPOINT_H

#include "chrono/functions/ChFunctionPosition.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A motion function p=f(s) where p(t) is an externally-provided sample,
/// as a ZERO_ORDER_HOLD (zero order hold) of FIRST_ORDER_HOLD (first order).
/// You must keep the setpoint p updated via multiple
/// calls to SetSetpoint(), for example calling SetSetpoint()
/// at each timestep in the simulation loop.
/// It is assumed that one will later evaluate GetPos(), GetLinVel() etc. in close vicinity of the setpoint (old
/// setpoints are not saved), preferably ecactly at the same s, but in vicinity of s there will be extrapolation (after
/// last s) and interpolation (before last s) according to: If in ZERO_ORDER_HOLD mode: value p will persist
/// indefinitely until next call, derivative p_ds and p_dsds will be zero. If in FIRST_ORDER_HOLD mode: value p will
/// interpolate linearly from the previous value, derivative p_ds will be constant, p_dsds will be zero. If in SOH mode:
/// value p will interpolate quadratically, derivative p_ds will be linear, p_dsds will be constant. Default: uses
/// FIRST_ORDER_HOLD mode. Use SetMode() to change it.

class ChApi ChFunctionPositionSetpoint : public ChFunctionPosition {
  public:
    ChFunctionPositionSetpoint();
    ChFunctionPositionSetpoint(const ChFunctionPositionSetpoint& other);
    virtual ~ChFunctionPositionSetpoint();

    /// "Virtual" copy constructor.
    virtual ChFunctionPositionSetpoint* Clone() const override { return new ChFunctionPositionSetpoint(*this); }

    /// Type of setpoint interpolation/extrapolation - zero order hold, first order hold, etc.
    enum eChSetpointMode {
        ZERO_ORDER_HOLD,  ///< Zero  Order Hold: p constant,  p_ds = p_dsds = 0                 in neighbour of setpoint
                          ///< s
        FIRST_ORDER_HOLD,  ///< First Order Hold: p linear,    p_ds constant, p_dsds = 0         in neighbour of
                           ///< setpoint s
        SOH,      ///< First Order Hold: p parabolic, p_ds linear,   p_dsds  constant   in neighbour of setpoint s
        OVERRIDE  ///< p, p_ds, p_dsds are set via SetSetpointAndDerivatives() and will be considered constant in
                  ///< GetVal(s) regardless of s until next SetSetpointAndDerivatives()
    };

    /// @cond
    CH_ENUM_MAPPER_BEGIN(eChSetpointMode);
    CH_ENUM_VAL(ZERO_ORDER_HOLD);
    CH_ENUM_VAL(FIRST_ORDER_HOLD);
    CH_ENUM_VAL(SOH);
    CH_ENUM_VAL(OVERRIDE);
    CH_ENUM_MAPPER_END(eChSetpointMode);
    /// @endcond

    /// Sets the extrapolation/interpolation mode
    void SetMode(eChSetpointMode mmode) { mode = mmode; }
    /// Gets the extrapolation/interpolation mode
    eChSetpointMode GetMode() { return mode; }

    /// Use this to go back to s=0 (the SetSetpoint() function works only if called at increasing s values)
    void Reset(double ms = 0);

    /// Set the setpoint, and compute its derivatives (speed, acceleration) automatically
    /// by backward differentiation (only if s is called at increasing small steps, most often s is time).
    /// Note: each time must be called with increasing s so that internally it add sthe 'new' setpoint and scrolls
    /// the previous samples for computing extrapolation/interpolation, but if called multiple times with exactly the
    /// same s value, the buffer of past samples is not scrolled: it just recompute setpoint and
    /// derivatives according to the 'updated' setpoint.
    /// If in ZERO_ORDER_HOLD mode: value p will persist indefinitely until next call, derivative p_ds and p_dsds will
    /// be zero. If in FIRST_ORDER_HOLD mode: value p will interpolate linearly from the previous value, derivative p_ds
    /// will be constant, p_dsds will be zero. If in SOH mode: value p will interpolate quadratically, derivative p_ds
    /// will be linear, p_dsds will be constant.
    virtual void SetSetpoint(ChVector3d p_setpoint, double s);

    /// Set the setpoint, and also its derivatives. Moreover, changes the mode to eChSetpointMode::OVERRIDE, so
    /// all values will persist indefinitely until next call, that is multiple calls to GetPos(s) GetLinVel() etc. will
    /// give same results (non interpolated) regardless of s.
    virtual void SetSetpointAndDerivatives(ChVector3d p_setpoint,
                                           ChVector3d p_setpoint_ds,
                                           ChVector3d p_setpoint_dsds) {
        mode = eChSetpointMode::OVERRIDE;
        P = p_setpoint;
        P_ds = p_setpoint_ds;
        P_dsds = p_setpoint_dsds;
    }

    /// Return the position imposed by the function, at \a s.
    virtual ChVector3d GetPos(double s) const override;

    /// Return the linear velocity imposed by the function, at \a s.
    virtual ChVector3d GetLinVel(double s) const override;

    /// Return the linear acceleration imposed the function, at \a s.
    virtual ChVector3d GetLinAcc(double s) const override;

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    eChSetpointMode mode;
    double S;
    ChVector3d P;
    ChVector3d P_ds;
    ChVector3d P_dsds;
    double last_s;
    ChVector3d last_P;
    ChVector3d last_P_ds;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPositionSetpoint, 0)

}  // end namespace chrono

#endif
