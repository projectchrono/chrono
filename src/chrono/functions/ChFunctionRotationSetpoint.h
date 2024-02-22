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

#ifndef CHFUNCTIONROTATION_SETPOINT_H
#define CHFUNCTIONROTATION_SETPOINT_H

#include "chrono/functions/ChFunctionRotation.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A rotation q=f(s) provided from a rotation sample, continuously updated by the user,
/// behaving as a ZERO_ORDER_HOLD (zero order hold) of FIRST_ORDER_HOLD (first order).
/// You must keep the setpoint q updated via multiple
/// calls to SetSetpoint(), for example calling SetSetpoint()
/// at each timestep in the simulation loop.
/// It is assumed that one will later evaluate GetQuat(), GetAngVel() etc. in close vicinity of the setpoint (setpoint
/// history is not saved), preferably ecactly at the same s, but in vicinity of s there will be extrapolation (after
/// last s) and interpolation (before last s) according to: If in ZERO_ORDER_HOLD mode: rotation q will persist
/// indefinitely until next call, angular velocity and angular acceleration will be zero. If in FIRST_ORDER_HOLD mode:
/// rotation q will interpolate linearly from the previous value, angular velocity will be constant, angular
/// acceleration will be zero. Default: uses FIRST_ORDER_HOLD mode. Use SetMode() to change it.

class ChApi ChFunctionRotationSetpoint : public ChFunctionRotation {
  public:
    ChFunctionRotationSetpoint();
    ChFunctionRotationSetpoint(const ChFunctionRotationSetpoint& other);
    virtual ~ChFunctionRotationSetpoint();

    /// "Virtual" copy constructor.
    virtual ChFunctionRotationSetpoint* Clone() const override { return new ChFunctionRotationSetpoint(*this); }

    /// Type of setpoint interpolation/extrapolation - zero order hold, first order hold, etc.
    enum eChSetpointMode {
        ZERO_ORDER_HOLD,   ///< Zero  Order Hold: q constant,  w = a = 0                 in neighbour of setpoint s
        FIRST_ORDER_HOLD,  ///< First Order Hold: q linear,    w constant, a = 0         in neighbour of setpoint s
        // SOH,	///< Second Order Hold: q parabolic, w linear,   a  constant   in neighbour of setpoint s
        OVERRIDE  ///< p, w, a are set via SetSetpointAndDerivatives() and will be considered constant in GetQuat(s)
                  ///< regardless of s until next SetSetpointAndDerivatives()
    };

    /// @cond
    CH_ENUM_MAPPER_BEGIN(eChSetpointMode);
    CH_ENUM_VAL(ZERO_ORDER_HOLD);
    CH_ENUM_VAL(FIRST_ORDER_HOLD);
    // CH_ENUM_VAL(SOH);
    CH_ENUM_VAL(OVERRIDE);
    CH_ENUM_MAPPER_END(eChSetpointMode);
    /// @endcond

    /// Sets the extrapolation/interpolation mode
    void SetMode(eChSetpointMode mmode) { mode = mmode; }
    /// Gets the extrapolation/interpolation mode
    eChSetpointMode GetMode() { return mode; }

    /// Use this to go back to s=0 (the SetSetpoint() function works only if called at increasing s values)
    void Reset(double ms = 0);

    /// Set the rotation setpoint, and compute its derivatives (angular speed, angular acceleration) automatically
    /// by backward differentiation (only if s is called at increasing small steps, most often s is time).
    /// Note: each time must be called with increasing s so that internally it add sthe 'new' setpoint and scrolls
    /// the previous samples for computing extrapolation/interpolation, but if called multiple times with exactly the
    /// same s value, the buffer of past samples is not scrolled: it just recompute setpoint and
    /// derivatives according to the 'updated' setpoint.
    /// If in ZERO_ORDER_HOLD mode: rotation q will persist indefinitely until next call, angular velocity and angular
    /// acceleration will be zero. If in FIRST_ORDER_HOLD mode: rotation q will interpolate linearly from the previous
    /// value, angular velocity will be constant, angular acceleration will be zero.
    virtual void SetSetpoint(ChQuaternion<> q_setpoint, double s);

    /// Set the setpoint, and also its derivatives (angular velocity, angular acceleration). Moreover, changes the mode
    /// to eChSetpointMode::OVERRIDE, so all values will persist indefinitely until next call, that is multiple calls to
    /// GetQuat(s) GetAngVel() etc. will give same results (non interpolated) regardless of s.
    virtual void SetSetpointAndDerivatives(ChQuaternion<> q_setpoint,
                                           ChVector3d w_loc_setpoint,
                                           ChVector3d a_loc_setpoint) {
        mode = eChSetpointMode::OVERRIDE;
        Q = q_setpoint;
        W = w_loc_setpoint;
        A = a_loc_setpoint;
    }

    /// Return the q value of the function, at s, as q=f(s).
    virtual ChQuaternion<> GetQuat(double s) const override;

    /// Return the derivative of the rotation function, at s, expressed as angular velocity w in local frame.
    virtual ChVector3d GetAngVel(double s) const override;

    /// Return the derivative of the rotation function, at s, expressed as angular acceleration in local frame.
    virtual ChVector3d GetAngAcc(double s) const override;

    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    eChSetpointMode mode;
    double S;
    ChQuaternion<> Q;
    ChVector3d W;
    ChVector3d A;
    double last_S;
    ChQuaternion<> last_Q;
    ChVector3d last_W;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotationSetpoint, 0)

}  // end namespace chrono

#endif
