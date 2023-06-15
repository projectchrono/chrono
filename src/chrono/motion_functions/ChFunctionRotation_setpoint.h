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

#include "chrono/motion_functions/ChFunctionRotation.h"


namespace chrono {

/// @addtogroup chrono_functions
/// @{


/// A rotation q=f(s) provided from a rotation sample, continuously updated by the user,
/// behaving as a ZOH (zero order hold) of FOH (first order).
/// You must keep the setpoint q updated via multiple
/// calls to SetSetpoint(), for example calling SetSetpoint()
/// at each timestep in the simulation loop.
/// It is assumed that one will later evaluate Get_q(), Get_w_loc() etc. in close vicinity of the setpoint (setpoint history is not saved),
/// preferably ecactly at the same s, but in vicinity of s there will be extrapolation (after last s) and interpolation (before last s) according to:
/// If in ZOH mode: rotation q will persist indefinitely until next call, angular velocity and angular acceleration will be zero. 
/// If in FOH mode: rotation q will interpolate linearly from the previous value, angular velocity will be constant, angular acceleration will be zero.
/// Default: uses FOH mode. Use SetMode() to change it.

class ChApi ChFunctionRotation_setpoint : public ChFunctionRotation {

  public:
	ChFunctionRotation_setpoint();
	ChFunctionRotation_setpoint(const ChFunctionRotation_setpoint& other);
	virtual ~ChFunctionRotation_setpoint();

    /// "Virtual" copy constructor.
    virtual ChFunctionRotation_setpoint* Clone() const override { return new ChFunctionRotation_setpoint(*this); }


	/// Type of setpoint interpolation/extrapolation - zero order hold, first order hold, etc.
	enum eChSetpointMode { 
		ZOH,	///< Zero  Order Hold: q constant,  w = a = 0                 in neighbour of setpoint s
		FOH,    ///< First Order Hold: q linear,    w constant, a = 0         in neighbour of setpoint s
		//SOH,	///< First Order Hold: q parabolic, w linear,   a  constant   in neighbour of setpoint s
		OVERRIDE ///< p, w, a are set via SetSetpointAndDerivatives() and will be considered constant in Get_q(s) regardless of s until next SetSetpointAndDerivatives()
	};

	/// @cond
    CH_ENUM_MAPPER_BEGIN(eChSetpointMode);
    CH_ENUM_VAL(ZOH);
    CH_ENUM_VAL(FOH);
	//CH_ENUM_VAL(SOH);
    CH_ENUM_VAL(OVERRIDE);
    CH_ENUM_MAPPER_END(eChSetpointMode);
	/// @endcond

	/// Sets the extrapolation/interpolation mode
	void SetMode(eChSetpointMode mmode) {
		mode = mmode;
	}
	/// Gets the extrapolation/interpolation mode
	eChSetpointMode GetMode() {
		return mode;
	}

	/// Use this to go back to s=0 (the SetSetpoint() function works only if called at increasing s values)
	void Reset(double ms = 0);

	/// Set the rotation setpoint, and compute its derivatives (angular speed, angular acceleration) automatically
    /// by backward differentiation (only if s is called at increasing small steps, most often s is time).
	/// Note: each time must be called with increasing s so that internally it add sthe 'new' setpoint and scrolls 
	/// the previous samples for computing extrapolation/interpolation, but if called multiple times with exactly the 
	/// same s value, the buffer of past samples is not scrolled: it just recompute setpoint and 
	/// derivatives according to the 'updated' setpoint.
	/// If in ZOH mode: rotation q will persist indefinitely until next call, angular velocity and angular acceleration will be zero. 
	/// If in FOH mode: rotation q will interpolate linearly from the previous value, angular velocity will be constant, angular acceleration will be zero.
    virtual void SetSetpoint(ChQuaternion<> q_setpoint, double s);

    /// Set the setpoint, and also its derivatives (angular velocity, angular acceleration). Moreover, changes the mode to eChSetpointMode::OVERRIDE, so 
    /// all values will persist indefinitely until next call, that is multiple calls to Get_q(s) Get_w_loc() etc. will give same 
	/// results (non interpolated) regardless of s.
    virtual void SetSetpointAndDerivatives(ChQuaternion<> q_setpoint, ChVector<> w_loc_setpoint, ChVector<> a_loc_setpoint) {
		mode = eChSetpointMode::OVERRIDE;
        Q = q_setpoint;
        W = w_loc_setpoint;
        A = a_loc_setpoint;
    }
	

	/// Return the q value of the function, at s, as q=f(s).
    virtual ChQuaternion<> Get_q(double s) const override;

    /// Return the derivative of the rotation function, at s, expressed as angular velocity w in local frame.
	virtual ChVector<> Get_w_loc(double s) const override;

    /// Return the derivative of the rotation function, at s, expressed as angular acceleration in local frame.
	virtual ChVector<> Get_a_loc(double s) const override;


    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

	

private:
	eChSetpointMode mode;
	double	   S;
	ChQuaternion<> Q;
    ChVector<>     W;
    ChVector<>     A;
	double	       last_S;
	ChQuaternion<> last_Q;
    ChVector<>     last_W;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotation_setpoint, 0)

}  // end namespace chrono

#endif
