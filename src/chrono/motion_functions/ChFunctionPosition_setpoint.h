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

#include "chrono/motion_functions/ChFunctionPosition.h"


namespace chrono {

/// @addtogroup chrono_functions
/// @{


/// A motion function p=f(s) where p(t) is an externally-provided sample,
/// as a ZOH (zero order hold) of FOH (first order). 
/// You must keep the setpoint p updated via multiple
/// calls to SetSetpoint(), for example calling SetSetpoint()
/// at each timestep in the simulation loop.
/// It is assumed that one will later evaluate Get_p(), Get_p_ds() etc. in close vicinity of the setpoint (old setpoints are not saved),
/// preferably ecactly at the same s, but in vicinity of s there will be extrapolation (after last s) and interpolation (before last s) according to:
/// If in ZOH mode: value p will persist indefinitely until next call, derivative p_ds and p_dsds will be zero. 
/// If in FOH mode: value p will interpolate linearly from the previous value, derivative p_ds will be constant, p_dsds will be zero.
/// If in SOH mode: value p will interpolate quadratically, derivative p_ds will be linear, p_dsds will be constant.
/// Default: uses FOH mode. Use SetMode() to change it.

class ChApi ChFunctionPosition_setpoint : public ChFunctionPosition {

  public:
	ChFunctionPosition_setpoint();
	ChFunctionPosition_setpoint(const ChFunctionPosition_setpoint& other);
	virtual ~ChFunctionPosition_setpoint();

    /// "Virtual" copy constructor.
    virtual ChFunctionPosition_setpoint* Clone() const override { return new ChFunctionPosition_setpoint(*this); }


	/// Type of setpoint interpolation/extrapolation - zero order hold, first order hold, etc.
	enum eChSetpointMode { 
		ZOH,	///< Zero  Order Hold: p constant,  p_ds = p_dsds = 0                 in neighbour of setpoint s
		FOH,    ///< First Order Hold: p linear,    p_ds constant, p_dsds = 0         in neighbour of setpoint s
		SOH,	///< First Order Hold: p parabolic, p_ds linear,   p_dsds  constant   in neighbour of setpoint s
		OVERRIDE ///< p, p_ds, p_dsds are set via SetSetpointAndDerivatives() and will be considered constant in Get_p(s) regardless of s until next SetSetpointAndDerivatives()
	};

	/// @cond
    CH_ENUM_MAPPER_BEGIN(eChSetpointMode);
    CH_ENUM_VAL(ZOH);
    CH_ENUM_VAL(FOH);
	CH_ENUM_VAL(SOH);
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

	/// Set the setpoint, and compute its derivatives (speed, acceleration) automatically
    /// by backward differentiation (only if s is called at increasing small steps, most often s is time).
	/// Note: each time must be called with increasing s so that internally it add sthe 'new' setpoint and scrolls 
	/// the previous samples for computing extrapolation/interpolation, but if called multiple times with exactly the 
	/// same s value, the buffer of past samples is not scrolled: it just recompute setpoint and 
	/// derivatives according to the 'updated' setpoint.
    /// If in ZOH mode: value p will persist indefinitely until next call, derivative p_ds and p_dsds will be zero. 
	/// If in FOH mode: value p will interpolate linearly from the previous value, derivative p_ds will be constant, p_dsds will be zero.
	/// If in SOH mode: value p will interpolate quadratically, derivative p_ds will be linear, p_dsds will be constant.
    virtual void SetSetpoint(ChVector<> p_setpoint, double s);

    /// Set the setpoint, and also its derivatives. Moreover, changes the mode to eChSetpointMode::OVERRIDE, so 
    /// all values will persist indefinitely until next call, that is multiple calls to Get_p(s) Get_p_ds() etc. will give same 
	/// results (non interpolated) regardless of s.
    virtual void SetSetpointAndDerivatives(ChVector<> p_setpoint, ChVector<> p_setpoint_ds, ChVector<> p_setpoint_dsds) {
		mode = eChSetpointMode::OVERRIDE;
        P = p_setpoint;
        P_ds = p_setpoint_ds;
        P_dsds = p_setpoint_dsds;
    }
	

    /// Return the p value of the function, at s, as p=f(s).
	virtual ChVector<> Get_p(double s) const override;

    /// Return the dp/ds derivative of the function, at s.
	virtual ChVector<> Get_p_ds(double s) const override;

    /// Return the ddp/dsds double derivative of the function, at s.
    virtual ChVector<> Get_p_dsds(double s) const override;


    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

	

private:
	eChSetpointMode mode;
	double	   S;
	ChVector<> P;
    ChVector<> P_ds;
    ChVector<> P_dsds;
	double last_s;
	ChVector<> last_P;
    ChVector<> last_P_ds;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPosition_setpoint, 0)

}  // end namespace chrono

#endif
