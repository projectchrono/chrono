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

#ifndef CHFUNCTIONROTATION_AXIS_H
#define CHFUNCTIONROTATION_AXIS_H

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunctionRotation.h"


namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A rotation function q=f(s) where q(s) is defined with axis V and angle alpha, assuming fixed axis of rotation V
/// and a angle of rotation alpha about that axis, expressed with a ChFunction object  alpha=alpha(s).
/// By default, the axis is the Z axis, and rotation is constant zero rotation.

class ChApi ChFunctionRotation_axis : public ChFunctionRotation {

  public:
	ChFunctionRotation_axis();
	ChFunctionRotation_axis(const ChFunctionRotation_axis& other);
	virtual ~ChFunctionRotation_axis();

    /// "Virtual" copy constructor.
    virtual ChFunctionRotation_axis* Clone() const override { return new ChFunctionRotation_axis(*this); }

	/// Set the angle(s) function expressing the rotation about the fixed axis of rotation. Angle assumed in radians.
	void SetFunctionAngle(std::shared_ptr<ChFunction> mx) {
		this->fangle = mx;
	}
	/// Get the angle(s) function expressing the rotation about the fixed axis of rotation. Angle assumed in radians.
	std::shared_ptr<ChFunction> GetFunctionAngle() {
		return this->fangle;
	}

	/// Set the fixed axis of rotation. It must be unit length. Default Z vector.
	void SetAxis(const ChVector<>& mv) {
		this->axis = mv;
	}
	/// Get the fixed axis of rotation. 
	ChVector<> GetAxis() {
		return this->axis;
	}

    
   
	/// Return the rotation as a quaternion, function of s, as q=f(s).
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
	std::shared_ptr<ChFunction> fangle;
	ChVector<> axis;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotation_axis, 0)

}  // end namespace chrono

#endif
