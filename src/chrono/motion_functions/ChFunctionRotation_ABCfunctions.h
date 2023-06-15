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

#ifndef CHFUNCTIONROTATION_ABCFUNCTIONS_H
#define CHFUNCTIONROTATION_ABCFUNCTIONS_H

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunctionRotation.h"


namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A rotation function q=f(s) where q(s) is defined with three ChFunction objects, each per 
/// an an angle in an intrinsic triplets of angles (ex. Eulero angles, Cardano angles, etc),
/// ex. A=A(s), B=B(s), C=C(s).
/// By default, rotation is constant zero rotation. 
/// By default, the angleset is AngleSet::RXYZ (sequence: X-Y'-Z'' intrinsic), may be changed.

class ChApi ChFunctionRotation_ABCfunctions : public ChFunctionRotation {

  public:
	ChFunctionRotation_ABCfunctions();
	ChFunctionRotation_ABCfunctions(const ChFunctionRotation_ABCfunctions& other);
	virtual ~ChFunctionRotation_ABCfunctions();

    /// "Virtual" copy constructor.
    virtual ChFunctionRotation_ABCfunctions* Clone() const override { return new ChFunctionRotation_ABCfunctions(*this); }

	/// Set the angle function A=A(s) expressing the rotation about the first axis of rotation. Angle assumed in radians.
	void SetFunctionAngleA(std::shared_ptr<ChFunction> mx) {
		this->angleA = mx;
	}
	/// Get the angle function A=A(s) expressing the rotation about the fixed axis of rotation. Angle assumed in radians.
	std::shared_ptr<ChFunction> GetFunctionAngleA() const {
		return this->angleA;
	}

	/// Set the angle function B=B(s) expressing the rotation about the second axis of rotation. Angle assumed in radians.
	void SetFunctionAngleB(std::shared_ptr<ChFunction> mx) {
		this->angleB = mx;
	}
	/// Get the angle function B=B(s) expressing the rotation about the second axis of rotation. Angle assumed in radians.
	std::shared_ptr<ChFunction> GetFunctionAngleB() const {
		return this->angleB;
	}

	/// Set the angle function C=C(s) expressing the rotation about the second axis of rotation. Angle assumed in radians.
	void SetFunctionAngleC(std::shared_ptr<ChFunction> mx) {
		this->angleC = mx;
	}
	/// Get the angle function C=C(s) expressing the rotation about the second axis of rotation. Angle assumed in radians.
	std::shared_ptr<ChFunction> GetFunctionAngleC() const {
		return this->angleC;
	}

	/// Set the angle set: Cardano, Eulero, etc. The angle set define the order of the xyz axes of the intrinsic rotation 
	/// with the three A(s) B(s) C(s) rotations, for example Eulero angle set means that there is a rotation about Z, X', Z'',
	/// Cardano angle set means Z,X',Y', etc. 
	void SetAngleset(const AngleSet mset) {
		this->angleset = mset;
	}
	/// Get the angle set: Cardano, Eulero, etc. The angle set define the order of the xyz axes of the intrinsic rotation.
	AngleSet GetAngleset() const {
		return this->angleset;
	}


   
	/// Return the rotation as a quaternion, function of s, as q=f(s).
	virtual ChQuaternion<> Get_q(double s) const override;


    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

private:
	std::shared_ptr<ChFunction> angleA;
	std::shared_ptr<ChFunction> angleB;
	std::shared_ptr<ChFunction> angleC;
	AngleSet angleset;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotation_ABCfunctions, 0)

}  // end namespace chrono

#endif
