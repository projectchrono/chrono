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

#ifndef CHFUNCTIONPOSITION_XYZFUNCTIONS_H
#define CHFUNCTIONPOSITION_XYZFUNCTIONS_H

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunctionPosition.h"


namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A motion function p=f(s) where p(t) is defined with three
/// independent ChFunction objects, each for px, py, pz component.

class ChApi ChFunctionPosition_XYZfunctions : public ChFunctionPosition {

  public:
	ChFunctionPosition_XYZfunctions();
	ChFunctionPosition_XYZfunctions(const ChFunctionPosition_XYZfunctions& other);
	virtual ~ChFunctionPosition_XYZfunctions();

    /// "Virtual" copy constructor.
    virtual ChFunctionPosition_XYZfunctions* Clone() const override { return new ChFunctionPosition_XYZfunctions(*this); }

	/// Set the fx(s) function for the X component of the motion, ie. p.x = fx(s)
	void SetFunctionX(std::shared_ptr<ChFunction> mx) {
		this->px = mx;
	}
	/// Get the fx(s) function for the X component of the motion, ie. p.x = fx(s)
	std::shared_ptr<ChFunction> GetFunctionX() {
		return this->px;
	}
	/// Set the fy(s) function for the Y component of the motion, ie. p.y = fy(s)
	void SetFunctionY(std::shared_ptr<ChFunction> my) {
		this->py = my;
	}
	/// Get the fy(s) function for the Y component of the motion, ie. p.y = fy(s)
	std::shared_ptr<ChFunction> GetFunctionY() {
		return this->py;
	}
	/// Set the fz(s) function for the Z component of the motion, ie. p.z = fz(s)
	void SetFunctionZ(std::shared_ptr<ChFunction> mz) {
		this->pz = mz;
	}
	/// Get the fz(s) function for the Z component of the motion, ie. p.z = fz(s)
	std::shared_ptr<ChFunction> GetFunctionZ() {
		return this->pz;
	}

    /// Return the p value of the function, at s, as p=f(s).
	virtual ChVector<> Get_p(double s) const override;

    /// Return the dp/ds derivative of the function, at s.
	virtual ChVector<> Get_p_ds(double s) const override;

    /// Return the ddp/dsds double derivative of the function, at s.
    virtual ChVector<> Get_p_dsds(double s) const override;

    /// Return an estimate of the domain of the function argument.
	virtual void Estimate_s_domain(double& smin, double& smax) const override;

   
    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

private:
	std::shared_ptr<ChFunction> px;
	std::shared_ptr<ChFunction> py;
	std::shared_ptr<ChFunction> pz;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPosition_XYZfunctions, 0)

}  // end namespace chrono

#endif
