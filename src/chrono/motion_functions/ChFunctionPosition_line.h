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

#ifndef CHFUNCTIONPOSITION_LINE_H
#define CHFUNCTIONPOSITION_LINE_H

#include "chrono/geometry/ChLine.h"
#include "chrono/motion_functions/ChFunctionPosition.h"
#include "chrono/motion_functions/ChFunction_Base.h"


namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// A motion function p=f(s) where p(t) is defined with a
/// ChLine geometry object, ex. ChLineArc or ChLineBspline etc.

class ChApi ChFunctionPosition_line : public ChFunctionPosition {

  public:
	ChFunctionPosition_line();
	ChFunctionPosition_line(const ChFunctionPosition_line& other);
	virtual ~ChFunctionPosition_line();

    /// "Virtual" copy constructor.
    virtual ChFunctionPosition_line* Clone() const override { return new ChFunctionPosition_line(*this); }


	/// Get the trajectory line
    std::shared_ptr<geometry::ChLine> GetLine() const { return trajectory_line; }

    /// Sets the trajectory line (take ownership - does not copy line)
	void SetLine(std::shared_ptr<geometry::ChLine> mline) { trajectory_line = mline; }


	/// Gets the address of the function u=u(s) telling
    /// how the curvilinear parameter u of the spline changes in s (time).
    std::shared_ptr<ChFunction> GetSpaceFunction() const { return space_fx; }

    /// Sets the function u=u(s) telling how the curvilinear parameter
    /// of the spline changes in s (time).
	/// Otherwise, by default, is a linear ramp, so evaluates the spline from begin at s=0 to end at s=1 
	void SetSpaceFunction(std::shared_ptr<ChFunction> m_funct) { space_fx = m_funct; }


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
	std::shared_ptr<geometry::ChLine> trajectory_line;

	std::shared_ptr<ChFunction> space_fx;  
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionPosition_line, 0)

}  // end namespace chrono

#endif
