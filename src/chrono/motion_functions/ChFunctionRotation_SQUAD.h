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

#ifndef CHFUNCTIONROTATION_SQUAD_H
#define CHFUNCTIONROTATION_SQUAD_H

#include "chrono/motion_functions/ChFunctionRotation.h"
#include "chrono/motion_functions/ChFunction_Base.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{


/// A rotation function q=f(s) that interpolates n rotations using a SQUAD 
/// spherical quadrangle interpolation between quaternions. 
/// Differently from the ChFunctionRotation_spline of order 3, 
/// this cubic interpolation really passes through the control points.
/// In the original single-span SQUAD algorithm, 4 quaternions are used: the interpolation
/// passes exactly in 1st and 4th, whereas 2nd and 3rd are 'magnetic' as ctrl points 
/// in splines; in our implementation the 2nd and 3rd are computed automatically
/// given the sequence of the many SQUAD spans, per each span, enforcing continuity inter span.

class ChApi ChFunctionRotation_SQUAD : public ChFunctionRotation {

  public:
    /// Constructor. By default constructs a linear SLERP between two identical null rotations
    ChFunctionRotation_SQUAD();

    /// Constructor from a given array of control points; each control point is a rotation to interpolate. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    ChFunctionRotation_SQUAD(
        const std::vector<ChQuaternion<> >& mrotations,  ///< rotations, to interpolate. Required: at least n = 2. 
        ChVectorDynamic<>* mknots = 0					 ///< knots, as many as control points. If not provided, initialized to uniform.
    );

	ChFunctionRotation_SQUAD(const ChFunctionRotation_SQUAD& other);
	virtual ~ChFunctionRotation_SQUAD();

    /// "Virtual" copy constructor.
    virtual ChFunctionRotation_SQUAD* Clone() const override { return new ChFunctionRotation_SQUAD(*this); }


	/// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert u->U,
    /// where u is in knot range, calling this:
    double ComputeUfromKnotU(const double u) const {
        return (u - knots(0)) / (knots(knots.size()-1) - knots(0));
    }
    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert U->u,
    /// where u is in knot range, calling this:
    double ComputeKnotUfromU(const double U) const {
        return U * (knots(knots.size()-1) - knots(0)) + knots(0);
    }

    /// Access the rotations, ie. quaternion SQUAD control points
    std::vector<ChQuaternion<> >& Rotations() { return rotations; }

    /// Access the knots
    ChVectorDynamic<>& Knots() { return knots; }

    /// Get the order of spline
    int GetOrder() { return p; }

    /// Initial easy setup from a given array of rotations (quaternion control points). Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    virtual void SetupData(
        const std::vector<ChQuaternion<> >& mrotations,  ///< rotations, to interpolate. Required: at least n = 2. 
        ChVectorDynamic<>* mknots = 0  ///< knots, as many as control points. If not provided, initialized to uniform.
    );
	


	/// Gets the address of the function u=u(s) telling
    /// how the curvilinear parameter u of the spline changes in s (time).
    std::shared_ptr<ChFunction> GetSpaceFunction() const { return space_fx; }

    /// Sets the function u=u(s) telling how the curvilinear parameter
    /// of the spline changes in s (time).
	/// Otherwise, by default, is a linear ramp, so evaluates the spline from begin at s=0 to end at s=1 
	void SetSpaceFunction(std::shared_ptr<ChFunction> m_funct) { space_fx = m_funct; }

	/// Set as closed periodic spline: start and end rotations will match at 0 and 1 abscyssa as q(0)=q(1),
	/// and the Evaluate() and Derive() functions will operate in periodic way (abscyssa 
	/// greater than 1 or smaller than 0 will wrap to 0..1 range).
	/// The closure will change the knot vector (multiple start end knots will be lost) and
	/// will create auxiliary p control points at the end that will be wrapped to the beginning control points.
	void SetClosed(bool mc);

	/// Tell if the rotation spline is closed periodic
	bool GetClosed() {return closed;}


	/// Return the q value of the function, at s, as q=f(s).
	/// Parameter s always work in 0..1 range, even if knots are not in 0..1 range.
    /// So if you want to use s in knot range, use ComputeUfromKnotU().
    virtual ChQuaternion<> Get_q(double s) const override;

    /// Return the derivative of the rotation function, at s, expressed as angular velocity w in local frame.
	//virtual ChVector<> Get_w_loc(double s) const override;

    /// Return the derivative of the rotation function, at s, expressed as angular acceleration in local frame.
	//virtual ChVector<> Get_a_loc(double s) const override;


    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

	

private:
	std::vector<ChQuaternion<> > rotations;
    ChVectorDynamic<> knots;
    int p;
	bool closed;

	std::shared_ptr<ChFunction> space_fx;  
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotation_SQUAD, 0)

}  // end namespace chrono

#endif
