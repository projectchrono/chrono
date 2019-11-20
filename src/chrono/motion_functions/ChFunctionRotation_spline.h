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

#ifndef CHFUNCTIONROTATION_SPLINE_H
#define CHFUNCTIONROTATION_SPLINE_H

#include "chrono/motion_functions/ChFunctionRotation.h"


namespace chrono {

/// @addtogroup chrono_functions
/// @{


/// A rotation function q=f(s) that interpolates n rotations using a "quaternion spline" of
/// generic order. 
/// For order 1 (linear) this boils down to a classical SLERP interpolation.
/// For higher orders, this operates as in the paper  "A C^2-continuous B-spline quaternion curve interpolating
/// a given sequence of solid orientations", Myoung-Jun Kim, Myung-Soo Kim. 1995. DOI:10.1109/CA.1995.393545.
/// Note, except for order 1 (linear) the rotation does not pass through control points, 
/// just like for positions in Bspline, except for first and last point. Exact interpolation requires the 'inversion' of the control points (to do).

class ChApi ChFunctionRotation_spline : public ChFunctionRotation {

  public:
    /// Constructor. By default constructs a linear SLERP between two identical null rotations
    ChFunctionRotation_spline();

    /// Constructor from a given array of control points; each control point is a rotation to interpolate. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    ChFunctionRotation_spline(
        int morder,                             ///< order p: 1= linear, 2=quadratic, etc.
        std::vector<ChQuaternion<> >& mrotations,  ///< control points, size n. Each is a rotation. Required: at least n >= p+1
        ChVectorDynamic<>* mknots = 0           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
    );

	ChFunctionRotation_spline(const ChFunctionRotation_spline& other);
	virtual ~ChFunctionRotation_spline();

    /// "Virtual" copy constructor.
    virtual ChFunctionRotation_spline* Clone() const override { return new ChFunctionRotation_spline(*this); }


	/// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert u->U,
    /// where u is in knot range, calling this:
    double ComputeUfromKnotU(const double u) const {
        return (u - knots(0)) / (knots(knots.size() - 1) - knots(0));
    }
    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert U->u,
    /// where u is in knot range, calling this:
    double ComputeKnotUfromU(const double U) const {
        return U * (knots(knots.size() - 1) - knots(0)) + knots(0);
    }

    /// Access the rotations, ie. quaternion spline control points
    std::vector<ChQuaternion<> >& Rotations() { return rotations; }

    /// Access the knots
    ChVectorDynamic<>& Knots() { return knots; }

    /// Get the order of spline
    int GetOrder() { return p; }

    /// Initial easy setup from a given array of rotations (quaternion spline control points). Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    /// If the weights are not provided, a constant weight vector is made.
    virtual void SetupData(
        int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
        std::vector<ChQuaternion<> >& mrotations,  ///< rotations, size n. Required: at least n >= p+1
        ChVectorDynamic<>* mknots = 0  ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
    );
	

	/// Return the q value of the function, at s, as q=f(s).
	/// Parameter s always work in 0..1 range, even if knots are not in 0..1 range.
    /// So if you want to use s in knot range, use ComputeUfromKnotU().
    virtual ChQuaternion<> Get_q(double s) const;

    /// Return the derivative of the rotation function, at s, expressed as angular velocity w in local frame.
	//virtual ChVector<> Get_w_loc(double s) const;

    /// Return the derivative of the rotation function, at s, expressed as angular acceleration in local frame.
	//virtual ChVector<> Get_a_loc(double s) const;


    /// Method to allow serialization of transient data to archives
    virtual void ArchiveOUT(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive);

	

private:
	std::vector<ChQuaternion<> > rotations;
    ChVectorDynamic<> knots;
    int p;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunctionRotation_spline, 0)

}  // end namespace chrono

#endif
