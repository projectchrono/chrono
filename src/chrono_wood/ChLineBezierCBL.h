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

#ifndef CHC_LINE_BEZIER_H
#define CHC_LINE_BEZIER_H

#include <cmath>
#include <vector>

#include "chrono_wood/ChWoodApi.h"
#include "chrono_wood/ChBasisToolsBeziers.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChBasisToolsBSpline.h"

using namespace chrono;

namespace chrono {
namespace wood {

/// Geometric object representing a Bezier spline.
class ChWoodApi ChLineBezierCBL : public ChLine {
  public:
    std::vector<ChVector3d > points;
    ChVectorDynamic<> knots;
    int p;
    bool closed;

  public:
    /// Constructor. By default, a segment (order = 1, two points on X axis, at -1, +1)
    ChLineBezierCBL();

    /// Constructor from a given array of control points. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    ChLineBezierCBL(
        int morder,                               ///< order p: 1= linear, 2=quadratic, etc.
        const std::vector<ChVector3d >& mpoints,  ///< control points, size n. Required: at least n >= p+1
        ChVectorDynamic<>* mknots = 0  ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
    );

    ChLineBezierCBL(const ChLineBezierCBL& source);
    ~ChLineBezierCBL() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineBezierCBL* Clone() const override { return new ChLineBezierCBL(*this); }

    virtual int GetComplexity() const override { return (int)points.size(); }

    /// Evaluates a point on the line, given parametric coordinate U.
    /// Parameter U always work in 0..1 range, even if knots are not in 0..1 range.
    /// So if you want to use u' in knot range, use ComputeUfromKnotU().
    /// Computed value goes into the 'pos' reference.
    /// It must be implemented by inherited classes.
    virtual ChVector3d Evaluate(double U) const override;

    /// Evaluates a tangent versor, given parametric coordinate.
    /// Parameter U always work in 0..1 range.
    /// Computed value goes into the 'pos' reference.
    virtual void Derive(ChVector3d& dir, const double parU);

    // Bezier specific functions

    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert u->U,
    /// where u is in knot range, calling this:
    double ComputeUfromKnotU(const double u) const { return (u - knots(p)) / (knots(knots.size() - 1 - p) - knots(p)); }
    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert U->u,
    /// where u is in knot range, calling this:
    double ComputeKnotUfromU(const double U) const { return U * (knots(knots.size() - 1 - p) - knots(p)) + knots(p); }

    /// Access the points
    std::vector<ChVector3d >& Points() { return points; }

    /// Access the knots
    ChVectorDynamic<>& Knots() { return knots; }

    /// Get the order of spline
    int GetOrder() { return p; }

    /// Initial easy setup from a given array of control points. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    virtual void SetupData(
        int morder,                               ///< order p: 1= linear, 2=quadratic, etc.
        const std::vector<ChVector3d >& mpoints,  ///< control points, size n. Required: at least n >= p+1
        ChVectorDynamic<>* mknots = 0  ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
    );

    /// Set as closed spline: start and end will overlap at 0 and 1 abscyssa as p(0)=p(1),
    /// and the Evaluate() and Derive() functions will operate in periodic way (abscyssa
    /// greater than 1 or smaller than 0 will wrap to 0..1 range).
    /// The closure will change the knot vector (multiple start end knots will be lost) and
    /// will create auxiliary p control points at the end that will be wrapped to the beginning control point.
    void SetClosed(bool mc);

    /// Tell if the spline is closed.
    bool GetClosed() { return closed; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

}  // end namespace wood

CH_CLASS_VERSION(wood::ChLineBezierCBL, 0)

}  // end namespace chrono

#endif
