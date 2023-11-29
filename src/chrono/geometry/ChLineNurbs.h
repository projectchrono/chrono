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

#ifndef CHC_LINENURBS_H
#define CHC_LINENURBS_H

#include <cmath>
#include <vector>

#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChBasisToolsNurbs.h"

namespace chrono {
namespace geometry {

/// Geometric object representing a NURBS spline.
class ChApi ChLineNurbs : public ChLine {
  public:
    std::vector<ChVector<> > points;
    ChVectorDynamic<> weights;
    ChVectorDynamic<> knots;
    int p;

  public:
    /// Constructor. By default, a segment (order = 1, two points on X axis, at -1, +1)
    ChLineNurbs();

    /// Constructor from a given array of control points. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    /// If the weights are not provided, a constant weight vector is made.
    ChLineNurbs(
        int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
        std::vector<ChVector<> >& mpoints,  ///< control points, size n. Required: at least n >= p+1
        ChVectorDynamic<>* mknots = 0,  ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
        ChVectorDynamic<>* weights = 0  ///< weights, size w. Required w=n. If not provided, all weights as 1.
    );

    ChLineNurbs(const ChLineNurbs& source);
    ~ChLineNurbs() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineNurbs* Clone() const override { return new ChLineNurbs(*this); }

    virtual int Get_complexity() const override { return (int)points.size(); }

    /// Return a point on the line, given parametric coordinate U (in [0,1]).
    virtual ChVector<> Evaluate(double U) const override;

    /// Return the tangent unit vector at the parametric coordinate U (in [0,1]).
    virtual ChVector<> GetTangent(double parU) const override;

    // NURBS specific functions

    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert u->U,
    /// where u is in knot range, calling this:
    double ComputeUfromKnotU(double u) const { return (u - knots(p)) / (knots(knots.size() - 1 - p) - knots(p)); }
    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert U->u,
    /// where u is in knot range, calling this:
    double ComputeKnotUfromU(double U) const { return U * (knots(knots.size() - 1 - p) - knots(p)) + knots(p); }

    /// Access the points
    std::vector<ChVector<> >& Points() { return points; }

    /// Access the weights
    ChVectorDynamic<>& Weights() { return weights; }

    /// Access the knots
    ChVectorDynamic<>& Knots() { return knots; }

    /// Get the order of spline
    int GetOrder() { return p; }

    /// Initial easy setup from a given array of control points. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    /// If the weights are not provided, a constant weight vector is made.
    virtual void SetupData(
        int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
        std::vector<ChVector<> >& mpoints,  ///< control points, size n. Required: at least n >= p+1
        ChVectorDynamic<>* mknots = 0,  ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
        ChVectorDynamic<>* weights = 0  ///< weights, size w. Required w=n. If not provided, all weights as 1.
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChLineNurbs, 0)

}  // end namespace chrono

#endif
