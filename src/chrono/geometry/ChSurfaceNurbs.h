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

#ifndef CHC_SURFACENURBS_H
#define CHC_SURFACENURBS_H

#include <cmath>
#include <vector>

#include "chrono/geometry/ChSurface.h"
#include "chrono/geometry/ChBasisToolsNurbs.h"

namespace chrono {
namespace geometry {

/// Geometric object representing a NURBS surface.
class ChApi ChSurfaceNurbs : public ChSurface {
  public:
    ChMatrixDynamic<ChVector<> > points;
    ChMatrixDynamic<> weights;
    ChVectorDynamic<> knots_u;
    ChVectorDynamic<> knots_v;
    int p_u;
    int p_v;

  public:
    /// Constructor. By default, a segment (order = 1, two points on X axis, at -1, +1)
    ChSurfaceNurbs();

    /// Constructor from a given array of control points. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    /// If the weights are not provided, a constant weight vector is made.
    ChSurfaceNurbs(int morder_u,  ///< order pu: 1= linear, 2=quadratic, etc.
                   int morder_v,  ///< order pv: 1= linear, 2=quadratic, etc.
                   ChMatrixDynamic<ChVector<>>& mpoints,  ///< control points, size nuxnv. Required: nu at least pu+1, same for v
                   ChVectorDynamic<>* mknots_u = 0,  ///< knots, size ku. Required ku=nu+pu+1. If not provided, initialized to uniform.
                   ChVectorDynamic<>* mknots_v = 0,  ///< knots, size kv. Required ku=nu+pu+1. If not provided, initialized to uniform.
                   ChMatrixDynamic<>* weights = 0  ///< weights, size nuxnv. If not provided, all weights as 1.
    );

    ChSurfaceNurbs(const ChSurfaceNurbs& source);
    ~ChSurfaceNurbs() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSurfaceNurbs* Clone() const override { return new ChSurfaceNurbs(*this); }

    // virtual int Get_complexity() const override { return points.GetRows(); }

    /// Evaluates a point on the line, given parametric coordinate U.
    /// Parameter U always work in 0..1 range, even if knots are not in 0..1 range.
    /// So if you want to use u' in knot range, use ComputeUfromKnotU().
    /// Computed value goes into the 'pos' reference.
    /// It must be implemented by inherited classes.
    virtual void Evaluate(ChVector<>& pos, const double parU, const double parV) const override;

    /// Evaluates normal
    // virtual void Normal(ChVector<>& dir, const double parU) const override;

    // NURBS specific functions

    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert u->U,
    /// where u is in knot range, calling this:
    double ComputeUfromKnotU(const double u) const {
        return (u - knots_u(p_u)) / (knots_u(knots_u.size() - 1-p_u) - knots_u(p_u));
    }
    /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert U->u,
    /// where u is in knot range, calling this:
    double ComputeKnotUfromU(const double U) const {
        return U * (knots_u(knots_u.size() - 1-p_u) - knots_u(p_u)) + knots_u(p_u);
    }

    /// When using Evaluate() etc. you need V parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert v->V,
    /// where v is in knot range, calling this:
    double ComputeVfromKnotV(const double v) const {
        return (v - knots_v(p_v)) / (knots_v(knots_v.size() - 1-p_v) - knots_v(p_v));
    }
    /// When using Evaluate() etc. you need V parameter to be in 0..1 range,
    /// but knot range is not necessarily in 0..1. So you can convert V->v,
    /// where v is in knot range, calling this:
    double ComputeKnotVfromV(const double V) const {
        return V * (knots_v(knots_v.size() - 1-p_v) - knots_v(p_v)) + knots_v(p_v);
    }

    /// Access the points
    ChMatrixDynamic<ChVector<> >& Points() { return points; }

    /// Access the weights
    ChMatrixDynamic<>& Weights() { return weights; }

    /// Access the U knots
    ChVectorDynamic<>& Knots_u() { return knots_u; }

    /// Access the U knots
    ChVectorDynamic<>& Knots_v() { return knots_v; }

    /// Get the order in u direction
    int GetOrder_u() { return p_u; }

    /// Get the order in v direction
    int GetOrder_v() { return p_v; }

    /// Initial easy setup from a given array of control points. Input data is copied.
    /// If the knots are not provided, a uniformly spaced knot vector is made.
    /// If the weights are not provided, a constant weight vector is made.
    virtual void SetupData(int morder_u,  ///< order pu: 1= linear, 2=quadratic, etc.
                           int morder_v,  ///< order pv: 1= linear, 2=quadratic, etc.
                           ChMatrixDynamic<ChVector<> >&
                               mpoints,  ///< control points, size nuxnv. Required: at least nu >= pu+1, same for v
                           ChVectorDynamic<>* mknots_u =
                               0,  ///< knots u, size ku. Required ku=nu+pu+1. If not provided, initialized to uniform.
                           ChVectorDynamic<>* mknots_v =
                               0,  ///< knots v, size kv. Required ku=nu+pu+1. If not provided, initialized to uniform.
                           ChMatrixDynamic<>* weights = 0  ///< weights, size nuxnv. If not provided, all weights as 1.
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChSurfaceNurbs, 0)

}  // end namespace chrono

#endif
