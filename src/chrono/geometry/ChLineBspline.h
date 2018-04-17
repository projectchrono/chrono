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

#ifndef CHC_LINEBSPLINE_H
#define CHC_LINEBSPLINE_H

#include <cmath>
#include <vector>

#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChBasisToolsBspline.h"

namespace chrono {
namespace geometry {

/// Geometric object representing a Bspline spline.

class ChApi ChLineBspline : public ChLine {

  public:
    std::vector< ChVector<> > points;
    ChVectorDynamic<>         knots;
    int                       p;

  public:
        /// Constructor. By default, a segment (order = 1, two points on X axis, at -1, +1)
    ChLineBspline();

        /// Constructor from a given array of control points. Input data is copied.
        /// If the knots are not provided, a uniformly spaced knot vector is made.
    ChLineBspline(int morder,                      ///< order p: 1= linear, 2=quadratic, etc.
                std::vector< ChVector<> >& mpoints,///< control points, size n. Required: at least n >= p+1
                ChVectorDynamic<>* mknots = 0      ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform. 
                );

    ChLineBspline(const ChLineBspline& source);
    ~ChLineBspline() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineBspline* Clone() const override { return new ChLineBspline(*this); }

    virtual int Get_complexity() const override { return (int)points.size(); }

        /// Evaluates a point on the line, given parametric coordinate U.
        /// Parameter U always work in 0..1 range, even if knots are not in 0..1 range.
        /// So if you want to use u' in knot range, use ComputeUfromKnotU().
        /// Computed value goes into the 'pos' reference.
        /// It must be implemented by inherited classes.
    virtual void Evaluate(ChVector<>& pos,
                          const double parU) const override;

        /// Evaluates a tangent versor, given parametric coordinate.
        /// Parameter U always work in 0..1 range.
        /// Computed value goes into the 'pos' reference.
    virtual void Derive(ChVector<>& dir, const double parU) const override;


    // Bspline specific functions
    
        /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
        /// but knot range is not necessarily in 0..1. So you can convert u->U, 
        /// where u is in knot range, calling this:
    double ComputeUfromKnotU(const double u) const {
        return (u - this->knots(0)) / (knots(knots.GetRows()-1) - knots(0));
    }
        /// When using Evaluate() etc. you need U parameter to be in 0..1 range,
        /// but knot range is not necessarily in 0..1. So you can convert U->u, 
        /// where u is in knot range, calling this:
    double ComputeKnotUfromU(const double U) const {
        return U*(knots(knots.GetRows()-1) - knots(0)) + this->knots(0);
    }

        /// Access the points
    std::vector< ChVector<> >& Points() { return points; } 

        /// Access the knots
    ChVectorDynamic<>& Knots() { return knots; } 

        /// Get the order of spline
    int GetOrder() { return p; }

        /// Initial easy setup from a given array of control points. Input data is copied.
        /// If the knots are not provided, a uniformly spaced knot vector is made.
        /// If the weights are not provided, a constant weight vector is made.
    virtual void SetupData( int morder,                 ///< order p: 1= linear, 2=quadratic, etc.
                    std::vector< ChVector<> >& mpoints, ///< control points, size n. Required: at least n >= p+1
                    ChVectorDynamic<>* mknots = 0       ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform. 
                    );



    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChLineBspline>();
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(points);
        marchive << CHNVP(knots);
        marchive << CHNVP(p);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChLineBspline>();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(points);
        marchive >> CHNVP(knots);
        marchive >> CHNVP(p);
    }
};



}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChLineBspline,0)

}  // end namespace chrono

#endif
