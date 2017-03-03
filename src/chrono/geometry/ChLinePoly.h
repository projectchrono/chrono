// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_LINEPOLY_H
#define CHC_LINEPOLY_H

#include <cmath>

#include "chrono/geometry/ChLine.h"

namespace chrono {
namespace geometry {

/// Geometric object representing a polygonal line in 3D space, controlled by control points.

class ChApi ChLinePoly : public ChLine {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinePoly)

  private:
    std::vector<ChVector<> > points;  ///< control points
    int degree;                       ///< polynomial degree

  public:
    ChLinePoly(int mnumpoints = 1);
    ChLinePoly(const ChLinePoly& source);
    ~ChLinePoly() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinePoly* Clone() const override { return new ChLinePoly(*this); }

    virtual GeometryType GetClassType() const override { return LINE_POLY; }

    virtual int Get_complexity() const override { return (int)points.size(); }
    virtual void Set_complexity(int mc) override {};

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(ChVector<>& pos,
                          const double parU,
                          const double parV = 0.,
                          const double parW = 0.) const override;

    /// Returns curve length. sampling does not matter
    virtual double Length(int sampling) const override;

    /// Draw into the current graph viewport of a ChFile_ps file
    virtual bool DrawPostscript(ChFile_ps* mfle, int markpoints, int bezier_interpolate) override;

    /// Gets the number of control points
    size_t Get_numpoints() const;

    /// Get the degree of the curve (1= linear,
    /// 2= quadric, 3= cubic, etc.)
    int Get_degree() const;

    /// Get the n-th control point
    ChVector<> Get_point(size_t mnum) const;

    /// Set the n-th control point
    bool Set_point(int mnum, ChVector<> mpoint);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChLinePoly>();
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(points);
        marchive << CHNVP(degree);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChLinePoly>();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(points);
        marchive >> CHNVP(degree);
    }
};

}  // end namespace geometry

CH_CLASS_VERSION(geometry::ChLinePoly,0)

}  // end namespace chrono

#endif