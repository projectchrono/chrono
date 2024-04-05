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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHC_LINEPOLY_H
#define CHC_LINEPOLY_H

#include <cmath>

#include "chrono/geometry/ChLine.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Geometric object representing a polygonal line in 3D space, controlled by control points.
class ChApi ChLinePoly : public ChLine {
  private:
    std::vector<ChVector3d> points;  ///< control points
    int degree;                      ///< polynomial degree

  public:
    ChLinePoly(int mnumpoints = 1);
    ChLinePoly(const ChLinePoly& source);
    ~ChLinePoly() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinePoly* Clone() const override { return new ChLinePoly(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::LINE_POLY; }

    virtual int GetComplexity() const override { return (int)points.size(); }
    virtual void SetComplexity(int mc) override{};

    /// Return a point on the line, given parametric coordinate U (in [0,1]).
    virtual ChVector3d Evaluate(double U) const override;

    /// Returns curve length. sampling does not matter
    virtual double Length(int sampling) const override;

    /// Gets the number of control points
    size_t GetNumPoints() const;

    /// Get the degree of the curve (1= linear, 2= quadric, 3= cubic, etc.)
    int GetDegree() const;

    /// Get the n-th control point.
    ChVector3d GetPoint(size_t mnum) const;

    /// Set the n-th control point
    bool SetPoint(int mnum, const ChVector3d& mpoint);

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLinePoly, 0)

}  // end namespace chrono

#endif