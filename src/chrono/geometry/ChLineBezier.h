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
// Authors: Radu Serban
// =============================================================================
//
// Geometric object representing a piecewise cubic Bezier curve in 3D.
//
// =============================================================================

#ifndef CHC_LINE_BEZIER_H
#define CHC_LINE_BEZIER_H

#include <cmath>

#include "chrono/core/ChBezierCurve.h"
#include "chrono/geometry/ChLine.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Geometric object representing a piecewise cubic Bezier curve in 3D.
class ChApi ChLineBezier : public ChLine {
  public:
    ChLineBezier() {}
    ChLineBezier(std::shared_ptr<ChBezierCurve> path);
    ChLineBezier(const std::string& filename);
    ChLineBezier(const ChLineBezier& source);
    ~ChLineBezier() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineBezier* Clone() const override { return new ChLineBezier(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::LINE_BEZIER; }

    virtual void SetClosed(bool mc) override {}
    virtual void SetComplexity(int mc) override {}

    /// Compute bounding in the frame of the Bezier curve knots.
    virtual ChAABB GetBoundingBox() const override;

    /// Return a point on the line, given parametric coordinate U (in [0,1]).
    virtual ChVector3d Evaluate(double U) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChBezierCurve> m_path;  ///< handle to a Bezier curve
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLineBezier, 0)

}  // end namespace chrono

#endif
