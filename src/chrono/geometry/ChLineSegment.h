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

#ifndef CHC_LINESEGMENT_H
#define CHC_LINESEGMENT_H

#include <cmath>

#include "chrono/core/ChFrame.h"
#include "chrono/geometry/ChLine.h"

namespace chrono {

/// @addtogroup chrono_geometry
/// @{

/// Geometric object representing a segment in 3D space with two end points.
class ChApi ChLineSegment : public ChLine {
  public:
    ChLineSegment(const ChVector3d A = VNULL, const ChVector3d B = VNULL) : pA(A), pB(B) {}
    ChLineSegment(const ChLineSegment& source);
    ~ChLineSegment() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLineSegment* Clone() const override { return new ChLineSegment(*this); }

    /// Get the class type as an enum.
    virtual Type GetType() const override { return Type::LINE_SEGMENT; }

    virtual int GetComplexity() const override { return 2; }

    /// Return a point on the line, given parametric coordinate U (in [0,1]).
    virtual ChVector3d Evaluate(double U) const override;

    /// Return curve length.
    virtual double Length(int sampling) const override { return (pA - pB).Length(); }

    /// Get the segment length.
    double GetLength() const { return Length(0); }

    /// Return the segment frame.
    /// This is a frame centered at the midpoint and with Z axis along the segment.
    ChFrame<> GetFrame() const;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

    ChVector3d pA;  ///< first segment endpoint
    ChVector3d pB;  ///< second segment endpoint
};

/// @} chrono_geometry

CH_CLASS_VERSION(ChLineSegment, 0)

}  // end namespace chrono

#endif
