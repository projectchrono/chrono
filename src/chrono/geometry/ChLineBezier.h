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
namespace geometry {

/// Geometric object representing a piecewise cubic Bezier curve in 3D.
class ChApi ChLineBezier : public ChLine {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLineBezier)

  public:
    ChLineBezier() : m_own_data(false), m_path(NULL) {}
    ChLineBezier(ChBezierCurve* path);
    ChLineBezier(const std::string& filename);
    ChLineBezier(const ChLineBezier& source);
    ChLineBezier(const ChLineBezier* source);
    ~ChLineBezier();

    virtual GeometryType GetClassType() const override { return LINE_BEZIER; }

    virtual void Set_closed(bool mc) override {}
    virtual void Set_complexity(int mc) override {}

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(ChVector<>& pos,
                          const double parU,
                          const double parV = 0.,
                          const double parW = 0.) const override;

    virtual void ArchiveOUT(ChArchiveOut& marchive) override {
        // version number
        marchive.VersionWrite<ChLineBezier>();
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(m_own_data);
        marchive << CHNVP(m_path);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        int version = marchive.VersionRead<ChLineBezier>();
        // deserialize parent class
        ChLine::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(m_own_data);
        marchive >> CHNVP(m_path);
    }

  private:
    bool m_own_data;        ///< flag indicating if this object owns the data
    ChBezierCurve* m_path;  ///< pointer to a Bezier curve
};

}  // end of namespace geometry

CH_CLASS_VERSION(geometry::ChLineBezier,0)

}  // end of namespace chrono

#endif
