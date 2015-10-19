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

#include <math.h>

#include "chrono/geometry/ChCLine.h"
#include "chrono/core/ChBezierCurve.h"

namespace chrono {
namespace geometry {

#define CH_GEOCLASS_LINEBEZIER 21

/// Geometric object representing a piecewise cubic Bezier curve in 3D.
class ChApi ChLineBezier : public ChLine {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChLineBezier, ChLine);

  public:
    ChLineBezier() : m_own_data(false), m_path(NULL) {}
    ChLineBezier(ChBezierCurve* path);
    ChLineBezier(const std::string& filename);
    ChLineBezier(const ChLineBezier& source);
    ChLineBezier(const ChLineBezier* source);

    ~ChLineBezier();

    virtual int GetClassType() { return CH_GEOCLASS_LINEBEZIER; }

    virtual void Set_closed(bool mc) {}
    virtual void Set_complexity(int mc) {}

    /// Curve evaluation (only parU is used, in 0..1 range)
    virtual void Evaluate(Vector& pos, const double parU, const double parV = 0., const double parW = 0.);

    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite(1);
        // serialize parent class
        ChLine::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(m_own_data);
        marchive << CHNVP(m_path);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead();
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
}  // end of namespace chrono

#endif
