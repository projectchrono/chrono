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

#include <cmath>

#include "chrono/core/ChMathematics.h"
#include "chrono/geometry/ChLineBezier.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineBezier)

ChLineBezier::ChLineBezier(std::shared_ptr<ChBezierCurve> path) : m_path(path) {
    complexityU = static_cast<int>(m_path->getNumPoints());
}

ChLineBezier::ChLineBezier(const std::string& filename) {
    m_path = ChBezierCurve::read(filename);
    complexityU = static_cast<int>(m_path->getNumPoints());
}

ChLineBezier::ChLineBezier(const ChLineBezier& source) : ChLine(source) {
    m_path = source.m_path;
    complexityU = source.complexityU;
}

void ChLineBezier::Evaluate(ChVector<>& pos, const double parU) const {
    double par = ChClamp(parU, 0.0, 1.0);
    size_t numIntervals = m_path->getNumPoints() - 1;
    double epar = par * numIntervals;
    size_t i = static_cast<size_t>(std::floor(par * numIntervals));
    ChClampValue(i, size_t(0), numIntervals - 1);
    double t = epar - (double)i;

    pos = m_path->eval(i, t);
}

void ChLineBezier::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLineBezier>();
    // serialize parent class
    ChLine::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(m_path);
}

void ChLineBezier::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLineBezier>();
    // deserialize parent class
    ChLine::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_path);
}

}  // end of namespace geometry
}  // end of namespace chrono
