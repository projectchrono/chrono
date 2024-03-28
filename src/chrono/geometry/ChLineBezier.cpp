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

#include "chrono/utils/ChUtils.h"
#include "chrono/geometry/ChLineBezier.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineBezier)

ChLineBezier::ChLineBezier(std::shared_ptr<ChBezierCurve> path) : m_path(path) {
    complexityU = static_cast<int>(m_path->GetNumPoints());
}

ChLineBezier::ChLineBezier(const std::string& filename) {
    m_path = ChBezierCurve::Read(filename);
    complexityU = static_cast<int>(m_path->GetNumPoints());
}

ChLineBezier::ChLineBezier(const ChLineBezier& source) : ChLine(source) {
    m_path = source.m_path;
    complexityU = source.complexityU;
}

ChAABB ChLineBezier::GetBoundingBox() const {
    ChAABB aabb;
    for (const auto& p : m_path->GetPoints()) {
        aabb.min = Vmin(aabb.min, p);
        aabb.max = Vmax(aabb.max, p);
    }

    return aabb;
}

ChVector3d ChLineBezier::Evaluate(double parU) const {
    double par = ChClamp(parU, 0.0, 1.0);
    size_t numIntervals = m_path->GetNumPoints() - 1;
    double epar = par * numIntervals;
    size_t i = static_cast<size_t>(std::floor(par * numIntervals));
    ChClampValue(i, size_t(0), numIntervals - 1);
    double t = epar - (double)i;

    return m_path->Eval(i, t);
}

void ChLineBezier::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineBezier>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_path);
}

void ChLineBezier::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineBezier>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_path);
}

}  // end of namespace chrono
