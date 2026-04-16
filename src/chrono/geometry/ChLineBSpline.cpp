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

#include "chrono/geometry/ChLineBSpline.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineBSpline)

ChLineBSpline::ChLineBSpline() {
    std::vector<ChVector3d> mpoints = {ChVector3d(-1, 0, 0), ChVector3d(1, 0, 0)};
    closed = false;
    Setup(1, mpoints);
}

ChLineBSpline::ChLineBSpline(int morder,                              // order p: 1= linear, 2=quadratic, etc.
                             const std::vector<ChVector3d>& mpoints,  // control points, size n. Required: at least n >= p+1
                             const ChVectorDynamic<>* mknots          // knots, size k. Required k=n+p+1.
) {
    closed = false;
    Setup(morder, mpoints, mknots);
}

ChLineBSpline::ChLineBSpline(const ChLineBSpline& source) : ChLine(source) {
    m_points = source.m_points;
    m_p = source.m_p;
    m_knots = source.m_knots;
    closed = source.closed;
}

ChVector3d ChLineBSpline::Evaluate(double parU) const {
    double mU = 0;
    if (closed)
        mU = fmod(parU, 1.0);
    else
        mU = parU;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBSpline::FindSpan(m_p, u, m_knots);

    ChVectorDynamic<> N(m_p + 1);
    ChBasisToolsBSpline::BasisEvaluate(m_p, spanU, u, m_knots, N);

    ChVector3d pos = VNULL;
    int uind = spanU - m_p;
    for (int i = 0; i <= m_p; i++) {
        pos += m_points[uind + i] * N(i);
    }

    return pos;
}

ChVector3d ChLineBSpline::GetTangent(double parU) const {
    double mU = 0;
    if (closed)
        mU = fmod(parU, 1.0);
    else
        mU = parU;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBSpline::FindSpan(m_p, u, m_knots);

    ChMatrixDynamic<> NdN(2, m_p + 1);  // basis on 1st row and their 1st derivatives on 2nd row
    ChBasisToolsBSpline::BasisEvaluateDeriv(m_p, spanU, u, m_knots, NdN);

    ChVector3d dir = VNULL;
    int uind = spanU - m_p;
    for (int i = 0; i <= m_p; i++) {
        dir += m_points[uind + i] * NdN(1, i);
    }

    return dir;
}

void ChLineBSpline::Setup(int order,                              // order p: 1= linear, 2=quadratic, etc.
                          const std::vector<ChVector3d>& points,  // control points, size n. Required: at least n >= p+1
                          const ChVectorDynamic<>* knots          // knots, size k. Required k=n+p+1
) {
    if (order < 1)
        throw std::invalid_argument("ChLineBSpline::Setup requires order >= 1.");

    if (points.size() < order + 1)
        throw std::invalid_argument("ChLineBSpline::Setup requires at least order+1 control points.");

    if (knots && (size_t)knots->size() != (points.size() + order + 1))
        throw std::invalid_argument("ChLineBSpline::Setup: knots must have size=n_points+order+1");

    m_p = order;
    m_points = points;
    int n = (int)m_points.size();

    if (knots)
        m_knots = *knots;
    else {
        m_knots.setZero(n + m_p + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(m_knots, m_p);
    }
}

void ChLineBSpline::SetClosed(bool mc) {
    if (closed == mc)
        return;

    // switch open->closed
    if (mc == true) {
        // add p control points to be wrapped: resize knots and control points
        auto n = m_points.size();
        n += m_p;
        m_points.resize(n);
        m_knots.setZero(n + m_p + 1);

        // recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniform(m_knots, m_p);

        // wrap last control points
        for (int i = 0; i < m_p; ++i)
            m_points[n - m_p + i] = m_points[i];
    }

    // switch closed->open
    if (mc == false) {
        // remove p control points that was wrapped: resize knots and control points
        auto n = m_points.size();
        n -= m_p;
        m_points.resize(n);
        m_knots.setZero(n + m_p + 1);

        // recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(m_knots, m_p);
    }

    closed = mc;
}

void ChLineBSpline::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineBSpline>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_points);
    archive_out << CHNVP(m_knots);
    archive_out << CHNVP(m_p);
    // archive_out << CHNVP(closed); // already serialized by parent class?
}

void ChLineBSpline::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineBSpline>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_points);
    archive_in >> CHNVP(m_knots);
    archive_in >> CHNVP(m_p);
    // archive_in >> CHNVP(closed); // already serialized by parent class?
}

}  // end namespace chrono
