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
    const std::vector<ChVector3d> mpoints = {ChVector3d(-1, 0, 0), ChVector3d(1, 0, 0)};
    closed = false;
    Setup(1, mpoints);
}

ChLineBSpline::ChLineBSpline(
    int morder,                              // order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChVector3d>& mpoints,  // control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots                // knots, size k. Required k=n+p+1.
) {
    closed = false;
    Setup(morder, mpoints, mknots);
}

ChLineBSpline::ChLineBSpline(const ChLineBSpline& source) : ChLine(source) {
    points = source.points;
    p = source.p;
    knots = source.knots;
    closed = source.closed;
}

ChVector3d ChLineBSpline::Evaluate(double parU) const {
    double mU;
    if (closed)
        mU = fmod(parU, 1.0);
    else
        mU = parU;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBSpline::FindSpan(p, u, knots);

    ChVectorDynamic<> N(p + 1);
    ChBasisToolsBSpline::BasisEvaluate(p, spanU, u, knots, N);

    ChVector3d pos = VNULL;
    int uind = spanU - p;
    for (int i = 0; i <= p; i++) {
        pos += points[uind + i] * N(i);
    }

    return pos;
}

ChVector3d ChLineBSpline::GetTangent(double parU) const {
    double mU;
    if (closed)
        mU = fmod(parU, 1.0);
    else
        mU = parU;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBSpline::FindSpan(p, u, knots);

    ChMatrixDynamic<> NdN(2, p + 1);  // basis on 1st row and their 1st derivatives on 2nd row
    ChBasisToolsBSpline::BasisEvaluateDeriv(p, spanU, u, knots, NdN);

    ChVector3d dir = VNULL;
    int uind = spanU - p;
    for (int i = 0; i <= p; i++) {
        dir += points[uind + i] * NdN(1, i);
    }

    return dir;
}

void ChLineBSpline::Setup(
    int morder,                              // order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChVector3d>& mpoints,  // control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots                // knots, size k. Required k=n+p+1
) {
    if (morder < 1)
        throw std::invalid_argument("ChLineBSpline::Setup requires order >= 1.");

    if (mpoints.size() < morder + 1)
        throw std::invalid_argument("ChLineBSpline::Setup requires at least order+1 control points.");

    if (mknots && (size_t)mknots->size() != (mpoints.size() + morder + 1))
        throw std::invalid_argument("ChLineBSpline::Setup: knots must have size=n_points+order+1");

    p = morder;
    points = mpoints;
    int n = (int)points.size();

    if (mknots)
        knots = *mknots;
    else {
        knots.setZero(n + p + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(knots, p);
    }
}

void ChLineBSpline::SetClosed(bool mc) {
    if (closed == mc)
        return;

    // switch open->closed
    if (mc == true) {
        // add p control points to be wrapped: resize knots and control points
        auto n = points.size();
        n += p;
        points.resize(n);
        knots.setZero(n + p + 1);

        // recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniform(knots, p);

        // wrap last control points
        for (int i = 0; i < p; ++i)
            points[n - p + i] = points[i];
    }

    // switch closed->open
    if (mc == false) {
        // remove p control points that was wrapped: resize knots and control points
        auto n = points.size();
        n -= p;
        points.resize(n);
        knots.setZero(n + p + 1);

        // recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(knots, p);
    }

    closed = mc;
}

void ChLineBSpline::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineBSpline>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(points);
    archive_out << CHNVP(knots);
    archive_out << CHNVP(p);
    archive_out << CHNVP(closed);
}

void ChLineBSpline::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChLineBSpline>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(points);
    archive_in >> CHNVP(knots);
    archive_in >> CHNVP(p);
    archive_in >> CHNVP(closed);
}

}  // end namespace chrono
