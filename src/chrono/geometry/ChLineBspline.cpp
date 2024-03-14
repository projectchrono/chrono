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

#include "chrono/geometry/ChLineBspline.h"

namespace chrono {


// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineBspline)

ChLineBspline::ChLineBspline() {
    const std::vector<ChVector3d > mpoints = {ChVector3d(-1, 0, 0), ChVector3d(1, 0, 0)};
	this->closed = false;
    this->Setup(1, mpoints);
}

ChLineBspline::ChLineBspline(
    int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChVector3d >& mpoints,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
) {
	this->closed = false;
    this->Setup(morder, mpoints, mknots);
}

ChLineBspline::ChLineBspline(const ChLineBspline& source) : ChLine(source) {
    this->points = source.points;
    this->p = source.p;
    this->knots = source.knots;
	this->closed = source.closed;
}

ChVector3d ChLineBspline::Evaluate(double parU) const {
	double mU;
	if (this->closed)
		mU = fmod(parU, 1.0);
	else
		mU = parU;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBSpline::FindSpan(this->p, u, this->knots);

    ChVectorDynamic<> N(this->p + 1);
    ChBasisToolsBSpline::BasisEvaluate(this->p, spanU, u, this->knots, N);

    ChVector3d pos = VNULL;
    int uind = spanU - p;
    for (int i = 0; i <= this->p; i++) {
        pos += points[uind + i] * N(i);
    }

    return pos;
}

ChVector3d ChLineBspline::GetTangent(double parU) const {
    double mU;
	if (this->closed)
		mU = fmod(parU, 1.0);
	else
		mU = parU;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBSpline::FindSpan(this->p, u, this->knots);

    ChMatrixDynamic<> NdN(2, p + 1);  // basis on 1st row and their 1st derivatives on 2nd row
    ChBasisToolsBSpline::BasisEvaluateDeriv(this->p, spanU, u, this->knots, NdN);

    ChVector3d dir = VNULL;
    int uind = spanU - p;
    for (int i = 0; i <= this->p; i++) {
        dir += points[uind + i] * NdN(1, i);
    }

    return dir;
}

void ChLineBspline::Setup(
    int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChVector3d >& mpoints,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
) {
    if (morder < 1)
        throw std::invalid_argument("ChLineBspline::Setup requires order >= 1.");

    if (mpoints.size() < morder + 1)
        throw std::invalid_argument("ChLineBspline::Setup requires at least order+1 control points.");

    if (mknots && (size_t)mknots->size() != (mpoints.size() + morder + 1))
        throw std::invalid_argument("ChLineBspline::Setup: knots must have size=n_points+order+1");

    this->p = morder;
    this->points = mpoints;
    int n = (int)points.size();

    if (mknots)
        this->knots = *mknots;
    else {
        this->knots.setZero(n + p + 1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(this->knots, p);
    }
}

void ChLineBspline::SetClosed(bool mc) {
	if (this->closed == mc)
		return;

	// switch open->closed
	if (mc == true) {
		// add p control points to be wrapped: resize knots and control points
		auto n = this->points.size();
		n += p; 
		this->points.resize(n);
		this->knots.setZero(n + p + 1);
		
		// recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniform(this->knots, p);
		
		// wrap last control points
		for (int i = 0; i < p; ++i)
			this->points[n - p + i] = this->points[i];
	}
	
	// switch closed->open
	if (mc == false) {
		// remove p control points that was wrapped: resize knots and control points
		auto n = this->points.size();
		n -= p; 
		this->points.resize(n);
		this->knots.setZero(n + p + 1);

		// recompute knot vector spacing
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(this->knots, p);
	}

	this->closed = mc;
}

void ChLineBspline::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChLineBspline>();
    // serialize parent class
    ChLine::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(points);
    archive_out << CHNVP(knots);
    archive_out << CHNVP(p);
	archive_out << CHNVP(closed);
}

void ChLineBspline::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/ archive_in.VersionRead<ChLineBspline>();
    // deserialize parent class
    ChLine::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(points);
    archive_in >> CHNVP(knots);
    archive_in >> CHNVP(p);
	archive_in >> CHNVP(closed);
}


}  // end namespace chrono
