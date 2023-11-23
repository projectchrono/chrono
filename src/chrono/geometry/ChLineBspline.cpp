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
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineBspline)

ChLineBspline::ChLineBspline() {
    const std::vector<ChVector<> > mpoints = {ChVector<>(-1, 0, 0), ChVector<>(1, 0, 0)};
	this->closed = false;
    this->SetupData(1, mpoints);
}

ChLineBspline::ChLineBspline(
    int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChVector<> >& mpoints,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
) {
	this->closed = false;
    this->SetupData(morder, mpoints, mknots);
}

ChLineBspline::ChLineBspline(const ChLineBspline& source) : ChLine(source) {
    this->points = source.points;
    this->p = source.p;
    this->knots = source.knots;
	this->closed = source.closed;
}

ChVector<> ChLineBspline::Evaluate(double parU) const {
	double mU;
	if (this->closed)
		mU = fmod(parU, 1.0);
	else
		mU = parU;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBspline::FindSpan(this->p, u, this->knots);

    ChVectorDynamic<> N(this->p + 1);
    ChBasisToolsBspline::BasisEvaluate(this->p, spanU, u, this->knots, N);

    ChVector<> pos = VNULL;
    int uind = spanU - p;
    for (int i = 0; i <= this->p; i++) {
        pos += points[uind + i] * N(i);
    }

    return pos;
}

ChVector<> ChLineBspline::GetTangent(double parU) const {
    double mU;
	if (this->closed)
		mU = fmod(parU, 1.0);
	else
		mU = parU;

    double u = ComputeKnotUfromU(mU);

    int spanU = ChBasisToolsBspline::FindSpan(this->p, u, this->knots);

    ChMatrixDynamic<> NdN(2, p + 1);  // basis on 1st row and their 1st derivatives on 2nd row
    ChBasisToolsBspline::BasisEvaluateDeriv(this->p, spanU, u, this->knots, NdN);

    ChVector<> dir = VNULL;
    int uind = spanU - p;
    for (int i = 0; i <= this->p; i++) {
        dir += points[uind + i] * NdN(1, i);
    }

    return dir;
}

void ChLineBspline::SetupData(
    int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChVector<> >& mpoints,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
) {
    if (morder < 1)
        throw ChException("ChLineBspline::SetupData requires order >= 1.");

    if (mpoints.size() < morder + 1)
        throw ChException("ChLineBspline::SetupData requires at least order+1 control points.");

    if (mknots && (size_t)mknots->size() != (mpoints.size() + morder + 1))
        throw ChException("ChLineBspline::SetupData: knots must have size=n_points+order+1");

    this->p = morder;
    this->points = mpoints;
    int n = (int)points.size();

    if (mknots)
        this->knots = *mknots;
    else {
        this->knots.setZero(n + p + 1);
        ChBasisToolsBspline::ComputeKnotUniformMultipleEnds(this->knots, p);
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
        ChBasisToolsBspline::ComputeKnotUniform(this->knots, p);
		
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
        ChBasisToolsBspline::ComputeKnotUniformMultipleEnds(this->knots, p);
	}

	this->closed = mc;
}

void ChLineBspline::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLineBspline>();
    // serialize parent class
    ChLine::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(points);
    marchive << CHNVP(knots);
    marchive << CHNVP(p);
	marchive << CHNVP(closed);
}

void ChLineBspline::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLineBspline>();
    // deserialize parent class
    ChLine::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(points);
    marchive >> CHNVP(knots);
    marchive >> CHNVP(p);
	marchive >> CHNVP(closed);
}

}  // end namespace geometry
}  // end namespace chrono
