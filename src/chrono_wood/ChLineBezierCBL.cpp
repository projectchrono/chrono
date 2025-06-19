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

#include "chrono_wood/ChLineBezierCBL.h"
#include "chrono_wood/ChBasisToolsBeziers.h"

namespace chrono {
namespace wood {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLineBezierCBL)

ChLineBezierCBL::ChLineBezierCBL() {
    const std::vector<ChVector3d > mpoints = {ChVector3d(-1, 0, 0), ChVector3d(1, 0, 0)};
    this->closed = false;
    this->SetupData(1, mpoints);
}

ChLineBezierCBL::ChLineBezierCBL(
    int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChVector3d >& mpoints,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
) {
    this->closed = false;
    this->SetupData(morder, mpoints, mknots);
}

ChLineBezierCBL::ChLineBezierCBL(const ChLineBezierCBL& source) : ChLine(source) {
    this->points = source.points;
    this->p = source.p;
    this->knots = source.knots;
	this->closed = source.closed;
}

ChVector3d ChLineBezierCBL::Evaluate(const double parU) const {
	double mU;
	/*if (this->closed)
		mU = fmod(parU, 1.0);
	else
		mU = parU;*/
    mU=parU;
    double uu = ComputeKnotUfromU(mU);   
    int spanU = ChBasisToolsBSpline::FindSpan(this->p, uu, this->knots);

    ChVectorDynamic<> N(this->p + 1);
    double u=(2.*mU)-1.;
    ChBasisToolsBeziers::BasisEvaluate(u, N);
    //std::cout<<"parU: "<<u<<"\nN:\n"<<N<<"\n";
    ChVector3d pos = VNULL;
    //int uind = 0;
    int uind = spanU - p;
    for (int i = 0; i <= this->p; i++) {
        pos += points[uind + i] * N(i);
    }
    
    return pos;
}

void ChLineBezierCBL::Derive(ChVector3d& dir, const double parU) {
	double mU;
	/*if (this->closed)
		mU = fmod(parU, 1.0);
	else
		mU = parU;*/
    mU=parU;  
    double uu = ComputeKnotUfromU(mU);    
    int spanU = ChBasisToolsBSpline::FindSpan(this->p, uu, this->knots);
    ChVectorDynamic<> N(this->p + 1);
    ChVectorDynamic<> NdN(this->p + 1);  // basis on 1st row and their 1st derivatives on 2nd row
    double u=(2.*mU)-1.;
    ChBasisToolsBeziers::BasisEvaluateDeriv(u, N, NdN); // TODO JBC: this computes N but does not use it. Call function that only returns derivative ?
    dir = VNULL;
    //int uind = 0;
    int uind = spanU - p;    
    for (int i = 0; i <= this->p; i++) {
        dir += points[uind + i] * NdN(i);
    }    
}

void ChLineBezierCBL::SetupData(
    int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChVector3d >& mpoints,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
) {
    if (morder < 1)
        throw std::invalid_argument("ChLineBezierCBL::SetupData requires order >= 1.");

    if (mpoints.size() < morder + 1)
        throw std::invalid_argument("ChLineBezierCBL::SetupData requires at least order+1 control points.");

    if (mknots && (size_t)mknots->size() != (mpoints.size() + morder + 1))
        throw std::invalid_argument("ChLineBezierCBL::SetupData: knots must have size=n_points+order+1");

    this->p = morder;
    this->points = mpoints;
    int n = (int)points.size();

    if (mknots)
        this->knots = *mknots;
    else {
    	int p1=p+1;
    	int Nsubline= (int) n/p1;
        this->knots.setZero(Nsubline + 2*p1-1);
        ChBasisToolsBSpline::ComputeKnotUniformMultipleEnds(this->knots, p);        
    }
}

void ChLineBezierCBL::SetClosed(bool mc) {
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

void ChLineBezierCBL::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLineBezierCBL>();
    // serialize parent class
    ChLine::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(points);
    marchive << CHNVP(knots);
    marchive << CHNVP(p);
	marchive << CHNVP(closed);
}

void ChLineBezierCBL::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLineBezierCBL>();
    // deserialize parent class
    ChLine::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(points);
    marchive >> CHNVP(knots);
    marchive >> CHNVP(p);
	marchive >> CHNVP(closed);
}

}  // end namespace wood
}  // end namespace chrono
