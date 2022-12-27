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



#include "chrono/motion_functions/ChFunctionRotation_spline.h"
#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono/motion_functions/ChFunction_Ramp.h"
#include "chrono/geometry/ChBasisToolsBspline.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChFunctionRotation_spline) 

ChFunctionRotation_spline::ChFunctionRotation_spline() {
    const std::vector<ChQuaternion<> > mrotations = {QUNIT, QUNIT};
	this->closed = false;
    this->SetupData(1, mrotations);

	// default s(t) function. User will provide better fx.
    space_fx = chrono_types::make_shared<ChFunction_Ramp>(0, 1.);
}

ChFunctionRotation_spline::ChFunctionRotation_spline(
    int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChQuaternion<> >& mrotations,  ///< control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
) {
	this->closed = false;
    this->SetupData(morder, mrotations, mknots);

	// default s(t) function. User will provide better fx.
    space_fx = chrono_types::make_shared<ChFunction_Ramp>(0, 1.);
}

ChFunctionRotation_spline::ChFunctionRotation_spline(const ChFunctionRotation_spline& other) {
    this->rotations = other.rotations;
    this->p = other.p;
    this->knots = other.knots;
	this->space_fx = other.space_fx;
	this->closed = other.closed;
}

ChFunctionRotation_spline::~ChFunctionRotation_spline() {

}

void ChFunctionRotation_spline::SetupData(
    int morder,                         ///< order p: 1= linear, 2=quadratic, etc.
    const std::vector<ChQuaternion<> >& mrotations,  ///< rotation control points, size n. Required: at least n >= p+1
    ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
) {
    if (morder < 1)
        throw ChException("ChFunctionRotation_spline::SetupData requires order >= 1.");

    if (mrotations.size() < morder + 1)
        throw ChException("ChFunctionRotation_spline::SetupData requires at least order+1 control points.");

    if (mknots && (size_t)mknots->size() != (mrotations.size() + morder + 1))
        throw ChException("ChFunctionRotation_spline::SetupData: knots must have size=n_points+order+1");

    this->p = morder;
    this->rotations = mrotations;
    int n = (int)rotations.size();

    if (mknots)
        this->knots = *mknots;
    else {
        this->knots.setZero(n + p + 1);
        geometry::ChBasisToolsBspline::ComputeKnotUniformMultipleEnds(this->knots, p);
    }
}



ChQuaternion<> ChFunctionRotation_spline::Get_q(double s) const {
	
	double fs = space_fx->Get_y(s);

	double mU;
	if (this->closed)
		mU = fmod(fs, 1.0);
	else
		mU = fs;

    double u = ComputeKnotUfromU(mU);

    int spanU = geometry::ChBasisToolsBspline::FindSpan(this->p, u, this->knots);

    ChVectorDynamic<> N(this->p + 1);
    geometry::ChBasisToolsBspline::BasisEvaluate(this->p, spanU, u, this->knots, N);

	// Use the quaternion spline interpolation as in Kim & Kim 1995 paper, ie. with cumulative basis.
	// Note: now, instead of creating a ChVectorDynamic<> Bi(this->p + 1); vector containing all the basis
	// recomputed in cumulative form, we just use a single scalar Bi and update it only when needed.
	
    int uind = spanU - p;
	ChQuaternion<> qqpowBi;
	ChQuaternion<> q0 = rotations[uind + 0];  // q0
	ChQuaternion<> q; 
	double Bi = 0;
	// should be Bi(0) = 1 in most cases, anyway: compute first cumulative basis:
	for (int j = 0; j < N.size(); ++j)
		Bi += N(j);  
	q.Q_from_Rotv(q0.Q_to_Rotv() * Bi);

    for (int i = 1; i <= this->p; i++) {
		// update cumulative basis - avoids repeating the sums, as it should be:  Bi=0; for (int j = i; j < N.size(); ++j) {Bi += N(j);} 
		Bi -= N(i-1); 
		// compute delta rotation
		qqpowBi.Q_from_Rotv((rotations[uind + i - 1].GetConjugate() * rotations[uind + i]).Q_to_Rotv() * Bi);
		// apply delta rotation
		q *= qqpowBi;
    }

	return q;
}


void ChFunctionRotation_spline::SetClosed(bool mc) {
	if (this->closed == mc)
		return;

	// switch open->closed
	if (mc == true) {
		// add p control points to be wrapped: resize knots and control points
		auto n = this->rotations.size();
		n += p; 
		this->rotations.resize(n);
		this->knots.setZero(n + p + 1);
		
		// recompute knot vector spacing
        geometry::ChBasisToolsBspline::ComputeKnotUniform(this->knots, p);
		
		// wrap last control points
		for (int i = 0; i < p; ++i)
			this->rotations[n - p + i] = this->rotations[i];
	}
	
	// switch closed->open
	if (mc == false) {
		// remove p control points that was wrapped: resize knots and control points
		auto n = this->rotations.size();
		n -= p; 
		this->rotations.resize(n);
		this->knots.setZero(n + p + 1);

		// recompute knot vector spacing
        geometry::ChBasisToolsBspline::ComputeKnotUniformMultipleEnds(this->knots, p);
	}

	this->closed = mc;
}


void ChFunctionRotation_spline::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionRotation_spline>();
	// serialize parent class
    ChFunctionRotation::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(rotations);
    ////marchive << CHNVP(knots);  //**TODO MATRIX DESERIALIZATION
    marchive << CHNVP(p);
	marchive << CHNVP(space_fx);
	marchive << CHNVP(closed);

}

void ChFunctionRotation_spline::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionRotation_spline>();
	// deserialize parent class
    ChFunctionRotation::ArchiveIN(marchive);
    // deserialize all member data:
    marchive >> CHNVP(rotations);
    ////marchive >> CHNVP(knots);  //**TODO MATRIX DESERIALIZATION
    marchive >> CHNVP(p);
	marchive >> CHNVP(space_fx);
	marchive >> CHNVP(closed);
}



}  // end namespace chrono