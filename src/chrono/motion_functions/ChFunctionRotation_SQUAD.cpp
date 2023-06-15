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



#include "chrono/motion_functions/ChFunctionRotation_SQUAD.h"
#include "chrono/motion_functions/ChFunction_Const.h"
#include "chrono/motion_functions/ChFunction_Ramp.h"
#include "chrono/geometry/ChBasisToolsBspline.h"

namespace chrono {

	// Register into the object factory, to enable run-time dynamic creation and persistence
	CH_FACTORY_REGISTER(ChFunctionRotation_SQUAD)

		ChFunctionRotation_SQUAD::ChFunctionRotation_SQUAD() {
		const std::vector<ChQuaternion<> > mrotations = { QUNIT, QUNIT };
		this->closed = false;
		this->SetupData(mrotations);

		// default s(t) function. User will provide better fx.
		space_fx = chrono_types::make_shared<ChFunction_Ramp>(0, 1.);
	}

	ChFunctionRotation_SQUAD::ChFunctionRotation_SQUAD(
		const std::vector<ChQuaternion<> >& mrotations,  ///< control points, size n. Required: at least n >= p+1
		ChVectorDynamic<>* mknots           ///< knots, size k. Required k=n+p+1. If not provided, initialized to uniform.
	) {
		this->closed = false;
		this->SetupData(mrotations, mknots);

		// default s(t) function. User will provide better fx.
		space_fx = chrono_types::make_shared<ChFunction_Ramp>(0, 1.);
	}

	ChFunctionRotation_SQUAD::ChFunctionRotation_SQUAD(const ChFunctionRotation_SQUAD& other) {
		this->rotations = other.rotations;
		this->p = other.p;
		this->knots = other.knots;
		this->space_fx = other.space_fx;
		this->closed = other.closed;
	}

	ChFunctionRotation_SQUAD::~ChFunctionRotation_SQUAD() {

	}

	void ChFunctionRotation_SQUAD::SetupData(
		const std::vector<ChQuaternion<> >& mrotations,  ///< rotation control points, size n. Required: at least n >= p+1
		ChVectorDynamic<>* mknots           ///< knots, as many as control points. If not provided, initialized to uniform.
	) {
		if (mrotations.size() < 2)
			throw ChException("ChFunctionRotation_SQUAD::SetupData requires at least 2control points.");

		if (mknots && (size_t)mknots->size() != mrotations.size())
			throw ChException("ChFunctionRotation_SQUAD::SetupData: knots must be as many as control points");

		this->rotations = mrotations;
		auto n = (int)rotations.size();

		if (mknots)
			this->knots = *mknots;
		else {
			this->knots.setZero(n);
			geometry::ChBasisToolsBspline::ComputeKnotUniform(this->knots, 1);
		}
	}


ChQuaternion<> SLERP(const ChQuaternion<>& qa, const ChQuaternion<>& qb, double t) {
	ChQuaternion<> qdelta = qa.GetConjugate() * qb;
	ChQuaternion<> qdelta_t; qdelta_t.Q_from_Rotv(qdelta.Q_to_Rotv() * t);

	return  qa * qdelta_t;
};

ChQuaternion<> QUADRANGLE(const ChQuaternion<>& q0, const ChQuaternion<>& q1, const ChQuaternion<>& q2) {
       
	ChQuaternion<> qInv = q1.GetConjugate();

	ChQuaternion<> sq1 = qInv * q2;
        
	ChVector<> cart0 = sq1.Q_to_Rotv(); 
 
	ChQuaternion<> sq0 = qInv * q0;

	ChVector<> cart1 = sq0.Q_to_Rotv();

	ChVector<> cart_aux = (cart0 + cart1) * -0.25;

	ChQuaternion<> q_aux; q_aux.Q_from_Rotv(cart_aux);

	return q1 * q_aux; 
 };



ChQuaternion<> ChFunctionRotation_SQUAD::Get_q(double s) const {
	
	double fs = space_fx->Get_y(s);

	double mU;
	if (this->closed)
		mU = fmod(fs, 1.0);
	else
		mU = fs;

    double u = ComputeKnotUfromU(mU);

	// Linear search of span in knot vector. Bisection could be faster, but for small number of knots this is efficient anyway.
	int spanU = 0;
	for (; spanU < this->knots.size()-2; ++spanU)
		if (u < this->knots(spanU+1))
			break;
	
	double span_t = (u - this->knots(spanU)) / (this->knots(spanU + 1) - this->knots(spanU));

	ChQuaternion<> q1 = rotations[spanU];
	ChQuaternion<> q2 = rotations[spanU+1];
	
	ChQuaternion<> q0;
	ChQuaternion<> q3;
	
	if (spanU > 0) {
		q0 = rotations[spanU - 1];
	} 
	else {
		if (this->closed)
			q0 = rotations[rotations.size()-2];
		else
			q0 = q1;
	}

	if (spanU < rotations.size()-2) {
		q3 = rotations[spanU +2];
	} 
	else {
		if (this->closed)
			q3 = rotations[1];
		else
			q3 = q2;
	}

	// TEST: do a simple SLERP
	//return SLERP(q1, q2, span_t);

	ChQuaternion<> s0 = QUADRANGLE(q0, q1, q2);
	ChQuaternion<> s1 = QUADRANGLE(q1, q2, q3);
	
	ChQuaternion<> slerp0 = SLERP(q1, q2, span_t);
    ChQuaternion<> slerp1 = SLERP(s0, s1, span_t);
        
	return SLERP(slerp0, slerp1, 2.0 * span_t * (1.0 - span_t));

}




void ChFunctionRotation_SQUAD::SetClosed(bool mc) {
	if (this->closed == mc)
		return;

	// switch open->closed
	if (mc == true) {
		// add p control points to be wrapped: resize knots and control points
		auto n = this->rotations.size();
		n += 1; 
		this->rotations.resize(n);
		this->knots.setZero(n);
		
		// recompute knot vector spacing
        geometry::ChBasisToolsBspline::ComputeKnotUniform(this->knots, 1);
		
		// wrap last control point
		this->rotations[n - 1] = this->rotations[0];
	}
	
	// switch closed->open
	if (mc == false) {
		// remove p control points that was wrapped: resize knots and control points
		auto n = this->rotations.size();
		n -= 1; 
		this->rotations.resize(n);
		this->knots.setZero(n);

		// recompute knot vector spacing
        geometry::ChBasisToolsBspline::ComputeKnotUniform(this->knots, 1);
	}

	this->closed = mc;
}


void ChFunctionRotation_SQUAD::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChFunctionRotation_SQUAD>();
	// serialize parent class
    ChFunctionRotation::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(rotations);
    ////marchive << CHNVP(knots);  //**TODO MATRIX DESERIALIZATION
    marchive << CHNVP(p);
	marchive << CHNVP(space_fx);
	marchive << CHNVP(closed);

}

void ChFunctionRotation_SQUAD::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChFunctionRotation_SQUAD>();
	// deserialize parent class
    ChFunctionRotation::ArchiveIn(marchive);
    // deserialize all member data:
    marchive >> CHNVP(rotations);
    ////marchive >> CHNVP(knots);  //**TODO MATRIX DESERIALIZATION
    marchive >> CHNVP(p);
	marchive >> CHNVP(space_fx);
	marchive >> CHNVP(closed);
}



}  // end namespace chrono