//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "chrono_fea/ChLinkPointPoint.h"
#include "physics/ChSystem.h"
#include "physics/ChIndexedNodes.h"

using namespace chrono;
using namespace fea;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkPointPoint> a_registration_ChLinkPointPoint;

//////////////////////////////////////
//////////////////////////////////////

ChLinkPointPoint::ChLinkPointPoint() {
	this->react= VNULL;
	this->cache_li_speed = VNULL;
	this->cache_li_pos = VNULL;

	SetIdentifier(GetUniqueIntID()); // mark with unique ID
}

ChLinkPointPoint::~ChLinkPointPoint() {
}

void ChLinkPointPoint::Copy(ChLinkPointPoint* source) {
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

		// copy class data
	react = source->react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
}

ChCoordsys<> ChLinkPointPoint::GetLinkAbsoluteCoords() {
	return CSYSNORM;
}


int ChLinkPointPoint::Initialize(ChSharedPtr<ChNodeFEAxyz> anodeA,  ///< node to join
						         ChSharedPtr<ChNodeFEAxyz> anodeB  ///< node to join
                                 ) {
	assert(!anodeA.IsNull() && !anodeB.IsNull());

	this->mnodeA = anodeA;
    this->mnodeB = anodeB;

	this->constraint1.SetVariables(&(this->mnodeA->Variables()), &(this->mnodeB->Variables()));
	this->constraint2.SetVariables(&(this->mnodeA->Variables()), &(this->mnodeB->Variables()));
	this->constraint3.SetVariables(&(this->mnodeA->Variables()), &(this->mnodeB->Variables()));

	//this->SetSystem(this->body->GetSystem());

	return true;
}

void ChLinkPointPoint::Update(double mytime, bool update_assets) {
  // Inherit time changes of parent class
  ChPhysicsItem::Update(mytime, update_assets);

  // update class data
  // ...
}

ChMatrix<> ChLinkPointPoint::GetC() {
    ChVector<> res = mnodeA->GetPos() - mnodeB->GetPos();
    ChMatrixNM<double, 3, 1> C;
    C(0, 0) = res.x;
    C(1, 0) = res.y;
    C(2, 0) = res.z;
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPointPoint::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
	L(off_L+0) = this->react.x;
	L(off_L+1) = this->react.y;
	L(off_L+2) = this->react.z;
}

void ChLinkPointPoint::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
	this->react.x = L(off_L+0);
	this->react.y = L(off_L+1);
	this->react.z = L(off_L+2);
}

void ChLinkPointPoint::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*Cq'*L 
					const ChVectorDynamic<>& L,  ///< the L vector 
					const double c				 ///< a scaling factor
                                           ) {
	if (!IsActive())
    return;

	constraint1.MultiplyTandAdd(R, L(off_L+0) *c);
	constraint2.MultiplyTandAdd(R, L(off_L+1) *c);
	constraint3.MultiplyTandAdd(R, L(off_L+2) *c);
}

void ChLinkPointPoint::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
					ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
					const double c,				 ///< a scaling factor
					bool do_clamp,				 ///< apply clamping to c*C?
					double recovery_clamp		 ///< value for min/max clamping of c*C
                                           ) {
	if (!IsActive())
    return;

	ChVector<> res = mnodeA->GetPos() - mnodeB->GetPos(); 
	ChVector<> cres = res * c;

	if (do_clamp) {
		cres.x = ChMin(ChMax(cres.x, -recovery_clamp), recovery_clamp);
		cres.y = ChMin(ChMax(cres.y, -recovery_clamp), recovery_clamp);
		cres.z = ChMin(ChMax(cres.z, -recovery_clamp), recovery_clamp);
	}
	Qc(off_L+0) += cres.x;
	Qc(off_L+1) += cres.y;
	Qc(off_L+2) += cres.z;
}

void ChLinkPointPoint::IntToLCP(const unsigned int off_v,  ///< offset in v, R
					const ChStateDelta& v,
					const ChVectorDynamic<>& R,
					const unsigned int off_L,			///< offset in L, Qc
					const ChVectorDynamic<>& L,
                                const ChVectorDynamic<>& Qc) {
	if (!IsActive())
    return;

	constraint1.Set_l_i(L(off_L+0));
	constraint2.Set_l_i(L(off_L+1));
	constraint3.Set_l_i(L(off_L+2));

	constraint1.Set_b_i(Qc(off_L+0));
	constraint2.Set_b_i(Qc(off_L+1));
	constraint3.Set_b_i(Qc(off_L+2));
}

void ChLinkPointPoint::IntFromLCP(const unsigned int off_v,  ///< offset in v
					ChStateDelta& v,
					const unsigned int off_L,			///< offset in L
                                  ChVectorDynamic<>& L) {
	if (!IsActive())
    return;

	L(off_L+0)=constraint1.Get_l_i();
	L(off_L+1)=constraint2.Get_l_i();
	L(off_L+2)=constraint3.Get_l_i();
}

////////// LCP INTERFACES ////

void ChLinkPointPoint::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint1);
	mdescriptor.InsertConstraint(&constraint2);
	mdescriptor.InsertConstraint(&constraint3);
}

void ChLinkPointPoint::ConstraintsBiReset() {
	constraint1.Set_b_i(0.);
	constraint2.Set_b_i(0.);
	constraint3.Set_b_i(0.);
}
 
void ChLinkPointPoint::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {

	ChVector<> res = mnodeA->GetPos() -  mnodeB->GetPos(); 

	this->constraint1.Set_b_i(constraint1.Get_b_i() +  factor * res.x);
	this->constraint2.Set_b_i(constraint2.Get_b_i() +  factor * res.y);
	this->constraint3.Set_b_i(constraint3.Get_b_i() +  factor * res.z);
}

void ChLinkPointPoint::ConstraintsBiLoad_Ct(double factor) {
	//if (!this->IsActive())
	//	return;

	// nothing
}

void ChLinkPointPoint::ConstraintsLoadJacobians() {
		// compute jacobians
	ChMatrix33<> Jxa;
    Jxa.FillDiag(1.0);

    ChMatrix33<> Jxb;
    Jxb.FillDiag(-1.0);
	

	this->constraint1.Get_Cq_a()->PasteClippedMatrix(&Jxa, 0,0, 1,3, 0,0);
	this->constraint2.Get_Cq_a()->PasteClippedMatrix(&Jxa, 1,0, 1,3, 0,0);
	this->constraint3.Get_Cq_a()->PasteClippedMatrix(&Jxa, 2,0, 1,3, 0,0);

	this->constraint1.Get_Cq_b()->PasteClippedMatrix(&Jxb, 0,0, 1,3, 0,0);
	this->constraint2.Get_Cq_b()->PasteClippedMatrix(&Jxb, 1,0, 1,3, 0,0);
	this->constraint3.Get_Cq_b()->PasteClippedMatrix(&Jxb, 2,0, 1,3, 0,0);
}
 
void ChLinkPointPoint::ConstraintsFetch_react(double factor) {
	// From constraints to react vector:
	this->react.x = constraint1.Get_l_i() * factor; 
	this->react.y = constraint2.Get_l_i() * factor; 
	this->react.z = constraint3.Get_l_i() * factor; 
}

// Following functions are for exploiting the contact persistence

void ChLinkPointPoint::ConstraintsLiLoadSuggestedSpeedSolution() {
	constraint1.Set_l_i(this->cache_li_speed.x);
	constraint2.Set_l_i(this->cache_li_speed.y);
	constraint3.Set_l_i(this->cache_li_speed.z);
}

void ChLinkPointPoint::ConstraintsLiLoadSuggestedPositionSolution() {
	constraint1.Set_l_i(this->cache_li_pos.x);
	constraint2.Set_l_i(this->cache_li_pos.y);
	constraint3.Set_l_i(this->cache_li_pos.z);
}

void ChLinkPointPoint::ConstraintsLiFetchSuggestedSpeedSolution() {
	this->cache_li_speed.x = (float)constraint1.Get_l_i();
	this->cache_li_speed.y = (float)constraint2.Get_l_i();
	this->cache_li_speed.z = (float)constraint3.Get_l_i();
}

void ChLinkPointPoint::ConstraintsLiFetchSuggestedPositionSolution() {
	this->cache_li_pos.x =  (float)constraint1.Get_l_i();
	this->cache_li_pos.y =  (float)constraint2.Get_l_i();
	this->cache_li_pos.z =  (float)constraint3.Get_l_i();
}

//////// FILE I/O

void ChLinkPointPoint::StreamOUT(ChStreamOutBinary& mstream) {
    /*
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	mstream < this->react;
    */
}

void ChLinkPointPoint::StreamIN(ChStreamInBinary& mstream) {
    /*
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize class
	mstream >> this->react;
    */
}

/////////////////////
