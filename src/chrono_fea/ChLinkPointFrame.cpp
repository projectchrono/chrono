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

#include "chrono_fea/ChLinkPointFrame.h"
#include "physics/ChSystem.h"
#include "physics/ChIndexedNodes.h"

using namespace chrono;
using namespace fea;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkPointFrame> a_registration_ChLinkPointFrame;

//////////////////////////////////////
//////////////////////////////////////

ChLinkPointFrame::ChLinkPointFrame() {
	this->react= VNULL;
	this->cache_li_speed = VNULL;
	this->cache_li_pos = VNULL;
	this->attach_reference = CSYSNORM;

	SetIdentifier(GetUniqueIntID()); // mark with unique ID
}

ChLinkPointFrame::~ChLinkPointFrame() {
}

void ChLinkPointFrame::Copy(ChLinkPointFrame* source) {
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

		// copy class data
	attach_reference = source->attach_reference;
	react = source->react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
}

ChCoordsys<> ChLinkPointFrame::GetLinkAbsoluteCoords() {
    if (this->body) {
		ChCoordsys<> linkcsys = attach_reference >> (*this->body);
		return linkcsys;
	}
	return CSYSNORM;
}

int ChLinkPointFrame::Initialize(ChSharedPtr<ChIndexedNodes> mnodes, ///< nodes container
						   unsigned int mnode_index, ///< index of the node to join
						   ChSharedPtr<ChBodyFrame>  mbody,   ///< body to join 
                                 ChVector<>* mattach) {
	assert(!mnodes.IsNull());

	ChSharedPtr<ChNodeFEAxyz> anode(mnodes->GetNode(mnode_index).DynamicCastTo<ChNodeFEAxyz>() ); // downcasting
	
	if (anode.IsNull()) 
        return false;  // downcasting wasn't successfull (in a ChIndexedNodes, different types of nodes could be
                       // present..)

	return this->Initialize(anode, mbody, mattach);
}

int ChLinkPointFrame::Initialize(ChSharedPtr<ChNodeFEAxyz> anode,  ///< node to join
						   ChSharedPtr<ChBodyFrame>  mbody,		///< body to join 
                                 ChVector<>* mattach) {
	assert(!anode.IsNull() && !mbody.IsNull());

	this->body = mbody;
	this->mnode = anode;

	this->constraint1.SetVariables(&(this->mnode->Variables()), &(this->body->Variables()));
	this->constraint2.SetVariables(&(this->mnode->Variables()), &(this->body->Variables()));
	this->constraint3.SetVariables(&(this->mnode->Variables()), &(this->body->Variables()));

	//this->SetSystem(this->body->GetSystem());

    if (mattach) {
		this->attach_reference.pos = body->TransformPointParentToLocal(*mattach);
    } else {
		// downcasting
        if (mnode.IsNull())
            return false;

		ChVector<> temp= mnode->GetPos(); 
		this->attach_reference.pos = body->TransformPointParentToLocal(temp);
	}

	return true;
}

void ChLinkPointFrame::Update(double mytime, bool update_assets) {
  // Inherit time changes of parent class
  ChPhysicsItem::Update(mytime, update_assets);

  // update class data
  // ...
}

ChMatrix<> ChLinkPointFrame::GetC() {
    ChMatrix33<> Arw(attach_reference.rot >> body->GetRot());
    ChVector<> res = Arw.MatrT_x_Vect(mnode->GetPos() - body->TransformPointLocalToParent(attach_reference.pos));
    ChMatrixNM<double, 3, 1> C;
    C(0, 0) = res.y;
    C(1, 0) = res.z;
    C(2, 0) = res.z;
    return C;
}

//// STATE BOOKKEEPING FUNCTIONS

void ChLinkPointFrame::IntStateGatherReactions(const unsigned int off_L, ChVectorDynamic<>& L) {
	L(off_L+0) = this->react.x;
	L(off_L+1) = this->react.y;
	L(off_L+2) = this->react.z;
}

void ChLinkPointFrame::IntStateScatterReactions(const unsigned int off_L, const ChVectorDynamic<>& L) {
	this->react.x = L(off_L+0);
	this->react.y = L(off_L+1);
	this->react.z = L(off_L+2);
}

void ChLinkPointFrame::IntLoadResidual_CqL(const unsigned int off_L,    ///< offset in L multipliers
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

void ChLinkPointFrame::IntLoadConstraint_C(const unsigned int off_L,  ///< offset in Qc residual
					ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
					const double c,				 ///< a scaling factor
					bool do_clamp,				 ///< apply clamping to c*C?
					double recovery_clamp		 ///< value for min/max clamping of c*C
                                           ) {
	if (!IsActive())
    return;

	ChMatrix33<> Arw (attach_reference.rot >> body->GetRot());

	ChVector<> res = Arw.MatrT_x_Vect(mnode->GetPos() - this->body->TransformPointLocalToParent(this->attach_reference.pos) ); 
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

void ChLinkPointFrame::IntToLCP(const unsigned int off_v,  ///< offset in v, R
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

void ChLinkPointFrame::IntFromLCP(const unsigned int off_v,  ///< offset in v
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

void ChLinkPointFrame::InjectConstraints(ChLcpSystemDescriptor& mdescriptor) {
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint1);
	mdescriptor.InsertConstraint(&constraint2);
	mdescriptor.InsertConstraint(&constraint3);
}

void ChLinkPointFrame::ConstraintsBiReset() {
	constraint1.Set_b_i(0.);
	constraint2.Set_b_i(0.);
	constraint3.Set_b_i(0.);
}
 
void ChLinkPointFrame::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp) {
	//if (!this->IsActive())
	//	return;

	if (mnode.IsNull()) 
		return;

	ChMatrix33<> Arw (attach_reference.rot >> body->GetRot());

	ChVector<> res = Arw.MatrT_x_Vect(mnode->GetPos() - this->body->TransformPointLocalToParent(this->attach_reference.pos) ); 

	this->constraint1.Set_b_i(constraint1.Get_b_i() +  factor * res.x);
	this->constraint2.Set_b_i(constraint2.Get_b_i() +  factor * res.y);
	this->constraint3.Set_b_i(constraint3.Get_b_i() +  factor * res.z);
}

void ChLinkPointFrame::ConstraintsBiLoad_Ct(double factor) {
	//if (!this->IsActive())
	//	return;

	// nothing
}

void ChLinkPointFrame::ConstraintsLoadJacobians() {
		// compute jacobians
	ChMatrix33<> Aro (attach_reference.rot); 
	ChMatrix33<> Aow (body->GetRot());
	ChMatrix33<> Arw = Aow * Aro;

	ChMatrix33<> Jxn;
	Jxn.CopyFromMatrixT(Arw);

	ChMatrix33<> Jxb;
	Jxb.CopyFromMatrixT(Arw);
	Jxb.MatrNeg();

	ChMatrix33<> atilde;
	atilde.Set_X_matrix(Aow.MatrT_x_Vect(mnode->GetPos()- body->GetPos()));
	ChMatrix33<> Jrb;
	Jrb.MatrTMultiply(Aro, atilde);

	this->constraint1.Get_Cq_a()->PasteClippedMatrix(&Jxn, 0,0, 1,3, 0,0);
	this->constraint2.Get_Cq_a()->PasteClippedMatrix(&Jxn, 1,0, 1,3, 0,0);
	this->constraint3.Get_Cq_a()->PasteClippedMatrix(&Jxn, 2,0, 1,3, 0,0);

	this->constraint1.Get_Cq_b()->PasteClippedMatrix(&Jxb, 0,0, 1,3, 0,0);
	this->constraint2.Get_Cq_b()->PasteClippedMatrix(&Jxb, 1,0, 1,3, 0,0);
	this->constraint3.Get_Cq_b()->PasteClippedMatrix(&Jxb, 2,0, 1,3, 0,0);
	this->constraint1.Get_Cq_b()->PasteClippedMatrix(&Jrb, 0,0, 1,3, 0,3);
	this->constraint2.Get_Cq_b()->PasteClippedMatrix(&Jrb, 1,0, 1,3, 0,3);
	this->constraint3.Get_Cq_b()->PasteClippedMatrix(&Jrb, 2,0, 1,3, 0,3);
}
 
void ChLinkPointFrame::ConstraintsFetch_react(double factor) {
	// From constraints to react vector:
	this->react.x = constraint1.Get_l_i() * factor; 
	this->react.y = constraint2.Get_l_i() * factor; 
	this->react.z = constraint3.Get_l_i() * factor; 
}

// Following functions are for exploiting the contact persistence

void ChLinkPointFrame::ConstraintsLiLoadSuggestedSpeedSolution() {
	constraint1.Set_l_i(this->cache_li_speed.x);
	constraint2.Set_l_i(this->cache_li_speed.y);
	constraint3.Set_l_i(this->cache_li_speed.z);
}

void ChLinkPointFrame::ConstraintsLiLoadSuggestedPositionSolution() {
	constraint1.Set_l_i(this->cache_li_pos.x);
	constraint2.Set_l_i(this->cache_li_pos.y);
	constraint3.Set_l_i(this->cache_li_pos.z);
}

void ChLinkPointFrame::ConstraintsLiFetchSuggestedSpeedSolution() {
	this->cache_li_speed.x = (float)constraint1.Get_l_i();
	this->cache_li_speed.y = (float)constraint2.Get_l_i();
	this->cache_li_speed.z = (float)constraint3.Get_l_i();
}

void ChLinkPointFrame::ConstraintsLiFetchSuggestedPositionSolution() {
	this->cache_li_pos.x =  (float)constraint1.Get_l_i();
	this->cache_li_pos.y =  (float)constraint2.Get_l_i();
	this->cache_li_pos.z =  (float)constraint3.Get_l_i();
}

//////// FILE I/O

void ChLinkPointFrame::StreamOUT(ChStreamOutBinary& mstream) {
    /*
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	//mstream << this->node_index;
	mstream < this->attach_reference;
	mstream < this->react;
    */
}

void ChLinkPointFrame::StreamIN(ChStreamInBinary& mstream) {
    /*
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize class
	//mstream >> this->node_index;
	mstream >> this->attach_reference;
	mstream >> this->react;
    */
}

/////////////////////
