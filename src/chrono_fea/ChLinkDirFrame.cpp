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



#include "chrono_fea/ChLinkDirFrame.h"
#include "physics/ChSystem.h"
#include "physics/ChIndexedNodes.h"


using namespace chrono;
using namespace fea;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkDirFrame> a_registration_ChLinkDirFrame;



//////////////////////////////////////
//////////////////////////////////////



ChLinkDirFrame::ChLinkDirFrame ()
{
	this->react= VNULL;
	this->cache_li_speed = VNULL;
	this->cache_li_pos = VNULL;
	this->direction = VECT_X;

	SetIdentifier(GetUniqueIntID()); // mark with unique ID

}


ChLinkDirFrame::~ChLinkDirFrame ()
{
	
}

void ChLinkDirFrame::Copy(ChLinkDirFrame* source)
{
		// copy the parent class data...
	ChPhysicsItem::Copy(source);

		// copy class data

	react = source->react;
	cache_li_speed = source->cache_li_speed;
	cache_li_pos = source->cache_li_pos;
	direction= source->direction;
}

ChCoordsys<> ChLinkDirFrame::GetLinkAbsoluteCoords()
{
	if (this->body)
	{
		ChCoordsys<> linkcsys(
							this->mnode->GetPos(), // force to place the csys at node for aesthetics (the position is meaningless in this type of constraint anyway)
							this->csys_direction.rot >> (*this->body).GetRot()
							);
		return linkcsys;
	}
	return CSYSNORM;
}


void ChLinkDirFrame::SetDirectionInBodyCoords(ChVector<> mattach) 
{
	direction = mattach; 
	ChMatrix33<> rot;	
	rot.Set_A_Xdir(this->direction);
	this->csys_direction.rot = rot.Get_A_quaternion();
	this->csys_direction.pos = VNULL;
}

void ChLinkDirFrame::SetDirectionInAbsoluteCoords(ChVector<> mattach) 
{	
	direction = body->TransformDirectionParentToLocal(mattach); 
	SetDirectionInBodyCoords(direction);
}


int ChLinkDirFrame::Initialize(ChSharedPtr<ChNodeFEAxyzD> anode,  ///< node to join
						   ChSharedPtr<ChBodyFrame>  mbody,		///< body to join 
						   ChVector<>* mdir 	
						   )
{
	assert(!anode.IsNull() && !mbody.IsNull());

	this->body = mbody;
	this->mnode = anode;

	this->constraint1.SetVariables(&(this->mnode->Variables_D()), &(this->body->Variables()));
	this->constraint2.SetVariables(&(this->mnode->Variables_D()), &(this->body->Variables()));

	//this->SetSystem(this->body->GetSystem());

	if (mdir)
	{
		this->direction = body->TransformDirectionParentToLocal(*mdir);
	}
	else
	{
		// downcasting
		if (mnode.IsNull()) return false;

		ChVector<> temp= mnode->GetD(); 
		this->direction = body->TransformDirectionParentToLocal(temp);
	}

	return true;
}



void ChLinkDirFrame::Update(double mytime, bool update_assets)
{
  // Inherit time changes of parent class
  ChPhysicsItem::Update(mytime, update_assets);

  // ...
}

ChMatrix<> ChLinkDirFrame::GetC() {
    ChMatrix33<> Arw = csys_direction.rot >> body->coord.rot;
    ChVector<> res = Arw.MatrT_x_Vect(mnode->GetD());
    ChMatrixNM<double, 2, 1> C;
    C(0, 0) = res.y;
    C(1, 0) = res.z;
    return C;
}


//// STATE BOOKKEEPING FUNCTIONS

void ChLinkDirFrame::IntStateGatherReactions(const unsigned int off_L,	ChVectorDynamic<>& L)
{
	L(off_L+0) = 2*this->react.y;
	L(off_L+1) = 2*this->react.z;
}

void ChLinkDirFrame::IntStateScatterReactions(const unsigned int off_L,	const ChVectorDynamic<>& L)
{
	this->react.y = 0.5*L(off_L+0);
	this->react.z = 0.5*L(off_L+1);
}

void ChLinkDirFrame::IntLoadResidual_CqL(
					const unsigned int off_L,	 ///< offset in L multipliers
					ChVectorDynamic<>& R,		 ///< result: the R residual, R += c*Cq'*L 
					const ChVectorDynamic<>& L,  ///< the L vector 
					const double c				 ///< a scaling factor
					)
{
	if (!IsActive())
    return;

	constraint1.MultiplyTandAdd(R, L(off_L+0) *c);
	constraint2.MultiplyTandAdd(R, L(off_L+1) *c);
}

void ChLinkDirFrame::IntLoadConstraint_C(
					const unsigned int off_L,	 ///< offset in Qc residual
					ChVectorDynamic<>& Qc,		 ///< result: the Qc residual, Qc += c*C 
					const double c,				 ///< a scaling factor
					bool do_clamp,				 ///< apply clamping to c*C?
					double recovery_clamp		 ///< value for min/max clamping of c*C
					)
{
	if (!IsActive())
    return;

	ChMatrix33<> Arw = this->csys_direction.rot >> this->body->coord.rot;
	ChVector<> res = Arw.MatrT_x_Vect (mnode->GetD());

	ChVector<> cres = res * c;

	if (do_clamp) {
		cres.y = ChMin(ChMax(cres.y, -recovery_clamp), recovery_clamp);
		cres.z = ChMin(ChMax(cres.z, -recovery_clamp), recovery_clamp);
	}
	Qc(off_L+0) += cres.y;
	Qc(off_L+1) += cres.z;
}

void ChLinkDirFrame::IntToLCP(
					const unsigned int off_v,			///< offset in v, R
					const ChStateDelta& v,
					const ChVectorDynamic<>& R,
					const unsigned int off_L,			///< offset in L, Qc
					const ChVectorDynamic<>& L,
					const ChVectorDynamic<>& Qc
					)
{
	if (!IsActive())
    return;

	constraint1.Set_l_i(L(off_L+0));
	constraint2.Set_l_i(L(off_L+1));

	constraint1.Set_b_i(Qc(off_L+0));
	constraint2.Set_b_i(Qc(off_L+1));
}

void ChLinkDirFrame::IntFromLCP(
					const unsigned int off_v,			///< offset in v
					ChStateDelta& v,
					const unsigned int off_L,			///< offset in L
					ChVectorDynamic<>& L
					)
{
	if (!IsActive())
    return;

	L(off_L+0)=constraint1.Get_l_i();
	L(off_L+1)=constraint2.Get_l_i();
}


////////// LCP INTERFACES ////


void ChLinkDirFrame::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	//if (!this->IsActive())
	//	return;

	mdescriptor.InsertConstraint(&constraint1);
	mdescriptor.InsertConstraint(&constraint2);
}

void ChLinkDirFrame::ConstraintsBiReset()
{
	constraint1.Set_b_i(0.);
	constraint2.Set_b_i(0.);
}
 
void ChLinkDirFrame::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	//if (!this->IsActive())
	//	return;

	if (mnode.IsNull()) 
		return;
	ChMatrix33<> Arw = this->csys_direction.rot >> this->body->coord.rot;
	ChVector<> res = Arw.MatrT_x_Vect (mnode->GetD());

	this->constraint1.Set_b_i(constraint1.Get_b_i() +  factor * res.y);
	this->constraint2.Set_b_i(constraint2.Get_b_i() +  factor * res.z);
}

void ChLinkDirFrame::ConstraintsBiLoad_Ct(double factor)
{
	//if (!this->IsActive())
	//	return;

	// nothing
}


void ChLinkDirFrame::ConstraintsLoadJacobians()
{
		// compute jacobians
	ChMatrix33<> Aow(this->body->coord.rot);
	ChMatrix33<> Aro(this->csys_direction.rot);
	ChMatrix33<> Arw(this->csys_direction.rot >> this->body->coord.rot);

	ChVector<> Zo = Aow.MatrT_x_Vect( mnode->GetD() );
	
	ChMatrix33<> ztilde;
	ztilde.Set_X_matrix(Zo);

	ChMatrix33<> Jrb;
	Jrb.MatrTMultiply(Aro, ztilde);

	ChMatrix33<> Jra;
	Jra.CopyFromMatrixT(Arw);

	this->constraint1.Get_Cq_a()->PasteClippedMatrix(&Jra, 1,0, 1,3, 0,0);
	this->constraint2.Get_Cq_a()->PasteClippedMatrix(&Jra, 2,0, 1,3, 0,0);

	this->constraint1.Get_Cq_b()->PasteClippedMatrix(&Jrb, 1,0, 1,3, 0,3);
	this->constraint2.Get_Cq_b()->PasteClippedMatrix(&Jrb, 2,0, 1,3, 0,3);
}
 

void ChLinkDirFrame::ConstraintsFetch_react(double factor)
{
	// From constraints to react vector:
	this->react.y = constraint1.Get_l_i() * factor; 
	this->react.z = constraint2.Get_l_i() * factor; 
}

// Following functions are for exploiting the contact persistence

void  ChLinkDirFrame::ConstraintsLiLoadSuggestedSpeedSolution()
{
	constraint1.Set_l_i(this->cache_li_speed.y);
	constraint2.Set_l_i(this->cache_li_speed.z);
}

void  ChLinkDirFrame::ConstraintsLiLoadSuggestedPositionSolution()
{
	constraint1.Set_l_i(this->cache_li_pos.y);
	constraint2.Set_l_i(this->cache_li_pos.z);
}

void  ChLinkDirFrame::ConstraintsLiFetchSuggestedSpeedSolution()
{
	this->cache_li_speed.y = (float)constraint1.Get_l_i();
	this->cache_li_speed.z = (float)constraint2.Get_l_i();
}

void  ChLinkDirFrame::ConstraintsLiFetchSuggestedPositionSolution()
{
	this->cache_li_pos.y =  (float)constraint1.Get_l_i();
	this->cache_li_pos.z =  (float)constraint2.Get_l_i();
}



//////// FILE I/O

void ChLinkDirFrame::StreamOUT(ChStreamOutBinary& mstream)
{
    /*
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data
	//mstream << this->node_index;
	mstream < this->direction;
	mstream < this->react;
    */
}

void ChLinkDirFrame::StreamIN(ChStreamInBinary& mstream)
{
    /*
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// deserialize class
	//mstream >> this->node_index;
	mstream >> this->direction;
	mstream >> this->react;
    */
}





/////////////////////
