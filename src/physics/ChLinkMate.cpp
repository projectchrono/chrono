///////////////////////////////////////////////////
//
//   ChLinkMate.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
#include "physics/ChLinkMate.h"
#include "physics/ChSystem.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{



////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMate> a_registration_ChLinkMate;

ChLink* ChLinkMate::new_Duplicate ()
{
    ChLinkMate* m_l = new ChLinkMate;
    m_l->Copy(this);
    return (m_l);
}


// ...





////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMateGeneric> a_registration_ChLinkMateGeneric;



ChLinkMateGeneric::ChLinkMateGeneric (bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz)
{ 
	c_x = mc_x;
	c_y = mc_y;
	c_z = mc_z;
	c_rx = mc_rx;
	c_ry = mc_ry;
	c_rz = mc_rz;

	C =0;
	cache_li_pos = 0;
	cache_li_speed = 0;

	mask = new ChLinkMask();

	SetupLinkMask();
}

ChLinkMateGeneric::~ChLinkMateGeneric ()
{
	if (C)
		delete C;
	C=0;

	if (cache_li_pos)
		delete cache_li_pos;
	cache_li_pos=0;

	if (cache_li_speed)
		delete cache_li_speed;
	cache_li_speed=0;

	if (mask)
		delete mask; 
	mask=0;
}


void ChLinkMateGeneric::Copy(ChLinkMateGeneric* source)
{
    // first copy the parent class data...
    //
    ChLinkMate::Copy(source);
	
	c_x = source->c_x;
	c_y = source->c_y;
	c_z = source->c_z;
	c_rx = source->c_rx;
	c_ry = source->c_ry;
	c_rz = source->c_rz;

	SetupLinkMask();
}

ChLink* ChLinkMateGeneric::new_Duplicate ()
{
    ChLinkMateGeneric * m_l = new ChLinkMateGeneric;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}


void ChLinkMateGeneric::SetConstrainedCoords(bool mc_x, bool mc_y, bool mc_z, bool mc_rx, bool mc_ry, bool mc_rz)
{
	c_x = mc_x;
	c_y = mc_y;
	c_z = mc_z;
	c_rx = mc_rx;
	c_ry = mc_ry;
	c_rz = mc_rz;

	SetupLinkMask();
}


void ChLinkMateGeneric::SetupLinkMask()
{
	int nc = 0;
	if (c_x) nc++;
	if (c_y) nc++;
	if (c_z) nc++;
	if (c_rx) nc++;
	if (c_ry) nc++;
	if (c_rz) nc++;

	mask->ResetNconstr(nc);

	if (C)
		delete C;
	C   = new ChMatrixDynamic<>(nc,1);

	if (cache_li_pos)
		delete cache_li_pos;
	cache_li_pos   = new ChMatrixDynamic<>(nc,1);

	if (cache_li_speed)
		delete cache_li_speed;
	cache_li_speed = new ChMatrixDynamic<>(nc,1);

	ChangedLinkMask();
}

void ChLinkMateGeneric::ChangedLinkMask()
{
	ndoc   = mask->GetMaskDoc();
	ndoc_c = mask->GetMaskDoc_c();
	ndoc_d = mask->GetMaskDoc_d();
}


void ChLinkMateGeneric::SetDisabled(bool mdis)
{
	ChLinkMate::SetDisabled(mdis);

    if (mask->SetAllDisabled(mdis) >0)
        ChangedLinkMask ();
}

void ChLinkMateGeneric::SetBroken(bool mbro)
{
	ChLinkMate::SetBroken(mbro);

    if (mask->SetAllBroken(mbro) >0)
        ChangedLinkMask ();
}


int ChLinkMateGeneric::RestoreRedundant()
{
    int mchanges = mask->RestoreRedundant();
    if (mchanges)
        ChangedLinkMask ();
    return mchanges;
}


void ChLinkMateGeneric::Update (double mytime)
{
    // Inherit time changes of parent class (ChLink), basically doing nothing :)
    ChLink::UpdateTime(mytime);

	if(this->Body1 && this->Body2)
	{
		this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());
	
		ChFrame<> aframe = this->frameA >> (*this->Body1);  
		ChFrame<> bframe;  
		static_cast<ChFrame<>*>(this->Body2)->TrasformParentToLocal(aframe, bframe);
		this->frameB.TrasformParentToLocal(bframe, aframe);
		// Now 'aframe' contains the position/rotation of frame 1 respect to frame 2, in frame 2 coords.

		ChMatrix33<> Jx1, Jx2, Jr1, Jr2, Jw1, Jw2;
		ChMatrix33<> Ps1, Ps2;
		Ps1.Set_X_matrix(this->frameA.GetPos());
		Ps2.Set_X_matrix(this->frameB.GetPos());
		
		ChMatrix33<> abs_plane;
		abs_plane.MatrMultiply(*this->Body2->GetA(), *this->frameB.GetA());

		Jx1.CopyFromMatrixT(abs_plane);
		Jx2.CopyFromMatrixT(abs_plane);
		Jx2.MatrNeg();

		Jw1.MatrTMultiply(abs_plane, *( const_cast<ChFrame<>*>(&frameA)->GetA()) );
		Jr1.MatrMultiply(Jw1, Ps1);

		Jw2.MatrTMultiply(abs_plane, *( const_cast<ChFrame<>*>(&frameB)->GetA()) );
		Jr2.MatrMultiply(Jw2, Ps2);

		Jr1.MatrNeg();
		Jw2.MatrNeg();

		int nc = 0;

		if (c_x) 
		{
			this->C->ElementN(nc) = aframe.GetPos().x;
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jx1, 0,0, 1,3, 0,0);
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jr1, 0,0, 1,3, 0,3);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jx2, 0,0, 1,3, 0,0);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jr2, 0,0, 1,3, 0,3);
			nc++;
		}
		if (c_y) 
		{
			this->C->ElementN(nc) = aframe.GetPos().y;
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jx1, 1,0, 1,3, 0,0);
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jr1, 1,0, 1,3, 0,3);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jx2, 1,0, 1,3, 0,0);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jr2, 1,0, 1,3, 0,3);
			nc++;
		}
		if (c_z) 
		{
			this->C->ElementN(nc) = aframe.GetPos().z;
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jx1, 2,0, 1,3, 0,0);
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jr1, 2,0, 1,3, 0,3);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jx2, 2,0, 1,3, 0,0);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jr2, 2,0, 1,3, 0,3);
			nc++;
		}
		if (c_rx) 
		{
			this->C->ElementN(nc) = aframe.GetRot().e1;
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jw1, 0,0, 1,3, 0,3);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jw2, 0,0, 1,3, 0,3);
			nc++;
		}
		if (c_ry) 
		{
			this->C->ElementN(nc) = aframe.GetRot().e2;
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jw1, 1,0, 1,3, 0,3);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jw2, 1,0, 1,3, 0,3);
			nc++;
		}
		if (c_rz) 
		{
			this->C->ElementN(nc) = aframe.GetRot().e3;
			this->mask->Constr_N(nc).Get_Cq_a()->PasteClippedMatrix(&Jw1, 2,0, 1,3, 0,3);
			this->mask->Constr_N(nc).Get_Cq_b()->PasteClippedMatrix(&Jw2, 2,0, 1,3, 0,3);
			nc++;
		}

		

	}

}



int ChLinkMateGeneric::Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChFrame<> mpos1,			///< mate frame (slave), for 1st body (rel. or abs., see flag above)
						   ChFrame<> mpos2  		///< mate frame (master), for 2nd body (rel. or abs., see flag above) 
						   )
{
	assert(mbody1.get_ptr() != mbody2.get_ptr());
	assert(mbody1->GetSystem() == mbody2->GetSystem());

	this->Body1 = mbody1.get_ptr();
	this->Body2 = mbody2.get_ptr();
	this->SetSystem(mbody1->GetSystem());

	this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());
	
	if (pos_are_relative)
	{
		this->frameA = mpos1;
		this->frameB = mpos2;
	}
	else
	{
		// from abs to body-rel
		static_cast<ChFrame<>*>(this->Body2)->TrasformParentToLocal(mpos1, this->frameA);
		static_cast<ChFrame<>*>(this->Body2)->TrasformParentToLocal(mpos2, this->frameB);
	}
	return true;
}



////////// LCP INTERFACES ////


void ChLinkMateGeneric::InjectConstraints(ChLcpSystemDescriptor& mdescriptor)
{
	if (!this->IsActive())
		return;

	for (int i=0; i< mask->nconstr; i++)
	{
		if (mask->Constr_N(i).IsActive())
			mdescriptor.InsertConstraint(&mask->Constr_N(i));
	}
}

void ChLinkMateGeneric::ConstraintsBiReset()
{
	if (!this->IsActive())
		return;

	for (int i=0; i< mask->nconstr; i++)
	{
		mask->Constr_N(i).Set_b_i(0.);
	}
}
 
void ChLinkMateGeneric::ConstraintsBiLoad_C(double factor, double recovery_clamp, bool do_clamp)
{
	if (!this->IsActive())
		return;

	int cnt=0;
	for (int i=0; i< mask->nconstr; i++)
	{
		if (mask->Constr_N(i).IsActive())
		{
			if (do_clamp)
			{
				if (mask->Constr_N(i).IsUnilateral())
					mask->Constr_N(i).Set_b_i( mask->Constr_N(i).Get_b_i() + ChMax (factor * C->ElementN(cnt), -recovery_clamp)  );
				else 
					mask->Constr_N(i).Set_b_i( mask->Constr_N(i).Get_b_i() + ChMin(ChMax (factor * C->ElementN(cnt), -recovery_clamp), recovery_clamp)  );
			}
			else
				mask->Constr_N(i).Set_b_i( mask->Constr_N(i).Get_b_i() + factor * C->ElementN(cnt)  );

			cnt++;
		}
	}
}

void ChLinkMateGeneric::ConstraintsBiLoad_Ct(double factor)
{
	if (!this->IsActive())
		return;

	// NOT NEEDED BECAUSE NO RHEONOMIC TERM
	
}



void ChLinkMateGeneric::ConstraintsLoadJacobians()
{
	// already loaded when doing Update (which used the matrices of the scalar constraint objects)

	
}
 

void ChLinkMateGeneric::ConstraintsFetch_react(double factor)
{
	react_force  = VNULL;
	react_torque = VNULL;

	if (!this->IsActive())
		return;

	int nc = 0;
	if (c_x) 
	{
		if (mask->Constr_N(nc).IsActive())
			react_force.x = - mask->Constr_N(nc).Get_l_i() * factor;
		nc++;
	}
	if (c_y) 
	{
		if (mask->Constr_N(nc).IsActive())
			react_force.y = - mask->Constr_N(nc).Get_l_i() * factor;
		nc++;
	}
	if (c_z) 
	{
		if (mask->Constr_N(nc).IsActive())
			react_force.z = - mask->Constr_N(nc).Get_l_i() * factor;
		nc++;
	}
	if (c_rx) 
	{
		if (mask->Constr_N(nc).IsActive())
			react_torque.x = - 0.5*mask->Constr_N(nc).Get_l_i() * factor;
		nc++;
	}
	if (c_ry) 
	{
		if (mask->Constr_N(nc).IsActive())
			react_torque.y = - 0.5*mask->Constr_N(nc).Get_l_i() * factor;
		nc++;
	}
	if (c_rz) 
	{
		if (mask->Constr_N(nc).IsActive())
			react_torque.z = - 0.5*mask->Constr_N(nc).Get_l_i() * factor;
		nc++;
	}

	
}


int ChLinkMateGeneric::Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpt1,			
						   ChVector<> mpt2,  		
						   ChVector<> mnorm1,		
						   ChVector<> mnorm2  
						   )
{
	assert(mbody1.get_ptr() != mbody2.get_ptr());
	assert(mbody1->GetSystem() == mbody2->GetSystem());

	this->Body1 = mbody1.get_ptr();
	this->Body2 = mbody2.get_ptr();
	this->SetSystem(mbody1->GetSystem());

	this->mask->SetTwoBodiesVariables(&Body1->Variables(), &Body2->Variables());
	
	ChVector<> mx, my, mz, mN;
	ChMatrix33<> mrot;

	ChFrame<> mfr1;
	ChFrame<> mfr2;

	if (pos_are_relative)
	{
		mN = mnorm1;
		mN.DirToDxDyDz(&mx, &my, &mz);
		mrot.Set_A_axis(mx,my,mz);
		mfr1.SetRot(mrot);
		mfr1.SetPos(mpt1);

		mN = - mnorm2;
		mN.DirToDxDyDz(&mx, &my, &mz);
		mrot.Set_A_axis(mx,my,mz);
		mfr2.SetRot(mrot);
		mfr2.SetPos(mpt2);
	}
	else
	{
		// from abs to body-rel
		mN = this->Body1->Dir_World2Body(&mnorm1);
		mN.DirToDxDyDz(&mx, &my, &mz);
		mrot.Set_A_axis(mx,my,mz);
		mfr1.SetRot(mrot);
		mfr1.SetPos(this->Body1->Point_World2Body(&mpt1));

		mN = - this->Body2->Dir_World2Body(&mnorm2);
		mN.DirToDxDyDz(&mx, &my, &mz);
		mrot.Set_A_axis(mx,my,mz);
		mfr2.SetRot(mrot);
		mfr2.SetPos(this->Body2->Point_World2Body(&mpt2));
	}

	this->frameA = mfr1;
	this->frameB = mfr2;

	return true;
}



// 
// Following functions are for exploiting the contact persistence
//

void  ChLinkMateGeneric::ConstraintsLiLoadSuggestedSpeedSolution()
{
	int cnt=0;
	for (int i=0; i< mask->nconstr; i++)
	{
		if (mask->Constr_N(i).IsActive())
		{
			mask->Constr_N(i).Set_l_i( cache_li_speed->ElementN(cnt) );
			cnt++;
		}
	}
}

void  ChLinkMateGeneric::ConstraintsLiLoadSuggestedPositionSolution()
{
	int cnt=0;
	for (int i=0; i< mask->nconstr; i++)
	{
		if (mask->Constr_N(i).IsActive())
		{
			mask->Constr_N(i).Set_l_i( cache_li_pos->ElementN(cnt) ) ;
			cnt++;
		}
	}
}

void  ChLinkMateGeneric::ConstraintsLiFetchSuggestedSpeedSolution()
{
	int cnt=0;
	for (int i=0; i< mask->nconstr; i++)
	{
		if (mask->Constr_N(i).IsActive())
		{
			cache_li_speed->ElementN(cnt) = mask->Constr_N(i).Get_l_i();
			cnt++;
		}
	}
}

void  ChLinkMateGeneric::ConstraintsLiFetchSuggestedPositionSolution()
{
	int cnt=0;
	for (int i=0; i< mask->nconstr; i++)
	{
		if (mask->Constr_N(i).IsActive())
		{
			cache_li_pos->ElementN(cnt) = mask->Constr_N(i).Get_l_i();
			cnt++;
		}
	}
}





////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkMatePlane> a_registration_ChLinkMatePlane;


void ChLinkMatePlane::Copy(ChLinkMatePlane* source)
{
    // first copy the parent class data...
    //
    ChLinkMateGeneric::Copy(source);
	
	this->flipped = source->flipped;
	this->separation = source->separation;
}

ChLink* ChLinkMatePlane::new_Duplicate ()
{
    ChLinkMatePlane * m_l = new ChLinkMatePlane;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}



void ChLinkMatePlane::SetFlipped(bool doflip)
{
	if (doflip != this->flipped)
	{
		// swaps direction of X axis by flippping 180° the frame A (slave)

		ChFrame<> frameRotator(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y));
		this->frameA.ConcatenatePostTransformation(frameRotator);

		this->flipped = doflip;
	}
}


int ChLinkMatePlane::Initialize(ChSharedBodyPtr& mbody1,	///< first body to link
						   ChSharedBodyPtr& mbody2, ///< second body to link
						   bool pos_are_relative,	///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpt1,			///< point on slave plane, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpt2,  		///< point on master plane, for 2nd body (rel. or abs., see flag above)
						   ChVector<> mnorm1,		///< normal of slave plane, for 1st body (rel. or abs., see flag above)
						   ChVector<> mnorm2  		///< normal of master plane, for 2nd body (rel. or abs., see flag above)
						   )
{
	// set the two frames so that they have the X axis aligned when the
	// two normals are opposed (default behavior, otherwise is considered 'flipped')

	ChVector<> mnorm1_reversed;
	if (!this->flipped)
		mnorm1_reversed = -mnorm1;
	else
		mnorm1_reversed = mnorm1;

	return ChLinkMateGeneric::Initialize(mbody1, mbody2, 
								pos_are_relative, 
								mpt1, mpt2, 
								mnorm1_reversed, mnorm2);

}


/// Override _all_ time, jacobian etc. updating.
void ChLinkMatePlane::Update (double mtime)
{
	// Parent class inherit
	ChLinkMateGeneric::Update(mtime);

	// .. then add the effect of imposed distance on C residual vector
	this->C->Element(0,0) -= this->separation; // for this mate, C = {Cx, Cry, Crz}

}


} // END_OF_NAMESPACE____


