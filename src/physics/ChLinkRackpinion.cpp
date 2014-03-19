//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

 
 
#include "physics/ChLinkRackpinion.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

 
namespace chrono
{



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkRackpinion> a_registration_ChLinkRackpinion;


ChLinkRackpinion::ChLinkRackpinion ()
		: ChLinkMateGeneric(true, false, false, false, false, false)
{
    R = 0.1;
    alpha = 0;
    beta = 0;
    phase = 0;
    checkphase = false;
    a1 = 0;

	local_pinion.SetIdentity();
	local_rack.SetIdentity();

    contact_pt = VNULL;
}

ChLinkRackpinion::~ChLinkRackpinion ()
{ }

void ChLinkRackpinion::Copy(ChLinkRackpinion* source)
{
    // first copy the parent class data...
    //
    ChLinkMateGeneric::Copy(source);

    // copy custom data:
    R = source->R;
    alpha = source->alpha;
    beta = source->beta;
    phase = source->phase;
    a1 = source->a1;
    checkphase = source->checkphase;

	contact_pt = source->contact_pt;
	local_pinion = source->local_pinion;
	local_rack = source->local_rack;
}

ChLink* ChLinkRackpinion::new_Duplicate ()
{
    ChLinkRackpinion* m_l = new ChLinkRackpinion;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}


ChVector<>  ChLinkRackpinion::GetAbsPinionDir()
{
	if (this->Body1)
	{
		ChFrame<double> absframe;
		((ChFrame<double>*)Body1)->TrasformLocalToParent(local_pinion, absframe);
		return absframe.GetA()->Get_A_Zaxis();
	} else return VECT_Z;
}

ChVector<>  ChLinkRackpinion::GetAbsPinionPos() 
{
	if (this->Body1)
	{
		ChFrame<double> absframe;
		((ChFrame<double>*)Body1)->TrasformLocalToParent(local_pinion, absframe);
		return absframe.GetPos();
	} else return VNULL;
}

ChVector<> ChLinkRackpinion::GetAbsRackDir() 
{
	if (this->Body2)
	{
		ChFrame<double> absframe;
		((ChFrame<double>*)Body2)->TrasformLocalToParent(local_rack, absframe);
		return absframe.GetA()->Get_A_Zaxis();
	} else return VECT_Z;
}

ChVector<>  ChLinkRackpinion::GetAbsRackPos() 
{
	if (this->Body2)
	{
		ChFrame<double> absframe;
		((ChFrame<double>*)Body2)->TrasformLocalToParent(local_rack, absframe);
		return absframe.GetPos();
	} else return VNULL;
}




void ChLinkRackpinion::UpdateTime (double mytime)
{
    // First, inherit to parent class
    ChLinkMateGeneric::UpdateTime(mytime);

	ChFrame<double> abs_pinion;
	ChFrame<double> abs_rack;

	((ChFrame<double>*)Body1)->TrasformLocalToParent(local_pinion, abs_pinion);
	((ChFrame<double>*)Body2)->TrasformLocalToParent(local_rack, abs_rack);

	ChVector<> abs_distpr = abs_pinion.GetPos() - abs_rack.GetPos();
	ChVector<> abs_Dpin = abs_pinion.GetA()->Get_A_Zaxis();
	ChVector<> abs_Dx;
	ChVector<> abs_Dy;
	ChVector<> abs_Dz; 
	abs_Dpin.DirToDxDyDz(abs_Dz, abs_Dx, abs_Dy, abs_rack.GetA()->Get_A_Xaxis() ); // with z as pinion shaft and x as suggested rack X dir

	/*
	GetLog() << "abs_distpr " << abs_distpr << "\n";
	GetLog() << "abs_rack Xaxis()" << abs_rack.GetA()->Get_A_Xaxis() << "\n";
	GetLog() << "abs_Dpin " << abs_Dpin << "\n";
	GetLog() << "abs_Dx " << abs_Dx << "\n";
	*/

	ChVector<> abs_Ro = abs_Dy * Vdot(abs_Dy, abs_distpr);
	
	if ( Vdot(abs_Ro, abs_distpr) > 0)
		abs_Ro = -abs_Ro;

	ChVector<> abs_Dr = abs_Ro.GetNormalized();
	ChVector<> abs_R  = abs_Dr * this->GetPinionRadius();
	this->contact_pt  = abs_pinion.GetPos() + abs_R;

	double runX = Vdot( abs_distpr, abs_Dx );

	//ChVector<> abs_runX    = Vdot( abs_distpr, abs_Dx );
	
	// Absolute frame of link reference
	ChMatrix33<> ma1;
	ma1.Set_A_axis(abs_Dx, abs_Dy, abs_Dz);
	ChFrame<> abs_contact(this->contact_pt, ma1);

	ChMatrix33<> mrot;

	// rotate link frame on its Y because of beta
    mrot.Set_A_Rxyz( ChVector<>(0,this->beta,0) );
	ChFrame<> mrotframe(VNULL, mrot);
	abs_contact.ConcatenatePostTransformation ( mrotframe ); // or: abs_contact *= mrotframe;

	// rotate link frame on its Z because of alpha
	if (this->react_force.x < 0)
		mrot.Set_A_Rxyz( ChVector<>(0,0, this->alpha) );
	else
		mrot.Set_A_Rxyz( ChVector<>(0,0,-this->alpha) );
	mrotframe.SetRot(mrot);
	abs_contact.ConcatenatePostTransformation ( mrotframe ); // or: abs_contact *= mrotframe;

	// Set the link frame 'abs_contact' to relative frames to the two connected ChBodyFrame
	((ChFrame<double>*)Body1)->TrasformParentToLocal(abs_contact, this->frame1);
	((ChFrame<double>*)Body2)->TrasformParentToLocal(abs_contact, this->frame2);
}



void ChLinkRackpinion::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkMateGeneric::StreamOUT(mstream);

		// stream out all member data
	mstream << R;
    mstream << alpha;
    mstream << beta;
    mstream << phase;
    mstream << checkphase;
    mstream << a1;
	mstream << local_pinion;
	mstream << local_rack;
}

void ChLinkRackpinion::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkMateGeneric::StreamIN(mstream);

		// stream in all member data
	mstream >> R;
    mstream >> alpha;
    mstream >> beta;
    mstream >> phase;
    mstream >> checkphase;
    mstream >> a1;
	mstream >> local_pinion;
	mstream >> local_rack;
}






///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


