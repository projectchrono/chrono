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

///////////////////////////////////////////////////
//
//   ChLinkPulley.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
 
#include "physics/ChLinkPulley.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

 
namespace chrono
{


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR PULLEYS
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkPulley> a_registration_ChLinkPulley;


ChLinkPulley::ChLinkPulley ()
{
    type = LNK_PULLEY;    // initializes type

    a1 = 0;
    a2 = 0;
    r1 = 1;
    r2 = 1;
    tau = 1;
    phase = 0;
    checkphase = FALSE;
	shaft_dist = 0;

	local_shaft1.SetIdentity();
	local_shaft2.SetIdentity();

    belt_up1= VNULL;
	belt_up2= VNULL;
	belt_low1= VNULL;
	belt_low2= VNULL;

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(true, false, false,
                       false, false, false, false,
                       false, false);
    ChangedLinkMask();
}

ChLinkPulley::~ChLinkPulley ()
{ }

void ChLinkPulley::Copy(ChLinkPulley* source)
{
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

    // copy custom data:
    tau = source->tau;
    phase = source->phase;
    a1 = source->a1;
    a2 = source->a2;
    checkphase = source->checkphase;
    r1 = source->r1;
    r2 = source->r2;
	belt_up1 = source->belt_up1;
	belt_up2 = source->belt_up2;
	belt_low1 = source->belt_low1;
	belt_low2 = source->belt_low2;
	local_shaft1 = source->local_shaft1;
	local_shaft2 = source->local_shaft2;
    shaft_dist = source->shaft_dist;
}

ChLink* ChLinkPulley::new_Duplicate ()
{
    ChLinkPulley* m_l;
    m_l = new ChLinkPulley;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}


Vector  ChLinkPulley::Get_shaft_dir1()
{
	if (this->Body1)
	{
		ChFrame<double> absframe;
		((ChFrame<double>*)Body1)->TransformLocalToParent(local_shaft1, absframe);
		return absframe.GetA()->Get_A_Zaxis();
	} else return VECT_Z;
}

Vector  ChLinkPulley::Get_shaft_dir2() 
{
	if (this->Body1)
	{
		ChFrame<double> absframe;
		((ChFrame<double>*)Body2)->TransformLocalToParent(local_shaft2, absframe);
		return absframe.GetA()->Get_A_Zaxis();
	} else return VECT_Z;
}

Vector  ChLinkPulley::Get_shaft_pos1() 
{
	if (this->Body1)
	{
		ChFrame<double> absframe;
		((ChFrame<double>*)Body1)->TransformLocalToParent(local_shaft1, absframe);
		return absframe.GetPos();
	} else return VNULL;
}

Vector  ChLinkPulley::Get_shaft_pos2() 
{
	if (this->Body1)
	{
		ChFrame<double> absframe;
		((ChFrame<double>*)Body2)->TransformLocalToParent(local_shaft2, absframe);
		return absframe.GetPos();
	} else return VNULL;
}




void ChLinkPulley::UpdateTime (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);

	ChFrame<double> abs_shaft1;
	ChFrame<double> abs_shaft2;

	((ChFrame<double>*)Body1)->TransformLocalToParent(local_shaft1, abs_shaft1);
	((ChFrame<double>*)Body2)->TransformLocalToParent(local_shaft2, abs_shaft2);

	ChVector<> dcc_w = Vsub(Get_shaft_pos2(),
                            Get_shaft_pos1());

		// compute actual rotation of the two wheels (relative to truss).
    Vector md1 = abs_shaft1.GetA()->MatrT_x_Vect(dcc_w);
    md1.z = 0;  md1 = Vnorm (md1);
    Vector md2 = abs_shaft2.GetA()->MatrT_x_Vect(dcc_w);
    md2.z = 0;  md2 = Vnorm (md2);

	double periodic_a1 = ChAtan2(md1.x, md1.y);
	double periodic_a2 = ChAtan2(md2.x, md2.y);
	double old_a1 = a1; 
	double old_a2 = a2;
	double turns_a1 = floor (old_a1 / CH_C_2PI);
	double turns_a2 = floor (old_a2 / CH_C_2PI);
	double a1U = turns_a1 * CH_C_2PI + periodic_a1 + CH_C_2PI;
	double a1M = turns_a1 * CH_C_2PI + periodic_a1;
	double a1L = turns_a1 * CH_C_2PI + periodic_a1 - CH_C_2PI;
	a1 = a1M;
	if (fabs(a1U - old_a1) < fabs(a1M - old_a1))
		a1 = a1U;
	if (fabs(a1L - a1) < fabs(a1M - a1))
		a1 = a1L;
	double a2U = turns_a2 * CH_C_2PI + periodic_a2 + CH_C_2PI;
	double a2M = turns_a2 * CH_C_2PI + periodic_a2;
	double a2L = turns_a2 * CH_C_2PI + periodic_a2 - CH_C_2PI;
	a2 = a2M;
	if (fabs(a2U - old_a2) < fabs(a2M - old_a2))
		a2 = a2U;
	if (fabs(a2L - a2) < fabs(a2M - a2))
		a2 = a2L;

	     // correct marker positions if phasing is not correct
	double m_delta =0;
    if (this->checkphase)
    {
		double realtau = tau; 
		//if (this->epicyclic) 
		//	realtau = -tau;
        
        m_delta = a1 - phase - (a2/realtau);

        if (m_delta> CH_C_PI) m_delta -= (CH_C_2PI);		 // range -180..+180 is better than 0...360
        if (m_delta> (CH_C_PI/4.0)) m_delta = (CH_C_PI/4.0); // phase correction only in +/- 45°
        if (m_delta<-(CH_C_PI/4.0)) m_delta =-(CH_C_PI/4.0);
		//***TODO***
    }


    // Move markers 1 and 2 to align them as pulley ends

	ChVector<> d21_w = dcc_w - Get_shaft_dir1()* Vdot (Get_shaft_dir1(), dcc_w);
	ChVector<> D21_w = Vnorm(d21_w);

	this->shaft_dist = d21_w.Length();
	
	ChVector<> U1_w = Vcross(Get_shaft_dir1(), D21_w);

	double gamma1 = acos( (r1-r2) / shaft_dist);

	ChVector<> Ru_w =  D21_w*cos(gamma1) + U1_w*sin(gamma1);
	ChVector<> Rl_w =  D21_w*cos(gamma1) - U1_w*sin(gamma1);

	this->belt_up1  = Get_shaft_pos1()+ Ru_w*r1;
	this->belt_low1 = Get_shaft_pos1()+ Rl_w*r1;
	this->belt_up2  = Get_shaft_pos1()+ d21_w + Ru_w*r2;
	this->belt_low2 = Get_shaft_pos1()+ d21_w + Rl_w*r2;

		// marker alignment
	ChMatrix33<> maU;
	ChMatrix33<> maL;

	ChVector<> Dxu = Vnorm(belt_up2 - belt_up1);
	ChVector<> Dyu = Ru_w;
	ChVector<> Dzu = Vnorm (Vcross(Dxu, Dyu));
	Dyu = Vnorm (Vcross(Dzu, Dxu));
	maU.Set_A_axis(Dxu,Dyu,Dzu);

            // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);
    marker1->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

	ChCoordsys<> newmarkpos;

        // move marker1 in proper positions
    newmarkpos.pos = this->belt_up1;
    newmarkpos.rot = maU.Get_A_quaternion();
    marker1->Impose_Abs_Coord(newmarkpos);        //move marker1 into teeth position
        // move marker2 in proper positions
    newmarkpos.pos = this->belt_up2;
    newmarkpos.rot = maU.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);        //move marker2 into teeth position

	double phase_correction_up = m_delta*r1;
	double phase_correction_low = - phase_correction_up;
	double hU = Vlength(belt_up2- belt_up1)  + phase_correction_up;
	double hL = Vlength(belt_low2- belt_low1) + phase_correction_low;

        // imposed relative positions/speeds
    deltaC.pos = ChVector<>(-hU, 0, 0);
    deltaC_dt.pos = VNULL;
    deltaC_dtdt.pos = VNULL;

    deltaC.rot = QUNIT;             // no relative rotations imposed!
    deltaC_dt.rot = QNULL;
    deltaC_dtdt.rot = QNULL;
}



void ChLinkPulley::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream << tau;
    mstream << phase;
    mstream << checkphase;
    mstream << a1;
    mstream << a2;
    mstream << r1;
    mstream << r2;
	mstream << local_shaft1;
	mstream << local_shaft2;
}

void ChLinkPulley::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

		// stream in all member data
	mstream >> tau;
    mstream >> phase;
    mstream >> checkphase;
    mstream >> a1;
    mstream >> a2;
    mstream >> r1;
    mstream >> r2;
	mstream >> local_shaft1;
	mstream >> local_shaft2;
}






///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


