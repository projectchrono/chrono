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
//   ChLinkTrajectory.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 

#include "physics/ChLinkTrajectory.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). 
#include "geometry/ChCLineSegment.h"

namespace chrono
{

 

using namespace geometry;



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkTrajectory> a_registration_ChLinkTrajectory;


                // BUILDERS
ChLinkTrajectory::ChLinkTrajectory ()
{
    type = LNK_TRAJECTORY;       // initializes type

	// default s(t) function. User will provide better fx.
	space_fx = ChSharedPtr<ChFunction> (new ChFunction_Ramp(0, 1.));

	// default trajectory is a segment
	trajectory_line = ChSharedPtr<ChLine> (new ChLineSegment());


            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(true, true, true,
                       false, false, false, false);

    ChangedLinkMask();
}


            // DESTROYER
ChLinkTrajectory::~ChLinkTrajectory ()
{

    // nothing..
}

void ChLinkTrajectory::Copy(ChLinkTrajectory* source)
{
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

	// copy own data

	space_fx = ChSharedPtr<ChFunction>(source->space_fx->new_Duplicate()); // deep copy

	trajectory_line = ChSharedPtr<ChLine>((ChLine*)source->trajectory_line->Duplicate()); // deep copy
}


ChLink* ChLinkTrajectory::new_Duplicate ()   
{
    ChLinkTrajectory* m_l;
    m_l = new ChLinkTrajectory;  
    m_l->Copy(this);
    return (m_l);
}



void ChLinkTrajectory::Set_space_fx (ChSharedPtr<ChFunction> m_funct)
{
    space_fx = m_funct;
}

void ChLinkTrajectory::Set_trajectory_line (ChSharedPtr<geometry::ChLine> mline)
{
    trajectory_line = mline;
}



/////////    UPDATE TIME
/////////

void ChLinkTrajectory::UpdateTime (double time)
{
    ChTime = time;

	double tr_time = space_fx->Get_y(time);

	//R3OBJ* markerobj = ((ChExternalObjectR3D*)marker2->GetExternalObject())->Get_r3d_object();
    //R3OBJ* lineobj = R3SendMsgA(markerobj, R3LEVM_GETSUBBYORDNUM, (void *)0);
    if (trajectory_line)
    {
        Vector param, result, resultB, resultA;
        double tstep = BDF_STEP_HIGH;
        double timeB = tr_time + tstep;
        double timeA = tr_time - tstep;
        param.y= 0; param.z= 0;
        param.x= fmod(tr_time, 1);
		trajectory_line->Evaluate(result, param.x);
        param.x= fmod(timeB, 1);
		trajectory_line->Evaluate(resultB, param.x);
        param.x= fmod(timeA, 1);
		trajectory_line->Evaluate(resultA, param.x);

        ChMatrix33<> mw;
        mw.Set_A_quaternion(marker2->GetAbsCoord().rot);

        deltaC.pos = mw.MatrT_x_Vect(
                            Vsub (result, marker2->GetAbsCoord().pos));  // ***  CORRECT?
        deltaC_dt.pos =  mw.MatrT_x_Vect(
                            Vmul( Vsub(resultB, resultA), 1/(timeB-timeA)) );
        deltaC_dtdt.pos =  mw.MatrT_x_Vect (
                            Vmul   ( Vadd (Vadd (resultA, resultB),
                                   Vmul (result,-2)), 4/pow(timeB-timeA, 2) ) );
        deltaC.rot = QUNIT;
        deltaC_dt.rot = QNULL;
        deltaC_dtdt.rot = QNULL;
    }
	else
	{
		GetLog() << "NO TRAJECTORY \n";
	}


}



void ChLinkTrajectory::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(2);
		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream.AbstractWrite(this->space_fx.get_ptr()); //***TODO*** proper serialize for ChSharedPtr
	mstream.AbstractWrite(this->trajectory_line.get_ptr()); //***TODO*** proper serialize for ChSharedPtr
	mstream << modulo_s;
}


void ChLinkTrajectory::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

		// stream in all member data
	ChFunction* newfun;
	mstream.AbstractReadCreate(&newfun); //***TODO*** proper deserialize for ChSharedPtr
	this->space_fx = ChSharedPtr<ChFunction>(newfun);

	ChLine* mline=0;
	mstream.AbstractReadCreate(&mline); //***TODO*** proper deserialize for ChSharedPtr
	this->trajectory_line = ChSharedPtr<ChLine>(mline);

	mstream >> modulo_s;
}





///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


