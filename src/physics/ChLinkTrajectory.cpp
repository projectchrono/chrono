///////////////////////////////////////////////////
//
//   ChLinkTrajectory.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 

#include "physics/ChLinkTrajectory.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). 
 

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

	space_fx = new ChFunction_Ramp(0, 1.);

	trajectory_line = 0;

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to X  only. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(true, true, true,
                       false, false, false, false,
                       false, false);

    ChangedLinkMask();
}


            // DESTROYER
ChLinkTrajectory::~ChLinkTrajectory ()
{
	if (space_fx) 
		delete space_fx;
	space_fx=0;

	if (trajectory_line) 
		delete trajectory_line;
	trajectory_line=0;

    // nothing..
}

void ChLinkTrajectory::Copy(ChLinkTrajectory* source)
{
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

	// copy own data

	if (space_fx) 
		delete space_fx;
    space_fx =   source->space_fx->new_Duplicate();

	if (trajectory_line) 
		delete trajectory_line;
    trajectory_line = static_cast<ChLine*> (source->trajectory_line->Duplicate());
}


ChLink* ChLinkTrajectory::new_Duplicate ()   
{
    ChLinkTrajectory* m_l;
    m_l = new ChLinkTrajectory;  
    m_l->Copy(this);
    return (m_l);
}



void ChLinkTrajectory::Set_space_fx (ChFunction* m_funct)
{
    if (space_fx) 
		delete space_fx;
    space_fx = m_funct;
}

void ChLinkTrajectory::Set_trajectory_line (ChLine* mline)
{
	if (trajectory_line) 
		delete trajectory_line;
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
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream.AbstractWrite(space_fx);
	mstream.AbstractWrite(trajectory_line);
}


void ChLinkTrajectory::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

		// stream in all member data
	ChFunction* ffoo=0;
	mstream.AbstractReadCreate(&ffoo);		Set_space_fx(ffoo);
	ChLine* mline=0;
	mstream.AbstractReadCreate(&mline);		Set_trajectory_line(mline);
}





///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


