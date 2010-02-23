///////////////////////////////////////////////////
//
//   ChLink.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include "physics/ChLinkBrake.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). 
 

namespace chrono
{


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR BRAKE LINK
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkBrake> a_registration_ChLinkBrake;


ChLinkBrake::ChLinkBrake ()
{
    type = LNK_BRAKE;       // initializes type

    brake_torque = 0.0;
    stick_ratio = 1.1;

    brake_mode = BRAKE_ROTATION;

    last_dir = 0;
    must_stick = FALSE;

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false,
                       false, false, false, false,
                       false, false);

    ChangedLinkMask();
}

ChLinkBrake::~ChLinkBrake ()
{
    // ..
}

void ChLinkBrake::Copy(ChLinkBrake* source)
{
    // first copy the parent class data...
    //
    ChLinkLock::Copy(source);

    // copy custom data:
    brake_torque = source->brake_torque;
    stick_ratio  = source->stick_ratio;
    brake_mode  = source->brake_mode;

    last_dir = source->last_dir;
    must_stick = source->must_stick;
}

ChLink* ChLinkBrake::new_Duplicate ()
{
    ChLinkBrake* m_l;
    m_l = new ChLinkBrake;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkBrake::Set_brake_mode(int mmode)
{
    if (mmode != brake_mode)
    {
        brake_mode = mmode;

        // reset mask for default free brake
		((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);
        ((ChLinkMaskLF*)mask)->Constr_X().SetMode(CONSTRAINT_FREE);

        ChangedLinkMask();
    }
}

void ChLinkBrake::SetDisabled(bool mdis)
{
	ChLinkLock::SetDisabled(mdis);

	((ChLinkMaskLF*)mask)->Constr_E3().SetMode(CONSTRAINT_FREE);
    ((ChLinkMaskLF*)mask)->Constr_X().SetMode(CONSTRAINT_FREE);

    ChangedLinkMask();
}

// UPDATING ///



// Update time: just change internal time, do not let parent class modify deltaC !

void ChLinkBrake::UpdateTime (double time)
{
    ChTime = time;
}

// Update forces: if not sticked, apply torque

void ChLinkBrake::UpdateForces (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateForces(mytime);

	if (this->IsDisabled()) return;

    // then, if not sticking,
    if (this->brake_torque)
    {
        if (brake_mode == BRAKE_ROTATION)
        {
			if ( ((ChLinkMaskLF*)mask)->Constr_E3().IsActive() == false)
            {
                int mdir;

                Vector mv_torque = Vmul(VECT_Z, this->brake_torque);
                mdir = 0;   // clockwise torque

                if (Vdot(this->relWvel, mv_torque) > 0.0)
                {
                    mv_torque = Vmul (mv_torque, -1.0);         // keep torque always opposed to ang speed.
                    mdir = 1;   // counterclockwise torque
                }

                if (mdir != this->last_dir)
                    this->must_stick = TRUE;
                this->last_dir = mdir;

                // +++ADD TO LINK TORQUE VECTOR
                C_torque = Vadd(C_torque, mv_torque);
            }
        }
        if (brake_mode == BRAKE_TRANSLATEX)
        {
			if ( ((ChLinkMaskLF*)mask)->Constr_X().IsActive() == false)
            {
                int mdir;

                Vector mv_force = Vmul(VECT_X, this->brake_torque);
                mdir = 0;       // F-->  rear motion: frontfacing break force

                if (this->relM_dt.pos.x > 0.0)
                {
                    mv_force = Vmul (mv_force, -1.0);    // break force always opposed to speed
                    mdir = 1;   // F<-- backfacing breakforce for front motion
                }

                if (mdir != this->last_dir)
                    this->must_stick = TRUE;
                this->last_dir = mdir;

                // +++ADD TO LINK TORQUE VECTOR
                C_force = Vadd(C_force, mv_force);
            }
        }
    }

    // turn off sticking feature if stick ration not > 1.0
    if (this->stick_ratio <= 1.0)
        must_stick = FALSE;
}



// FILE I/O

void ChLinkBrake::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream << brake_torque;
    mstream << stick_ratio;
    mstream << brake_mode;
}

void ChLinkBrake::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

		// stream in all member data
	mstream >> brake_torque;
    mstream >> stick_ratio;
    mstream >> brake_mode;
}





///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


