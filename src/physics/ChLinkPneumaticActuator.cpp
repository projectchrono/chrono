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
//   ChLinkPneumaticActuator.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
 
#include "physics/ChLinkPneumaticActuator.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.


namespace chrono
{



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkPneumaticActuator> a_registration_ChLinkPneumaticActuator;


ChLinkPneumaticActuator::ChLinkPneumaticActuator ()
{
    type = LNK_PNEUMATIC;       // initializes type

    pneuma = new pneumatics::AssePneumatico();
    offset = pneuma->Get_L() + 0.1;

    pA = pB = pneuma->Get_Ps(); // default state (initial chamber pressure = ambient pressure)
    pA_dt = pB_dt = 0;
    pneu_F = 0;
    last_force_time = 0;

            // Mask: initialize our LinkMaskLF (lock formulation mask)
            // to free. It was a LinkMaskLF because this class inherited from LinkLock.
    ((ChLinkMaskLF*)mask)->SetLockMask(false, false, false,
                       false, false, false, false,
                       false, false);

            // set upper lower limits, active
    limit_X->Set_active(TRUE);
    limit_X->Set_min( offset);// 0.0);
    limit_X->Set_max(pneuma->Get_L()+offset);
    limit_X->Set_maxElastic(0.0);
    limit_X->Set_minElastic(0.0);

    ChangedLinkMask();
}

ChLinkPneumaticActuator::~ChLinkPneumaticActuator ()
{
    if (pneuma) delete pneuma;
    pneuma = NULL;
}

void ChLinkPneumaticActuator::Copy(ChLinkPneumaticActuator* source)
{
    // first copy the parent class data...
    //

    ChLinkLock::Copy(source);

    // copy custom data:
    pneuma->Set_Ci( source->pneuma->Get_Ci() );
    pneuma->Set_Co( source->pneuma->Get_Co() );
    pneuma->Set_Bi( source->pneuma->Get_Bi() );
    pneuma->Set_Bo( source->pneuma->Get_Bo() );
    pneuma->Set_Ps( source->pneuma->Get_Ps() );
    pneuma->Set_Pma( source->pneuma->Get_Pma() );
    pneuma->Set_Pmb( source->pneuma->Get_Pmb() );
    pneuma->Set_L( source->pneuma->Get_L() );
    pneuma->Set_Wa( source->pneuma->Get_Wa() );
    pneuma->Set_Wb( source->pneuma->Get_Wb() );
    pneuma->Set_A( source->pneuma->Get_A() );
    pneuma->Set_Alfa( source->pneuma->Get_Alfa() );
    pneuma->Set_Gamma( source->pneuma->Get_Gamma() );
    pneuma->Set_ValvA_min( source->pneuma->Get_ValvA_min() );
    pneuma->Set_ValvA_max( source->pneuma->Get_ValvA_max() );
    pneuma->Set_ValvA_close( source->pneuma->Get_ValvA_close() );
    pneuma->Set_ValvB_min( source->pneuma->Get_ValvB_min() );
    pneuma->Set_ValvB_max( source->pneuma->Get_ValvB_max() );
    pneuma->Set_ValvB_close( source->pneuma->Get_ValvB_close() );
    pneuma->SetupAssePneumatico(); // setup into sub objects
    this->offset = source->offset;
    this->pA = source->pA;
    this->pB = source->pB;
    this->pA_dt = source->pA_dt;
    this->pB_dt = source->pB_dt;
    pneuma->Set_P(source->pA, source->pB); // this also copies state into internal structures
    pneuma->Set_Pos(source->Get_pneu_pos(), source->Get_pneu_pos_dt() );
    this->pneu_F = source->pneu_F;
    this->last_force_time = source->last_force_time;
}


void ChLinkPneumaticActuator::Set_lin_offset(double mset)
{
    offset = mset;
    limit_X->Set_min(offset);
    limit_X->Set_max(pneuma->Get_L()+offset);
}

void ChLinkPneumaticActuator::Set_pneu_L(double mset)
{
    pneuma->Set_L(mset);
    limit_X->Set_max(pneuma->Get_L()+offset);
}


ChLink* ChLinkPneumaticActuator::new_Duplicate ()
{
    ChLinkPneumaticActuator* m_l;
    m_l = new ChLinkPneumaticActuator;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}

void ChLinkPneumaticActuator::UpdateTime (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateTime(mytime);

        // Move (well, rotate...) marker 2 to align it in actuator direction

        // ! Require that the BDF routine of marker won't handle speed and acc.calculus of the moved marker 2!
    marker2->SetMotionType(ChMarker::M_MOTION_EXTERNAL);

    ChMatrix33<> ma;
    ma.Set_A_quaternion(marker2->GetAbsCoord().rot);

    Vector absdist = Vsub(marker1->GetAbsCoord().pos,
                          marker2->GetAbsCoord().pos);

    Vector mx = Vnorm (absdist);

    Vector my = ma.Get_A_Yaxis();
    if (Vequal(mx, my))
        if  (mx.x == 1.0)   my = VECT_Y;
        else                my = VECT_X;
    Vector mz = Vnorm (Vcross(mx, my));
    my = Vnorm (Vcross(mz, mx));

    ma.Set_A_axis(mx,my,mz);

    Coordsys newmarkpos;
    newmarkpos.pos = marker2->GetAbsCoord().pos;
    newmarkpos.rot = ma.Get_A_quaternion();
    marker2->Impose_Abs_Coord(newmarkpos);        //rotate "main" marker2 into tangent position

}

void ChLinkPneumaticActuator::UpdateForces (double mytime)
{
    // First, inherit to parent class
    ChLinkLock::UpdateForces(mytime);

    // DEFAULTS set null forces
    this->pneu_F = 0;

    // COMPUTE PNEUMATIC FORCE!!

        // 1a - set current state (pressure A and B)
    this->pneuma->Set_P(this->pA, this->pB);
        // 1b - set current state (position, speed)
    this->pneuma->Set_Pos(this->Get_pneu_pos(), this->Get_pneu_pos_dt() );

        // 2- compute new force for this state
    this->pneuma->Update(); // needed, so that F is computed in pneuma*
    this->pneu_F = this->pneuma->Get_F();

    // Security clamping on plausible limit, to avoid divergence
    //if (this->pneu_F > 100000) this->pneu_F = 100000;


        // 3- compute new pressures by 'local' integration, from previous
        //    values of pressures.
    this->pneuma->Get_P_dt(&this->pA_dt, &this->pB_dt);

    double mforce_timestep = mytime - this->last_force_time;
    if ( (mforce_timestep < 0.1) &&
         (mforce_timestep >0) )
    {
        this->pA = this->pA + mforce_timestep * this->pA_dt;
        this->pB = this->pB + mforce_timestep * this->pB_dt;

        if (this->pA < 0) this->pA = 0;
        if (this->pB < 0) this->pB = 0;

        this->pneuma->Set_P(this->pA, this->pB);
    }

    this->last_force_time = mytime;


    // +++ADD PNEUMATIC FORCE TO LINK INTERNAL FORCE VECTOR (on x axis only)

    C_force.x = C_force.x + this->pneu_F;
}



void ChLinkPneumaticActuator::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkLock::StreamOUT(mstream);

		// stream out all member data
	mstream << pneuma->Get_Ci();
    mstream << pneuma->Get_Co();
    mstream << pneuma->Get_Bi();
    mstream << pneuma->Get_Bo();
    mstream << pneuma->Get_Ps();
    mstream << pneuma->Get_Pma();
    mstream << pneuma->Get_Pmb();
    mstream << pneuma->Get_L();
    mstream << pneuma->Get_Wa();
    mstream << pneuma->Get_Wb();
    mstream << pneuma->Get_A();
    mstream << pneuma->Get_Alfa();
    mstream << pneuma->Get_Gamma();
    mstream << pneuma->Get_ValvA_min();
    mstream << pneuma->Get_ValvA_max();
    mstream << pneuma->Get_ValvA_close();
    mstream << pneuma->Get_ValvB_min();
    mstream << pneuma->Get_ValvB_max();
    mstream << pneuma->Get_ValvB_close();
    mstream << pA;
    mstream << pB;
    mstream << pA_dt;
    mstream << pB_dt;
    mstream << pneu_F;
    mstream << offset;
}

void ChLinkPneumaticActuator::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkLock::StreamIN(mstream);

		// stream in all member data
	double dfoo;
	mstream >> dfoo;	pneuma->Set_Ci(dfoo);
    mstream >> dfoo;	pneuma->Set_Co(dfoo);
    mstream >> dfoo;	pneuma->Set_Bi(dfoo);
    mstream >> dfoo;	pneuma->Set_Bo(dfoo);
    mstream >> dfoo;	pneuma->Set_Ps(dfoo);
    mstream >> dfoo;	pneuma->Set_Pma(dfoo);
    mstream >> dfoo;	pneuma->Set_Pmb(dfoo);
    mstream >> dfoo;	pneuma->Set_L(dfoo);
    mstream >> dfoo;	pneuma->Set_Wa(dfoo);
    mstream >> dfoo;	pneuma->Set_Wb(dfoo);
    mstream >> dfoo;	pneuma->Set_A(dfoo);
    mstream >> dfoo;	pneuma->Set_Alfa(dfoo);
    mstream >> dfoo;	pneuma->Set_Gamma(dfoo);
    mstream >> dfoo;	pneuma->Set_ValvA_min(dfoo);
    mstream >> dfoo;	pneuma->Set_ValvA_max(dfoo);
    mstream >> dfoo;	pneuma->Set_ValvA_close(dfoo);
    mstream >> dfoo;	pneuma->Set_ValvB_min(dfoo);
    mstream >> dfoo;	pneuma->Set_ValvB_max(dfoo);
    mstream >> dfoo;	pneuma->Set_ValvB_close(dfoo);
    mstream >> pA;
    mstream >> pB;
    mstream >> pA_dt;
    mstream >> pB_dt;
    mstream >> pneu_F;
    mstream >> dfoo;	Set_lin_offset(dfoo);
}




///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


