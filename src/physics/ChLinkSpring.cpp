///////////////////////////////////////////////////
//
//   ChLinkSpring.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
  
 
#include "physics/ChLinkSpring.h"
#include "core/ChMemory.h" // must be last include (memory leak debugger). 


namespace chrono
{




////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
//
//   CLASS FOR SPRING LINKS
//
//

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLinkSpring> a_registration_ChLinkSpring;


                // BUILDERS
ChLinkSpring::ChLinkSpring ()
{
    spr_restlenght = 0;     // custom variables
    spr_k = 100;
    spr_r = 5;
    spr_f = 0;

    mod_f_time =   new ChFunction_Const (1);
    mod_k_d =   new ChFunction_Const (1);
    mod_k_speed =   new ChFunction_Const (1);
    mod_r_d =   new ChFunction_Const (1);
    mod_r_speed =   new ChFunction_Const (1);

    spr_react = 0.0;
}


            // DESTROYER
ChLinkSpring::~ChLinkSpring ()
{
    // delete custom instanced data...
    if (mod_f_time) delete mod_f_time;
    if (mod_k_d) delete mod_k_d;
    if (mod_k_speed) delete mod_k_speed;
    if (mod_r_d) delete mod_r_d;
    if (mod_r_speed) delete mod_r_speed;
}


void ChLinkSpring::Copy(ChLinkSpring* source)
{
    // first copy the parent class data...
    //
    ChLinkMarkers::Copy(source);

    // copy custom data:
    spr_restlenght = source->spr_restlenght;
    spr_f = source->spr_f;
    spr_k = source->spr_k;
    spr_r = source->spr_r;
    spr_react = source->spr_react;
    if (mod_f_time) delete mod_f_time;
         mod_f_time =   source->mod_f_time->new_Duplicate();
    if (mod_k_d) delete mod_k_d;
         mod_k_d =   source->mod_k_d->new_Duplicate();
    if (mod_k_speed) delete mod_k_speed;
         mod_k_speed =   source->mod_k_speed->new_Duplicate();
    if (mod_r_d) delete mod_r_d;
         mod_r_d =   source->mod_r_d->new_Duplicate();
    if (mod_r_speed) delete mod_r_speed;
         mod_r_speed =   source->mod_r_speed->new_Duplicate();
}


ChLink* ChLinkSpring::new_Duplicate ()
{
    ChLinkSpring* m_l;
    m_l = new ChLinkSpring;  // inherited classes should write here: m_l = new MyInheritedLink;
    m_l->Copy(this);
    return (m_l);
}




int ChLinkSpring::Initialize(ChSharedPtr<ChBody>& mbody1,   ///< first body to link
						   ChSharedPtr<ChBody>& mbody2,		 ///< second body to link
						   bool pos_are_relative,///< true: following posit. are considered relative to bodies. false: pos.are absolute
						   ChVector<> mpos1,	 ///< position of distance endpoint, for 1st body (rel. or abs., see flag above)
						   ChVector<> mpos2,	 ///< position of distance endpoint, for 2nd body (rel. or abs., see flag above) 
						   bool auto_rest_length,///< if true, initializes the imposed distance as the distance between mpos1 and mpos2
						   double mrest_length   ///< imposed distance (no need to define, if auto_distance=true.)
						   )
{
	// first, initialize as all constraint with markers. In this case, create the two markers also!.
	ChLinkMarkers::Initialize(mbody1,mbody2, CSYSNORM);

	if (pos_are_relative)
	{
		this->marker1->Impose_Rel_Coord(ChCoordsys<>(mpos1,QUNIT));
		this->marker2->Impose_Rel_Coord(ChCoordsys<>(mpos2,QUNIT));
	}
	else
	{
		this->marker1->Impose_Abs_Coord(ChCoordsys<>(mpos1,QUNIT));
		this->marker2->Impose_Abs_Coord(ChCoordsys<>(mpos2,QUNIT));
	}
	 
	ChVector<> AbsDist = this->marker1->GetAbsCoord().pos - this->marker2->GetAbsCoord().pos;
	this->dist = AbsDist.Length();

	if (auto_rest_length)
	{
		this->spr_restlenght = this->dist;
	}
	else
	{
		this->spr_restlenght = mrest_length; 
	}

	return true;
}






void ChLinkSpring::Set_mod_f_time (ChFunction* m_funct)
{
    if (mod_f_time) delete mod_f_time;
    mod_f_time = m_funct;
}
void ChLinkSpring::Set_mod_k_d    (ChFunction* m_funct)
{
    if (mod_k_d) delete mod_k_d;
    mod_k_d = m_funct;
}
void ChLinkSpring::Set_mod_r_d    (ChFunction* m_funct)
{
    if (mod_r_d) delete mod_r_d;
    mod_r_d = m_funct;
}
void ChLinkSpring::Set_mod_k_speed(ChFunction* m_funct)
{
    if (mod_k_speed) delete mod_k_speed;
    mod_k_speed = m_funct;
}
void ChLinkSpring::Set_mod_r_speed(ChFunction* m_funct)
{
    if (mod_r_speed) delete mod_r_speed;
    mod_r_speed = m_funct;
}


void ChLinkSpring::UpdateForces (double mytime)
{
    // Inherit force computation:
    // also base class can add its own forces.
    ChLinkMarkers::UpdateForces(mytime);

    spr_react = 0.0;
    Vector m_force;
    double deform = Get_SpringDeform();

    spr_react  =  spr_f * mod_f_time->Get_y(ChTime);
    spr_react -= (spr_k * mod_k_d->Get_y(deform) * mod_k_speed->Get_y(dist_dt)) * (deform);
    spr_react -= (spr_r * mod_r_d->Get_y(deform) * mod_r_speed->Get_y(dist_dt)) * (dist_dt);

    m_force = Vmul (Vnorm(relM.pos), spr_react);

    C_force = Vadd(C_force, m_force);  
}



void ChLinkSpring::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChLinkMarkers::StreamOUT(mstream);

		// stream out all member data
	mstream << spr_restlenght;
    mstream << spr_f;
    mstream << spr_k;
    mstream << spr_r;
	mstream.AbstractWrite(mod_f_time);
	mstream.AbstractWrite(mod_k_d);
    mstream.AbstractWrite(mod_k_speed);
    mstream.AbstractWrite(mod_r_d);
    mstream.AbstractWrite(mod_r_speed);
}

void ChLinkSpring::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChLinkMarkers::StreamIN(mstream);

		// stream in all member data
	ChFunction* ffoo;
	mstream >> spr_restlenght;
    mstream >> spr_f;
    mstream >> spr_k;
    mstream >> spr_r;
	mstream.AbstractReadCreate(&ffoo);		Set_mod_f_time(ffoo);
	mstream.AbstractReadCreate(&ffoo);		Set_mod_k_d(ffoo);
    mstream.AbstractReadCreate(&ffoo);		Set_mod_k_speed(ffoo);
    mstream.AbstractReadCreate(&ffoo);		Set_mod_r_d(ffoo);
    mstream.AbstractReadCreate(&ffoo);		Set_mod_r_speed(ffoo);
}





///////////////////////////////////////////////////////////////



} // END_OF_NAMESPACE____


