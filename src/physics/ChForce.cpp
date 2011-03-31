///////////////////////////////////////////////////
//
//   ChForce.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include <math.h>

#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "physics/ChForce.h"
#include "physics/ChBody.h"
#include "physics/ChExternalObject.h"
 

namespace chrono
{

//////////////////////////////////////
//////////////////////////////////////

// CLASS FOR FORCES


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChForce> a_registration_ChForce;



///////////////////////////////////////

ChForce::ChForce()
{
	Body = NULL;

	Qf = new ChMatrixDynamic<double>(BODY_QDOF,1);
	vpoint = VNULL;
	vrelpoint = VNULL;
	force = VNULL;
	relforce = VNULL;
	vdir   = VECT_X;
	vreldir = VECT_X;
	restpos = VNULL;
	mforce = 0;
	modula =   new ChFunction_Const (1);

	align = FDIR_BODY;
	frame = FPOS_BODY;
	mode = FTYPE_FORCE;

	move_x = new ChFunction_Const (0);
	move_y = new ChFunction_Const (0);
	move_z = new ChFunction_Const (0);
	f_x = new ChFunction_Const (0);
	f_y = new ChFunction_Const (0);
	f_z = new ChFunction_Const (0);

	ChTime = 0;
}

ChForce::~ChForce ()
{
	delete Qf;
	if (modula) delete modula;
	if (move_x) delete move_x;
	if (move_y) delete move_y;
	if (move_z) delete move_z;
	if (f_x) delete f_x;
	if (f_y) delete f_y;
	if (f_z) delete f_z;
}

void ChForce::Copy(ChForce* source)
{
	// first copy the parent class data...
	ChObj::Copy(source);

	Body = source->Body;

	mforce = source->mforce;
	force = source->force;
	relforce = source->relforce;
	vdir   = source->vdir;
	vreldir = source->vreldir;
	vpoint = source->vpoint;
	vrelpoint = source->vrelpoint;
	restpos = source->restpos;
	align = source->align;
	frame = source->frame;
	mode = source->mode;

	ChTime = source->ChTime;

	Qf->CopyFromMatrix(*source->Qf);

	if (modula) delete modula;
	if (source->modula) modula = source->modula->new_Duplicate();
		else modula = NULL;
	if (move_x) delete move_x;
	if (source->move_x) move_x = source->move_x->new_Duplicate();
		else move_x = NULL;
	if (move_y) delete move_y;
	if (source->move_y) move_y = source->move_y->new_Duplicate();
		else move_y = NULL;
	if (move_z) delete move_z;
	if (source->move_z) move_z = source->move_z->new_Duplicate();
		else move_z = NULL;
	if (f_x) delete f_x;
	if (source->f_x) f_x = source->f_x->new_Duplicate();
		else f_x = NULL;
	if (f_y) delete f_y;
	if (source->f_y) f_y = source->f_y->new_Duplicate();
		else f_y = NULL;
	if (f_z) delete f_z;
	if (source->f_z) f_z = source->f_z->new_Duplicate();
		else f_z = NULL;
}

//////



void ChForce::SetModulation	(ChFunction* m_funct)
{
	if (modula) delete modula;
	modula = m_funct;
}
void ChForce::SetMove_x	(ChFunction* m_funct)
{
	if (move_x) delete move_x;
	move_x = m_funct;
}
void ChForce::SetMove_y	(ChFunction* m_funct)
{
	if (move_y) delete move_y;
	move_y = m_funct;
}
void ChForce::SetMove_z	(ChFunction* m_funct)
{
	if (move_z) delete move_z;
	move_z = m_funct;
}
void ChForce::SetF_x	(ChFunction* m_funct)
{
	if (f_x) delete f_x;
	f_x = m_funct;
}
void ChForce::SetF_y	(ChFunction* m_funct)
{
	if (f_y) delete f_y;
	f_y = m_funct;
}
void ChForce::SetF_z	(ChFunction* m_funct)
{
	if (f_z) delete f_z;
	f_z = m_funct;
}



////// Impose absolute or relative positions, also
//     setting the correct "rest position".

void ChForce::SetVpoint (Vector mypoint)
{
			// abs pos
	vpoint   = mypoint;
			// rel pos
	vrelpoint= GetBody()->Point_World2Body(&vpoint);
			// computes initial rest position.
	Vector displace = VNULL;
	if (move_x) displace.x = move_x->Get_y(ChTime);
	if (move_y) displace.y = move_y->Get_y(ChTime);
	if (move_z) displace.z = move_z->Get_y(ChTime);
	if (frame == FPOS_WORLD)
		restpos = Vsub (vpoint, displace);
	if (frame == FPOS_BODY)
		restpos = Vsub (vrelpoint, displace);
};

void ChForce::SetVrelpoint (Vector myrelpoint)
{
			// rel pos
	vrelpoint= myrelpoint;
			// abs pos
	vpoint   = GetBody()->Point_Body2World(&vrelpoint);
			// computes initial rest position.
	Vector displace = VNULL;
	if (move_x) displace.x = move_x->Get_y(ChTime);
	if (move_y) displace.y = move_y->Get_y(ChTime);
	if (move_z) displace.z = move_z->Get_y(ChTime);
	if (frame == FPOS_WORLD)
		restpos = Vsub (vpoint, displace);
	if (frame == FPOS_BODY)
		restpos = Vsub (vrelpoint, displace);
};


////// Impose absolute or relative force directions

void ChForce::SetDir (Vector newf)
{
	vdir = Vnorm(newf);
	vreldir = GetBody()->Dir_World2Body(&vdir);
	UpdateState(); // update also F
}

void ChForce::SetRelDir (Vector newf)
{
	vreldir = Vnorm(newf);
	vdir = GetBody()->Dir_Body2World(&vreldir);
	UpdateState(); // update also F
}

////// Impose module

void ChForce::SetMforce (double newf)
{
	mforce = newf;
	UpdateState(); // update also F
}


////// force as applied to body
void ChForce::GetBodyForceTorque (Vector* body_force, Vector* body_torque)
{
	ChMatrix33<> Xpos;

	switch (mode)
	{
	case FTYPE_FORCE:
		*body_force = this->force;				  // Fb = F.w

		Xpos.Set_X_matrix (this->vrelpoint);
		*body_torque= Xpos.MatrT_x_Vect(this->relforce);
		*body_torque= Vmul (*body_torque, -1.0);  // Mb = - [u]'[A]'F,w   = - [u]'F,l
		break;

	case FTYPE_TORQUE:
		*body_force = VNULL;				 	 // Fb = 0;
		*body_torque = relforce;				 // Mb = [A]'F,w   = F,l
		break;

	default:
		break;
	}
}

//////
////// Updating
//////



void ChForce::UpdateTime (double mytime)
{
	ChTime = mytime;

	//... put time-sensitive stuff here..
}

void ChForce::UpdateState ()
{
	ChBody* my_body;
	double modforce;
	Vector vectforce;
	Vector vmotion;
	Vector xyzforce;
	ChMatrixNM<double,3,1> mat_force;
	ChMatrix33<> Xpos;
	ChMatrixNM<double,4,1> Qfrot;

	my_body = GetBody();


	// ====== Update the position of point of application

	vmotion = VNULL;
	if (move_x) vmotion.x = move_x->Get_y(ChTime);
	if (move_y) vmotion.y = move_y->Get_y(ChTime);
	if (move_z) vmotion.z = move_z->Get_y(ChTime);

	switch  (frame)
	{
	case FPOS_WORLD:
		vpoint = Vadd (restpos, vmotion);	// Uw
		vrelpoint = my_body->Point_World2Body(&vpoint); // Uo1 = [A]'(Uw-Xo1)
		break;
	case FPOS_BODY:
		vrelpoint = Vadd (restpos, vmotion);	// Uo1
		vpoint = my_body->Point_Body2World(&vrelpoint); // Uw = Xo1+[A]Uo1
		break;
	}


	// ====== Update the fm force vector and add fv

	modforce = mforce * modula->Get_y(ChTime);

	vectforce = VNULL;
	xyzforce = VNULL;
	if (f_x) xyzforce.x = f_x->Get_y(ChTime);
	if (f_y) xyzforce.y = f_y->Get_y(ChTime);
	if (f_z) xyzforce.z = f_z->Get_y(ChTime);

	switch  (align)
	{
	case FDIR_WORLD:
		vreldir = my_body->Dir_World2Body(&vdir);
		vectforce = Vmul (vdir, modforce);
		vectforce = Vadd (vectforce, xyzforce);
		break;
	case FDIR_BODY:
		vdir = my_body->Dir_Body2World(&vreldir);
		vectforce = Vmul (vdir, modforce);
		xyzforce = my_body->Dir_Body2World(&xyzforce);
		vectforce = Vadd (vectforce, xyzforce);
		break;
	}

	force = vectforce;	// Fw
	relforce = my_body->Dir_World2Body(&force); // Fo1 = [A]'Fw

	// ====== Update the Qc lagrangian!

	switch (mode)
	{
	case FTYPE_FORCE:
		{
		Qf->SetElement(0,0,force.x);	// pos.lagrangian Qfx
		Qf->SetElement(1,0,force.y);
		Qf->SetElement(2,0,force.z);
											//   Qfrot= (-[A][u][G])'f
		Vector VQtemp;

		Xpos.Set_X_matrix (vrelpoint);

		VQtemp= Xpos.MatrT_x_Vect(relforce);  // = [u]'[A]'F,w

		mat_force.PasteVector (VQtemp,0,0);

		ChMatrixNM<double,3,4> mGl;
		ChFrame<>::SetMatrix_Gl(mGl, my_body->GetCoord().rot);

		Qfrot.MatrTMultiply (mGl, mat_force);
		Qfrot.MatrNeg();				// Q = - [Gl]'[u]'[A]'F,w
		Qf->PasteMatrix (&Qfrot, 3,0);
		break;
		}

	case FTYPE_TORQUE:
		Qf->SetElement(0,0,0);	// pos.lagrangian Qfx
		Qf->SetElement(1,0,0);
		Qf->SetElement(2,0,0);

								// rot.lagangian
		mat_force.PasteVector (relforce,0,0);

		ChMatrixNM<double,3,4> mGl;
		ChFrame<>::SetMatrix_Gl(mGl, my_body->GetCoord().rot);

		Qfrot.MatrTMultiply (mGl, mat_force);
		Qf->PasteMatrix (&Qfrot, 3,0);

		break;
	}
}

void ChForce::Update (double mytime)
{
	UpdateTime (mytime);
	UpdateState ();
}


void ChForce::UpdateExternalGeometry ()
{
	if (GetExternalObject())
		GetExternalObject()->onChronoChanged();
}




////// File  I/O



void ChForce::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChObj::StreamOUT(mstream);

		// stream out all member data
	mstream << GetMode();
	mstream << GetFrame();
	mstream << GetAlign();
	mstream << vrelpoint;
	mstream << vpoint;
	mstream.AbstractWrite(this->move_x);
	mstream.AbstractWrite(this->move_y);
	mstream.AbstractWrite(this->move_z);
	mstream << restpos;
	mstream.AbstractWrite(this->f_x);
	mstream.AbstractWrite(this->f_y);
	mstream.AbstractWrite(this->f_z);
	mstream << GetMforce();
	mstream.AbstractWrite(this->modula);
	mstream << vreldir;
	mstream << vdir;
}

void ChForce::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChObj::StreamIN(mstream);

		// stream in all member data
	double dfoo;
	int    ifoo;
	Vector vfoo;
	ChFunction* ffoo;

	mstream >> ifoo;				SetMode(ifoo);
	mstream >> ifoo;				SetFrame(ifoo);
	mstream >> ifoo;				SetAlign(ifoo);
	mstream >> vfoo;				SetVrelpoint(vfoo);
	mstream >> vfoo;				SetVpoint(vfoo);
	mstream.AbstractReadCreate(&ffoo);			SetMove_x(ffoo);
	mstream.AbstractReadCreate(&ffoo);			SetMove_y(ffoo);
	mstream.AbstractReadCreate(&ffoo);			SetMove_z(ffoo);
	mstream >> vfoo;				//SetRestpos(dfoo);//not needed,
	mstream.AbstractReadCreate(&ffoo);			SetF_x(ffoo);
	mstream.AbstractReadCreate(&ffoo);			SetF_y(ffoo);
	mstream.AbstractReadCreate(&ffoo);			SetF_z(ffoo);
	mstream >> dfoo;				SetMforce(dfoo);
	mstream.AbstractReadCreate(&ffoo);			SetModulation(ffoo);
	mstream >> vfoo;				SetRelDir(vfoo);
	mstream >> vfoo;				SetDir(vfoo);
}

void ChForce::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FORCE   " << GetName() <<"\n";

	//***TO DO***
}








} // END_OF_NAMESPACE____

///// eof

