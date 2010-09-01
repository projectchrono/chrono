///////////////////////////////////////////////////
//
//   ChMarker.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
 

#include "core/ChTrasform.h"
#include "physics/ChMarker.h"
#include "physics/ChGlobal.h"
#include "physics/ChBody.h"
#include "physics/ChExternalObject.h"
 


namespace chrono
{

#define MARKER_BDF_STEP 0.0001




//////////////////////////////////////
//////////////////////////////////////

// CLASS FOR MARKERS

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChMarker> a_registration_ChMarker;


ChMarker::ChMarker()
{
	Body = NULL;

	motion_X =   new ChFunction (0);  // default: no motion
	motion_Y =   new ChFunction (0);
	motion_Z =   new ChFunction (0);
	motion_ang = new ChFunction (0);
	motion_axis = VECT_Z;

	rest_coord=	CSYSNORM;

	motion_type = M_MOTION_FUNCTIONS;

	last_rel_coord     = CSYSNORM;
	last_rel_coord_dt  = CSYSNULL;
	last_time = 0;

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

	UpdateState();
}

ChMarker::ChMarker (char myname[], ChBody* myBody, Coordsys myrel_pos, Coordsys myrel_pos_dt, Coordsys myrel_pos_dtdt)
{
	SetName (myname);
	Body = myBody;

	motion_X =   new ChFunction (0);  // default: no motion
	motion_Y =   new ChFunction (0);
	motion_Z =   new ChFunction (0);
	motion_ang = new ChFunction (0);
	motion_axis = VECT_Z;

	rest_coord=	CSYSNORM;

	motion_type = M_MOTION_FUNCTIONS;

	SetCoord (myrel_pos);
	SetCoord_dt (myrel_pos_dt);
	SetCoord_dtdt (myrel_pos_dtdt);

	last_rel_coord     = CSYSNORM;
	last_rel_coord_dt  = CSYSNULL;
	last_time = 0;

	SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

	UpdateState();
};

ChMarker::~ChMarker ()
{
	if (motion_X) delete motion_X;
	if (motion_Y) delete motion_Y;
	if (motion_Z) delete motion_Z;
	if (motion_ang) delete motion_ang;
}

void ChMarker::Copy(ChMarker* source)
{
	// first copy the parent class data...
	ChObj::Copy(source);

	// first copy the parent class data...
	ChFrameMoving<double>::operator=(*source);


	Body = NULL;

		// Replace the default functions.
	if (motion_X) delete motion_X;
	if (motion_Y) delete motion_Y;
	if (motion_Z) delete motion_Z;
	if (motion_ang) delete motion_ang;
	motion_X =   source->motion_X->new_Duplicate();
	motion_Y =   source->motion_Y->new_Duplicate();
	motion_Z =   source->motion_Z->new_Duplicate();
	motion_ang = source->motion_ang->new_Duplicate();
	motion_axis= source->motion_axis;

	rest_coord =		source->rest_coord;

	motion_type =	source->motion_type;

	abs_frame =		source->abs_frame;

	last_rel_coord    = source->last_rel_coord;
	last_rel_coord_dt = source->last_rel_coord_dt;
	last_time		= source->last_time;
}


// setup the functions when user changes them.

void ChMarker::SetMotion_X	(ChFunction* m_funct)
{
	if (motion_X) delete motion_X;
	motion_X = m_funct;
}

void ChMarker::SetMotion_Y	(ChFunction* m_funct)
{
	if (motion_Y) delete motion_Y;
	motion_Y = m_funct;
}

void ChMarker::SetMotion_Z	(ChFunction* m_funct)
{
	if (motion_Z) delete motion_Z;
	motion_Z = m_funct;
}

void ChMarker::SetMotion_ang	(ChFunction* m_funct)
{
	if (motion_ang) delete motion_ang;
	motion_ang = m_funct;
}

void ChMarker::SetMotion_axis (Vector m_axis)
{
	motion_axis = m_axis;
}




// Coordinate setting, for user access

void ChMarker::Impose_Rel_Coord (Coordsys m_coord)
{
	Quaternion qtemp;
				// set the actual coordinates
	SetCoord(m_coord);
				// set the resting position coordinates
	rest_coord.pos.x = m_coord.pos.x - motion_X->Get_y(ChTime);
	rest_coord.pos.y = m_coord.pos.y - motion_Y->Get_y(ChTime);
	rest_coord.pos.z = m_coord.pos.z - motion_Z->Get_y(ChTime);
	qtemp = Q_from_AngAxis (-(motion_ang->Get_y(ChTime)), motion_axis);
	rest_coord.rot = Qcross (m_coord.rot, qtemp);  // ***%%% check
				// set also the absolute positions, and other.
	UpdateState () ;
}

void ChMarker::Impose_Abs_Coord (Coordsys m_coord)
{
	ChBody* my_body;
	my_body = GetBody();

	Coordsys csys;
			// trasform coordsys to local coordsys...
	csys.pos= ChTrasform<>::TrasformParentToLocal (m_coord.pos, my_body->GetCoord().pos, *my_body->GetA());
	csys.rot= Qcross (Qconjugate (my_body->GetCoord().rot), m_coord.rot);
			// apply the imposition on local  coordinate and resting coordinate:
	Impose_Rel_Coord (csys);
}




//// Utilities for coordinate transformations
///

Vector ChMarker::Point_World2Ref (Vector* mpoint)
{
	return abs_frame / *mpoint;
}

Vector ChMarker::Point_Ref2World (Vector* mpoint)
{
	return *(ChFrame<double>*)&abs_frame * *mpoint;
}

Vector ChMarker::Dir_World2Ref (Vector* mpoint)
{
	return abs_frame.GetA()->MatrT_x_Vect (*mpoint);
}

Vector ChMarker::Dir_Ref2World (Vector* mpoint)
{
	return abs_frame.GetA()->Matr_x_Vect (*mpoint);
}




// This handles the time-varying functions for the relative
// coordinates

void ChMarker::UpdateTime (double mytime)
{
	static Coordsys csys, csys_dt, csys_dtdt;
	static Quaternion qtemp;
	double ang, ang_dt, ang_dtdt;

	ChTime = mytime;

	// if a imposed motion (keyframed movement) affects the marker postion (example,from R3D animation system),
	// compute the speed and acceleration values by BDF (example,see the UpdatedExternalTime() function, later)
	// so the updating via motion laws can be skipped!
	if (this->motion_type == M_MOTION_KEYFRAMED) return;

	// skip realtive-position-functions evaluation also if
	// someone is already handling this from outside..
  	if (this->motion_type == M_MOTION_EXTERNAL) return;  // >>>>


	// positions:
		// update positions:    rel_pos
	csys.pos.x= motion_X->Get_y(mytime);
	csys.pos.y= motion_Y->Get_y(mytime);
	csys.pos.z= motion_Z->Get_y(mytime);
	if (motion_X->Get_Type() != FUNCT_MOCAP)
		csys.pos = Vadd(csys.pos, rest_coord.pos);

		// update speeds:		rel_pos_dt
	csys_dt.pos.x= motion_X->Get_y_dx(mytime);
	csys_dt.pos.y= motion_Y->Get_y_dx(mytime);
	csys_dt.pos.z= motion_Z->Get_y_dx(mytime);

		// update accelerations
	csys_dtdt.pos.x= motion_X->Get_y_dxdx(mytime);
	csys_dtdt.pos.y= motion_Y->Get_y_dxdx(mytime);
	csys_dtdt.pos.z= motion_Z->Get_y_dxdx(mytime);


	// rotations:

	ang		= motion_ang->Get_y(mytime);
	ang_dt	= motion_ang->Get_y_dx(mytime);
	ang_dtdt= motion_ang->Get_y_dxdx(mytime);

	if ((ang !=0)||(ang_dt !=0)||(ang_dtdt !=0))
	{
		// update q
		Vector motion_axis_versor = Vnorm(motion_axis);
		qtemp = Q_from_AngAxis (ang, motion_axis_versor);
		csys.rot = Qcross (qtemp, rest_coord.rot);
		// update q_dt
		csys_dt.rot = chrono::Qdt_from_AngAxis (csys.rot, ang_dt, motion_axis_versor);
		// update q_dtdt
		csys_dtdt.rot = chrono::Qdtdt_from_AngAxis (ang_dtdt, motion_axis_versor, csys.rot, csys_dt.rot);
	}
	else
	{
		csys.rot = this->coord.rot; //rel_pos.rot;
		csys_dt.rot = QNULL;
		csys_dtdt.rot = QNULL;
	}


	// Set the position, speed and acceleration in relative space,
	// automatically getting also the absolute values,
	if (!(csys==this->coord))
			SetCoord (csys);

	if (!(csys_dt==this->coord_dt) || !(csys_dt.rot==QNULL))
			SetCoord_dt (csys_dt);

	if (!(csys_dtdt==this->coord_dtdt) || !(csys_dtdt.rot==QNULL))
			SetCoord_dtdt (csys_dtdt);
};



void ChMarker::UpdateState ()
{
	if (!GetBody()) return;

	GetBody()->TrasformLocalToParent(*this, abs_frame);
};


void ChMarker::Update (double mytime)
{
	UpdateTime (mytime);
	UpdateState ();
}




void ChMarker::UpdateExternalGeometry ()
{
	// tell the R3 object to move itself where needed
	if (GetExternalObject())
		GetExternalObject()->onChronoChanged();
}

void ChMarker::UpdatedExternalTime (double prevtime, double mtime)
{
	double mstep = mtime-prevtime;

	Coordsys m_rel_pos_dt;
	Coordsys m_rel_pos_dtdt;

	// do not try to switch on the M_MOTION_KEYFRAMED mode if
	// we are already in the M_MOTION_EXTERNAL mode, maybe because
	// a link point-surface is already moving the marker and
	// it will handle the accelerations by itself
	if (this->motion_type == M_MOTION_EXTERNAL) return; // >>>>


	// otherwise see if a BDF is needed, cause an external 3rd party is moving the marker
	this->motion_type = M_MOTION_FUNCTIONS;

	if (  (!(Vequal(&coord.pos, &last_rel_coord.pos))||
		   !(Qequal(&coord.rot, &last_rel_coord.rot)) )
	     && (fabs(mstep) < 0.1)
	     && (mstep != 0)
	   ) // if POSITION or ROTATION ("rel_pos") has been changed in acceptable time step,.
	{
		if ((motion_X->Get_y(mtime)==0)
		 && (motion_Y->Get_y(mtime)==0)
		 && (motion_Z->Get_y(mtime)==0)
		 && (motion_ang->Get_y(mtime)==0)
		 && (motion_X->Get_Type() == FUNCT_CONST)
		 && (motion_Y->Get_Type() == FUNCT_CONST)
		 && (motion_Z->Get_Type() == FUNCT_CONST)
		 && (motion_ang->Get_Type() == FUNCT_CONST)
		 )	// .. and if motion wasn't caused by motion laws, then it was a keyframed movement!
		{
			// compute the relative speed by BDF !
			m_rel_pos_dt.pos = Vmul   ( Vsub (coord.pos, last_rel_coord.pos), 1/mstep);
			m_rel_pos_dt.rot = Qscale ( Qsub (coord.rot, last_rel_coord.rot), 1/mstep);

			// compute the relative acceleration by BDF !
			m_rel_pos_dtdt.pos = Vmul   ( Vsub (m_rel_pos_dt.pos, last_rel_coord_dt.pos), 1/mstep);
			m_rel_pos_dtdt.rot = Qscale ( Qsub (m_rel_pos_dt.rot, last_rel_coord_dt.rot), 1/mstep);

			// Set the position, speed and acceleration in relative space,
			// automatically getting also the absolute values,
			SetCoord_dt (m_rel_pos_dt);
			SetCoord_dtdt (m_rel_pos_dtdt);

			// update the remaining state variables
			this->UpdateState();

			// remember that the movement of this guy won't need further update
			// of speed and acc. via motion laws!
			this->motion_type = M_MOTION_KEYFRAMED;
		}
	}

	// restore state buffers and that's all.
	last_time		  = ChTime;
	last_rel_coord    = coord;
	last_rel_coord_dt = coord_dt;
}





////////////////  FILE I/O



void ChMarker::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(2);

		// serialize parent class too
	ChObj::StreamOUT(mstream);

		// deserialize parent class too
	ChFrameMoving<double>::StreamOUT(mstream);

		// stream out all member data

	mstream << abs_frame;
    mstream << rest_coord;
	mstream << (int)motion_type;
	mstream.AbstractWrite(GetMotion_X());
	mstream.AbstractWrite(GetMotion_Y());
	mstream.AbstractWrite(GetMotion_Z());
	mstream.AbstractWrite(GetMotion_ang());
	mstream << motion_axis;
}

void ChMarker::StreamIN(ChStreamInBinary& mstream)
{
	Coordsys cfoo;
	ChFunction* ffoo;
	Vector vfoo;
	int menum;

		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChObj::StreamIN(mstream);

		// stream in all member data
	if(version==1)
	{
		mstream >> cfoo;		SetCoord(cfoo);
		mstream >> cfoo;		SetCoord_dt(cfoo);
		mstream >> cfoo;		SetCoord_dtdt(cfoo);
		mstream >> cfoo;		SetAbsCoord(cfoo);
		mstream >> cfoo;		SetAbsCoord_dt(cfoo);
		mstream >> cfoo;		SetAbsCoord_dtdt(cfoo);
		mstream >> cfoo;		//SetRestCoord(cfoo); not needed?
	}
	if(version >1)
	{
			// deserialize parent class too
		ChFrameMoving<double>::StreamIN(mstream);

		mstream >> abs_frame;
		mstream >> rest_coord;
		mstream >> menum;		SetMotionType((eChMarkerMotion)menum);
	}

	mstream.AbstractReadCreate(&ffoo);	SetMotion_X(ffoo);
	mstream.AbstractReadCreate(&ffoo);	SetMotion_Y(ffoo);
	mstream.AbstractReadCreate(&ffoo);	SetMotion_Z(ffoo);
	mstream.AbstractReadCreate(&ffoo);	SetMotion_ang(ffoo);
	mstream >> motion_axis;
}

void ChMarker::StreamOUT(ChStreamOutAscii& mstream)
{
	//***TO DO***
}






} // END_OF_NAMESPACE____

////////// end
