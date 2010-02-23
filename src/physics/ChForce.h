#ifndef CHFORCE_H
#define CHFORCE_H

//////////////////////////////////////////////////
//
//   ChForce.h
//
//   Force definition.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChLog.h"
#include "core/ChMath.h"
#include "physics/ChFunction.h"
#include "physics/ChObject.h"


namespace chrono
{

// Forward reference
class ChBody;


#define CHCLASS_FORCE 3

///////////////////////////////

// Types of force application

#define FTYPE_FORCE   0
#define FTYPE_TORQUE  1

// Reference for alignment

#define FDIR_BODY     0
#define FDIR_WORLD    1

// Reference for position frame

#define FPOS_BODY     0
#define FPOS_WORLD    1


//  ChFORCE
/// Forces are objects which must be attached to
/// rigid bodies in order to apply torque or force to
/// such body. ChForce objects are able to represent
/// either forces and torques, depending on a flag.

class ChForce : public ChObj
{
							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChForce,ChObj);

private:
				//
	  			// DATA
				//

	ChBody* Body;		// object of application

	int mode;			// force or torque

	int frame;			// fix position  in body csys or world csys
	int align;			// fix direction in body csys or world csys

	Vector vpoint;		// absolute point of application
	Vector vrelpoint;	// relative point of application

	ChFunction* move_x;	// motion x (abs or rel, depends on 'frame')
	ChFunction* move_y;	// motion y  ""
	ChFunction* move_z;	// motion z  ""
	Vector restpos;		// t=0 position (abs or rel, depends on 'frame')

	ChFunction* f_x;	// fv strengh x (abs or rel, see 'align')
	ChFunction* f_y;	// fv strengh y  ""
	ChFunction* f_z;	// fv strengh z  ""

	double mforce;		// fm scalar force strenght
	ChFunction* modula;	// scalar force fm modulation

	Vector vdir;		// force/torque abs.direction
	Vector vreldir;		// force/torque rel direction

	Vector force;		// TOTAL force vect (abs.coord) = fm*vdir +fx+fy+fz
	Vector relforce;	// TOTAL force vect (rel.coord) = fm*vdir +fx+fy+fz

	ChMatrixDynamic<>* Qf;// lagrangian force

public:
				//
	  			// CONSTRUCTION
				//
	ChForce ();
	~ChForce();
	void Copy(ChForce* source);


				//
	  			// FUNCTIONS
				//

					/// Return the parent body (the force belongs to this rigid body)
	ChBody* GetBody () { return Body;}
					/// Sets the parent body (the force belongs to this rigid body)
	void SetBody (ChBody* newRB) {Body = newRB;}

					/// Sets the mode: FTYPE_FORCE or FTYPE_TORQUE
	void SetMode (int m_mode) {mode = m_mode;};
	int GetMode () {return mode;};

					/// Sets the alignment method: FDIR_BODY or FDIR_WORLD. (the
					/// force will rotate together with this reference.
	void SetAlign (int m_align) {align = m_align;};
	int  GetAlign () {return align;};

					/// Sets the alignment method: FPOS_BODY or FPOS_WORLD. (the
					/// force application point will follow this reference)
	void SetFrame (int m_frame) {frame = m_frame; SetVpoint(vpoint);};
	int  GetFrame () {return frame;};

					/// Gets the application point, in absolute coordinates.
	Vector GetVpoint () {return vpoint;};
					/// Gets the application point, in rigid body coordinates.
	Vector GetVrelpoint () {return vrelpoint;};

					/// Gets the application point, in absolute coordinates.
	void SetVpoint (Vector mypoint);
					/// Gets the application point, in rigid body coordinates.
	void SetVrelpoint (Vector myrelpoint);

					/// Gets the force (or torque) direction, in absolute coordinates.
	Vector GetDir () {return vdir;};
					/// Gets the force (or torque) direction, in rigid body coordinates.
	Vector GetRelDir () {return vreldir;};
					/// Sets the force (or torque) direction, in absolute coordinates.
	void SetDir (Vector newf);
					/// Sets the force (or torque) direction, in rigid body coordinates.
	void SetRelDir (Vector newf);

					/// Sets force (or torque) modulus.
	void SetMforce (double newf);
					/// Gets force (or torque) modulus.
	double GetMforce () {return mforce;};

					/// Sets a f(t) function for time-modulation of the force.
	void SetModulation	(ChFunction* m_funct);
	ChFunction* GetModulation() {return modula;};

					/// Sets a f(t) function for time dependency of position (on x axis)
	void SetMove_x	(ChFunction* m_funct);
	ChFunction* GetMove_x() {return move_x;};
					/// Sets a f(t) function for time dependency of position (on y axis)
	void SetMove_y	(ChFunction* m_funct);
	ChFunction* GetMove_y() {return move_y;};
					/// Sets a f(t) function for time dependency of position (on z axis)
	void SetMove_z	(ChFunction* m_funct);
	ChFunction* GetMove_z() {return move_z;};

					/// Sets a f(t) function for time dependency of force X component.
	void SetF_x	(ChFunction* m_funct);
	ChFunction* GetF_x() {return f_x;};
					/// Sets a f(t) function for time dependency of force Y component.
	void SetF_y	(ChFunction* m_funct);
	ChFunction* GetF_y() {return f_y;};
					/// Sets a f(t) function for time dependency of force Z component.
	void SetF_z	(ChFunction* m_funct);
	ChFunction* GetF_z() {return f_z;};

					/// Gets the instant force vector -or torque vector- in absolute coordinates.
	Vector GetForce () {return force;};
					/// Gets the instant force vector -or torque vector- in rigid body coordinates.
	Vector GetRelForce () {return relforce;};
					/// Gets the instant force vector -or torque vector- modulus.
	double GetForceMod () {return Vlenght(force);};

					/// Gets force-torque applied to rigid body, as lagrangian generalized force (7x1 matrix).
	ChMatrix<>* GetQf () {return Qf;}
					/// Gets force-torque applied to rigid body, as force vector (in absol.coords)
					/// and torque vector (in body coords).
	void GetBodyForceTorque (Vector* body_force, Vector* body_torque);

			// Updating

	void UpdateTime (double mytime);
	void UpdateState ();
	void Update (double mytime);

				/// Tells to the associated external object ChExternalObject() ,if any,
				/// that its 3D shape must be updated in order to syncronize to ChForce
				/// coordinates
	void UpdateExternalGeometry ();

			//
			// STREAMING
			//

					/// Method to allow serialization of transient data in ascii,
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	void StreamOUT(ChStreamOutAscii& mstream);

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);



};




} // END_OF_NAMESPACE____


#endif
