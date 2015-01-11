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

#ifndef CHMARKER_H
#define CHMARKER_H

//////////////////////////////////////////////////
//
//   ChMarker.h
//
//   "Marker" definition (an auxiliary frame, to be
//   attached to rigid bodies).
//   Each body needs markers to define links
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>

#include "core/ChLog.h"
#include "core/ChMath.h"
#include "core/ChFrameMoving.h"

#include "physics/ChFunction.h"
#include "physics/ChObject.h"




namespace chrono
{


// Forward reference
class ChBody;


#define CHCLASS_MARKER 5

///
/// Class for 'markers'.
///
///  Markers are auxiliary reference frames which belong to
/// rigid bodies ChBody() , and move together with them.
/// Most often, markers are used as references to build
/// ChLink() constraints between two rigid bodies.
///  The ChMarker objects allow also to user-define a
/// motion law of marker respect to parent ChBody, if
/// needed to represent imposed trajectories etc.

class ChApi ChMarker : public ChObj, public ChFrameMoving<double>  {

							// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChMarker,ChObj);


				//
	  			// DATA
				//


public:
	enum eChMarkerMotion {
							/// Uses it's own  x,y,z,angle ch functions (default)
		M_MOTION_FUNCTIONS =0,
							/// The marker is moved via external functions, for examples
							/// Real3d keyframing, so backward dirrerentiation will be used
							/// to guess derivatives.
		M_MOTION_KEYFRAMED =1,
							/// Someone (i.e. a constraint object) is moving the marker and
							/// will also provide the correct derivatives.
		M_MOTION_EXTERNAL  =2,
	};

private:
							/// The way the motion of this marker (if any) is handled.
 	eChMarkerMotion motion_type;

	ChFunction* motion_X;	// user imposed motion for X coord, body relative
	ChFunction* motion_Y;	// user imposed motion for Y coord, body relative
	ChFunction* motion_Z;	// user imposed motion for Z coord, body relative
	ChFunction* motion_ang;	// user imposed angle rotation about axis
	Vector		motion_axis;// this is the axis for the user imposed rotation


	ChBody* Body;			// points to parent body

	Coordsys rest_coord;	// relative resting position for the
							// coordsys, for function=0.

	Coordsys last_rel_coord;	// These values are set for each marker update, and are
	Coordsys last_rel_coord_dt;	// used internally to guess if there's some external routine
	double last_time;			// which moves the marker, so marker motion is guessed by BDF.


						// Auxiliary variables, computed after Updating functions..

							/// Absolute position of frame (frame translation and rotation
							/// expressed in absolute coordinate system).
							/// This is computed at each Update() call. Useful for high
							/// performance reasons.
	ChFrameMoving<double> abs_frame;


public:

			//
			// CONSTRUCTORS
			//

	ChMarker ();

	ChMarker (char myname[], ChBody* myBody, Coordsys myrel_pos, Coordsys myrel_pos_dt, Coordsys myrel_pos_dtdt);

	~ChMarker ();

	void Copy(ChMarker* source);



			//
			// FUNCTIONS
			//

				/// Gets the address of the parent rigid body.
	ChBody* GetBody () const { return Body;}
				/// Sets the parent rigid body.
	void SetBody (ChBody* newRB) {Body= newRB;}



				/// Set body-relative coord. and update auxiliary variables
				/// Also, current position becomes the 'resting position' coordinates
				/// for the current time.
	void Impose_Rel_Coord (const Coordsys& m_coord);

				/// Set absolute coordinates  and update auxiliary variables
				/// Also, current position becomes the 'resting position' coordinates
				/// for the current time.
	void Impose_Abs_Coord (const Coordsys& m_coord);


				/// Get the 'resting position' (that is, the position which the
				/// marker should have when the x,y,z motion laws are at time=0).
	Coordsys GetRest_Coord() {return rest_coord;}


			//
			// Body-relative coordinates
			//

				// No functions here...
				//  In order to get/set  body-relative coordinates,
				// you can use the methods of the ChFrameMoving parent
				// class: for example use  my_marker->SetCoord(newpos) to
				// impose a new position&rotation, etc.
				//  NOTE!! after each modification of the frame position,
				// speed, acceleration etc., you should remember to call UpdateState()
				// if you want to kee updated also the absolute coordinates , ie. the
				// auxiliary structure to get with GetAbsFrame().



			//
			// Absolute coordinates (auxiliary data)
			//

				/// Get reference to the inner 'absolute frame' auxiliary
				/// coordinates. This object (coordinates/speeds/accel. of marker
				/// expressed in absolute coordinates) is useful for performace
				/// reasons. Note! it is updated only after each Update() function.
	ChFrameMoving<double>& GetAbsFrame () { return abs_frame; }

				/// Get the translation and rotation (as a ChCoordsys) of the marker
				/// respect to the absolute coordinates.
	Coordsys GetAbsCoord () { return abs_frame.GetCoord(); }
				/// Get the speed of translation and rotation (as a derived ChCoordsys)
				/// of the marker respect to the absolute coordinates.
	Coordsys GetAbsCoord_dt () { return abs_frame.GetCoord_dt(); }
				/// Get the acceleration of translation and rotation (as a derived ChCoordsys)
				/// of the marker respect to the absolute coordinates.
	Coordsys GetAbsCoord_dtdt () { return abs_frame.GetCoord_dtdt(); }

				/// Set the translation and rotation (as a ChCoordsys) of the marker
				/// respect to the absolute coordinates.
				/// NOTE! inner use only, for the moment. Use  Impose_Abs_Coord() if needed.
	void SetAbsCoord (Coordsys newpos) {abs_frame.SetCoord(newpos);}
				/// Set the speed of translation and rotation (as a ChCoordsys) of the marker
				/// respect to the absolute coordinates.
				/// NOTE! inner use only, for the moment.
	void SetAbsCoord_dt (Coordsys newpos_dt) {abs_frame.SetCoord(newpos_dt);}
				/// Set the speed of translation and rotation (as a ChCoordsys) of the marker
				/// respect to the absolute coordinates.
				/// NOTE! inner use only, for the moment.
	void SetAbsCoord_dtdt (Coordsys newpos_dtdt) {abs_frame.SetCoord(newpos_dtdt);}

				/// Get the angular speed respect to absolute coordinates,
				/// expressed in  absolute coordinates.
	Vector  GetAbsWvel () {return abs_frame.GetWvel_par();}

				/// Get the angular acceleration respect to absolute coordinates,
				/// expressed in  absolute coordinates.
	Vector  GetAbsWacc () {return abs_frame.GetWacc_par();}



			//
			// Imposed motion
			//

				/// Set the imposed motion law, for translation on X body axis
	void SetMotion_X	(ChFunction* m_funct);
				/// Set the imposed motion law, for translation on Y body axis
	void SetMotion_Y	(ChFunction* m_funct);
				/// Set the imposed motion law, for translation on Z body axis
	void SetMotion_Z	(ChFunction* m_funct);
				/// Set the imposed motion law, for rotation about an axis
	void SetMotion_ang	(ChFunction* m_funct);
				/// Set the axis of rotation, if rotation motion law is used.
	void SetMotion_axis (Vector m_axis);

				/// The imposed motion law, for translation on X body axis
	ChFunction* GetMotion_X() {return motion_X;};
				/// The imposed motion law, for translation on Y body axis
	ChFunction* GetMotion_Y() {return motion_Y;};
				/// The imposed motion law, for translation on Z body axis
	ChFunction* GetMotion_Z() {return motion_Z;};
				/// The imposed motion law, for rotation about an axis
	ChFunction* GetMotion_ang() {return motion_ang;};
				/// Get the axis of rotation, if rotation motion law is used.
	Vector GetMotion_axis() {return motion_axis;};


				/// Sets the way the motion of this marker (if any) is handled (see
				/// the eChMarkerMotion enum options).
	void SetMotionType (eChMarkerMotion m_motion) {motion_type = m_motion;};

				/// Gets the way the motion of this marker (if any) is handled (see
				/// the eChMarkerMotion enum options).
	eChMarkerMotion GetMotionType () {return motion_type;}




			//
			// UPDATING
			//

				/// Updates the time.dependant variables (ex: ChFunction objects
				/// which impose the body-relative motion, etc.)
	void UpdateTime (double mytime);

				/// Given current state, updates auxiliary variables (for example
				/// the abs_frame data, containing the absolute pos/speed/acc of
				/// the marker.
	void UpdateState ();

				/// Both UpdateTime() and UpdateState() at once.
	void Update (double mytime);



					/// Someone (ex. an ChExternalObject() ) may send this message to
					/// the marker to tell that time has changed (even if simulation is
					/// not running! - so it is different from the usual UpdateTime() -)
	void UpdatedExternalTime (double prevtime, double mtime);

			//
			// UTILITIES
			//

	Vector Point_World2Ref (Vector* mpoint);
	Vector Point_Ref2World (Vector* mpoint);
	Vector Dir_World2Ref (Vector* mpoint);
	Vector Dir_Ref2World (Vector* mpoint);


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
