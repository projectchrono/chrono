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

#ifndef CHLINKTRAJECTORY_H
#define CHLINKTRAJECTORY_H

///////////////////////////////////////////////////
//
//   ChLinkTrajectory.h
//
//
//   Class for point-on-imposed-trajectory constraint
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChLinkLock.h"
#include "geometry/ChCLine.h"


namespace chrono
{

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_TRAJECTORY  22


///
/// ChLinkTrajectory class.
/// This class implements the 'point on an imposed 
/// trajectory' constraint. 
/// It can be used also to simulate the imposed motion
/// of objects in space (for motion capture, for example).
///

class ChApi ChLinkTrajectory : public ChLinkLock {

	CH_RTTI(ChLinkTrajectory,ChLinkLock);

protected:

						/// Function s=s(t) telling how the curvilinear
						/// parameter of the trajectory is visited in time.
	ChFunction* space_fx;
			
						/// The line for the trajectory.
	geometry::ChLine*		trajectory_line;

public:
						// builders and destroyers
	ChLinkTrajectory ();
	virtual ~ChLinkTrajectory ();
	virtual void Copy(ChLinkTrajectory* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


						/// Gets the address of the function s=s(t) telling 
						/// how the curvilinear parameter of the trajectory changes in time.
	ChFunction* Get_space_fx() {return space_fx;};
						
						/// Sets the function s=s(t) telling how the curvilinear parameter 
						/// of the trajectory changes in time. 
	void Set_space_fx	(ChFunction* m_funct);


						/// Get the address of the trajectory line
	geometry::ChLine* Get_trajectory_line() {return trajectory_line;}
						
						/// Sets the trajectory line (take ownership - does not copy line)
	void Set_trajectory_line (geometry::ChLine* mline);


							// UPDATING FUNCTIONS - "lock formulation" custom implementations

							// Overrides the parent class function. Here it moves the 
							// constraint mmain marker tangent to the line.
	virtual void UpdateTime (double mytime);


							// STREAMING

	virtual void StreamIN(ChStreamInBinary& mstream);
	virtual void StreamOUT(ChStreamOutBinary& mstream);

};






} // END_OF_NAMESPACE____

#endif
