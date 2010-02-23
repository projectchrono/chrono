#ifndef CHLINKPOINTSPLINE_H
#define CHLINKPOINTSPLINE_H

///////////////////////////////////////////////////
//
//   ChLinkPointSpline.h
//
//
//   Class for point-on-spline constraint
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChLinkLock.h"
#include "geometry/ChCLine.h"



namespace chrono
{

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_POINTSPLINE 20


///
/// ChLinkPointSpline class.
/// This class implements the 'point on a spline curve' 
/// constraint. It can be used also to simulate
/// curvilinear glyphs, etc.
///

class ChLinkPointSpline : public ChLinkLock {

	CH_RTTI(ChLinkPointSpline,ChLinkLock);

protected:

						/// The line for the trajectory.
	geometry::ChLine*		trajectory_line;

public:
						// builders and destroyers
	ChLinkPointSpline ();
	virtual ~ChLinkPointSpline ();
	virtual void Copy(ChLinkPointSpline* source);
	virtual ChLink* new_Duplicate ();	// always return base link class pointer


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
