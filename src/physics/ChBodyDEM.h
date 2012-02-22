#ifndef CHBODYDEM_H
#define CHBODYDEM_H

//////////////////////////////////////////////////
//
//   ChBodyDEM.h
//
//   Class for rigid bodies, that is rigid moving
//   parts with mass and collision geometry 
//   which interact through DEM approach.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "physics/ChBody.h"
#include "core/ChFrameMoving.h"
#include "core/ChShared.h"
#include "physics/ChPhysicsItem.h"
#include "physics/ChForce.h"
#include "physics/ChMarker.h"
#include "lcp/ChLcpVariablesBodyOwnMass.h"
#include "lcp/ChLcpConstraint.h"



namespace chrono
{

using namespace collision;



// Forward references (for parent hierarchy pointer)

class ChSystem;


///
/// Class for rigid bodies. A rigid body is an entity which
/// can move in 3D space, and can be constrained to other rigid
/// bodies using ChLink objects.
/// These objects have mass and inertia properties. A shape can also
/// be associated to the body, for collision detection.
///

class ChApi ChBodyDEM: public ChBody {

						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChBodyDEM, ChBody);


private:
			//
	  		// DATA
			//

	float kn;	// normal spring constant for DEM contacts
	float gn;   // normal damping constant for DEM contacts
	float kt;   // tangential spring constant for DEM contacts
	 


public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a rigid body.
	ChBodyDEM ();
				/// Destructor
	~ChBodyDEM ();

				/// Copy from another ChBodyDEM. 
				/// NOTE: all settings of the body are copied, but the
				/// child hierarchy of ChForces and ChMarkers (if any) are NOT copied.
	void Copy(ChBodyDEM* source);

				/// Instantiate the collision model
	virtual ChCollisionModel* InstanceCollisionModel();

				/// The spring coefficient for DEM contacts.
				/// Default 392400.0
	float  GetSpringCoefficient() {return kn;}
	void   SetSpringCoefficient(float mval) {kn = mval;}
				
				/// The damping coefficient for DEM contacts.
				/// Default 420.0
	float  GetDampingCoefficient() {return gn;}
	void   SetDampingCoefficient(float mval) {gn = mval;}

				/// The spring coefficient for DEM friction.
				/// Default 392400.0
	float  GetSpringCoefficientTangential() {return kt;}
	void   SetSpringCoefficientTangential(float mval) {kt = mval;}

				/// accumulate force on this body, or reset the force to zero
	//void AccumulateForce(ChVector<> ff) {Xforce+=ff;}
	//void ResetBodyForce() {Xforce = VNULL;}
	//void AccumulateTorque(ChVector<> tt) {Xtorque+=tt;}
	//void ResetBodyTorque() {Xtorque = VNULL;}


			//
			// STREAMING
			//


				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);

				/// Save data, including child markers and child forces
	int StreamOUTall (ChStreamOutBinary& m_file);
				/// Read data, including child markers and child forces
	int StreamINall  (ChStreamInBinary&  m_file);

};



typedef ChSharedPtr<ChBodyDEM> ChSharedBodyDEMPtr;



} // END_OF_NAMESPACE____


#endif
