#ifndef CHLINKCONTACT_H
#define CHLINKCONTACT_H

///////////////////////////////////////////////////
//
//   ChLinkContact.h
//
//   Classes for enforcing constraints (contacts)
//   created by collision detection.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChLink.h"
#include "collision/ChCCollisionPair.h"


namespace chrono
{

// Unique link identifier, for detecting type faster than with rtti.
#define LNK_CONTACT	40

///
/// Parent class for the ChLinkFastContact, ChLinkGPUcontact etc.
/// Not used directly - look at the children classes.
///

class ChLinkContact : public ChLink {

	CH_RTTI(ChLinkContact,ChLink);

protected:
				//
	  			// DATA
				//

	float kin_fri;			// kinem. friction coefficient;
	float sta_fri;			// static friction;
	float restitution;		// restitution coefficient;

public:
				//
	  			// CONSTRUCTORS
				//

	ChLinkContact ();

	virtual ~ChLinkContact ();
	virtual void Copy(ChLinkContact* source);

				//
	  			// FUNCTIONS
				//

	virtual int GetType	() {return LNK_CONTACT;}

					/// Initialize again this constraint.
	virtual void Reset(ChCollisionPair* mpair, ChBody* mbody1, ChBody* mbody2) = 0;

					/// Tells IsCreatedByCollisionDetection='true' because this 
					/// type of constraint is created automatially by ChSystem when 
					/// collisions occours. 
	virtual bool IsCreatedByCollisionDetection() {return true;};

					/// Get the number of scalar constraints imposed by this link (only unilateral constr.)
	virtual int GetDOC_d  () {return 3;}


					/// Get the kinematic friction assigned to this contact
	double Get_kin_fri() {return (double)kin_fri;};
	void   Set_kin_fri(double mset) {kin_fri = (float)mset;}

					/// Get the static friction assigned to this contact
	double Get_sta_fri() {return (double)sta_fri;};
	void   Set_sta_fri(double mset) {sta_fri = (float)mset;}

					/// Get the restitution coefficient assigned to this contact
	double Get_restitution() {return (double)restitution;};
	void   Set_restitution(double mset) {restitution = (float)mset;}

					/// Get the contact point on body 1, expressed in world absolute coordinates
	virtual ChVector<> GetContactP1() = 0;
					
					/// Get the contact point on body 2, expressed in world absolute coordinates
	virtual ChVector<> GetContactP2() = 0;
		
					/// Get the contact normal, exiting from body1 surface, expressed in world 
					/// absolute coordinates
	virtual ChVector<float> GetContactNormal() = 0;

					/// Get the contact distance (negative if interpenetrating).
	virtual double	   GetContactDistance() = 0;


};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
