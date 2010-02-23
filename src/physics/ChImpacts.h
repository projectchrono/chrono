#ifndef CHIMPACTS_H
#define CHIMPACTS_H

//////////////////////////////////////////////////
//  
//   ChImpacts.h
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChLists.h"
#include "core/ChMath.h"
#include "physics/ChBody.h"


namespace chrono 
{


	//*Forward reference*
class ChLink;


/// Class for impact data. ***OBSOLETE***
///
/// This data structure is used (sequenced in lists) as a colection
/// of data for impacts wich must be processed at the end of the
/// integration step. The PSystem object will process the list of Ch_Impact 
/// datas at once.

class ChImpact {
private:
				// Absolute position where the impact must be applied (impact point position)
	Vector Pabs;
				// Absolute direction where the impact must be applied (normal to impact surface)
	Vector Nabs;
				// Store the compute impulse here... (in absolute space)
	Vector Iabs; 

				// Impact local reference coordsys, with X axis as impulse direction Nabs and 
				// with axis Y and Z tangent to impact surface.
	ChMatrix33<> A;

public: 

	ChImpact();

				/// restitution coefficient
	double impactC;		
		
				/// tangential restitution coefficient (for Chatterjee-Ruina model)
	double impactCt;

				/// static and cinematic friction coefficients
	double s_friction;
	double k_friction;



				/// the body to whom the impact must be applied,
	ChBody* bodyA;	
				/// the body to whom the contrary impact is applied, 
	ChBody* bodyB;

				/// the link, if any, which created this impact
	ChLink*  mlink;


				/// sets absolute impact position Pabs 
	void Set_Pabs(Vector mPabs) {this->Pabs = mPabs;};
	
				/// sets absolute impact direction Nabs (as applied to BodyA!),
				/// and sets A automatically. 
	void Set_Nabs(Vector mNabs);

				/// After the impulse has been computed by some impact model,
				/// like for example Chatterjee-Ruina, store here the impulse (in rel coords)
	void Set_Iabs(Vector mIabs) {this->Iabs = mIabs;};


		// GETTING DATA 

				/// get impact position
	Vector Get_Pabs() {return this->Pabs;};
				/// get the impact direction Nabs (as applied to BodyA!)
	Vector Get_Nabs() {return this->Nabs;};
				/// get the impact direction Nabs (as applied to BodyA!)
	Vector Get_Iabs() {return this->Iabs;};
				/// get local coordsys matrix
	ChMatrix33<>* Get_A() {return &this->A;};

				/// computes the mutual speed of the two bodies in point Pabs,
				/// expressed in absolute coordinates.
	Vector Get_V12_abs();
				/// computes the mutual speed of the two bodies in point Pabs,
				/// expressed in impact local A coordinates 
				/// (hence, x component is normal speed, y z are tangential speeds)
	Vector Get_V12_rel();


};



} // END_OF_NAMESPACE____

#endif  // END of ChImpact.h 
