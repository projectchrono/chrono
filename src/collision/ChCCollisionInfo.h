#ifndef CHCCOLLISIONINFO_H
#define CHCCOLLISIONINFO_H

///////////////////////////////////////////////////
//
//   ChCCollisionInfo.h
//
//   Class for passing basic data about contacts
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChVector.h"
#include "collision/ChCCollisionModel.h"

namespace chrono
{
namespace collision 
{


///   Class for passing basic data about contact pairs

class ChCollisionInfo 
{
public:
	ChCollisionModel* modelA; ///<  model A
	ChCollisionModel* modelB; ///<  model B
	ChVector<> vpA;			  ///<  coll.point on A, in abs coords
	ChVector<> vpB;		      ///<  coll.point on B, in abs coords
	ChVector<> vN; 		      ///<  coll.normal, respect to A, in abs coords
	double distance;		  ///<  distance (negative for penetration)
	float* reaction_cache;	  ///<  pointer to some persistent user cache of reactions


		/// Basic default constructor
	ChCollisionInfo()
		{
			modelA = modelB = 0;
			vpA = vpB = VNULL;
			vN.Set(1,0,0);
			distance = 0.;
			reaction_cache=0;
		}

			/// Swap models, that is modelA becomes modelB and viceversa; 
			/// normal and so on are updates as well.
	void SwapModels()
		{
			ChCollisionModel* modeltemp;
			 modeltemp = modelA;
			 modelA = modelB;
			 modelB = modeltemp;
			ChVector<> vtemp;
			 vtemp = vpA;
			 vpA = vpB;
			 vpB = vtemp;
			vN = Vmul(vN, -1.0);
		}


};


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif
