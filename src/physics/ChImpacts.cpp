///////////////////////////////////////////////////
//
//   ChImpacts.cpp  ***OBSOLETE***
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChImpacts.h"
#include "physics/ChLinksAll.h"


namespace chrono 
{

	//  create and initialize impact structure.

ChImpact::ChImpact()
{
	impactC = 0.1;		
	impactCt = 0.0;
	s_friction = 0.0;
	k_friction = 0.0;
	Pabs = VNULL;
	Set_Nabs(VECT_X);	// also initializes A
	Iabs = VNULL;

	bodyA = NULL;	
	bodyB = NULL;
	mlink = NULL;
};

 

void ChImpact::Set_Nabs(Vector mNabs)
{
	Vector mx, my, mz, mVsingular;
	
	mVsingular = VECT_Y;
	XdirToDxDyDz(&mNabs, &mVsingular, &mx,  &my, &mz);
	
	// sets Nabs as normalized
	this->Nabs = mx;

	this->A.Set_A_axis(mx,my,mz);
}

				// computes the mutual speed of the two bodies in point Pabs,
				// expressed in absolute coordinates.
Vector ChImpact::Get_V12_abs()
{
	Vector mvAabs;
	Vector mvBabs;
	mvAabs = this->bodyA->RelPoint_AbsSpeed(&(bodyA->Point_World2Body(&this->Pabs)));
	mvBabs = this->bodyB->RelPoint_AbsSpeed(&(bodyB->Point_World2Body(&this->Pabs)));
	return Vsub(mvAabs, mvBabs);
}

				// computes the mutual speed of the two bodies in point Pabs,
				// expressed in impact local A coordinates 
				// (hence, x component is normal speed, y z are tangential speeds)
Vector ChImpact::Get_V12_rel()
{
	return this->A.MatrT_x_Vect(Get_V12_abs());
}


} // END_OF_NAMESPACE____

////// end
