//////////////////////////////////////////////////
//  
//   ChCModelSphereSetBody.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
 
#include "ChCModelSphereSetBody.h"
#include "physics/ChBody.h"


namespace chrono 
{
namespace collision 
{



ChModelSphereSetBody::ChModelSphereSetBody()
{
	mbody = 0;
}


ChModelSphereSetBody::~ChModelSphereSetBody()
{
}

void ChModelSphereSetBody::SyncPosition()
{
	ChBody* bpointer=GetBody();
	assert(bpointer);

	ChCoordsys<> bodyCoord=bpointer->GetCoord();

	// Update the bounding box points in global frame
	//ChVector<> tmin=bodyCoord.TrasformLocalToParent(myBBminLocal);
	//ChVector<> tmax=bodyCoord.TrasformLocalToParent(myBBmaxLocal);
	ChVector<> tmin=bpointer->GetPos()+myBBminLocal;
	ChVector<> tmax=bpointer->GetPos()+myBBmaxLocal;
	myBBminGlobal.Set(tmin.x,tmin.y,tmin.z);
	myBBmaxGlobal.Set(tmax.x,tmax.y,tmax.z);

	// Update the sphere positions in global frame
	thrust::host_vector< ChVector<float> > gPos;
	gPos.resize(nSpheres);
	for(uint i=0; i<nSpheres; i++)
	{
		gPos[i]=bodyCoord.TrasformLocalToParent(sphPosLocal[i]);
	}

	sphPosGlobal.swap(gPos);
}







} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

