//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//  
//   ChCModelSphereSetDEM.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////
 
  
 
#include "ChCModelSphereSetDEM.h"
#include "physics/ChBodyDEM.h"


namespace chrono 
{
namespace collision 
{



ChModelSphereSetDEM::ChModelSphereSetDEM()
{
	mbody = 0;
	this->model_envelope=0.0f;
}


ChModelSphereSetDEM::~ChModelSphereSetDEM()
{
}

void ChModelSphereSetDEM::SyncPosition()
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

