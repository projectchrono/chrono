//////////////////////////////////////////////////
//  
//   ChCModelGPUBody.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com 
// ------------------------------------------------
///////////////////////////////////////////////////
 
#ifndef CH_NOCUDA 
  
 
#include "ChCModelGPUBody.h" 
#include "physics/ChBody.h"


namespace chrono
{
namespace collision
{



ChModelGPUBody::ChModelGPUBody()
{
	mbody = 0;
}


ChModelGPUBody::~ChModelGPUBody()
{
}

void ChModelGPUBody::SyncPosition()
{
	ChBody* bpointer = GetBody();
	assert(bpointer);
	assert(bpointer->GetSystem());

	posX=bpointer->GetPos().x;
	posY=bpointer->GetPos().y;
	posZ=bpointer->GetPos().z;
}







} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif  // end of ! CH_NOCUDA
