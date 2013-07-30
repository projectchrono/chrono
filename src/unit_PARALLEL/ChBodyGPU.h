#ifndef CHBODYGPU_H
#define CHBODYGPU_H

//////////////////////////////////////////////////
//
//   ChBodyGPU.h
//   Derived Class for GPU rigid bodies,
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChBody.h"
#include "ChApiGPU.h"
#include "collision/ChCCollisionModelGPU.h"
#include "collision/ChCModelBulletBody.h"
namespace chrono {
using namespace collision;

//////////////////////////////////////
/// CLASS FOR SOLID GPU BODIES
//////////////////////////////////////
class ChApiGPU ChBodyGPU: public ChBody {
CH_RTTI(ChBodyGPU, ChPhysicsItem)
	;

public:

	// CONSTRUCTORS

	ChBodyGPU();
	~ChBodyGPU();

	virtual ChCollisionModel* InstanceCollisionModel();
	ChCollisionModel* SetCollisionModelBullet();

};

//typedef ChSharedPtr<ChBodyGPU> ChSharedBodyGPUPtr;
} // END_OF_NAMESPACE____

#endif

