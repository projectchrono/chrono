///////////////////////////////////////////////////
//
//   ChBody.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChBodyGPU.h"
#include "ChSystemGPU.h"
namespace chrono {
	using namespace collision;

	// Register into the object factory, to enable run-time
	// dynamic creation and persistence
	ChClassRegister<ChBodyGPU> a_registration_ChBodyGPU;

	//////////////////////////////////////
	//////////////////////////////////////

	/// CLASS FOR SOLID BODIES

	ChBodyGPU::ChBodyGPU() : ChBody() {
		delete collision_model;
		collision_model = InstanceCollisionModel();
		id = 0;
	}

	ChBodyGPU::~ChBodyGPU() {
	}

	ChCollisionModel* ChBodyGPU::InstanceCollisionModel() {
		ChCollisionModel* collision_model_t = (ChCollisionModelGPU*) new ChCollisionModelGPU();
		((ChCollisionModelGPU*) collision_model_t)->SetBody(this);
		return collision_model_t;
	}
} // END_OF_NAMESPACE____

/////////////////////