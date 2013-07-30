//   ChBody.cpp

#include "ChBodyGPU.h"
namespace chrono {
using namespace collision;

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChBodyGPU> a_registration_ChBodyGPU;

ChBodyGPU::ChBodyGPU() :
		ChBody(InstanceCollisionModel()) {
}

ChBodyGPU::~ChBodyGPU() {
}

ChCollisionModel* ChBodyGPU::InstanceCollisionModel() {
	ChCollisionModel* collision_model_t = (ChCollisionModelGPU*) new ChCollisionModelGPU();
	((ChCollisionModelGPU*) collision_model_t)->SetBody(this);
	return collision_model_t;
}
ChCollisionModel* ChBodyGPU::SetCollisionModelBullet() {
	if (this->collision_model)
		delete (this->collision_model);
	this->collision_model = (ChModelBulletBody*) new ChModelBulletBody();
	((ChModelBulletBody*) this->collision_model)->SetBody(this);
	return this->collision_model;
}

} // END_OF_NAMESPACE____

