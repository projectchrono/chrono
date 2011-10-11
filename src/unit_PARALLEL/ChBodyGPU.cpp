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


	ChBodyGPU::ChBodyGPU() :
		ChBody() {
		delete collision_model;
		collision_model = InstanceCollisionModel();
		id = 0;
		interDist.clear();
	}

	ChBodyGPU::~ChBodyGPU() {
	}

	ChCollisionModel* ChBodyGPU::InstanceCollisionModel() {
		ChCollisionModel* collision_model_t = (ChCollisionModelGPU*) new ChCollisionModelGPU();
		((ChCollisionModelGPU*) collision_model_t)->SetBody(this);
		return collision_model_t;
	}

	void ChBodyGPU::UpdateForces(double mytime) {
		// COMPUTE LAGRANGIAN FORCES APPLIED TO BODY

		Xforce = VNULL;
		Xtorque = VNULL;

		// 1 - force caused by stabilizing damper ***OFF***
		// 2a- force caused by accumulation of forces in body's accumulator Force_acc
		Xforce = Force_acc;
		// 2b- force caused by accumulation of torques in body's accumulator Force_acc
		Xtorque = Dir_World2Body(&Torque_acc);
		// 3 - accumulation of other applied forces
		std::vector<ChForce*>::iterator iforce = forcelist.begin();
		while ((iforce != forcelist.end())) {
			// update positions, f=f(t,q)
			(*iforce)->Update(mytime);

			ChVector<> mforce;
			ChVector<> mtorque;
			(*iforce)->GetBodyForceTorque(&mforce, &mtorque);
			Xforce += mforce;
			Xtorque += mtorque;
			iforce++;
		}

		// 4 - accumulation of script forces
		Xforce += Scr_force;
		Xtorque += Dir_World2Body(&Scr_torque);
		Xforce += this->GetSystem()->Get_G_acc() * this->GetMass();

	}

} // END_OF_NAMESPACE____


/////////////////////
