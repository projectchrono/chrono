#ifndef CHBODYGPU_H
#define CHBODYGPU_H

//////////////////////////////////////////////////
//
//   ChBodyGPU.h
//
//   Derived Class for GPU rigid bodies,
//
//
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "physics/ChBody.h"
#include "ChApiGPU.h"
#include "ChCCollisionModelGPU.h"
namespace chrono {
	using namespace collision;

	//class ChSystem;
	class ChApiGPU ChBodyGPU: public ChBody {
		CH_RTTI(ChBodyGPU,ChPhysicsItem);

	public:

		//
		// CONSTRUCTORS
		//
		/// Build a rigid body.
		ChBodyGPU();
		/// Destructor
		~ChBodyGPU();
		/// Instantiate the collision model
		virtual ChCollisionModel* InstanceCollisionModel();

		/// Returns the total applied force
		ChVector<> GetXForce() {
			return Xforce;
		}
		/// Returns the total applied torque
		ChVector<> GetXTorque() {
			return Xtorque;
		}
		/// Returns the gyroscopic torque
		ChVector<> GetGyro() {
			return gyro;
		}
		/// Set the applied force computed from contacts and the bilateral constraints acting on the body
		void SetAppliedForce(ChVector<> mForce) {
			mAppliedForce=mForce;
		}
		/// Get the applied force computed from contacts and the bilateral constraints acting on the body
		ChVector<> GetAppliedForce() {
			return mAppliedForce;
		}
		int id;
	private:
		ChVector<> mAppliedForce;
	};

	typedef ChSharedPtr<ChBodyGPU> ChSharedBodyGPUPtr;
} // END_OF_NAMESPACE____

#endif
