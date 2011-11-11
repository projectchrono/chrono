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
#include "ChGPUDataManager.h"
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
			virtual ChCollisionModel* InstanceCollisionModel();
			//void UpdateForces(double mytime);


			ChVector<> GetXForce() {
				return Xforce;
			}
			ChVector<> GetXTorque() {
				return Xtorque;
			}
			ChVector<> GetGyro() {
				return gyro;
			}
			void SetAppliedForce(ChVector<> mForce) {

				mAppliedForce=mForce;

			}
			ChVector<> GetAppliedForce() {
				return mAppliedForce;
			}

			int id;
			ChVector<> mAppliedForce;
			ChGPUDataManager *data_manager;

	};

	typedef ChSharedPtr<ChBodyGPU> ChSharedBodyGPUPtr;

} // END_OF_NAMESPACE____


#endif
