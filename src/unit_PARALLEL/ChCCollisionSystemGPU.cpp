//////////////////////////////////////////////////
//
//   ChCCollisionSystemGPU->cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCCollisionSystemGPU.h"

namespace chrono {
	namespace collision {
		ChCollisionSystemGPU::ChCollisionSystemGPU(ChLcpSystemDescriptorGPU* mdescriptor) {
			mSystemDescriptor = mdescriptor;
			mGPU = mSystemDescriptor->gpu_collision;
		}
		void ChCollisionSystemGPU::Add(ChCollisionModel* model) {
			if (model->GetPhysicsItem()->GetCollide() == true) {
				ChCollisionModelGPU *body = (ChCollisionModelGPU*) model;
				int type = body->GetShapeType();
				int body_id = ((ChBodyGPU*) body->GetBody())->id;
				int2 fam = I2(body->GetFamily(), body->GetNoCollFamily());
				for (int j = 0; j < body->GetNObjects(); j++) {
					float3 obA = body->mData[j].A;
					float3 obB = body->mData[j].B;
					float3 obC = body->mData[j].C;
					float4 obR = body->mData[j].R;
					int3 type = I3(body->mData[j].type, mGPU->number_of_models, body_id);
					mGPU->AddObject(obA, obB, obC, obR, fam, type);
				}
			}
			mGPU->data_container->number_of_models=mGPU->number_of_models;
		}

		void ChCollisionSystemGPU::Remove(ChCollisionModel* model) {}

		void ChCollisionSystemGPU::Run() {
			mGPU->Run();
		}
	} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____