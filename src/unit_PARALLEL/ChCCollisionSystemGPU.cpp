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
		ChCollisionSystemGPU::ChCollisionSystemGPU() {
			mGPU = new ChCCollisionGPU();
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

					mGPU->data_container->host_ObA_data.push_back(obA);
					mGPU->data_container->host_ObB_data.push_back(obB);
					mGPU->data_container->host_ObC_data.push_back(obC);
					mGPU->data_container->host_ObR_data.push_back(obR);
					mGPU->data_container->host_fam_data.push_back(fam);
					mGPU->data_container->host_typ_data.push_back(type);
					mGPU->number_of_models++;
				}
			}
			mGPU->data_container->number_of_models = mGPU->number_of_models;
		}

		void ChCollisionSystemGPU::Remove(ChCollisionModel* model) {
		}
		void ChCollisionSystemGPU::ComputeAABB(const int &i) {
			mGPU->ComputeAABB(i);
		}
		void ChCollisionSystemGPU::ComputeBounds(const int &i) {
			mGPU->ComputeBounds(i);
		}
		void ChCollisionSystemGPU::ComputeUpdateAABB(const int &i) {
			mGPU->UpdateAABB(i);
		}
		void ChCollisionSystemGPU::ComputeBroadPhase(const int &i) {
			mGPU->Broadphase(i);
		}
		void ChCollisionSystemGPU::ComputeNarrowPhase(const int &i) {
			mGPU->Narrowphase(i);
		}

		void ChCollisionSystemGPU::Run() {
			mGPU->Run();
		}
	} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____
