//////////////////////////////////////////////////
//
//   ChCCollisionSystemGPU->cpp
//
// ------------------------------------------------
//       Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCCollisionSystemGPU.h"

namespace chrono {
namespace collision {
ChCollisionSystemGPU::ChCollisionSystemGPU() {
	collision_envelope = .03;
	bpa = R3(20, 20, 20);
	max_body_per_bin = 100;
	min_body_per_bin = 20;

}
void ChCollisionSystemGPU::Add(ChCollisionModel *model) {
	if (model->GetPhysicsItem()->GetCollide() == true) {
		ChCollisionModelGPU *body = (ChCollisionModelGPU *) model;
		int body_id = ((ChBodyGPU *) body->GetBody())->id;
		int2 fam = I2(body->GetFamily(), body->GetNoCollFamily());

		for (int j = 0; j < body->GetNObjects(); j++) {
			real3 obA = body->mData[j].A;
			real3 obB = body->mData[j].B;
			if (body->mData[j].type != TRIANGLEMESH) {
				obB += R3(collision_envelope);
			}
			real3 obC = body->mData[j].C;
			real4 obR = body->mData[j].R;
			data_container->host_ObA_data.push_back(obA);
			data_container->host_ObB_data.push_back(obB);
			data_container->host_ObC_data.push_back(obC);
			data_container->host_ObR_data.push_back(obR);
			data_container->host_fam_data.push_back(fam);
			data_container->host_typ_data.push_back(body->mData[j].type);
			data_container->host_id_data.push_back(body_id);
			data_container->number_of_models++;
		}
	}
}

void ChCollisionSystemGPU::Remove(ChCollisionModel *model) {
//            ChCollisionModelGPU *body = (ChCollisionModelGPU *) model;
//            int body_id = ((ChBodyGPU *) body->GetBody())->id;
//
//            //for (int j = 0; j < body->GetNObjects(); j++) {
//            for (int i = 0; i < data_container->host_typ_data.size(); i++) {
//                if (data_container->host_typ_data[i].z == body_id) {
//                    data_container->host_ObA_data.erase(data_container->host_ObA_data.begin() + i);
//                    data_container->host_ObB_data.erase(data_container->host_ObB_data.begin() + i);
//                    data_container->host_ObC_data.erase(data_container->host_ObC_data.begin() + i);
//                    data_container->host_ObR_data.erase(data_container->host_ObR_data.begin() + i);
//                    data_container->host_fam_data.erase(data_container->host_fam_data.begin() + i);
//                    data_container->host_typ_data.erase(data_container->host_typ_data.begin() + i);
//                    data_container->number_of_models--;
//                    return;
//                }
//            }
//
//            //}
}

void ChCollisionSystemGPU::Run() {
	ChCAABBGenerator aabb_generator;
	ChCBroadphase broadphase;
	broadphase.setBinsPerAxis(bpa);
	broadphase.setBodyPerBin(max_body_per_bin, min_body_per_bin);

	ChCNarrowphase narrowphase;
	aabb_generator.GenerateAABB(data_container->gpu_data.device_typ_data, data_container->gpu_data.device_ObA_data, data_container->gpu_data.device_ObB_data, data_container->gpu_data.device_ObC_data,
			data_container->gpu_data.device_ObR_data, data_container->gpu_data.device_id_data, data_container->gpu_data.device_pos_data, data_container->gpu_data.device_rot_data,
			data_container->gpu_data.device_aabb_data);
	broadphase.detectPossibleCollisions(data_container->gpu_data.device_aabb_data, data_container->gpu_data.device_pair_data);

	narrowphase.SetCollisionEnvelope(collision_envelope);

	narrowphase.DoNarrowphase(data_container->gpu_data.device_typ_data, data_container->gpu_data.device_ObA_data, data_container->gpu_data.device_ObB_data, data_container->gpu_data.device_ObC_data,
			data_container->gpu_data.device_ObR_data, data_container->gpu_data.device_id_data, data_container->gpu_data.device_active_data, data_container->gpu_data.device_pos_data,
			data_container->gpu_data.device_rot_data, data_container->gpu_data.device_pair_data, data_container->gpu_data.device_norm_data, data_container->gpu_data.device_cpta_data,
			data_container->gpu_data.device_cptb_data, data_container->gpu_data.device_dpth_data, data_container->gpu_data.device_bids_data, data_container->gpu_data.number_of_contacts);
}

vector<int2> ChCollisionSystemGPU::GetOverlappingPairs() {
	vector<int2> pairs;
	pairs.resize(data_container->host_pair_data.size());
	data_container->DeviceToHostPairData();
	for (int i = 0; i < data_container->host_pair_data.size(); i++) {
		int2 pair = I2(int(data_container->host_pair_data[i] >> 32), int(data_container->host_pair_data[i] & 0xffffffff));
		pairs[i] = pair;

	}
	return pairs;
}

} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

