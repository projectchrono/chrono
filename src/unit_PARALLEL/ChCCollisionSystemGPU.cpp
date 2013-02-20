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
        }
        void ChCollisionSystemGPU::Add(ChCollisionModel *model) {
            if (model->GetPhysicsItem()->GetCollide() == true) {
                ChCollisionModelGPU *body = (ChCollisionModelGPU *) model;
                int body_id = ((ChBodyGPU *) body->GetBody())->id;
                int2 fam = I2(body->GetFamily(), body->GetNoCollFamily());

                for (int j = 0; j < body->GetNObjects(); j++) {
                    real3 obA = body->mData[j].A;
                    real3 obB = body->mData[j].B;
                    real3 obC = body->mData[j].C;
                    real4 obR = body->mData[j].R;
                    int3 type = I3(body->mData[j].type, data_container->number_of_models, body_id);
                    data_container->host_ObA_data.push_back(obA);
                    data_container->host_ObB_data.push_back(obB);
                    data_container->host_ObC_data.push_back(obC);
                    data_container->host_ObR_data.push_back(obR);
                    data_container->host_fam_data.push_back(fam);
                    data_container->host_typ_data.push_back(type);
                    data_container->number_of_models++;
                }
            }
        }

        void ChCollisionSystemGPU::Remove(ChCollisionModel *model) {
            ChCollisionModelGPU *body = (ChCollisionModelGPU *) model;
            int body_id = ((ChBodyGPU *) body->GetBody())->id;

            //for (int j = 0; j < body->GetNObjects(); j++) {
            for (int i = 0; i < data_container->host_typ_data.size(); i++) {
                if (data_container->host_typ_data[i].z == body_id) {
                    data_container->host_ObA_data.erase(data_container->host_ObA_data.begin() + i);
                    data_container->host_ObB_data.erase(data_container->host_ObB_data.begin() + i);
                    data_container->host_ObC_data.erase(data_container->host_ObC_data.begin() + i);
                    data_container->host_ObR_data.erase(data_container->host_ObR_data.begin() + i);
                    data_container->host_fam_data.erase(data_container->host_fam_data.begin() + i);
                    data_container->host_typ_data.erase(data_container->host_typ_data.begin() + i);
                    data_container->number_of_models--;
                    return;
                }
            }

            //}
        }

    } // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


