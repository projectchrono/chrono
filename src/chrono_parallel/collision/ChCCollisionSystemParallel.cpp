//////////////////////////////////////////////////
//
//   ChCCollisionSystemGPU->cpp
//
// ------------------------------------------------
//       Copyright:Alessandro Tasora / DeltaKnowledge
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_parallel/collision/ChCCollisionSystemParallel.h"

namespace chrono {
namespace collision {

ChCollisionSystemParallel::ChCollisionSystemParallel(ChParallelDataManager* dm) : data_manager(dm) {
  broadphase = new ChCBroadphase;
  narrowphase = new ChCNarrowphaseDispatch;
  aabb_generator = new ChCAABBGenerator;
  broadphase->data_manager = dm;
  narrowphase->data_manager = dm;
  aabb_generator->data_manager = dm;
}

ChCollisionSystemParallel::~ChCollisionSystemParallel() {
  delete narrowphase;
  delete broadphase;
  delete aabb_generator;
}

void ChCollisionSystemParallel::Add(ChCollisionModel* model) {
  if (model->GetPhysicsItem()->GetCollide() == true) {
    ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);
    int body_id = pmodel->GetBody()->GetId();
    short2 fam = S2(pmodel->GetFamilyGroup(), pmodel->GetFamilyMask());
    // The offset for this shape will the current total number of points in
    // the convex data list
    int convex_data_offset = data_manager->host_data.convex_data.size();
    // Insert the points into the global convex list
    data_manager->host_data.convex_data.insert(data_manager->host_data.convex_data.end(),
                                               pmodel->local_convex_data.begin(), pmodel->local_convex_data.end());

    for (int j = 0; j < pmodel->GetNObjects(); j++) {
      real3 obB = pmodel->mData[j].B;

      // Compute the global offset of the convex data structure based on the number of points
      // already present
      if (pmodel->mData[j].type == CONVEX) {
        obB.y += convex_data_offset;  // update to get the global offset
      }

      data_manager->host_data.ObA_rigid.push_back(pmodel->mData[j].A);
      data_manager->host_data.ObB_rigid.push_back(obB);
      data_manager->host_data.ObC_rigid.push_back(pmodel->mData[j].C);
      data_manager->host_data.ObR_rigid.push_back(pmodel->mData[j].R);
      data_manager->host_data.fam_rigid.push_back(fam);
      data_manager->host_data.margin_rigid.push_back(pmodel->mData[j].margin);
      data_manager->host_data.typ_rigid.push_back(pmodel->mData[j].type);
      data_manager->host_data.id_rigid.push_back(body_id);
      data_manager->num_rigid_shapes++;
    }
  }
}

void ChCollisionSystemParallel::Remove(ChCollisionModel* model) {
  //            ChCollisionModelGPU *body = (ChCollisionModelGPU *) model;
  //            int body_id = ((ChBodyGPU *) body->GetBody())->id;
  //
  //            //for (int j = 0; j < body->GetNObjects(); j++) {
  //            for (int i = 0; i < data_manager->host_typ_data.size(); i++) {
  //                if (data_manager->host_typ_data[i].z == body_id) {
  //                    data_manager->host_ObA_data.erase(data_manager->host_ObA_data.begin() + i);
  //                    data_manager->host_ObB_data.erase(data_manager->host_ObB_data.begin() + i);
  //                    data_manager->host_ObC_data.erase(data_manager->host_ObC_data.begin() + i);
  //                    data_manager->host_ObR_data.erase(data_manager->host_ObR_data.begin() + i);
  //                    data_manager->host_fam_data.erase(data_manager->host_fam_data.begin() + i);
  //                    data_manager->host_typ_data.erase(data_manager->host_typ_data.begin() + i);
  //                    data_manager->num_models--;
  //                    return;
  //                }
  //            }
  //
  //            //}
}

void ChCollisionSystemParallel::Run() {
  LOG(INFO) << "ChCollisionSystemParallel::Run()";
  if (data_manager->settings.collision.use_aabb_active) {
    custom_vector<bool> body_active(data_manager->num_rigid_bodies, false);
    GetOverlappingAABB(body_active, data_manager->settings.collision.aabb_min,
                       data_manager->settings.collision.aabb_max);
    for (int i = 0; i < data_manager->host_data.active_rigid.size(); i++) {
      if (data_manager->host_data.active_rigid[i] == true && data_manager->host_data.collide_rigid[i] == true) {
        data_manager->host_data.active_rigid[i] = body_active[i];
      }
    }
  }

  if (data_manager->num_rigid_shapes <= 0) {
    return;
  }

  data_manager->system_timer.start("collision_broad");
  aabb_generator->GenerateAABB();
  broadphase->DetectPossibleCollisions();
  data_manager->system_timer.stop("collision_broad");

  data_manager->system_timer.start("collision_narrow");
  narrowphase->Process();
  data_manager->system_timer.stop("collision_narrow");
}

void ChCollisionSystemParallel::GetOverlappingAABB(custom_vector<bool>& active_id, real3 Amin, real3 Amax) {
  aabb_generator->GenerateAABB();
#pragma omp parallel for
  for (int i = 0; i < data_manager->host_data.typ_rigid.size(); i++) {
    real3 Bmin = data_manager->host_data.aabb_min_rigid[i];
    real3 Bmax = data_manager->host_data.aabb_max_rigid[i];

    bool inContact = (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) &&
                     (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
    if (inContact) {
      active_id[data_manager->host_data.id_rigid[i]] = true;
    }
  }
}

std::vector<int2> ChCollisionSystemParallel::GetOverlappingPairs() {
  std::vector<int2> pairs;
  pairs.resize(data_manager->host_data.pair_rigid_rigid.size());
  for (int i = 0; i < data_manager->host_data.pair_rigid_rigid.size(); i++) {
    int2 pair = I2(int(data_manager->host_data.pair_rigid_rigid[i] >> 32),
                   int(data_manager->host_data.pair_rigid_rigid[i] & 0xffffffff));
    pairs[i] = pair;
  }
  return pairs;
}

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
