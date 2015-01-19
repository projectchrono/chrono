//////////////////////////////////////////////////
//
//   ChCCollisionSystemGPU->cpp
//
// ------------------------------------------------
//       Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_parallel/collision/ChCCollisionSystemParallel.h"

namespace chrono {
namespace collision {

ChCollisionSystemParallel::ChCollisionSystemParallel(ChParallelDataManager* dc)
: data_container(dc)
{
  broadphase = new ChCBroadphase;
  narrowphase = new ChCNarrowphaseDispatch;
}

ChCollisionSystemParallel::~ChCollisionSystemParallel() {
  delete narrowphase;
  delete broadphase;
}

void ChCollisionSystemParallel::Add(ChCollisionModel* model) {
  if (model->GetPhysicsItem()->GetCollide() == true) {
    ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);
    int body_id = pmodel->GetBody()->GetId();
    short2 fam = S2(pmodel->GetFamilyGroup(), pmodel->GetFamilyMask());

    for (int j = 0; j < pmodel->GetNObjects(); j++) {
      real3 obA = pmodel->mData[j].A;
      real3 obB = pmodel->mData[j].B;

      int convex_data_offset = data_container->host_data.convex_data.size();
      data_container->host_data.convex_data.insert(data_container->host_data.convex_data.end(), pmodel->local_convex_data.begin(), pmodel->local_convex_data.end());

      // Compute the global offset of the convex data structure based on the number of points
      // already present
      if (pmodel->mData[j].type == CONVEX) {
        obB.y += convex_data_offset;    // update to get the global offset
      }
      real3 obC = pmodel->mData[j].C;
      real4 obR = pmodel->mData[j].R;
      data_container->host_data.ObA_rigid.push_back(obA);
      data_container->host_data.ObB_rigid.push_back(obB);
      data_container->host_data.ObC_rigid.push_back(obC);
      data_container->host_data.ObR_rigid.push_back(obR);
      data_container->host_data.fam_rigid.push_back(fam);
      data_container->host_data.typ_rigid.push_back(pmodel->mData[j].type);
      data_container->host_data.id_rigid.push_back(body_id);
      data_container->num_shapes++;
    }
  }
}

void ChCollisionSystemParallel::Remove(ChCollisionModel* model) {
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
  //                    data_container->num_models--;
  //                    return;
  //                }
  //            }
  //
  //            //}
}

void ChCollisionSystemParallel::Run() {

  if (data_container->settings.collision.use_aabb_active) {
    custom_vector<bool> body_active(data_container->num_bodies, false);
    GetOverlappingAABB(body_active, data_container->settings.collision.aabb_min, data_container->settings.collision.aabb_max);
    for (int i = 0; i < data_container->host_data.active_data.size(); i++) {
      if (data_container->host_data.active_data[i] == true && data_container->host_data.collide_data[i] == true) {
        data_container->host_data.active_data[i] = body_active[i];
      }
    }
  }

  if (data_container->num_shapes <= 0) {
    return;
  }

  data_container->old_num_contacts = data_container->num_contacts;
  data_container->system_timer.start("collision_broad");

  aabb_generator.GenerateAABB(data_container->host_data.typ_rigid,
                              data_container->host_data.ObA_rigid,
                              data_container->host_data.ObB_rigid,
                              data_container->host_data.ObC_rigid,
                              data_container->host_data.ObR_rigid,
                              data_container->host_data.id_rigid,
                              data_container->host_data.convex_data,
                              data_container->host_data.pos_data,
                              data_container->host_data.rot_data,
                              data_container->settings.collision.collision_envelope,
                              data_container->host_data.aabb_rigid);

  // aabb_generator.GenerateAABBFluid(data_container->host_data.fluid_pos, data_container->fluid_rad, data_container->host_data.aabb_fluid);
  broadphase->setBinsPerAxis(data_container->settings.collision.bins_per_axis);
  broadphase->detectPossibleCollisions(data_container->host_data.aabb_rigid, data_container->host_data.fam_rigid, data_container->host_data.pair_rigid_rigid);
  data_container->system_timer.stop("collision_broad");

  data_container->measures.collision.bin_size_vec = broadphase->bin_size_vec;
  data_container->measures.collision.global_origin = broadphase->global_origin;
  data_container->measures.collision.grid_size = broadphase->grid_size;
  data_container->measures.collision.max_aabb_per_bin = broadphase->max_body_per_bin;
  data_container->measures.collision.max_bounding_point = broadphase->max_bounding_point;
  data_container->measures.collision.min_bounding_point = broadphase->min_bounding_point;
  data_container->measures.collision.numAABB = broadphase->numAABB;

  data_container->system_timer.start("collision_narrow");
  narrowphase->Process(data_container);
  data_container->system_timer.stop("collision_narrow");
}

void ChCollisionSystemParallel::GetOverlappingAABB(custom_vector<bool>& active_id, real3 Amin, real3 Amax) {
  ChCAABBGenerator aabb_generator;

  aabb_generator.GenerateAABB(data_container->host_data.typ_rigid,
                              data_container->host_data.ObA_rigid,
                              data_container->host_data.ObB_rigid,
                              data_container->host_data.ObC_rigid,
                              data_container->host_data.ObR_rigid,
                              data_container->host_data.id_rigid,
                              data_container->host_data.convex_data,
                              data_container->host_data.pos_data,
                              data_container->host_data.rot_data,
                              data_container->settings.collision.collision_envelope,
                              data_container->host_data.aabb_rigid);
#pragma omp parallel for
  for (int i = 0; i < data_container->host_data.typ_rigid.size(); i++) {
    real3 Bmin = data_container->host_data.aabb_rigid[i];
    real3 Bmax = data_container->host_data.aabb_rigid[i + data_container->host_data.typ_rigid.size()];

    bool inContact = (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) && (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
    if (inContact) {
      active_id[data_container->host_data.id_rigid[i]] = true;
    }
  }
}

std::vector<int2> ChCollisionSystemParallel::GetOverlappingPairs() {
  std::vector<int2> pairs;
  pairs.resize(data_container->host_data.pair_rigid_rigid.size());
  for (int i = 0; i < data_container->host_data.pair_rigid_rigid.size(); i++) {
    int2 pair = I2(int(data_container->host_data.pair_rigid_rigid[i] >> 32), int(data_container->host_data.pair_rigid_rigid[i] & 0xffffffff));
    pairs[i] = pair;
  }
  return pairs;
}

}    // END_OF_NAMESPACE____
}    // END_OF_NAMESPACE____
