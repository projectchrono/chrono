//////////////////////////////////////////////////
//
//   ChCCollisionSystemGPU->cpp
//
// ------------------------------------------------
//       Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/collision/ChCollision.h"

namespace chrono {
namespace collision {

ChCollisionSystemParallel::ChCollisionSystemParallel(ChParallelDataManager* dm) : data_manager(dm) {}

ChCollisionSystemParallel::~ChCollisionSystemParallel() {}

void ChCollisionSystemParallel::Add(ChCollisionModel* model) {
    if (model->GetPhysicsItem()->GetCollide() == true) {
        ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);
        int body_id = pmodel->GetBody()->GetId();
        short2 fam = S2(pmodel->GetFamilyGroup(), pmodel->GetFamilyMask());
        // The offset for this shape will the current total number of points in
        // the convex data list
        int convex_data_offset = data_manager->shape_data.convex_rigid.size();
        // Insert the points into the global convex list
        data_manager->shape_data.convex_rigid.insert(data_manager->shape_data.convex_rigid.end(),
                                                     pmodel->local_convex_data.begin(),
                                                     pmodel->local_convex_data.end());

        for (int j = 0; j < pmodel->GetNObjects(); j++) {
            real3 obA = pmodel->mData[j].A;
            real3 obB = pmodel->mData[j].B;
            real3 obC = pmodel->mData[j].C;
            int length = 1;
            int start = 0;
            // Compute the global offset of the convex data structure based on the number of points
            // already present

            switch (pmodel->mData[j].type) {
                case chrono::collision::SPHERE:
                    start = data_manager->shape_data.sphere_rigid.size();
                    data_manager->shape_data.sphere_rigid.push_back(obB.x);
                    break;
                case chrono::collision::ELLIPSOID:
                    start = data_manager->shape_data.box_like_rigid.size();
                    data_manager->shape_data.box_like_rigid.push_back(obB);
                    break;
                case chrono::collision::BOX:
                    start = data_manager->shape_data.box_like_rigid.size();
                    data_manager->shape_data.box_like_rigid.push_back(obB);
                    break;
                case chrono::collision::CYLINDER:
                    start = data_manager->shape_data.box_like_rigid.size();
                    data_manager->shape_data.box_like_rigid.push_back(obB);
                    break;
                case chrono::collision::CONE:
                    start = data_manager->shape_data.box_like_rigid.size();
                    data_manager->shape_data.box_like_rigid.push_back(obB);
                    break;
                case chrono::collision::CAPSULE:
                    start = data_manager->shape_data.capsule_rigid.size();
                    data_manager->shape_data.capsule_rigid.push_back(real2(obB.x, obB.y));
                    break;
                case chrono::collision::ROUNDEDBOX:
                    start = data_manager->shape_data.rbox_like_rigid.size();
                    data_manager->shape_data.rbox_like_rigid.push_back(real4(obB, obC.x));
                    break;
                case chrono::collision::ROUNDEDCYL:
                    start = data_manager->shape_data.rbox_like_rigid.size();
                    data_manager->shape_data.rbox_like_rigid.push_back(real4(obB, obC.x));
                    break;
                case chrono::collision::ROUNDEDCONE:
                    start = data_manager->shape_data.rbox_like_rigid.size();
                    data_manager->shape_data.rbox_like_rigid.push_back(real4(obB, obC.x));
                    break;
                case chrono::collision::CONVEX:
                    start = obB.y + convex_data_offset;
                    length = obB.x;
                    break;
                case chrono::collision::TRIANGLEMESH:
                    start = data_manager->shape_data.triangle_rigid.size();
                    data_manager->shape_data.triangle_rigid.push_back(obA);
                    data_manager->shape_data.triangle_rigid.push_back(obB);
                    data_manager->shape_data.triangle_rigid.push_back(obC);
                    break;
            }

            data_manager->shape_data.ObA_rigid.push_back(obA);
            data_manager->shape_data.ObR_rigid.push_back(pmodel->mData[j].R);
            data_manager->shape_data.start_rigid.push_back(start);
            data_manager->shape_data.length_rigid.push_back(length);

            data_manager->shape_data.fam_rigid.push_back(fam);
            data_manager->shape_data.typ_rigid.push_back(pmodel->mData[j].type);
            data_manager->shape_data.id_rigid.push_back(body_id);
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
        body_active.resize(data_manager->num_rigid_bodies);
        std::fill(body_active.begin(), body_active.end(), false);

        GetOverlappingAABB(body_active, data_manager->settings.collision.aabb_min,
                           data_manager->settings.collision.aabb_max);

#pragma omp parallel for
        for (int i = 0; i < data_manager->host_data.active_rigid.size(); i++) {
            if (data_manager->host_data.active_rigid[i] == true && data_manager->host_data.collide_rigid[i] == true) {
                data_manager->host_data.active_rigid[i] = body_active[i];
            }
        }
    }
    data_manager->system_timer.start("collision_broad");
    data_manager->aabb_generator->GenerateAABB();

    // Compute the bounding box of things
    data_manager->broadphase->DetermineBoundingBox();
    data_manager->broadphase->OffsetAABB();
    data_manager->broadphase->ComputeTopLevelResolution();
    // Everything is offset and ready to go!
    data_manager->broadphase->DispatchRigid();

    data_manager->system_timer.stop("collision_broad");

    data_manager->system_timer.start("collision_narrow");
    if (data_manager->num_fluid_bodies != 0) {
        data_manager->narrowphase->DispatchFluid();
    }
    if (data_manager->num_fea_tets != 0) {
        // narrowphase->DispatchTets();
    }
    if (data_manager->num_rigid_shapes != 0) {
        data_manager->narrowphase->ProcessRigids();

    } else {
        data_manager->host_data.c_counts_rigid_tet.clear();
        data_manager->host_data.c_counts_rigid_fluid.clear();
        data_manager->num_rigid_tet_contacts = 0;
        data_manager->num_rigid_fluid_contacts = 0;
    }

    data_manager->system_timer.stop("collision_narrow");
}

void ChCollisionSystemParallel::GetOverlappingAABB(custom_vector<char>& active_id, real3 Amin, real3 Amax) {
    data_manager->aabb_generator->GenerateAABB();
#pragma omp parallel for
    for (int i = 0; i < data_manager->shape_data.typ_rigid.size(); i++) {
        real3 Bmin = data_manager->host_data.aabb_min[i];
        real3 Bmax = data_manager->host_data.aabb_max[i];

        bool inContact = (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) &&
                         (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
        if (inContact) {
            active_id[data_manager->shape_data.id_rigid[i]] = true;
        }
    }
}

std::vector<vec2> ChCollisionSystemParallel::GetOverlappingPairs() {
    std::vector<vec2> pairs;
    pairs.resize(data_manager->host_data.contact_pairs.size());
    for (int i = 0; i < data_manager->host_data.contact_pairs.size(); i++) {
        vec2 pair = I2(int(data_manager->host_data.contact_pairs[i] >> 32),
                       int(data_manager->host_data.contact_pairs[i] & 0xffffffff));
        pairs[i] = pair;
    }
    return pairs;
}

}  // END_OF_NAMESPACE____
}  // END_OF_NAMESPACE____
