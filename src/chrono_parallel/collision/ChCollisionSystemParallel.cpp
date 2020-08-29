// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Parallel collsion system that calls a custom AABB generator,
// broadphase and narrowphase
//
// =============================================================================

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
        int convex_data_offset = (int)data_manager->shape_data.convex_rigid.size();
        // Insert the points into the global convex list
        data_manager->shape_data.convex_rigid.insert(data_manager->shape_data.convex_rigid.end(),
                                                     pmodel->local_convex_data.begin(),
                                                     pmodel->local_convex_data.end());

        // Shape index in the collision model
        int local_shape_index = 0;

        for (auto s : pmodel->GetShapes()) {
            auto shape = std::static_pointer_cast<ChCollisionShapeParallel>(s);
            real3 obA = shape->A;
            real3 obB = shape->B;
            real3 obC = shape->C;
            int length = 1;
            int start;
            // Compute the global offset of the convex data structure based on the number of points
            // already present

            switch (shape->GetType()) {
                case ChCollisionShape::Type::SPHERE:
                    start = (int)data_manager->shape_data.sphere_rigid.size();
                    data_manager->shape_data.sphere_rigid.push_back(obB.x);
                    break;
                case ChCollisionShape::Type::ELLIPSOID:
                    start = (int)data_manager->shape_data.box_like_rigid.size();
                    data_manager->shape_data.box_like_rigid.push_back(obB);
                    break;
                case ChCollisionShape::Type::BOX:
                    start = (int)data_manager->shape_data.box_like_rigid.size();
                    data_manager->shape_data.box_like_rigid.push_back(obB);
                    break;
                case ChCollisionShape::Type::CYLINDER:
                    start = (int)data_manager->shape_data.box_like_rigid.size();
                    data_manager->shape_data.box_like_rigid.push_back(obB);
                    break;
                case ChCollisionShape::Type::CONE:
                    start = (int)data_manager->shape_data.box_like_rigid.size();
                    data_manager->shape_data.box_like_rigid.push_back(obB);
                    break;
                case ChCollisionShape::Type::CAPSULE:
                    start = (int)data_manager->shape_data.capsule_rigid.size();
                    data_manager->shape_data.capsule_rigid.push_back(real2(obB.x, obB.y));
                    break;
                case ChCollisionShape::Type::ROUNDEDBOX:
                    start = (int)data_manager->shape_data.rbox_like_rigid.size();
                    data_manager->shape_data.rbox_like_rigid.push_back(real4(obB, obC.x));
                    break;
                case ChCollisionShape::Type::ROUNDEDCYL:
                    start = (int)data_manager->shape_data.rbox_like_rigid.size();
                    data_manager->shape_data.rbox_like_rigid.push_back(real4(obB, obC.x));
                    break;
                case ChCollisionShape::Type::ROUNDEDCONE:
                    start = (int)data_manager->shape_data.rbox_like_rigid.size();
                    data_manager->shape_data.rbox_like_rigid.push_back(real4(obB, obC.x));
                    break;
                case ChCollisionShape::Type::CONVEX:
                    start = (int)(obB.y + convex_data_offset);
                    length = (int)obB.x;
                    break;
                case ChCollisionShape::Type::TRIANGLE:
                    start = (int)data_manager->shape_data.triangle_rigid.size();
                    data_manager->shape_data.triangle_rigid.push_back(obA);
                    data_manager->shape_data.triangle_rigid.push_back(obB);
                    data_manager->shape_data.triangle_rigid.push_back(obC);
                    break;
                default:
                    start = -1;
                    break;
            }

            data_manager->shape_data.ObA_rigid.push_back(obA);
            data_manager->shape_data.ObR_rigid.push_back(shape->R);
            data_manager->shape_data.start_rigid.push_back(start);
            data_manager->shape_data.length_rigid.push_back(length);

            data_manager->shape_data.fam_rigid.push_back(fam);
            data_manager->shape_data.typ_rigid.push_back(shape->GetType());
            data_manager->shape_data.id_rigid.push_back(body_id);
            data_manager->shape_data.local_rigid.push_back(local_shape_index);
            data_manager->num_rigid_shapes++;
            local_shape_index++;
        }
    }
}

#define ERASE_MACRO(x, y) x.erase(x.begin() + y);
#define ERASE_MACRO_LEN(x, y, z) x.erase(x.begin() + y, x.begin() + y + z);

void ChCollisionSystemParallel::Remove(ChCollisionModel* model) {
    /*
    ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);
    int body_id = pmodel->GetBody()->GetId();
    //loop over the models we nned to remove
    //std::cout << "removing: " << pmodel->GetNumShapes() << " objects" << std::endl;
    for (int j = 0; j < pmodel->GetNumShapes(); j++) {
        //find a model to remove
        bool removed = false;
        for (int i = 0; i < data_manager->shape_data.id_rigid.size(); i++) {
            if (data_manager->shape_data.id_rigid[i] == body_id) {
                int index = i;
                data_manager->num_rigid_shapes--;

                int start = data_manager->shape_data.start_rigid[index];
                int length = data_manager->shape_data.length_rigid[index];
                int type = data_manager->shape_data.typ_rigid[index];

                //std::cout << "removing: type " << type << " " << start<< " " <<j << std::endl;


                switch (type) {
                case chrono::collision::SPHERE:
                    ERASE_MACRO_LEN(data_manager->shape_data.sphere_rigid, start, length);
                    break;
                case chrono::collision::ELLIPSOID:
                    ERASE_MACRO_LEN(data_manager->shape_data.box_like_rigid, start, length);
                    break;
                case chrono::collision::BOX:
                    ERASE_MACRO_LEN(data_manager->shape_data.box_like_rigid, start, length);
                    break;
                case chrono::collision::CYLINDER:
                    ERASE_MACRO_LEN(data_manager->shape_data.box_like_rigid, start, length);
                    break;
                case chrono::collision::CONE:
                    ERASE_MACRO_LEN(data_manager->shape_data.box_like_rigid, start, length);
                    break;
                case chrono::collision::CAPSULE:
                    ERASE_MACRO_LEN(data_manager->shape_data.capsule_rigid, start, length);
                    break;
                case chrono::collision::ROUNDEDBOX:
                    ERASE_MACRO_LEN(data_manager->shape_data.rbox_like_rigid, start, length);
                    break;
                case chrono::collision::ROUNDEDCYL:
                    ERASE_MACRO_LEN(data_manager->shape_data.rbox_like_rigid, start, length);
                    break;
                case chrono::collision::ROUNDEDCONE:
                    ERASE_MACRO_LEN(data_manager->shape_data.rbox_like_rigid, start, length);
                    break;
                case chrono::collision::CONVEX:
                    ERASE_MACRO_LEN(data_manager->shape_data.convex_rigid, start, length);
                    break;
                case chrono::collision::TRIANGLE:
                    ERASE_MACRO_LEN(data_manager->shape_data.convex_rigid, start, 3);
                    break;
                }

                ERASE_MACRO(data_manager->shape_data.ObA_rigid, index);
                ERASE_MACRO(data_manager->shape_data.ObR_rigid, index);
                ERASE_MACRO(data_manager->shape_data.start_rigid, index);
                ERASE_MACRO(data_manager->shape_data.length_rigid, index);

                ERASE_MACRO(data_manager->shape_data.fam_rigid, index);
                ERASE_MACRO(data_manager->shape_data.typ_rigid, index);
                ERASE_MACRO(data_manager->shape_data.id_rigid, index);
                removed = true;
                break;
            }
        }
        //std::cout << "decrement start "<< std::endl;
        if (removed) {
            //we removed a model, all of the starts are off by one, decrement all starts before removing a second model
            for (int i = 0; i < data_manager->shape_data.start_rigid.size(); i++) {
                if (data_manager->shape_data.start_rigid[i] != 0) {
                    data_manager->shape_data.start_rigid[i] -= 1;
                }
            }
        }

    }
*/
}
#undef ERASE_MACRO
#undef ERASE_MACRO_LEN

void ChCollisionSystemParallel::Run() {
    LOG(INFO) << "ChCollisionSystemParallel::Run()";
    if (data_manager->settings.collision.use_aabb_active) {
        body_active.resize(data_manager->num_rigid_bodies);
        std::fill(body_active.begin(), body_active.end(), false);

        GetOverlappingAABB(body_active, data_manager->settings.collision.aabb_min,
                           data_manager->settings.collision.aabb_max);

#pragma omp parallel for
        for (int i = 0; i < data_manager->host_data.active_rigid.size(); i++) {
            if (data_manager->host_data.active_rigid[i] != 0 && data_manager->host_data.collide_rigid[i] != 0) {
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

void ChCollisionSystemParallel::GetBoundingBox(ChVector<>& aabb_min, ChVector<>& aabb_max) const {
    aabb_min.x() = data_manager->measures.collision.min_bounding_point.x;
    aabb_min.y() = data_manager->measures.collision.min_bounding_point.y;
    aabb_min.z() = data_manager->measures.collision.min_bounding_point.z;

    aabb_max.x() = data_manager->measures.collision.max_bounding_point.x;
    aabb_max.y() = data_manager->measures.collision.max_bounding_point.y;
    aabb_max.z() = data_manager->measures.collision.max_bounding_point.z;
}

void ChCollisionSystemParallel::ReportContacts(ChContactContainer* mcontactcontainer) {
    assert(dynamic_cast<ChContactContainerParallel*>(mcontactcontainer));

    // Resize global arrays with composite material properties.
    // NOTE: important to do this here, to set size to zero if no contacts (in case some other added by a custom user
    // callback)
    mcontactcontainer->BeginAddContact();

    uint num_contacts = data_manager->num_rigid_contacts;
    if (num_contacts <= 0) {
        return;
    }

    auto container = static_cast<ChContactContainerParallel*>(mcontactcontainer);

    auto& bids = data_manager->host_data.bids_rigid_rigid;  // global IDs of bodies in contact
    auto& abody = data_manager->host_data.active_rigid;     // flags for active bodies
    auto& sids = data_manager->host_data.contact_pairs;     // global IDs of shapes in contact
    auto& sindex = data_manager->shape_data.local_rigid;    // collision model indexes of shapes in contact

    // Loop over all current contacts, create the composite material, and load material properties in the data manager
    // (on a per contact basis). Snce this is contact method-specific, we defer to the underlying contact container.
#pragma omp parallel for
    for (int i = 0; i < (signed)num_contacts; i++) {
        auto b1 = bids[i].x;                  // global IDs of bodies in contact
        auto b2 = bids[i].y;                  //
        auto s1 = int(sids[i] >> 32);         // global IDs of shapes in contact
        auto s2 = int(sids[i] & 0xffffffff);  //
        auto s1_index = sindex[s1];           // collision model indexes of shapes in contact
        auto s2_index = sindex[s2];           //

        container->AddContact(i, b1, s1, b2, s2);
    }

    mcontactcontainer->EndAddContact();
}

double ChCollisionSystemParallel::GetTimerCollisionBroad() const {
    return data_manager->system_timer.GetTime("collision_broad");
}

double ChCollisionSystemParallel::GetTimerCollisionNarrow() const {
    return data_manager->system_timer.GetTime("collision_narrow");
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

void ChCollisionSystemParallel::SetAABB(real3 aabbmin, real3 aabbmax) {
    data_manager->settings.collision.aabb_min = aabbmin;
    data_manager->settings.collision.aabb_max = aabbmax;
    data_manager->settings.collision.use_aabb_active = true;
}

bool ChCollisionSystemParallel::GetAABB(real3& aabbmin, real3& aabbmax) {
    aabbmin = data_manager->settings.collision.aabb_min;
    aabbmax = data_manager->settings.collision.aabb_max;

    return data_manager->settings.collision.use_aabb_active;
}

}  // end namespace collision
}  // end namespace chrono
