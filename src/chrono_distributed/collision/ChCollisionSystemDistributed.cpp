// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include <climits>

#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"
#include "chrono_distributed/ChDistributedDataManager.h"

#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/collision/ChCollisionModelParallel.h"
#include "chrono_parallel/ChDataManager.h"

using namespace chrono;
using namespace collision;

ChCollisionSystemDistributed::ChCollisionSystemDistributed(ChParallelDataManager* dm, ChDistributedDataManager* ddm)
    : ChCollisionSystemParallel(dm) {
    this->ddm = ddm;
    this->ddm->local_free_shapes = NULL;
}

ChCollisionSystemDistributed::~ChCollisionSystemDistributed() {}

// Called by chcollisionmodel::buildmodel (if system set), chbody::setcollide(true), chbody::setsystem (if system set)
// (called by addbody AND addbodyexchange)
void ChCollisionSystemDistributed::Add(ChCollisionModel* model) {
    ChParallelDataManager* dm = ddm->data_manager;
    ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);
    // Find space in ddm vectors - need one index for both start and count
    // need a chunk of body_shapes large enough for all shapes on this body

    int my_rank = ddm->my_sys->GetMyRank();

    // Can't assume model is associated with a body_shapes?
    int body_index = pmodel->GetBody()->GetId();
    int needed_count = pmodel->GetNObjects();

    bool found = false;
    struct LocalShapeNode* curr = ddm->local_free_shapes;
    struct LocalShapeNode* prev = NULL;

    // TODO could add counter for total frees spaces and do some compression if it gets too fragmented

    while (curr != NULL && !found) {
        if (curr->free && curr->size >= needed_count) {
            found = true;
            curr->free = false;

            // If there is leftover space, make a new node
            if (curr->size != needed_count) {
                struct LocalShapeNode* new_next = new struct LocalShapeNode();
                new_next->free = true;
                new_next->size = curr->size - needed_count;
                new_next->next = curr->next;
                new_next->body_shapes_index = curr->body_shapes_index + needed_count;

                curr->next = new_next;
                curr->size = needed_count;
            }
        } else {
            prev = curr;
            curr = curr->next;
        }
    }  // curr could be null from empty list OR going off the end?

    // The first free index of the portion of body_shapes
    int begin_shapes;
    // If a free portion of body_shapes was found, set up the body to use it
    if (found) {
        ddm->body_shape_start[body_index] = curr->body_shapes_index;
        ddm->body_shape_count[body_index] = needed_count;
        begin_shapes = curr->body_shapes_index;
    }
    // If there was no free portion of body_shapes large enough, create more space
    // in body_shapes and a new free-list node
    else {
        ddm->body_shape_count[body_index] = needed_count;  // TODO

        struct LocalShapeNode* new_end;
        // Add a node at the end
        if (curr == NULL && prev != NULL) {
            new_end = new struct LocalShapeNode();
            prev->next = new_end;
        }
        // If the first node doesn't exist yet
        else if (curr == NULL) {
            // Create first node
            ddm->local_free_shapes = new struct LocalShapeNode();
            new_end = ddm->local_free_shapes;
        }
        new_end->size = needed_count;
        new_end->free = false;
        new_end->body_shapes_index = ddm->body_shapes.size();  // This portion will begin right
                                                               // after the end of the current vector

        ddm->body_shape_start[body_index] =
            ddm->body_shapes.size();  // TODO make sure bsstart and bscount are pushed back at body add

        begin_shapes = new_end->body_shapes_index;

        // Create the space in body_shapes
        for (int i = 0; i < needed_count; i++) {
            ddm->body_shapes.push_back(0);
        }
    }

    // At this point there is space in ddm->body_shapes that is NOT set

    // TODO need places for ALL shapes in the model, else need to call collsyspar::add

    // Check for free spaces to insert into (DO need same shape type else can't deactivate that)
    std::vector<int> free_dm_shapes;
    for (int i = 0; i < needed_count; i++) {
        // Search data_manager->shape_data for an open and shape-matching spot
        for (int j = 0; j < dm->shape_data.id_rigid.size(); j++) {
            // If the index in the data manager is open and corresponds to the same shape type
            if (dm->shape_data.id_rigid[j] == UINT_MAX && dm->shape_data.typ_rigid[j] == pmodel->mData[i].type) {
                free_dm_shapes.push_back(j);
            }
        }  // End of for loop over the slots in data_manager->shape_data
    }      // End of for loop over the shapes for this model

    // If there is space for ALL shapes in the model in the data_manager
    // unload them.
    if (free_dm_shapes.size() == needed_count) {
        for (int i = 0; i < needed_count; i++) {
            int j = free_dm_shapes[i];
            dm->shape_data.id_rigid[j] = body_index;
            ddm->body_shapes[begin_shapes] = j;
            begin_shapes++;
            ddm->dm_free_shapes[j] = false;

            // type_rigid and start_rigid are unchanged because the shape type is the same
            int start = dm->shape_data.start_rigid[j];

            real3 obA = pmodel->mData[i].A;
            real3 obB = pmodel->mData[i].B;
            real3 obC = pmodel->mData[i].C;

            switch (pmodel->mData[i].type) {
                case chrono::collision::SPHERE:
#ifdef DistrDebug
                    GetLog() << "Adding sphere\n";
#endif
                    dm->shape_data.sphere_rigid[start] = obB.x;
                    break;
                case chrono::collision::ELLIPSOID:
                    dm->shape_data.box_like_rigid[start] = obB;
                    break;
                case chrono::collision::BOX:
                    dm->shape_data.box_like_rigid[start] = obB;
                    break;
                case chrono::collision::CYLINDER:
                    dm->shape_data.box_like_rigid[start] = obB;
                    break;
                case chrono::collision::CONE:
                    dm->shape_data.box_like_rigid[start] = obB;
                    break;
                case chrono::collision::CAPSULE:
                    dm->shape_data.capsule_rigid[start] = real2(obB.x, obB.y);
                    break;
                case chrono::collision::ROUNDEDBOX:
                    dm->shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
                    break;
                case chrono::collision::ROUNDEDCYL:
                    dm->shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
                    break;
                case chrono::collision::ROUNDEDCONE:
                    dm->shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
                    break;
                default:
                    ddm->my_sys->ErrorAbort("Shape not supported\n");
            }

            dm->shape_data.ObA_rigid[j] = obA;
            dm->shape_data.ObR_rigid[j] = pmodel->mData[i].R;
        }
    }
    // If there was not enough space in the data_manager for all shapes in the model,
    // call the regular add
    else {
        // TODO if (not enough slots for ALL shapes in model)
        GetLog() << "Calling Add rank " << my_rank << "\n";
        this->ChCollisionSystemParallel::Add(model);
        GetLog() << "Returning from Add rank " << my_rank << "\n";
        for (int i = 0; i < needed_count; i++) {
            ddm->dm_free_shapes.push_back(false);
            ddm->body_shapes[begin_shapes] = dm->shape_data.id_rigid.size() - needed_count + i;  // TODO ?
            begin_shapes++;
        }
    }
}

// Deactivates all shapes associated with the collision model
void ChCollisionSystemDistributed::Remove(ChCollisionModel* model) {
    ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);

    uint id = pmodel->GetBody()->GetId();
    int count = pmodel->GetNObjects();
    int start = ddm->body_shape_start[id];

    for (int i = 0; i < count; i++) {
        int index = start + i;
        /*ddm->my_free_shapes[index] = true;                    // Marks the spot in ddm->body_shapes as open*/
        ddm->dm_free_shapes[ddm->body_shapes[index]] = true;  // Marks the spot in data_manager->shape_data as open

        // Forces collision detection to ignore this shape
        ddm->data_manager->shape_data.id_rigid[ddm->body_shapes[index]] = UINT_MAX;
    }

    // TODO better Search
    struct LocalShapeNode* curr = ddm->local_free_shapes;
    while (curr != NULL) {
        if (curr->body_shapes_index == start) {
            curr->free = false;
            break;
        }
        curr = curr->next;
    }
}
