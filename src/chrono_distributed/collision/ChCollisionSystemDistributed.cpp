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

#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChBroadphaseUtils.h"
#include "chrono_parallel/collision/ChCollisionModelParallel.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"

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
    ChCollisionModelDistributed* pmodel = static_cast<ChCollisionModelDistributed*>(model);
    // Find space in ddm vectors - need one index for both start and count
    // need a chunk of body_shapes large enough for all shapes on this body
    int my_rank = ddm->my_sys->GetMyRank();
    int body_index = pmodel->GetBody()->GetId();  // TODO assuming this is set: see calling fxn
    int needed_count = pmodel->GetNObjects();     // Minimum size needed in ddm->body_shapes
    ChVector<> pos(pmodel->GetBody()->GetPos());

    /* TODO Only include shapes relevant to this sub-domain
        if (ddm->comm_status[body_index] == distributed::GLOBAL) {
            int splitaxis = my_sys->GetDomain()->GetSplitAxis();
            ChVector<> subhi(my_sys->GetDomain()->GetSubHi());
            ChVector<> sublo(my_sys->GetDomain()->GetSublo());
            double ghost_layer = my_sys->GetGhostLayer();
            for (int i = 0; i < needed_count; i++) {
                // TODO: If in this sub-domain, add to new pmodel
                real3 A = pmodel.mdata[i].A;
                quaternion R = pmodel.mdata[i].R;
                // TODO posalongsplitaxis
                if (posalongsplitaxis >= sublo[splitaxis] - ghost_layer &&
                    posalongsplitaxis <= subhi[splitaxis] + ghost_layer) {
                        // Add to new model
                        newmodel->Add
                }
            }
        }
    */

    // Find free chunk in ddm->body_shapes large enough for this model
    bool found = false;
    struct LocalShapeNode* curr = ddm->local_free_shapes;
    struct LocalShapeNode* prev = NULL;

    while (curr != NULL && !found) {
        // If curr is a large enough free chunk
        if (curr->free && curr->size >= needed_count) {
            found = true;
            curr->free = false;

            // If there is leftover space, insert a new node in the free list
            if (curr->size != needed_count) {
                struct LocalShapeNode* new_next = new struct LocalShapeNode();
                new_next->free = true;
                new_next->size = curr->size - needed_count;
                new_next->next = curr->next;
                new_next->body_shapes_index = curr->body_shapes_index + needed_count;

                curr->next = new_next;
                curr->size = needed_count;
            }
        }
        // If curr is not free or not large enough
        else {
            prev = curr;
            curr = curr->next;
        }
    }
    // NOTE: curr could be NULL for two reasons: empty list and going off the end.
    // found == true => curr identifies a large-enough free chunk

    // The first index of the free chunk of body_shapes
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
        ddm->body_shape_count[body_index] = needed_count;

        // Add a node at the end
        struct LocalShapeNode* new_end;

        // If the above while loop went off the end searching
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

        ddm->body_shape_start[body_index] = ddm->body_shapes.size();

        begin_shapes = new_end->body_shapes_index;

        // Create the space in body_shapes
        for (int i = 0; i < needed_count; i++) {
            ddm->body_shapes.push_back(0);
        }
    }

    /*
     * NOTE: At this point there is a large-enough chunk in ddm->body_shapes that is NOT set
     * beginning at begin_shapes
     */

    // TODO need places for ALL shapes in the model in data_manager->shape_data,
    // else need to call collsyspar::add

    // Check for free spaces to insert into (DO need same shape type else can't deactivate that)
    std::vector<int> free_dm_shapes;
    for (int i = 0; i < needed_count; i++) {
        // i identifies a shape in the model

        // Search data_manager->shape_data for a free and shape-matching spot
        for (int j = 0; j < dm->shape_data.id_rigid.size(); j++) {
            // If the index in the data manager is open and corresponds to the same shape type
            if (dm->shape_data.id_rigid[j] == UINT_MAX && dm->shape_data.typ_rigid[j] == pmodel->mData[i].type) {
                free_dm_shapes.push_back(j);
            }
        }  // End for loop over the slots in data_manager->shape_data
    }      // End for loop over the shapes for this model

    // If there is space for ALL shapes in the model in the data_manager
    // unload them.
    // TODO needed_count == 0... add called too early
    if (free_dm_shapes.size() == needed_count) {
#ifdef DistrDebug
        GetLog() << "needed_count " << needed_count << "\n";
#endif
        for (int i = 0; i < needed_count; i++) {
            GetLog() << "BIG DEAL\n";
            // i identifies a shape in the model

            int j = free_dm_shapes[i];  // Index into dm->shape_data

            GetLog() << "Setting id_rigid. gid " << pmodel->GetBody()->GetGid() << " local id " << body_index << "\n";

            dm->shape_data.id_rigid[j] = body_index;
            ddm->body_shapes[begin_shapes] = j;
            begin_shapes++;
            ddm->dm_free_shapes[j] = false;

            // type_rigid and start_rigid are unchanged because the shape type is the same
            int start = dm->shape_data.start_rigid[j];

            real3 obA = pmodel->mData[i].A;
            real3 obB = pmodel->mData[i].B;
            real3 obC = pmodel->mData[i].C;

            short2 fam = S2(pmodel->GetFamilyGroup(), pmodel->GetFamilyMask());

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
                    GetLog() << "Adding box\n";
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
            dm->shape_data.fam_rigid[j] = fam;
        }
    }
    // If there was not enough space in the data_manager for all shapes in the model,
    // call the regular add
    else {
#ifdef DistrDebug
        GetLog() << "ChCollisionSystemParallel::Add GID " << pmodel->GetBody()->GetGid() << "\n";
#endif
        this->ChCollisionSystemParallel::Add(model);
        for (int i = 0; i < needed_count; i++) {
            ddm->dm_free_shapes.push_back(false);
            ddm->body_shapes[begin_shapes] = dm->shape_data.id_rigid.size() - needed_count + i;  // TODO ?
            begin_shapes++;
        }
    }
}

// Id must be set before calling
// Deactivates all shapes associated with the collision model
void ChCollisionSystemDistributed::Remove(ChCollisionModel* model) {
    ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);
    uint id = pmodel->GetBody()->GetId();  // TODO could be wrong model with this ID OR could be calling Remove on the
                                           // body at wrong time TODO called by clearmodel

    int count = pmodel->GetNObjects();
    int start = ddm->body_shape_start[id];

    for (int i = 0; i < count; i++) {
        int index = start + i;
        ddm->dm_free_shapes[ddm->body_shapes[index]] = true;  // Marks the spot in data_manager->shape_data as open
#ifdef DistrDebug
        // Forces collision detection to ignore this shape
        GetLog() << "Deactivating shape for local ID " << id << " GID " << ddm->global_id[id] << " Rank "
                 << ddm->my_sys->GetMyRank() << "\n";
#endif
        ddm->data_manager->shape_data.id_rigid[ddm->body_shapes[index]] = UINT_MAX;
    }

    // TODO better Search
    struct LocalShapeNode* curr = ddm->local_free_shapes;
    while (curr != NULL) {
        if (curr->body_shapes_index == start) {
            curr->free = true;
            break;
        }
        curr = curr->next;
    }
}
