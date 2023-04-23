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

#include "chrono/collision/ChCollisionModelChrono.h"

#include "chrono_multicore/ChDataManager.h"

namespace chrono {
namespace collision {

ChCollisionSystemDistributed::ChCollisionSystemDistributed(ChMulticoreDataManager* dm, ChDistributedDataManager* ddm)
    : ChCollisionSystemChronoMulticore(dm) {
    this->ddm = ddm;
    // TODO replace
    this->ddm->local_free_shapes = NULL;
}

ChCollisionSystemDistributed::~ChCollisionSystemDistributed() {}

// Called by chcollisionmodel::buildmodel (if system set), chbody::setcollide(true), chbody::setsystem (if system set)
// (called by addbody AND addbodyexchange)
// TODO VERY EXPENSIVE
void ChCollisionSystemDistributed::Add(ChCollisionModel* model) {
    ChMulticoreDataManager* dm = ddm->data_manager;
    ChCollisionModelDistributed* pmodel = static_cast<ChCollisionModelDistributed*>(model);

    auto& shape_data = dm->cd_data->shape_data;

    // Find space in ddm vectors - need one index for both start and count
    // need a chunk of body_shapes large enough for all shapes on this body
    int body_index = pmodel->GetBody()->GetId();  // NOTE: Assumes this is set
    int needed_count = pmodel->GetNumShapes();    // Minimum size needed in ddm->body_shapes
    ChVector<> pos(pmodel->GetBody()->GetPos());

    // TODO Only include shapes relevant to this sub-domain

    bool found = false;
    struct LocalShapeNode* curr = ddm->local_free_shapes;
    struct LocalShapeNode* prev = NULL;

    // Find free chunk in ddm->body_shapes large enough for this model using free list TODO free list expensive
    // TODO consider first set of adds: could be adding a bunch of list nodes that just say one slot is full. Should
    // consolidate them together to say that a big chunk is taken.
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
            }
            curr->size = needed_count;
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
    int begin_shapes = 0;

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
            // If the previous chunk is not free, merge new and previous
            if (prev->free == false) {
                prev->size += needed_count;
                // This portion will begin right after the end of the current vector
                begin_shapes = static_cast<int>(ddm->body_shapes.size());
            }
            // If the previous chunk is free, add new node at end
            else {
                new_end = new struct LocalShapeNode();
                prev->next = new_end;
                new_end->size = needed_count;
                new_end->free = false;
                // This portion will begin right after the end of the current body_shapes
                new_end->body_shapes_index = static_cast<int>(ddm->body_shapes.size());
                new_end->next = NULL;
                begin_shapes = new_end->body_shapes_index;
            }
        }
        // If the first node doesn't exist yet
        else if (curr == NULL) {
            // Create first node
            ddm->local_free_shapes = new struct LocalShapeNode();
            new_end = ddm->local_free_shapes;
            new_end->size = needed_count;
            new_end->free = false;
            // This portion will begin right after the end of the current vector
            // TODO should be 0
            new_end->body_shapes_index = static_cast<int>(ddm->body_shapes.size());
            new_end->next = NULL;
            begin_shapes = new_end->body_shapes_index;  // TODO should be 0
        }

        ddm->body_shape_start[body_index] = static_cast<int>(ddm->body_shapes.size());

        // Create the space in body_shapes
        for (int i = 0; i < needed_count; i++) {
            ddm->body_shapes.push_back(0);
        }
    }

    /*
     * NOTE: At this point there is a large-enough chunk in ddm->body_shapes that is NOT set
     * beginning at begin_shapes
     */

    // TODO need places for ALL shapes in the model in data_manager->cd_data->shape_data,
    // else need to call collsysPAR::add

    // Check for free spaces to insert into (DO need same shape type else can't deactivate that)
    std::vector<int> free_dm_shapes;
    if (!ddm->initial_add) {
        for (int i = 0; i < needed_count; i++) {
            // i identifies a shape in the MODEL

            // TODO THIS IS THE EXPENSIVE PART
            // Search shape_data for a free and shape-matching spot
            for (int j = 0; j < shape_data.id_rigid.size(); j++) {
                // If the index in the data manager is open and corresponds to the same shape type
                if (shape_data.id_rigid[j] == UINT_MAX &&
                    shape_data.typ_rigid[j] == pmodel->GetShape(i)->GetType()) {
                    free_dm_shapes.push_back(j);
                    break;  // Found spot for this shape, break inner loop to get new i (shape)
                }
                // TODO: Early break from both loops if a spot cannot be found for a shape
            }
        }
    }
    // If there is space for ALL shapes in the model in the data_manager
    // unload them.
    if (free_dm_shapes.size() == needed_count) {
        for (int i = 0; i < needed_count; i++) {
            // i identifies a shape in the MODEL
            auto shape = std::static_pointer_cast<ChCollisionShapeChrono>(pmodel->GetShape(i));

            int j = free_dm_shapes[i];  // Index into shape_data

            shape_data.id_rigid[j] = body_index;
            ddm->body_shapes[begin_shapes + i] = j;
            ddm->dm_free_shapes[j] = false;

            // type_rigid and start_rigid are unchanged because the shape type is the same
            int start = shape_data.start_rigid[j];

            real3 obA = shape->A;
            real3 obB = shape->B;
            real3 obC = shape->C;

            short2 fam = S2(pmodel->GetFamilyGroup(), pmodel->GetFamilyMask());

            switch (shape->GetType()) {
                case ChCollisionShape::Type::SPHERE:
                    shape_data.sphere_rigid[start] = obB.x;
                    break;
                case ChCollisionShape::Type::TRIANGLEMESH:  // NOTE: There is space for all 3
                    shape_data.triangle_rigid[start] = obA;
                    shape_data.triangle_rigid[start + 1] = obB;
                    shape_data.triangle_rigid[start + 2] = obC;
                    break;
                case ChCollisionShape::Type::ELLIPSOID:
                    shape_data.box_like_rigid[start] = obB;
                    break;
                case ChCollisionShape::Type::BOX:
                    shape_data.box_like_rigid[start] = obB;
                    break;
                case ChCollisionShape::Type::CYLINDER:
                    shape_data.box_like_rigid[start] = obB;
                    break;
                case ChCollisionShape::Type::CONE:
                    shape_data.box_like_rigid[start] = obB;
                    break;
                case ChCollisionShape::Type::CAPSULE:
                    shape_data.capsule_rigid[start] = real2(obB.x, obB.y);
                    break;
                case ChCollisionShape::Type::ROUNDEDBOX:
                    shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
                    break;
                case ChCollisionShape::Type::ROUNDEDCYL:
                    shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
                    break;
                default:
                    ddm->my_sys->ErrorAbort("Shape not supported\n");
            }

            shape_data.ObA_rigid[j] = obA;
            shape_data.ObR_rigid[j] = shape->R;
            shape_data.fam_rigid[j] = fam;
        }
    }
    // If there was not enough space in the data_manager for all shapes in the model,
    // call the regular add // TODO faster jump to here
    else {
        this->ChCollisionSystemChronoMulticore::Add(model);
        for (int i = 0; i < needed_count; i++) {
            ddm->dm_free_shapes.push_back(false);
            ddm->body_shapes[begin_shapes] = static_cast<int>(shape_data.id_rigid.size()) - needed_count + i;  // TODO ?
            begin_shapes++;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
//
// // TODO finish this
// void ChCollisionSystemDistributed::NewAdd(ChCollisionModel* model) {
//     ChMulticoreDataManager* dm = ddm->data_manager;
//     ChCollisionModelDistributed* pmodel = static_cast<ChCollisionModelDistributed*>(model);
//     // Find space in ddm vectors - need one index for both start and count
//     // need a chunk of body_shapes large enough for all shapes on this body
//     int body_index = pmodel->GetBody()->GetId();  // NOTE: Assumes this is set
//     int needed_count = pmodel->GetNumShapes();    // Minimum size needed in ddm->body_shapes
//     ChVector<> pos(pmodel->GetBody()->GetPos());
//
//     // TODO Only include shapes relevant to this sub-domain
//
// 	// The first index of the free chunk of body_shapes
// 	int begin_shapes;
//     bool found = false;
//
//     // Find free chunk in ddm->body_shapes large enough for this model using free list
//     // TODO consider first set of adds: could be adding a bunch of list nodes that just say one slot is full. Should
//     // consolidate them together to say that a big chunk is taken.
//     for (auto curr = ddm->ddm_free_shapes.begin(), int index = 0; itr != ddm->ddm_free_shapes.end() && !found;
//          itr++, index++) {
//         // If curr is a large enough free chunk
//         if (curr->free && curr->size >= needed_count) {
//             found = true;
//             curr->free = false;
//
//             // If there is leftover space, insert a new node in the free list
//             if (curr->size != needed_count) {
//                 // Insert new free node after curr
// 				ddm->ddm_free_shapes.emplace(index + 1, curr->body_shapes_index + needed_count, curr->size -
// needed_count, true);
//             }
//             curr->size = needed_count;
//
// 			// If a free portion of body_shapes was found, set up the body to use it
// 			ddm->body_shape_start[body_index] = curr->body_shapes_index;
// 			ddm->body_shape_count[body_index] = needed_count;
// 			begin_shapes = curr->body_shapes_index;
//         }
//     }
//     // NOTE: curr could be NULL for two reasons: empty list and going off the end.
//     // found == true => curr identifies a large-enough free chunk
//
//     // If there was no free portion of body_shapes large enough, create more space
//     // in body_shapes and a new free-list node
//     if (!found) {
//         ddm->body_shape_count[body_index] = needed_count;
//
//         // Add a node at the end
//         struct LocalShapeNode* new_end;
//
//         // If the above while loop went off the end searching
//         if (curr == NULL && prev != NULL) {
//             // If the previous chunk is not free, merge new and previous
//             if (prev->free == false) {
//                 prev->size += needed_count;
//                 // This portion will begin right after the end of the current vector
//                 begin_shapes = ddm->body_shapes.size();
//             }
//             // If the previous chunk is free, add new node at end
//             else {
//                 new_end = new struct LocalShapeNode();
//                 prev->next = new_end;
//                 new_end->size = needed_count;
//                 new_end->free = false;
//                 new_end->body_shapes_index = ddm->body_shapes.size();  // This portion will begin right
//                                                                        // after the end of the current body_shapes
//                 new_end->next = NULL;
//                 begin_shapes = new_end->body_shapes_index;
//             }
//         }
//         // If the first node doesn't exist yet
//         else if (ddm->ddm_free_shapes.empty()) {
//             // Create first node
//             begin_shapes = 0;
// 			ddm->ddm_free_shapes.emplace_front(0, needed_count, false)
//         }
//
//         ddm->body_shape_start[body_index] = ddm->body_shapes.size();
//
//         // Create the space in body_shapes
//         for (int i = 0; i < needed_count; i++) {
//             ddm->body_shapes.push_back(0);
//         }
//     }
//
//     /*
//      * NOTE: At this point there is a large-enough chunk in ddm->body_shapes that is NOT set
//      * beginning at begin_shapes
//      */
//
//     // TODO need places for ALL shapes in the model in data_manager->shape_data,
//     // else need to call collsysPAR::add
//
//     // Check for free spaces to insert into (DO need same shape type else can't deactivate that)
//     std::vector<int> free_dm_shapes;
//     for (int i = 0; i < needed_count; i++) {
//         // i identifies a shape in the MODEL
//
//         // Search data_manager->shape_data for a free and shape-matching spot
//         for (int j = 0; j < dm->shape_data.id_rigid.size(); j++) {
//             // If the index in the data manager is open and corresponds to the same shape type
//             if (dm->shape_data.id_rigid[j] == UINT_MAX && dm->shape_data.typ_rigid[j] == pmodel->mData[i].type) {
//                 free_dm_shapes.push_back(j);
//                 break;  // Found spot for this shape, break inner loop to get new i (shape)
//             }
//             // TODO: Early break from both loops if a spot cannot be found for a shape
//         }  // End for loop over the slots in data_manager->shape_data
//     }      // End for loop over the shapes for this model
//
//     // If there is space for ALL shapes in the model in the data_manager
//     // unload them.
//     if (free_dm_shapes.size() == needed_count) {
//         for (int i = 0; i < needed_count; i++) {
//             // i identifies a shape in the MODEL
//
//             int j = free_dm_shapes[i];  // Index into dm->shape_data
//
//             dm->shape_data.id_rigid[j] = body_index;
//             ddm->body_shapes[begin_shapes + i] = j;
//             ddm->dm_free_shapes[j] = false;
//
//             // type_rigid and start_rigid are unchanged because the shape type is the same
//             int start = dm->shape_data.start_rigid[j];
//
//             real3 obA = pmodel->mData[i].A;
//             real3 obB = pmodel->mData[i].B;
//             real3 obC = pmodel->mData[i].C;
//
//             short2 fam = S2(pmodel->GetFamilyGroup(), pmodel->GetFamilyMask());
//
//             switch (pmodel->mData[i].type) {
//                 case chrono::collision::SPHERE:
//                     dm->shape_data.sphere_rigid[start] = obB.x;
//                     break;
//                 case chrono::collision::TRIANGLEMESH:  // NOTE: There is space for all 3
//                     dm->shape_data.triangle_rigid[start] = obA;
//                     dm->shape_data.triangle_rigid[start + 1] = obB;
//                     dm->shape_data.triangle_rigid[start + 2] = obC;
//                     break;
//                 case chrono::collision::ELLIPSOID:
//                     dm->shape_data.box_like_rigid[start] = obB;
//                     break;
//                 case chrono::collision::BOX:
//                     dm->shape_data.box_like_rigid[start] = obB;
//                     break;
//                 case chrono::collision::CYLINDER:
//                     dm->shape_data.box_like_rigid[start] = obB;
//                     break;
//                 case chrono::collision::CONE:
//                     dm->shape_data.box_like_rigid[start] = obB;
//                     break;
//                 case chrono::collision::CAPSULE:
//                     dm->shape_data.capsule_rigid[start] = real2(obB.x, obB.y);
//                     break;
//                 case chrono::collision::ROUNDEDBOX:
//                     dm->shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
//                     break;
//                 case chrono::collision::ROUNDEDCYL:
//                     dm->shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
//                     break;
//                 default:
//                     ddm->my_sys->ErrorAbort("Shape not supported\n");
//             }
//
//             dm->shape_data.ObA_rigid[j] = obA;
//             dm->shape_data.ObR_rigid[j] = pmodel->mData[i].R;
//             dm->shape_data.fam_rigid[j] = fam;
//         }
//     }
//     // If there was not enough space in the data_manager for all shapes in the model,
//     // call the regular add // TODO faster jump to here
//     else {
//         this->ChCollisionSystemChronoMulticore::Add(model);
//         for (int i = 0; i < needed_count; i++) {
//             ddm->dm_free_shapes.push_back(false);
//             ddm->body_shapes[begin_shapes] = dm->shape_data.id_rigid.size() - needed_count + i;  // TODO ?
//             begin_shapes++;
//         }
//     }
// }

//////////////////////////////////////////////////////////////////////////////

// Id must be set before calling
// Deactivates all shapes associated with the collision model
void ChCollisionSystemDistributed::Remove(ChCollisionModel* model) {
    ChCollisionModelChrono* pmodel = static_cast<ChCollisionModelChrono*>(model);
    uint id = pmodel->GetBody()->GetId();
    int count = pmodel->GetNumShapes();
    int start = ddm->body_shape_start[id];

    for (int i = 0; i < count; i++) {
        int index = start + i;
        ddm->dm_free_shapes[ddm->body_shapes[index]] = true;  // Marks the spot in shape_data as free

        // Forces collision detection to ignore this shape
        ddm->data_manager->cd_data->shape_data.id_rigid[ddm->body_shapes[index]] = UINT_MAX;
    }

    // TODO better Search.
    struct LocalShapeNode* curr = ddm->local_free_shapes;
    while (curr != NULL) {
        // iterate until curr contains start
        if (curr->body_shapes_index == start || curr->next == NULL || curr->next->body_shapes_index > start) {
            break;
        }
        curr = curr->next;
    }
    if (curr == NULL) {
        ddm->my_sys->ErrorAbort("ERROR: Went of end of free list ChCollisionModelDistributed::Remove()");
    }
    // At this point, curr contains start
    if (curr->body_shapes_index == start) {
        if (curr->size == count) {
            curr->free = true;
        } else {
            struct LocalShapeNode* new_next = new LocalShapeNode();
            new_next->size = curr->size - count;
            new_next->free = false;
            new_next->body_shapes_index = start + count;
            new_next->next = curr->next;

            curr->size = count;
            curr->free = true;
            curr->next = new_next;
        }
    } else if (curr->body_shapes_index < start) {
        if (curr->size == count + start - curr->body_shapes_index) {
            struct LocalShapeNode* new_next = new LocalShapeNode();
            new_next->size = count;
            new_next->free = true;
            new_next->body_shapes_index = start;
            new_next->next = curr->next;

            curr->size -= count;
            curr->next = new_next;
        } else if (count + start - curr->body_shapes_index < curr->size) {
            LocalShapeNode* node1 = new LocalShapeNode();
            LocalShapeNode* node2 = new LocalShapeNode();

            node1->size = count;
            node1->free = true;
            node1->body_shapes_index = start;
            node1->next = node2;

            node2->size = curr->size - (start - curr->body_shapes_index + count);
            node2->free = false;
            node2->body_shapes_index = start + count;
            node2->next = curr->next;

            curr->size = start - curr->body_shapes_index;
            curr->next = node1;
        } else {
            GetLog() << "ERROR: Unexpected case in ChCollisionSystemDistributed::Remove()\n";
        }
    } else {
        GetLog() << "ERROR: curr not set correctly ChCollisionSystemDistributed::Remove()\n";
    }
}

void ChCollisionSystemDistributed::GetOverlappingAABB(custom_vector<char>& active_id, real3 Amin, real3 Amax) {
    GenerateAABB();
    ////#pragma omp parallel for
    for (int i = 0; i < ddm->data_manager->cd_data->shape_data.typ_rigid.size(); i++) {
        auto id_rigid = ddm->data_manager->cd_data->shape_data.id_rigid[i];
        if (id_rigid == UINT_MAX) {
            continue;
        }
        real3 Bmin = ddm->data_manager->cd_data->aabb_min[i];
        real3 Bmax = ddm->data_manager->cd_data->aabb_max[i];

        bool inContact = (Amin.x <= Bmax.x && Bmin.x <= Amax.x) && (Amin.y <= Bmax.y && Bmin.y <= Amax.y) &&
                         (Amin.z <= Bmax.z && Bmin.z <= Amax.z);
        if (inContact) {
            active_id[id_rigid] = true;
        }
    }
}

} /* namespace collision */
} /* namespace chrono */
