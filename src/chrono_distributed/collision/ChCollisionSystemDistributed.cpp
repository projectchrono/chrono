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
}

ChCollisionSystemDistributed::~ChCollisionSystemDistributed() {}

// Called by chcollisionmodel::buildmodel (if system set), chbody::setcollide(true), chbody::setsystem (if system set)
// (called by addbody AND addbodyexchange)
void ChCollisionSystemDistributed::Add(ChCollisionModel* model) {
    GetLog() << "ColSys::Add\n";
    ChParallelDataManager* dm = ddm->data_manager;
    ChCollisionModelParallel* pmodel = static_cast<ChCollisionModelParallel*>(model);
    // Find space in ddm vectors - need one index for both start and count
    // need a chunk of body_shapes large enough for all shapes on this body
    // TODO better search
    bool found = false;
    int my_first;
    for (my_first = 0; my_first < ddm->data_manager->shape_data.id_rigid.size(); my_first++) {
        if (ddm->data_manager->shape_data.id_rigid[my_first] == UINT_MAX) {
            found = true;
            break;
        }
    }

    int begin_shapes;
    int count = pmodel->GetNObjects();
    int running_count = 0;
    for (begin_shapes = 0; begin_shapes < ddm->my_free_shapes.size(); begin_shapes++) {
        if (running_count == count) {
            found = found && true;
            break;
        }
        if (ddm->my_free_shapes[begin_shapes]) {
            running_count++;
        } else {
            running_count = 0;
        }
    }

    begin_shapes -= count;
    // my_first and begin_shapes are now valid for the ddm
    if (found) {
        // Check for free spaces to insert into (DO need same shape type else can't deactivate that)
        for (int i = 0; i < pmodel->GetNObjects(); i++) {
            // TODO Better outlining comment
            for (int j = 0; j < dm->shape_data.id_rigid.size(); j++) {
                // If the index in the data manager is open and corresponds to the same shape type
                if (dm->shape_data.id_rigid[j] == UINT_MAX && dm->shape_data.typ_rigid[j] == pmodel->mData[i].type) {
                    dm->shape_data.id_rigid[j] = pmodel->GetBody()->GetId();
                    ddm->body_shapes[begin_shapes] = j;
                    ddm->my_free_shapes[begin_shapes++] = false;
                    ddm->dm_free_shapes[j] = false;

                    // type_rigid and start_rigid are unchanged because the shape type is the same
                    int start = ddm->data_manager->shape_data.start_rigid[j];

                    real3 obA = pmodel->mData[i].A;
                    real3 obB = pmodel->mData[i].B;
                    real3 obC = pmodel->mData[i].C;

                    switch (pmodel->mData[i].type) {
                        case chrono::collision::SPHERE:
                            GetLog() << "Adding sphere\n";
                            ddm->data_manager->shape_data.sphere_rigid[start] = obB.x;
                            break;
                        case chrono::collision::ELLIPSOID:
                            ddm->data_manager->shape_data.box_like_rigid[start] = obB;
                            break;
                        case chrono::collision::BOX:
                            ddm->data_manager->shape_data.box_like_rigid[start] = obB;
                            break;
                        case chrono::collision::CYLINDER:
                            ddm->data_manager->shape_data.box_like_rigid[start] = obB;
                            break;
                        case chrono::collision::CONE:
                            ddm->data_manager->shape_data.box_like_rigid[start] = obB;
                            break;
                        case chrono::collision::CAPSULE:
                            ddm->data_manager->shape_data.capsule_rigid[start] = real2(obB.x, obB.y);
                            break;
                        case chrono::collision::ROUNDEDBOX:
                            ddm->data_manager->shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
                            break;
                        case chrono::collision::ROUNDEDCYL:
                            ddm->data_manager->shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
                            break;
                        case chrono::collision::ROUNDEDCONE:
                            ddm->data_manager->shape_data.rbox_like_rigid[start] = real4(obB, obC.x);
                            break;
                        default:
                            GetLog() << "Shape not supported\n";
                    }

                    ddm->data_manager->shape_data.ObA_rigid[j] = obA;
                    ddm->data_manager->shape_data.ObR_rigid[j] = pmodel->mData[i].R;
                }
                ddm->body_shape_count[j] = count;
                break;  // break to next shape in the model
            }
        }
    } else {
        // If no free spaces to insert into, add to end
        this->ChCollisionSystemParallel::Add(model);

        ddm->body_shape_count.push_back(count);
        ddm->body_shape_start.push_back(ddm->body_shapes.size());

        for (int i = 0; i < count; i++) {
            ddm->body_shapes.push_back(ddm->data_manager->num_rigid_shapes - count + i);
            ddm->my_free_shapes.push_back(false);
            ddm->dm_free_shapes.push_back(false);
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
        // ddm->my_free_shapes[index] = true; // Marks the spot in ddm->body_shapes as open
        // ddm->dm_free_shapes[ddm->body_shapes[index]] = true; // Marks the spot in data_manager->shape_data as
        // open

        // Forces collision detection to ignore this shape
        ddm->data_manager->shape_data.id_rigid[ddm->body_shapes[index]] = UINT_MAX;
    }
}
