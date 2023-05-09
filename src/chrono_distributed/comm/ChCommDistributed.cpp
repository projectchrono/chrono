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
// Authors: Nic Olsen, Radu Serban
// =============================================================================

#include <mpi.h>
#include <omp.h>
#include <climits>
#include <forward_list>
#include <memory>
#include <string>

#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"
#include "chrono_distributed/comm/ChCommDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono_multicore/ChDataManager.h"

#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

using namespace chrono;
using namespace collision;

ChCommDistributed::ChCommDistributed(ChSystemDistributed* my_sys) {
    this->my_sys = my_sys;
    this->data_manager = my_sys->data_manager;

    ddm = my_sys->ddm;

    /* Create and Commit all custom MPI Data Types */
    // Exchange
    MPI_Datatype type_exchange[4] = {MPI_UNSIGNED, MPI_BYTE, MPI_DOUBLE, MPI_INT};
    int blocklen_exchange[4] = {1, 1, 20, 1};
    MPI_Aint disp_exchange[4];
    disp_exchange[0] = offsetof(BodyExchange, gid);
    disp_exchange[1] = offsetof(BodyExchange, collide);
    disp_exchange[2] = offsetof(BodyExchange, pos);
    disp_exchange[3] = offsetof(BodyExchange, identifier);
    MPI_Datatype temp_type;
    MPI_Type_create_struct(4, blocklen_exchange, disp_exchange, type_exchange, &temp_type);
    MPI_Aint lb, extent;
    MPI_Type_get_extent(temp_type, &lb, &extent);
    MPI_Type_create_resized(temp_type, lb, extent, &BodyExchangeType);
    MPI_Type_commit(&BodyExchangeType);

    // Update
    MPI_Datatype type_update[3] = {MPI_UNSIGNED, MPI_INT, MPI_DOUBLE};
    int blocklen_update[3] = {1, 1, 13};
    MPI_Aint disp_update[3];
    disp_update[0] = offsetof(BodyUpdate, gid);
    disp_update[1] = offsetof(BodyUpdate, update_type);
    disp_update[2] = offsetof(BodyUpdate, pos);
    MPI_Type_create_struct(3, blocklen_update, disp_update, type_update, &BodyUpdateType);
    MPI_Type_commit(&BodyUpdateType);

    // Shape
    MPI_Datatype type_shape[5] = {MPI_UNSIGNED, MPI_INT, MPI_SHORT, MPI_DOUBLE, MPI_FLOAT};
    int blocklen_shape[5] = {1, 1, 2, 13, 6};
    MPI_Aint disp_shape[5];
    disp_shape[0] = offsetof(Shape, gid);
    disp_shape[1] = offsetof(Shape, type);
    disp_shape[2] = offsetof(Shape, coll_fam);
    disp_shape[3] = offsetof(Shape, A);
    disp_shape[4] = offsetof(Shape, mu);
    MPI_Datatype temp_type_s;
    MPI_Type_create_struct(5, blocklen_shape, disp_shape, type_shape, &temp_type_s);
    MPI_Aint lb_s, extent_s;
    MPI_Type_get_extent(temp_type_s, &lb_s, &extent_s);
    MPI_Type_create_resized(temp_type_s, lb_s, extent_s, &ShapeType);
    MPI_Type_commit(&ShapeType);
}

ChCommDistributed::~ChCommDistributed() {}

void ChCommDistributed::ProcessExchanges(int num_recv, BodyExchange* buf, int updown) {
    if (buf->gid == UINT_MAX) {
        return;
    }

    ddm->first_empty = 0;
    std::shared_ptr<ChBody> body;

    for (int n = 0; n < num_recv; n++) {
        // Find the next empty slot in the data manager.
        if (ddm->first_empty != data_manager->num_rigid_bodies &&
            ddm->comm_status[ddm->first_empty] != distributed::EMPTY) {
            while (ddm->first_empty < static_cast<int>(data_manager->num_rigid_bodies) &&
                   ddm->comm_status[ddm->first_empty] != distributed::EMPTY) {
                ddm->first_empty++;
            }
        }

        // If there are no empty spaces in the data manager, create
        // a new body to add
        if (ddm->first_empty == data_manager->num_rigid_bodies) {
            body = chrono_types::make_shared<ChBody>(chrono_types::make_shared<ChCollisionModelDistributed>());
            body->SetId(data_manager->num_rigid_bodies);
        }
        // If an empty space was found in the body manager
        else {
            body = (*data_manager->body_list)[ddm->first_empty];
        }

        UnpackExchange(buf + n, body);

        // Add the new body
        distributed::COMM_STATUS status = (updown == 1) ? distributed::GHOST_UP : distributed::GHOST_DOWN;
        if (ddm->first_empty == data_manager->num_rigid_bodies) {
            my_sys->AddBodyExchange(body, status);  // NOTE: Does not call colsys::add
        } else {
            ddm->comm_status[ddm->first_empty] = status;
            body->SetBodyFixed(false);
            ddm->gid_to_localid[body->GetGid()] = body->GetId();
            ddm->global_id[body->GetId()] = body->GetGid();
        }
        // NOTE: At this point, the body has collide == false and it has not touched the collision system
    }
}

void ChCommDistributed::ProcessUpdates(int num_recv, BodyUpdate* buf) {
    // If the buffer is empty
    if (buf->gid == UINT_MAX) {
        return;
    }
    std::shared_ptr<ChBody> body;
    for (int n = 0; n < num_recv; n++) {
        // Find the existing body
        int index = ddm->GetLocalIndex((buf + n)->gid);

        if (index != -1 && ddm->comm_status[index] != distributed::EMPTY) {
            if (ddm->comm_status[index] != distributed::GHOST_UP &&
                ddm->comm_status[index] != distributed::GHOST_DOWN) {
                my_sys->ErrorAbort(std::string("Trying to update a non-ghost body on rank ") +
                                   std::to_string(my_sys->my_rank) + std::string("GID ") +
                                   std::to_string((buf + n)->gid) + std::string("\n"));
            }
            body = (*data_manager->body_list)[index];
            UnpackUpdate(buf + n, body);
            if ((buf + n)->update_type == distributed::FINAL_UPDATE_GIVE) {
                GetLog() << "GIVE " << ddm->global_id[index] << " to rank " << my_sys->my_rank << "\n";
                ddm->comm_status[index] = distributed::OWNED;
            } else if ((buf + n)->update_type == distributed::UPDATE_TRANSFER_SHARE) {
                ddm->comm_status[index] = (ddm->comm_status[index] == distributed::GHOST_UP) ? distributed::SHARED_UP
                                                                                             : distributed::SHARED_DOWN;
            }
        } else {
            GetLog() << "GID " << (buf + n)->gid << " NOT found rank " << my_sys->my_rank << "\n";
            my_sys->ErrorAbort("Body to be updated not found\n");
        }
    }
}

void ChCommDistributed::ProcessTakes(int num_recv, uint* buf) {
    if (buf[0] == UINT_MAX) {
        return;
    }
    for (int i = 0; i < num_recv; i++) {
        int index = ddm->GetLocalIndex(buf[i]);
        my_sys->RemoveBodyExchange(index);
    }
}

// TODO might be able to do in parallel if check the number of shapes per body in a first pass
void ChCommDistributed::ProcessShapes(int num_recv, Shape* buf) {
    if (buf->gid == UINT_MAX) {
        return;
    }

    int n = 0;
    uint gid;

    ChSystemSMC::AdhesionForceModel adhesion_model = ddm->data_manager->settings.solver.adhesion_force_model;
    bool use_material_properties = ddm->data_manager->settings.solver.use_material_properties;

    // Each iteration handles all shapes for a single body
    while (n < num_recv) {
        gid = (buf + n)->gid;
        // Create a collision model
        int local_id = ddm->GetLocalIndex(gid);
        if (local_id == -1) {
            my_sys->ErrorAbort(std::string("ProcessShapes: GID ") + std::to_string(gid) + " not found on rank " +
                               std::to_string(my_sys->my_rank) + "\n");
        }
        std::shared_ptr<ChBody> body = (*ddm->data_manager->body_list)[local_id];

        body->GetCollisionModel()->SetFamilyGroup((buf + n)->coll_fam[0]);
        body->GetCollisionModel()->SetFamilyMask((buf + n)->coll_fam[1]);

        double* rot;
        double* data;

        // Each iteration handles a single shape for the body
        while (n < num_recv && (buf + n)->gid == gid) {
            ChVector<double> A((buf + n)->A[0], (buf + n)->A[1], (buf + n)->A[2]);  // shape position
            rot = (buf + n)->R;                                                     // quaternion
            data = (buf + n)->data;                                                 // shape-specific geometric data

            // Create the contact material
            auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            material->SetFriction((buf + n)->mu);
            switch (adhesion_model) {
                case ChSystemSMC::AdhesionForceModel::Perko:
                    // Not yet implemented. Falls through.
                case ChSystemSMC::AdhesionForceModel::Constant:
                    material->SetAdhesion((buf + n)->cohesion);
                    break;
                case ChSystemSMC::AdhesionForceModel::DMT:
                    material->SetAdhesionMultDMT((buf + n)->cohesion);
                    break;
            }
            if (use_material_properties) {
                material->SetYoungModulus((buf + n)->ym_kn);
                material->SetPoissonRatio((buf + n)->pr_kt);
                material->SetRestitution((buf + n)->restit_gn);
            } else {
                material->SetKn((buf + n)->ym_kn);
                material->SetKt((buf + n)->pr_kt);
                material->SetGn((buf + n)->restit_gn);
                material->SetGt((buf + n)->gt);
            }

            switch ((buf + n)->type) {
                case ChCollisionShape::Type::SPHERE:
                    body->GetCollisionModel()->AddSphere(material, data[0], A);
                    break;
                case ChCollisionShape::Type::BOX:
                    body->GetCollisionModel()->AddBox(material, 2 * data[0], 2 * data[1], 2 * data[2], A,
                                                      ChMatrix33<>(ChQuaternion<>(rot[0], rot[1], rot[2], rot[3])));
                    break;
                case ChCollisionShape::Type::TRIANGLEMESH:
                    //// RADU:  why this cast here?
                    std::static_pointer_cast<ChCollisionModelDistributed>(body->GetCollisionModel())
                        ->AddTriangle(material, A, ChVector<>(data[0], data[1], data[2]),
                                      ChVector<>(data[3], data[4], data[5]), ChVector<>(0, 0, 0),
                                      ChQuaternion<>(rot[0], rot[1], rot[2], rot[3]));
                    break;
                case ChCollisionShape::Type::ELLIPSOID:
                    body->GetCollisionModel()->AddEllipsoid(
                        material, 2 * data[0], 2 * data[1], 2 * data[2], A,
                        ChMatrix33<>(ChQuaternion<>(rot[0], rot[1], rot[2], rot[3])));
                    break;
                default:
                    GetLog() << "Error: gid " << gid << " rank " << my_sys->my_rank << " type " << (buf + n)->type
                             << "\n";
                    my_sys->ErrorAbort("Unpacking undefined collision shape\n");
            }
            n++;  // Advance to next shape in the buffer
        }
        body->SetCollide(true);  // NOTE: Calls colsys::add
                                 // NOTE: DO NOT BUILD MODEL
    }
}

// Handle all necessary communication
void ChCommDistributed::Exchange() {
    int my_rank = my_sys->my_rank;
    int num_ranks = my_sys->num_ranks;
    std::forward_list<int> exchanges_up;
    std::forward_list<int> exchanges_down;

    // Saves a reference copy for consistency in the threads.
    ddm->curr_status = ddm->comm_status;
    std::vector<BodyExchange> exchange_up_buf;
    std::vector<BodyExchange> exchange_down_buf;
    std::vector<BodyUpdate> update_up_buf;
    std::vector<BodyUpdate> update_down_buf;
    std::vector<Shape> shapes_up;
    std::vector<Shape> shapes_down;
    std::vector<uint> update_take_up;
    std::vector<uint> update_take_down;

    // Send Counts
    int num_exchange_up = 0;
    int num_exchange_down = 0;
    int num_update_up = 0;
    int num_update_down = 0;
    int num_shapes_up = 0;
    int num_shapes_down = 0;
    int num_take_up = 0;
    int num_take_down = 0;

#pragma omp parallel sections
    {
// Exchange Loop
#pragma omp section
        {
            for (uint i = 0; i < data_manager->num_rigid_bodies; i++) {
                // Skip empty bodies or those that this rank isn't responsible for
                int curr_status = ddm->curr_status[i];
                if (curr_status != distributed::OWNED)
                    continue;
                int location = my_sys->domain->GetBodyRegion(i);

                // If the body now affects the next sub-domain
                // If the body is being marked as shared for the first time, the whole
                // body must be packed to create a ghost on another rank
                if ((location == distributed::GHOST_UP || location == distributed::SHARED_UP)) {
                    BodyExchange b_ex = {};
                    PackExchange(&b_ex, i);
                    exchange_up_buf.push_back(b_ex);

                    num_exchange_up++;  // TODO might be able to eliminate
                    ddm->comm_status[i] = distributed::SHARED_UP;
                    exchanges_up.push_front(i);
                }

                // If the body now affects the previous sub-domain:
                // If the body is being
                // marked as shared for the first time, the whole
                // body must be packed to create a ghost on another rank
                else if ((location == distributed::GHOST_DOWN || location == distributed::SHARED_DOWN)) {
                    BodyExchange b_ex = {};
                    PackExchange(&b_ex, i);
                    exchange_down_buf.push_back(b_ex);

                    num_exchange_down++;  // TODO might be able to eliminate
                    ddm->comm_status[i] = distributed::SHARED_DOWN;
                    exchanges_down.push_front(i);
                }
            }
        }  // end of exchange section

        // Update Loop
#pragma omp section
        {
            // PACKING and UPDATING comm_status of existing bodies
            for (uint i = 0; i < data_manager->num_rigid_bodies; i++) {
                // Skip empty bodies or those that this rank isn't responsible for
                int curr_status = ddm->curr_status[i];
                int location = my_sys->domain->GetBodyRegion(i);

                if (curr_status != distributed::SHARED_UP && curr_status != distributed::SHARED_DOWN)
                    continue;

                // If the body has already been shared, it need only update its
                // corresponding ghost
                if (location == distributed::SHARED_UP && curr_status == distributed::SHARED_UP) {
                    BodyUpdate b_upd = {};
                    PackUpdate(&b_upd, i, distributed::UPDATE);
                    update_up_buf.push_back(b_upd);

                    num_update_up++;  // TODO might be able to eliminate
                } else if (location == distributed::GHOST_UP && curr_status == distributed::SHARED_UP) {
                    BodyUpdate b_upd = {};
                    PackUpdate(&b_upd, i, distributed::UPDATE_TRANSFER_SHARE);
                    update_up_buf.push_back(b_upd);

                    ddm->comm_status[i] = distributed::GHOST_UP;
                    num_update_up++;  // TODO might be able to eliminate
                }

                // If the body has already been shared, it need only update its
                // corresponding ghost
                else if (location == distributed::SHARED_DOWN && curr_status == distributed::SHARED_DOWN) {
                    BodyUpdate b_upd = {};
                    PackUpdate(&b_upd, i, distributed::UPDATE);
                    update_down_buf.push_back(b_upd);

                    num_update_down++;  // TODO might be able to eliminate
                } else if (location == distributed::GHOST_DOWN && curr_status == distributed::SHARED_DOWN) {
                    BodyUpdate b_upd = {};
                    PackUpdate(&b_upd, i, distributed::UPDATE_TRANSFER_SHARE);
                    update_down_buf.push_back(b_upd);

                    ddm->comm_status[i] = distributed::GHOST_DOWN;
                    num_update_down++;  // TODO might be able to eliminate
                }
                // If is shared up/down AND
                // If the body is no longer involved with this rank, it must be removed from
                // this rank
                else if ((location == distributed::UNOWNED_UP || location == distributed::UNOWNED_DOWN) &&
                         (ddm->comm_status[i] == distributed::SHARED_UP ||
                          ddm->comm_status[i] == distributed::SHARED_DOWN)) {
                    ////int up;
                    if (location == distributed::UNOWNED_UP && my_rank != num_ranks - 1) {
                        GetLog() << "GIVE " << ddm->global_id[i] << " from rank " << my_rank << "\n";
                        BodyUpdate b_upd = {};
                        PackUpdate(&b_upd, i, distributed::FINAL_UPDATE_GIVE);
                        update_up_buf.push_back(b_upd);
                        num_update_up++;  // TODO might be able to eliminate
                        ////up = 1;
                    } else if (location == distributed::UNOWNED_DOWN && my_rank != 0) {
                        GetLog() << "GIVE " << ddm->global_id[i] << " from rank " << my_rank << "\n";
                        BodyUpdate b_upd = {};
                        PackUpdate(&b_upd, i, distributed::FINAL_UPDATE_GIVE);
                        update_down_buf.push_back(b_upd);

                        num_update_down++;  // TODO might be able to eliminate
                        ////up = -1;
                    }

                    my_sys->RemoveBodyExchange(i);
                }
            }  // End of packing for loop
        }      // End of update section

        // Update Take Loop
#pragma omp section
        {
            // PACKING and UPDATING comm_status of existing bodies
            for (uint i = 0; i < data_manager->num_rigid_bodies; i++) {
                // Skip empty bodies or those that this rank isn't responsible for
                int curr_status = ddm->curr_status[i];
                if (curr_status != distributed::SHARED_DOWN && curr_status != distributed::SHARED_UP)
                    continue;

                int location = my_sys->domain->GetBodyRegion(i);

                // If the body is no longer involved with either neighbor, remove it from
                // the other rank and take ownership on this rank
                if (location == distributed::OWNED) {
                    if (curr_status == distributed::SHARED_UP) {
                        uint b_ut;
                        PackUpdateTake(&b_ut, i);
                        update_take_up.push_back(b_ut);
                        num_take_up++;  // TODO might be able to eliminate

                    } else if (curr_status == distributed::SHARED_DOWN) {
                        uint b_ut;
                        PackUpdateTake(&b_ut, i);
                        update_take_down.push_back(b_ut);
                        num_take_down++;  // TODO might be able to eliminate
                    }
                    ddm->comm_status[i] = distributed::OWNED;
                }
            }  // End of packing for loop
        }      // End of update take loop
    }          // End of parallel sections

    // MPI_Status status_exchange_up;
    // MPI_Status status_exchange_down;
    // MPI_Status status_update_up;
    // MPI_Status status_update_down;
    // MPI_Status status_take_up;
    // MPI_Status status_take_down;
    // MPI_Status status_shapes_up;
    // MPI_Status status_shapes_down;

    MPI_Request rq_exchange_up;
    MPI_Request rq_exchange_down;
    MPI_Request rq_update_up;
    MPI_Request rq_update_down;
    MPI_Request rq_take_up;
    MPI_Request rq_take_down;
    MPI_Request rq_shapes_up;
    MPI_Request rq_shapes_down;

    MPI_Status recv_status_exchange_up;
    MPI_Status recv_status_exchange_down;
    MPI_Status recv_status_update_up;
    MPI_Status recv_status_update_down;
    MPI_Status recv_status_take_up;
    MPI_Status recv_status_take_down;
    MPI_Status recv_status_shapes_up;
    MPI_Status recv_status_shapes_down;

    int num_recv_exchange_up;
    int num_recv_exchange_down;
    int num_recv_update_up;
    int num_recv_update_down;
    int num_recv_take_up;
    int num_recv_take_down;
    int num_recv_shapes_up = 0;
    int num_recv_shapes_down = 0;

    BodyExchange* recv_exchange_down = NULL;
    BodyExchange* recv_exchange_up = NULL;
    BodyUpdate* recv_update_down = NULL;
    BodyUpdate* recv_update_up = NULL;
    uint* recv_take_down = NULL;
    uint* recv_take_up = NULL;
    Shape* recv_shapes_down = NULL;
    Shape* recv_shapes_up = NULL;

#pragma omp parallel sections
    {
// Send/Recv
#pragma omp section
        {
            // SENDING - TODO sequential for now

            // Send empty message if there is nothing to send
            if (num_exchange_up == 0) {
                BodyExchange b_e = {};
                b_e.gid = UINT_MAX;
                exchange_up_buf.push_back(b_e);
                num_exchange_up = 1;
            }
            if (num_exchange_down == 0) {
                BodyExchange b_e = {};
                b_e.gid = UINT_MAX;
                exchange_down_buf.push_back(b_e);
                num_exchange_down = 1;
            }
            if (num_update_up == 0) {
                BodyUpdate b_u = {};
                b_u.gid = UINT_MAX;
                update_up_buf.push_back(b_u);
                num_update_up = 1;
            }
            if (num_update_down == 0) {
                BodyUpdate b_u = {};
                b_u.gid = UINT_MAX;
                update_down_buf.push_back(b_u);
                num_update_down = 1;
            }
            if (num_take_up == 0) {
                update_take_up.push_back(UINT_MAX);
                num_take_up = 1;
            }
            if (num_take_down == 0) {
                update_take_down.push_back(UINT_MAX);
                num_take_down = 1;
            }

            // Send Exchanges
            if (my_rank != num_ranks - 1) {
                MPI_Isend(&(exchange_up_buf[0]), num_exchange_up, BodyExchangeType, my_rank + 1, 1, my_sys->world,
                          &rq_exchange_up);
            }
            if (my_rank != 0) {
                MPI_Isend(&(exchange_down_buf[0]), num_exchange_down, BodyExchangeType, my_rank - 1, 2, my_sys->world,
                          &rq_exchange_down);
            }

            // Recv Exchanges
            if (my_rank != 0) {
                MPI_Probe(my_rank - 1, 1, my_sys->world, &recv_status_exchange_down);
                MPI_Get_count(&recv_status_exchange_down, BodyExchangeType, &num_recv_exchange_down);
                recv_exchange_down = new BodyExchange[num_recv_exchange_down];
                MPI_Recv(recv_exchange_down, num_recv_exchange_down, BodyExchangeType, my_rank - 1, 1, my_sys->world,
                         &recv_status_exchange_down);
            }
            if (my_rank != num_ranks - 1) {
                MPI_Probe(my_rank + 1, 2, my_sys->world, &recv_status_exchange_up);
                MPI_Get_count(&recv_status_exchange_up, BodyExchangeType, &num_recv_exchange_up);
                recv_exchange_up = new BodyExchange[num_recv_exchange_up];
                MPI_Recv(recv_exchange_up, num_recv_exchange_up, BodyExchangeType, my_rank + 1, 2, my_sys->world,
                         &recv_status_exchange_up);
            }

            // Send Updates
            if (my_rank != num_ranks - 1) {
                MPI_Isend(&(update_up_buf[0]), num_update_up, BodyUpdateType, my_rank + 1, 3, my_sys->world,
                          &rq_update_up);
            }
            if (my_rank != 0) {
                MPI_Isend(&(update_down_buf[0]), num_update_down, BodyUpdateType, my_rank - 1, 4, my_sys->world,
                          &rq_update_down);
            }

            // Recv Updates
            if (my_rank != 0) {
                MPI_Probe(my_rank - 1, 3, my_sys->world, &recv_status_update_down);
                MPI_Get_count(&recv_status_update_down, BodyUpdateType, &num_recv_update_down);
                recv_update_down = new BodyUpdate[num_recv_update_down];
                MPI_Recv(recv_update_down, num_recv_update_down, BodyUpdateType, my_rank - 1, 3, my_sys->world,
                         &recv_status_update_down);
            }
            if (my_rank != num_ranks - 1) {
                MPI_Probe(my_rank + 1, 4, my_sys->world, &recv_status_update_up);
                MPI_Get_count(&recv_status_update_up, BodyUpdateType, &num_recv_update_up);
                recv_update_up = new BodyUpdate[num_recv_update_up];
                MPI_Recv(recv_update_up, num_recv_update_up, BodyUpdateType, my_rank + 1, 4, my_sys->world,
                         &recv_status_update_up);
            }

            // Send Takes
            if (my_rank != num_ranks - 1) {
                MPI_Isend(&(update_take_up[0]), num_take_up, MPI_UNSIGNED, my_rank + 1, 5, my_sys->world, &rq_take_up);
            }
            if (my_rank != 0) {
                MPI_Isend(&(update_take_down[0]), num_take_down, MPI_UNSIGNED, my_rank - 1, 6, my_sys->world,
                          &rq_take_down);
            }

            // Recv Takes
            if (my_rank != 0) {
                MPI_Probe(my_rank - 1, 5, my_sys->world, &recv_status_take_down);
                MPI_Get_count(&recv_status_take_down, MPI_UNSIGNED, &num_recv_take_down);
                recv_take_down = new uint[num_recv_take_down];
                MPI_Recv(recv_take_down, num_recv_take_down, MPI_UNSIGNED, my_rank - 1, 5, my_sys->world,
                         &recv_status_take_down);
            }
            if (my_rank != num_ranks - 1) {
                MPI_Probe(my_rank + 1, 6, my_sys->world, &recv_status_take_up);
                MPI_Get_count(&recv_status_take_up, MPI_UNSIGNED, &num_recv_take_up);
                recv_take_up = new uint[num_recv_take_up];
                MPI_Recv(recv_take_up, num_recv_take_up, MPI_UNSIGNED, my_rank + 1, 6, my_sys->world,
                         &recv_status_take_up);
            }

            // Make sure all non-blocking communications are done.
            if (my_rank != num_ranks - 1) {
                MPI_Wait(&rq_exchange_up, MPI_STATUS_IGNORE);
                MPI_Wait(&rq_update_up, MPI_STATUS_IGNORE);
                MPI_Wait(&rq_take_up, MPI_STATUS_IGNORE);
            }
            if (my_rank != 0) {
                MPI_Wait(&rq_exchange_down, MPI_STATUS_IGNORE);
                MPI_Wait(&rq_update_down, MPI_STATUS_IGNORE);
                MPI_Wait(&rq_take_down, MPI_STATUS_IGNORE);
            }
        }  // End of send/recv section

// TODO could do in parallel if counting the spaces in the buffers in the first pass
// Pack Shapes Up
#pragma omp section
        {
            for (auto itr_up = exchanges_up.begin(); itr_up != exchanges_up.end(); itr_up++) {
                num_shapes_up += PackShapes(&shapes_up, *itr_up);
            }

            if (num_shapes_up == 0) {
                Shape shape;
                shape.gid = UINT_MAX;
                shapes_up.push_back(shape);
                num_shapes_up = 1;
            }
        }  // End of pack shapes up section

// Pack Shapes Down
#pragma omp section
        {
            for (auto itr_down = exchanges_down.begin(); itr_down != exchanges_down.end(); itr_down++) {
                num_shapes_down += PackShapes(&shapes_down, *itr_down);
            }
            if (num_shapes_down == 0) {
                Shape shape;
                shape.gid = UINT_MAX;
                shapes_down.push_back(shape);
                num_shapes_down = 1;
            }
        }  // End of pack shapes down section
    }      // End of parallel sections

    // TODO sections?
    if (my_rank != 0)
        ProcessExchanges(num_recv_exchange_down, recv_exchange_down, 0);
    if (my_rank != num_ranks - 1)
        ProcessExchanges(num_recv_exchange_up, recv_exchange_up, 1);

    if (my_rank != 0)
        ProcessUpdates(num_recv_update_down, recv_update_down);
    if (my_rank != num_ranks - 1)
        ProcessUpdates(num_recv_update_up, recv_update_up);

    if (my_rank != 0)
        ProcessTakes(num_recv_take_down, recv_take_down);
    if (my_rank != num_ranks - 1)
        ProcessTakes(num_recv_take_up, recv_take_up);

    // Send Shapes
    if (my_rank != num_ranks - 1) {
        MPI_Isend(&(shapes_up[0]), num_shapes_up, ShapeType, my_rank + 1, 7, my_sys->world, &rq_shapes_up);
    }
    if (my_rank != 0) {
        MPI_Isend(&(shapes_down[0]), num_shapes_down, ShapeType, my_rank - 1, 8, my_sys->world, &rq_shapes_down);
    }

    // Recv Shapes
    if (my_rank != 0) {
        MPI_Probe(my_rank - 1, 7, my_sys->world, &recv_status_shapes_down);
        MPI_Get_count(&recv_status_shapes_down, ShapeType, &num_recv_shapes_down);
        recv_shapes_down = new Shape[num_recv_shapes_down];
        MPI_Recv(recv_shapes_down, num_recv_shapes_down, ShapeType, my_rank - 1, 7, my_sys->world,
                 &recv_status_shapes_down);
    }
    if (my_rank != num_ranks - 1) {
        MPI_Probe(my_rank + 1, 8, my_sys->world, &recv_status_shapes_up);
        MPI_Get_count(&recv_status_shapes_up, ShapeType, &num_recv_shapes_up);
        ////GetLog() << "num_recv_shapes_up" << num_recv_shapes_up << "\n";
        recv_shapes_up = new Shape[num_recv_shapes_up];
        MPI_Recv(recv_shapes_up, num_recv_shapes_up, ShapeType, my_rank + 1, 8, my_sys->world, &recv_status_shapes_up);
    }

    if (my_rank != 0)
        ProcessShapes(num_recv_shapes_down, recv_shapes_down);
    if (my_rank != num_ranks - 1)
        ProcessShapes(num_recv_shapes_up, recv_shapes_up);

    if (my_rank != num_ranks - 1) {
        MPI_Wait(&rq_shapes_up, MPI_STATUS_IGNORE);
    }
    if (my_rank != 0) {
        MPI_Wait(&rq_shapes_down, MPI_STATUS_IGNORE);
    }

    // Free all dynamic memory used for recving
    delete[] recv_exchange_down;
    delete[] recv_exchange_up;
    delete[] recv_update_down;
    delete[] recv_update_up;
    delete[] recv_take_down;
    delete[] recv_take_up;
    delete[] recv_shapes_down;
    delete[] recv_shapes_up;

    MPI_Barrier(my_sys->world);
}

void ChCommDistributed::PackExchange(BodyExchange* buf, int index) {
    // Global Id
    buf->gid = ddm->global_id[index];

    auto& body = my_sys->Get_bodylist()[index];

    // User-controlled identifier
    buf->identifier = body->GetIdentifier();

    // Position and rotation
    real3 pos = data_manager->host_data.pos_rigid[index];
    buf->pos[0] = pos.x;
    buf->pos[1] = pos.y;
    buf->pos[2] = pos.z;

    // Rotation
    quaternion rot = data_manager->host_data.rot_rigid[index];
    buf->rot[0] = rot.w;
    buf->rot[1] = rot.x;
    buf->rot[2] = rot.y;
    buf->rot[3] = rot.z;

    // Velocity
    buf->vel[0] = data_manager->host_data.v[index * 6];
    buf->vel[1] = data_manager->host_data.v[index * 6 + 1];
    buf->vel[2] = data_manager->host_data.v[index * 6 + 2];

    // Angular Velocity
    ChVector<> omega((*data_manager->body_list)[index]->GetWvel_par());
    buf->vel[3] = omega.x();
    buf->vel[4] = omega.y();
    buf->vel[5] = omega.z();

    // Mass
    buf->mass = data_manager->host_data.mass_rigid[index];

    // Inertia
    ChVector<double> inertiaXX = body->GetInertiaXX();
    buf->inertiaXX[0] = inertiaXX.x();
    buf->inertiaXX[1] = inertiaXX.y();
    buf->inertiaXX[2] = inertiaXX.z();

    ChVector<double> inertiaXY = body->GetInertiaXY();
    buf->inertiaXY[0] = inertiaXY.x();
    buf->inertiaXY[1] = inertiaXY.y();
    buf->inertiaXY[2] = inertiaXY.z();

    ////auto body = (*(data_manager->body_list))[index];

    // Collision
    buf->collide = data_manager->host_data.collide_rigid[index];
}

// Unpacks the buffer into a body.
// Note: body is meant to be a ptr into the data structure where the body should be unpacked.
// The body must be in the bodylist already so that GetId is valid
void ChCommDistributed::UnpackExchange(BodyExchange* buf, std::shared_ptr<ChBody> body) {
    // Global Id
    body->SetGid(buf->gid);

    // User-controlled identifier
    body->SetIdentifier(buf->identifier);

    // Position and rotation
    body->SetPos(ChVector<double>(buf->pos[0], buf->pos[1], buf->pos[2]));

    // Rotation
    body->SetRot(ChQuaternion<double>(buf->rot[0], buf->rot[1], buf->rot[2], buf->rot[3]));

    // Linear and Angular Velocities
    body->SetPos_dt(ChVector<double>(buf->vel[0], buf->vel[1], buf->vel[2]));

    body->SetWvel_par(ChVector<double>(buf->vel[3], buf->vel[4], buf->vel[5]));

    // Mass
    body->SetMass(buf->mass);
    body->SetInertiaXX(ChVector<double>(buf->inertiaXX[0], buf->inertiaXX[1], buf->inertiaXX[2]));
    body->SetInertiaXY(ChVector<double>(buf->inertiaXY[0], buf->inertiaXY[1], buf->inertiaXY[2]));
}

// Only packs the essentials for a body update
void ChCommDistributed::PackUpdate(BodyUpdate* buf, int index, int update_type) {
    buf->update_type = update_type;

    // Global Id
    buf->gid = ddm->global_id[index];

    // Position
    buf->pos[0] = data_manager->host_data.pos_rigid[index].x;
    buf->pos[1] = data_manager->host_data.pos_rigid[index].y;
    buf->pos[2] = data_manager->host_data.pos_rigid[index].z;

    // Rotation
    buf->rot[0] = data_manager->host_data.rot_rigid[index].w;
    buf->rot[1] = data_manager->host_data.rot_rigid[index].x;
    buf->rot[2] = data_manager->host_data.rot_rigid[index].y;
    buf->rot[3] = data_manager->host_data.rot_rigid[index].z;

    // Velocity
    buf->vel[0] = data_manager->host_data.v[index * 6];
    buf->vel[1] = data_manager->host_data.v[index * 6 + 1];
    buf->vel[2] = data_manager->host_data.v[index * 6 + 2];

    // Angular Velocity
    ChVector<> omega((*data_manager->body_list)[index]->GetWvel_par());
    buf->vel[3] = omega.x();
    buf->vel[4] = omega.y();
    buf->vel[5] = omega.z();
}

void ChCommDistributed::UnpackUpdate(BodyUpdate* buf, std::shared_ptr<ChBody> body) {
    // Position
    body->SetPos(ChVector<double>(buf->pos[0], buf->pos[1], buf->pos[2]));

    // Rotation
    body->SetRot(ChQuaternion<double>(buf->rot[0], buf->rot[1], buf->rot[2], buf->rot[3]));

    // Linear Velocity
    body->SetPos_dt(ChVector<double>(buf->vel[0], buf->vel[1], buf->vel[2]));

    // Angular Velocity
    body->SetWvel_par(ChVector<double>(buf->vel[3], buf->vel[4], buf->vel[5]));
}

// Packs all shapes for a single body into the buffer
int ChCommDistributed::PackShapes(std::vector<Shape>* buf, int index) {
    int shape_count = ddm->body_shape_count[index];
    shape_container& shape_data = data_manager->cd_data->shape_data;

    auto& body = my_sys->Get_bodylist()[index];

    ChSystemSMC::AdhesionForceModel adhesion_model = ddm->data_manager->settings.solver.adhesion_force_model;
    bool use_material_properties = ddm->data_manager->settings.solver.use_material_properties;

    // Pack each shape on the body
    for (int i = 0; i < shape_count; i++) {
        Shape shape;

        shape.gid = ddm->global_id[index];
        int shape_index =
            ddm->body_shapes[ddm->body_shape_start[index] + i];  // index of the shape in data_manager->shape_data

        int type = shape_data.typ_rigid[shape_index];
        int start = shape_data.start_rigid[shape_index];
        shape.type = type;

        shape.A[0] = shape_data.ObA_rigid[shape_index].x;
        shape.A[1] = shape_data.ObA_rigid[shape_index].y;
        shape.A[2] = shape_data.ObA_rigid[shape_index].z;

        shape.R[0] = shape_data.ObR_rigid[shape_index].w;
        shape.R[1] = shape_data.ObR_rigid[shape_index].x;
        shape.R[2] = shape_data.ObR_rigid[shape_index].y;
        shape.R[3] = shape_data.ObR_rigid[shape_index].z;

        shape.coll_fam[0] = shape_data.fam_rigid[shape_index].x;
        shape.coll_fam[1] = shape_data.fam_rigid[shape_index].y;

        switch (type) {
            case ChCollisionShape::Type::SPHERE:
                shape.data[0] = shape_data.sphere_rigid[start];
                break;
            case ChCollisionShape::Type::BOX:
                shape.data[0] = shape_data.box_like_rigid[start].x;
                shape.data[1] = shape_data.box_like_rigid[start].y;
                shape.data[2] = shape_data.box_like_rigid[start].z;
                break;
            case ChCollisionShape::Type::TRIANGLEMESH:
                // Pack B
                shape.data[0] = shape_data.triangle_rigid[start + 1].x;
                shape.data[1] = shape_data.triangle_rigid[start + 1].y;
                shape.data[2] = shape_data.triangle_rigid[start + 1].z;

                // Pack C
                shape.data[3] = shape_data.triangle_rigid[start + 2].x;
                shape.data[4] = shape_data.triangle_rigid[start + 2].y;
                shape.data[5] = shape_data.triangle_rigid[start + 2].z;
                break;
            case ChCollisionShape::Type::ELLIPSOID:
                // Pack B
                shape.data[0] = shape_data.box_like_rigid[start].x;
                shape.data[1] = shape_data.box_like_rigid[start].y;
                shape.data[2] = shape_data.box_like_rigid[start].z;
                break;

            default:
                my_sys->ErrorAbort("Invalid shape for transfer\n");
        }

        // Pack contact material
        auto material =
            std::static_pointer_cast<ChMaterialSurfaceSMC>(body->GetCollisionModel()->GetShape(i)->GetMaterial());
        shape.mu = material->GetSfriction();
        switch (adhesion_model) {
            case ChSystemSMC::AdhesionForceModel::Perko:
                // Not yet implemented. Falls through.
            case ChSystemSMC::AdhesionForceModel::Constant:
                shape.cohesion = material->GetAdhesion();
                break;
            case ChSystemSMC::AdhesionForceModel::DMT:
                shape.cohesion = material->GetAdhesionMultDMT();
                break;
        }
        if (use_material_properties) {
            shape.ym_kn = material->GetYoungModulus();
            shape.pr_kt = material->GetPoissonRatio();
            shape.restit_gn = material->GetRestitution();
        } else {
            shape.ym_kn = material->GetKn();
            shape.pr_kt = material->GetKt();
            shape.restit_gn = material->GetGn();
            shape.gt = material->GetGt();
        }

        buf->push_back(shape);
    }
    return shape_count;
}

inline void ChCommDistributed::PackUpdateTake(uint* buf, int index) {
    *buf = ddm->global_id[index];
}