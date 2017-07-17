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

#include <mpi.h>
#include <memory>
#include <omp.h>
#include <forward_list>

#include "chrono_distributed/comm/ChCommDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"
#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"
#include "chrono_distributed/other_types.h"

#include "chrono_parallel/ChDataManager.h"

#include "chrono/physics/ChBody.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

using namespace chrono;
using namespace collision;

ChCommDistributed::ChCommDistributed(ChSystemDistributed* my_sys) {
    this->my_sys = my_sys;
    this->data_manager = my_sys->data_manager;

    ddm = my_sys->ddm;

    // Create and Commit all custom MPI Data Types
    // Exchange // TODO packing and array safety...
    MPI_Datatype type_exchange[4] = {MPI_UNSIGNED, MPI_BYTE, MPI_DOUBLE, MPI_FLOAT};
    int blocklen_exchange[4] = {1, 1, 20, 6};
    MPI_Aint disp_exchange[4];
    disp_exchange[0] = offsetof(BodyExchange, gid);
    disp_exchange[1] = offsetof(BodyExchange, collide);
    disp_exchange[2] = offsetof(BodyExchange, pos);
    disp_exchange[3] = offsetof(BodyExchange, mu);
    MPI_Datatype temp_type;
    MPI_Type_create_struct(4, blocklen_exchange, disp_exchange, type_exchange, &temp_type);
    MPI_Aint lb, extent;
    MPI_Type_get_extent(temp_type, &lb, &extent);  // TODO change extent and possibly internal?
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
    MPI_Datatype type_shape[3] = {MPI_UNSIGNED, MPI_INT, MPI_DOUBLE};
    int blocklen_shape[3] = {1, 1, 9};
    MPI_Aint disp_shape[3];
    disp_shape[0] = offsetof(Shape, gid);
    disp_shape[1] = offsetof(Shape, type);
    disp_shape[2] = offsetof(Shape, A);
    // MPI_Type_create_struct(3, blocklen_shape, disp_shape, type_shape, &ShapeType);
    // MPI_Type_commit(&ShapeType);

    MPI_Datatype temp_type_s;
    MPI_Type_create_struct(3, blocklen_shape, disp_shape, type_shape, &temp_type_s);
    MPI_Aint lb_s, extent_s;
    MPI_Type_get_extent(temp_type_s, &lb_s, &extent_s);  // TODO change extent and possibly internal?
    MPI_Type_create_resized(temp_type_s, lb_s, extent_s, &ShapeType);
    MPI_Type_commit(&ShapeType);
}

ChCommDistributed::~ChCommDistributed() {}

void ChCommDistributed::ProcessExchanges(int num_recv, BodyExchange* buf, int updown) {
    if (buf->gid == UINT_MAX) {
        return;
    }
    // ddm->first_empty = 0;
    std::shared_ptr<ChBody> body;

    for (int n = 0; n < num_recv; n++) {
        ddm->data_manager->system_timer.start("FirstEmpty");
        // Find the next empty slot in the data manager.
        if (ddm->first_empty != data_manager->num_rigid_bodies &&
            ddm->comm_status[ddm->first_empty] != distributed::EMPTY) {
            while (ddm->first_empty < data_manager->num_rigid_bodies &&
                   ddm->comm_status[ddm->first_empty] != distributed::EMPTY) {
                ddm->first_empty++;
            }
        }
        ddm->data_manager->system_timer.start("FirstEmpty");
        // GetLog() << "Rank " << my_sys->GetMyRank() << " First Empty: " <<
        // ddm->data_manager->system_timer.GetTime("FirstEmpty") << "\n";
        // If there are no empty spaces in the data manager, create
        // a new body to add
        if (ddm->first_empty == data_manager->num_rigid_bodies) {
            body = std::make_shared<ChBody>(std::make_shared<ChCollisionModelDistributed>(), ChMaterialSurface::SMC);
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
            my_sys->AddBodyExchange(body, status);
        } else {
            ddm->comm_status[ddm->first_empty] = status;
            body->SetBodyFixed(false);
            ddm->gid_to_localid[body->GetGid()] = body->GetId();
        }
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
        // GetLog() << "Updating GID" << (buf + n)->gid << "Rank " << my_sys->GetMyRank() << " index: " << index <<
        // "\n";
        if (index != -1 && ddm->comm_status[index] != distributed::EMPTY) {
            if (ddm->comm_status[index] != distributed::GHOST_UP &&
                ddm->comm_status[index] != distributed::GHOST_DOWN) {
                GetLog() << "Trying to update a non-ghost body on rank " << my_sys->GetMyRank() << "GID "
                         << (buf + n)->gid << "\n";
            }
            body = (*data_manager->body_list)[index];
            UnpackUpdate(buf + n, body);
            if ((buf + n)->update_type == distributed::FINAL_UPDATE_GIVE) {
                ddm->comm_status[index] = distributed::OWNED;
                GetLog() << "Owning gid " << (buf + n)->gid << " index " << index << " rank " << my_sys->GetMyRank()
                         << "\n";
            }
        } else {
            GetLog() << "GID " << buf->gid << " NOT found rank " << my_sys->GetMyRank() << "\n";
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

    // Each iteration handles all shapes for a single body
    while (n < num_recv) {
        gid = (buf + n)->gid;
        // Create a collision model
        GetLog() << "Unpacking shapes for gid " << (buf + n)->gid << "\n";

        auto model = std::make_shared<ChCollisionModelDistributed>();

        GetLog() << "Setting collision model\n";
        // Done creating this body's model
        (*ddm->data_manager->body_list)[ddm->GetLocalIndex(gid)]->SetCollisionModel(model);

        // Single model
        ChVector<double> A((buf + n)->A[0], (buf + n)->A[1], (buf + n)->A[2]);
        // Each iteration handles a single shape for the body
        while ((buf + n)->gid == gid) {
            switch ((buf + n)->type) {
                case chrono::collision::SPHERE:
                    GetLog() << "Unpacking Sphere rank " << my_sys->GetMyRank() << "\n";
                    model->AddSphere((buf + n)->data[0], A);
                    break;
                    /*    case chrono::collision::BOX:
                            GetLog() << "Unpacking Box\n";
                            model->AddBox((buf + n)->data[0], (buf + n)->data[1], (buf + n)->data[2],
                                          A);  // TODO does anything with pos ? // TODO need to add rotation! also
                       how?*/
                    break;
                default:
                    GetLog() << "Unpacking undefined collision shape\n";
            }
            n++;
        }

        GetLog() << "Building collision model\n";
        model->BuildModel();  // Calls collisionsystem::add
    }
}

// Handle all necessary communication
void ChCommDistributed::Exchange() {
    MPI_Barrier(my_sys->world);

    int my_rank = my_sys->GetMyRank();
    int num_ranks = my_sys->GetNumRanks();
    std::forward_list<int> exchanges_up;
    std::forward_list<int> exchanges_down;

    // Saves a reference copy for consistency in the threads.
    ddm->curr_status = ddm->comm_status;

    // TODO do scans to count sizes before allocating
    // Send Buffers
    BodyExchange exchange_up_buf[10000];
    BodyExchange exchange_down_buf[10000];
    BodyUpdate update_up_buf[10000];
    BodyUpdate update_down_buf[10000];
    Shape shapes_up[10000];
    Shape shapes_down[10000];
    uint update_take_up[10000];
    uint update_take_down[10000];

    // Send Counts
    int num_exchange_up = 0;
    int num_exchange_down = 0;
    int num_update_up = 0;
    int num_update_down = 0;
    int num_shapes_up = 0;
    int num_shapes_down = 0;
    int num_take_up = 0;
    int num_take_down = 0;

    // Output buffers for thread-safety
    std::string exchange_string = "";
    std::string update_string = "";
    std::string take_string = "";

#pragma omp parallel sections
    {
// Exchange Loop
#pragma omp section
        {
            for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
                // Skip empty bodies or those that this rank isn't responsible for
                int curr_status = ddm->curr_status[i];
                if (curr_status != distributed::OWNED)
                    continue;
                int location = my_sys->domain->GetBodyRegion(i);

                /*
                                if (curr_status == distributed::GLOBAL || curr_status == distributed::EMPTY ||
                                    curr_status == distributed::GHOST_UP || curr_status == distributed::GHOST_DOWN ||
                                    curr_status == distributed::SHARED_UP ||
                                    curr_status == distribtued::SHARED_DOWN)  // TODO Ok to skipped already shared
                                    continue;
                */

                // If the body now affects the next sub-domain
                // If the body is being marked as shared for the first time, the whole
                // body must be packed to create a ghost on another rank
                if ((location == distributed::GHOST_UP || location == distributed::SHARED_UP)) {  // &&
                    // curr_status == distributed::OWNED) {
                    GetLog() << "Exchange: rank " << my_rank << " -- " << ddm->global_id[i] << " --> rank "
                             << my_rank + 1 << " index " << i << "\n";
                    exchange_string += std::string("eu,") + std::to_string(ddm->global_id[i]) + "\n";
                    PackExchange(exchange_up_buf + num_exchange_up, i);
                    num_exchange_up++;
                    ddm->comm_status[i] = distributed::SHARED_UP;
                    exchanges_up.push_front(i);
                }

                // If the body now affects the previous sub-domain:
                // If the body is being marked as shared for the first time, the whole
                // body must be packed to create a ghost on another rank
                else if ((location == distributed::GHOST_DOWN || location == distributed::SHARED_DOWN)) {  // &&
                    //                         curr_status == distributed::OWNED) {
                    GetLog() << "Exchange: rank " << my_rank << " -- " << ddm->global_id[i] << " --> rank "
                             << my_rank - 1 << " index " << i << "\n";
                    exchange_string += std::string("ed,") + std::to_string(ddm->global_id[i]) + "\n";
                    PackExchange(exchange_down_buf + num_exchange_down, i);
                    num_exchange_down++;
                    ddm->comm_status[i] = distributed::SHARED_DOWN;
                    exchanges_down.push_front(i);
                }
            }
        }  // end of exchange section

// Update Loop
#pragma omp section
        {
            // PACKING and UPDATING comm_status of existing bodies
            for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
                // Skip empty bodies or those that this rank isn't responsible for
                int curr_status = ddm->curr_status[i];
                int location = my_sys->domain->GetBodyRegion(i);

                /*if (curr_status == distributed::GLOBAL || curr_status == distributed::EMPTY ||
                    curr_status == distributed::GHOST_UP || curr_status == distributed::GHOST_DOWN)
                    continue;
                */

                if (curr_status != distributed::SHARED_UP && curr_status != distributed::SHARED_DOWN)
                    continue;

                // If the body now affects the next sub-domain
                if ((location == distributed::GHOST_UP || location == distributed::SHARED_UP) &&
                    curr_status == distributed::SHARED_UP) {
                    // If the body has already been shared, it need only update its
                    // corresponding ghost
                    // GetLog() << "Update: rank " << my_rank << " -- " << ddm->global_id[i] << " --> rank " << my_rank
                    // + 1
                    //         << "\n";
                    update_string += std::string("uu,") + std::to_string(ddm->global_id[i]) + "\n";

                    PackUpdate(update_up_buf + num_update_up, i, distributed::UPDATE);
                    num_update_up++;
                }

                // If the body now affects the previous sub-domain:
                else if ((location == distributed::GHOST_DOWN || location == distributed::SHARED_DOWN) &&
                         curr_status == distributed::SHARED_DOWN) {
                    // If the body has already been shared, it need only update its
                    // corresponding ghost
                    // GetLog() << "Update: rank " << my_rank << "--" << ddm->global_id[i] << "-->rank " << my_rank - 1
                    //         << "\n";
                    update_string += std::string("ud,") + std::to_string(ddm->global_id[i]) + "\n";

                    PackUpdate(update_down_buf + num_update_down, i, distributed::UPDATE);
                    num_update_down++;
                }

                // If the body is no longer involved with this rank, it must be removed from
                // this rank
                else if (location == distributed::UNOWNED_UP || location == distributed::UNOWNED_DOWN) {
                    int up;
                    if (location == distributed::UNOWNED_UP && my_rank != num_ranks - 1) {
                        update_string += std::string("gu,") + std::to_string(ddm->global_id[i]) + "\n";
                        PackUpdate(update_up_buf + num_update_up, i, distributed::FINAL_UPDATE_GIVE);
                        num_update_up++;
                        up = 1;
                    } else if (location == distributed::UNOWNED_DOWN && my_rank != 0) {
                        update_string += std::string("gd,") + std::to_string(ddm->global_id[i]) + "\n";
                        PackUpdate(update_down_buf + num_update_down, i, distributed::FINAL_UPDATE_GIVE);
                        num_update_down++;
                        up = -1;
                    }

                    GetLog() << "Give: " << my_rank << " -- " << ddm->global_id[i] << " --> " << my_rank + up << "\n";

                    my_sys->RemoveBodyExchange(i);
                }
            }  // End of packing for loop
        }      // End of update section

// Update Take Loop
#pragma omp section
        {
            // PACKING and UPDATING comm_status of existing bodies
            for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
                // Skip empty bodies or those that this rank isn't responsible for
                int curr_status = ddm->curr_status[i];
                if (curr_status != distributed::SHARED_DOWN && curr_status != distributed::SHARED_UP)
                    continue;

                int location = my_sys->domain->GetBodyRegion(i);

                /*                if (curr_status == distributed::GLOBAL || curr_status == distributed::EMPTY ||
                                    curr_status == distributed::GHOST_UP || curr_status == distributed::GHOST_DOWN)
                                    continue;
                */

                // If the body is no longer involved with either neighbor, remove it from
                // the other rank
                // and take ownership on this rank
                if (location == distributed::OWNED) {
                    if (curr_status == distributed::SHARED_UP) {
                        take_string += std::string("tu,") + std::to_string(ddm->global_id[i]) + "\n";
                        PackUpdateTake(update_take_up + num_take_up, i);
                        num_take_up++;

                    } else if (curr_status == distributed::SHARED_DOWN) {
                        take_string += std::string("td,") + std::to_string(ddm->global_id[i]) + "\n";
                        PackUpdateTake(update_take_down + num_take_down, i);
                        num_take_down++;
                    }
                    ddm->comm_status[i] = distributed::OWNED;
                }
            }  // End of packing for loop
        }      // End of update take loop
    }          // End of parallel sections

    std::string output_string = exchange_string + update_string + take_string;
    my_sys->debug_stream << output_string;
    my_sys->debug_stream << "*\n";

    MPI_Status status_exchange_up;
    MPI_Status status_exchange_down;
    MPI_Status status_update_up;
    MPI_Status status_update_down;
    MPI_Status status_take_up;
    MPI_Status status_take_down;
    MPI_Status status_shapes_up;
    MPI_Status status_shapes_down;

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
    int num_recv_shapes_up;
    int num_recv_shapes_down;

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
                exchange_up_buf->gid = UINT_MAX;
                num_exchange_up = 1;
            }
            if (num_exchange_down == 0) {
                exchange_down_buf->gid = UINT_MAX;
                num_exchange_down = 1;
            }
            if (num_update_up == 0) {
                update_up_buf->gid = UINT_MAX;
                num_update_up = 1;
            }
            if (num_update_down == 0) {
                update_down_buf->gid = UINT_MAX;
                num_update_down = 1;
            }
            if (num_take_up == 0) {
                update_take_up[0] = UINT_MAX;
                num_take_up = 1;
            }
            if (num_take_down == 0) {
                update_take_down[0] = UINT_MAX;
                num_take_down = 1;
            }

            // Send Exchanges
            if (my_rank != num_ranks - 1) {
                MPI_Isend(exchange_up_buf, num_exchange_up, BodyExchangeType, my_rank + 1, 1, my_sys->world,
                          &rq_exchange_up);
            }
            if (my_rank != 0) {
                MPI_Isend(exchange_down_buf, num_exchange_down, BodyExchangeType, my_rank - 1, 2, my_sys->world,
                          &rq_exchange_down);
            }

            // Recv Exchanges
            if (my_sys->GetMyRank() != 0) {
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
                MPI_Isend(update_up_buf, num_update_up, BodyUpdateType, my_rank + 1, 3, my_sys->world, &rq_update_up);
            }
            if (my_rank != 0) {
                MPI_Isend(update_down_buf, num_update_down, BodyUpdateType, my_rank - 1, 4, my_sys->world,
                          &rq_update_down);
            }

            // Recv Updates
            if (my_sys->GetMyRank() != 0) {
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
                MPI_Isend(update_take_up, num_take_up, MPI_UNSIGNED, my_rank + 1, 5, my_sys->world, &rq_take_up);
            }
            if (my_rank != 0) {
                MPI_Isend(update_take_down, num_take_down, MPI_UNSIGNED, my_rank - 1, 6, my_sys->world, &rq_take_down);
            }

            // Recv Takes
            if (my_sys->GetMyRank() != 0) {
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
        }  // End of send/recv section

// TODO could do in parallel if counting the spaces in the buffers in the first pass
// Pack Shapes Up
#pragma omp section
        {
            for (auto itr_up = exchanges_up.begin(); itr_up != exchanges_up.end(); itr_up++) {
                num_shapes_up += PackShapes(shapes_up + num_shapes_up, *itr_up);
            }
            if (num_shapes_up == 0) {
                shapes_up->gid = UINT_MAX;
                num_shapes_up = 1;
            }
        }  // End of pack shapes up section
// Pack Shapes Down
#pragma omp section
        {
            for (auto itr_down = exchanges_down.begin(); itr_down != exchanges_down.end(); itr_down++) {
                num_shapes_down += PackShapes(shapes_down + num_shapes_down, *itr_down);
            }
            if (num_shapes_down == 0) {
                shapes_down->gid = UINT_MAX;
                num_shapes_down = 1;
            }  // End of pack shapes down section
        }
    }  // End of parallel sections

    // TODO sections
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
        MPI_Isend(shapes_up, num_shapes_up, ShapeType, my_rank + 1, 7, my_sys->world, &rq_shapes_up);
    }
    if (my_rank != 0) {
        MPI_Isend(shapes_down, num_shapes_down, ShapeType, my_rank - 1, 8, my_sys->world, &rq_shapes_down);
    }

    // Recv Shapes
    if (my_sys->GetMyRank() != 0) {
        MPI_Probe(my_rank - 1, 7, my_sys->world, &recv_status_shapes_down);
        MPI_Get_count(&recv_status_shapes_down, ShapeType, &num_recv_shapes_down);
        recv_shapes_down = new Shape[num_recv_shapes_down];
        MPI_Recv(recv_shapes_down, num_recv_shapes_down, ShapeType, my_rank - 1, 7, my_sys->world,
                 &recv_status_shapes_down);
    }
    if (my_rank != num_ranks - 1) {
        MPI_Probe(my_rank + 1, 8, my_sys->world, &recv_status_shapes_up);
        MPI_Get_count(&recv_status_shapes_up, ShapeType, &num_recv_shapes_up);
        recv_shapes_up = new Shape[num_recv_shapes_up];
        MPI_Recv(recv_shapes_up, num_recv_shapes_up, ShapeType, my_rank + 1, 8, my_sys->world, &recv_status_shapes_up);
    }

    if (my_rank != 0)
        ProcessShapes(num_recv_shapes_down, recv_shapes_down);
    if (my_rank != num_ranks - 1)
        ProcessShapes(num_recv_shapes_up, recv_shapes_up);

    my_sys->PrintShapeData();
}

void ChCommDistributed::PackExchange(BodyExchange* buf, int index) {
    // Global Id
    buf->gid = ddm->global_id[index];

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
    buf->vel[3] = data_manager->host_data.v[index * 6 + 3];
    buf->vel[4] = data_manager->host_data.v[index * 6 + 4];
    buf->vel[5] = data_manager->host_data.v[index * 6 + 5];

    // Mass
    buf->mass = data_manager->host_data.mass_rigid[index];

    // Inertia
    ChVector<double> inertiaXX = my_sys->bodylist[index]->GetInertiaXX();
    buf->inertiaXX[0] = inertiaXX.x();
    buf->inertiaXX[1] = inertiaXX.y();
    buf->inertiaXX[2] = inertiaXX.z();

    ChVector<double> inertiaXY = my_sys->bodylist[index]->GetInertiaXY();
    buf->inertiaXY[0] = inertiaXY.x();
    buf->inertiaXY[1] = inertiaXY.y();
    buf->inertiaXY[2] = inertiaXY.z();

    // Material SMC
    buf->mu = data_manager->host_data.mu[index];  // Static Friction

    buf->adhesionMultDMT = data_manager->host_data.adhesionMultDMT_data[index];  // Adhesion

    if (data_manager->settings.solver.use_material_properties) {
        buf->ym_kn = data_manager->host_data.elastic_moduli[index].x;  // Young's Modulus
        buf->pr_kt = data_manager->host_data.elastic_moduli[index].y;  // Poisson Ratio
        buf->restit_gn = data_manager->host_data.cr[index];            // Coefficient of Restitution

    } else {
        buf->ym_kn = data_manager->host_data.dem_coeffs[index].x;
        buf->pr_kt = data_manager->host_data.dem_coeffs[index].y;
        buf->restit_gn = data_manager->host_data.dem_coeffs[index].z;
        buf->gt = data_manager->host_data.dem_coeffs[index].w;
    }

    // Collision
    buf->collide = data_manager->host_data.collide_rigid[index];
}

// Unpacks a sphere body from the buffer into body.
// Note: body is meant to be a ptr into the data structure
// where the body should be unpacked.
// The body must be in the bodylist already so that GetId is valid
void ChCommDistributed::UnpackExchange(BodyExchange* buf, std::shared_ptr<ChBody> body) {
    // Global Id
    body->SetGid(buf->gid);

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

    // Material SMC
    std::shared_ptr<ChMaterialSurfaceSMC> mat = std::make_shared<ChMaterialSurfaceSMC>();

    mat->SetFriction(buf->mu);  // Static Friction
    mat->adhesionMultDMT = buf->adhesionMultDMT;

    if (data_manager->settings.solver.use_material_properties) {
        mat->young_modulus = buf->ym_kn;
        mat->poisson_ratio = buf->pr_kt;
        mat->restitution = buf->restit_gn;
    } else {
        mat->kn = buf->ym_kn;
        mat->kt = buf->pr_kt;
        mat->gn = buf->restit_gn;
        mat->gt = buf->gt;
    }

    body->SetMaterialSurface(mat);

    bool collide = buf->collide;
    body->GetCollisionModel()->ClearModel();
}

// Only packs the essentials for a body update
void ChCommDistributed::PackUpdate(BodyUpdate* buf, int index, int update_type) {
    // GetLog() << "Packing update GID " << ddm->global_id[index] << " rank " << my_sys->GetMyRank()
    //         << ((update_type == distributed::FINAL_UPDATE_GIVE) ? " give" : " normal") << "\n";

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
    buf->vel[3] = data_manager->host_data.v[index * 6 + 3];
    buf->vel[4] = data_manager->host_data.v[index * 6 + 4];
    buf->vel[5] = data_manager->host_data.v[index * 6 + 5];
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
int ChCommDistributed::PackShapes(Shape* buf, int index) {
    int shape_count = ddm->body_shape_count[index];

    // Pack each shape on the body
    for (int i = 0; i < shape_count; i++) {
        (buf + i)->gid = ddm->global_id[index];
        GetLog() << "Packing shapes for gid " << ddm->global_id[index];

        int shape_index =
            ddm->body_shapes[ddm->body_shape_start[index] + i];  // index of the shape in data_manager->shape_data

        int type = data_manager->shape_data.typ_rigid[shape_index];
        int start = data_manager->shape_data.start_rigid[shape_index];
        (buf + i)->type = type;

        (buf + i)->A[0] = data_manager->shape_data.ObA_rigid[shape_index].x;
        (buf + i)->A[1] = data_manager->shape_data.ObA_rigid[shape_index].y;
        (buf + i)->A[2] = data_manager->shape_data.ObA_rigid[shape_index].z;

        (buf + i)->R[0] = data_manager->shape_data.ObR_rigid[shape_index].x;  // TODO the order of array is wxyz
        (buf + i)->R[1] = data_manager->shape_data.ObR_rigid[shape_index].y;
        (buf + i)->R[2] = data_manager->shape_data.ObR_rigid[shape_index].z;
        (buf + i)->R[3] = data_manager->shape_data.ObR_rigid[shape_index].w;

        // buf->fam = data_manager->shape_data.fam_rigid[shape_index]; short2??

        switch (type) {
            case chrono::collision::SPHERE:
                (buf + i)->data[0] = data_manager->shape_data.sphere_rigid[start];
                GetLog() << "Packing sphere: " << (buf + i)->data[0];
                break;
            case chrono::collision::BOX:
                (buf + i)->data[0] = data_manager->shape_data.box_like_rigid[start].x;
                (buf + i)->data[1] = data_manager->shape_data.box_like_rigid[start].y;
                (buf + i)->data[2] = data_manager->shape_data.box_like_rigid[start].z;
                break;
            default:
                GetLog() << "Invalid shape for transfer\n";
        }
    }
    return shape_count;
}

void ChCommDistributed::PackUpdateTake(uint* buf, int index) {
    *buf = ddm->global_id[index];
}
