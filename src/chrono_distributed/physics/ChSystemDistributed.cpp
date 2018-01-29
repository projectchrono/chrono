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

#include <cstdlib>

#include <mpi.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>

#include "chrono/collision/ChCCollisionSystem.h"
#include "chrono/physics/ChBody.h"

#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"
#include "chrono_distributed/other_types.h"
#include "chrono_distributed/physics/ChDomainDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/math/other_types.h"
#include "chrono_parallel/math/real3.h"

using namespace chrono;
using namespace collision;

/// Helper function for visualizing the shapes free list.
static void PrintNode(LocalShapeNode* node) {
    std::cout << "| index = " << node->body_shapes_index << " size = " << node->size << " free = " << node->free
              << "| ---> ";
}

ChSystemDistributed::ChSystemDistributed(MPI_Comm world, double ghost_layer, unsigned int max_objects) {
    this->world = world;
    MPI_Comm_size(world, &num_ranks);
    MPI_Comm_rank(world, &my_rank);
    int name_len = -1;
    MPI_Get_processor_name(node_name, &name_len);

    ddm = new ChDistributedDataManager(this);
    domain = new ChDomainDistributed(this);
    comm = new ChCommDistributed(this);

    this->ghost_layer = ghost_layer;
    this->num_bodies_global = 0;

    data_manager->system_timer.AddTimer("Exchange");

    // Reserve starting space
    int init = max_objects;  // / num_ranks;
    bodylist.reserve(init);

    ddm->global_id.reserve(init);
    ddm->comm_status.reserve(init);
    ddm->body_shapes.reserve(init);
    ddm->body_shape_start.reserve(init);
    ddm->body_shape_count.reserve(init);

    data_manager->host_data.pos_rigid.reserve(init);
    data_manager->host_data.rot_rigid.reserve(init);
    data_manager->host_data.active_rigid.reserve(init);
    data_manager->host_data.collide_rigid.reserve(init);
    data_manager->host_data.mass_rigid.reserve(init);

    data_manager->host_data.elastic_moduli.reserve(init);
    data_manager->host_data.mu.reserve(init);
    data_manager->host_data.cr.reserve(init);
    data_manager->host_data.smc_coeffs.reserve(init);
    data_manager->host_data.adhesionMultDMT_data.reserve(init);

    data_manager->shape_data.fam_rigid.reserve(init);
    data_manager->shape_data.id_rigid.reserve(init);
    data_manager->shape_data.typ_rigid.reserve(init);
    data_manager->shape_data.start_rigid.reserve(init);
    data_manager->shape_data.length_rigid.reserve(init);
    data_manager->shape_data.ObR_rigid.reserve(init);
    data_manager->shape_data.ObA_rigid.reserve(init);

    data_manager->shape_data.sphere_rigid.reserve(init);

    collision_system = std::make_shared<ChCollisionSystemDistributed>(data_manager, ddm);

    // Co-simulation
    /* Create and Commit all custom MPI Data Types */
    // CosimForce
    MPI_Datatype type_cosim_force[3] = {MPI_UNSIGNED, MPI_INT, MPI_DOUBLE};
    int blocklen_cosim_force[3] = {1, 1, 3};
    MPI_Aint disp_cosim_force[3];
    disp_cosim_force[0] = offsetof(CosimForce, gid);
    disp_cosim_force[1] = offsetof(CosimForce, owner_rank);
    disp_cosim_force[2] = offsetof(CosimForce, force);
    MPI_Type_create_struct(3, blocklen_cosim_force, disp_cosim_force, type_cosim_force, &CosimForceType);
    PMPI_Type_commit(&CosimForceType);

    // CosimDispl
    MPI_Datatype type_cosim_displ[2] = {MPI_DOUBLE, MPI_UNSIGNED};
    int blocklen_cosim_displ[2] = {13, 1};
    MPI_Aint disp_cosim_displ[2];
    disp_cosim_displ[0] = offsetof(CosimDispl, A);
    disp_cosim_displ[1] = offsetof(CosimDispl, gid);
    MPI_Type_create_struct(2, blocklen_cosim_displ, disp_cosim_displ, type_cosim_displ, &CosimDisplType);
    PMPI_Type_commit(&CosimDisplType);
}

ChSystemDistributed::~ChSystemDistributed() {
    delete domain;
    delete comm;
    // delete ddm;
}

bool ChSystemDistributed::InSub(ChVector<double> pos) {
    int split_axis = domain->GetSplitAxis();

    double pos_axis = pos[split_axis];
    double lo = domain->sublo[split_axis];
    double hi = domain->subhi[split_axis];

    return (pos_axis >= lo - this->ghost_layer) && (pos_axis <= hi + this->ghost_layer);
}

bool ChSystemDistributed::Integrate_Y() {
    assert(domain->IsSplit());

    bool ret = ChSystemParallelSMC::Integrate_Y();
    if (num_ranks != 1) {
        data_manager->system_timer.start("Exchange");
        comm->Exchange();
        data_manager->system_timer.stop("Exchange");
    }
#ifdef DistrProfile
    PrintEfficiency();
#endif
    return ret;
}

void ChSystemDistributed::UpdateRigidBodies() {
    this->ChSystemParallel::UpdateRigidBodies();

#pragma omp parallel for
    for (int i = 0; i < bodylist.size(); i++) {
        ddm->global_id[i] = bodylist[i]->GetGid();
    }
}

void ChSystemDistributed::AddBodyAllRanks(std::shared_ptr<ChBody> newbody) {
    newbody->SetGid(num_bodies_global);
    distributed::COMM_STATUS status = distributed::GLOBAL;

    ddm->body_shape_start.push_back(0);
    ddm->body_shape_count.push_back(0);

    ddm->comm_status.push_back(status);
    ddm->global_id.push_back(newbody->GetGid());

    newbody->SetId(data_manager->num_rigid_bodies);
    bodylist.push_back(newbody);

    ddm->gid_to_localid[newbody->GetGid()] = newbody->GetId();

    data_manager->num_rigid_bodies++;
    newbody->SetSystem(this);

    // actual data is set in UpdateBodies().
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
    data_manager->host_data.collide_rigid.push_back(true);

    // Let derived classes reserve space for specific material surface data
    ChSystemParallelSMC::AddMaterialSurfaceData(newbody);
}

void ChSystemDistributed::AddBody(std::shared_ptr<ChBody> newbody) {
    newbody->SetGid(num_bodies_global);
    distributed::COMM_STATUS status = domain->GetBodyRegion(newbody);

    // Check for collision with this sub-domain
    if (newbody->GetBodyFixed()) {
        ChVector<double> body_min;
        ChVector<double> body_max;
        ChVector<double> sublo(domain->GetSubLo());
        ChVector<double> subhi(domain->GetSubHi());

        newbody->GetCollisionModel()->GetAABB(body_min, body_max);

        // If the part of the body lies in this sub-domain, add it
        if ((body_min.x() <= subhi.x() && sublo.x() <= body_max.x()) &&
            (body_min.y() <= subhi.y() && sublo.y() <= body_max.y()) &&
            (body_min.z() <= subhi.z() && sublo.z() <= body_max.z())) {
            status = distributed::GLOBAL;
            // TODO cut off the shapes that don't affect this sub-domain?
        }
    }

    if (status == distributed::UNOWNED_UP || status == distributed::UNOWNED_DOWN) {
        return;
    }
    // Makes space for shapes TODO Does this work for mid-simulation add by user?
    ddm->body_shape_start.push_back(0);
    ddm->body_shape_count.push_back(0);

    ddm->comm_status.push_back(status);
    ddm->global_id.push_back(newbody->GetGid());

    newbody->SetId(data_manager->num_rigid_bodies);
    bodylist.push_back(newbody);

    ddm->gid_to_localid[newbody->GetGid()] = newbody->GetId();

    data_manager->num_rigid_bodies++;
    newbody->SetSystem(this);  // TODO Syncs collision model

    // actual data is set in UpdateBodies().
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
    data_manager->host_data.collide_rigid.push_back(true);

    // Let derived classes reserve space for specific material surface data
    ChSystemParallelSMC::AddMaterialSurfaceData(newbody);
}

// Should only be called to add a body when there are no free spaces to insert it into
void ChSystemDistributed::AddBodyExchange(std::shared_ptr<ChBody> newbody, distributed::COMM_STATUS status) {
    ddm->comm_status.push_back(status);
    ddm->global_id.push_back(newbody->GetGid());
    newbody->SetId(data_manager->num_rigid_bodies);
    bodylist.push_back(newbody);

    ddm->body_shape_start.push_back(0);
    ddm->body_shape_count.push_back(0);

    data_manager->num_rigid_bodies++;
    newbody->SetBodyFixed(false);

    // NOTE: Ensures that colsys::add isn't called until shapes are added
    newbody->SetCollide(false);
    // NOTE: Clears NEW model - Doesn't call colsys::remove because system isn't set yet
    newbody->GetCollisionModel()->ClearModel();
    newbody->SetSystem(this);

    // Actual data is set in UpdateBodies()
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
    data_manager->host_data.collide_rigid.push_back(true);

    ddm->gid_to_localid[newbody->GetGid()] = newbody->GetId();
    // Let derived classes reserve space for specific material surface data
    ChSystemParallelSMC::AddMaterialSurfaceData(newbody);
}

void ChSystemDistributed::RemoveBodyExchange(int index) {
    ddm->comm_status[index] = distributed::EMPTY;
    bodylist[index]->SetBodyFixed(true);
    bodylist[index]->SetCollide(false);                  // NOTE: Calls collisionsystem::remove
    bodylist[index]->GetCollisionModel()->ClearModel();  // NOTE: Ensures new model is clear
    ddm->gid_to_localid.erase(ddm->global_id[index]);
}

// Trusts the ID to be correct on the body
void ChSystemDistributed::RemoveBody(std::shared_ptr<ChBody> body) {
    int index = body->GetId();
    if (bodylist.size() <= index || body.get() != bodylist[index].get())
        return;

    ddm->comm_status[index] = distributed::EMPTY;
    bodylist[index]->SetBodyFixed(true);
    bodylist[index]->SetCollide(false);
    if (index < ddm->first_empty)
        ddm->first_empty = index;
    GetLog() << "REMOVEBODY\n";
    ddm->gid_to_localid.erase(body->GetGid());
}

// Used to end the program on an error and print a message.
void ChSystemDistributed::ErrorAbort(std::string msg) {
    GetLog() << msg << "\n";
    MPI_Abort(world, MPI_ERR_OTHER);
}

void ChSystemDistributed::PrintBodyStatus() {
    GetLog() << "Rank: " << my_rank << "\n";
    GetLog() << "\tBodylist:\n";
    std::vector<std::shared_ptr<ChBody>>::iterator bl_itr = bodylist.begin();
    int i = 0;
    for (; bl_itr != bodylist.end(); bl_itr++, i++) {
        ChVector<double> pos = (*bl_itr)->GetPos();
        ChVector<double> vel = (*bl_itr)->GetPos_dt();
        if (ddm->comm_status[i] != distributed::EMPTY) {
            printf("\tGlobal ID: %d Pos: %.2f,%.2f,%.2f. Vel: %.2f, %.2f, %.2f, Active: %d Collide: %d Rank: %d\n",
                   (*bl_itr)->GetGid(), pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z(), (*bl_itr)->IsActive(),
                   (*bl_itr)->GetCollide(), my_rank);
        }
    }

    GetLog() << "\tData Manager:\n";
    for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
        int status = ddm->comm_status[i];
        unsigned int gid = ddm->global_id[i];

        if (status != distributed::EMPTY) {
            if (status == distributed::SHARED_DOWN) {
                GetLog() << "\tGlobal ID: " << gid << " Shared down";
            } else if (status == distributed::SHARED_UP) {
                GetLog() << "\tGlobal ID: " << gid << " Shared up";
            } else if (status == distributed::GHOST_UP) {
                GetLog() << "\tGlobal ID: " << gid << " Ghost up";
            } else if (status == distributed::GHOST_DOWN) {
                GetLog() << "\tGlobal ID: " << gid << " Ghost down";
            } else if (status == distributed::OWNED) {
                GetLog() << "\tGlobal ID: " << gid << " Owned";
            } else if (status == distributed::GLOBAL) {
                GetLog() << "\tGlobal ID: " << gid << " Global";
            } else {
                GetLog() << "\tERROR: Global ID: " << gid << " Undefined comm_status\n";
            }

            real3 pos = data_manager->host_data.pos_rigid[i];
            fprintf(stdout, " Pos: %.2f,%.2f,%.2f RANK: %d\n", pos.x, pos.y, pos.z, my_rank);
        }
    }
}

void ChSystemDistributed::PrintShapeData() {
    std::vector<std::shared_ptr<ChBody>>::iterator bl_itr = bodylist.begin();
    int i = 0;
    for (; bl_itr != bodylist.end(); bl_itr++, i++) {
        if (ddm->comm_status[i] != distributed::EMPTY) {
            int body_start = ddm->body_shape_start[i];
            printf("Body %d: ", ddm->global_id[i]);
            for (int j = 0; j < ddm->body_shape_count[i]; j++) {
                std::string msg;
                int shape_index = ddm->body_shapes[body_start + j];

                switch (data_manager->shape_data.typ_rigid[shape_index]) {
                    case chrono::collision::SPHERE:
                        printf(
                            "%d | Sphere: r %.3f, ", my_rank,
                            data_manager->shape_data.sphere_rigid[data_manager->shape_data.start_rigid[shape_index]]);
                        break;
                    case chrono::collision::BOX:
                        printf("%d | Box: ", my_rank);  // TODO
                        break;
                    default:
                        printf("Undefined Shape, ");
                }
            }
            printf("\n");
        }
    }

    printf("%d | NumShapes: %d NumBodies: %d\n", my_rank, ddm->data_manager->shape_data.id_rigid.size(), i);
    printf("%d | num_rigid_shapes: %d, num_rigid_bodies: %d\n", my_rank, ddm->data_manager->num_rigid_shapes,
           ddm->data_manager->num_rigid_bodies);
}

void ChSystemDistributed::PrintEfficiency() {
    double used = 0.0;
    for (int i = 0; i < bodylist.size(); i++) {
        if (ddm->comm_status[i] != distributed::EMPTY) {
            used += 1.0;
        }
    }
    used = used / bodylist.size();

    double shapes_used = 0.0;
    for (int i = 0; i < data_manager->shape_data.id_rigid.size(); i++) {
        if (data_manager->shape_data.id_rigid[i] != UINT_MAX) {
            shapes_used += 1.0;
        }
    }

    shapes_used = shapes_used / data_manager->shape_data.id_rigid.size();

    FILE* fp;
    std::string filename = std::to_string(my_rank) + "Efficency.txt";
    fp = fopen(filename.c_str(), "a");
    if (fp != NULL) {
        fprintf(fp, "Bodies: %.2f Shapes: %.2f\n", used, shapes_used);
        fclose(fp);
    }
}

double ChSystemDistributed::GetLowestZ(uint* local_id) {
    double min = 0;
    for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
        if (ddm->comm_status[i] != distributed::EMPTY && data_manager->host_data.pos_rigid[i][2] < min) {
            min = data_manager->host_data.pos_rigid[i][2];
            *local_id = i;
        }
    }
    return min;
}

void ChSystemDistributed::CheckIds() {
    for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
        if (bodylist[i]->GetId() != i) {
            GetLog() << "Mismatched ID " << i << " Ranks " << my_rank << "\n";
        }
    }
}

// Co-simuation
// Call on all ranks, rank 0 will return results
// Make sure forces is large enough to hold all of the data
int ChSystemDistributed::CollectCosimForces(CosimForce* forces) {
    // Gather forces on cosim bodies
    std::vector<CosimForce> send;
    for (uint i = ddm->first_cosim; i <= ddm->last_cosim; i++) {
        int local = ddm->GetLocalIndex(i);
        if (local != -1 &&
            (ddm->comm_status[local] == distributed::OWNED || ddm->comm_status[local] == distributed::SHARED_UP ||
             ddm->comm_status[local] == distributed::SHARED_DOWN)) {
            // Get force on body at index local
            int contact_index = data_manager->host_data.ct_body_map[local];
            real3 f = data_manager->host_data.ct_body_force[contact_index];
            CosimForce cf = {.gid = i, .owner_rank = my_rank, .force = {f[0], f[1], f[2]}};
            send.push_back(std::move(cf));
        }
    }

    // Rank 0 recvs all messages sent to it and appends the values into its gid vector
    MPI_Request r_bar;
    MPI_Status s_bar;
    CosimForce* buf = forces;
    int num_gids = 0;
    if (my_rank == 0) {
        // Write rank 0 values
        if (send.size() > 0) {
            std::memcpy(buf, send.data(), sizeof(CosimForce) * send.size());
            buf += send.size();
            num_gids += send.size();
        }
        MPI_Ibarrier(MPI_COMM_WORLD, &r_bar);
        MPI_Status s_prob;
        int message_waiting = 0;
        int r_bar_flag = 0;

        MPI_Test(&r_bar, &r_bar_flag, &s_bar);
        while (!r_bar_flag) {
            MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &message_waiting, &s_prob);
            if (message_waiting) {
                int count;
                MPI_Get_count(&s_prob, CosimForceType, &count);
                MPI_Request r_recv;
                MPI_Irecv(buf, count, CosimForceType, s_prob.MPI_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &r_recv);
                buf += count;
                num_gids += count;
            }
            MPI_Test(&r_bar, &r_bar_flag, &s_bar);
        }
    }
    // All other ranks send their elements, if they have any
    else {
        if (send.size() > 0) {
            MPI_Request r_send;
            // Non-blocking synchronous Send
            MPI_Issend(send.data(), send.size(), CosimForceType, 0, 0, MPI_COMM_WORLD, &r_send);

            MPI_Status s_send;
            MPI_Wait(&r_send, &s_send);
        }
        // Reaching here indicates to the comm that this rank's message has been recved by rank 0
        MPI_Ibarrier(MPI_COMM_WORLD, &r_bar);
    }

    MPI_Status stat;
    MPI_Wait(&r_bar, &stat);  // TODO makes all ranks wait? Could non-masters be doing anything?

    return num_gids;
}

void ChSystemDistributed::DistributeCosimPositions(CosimDispl* displacements, uint* GIDs, int* ranks, int size) {
    CosimDispl* displ;
    int count;
    CosimDispl empty;
    empty.gid = UINT_MAX;

    // Rank 0 sends the appropriate data to each rank
    if (my_rank == 0) {
        std::vector<std::vector<CosimDispl>> send_bufs;
        send_bufs.resize(num_ranks);
        for (int i = 0; i < size; i++) {
            send_bufs[ranks[i]].push_back(displacements[i]);
        }

        for (int i = 0; i < num_ranks; i++) {
            if (send_bufs[i].empty()) {
                send_bufs[i].push_back(empty);
            }
        }

        for (int i = 1; i < num_ranks; i++) {
            MPI_Request req;
            MPI_Send(&(send_bufs[i][0]), send_bufs[i].size(), CosimDisplType, i, 0, world);
        }
        count = send_bufs[0].size();
        displ = new CosimDispl[count];
        std::copy(send_bufs[0].begin(), send_bufs[0].end(), displ);
    }
    // Other ranks recieve their data from rank 0
    else {
        MPI_Status stat;
        MPI_Probe(0, 0, world, &stat);
        MPI_Get_count(&stat, CosimDisplType, &count);
        displ = new CosimDispl[count];
        MPI_Recv(displ, count, CosimDisplType, 0, 0, world, &stat);
    }

    // If non-empty message
    if (displ[0].gid != UINT_MAX) {
        // Update the positions of each cosimulation triangle on the rank
        for (int i = 0; i < count; i++) {
            int local = ddm->gid_to_localid[displ[i].gid];  // NOTE: Don't need to check since already validated

            // Get shape index NOTE: Supposes one shape per cosim body
            // type is triangle, count is 1,
            shape_container& shape_data = data_manager->shape_data;
            int start = ddm->body_shape_start[local];
            int cs_index = ddm->body_shapes[start];
            int cs_start = shape_data.start_rigid[cs_index];

            // Overwrite triangle data
            shape_data.ObA_rigid[cs_index] = real3(displ[i].A[0], displ[i].A[1], displ[i].A[2]);
            shape_data.ObR_rigid[cs_index] = quaternion(displ[i].R[0], displ[i].R[1], displ[i].R[2], displ[i].R[3]);
            shape_data.triangle_rigid[cs_start] = real3(displ[i].A[0], displ[i].A[1], displ[i].A[2]);
            shape_data.triangle_rigid[cs_start + 1] = real3(displ[i].B[0], displ[i].B[1], displ[i].B[2]);
            shape_data.triangle_rigid[cs_start + 2] = real3(displ[i].C[0], displ[i].C[1], displ[i].C[2]);
        }
    }

    delete[] displ;
}

void ChSystemDistributed::SetFirstCosimGID(uint gid) {
    ddm->first_cosim = gid;
}
void ChSystemDistributed::SetLastCosimGID(uint gid) {
    ddm->last_cosim = gid;
}

void ChSystemDistributed::SanityCheck() {
    // Check all shapes
    for (auto itr = data_manager->shape_data.id_rigid.begin(); itr != data_manager->shape_data.id_rigid.end(); itr++) {
        if (*itr != UINT_MAX) {
            int local_id = *itr;
            distributed::COMM_STATUS stat = ddm->comm_status[local_id];
            if (stat == distributed::UNOWNED_UP || stat == distributed::UNOWNED_DOWN) {
                GetLog() << "ERROR: Deactivated shape on Activated id. ID: " << local_id << " rank " << my_rank << "\n";
            }
        }
    }

    // Check gid mapping
    for (int i = 0; i < ddm->global_id.size(); i++) {
        uint gid = ddm->global_id[i];
        int local_id = ddm->GetLocalIndex(gid);
        if (ddm->comm_status[i] != distributed::EMPTY && i != local_id) {
            GetLog() << "ERROR: Mismatched local index. i: " << i << ". gid " << gid << ". local_id " << local_id
                     << ". rank " << my_rank << "\n";
        }
    }

    // Check comm_status
    for (uint local_id = 0; local_id < ddm->comm_status.size(); local_id++) {
        uint gid = ddm->global_id[local_id];
        if (ddm->GetLocalIndex(gid) == -1 && ddm->comm_status[local_id] != distributed::EMPTY) {
            GetLog() << "ERROR: Unmapped GID with non-EMPTY comm_status rank " << my_rank << "\n";
        }
    }

    // Check free list and shape mapping
    // GetLog() << "rank " << my_rank << " ";
    LocalShapeNode* curr = ddm->local_free_shapes;
    while (curr != NULL) {
        // PrintNode(curr);
        if (curr->next != NULL && (curr->body_shapes_index + curr->size != curr->next->body_shapes_index)) {
            GetLog() << "ERROR: Free list has gap or overlap rank " << my_rank << "\n";
        }

        curr = curr->next;
    }
    // std::cout << " NULL\n";
}