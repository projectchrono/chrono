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
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/physics/ChBody.h"

#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/collision/ChCollisionModelDistributed.h"
#include "chrono_distributed/collision/ChCollisionSystemDistributed.h"
#include "chrono_distributed/physics/ChDomainDistributed.h"
#include "chrono_distributed/physics/ChSystemDistributed.h"

#include "chrono_multicore/ChDataManager.h"
#include "chrono_multicore/ChMulticoreDefines.h"

using namespace chrono;
using namespace collision;

// Structure of force data used internally for MPI sending contact forces.
struct internal_force {
    uint gid;
    double force[3];
};

// Helper function for visualizing the shapes free list.
static void PrintNode(LocalShapeNode* node) {
    std::cout << "| index = " << node->body_shapes_index << " size = " << node->size << " free = " << node->free
              << "| ---> ";
}

ChSystemDistributed::ChSystemDistributed(MPI_Comm communicator, double ghostlayer, unsigned int maxobjects)
    : ghost_layer(ghostlayer), master_rank(0), num_bodies_global(0) {
    MPI_Comm_dup(communicator, &world);
    MPI_Comm_size(world, &num_ranks);
    MPI_Comm_rank(world, &my_rank);
    int name_len = -1;
    MPI_Get_processor_name(node_name, &name_len);

    ddm = new ChDistributedDataManager(this);
    domain = new ChDomainDistributed(this);
    comm = new ChCommDistributed(this);

    data_manager->system_timer.AddTimer("Exchange");

    // Reserve starting space
    int init = maxobjects;  // / num_ranks;
    assembly.bodylist.reserve(init);

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

    data_manager->cd_data->shape_data.fam_rigid.reserve(init);
    data_manager->cd_data->shape_data.id_rigid.reserve(init);
    data_manager->cd_data->shape_data.typ_rigid.reserve(init);
    data_manager->cd_data->shape_data.start_rigid.reserve(init);
    data_manager->cd_data->shape_data.length_rigid.reserve(init);
    data_manager->cd_data->shape_data.ObR_rigid.reserve(init);
    data_manager->cd_data->shape_data.ObA_rigid.reserve(init);

    data_manager->cd_data->shape_data.sphere_rigid.reserve(init);

    collision_system = chrono_types::make_shared<ChCollisionSystemDistributed>(data_manager, ddm);

    /* Create and Commit all custom MPI Data Types */
    MPI_Datatype type_force[2] = {MPI_UNSIGNED, MPI_DOUBLE};
    int blocklen_force[2] = {1, 3};
    MPI_Aint disp_force[2];
    disp_force[0] = offsetof(internal_force, gid);
    disp_force[1] = offsetof(internal_force, force);
    MPI_Type_create_struct(2, blocklen_force, disp_force, type_force, &InternalForceType);
    PMPI_Type_commit(&InternalForceType);
}

ChSystemDistributed::~ChSystemDistributed() {
    delete domain;
    delete comm;
    // delete ddm;
}

bool ChSystemDistributed::InSub(const ChVector<double>& pos) const {
    int split_axis = domain->GetSplitAxis();

    double pos_axis = pos[split_axis];
    double lo = domain->sublo[split_axis];
    double hi = domain->subhi[split_axis];

    return (pos_axis >= lo - this->ghost_layer) && (pos_axis <= hi + this->ghost_layer);
}

bool ChSystemDistributed::Integrate_Y() {
    assert(domain->IsSplit());
    ddm->initial_add = false;

    bool ret = ChSystemMulticoreSMC::Integrate_Y();
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
    this->ChSystemMulticore::UpdateRigidBodies();

#pragma omp parallel for
    for (int i = 0; i < assembly.bodylist.size(); i++) {
        ddm->global_id[i] = assembly.bodylist[i]->GetGid();
    }
}

ChBody* ChSystemDistributed::NewBody() {
    return new ChBody(chrono_types::make_shared<collision::ChCollisionModelDistributed>());
}

ChBodyAuxRef* ChSystemDistributed::NewBodyAuxRef() {
    return new ChBodyAuxRef(chrono_types::make_shared<collision::ChCollisionModelDistributed>());
}

void ChSystemDistributed::AddBodyAllRanks(std::shared_ptr<ChBody> newbody) {
    newbody->SetGid(num_bodies_global);
    num_bodies_global++;

    distributed::COMM_STATUS status = distributed::GLOBAL;

    ddm->body_shape_start.push_back(0);
    ddm->body_shape_count.push_back(0);

    ddm->comm_status.push_back(status);
    ddm->global_id.push_back(newbody->GetGid());

    newbody->SetId(data_manager->num_rigid_bodies);
    assembly.bodylist.push_back(newbody);

    ddm->gid_to_localid[newbody->GetGid()] = newbody->GetId();

    data_manager->num_rigid_bodies++;
    newbody->SetSystem(this);

    // actual data is set in UpdateBodies().
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
    data_manager->host_data.collide_rigid.push_back(true);

    // Let derived classes reserve space for specific material surface data
    ChSystemMulticoreSMC::AddMaterialSurfaceData(newbody);
}

void ChSystemDistributed::AddBody(std::shared_ptr<ChBody> newbody) {
    // Assign global ID to the body (whether or not it is kept on this rank)
    newbody->SetGid(num_bodies_global);

    // Increment global body ID counter.
    num_bodies_global++;

    // Add body on the rank whose sub-domain contains the current body position.
    if (!InSub(newbody->GetPos())) {
        return;
    }

    distributed::COMM_STATUS status = domain->GetBodyRegion(newbody);

    // Check for collision with this sub-domain
    if (newbody->GetBodyFixed()) {
        ChVector<double> body_min;
        ChVector<double> body_max;
        ChVector<double> sublo(domain->GetSubLo());
        ChVector<double> subhi(domain->GetSubHi());

        newbody->GetCollisionModel()->GetAABB(body_min, body_max);

        body_min += newbody->GetPos();
        body_max += newbody->GetPos();

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
    assembly.bodylist.push_back(newbody);

    ddm->gid_to_localid[newbody->GetGid()] = newbody->GetId();

    data_manager->num_rigid_bodies++;
    newbody->SetSystem(this);  // NOTE Syncs collision model

    // Actual data is set in UpdateBodies().
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
    data_manager->host_data.collide_rigid.push_back(true);

    // Let derived classes reserve space for specific material surface data
    ChSystemMulticoreSMC::AddMaterialSurfaceData(newbody);
}

// Should only be called to add a body when there are no free spaces to insert it into
void ChSystemDistributed::AddBodyExchange(std::shared_ptr<ChBody> newbody, distributed::COMM_STATUS status) {
    ddm->comm_status.push_back(status);
    ddm->global_id.push_back(newbody->GetGid());
    newbody->SetId(data_manager->num_rigid_bodies);
    assembly.bodylist.push_back(newbody);

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
    ChSystemMulticoreSMC::AddMaterialSurfaceData(newbody);
}

void ChSystemDistributed::RemoveBodyExchange(int index) {
    ddm->comm_status[index] = distributed::EMPTY;
    assembly.bodylist[index]->SetBodyFixed(true);
    assembly.bodylist[index]->SetCollide(false);                  // NOTE: Calls collisionsystem::remove
    assembly.bodylist[index]->GetCollisionModel()->ClearModel();  // NOTE: Ensures new model is clear
    ddm->gid_to_localid.erase(ddm->global_id[index]);
}

// Trusts the ID to be correct on the body
void ChSystemDistributed::RemoveBody(std::shared_ptr<ChBody> body) {
    uint index = body->GetId();
    if (assembly.bodylist.size() <= index || body.get() != assembly.bodylist[index].get())
        return;

    ddm->comm_status[index] = distributed::EMPTY;
    assembly.bodylist[index]->SetBodyFixed(true);
    assembly.bodylist[index]->SetCollide(false);  // NOTE: Calls collisionsystem::remove
    if (index < ddm->first_empty)
        ddm->first_empty = index;
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
    std::vector<std::shared_ptr<ChBody>>::iterator bl_itr = assembly.bodylist.begin();
    uint i = 0;
    for (; bl_itr != assembly.bodylist.end(); bl_itr++, i++) {
        ChVector<double> pos = (*bl_itr)->GetPos();
        ChVector<double> vel = (*bl_itr)->GetPos_dt();
        if (ddm->comm_status[i] != distributed::EMPTY) {
            printf("\tGlobal ID: %d Pos: %.2f,%.2f,%.2f. Vel: %.2f, %.2f, %.2f, Active: %d Collide: %d Rank: %d\n",
                   (*bl_itr)->GetGid(), pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z(), (*bl_itr)->IsActive(),
                   (*bl_itr)->GetCollide(), my_rank);
        }
    }

    GetLog() << "\tData Manager:\n";
    for (i = 0; i < data_manager->num_rigid_bodies; i++) {
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
    const auto& shape_data = data_manager->cd_data->shape_data;

    std::vector<std::shared_ptr<ChBody>>::iterator bl_itr = assembly.bodylist.begin();
    int i = 0;
    for (; bl_itr != assembly.bodylist.end(); bl_itr++, i++) {
        if (ddm->comm_status[i] != distributed::EMPTY) {
            int body_start = ddm->body_shape_start[i];
            printf("Body %d: ", ddm->global_id[i]);
            for (int j = 0; j < ddm->body_shape_count[i]; j++) {
                std::string msg;
                int shape_index = ddm->body_shapes[body_start + j];

                switch (shape_data.typ_rigid[shape_index]) {
                    case ChCollisionShape::Type::SPHERE:
                        printf(
                            "%d | Sphere: r %.3f, ", my_rank,
                            shape_data.sphere_rigid[shape_data.start_rigid[shape_index]]);
                        break;
                    case ChCollisionShape::Type::BOX:
                        printf("%d | Box: ", my_rank);
                        break;
                    default:
                        printf("Undefined Shape, ");
                }
            }
            printf("\n");
        }
    }

    printf("%d | NumShapes: %lu NumBodies: %d\n", my_rank, (unsigned long)shape_data.id_rigid.size(), i);
    printf("%d | num_rigid_shapes: %d, num_rigid_bodies: %d\n", my_rank, data_manager->cd_data->num_rigid_shapes,
           ddm->data_manager->num_rigid_bodies);
}

void ChSystemDistributed::PrintEfficiency() {
    const auto& shape_data = data_manager->cd_data->shape_data;

    double used = 0.0;
    for (int i = 0; i < assembly.bodylist.size(); i++) {
        if (ddm->comm_status[i] != distributed::EMPTY) {
            used += 1.0;
        }
    }
    used = used / assembly.bodylist.size();

    double shapes_used = 0.0;
    for (int i = 0; i < shape_data.id_rigid.size(); i++) {
        if (shape_data.id_rigid[i] != UINT_MAX) {
            shapes_used += 1.0;
        }
    }

    shapes_used = shapes_used / shape_data.id_rigid.size();

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
    for (uint i = 0; i < data_manager->num_rigid_bodies; i++) {
        if (ddm->comm_status[i] != distributed::EMPTY && data_manager->host_data.pos_rigid[i][2] < min) {
            min = data_manager->host_data.pos_rigid[i][2];
            *local_id = i;
        }
    }
    return min;
}

double ChSystemDistributed::GetHighestZ() {
    double max = DBL_MIN;
    for (uint i = 0; i < data_manager->num_rigid_bodies; i++) {
        if (ddm->comm_status[i] != distributed::EMPTY && data_manager->host_data.pos_rigid[i][2] > max) {
            max = data_manager->host_data.pos_rigid[i][2];
        }
    }
    double reduction;
    MPI_Allreduce(&max, &reduction, 1, MPI_DOUBLE, MPI_MAX, world);
    return reduction;
}

void ChSystemDistributed::CheckIds() {
    for (uint i = 0; i < data_manager->num_rigid_bodies; i++) {
        if (assembly.bodylist[i]->GetId() != i) {
            GetLog() << "Mismatched ID " << i << " Ranks " << my_rank << "\n";
        }
    }
}

void ChSystemDistributed::SanityCheck() {
    const auto& shape_data = data_manager->cd_data->shape_data;

    // Check all shapes
    for (auto itr = shape_data.id_rigid.begin(); itr != shape_data.id_rigid.end(); itr++) {
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

int ChSystemDistributed::RemoveBodiesBelow(double z) {
    int count = 0;
    for (uint i = 0; i < data_manager->num_rigid_bodies; i++) {
        auto status = ddm->comm_status[i];
        if (status != distributed::EMPTY && data_manager->host_data.pos_rigid[i][2] < z) {
            RemoveBody(assembly.bodylist[i]);
            if (status == distributed::OWNED || status == distributed::SHARED_DOWN ||
                status == distributed::SHARED_UP) {
                count++;
            }
        }
    }
    int final_count;
    MPI_Reduce((void*)&count, (void*)&final_count, 1, MPI_INT, MPI_SUM, 0, world);
    return final_count;
}

void ChSystemDistributed::SetBodyStates(const std::vector<uint>& gids, const std::vector<BodyState>& states) {
    for (size_t i = 0; i < gids.size(); i++) {
        SetBodyState(gids[i], states[i]);
    }
}

void ChSystemDistributed::SetBodyState(uint gid, const BodyState& state) {
    int local_id = ddm->GetLocalIndex(gid);
    if (local_id != -1 && ddm->comm_status[local_id] != distributed::EMPTY) {
        assembly.bodylist[local_id]->SetPos(state.pos);
        assembly.bodylist[local_id]->SetRot(state.rot);
        assembly.bodylist[local_id]->SetPos_dt(state.pos_dt);
        assembly.bodylist[local_id]->SetRot_dt(state.rot_dt);
    }
}

void ChSystemDistributed::SetSphereShapes(const std::vector<uint>& gids,
                                          const std::vector<int>& shape_idx,
                                          const std::vector<double>& radii) {
    for (size_t i = 0; i < gids.size(); i++) {
        SetSphereShape(gids[i], shape_idx[i], radii[i]);
    }
}

void ChSystemDistributed::SetSphereShape(uint gid, int shape_idx, double radius) {
    auto& shape_data = data_manager->cd_data->shape_data;

    int local_id = ddm->GetLocalIndex(gid);
    if (local_id != -1 && ddm->comm_status[local_id] != distributed::EMPTY) {
        int ddm_start = ddm->body_shape_start[local_id];
        int dm_start = ddm->body_shapes[ddm_start + shape_idx];  // index in data_manager of the desired shape
        int sphere_start = shape_data.start_rigid[dm_start];
        shape_data.sphere_rigid[sphere_start] = radius;
    }
}

void ChSystemDistributed::SetTriangleShapes(const std::vector<uint>& gids,
                                            const std::vector<int>& shape_idx,
                                            const std::vector<TriData>& new_shapes) {
    for (size_t i = 0; i < gids.size(); i++) {
        SetTriangleShape(gids[i], shape_idx[i], new_shapes[i]);
    }
}

void ChSystemDistributed::SetTriangleShape(uint gid, int shape_idx, const TriData& new_shape) {
    auto& shape_data = data_manager->cd_data->shape_data;

    int local_id = ddm->GetLocalIndex(gid);
    if (local_id != -1 && ddm->comm_status[local_id] != distributed::EMPTY) {
        int ddm_start = ddm->body_shape_start[local_id];
        int dm_start = ddm->body_shapes[ddm_start + shape_idx];  // index in data_manager of the desired shape
        int triangle_start = shape_data.start_rigid[dm_start];
        shape_data.triangle_rigid[triangle_start] = real3(new_shape.v1.x(), new_shape.v1.y(), new_shape.v1.z());
        shape_data.triangle_rigid[triangle_start + 1] = real3(new_shape.v2.x(), new_shape.v2.y(), new_shape.v2.z());
        shape_data.triangle_rigid[triangle_start + 2] = real3(new_shape.v3.x(), new_shape.v3.y(), new_shape.v3.z());
    }
}

// TODO: only return for bodies with nonzero contact force
// std::vector<std::pair<uint, ChVector<>>> ChSystemDistributed::GetBodyContactForces(
//     const std::vector<uint>& gids) const {
//     // Gather forces on specified bodies
//     std::vector<internal_force> send;
//     for (uint i = 0; i < gids.size(); i++) {
//         uint gid = gids[i];
//         int local = ddm->GetLocalIndex(gid);
//         if (local != -1 &&
//             (ddm->comm_status[local] == distributed::OWNED || ddm->comm_status[local] == distributed::SHARED_UP ||
//              ddm->comm_status[local] == distributed::SHARED_DOWN)) {
//             // Get force on body at index local
//             int contact_index = data_manager->host_data.ct_body_map[local];
//             if (contact_index != -1) {
//                 real3 f = data_manager->host_data.ct_body_force[contact_index];
//                 internal_force cf = {gid, {f[0], f[1], f[2]}};
//                 send.push_back(std::move(cf));
//             }
//         }
//     }
//
//     // Master rank receives all messages sent to it and appends the values to its gid vector
//     MPI_Request r_bar;
//     MPI_Status s_bar;
//     internal_force* buffer = new internal_force[gids.size()];  // Beginning of buffer
//     internal_force* buf = buffer;                              // Moving pointer into buffer
//     int num_gids = 0;
//     if (my_rank == master_rank) {
//         // Write own values
//         if (send.size() > 0) {
//             std::memcpy(buf, send.data(), sizeof(internal_force) * send.size());
//             buf += send.size();
//             num_gids += send.size();
//         }
//         MPI_Ibarrier(world, &r_bar);
//         MPI_Status s_prob;
//         int message_waiting = 0;
//         int r_bar_flag = 0;
//
//         MPI_Test(&r_bar, &r_bar_flag, &s_bar);
//         while (!r_bar_flag) {
//             MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, world, &message_waiting, &s_prob);
//             if (message_waiting) {
//                 int count;
//                 MPI_Get_count(&s_prob, InternalForceType, &count);
//                 std::cout << "COUNT A: " << count << "\n";
//                 MPI_Request r_recv;
//                 MPI_Irecv(buf, count, InternalForceType, s_prob.MPI_SOURCE, MPI_ANY_TAG, world, &r_recv);
//                 buf += count;
//                 num_gids += count;
//             }
//             MPI_Test(&r_bar, &r_bar_flag, &s_bar);
//         }
//     } else {
//         // All other ranks send their elements, if they have any
//         if (send.size() > 0) {
//             MPI_Request r_send;
//             // Non-blocking synchronous Send
//             MPI_Issend(send.data(), send.size(), InternalForceType, master_rank, 0, world, &r_send);
//
//             MPI_Status s_send;
//             MPI_Wait(&r_send, &s_send);
//         }
//         // Reaching here indicates to the comm that this rank's message has been recved by rank 0
//         MPI_Ibarrier(world, &r_bar);
//     }
//
//     MPI_Status stat;
//     MPI_Wait(&r_bar, &stat);  // Wait for completion of all sending
//
//     // At this point, buf holds all forces on master_rank. All other ranks have num_gids=0.
//     std::vector<std::pair<uint, ChVector<>>> forces;
//     for (int i = 0; i < num_gids; i++) {
//         ChVector<> frc(buffer[i].force[0], buffer[i].force[1], buffer[i].force[2]);
//         forces.push_back(std::make_pair(buffer[i].gid, frc));
//     }
//
//     delete[] buffer;
//     return forces;
// }

std::vector<std::pair<uint, ChVector<>>> ChSystemDistributed::GetBodyContactForces(
    const std::vector<uint>& gids) const {
    // Gather forces on specified bodies
    std::vector<internal_force> send;
    for (uint i = 0; i < gids.size(); i++) {
        uint gid = gids[i];
        int local = ddm->GetLocalIndex(gid);
        if (local != -1 &&
            (ddm->comm_status[local] == distributed::OWNED || ddm->comm_status[local] == distributed::SHARED_UP ||
             ddm->comm_status[local] == distributed::SHARED_DOWN)) {
            // Get force on body at index local
            int contact_index = data_manager->host_data.ct_body_map[local];
            if (contact_index != -1) {
                real3 f = data_manager->host_data.ct_body_force[contact_index];
                if (!IsZero(f)) {
                    internal_force cf = {gid, {f[0], f[1], f[2]}};
                    send.push_back(std::move(cf));
                }
            }
        }
    }

    int num_send = static_cast<int>(send.size());

    // Send all forces to the master rank
    // Buffer to collect forces on the master rank
    internal_force* buffer = new internal_force[gids.size()];
    int index = 0;  // Working index into buffer
    if (my_rank == master_rank) {
        std::memcpy(buffer, send.data(), sizeof(internal_force) * send.size());
        index += num_send;

        // Recv from all other ranks
        for (int i = 1; i < num_ranks; i++) {
            int count = 0;
            MPI_Status status;
            MPI_Probe(i, 0, world, &status);
            MPI_Get_count(&status, InternalForceType, &count);
            MPI_Recv(buffer + index, count, InternalForceType, i, 0, world, &status);
            index += count;
        }
    } else {
        MPI_Send(send.data(), num_send, InternalForceType, master_rank, 0, world);
    }

    // At this point, buf holds all forces on master_rank. All other ranks have index=0.
    std::vector<std::pair<uint, ChVector<>>> forces;
    for (int i = 0; i < index; i++) {
        ChVector<> frc(buffer[i].force[0], buffer[i].force[1], buffer[i].force[2]);
        forces.push_back(std::make_pair(buffer[i].gid, frc));
    }

    delete[] buffer;
    return forces;
}

// NOTE: This function implies that real is double
real3 ChSystemDistributed::GetBodyContactForce(uint gid) const {
    real3 force(0);

    // Check if specified body is owned by this rank and get force
    int local = ddm->GetLocalIndex(gid);
    bool found = local != -1 &&
                 (ddm->comm_status[local] == distributed::OWNED || ddm->comm_status[local] == distributed::SHARED_UP ||
                  ddm->comm_status[local] == distributed::SHARED_DOWN);
    if (found) {
        // Get force on body at index local
        int contact_index = data_manager->host_data.ct_body_map[local];
        if (contact_index != -1) {
            force = data_manager->host_data.ct_body_force[contact_index];
        }
    }

    // Master rank receives from owning rank
    MPI_Request r_bar;
    MPI_Status s_bar;
    if (my_rank == master_rank) {
        MPI_Ibarrier(world, &r_bar);
        MPI_Status s_prob;
        int message_waiting = 0;
        int r_bar_flag = 0;

        // Recving loop
        MPI_Test(&r_bar, &r_bar_flag, &s_bar);
        while (!r_bar_flag) {
            MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, world, &message_waiting, &s_prob);
            if (message_waiting) {
                MPI_Request r_recv;
                MPI_Irecv(&force, 3, MPI_DOUBLE, s_prob.MPI_SOURCE, MPI_ANY_TAG, world, &r_recv);
            }
            MPI_Test(&r_bar, &r_bar_flag, &s_bar);
        }
    } else {
        // All other ranks send their elements, if they have any
        if (found) {
            MPI_Request r_send;
            // Non-blocking synchronous Send
            MPI_Issend(&force, 3, MPI_DOUBLE, master_rank, 0, world, &r_send);

            MPI_Status s_send;
            MPI_Wait(&r_send, &s_send);
        }
        // Reaching here indicates to the comm that this rank's message has been recved by rank 0
        MPI_Ibarrier(world, &r_bar);
    }

    MPI_Status stat;
    MPI_Wait(&r_bar, &stat);  // Wait for completion of all sending

    return force;
}
