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

ChSystemDistributed::ChSystemDistributed(MPI_Comm world, double ghost_layer, unsigned int max_objects) {
    this->world = world;
    MPI_Comm_size(world, &num_ranks);
    MPI_Comm_rank(world, &my_rank);
    int name_len;
    node_name = new char[50];
    MPI_Get_processor_name(node_name, &name_len);

    rank_digits = std::pow(10, std::floor(std::log10(num_ranks)) + 1);

    ddm = new ChDistributedDataManager(this);
    domain = new ChDomainDistributed(this);
    comm = new ChCommDistributed(this);

    this->ghost_layer = ghost_layer;
    this->num_bodies_global = 0;

    data_manager->system_timer.AddTimer("B1");
    data_manager->system_timer.AddTimer("B2");
    data_manager->system_timer.AddTimer("B3");
    data_manager->system_timer.AddTimer("B4");
    data_manager->system_timer.AddTimer("B5");
    data_manager->system_timer.AddTimer("A");

    data_manager->system_timer.AddTimer("Send");
    data_manager->system_timer.AddTimer("Recv");
    data_manager->system_timer.AddTimer("FirstEmpty");
    data_manager->system_timer.AddTimer("UnpackBody");
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
}

ChSystemDistributed::~ChSystemDistributed() {
    delete domain;
    delete comm;

#ifdef DistrDebug
    debug_stream.close();
#endif
    // delete ddm;
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

// The bodies added this way must be guarenteed to be OWNED, SHARED_UP, or SHARED_DOWN
// on this rank.
// TODO do an update to create GHOST_UP and GHOST_DOWN bodies before first time step - could call an UpdateRigidBodies before first timestep
void ChSystemDistributed::AddBodyTrust(std::shared_ptr<ChBody> newbody) {
    newbody->SetGid(num_bodies_global * rank_digits + my_rank);  // gid = <lgid><rank>
    num_bodies_global++;
    distributed::COMM_STATUS status = domain->GetBodyRegion(newbody);

    // Check for collision with this sub-domain
    if (newbody->GetBodyFixed()) {
        ChVector<double> body_min;
        ChVector<double> body_max;
        ChVector<double> sublo(domain->GetSubLo());
        ChVector<double> subhi(domain->GetSubHi());

        newbody->GetCollisionModel()->GetAABB(body_min, body_max);

#ifdef DistrDebug
        printf("AABB: Min: %.3f %.3f %.3f  Max: %.3f %.3f %.3f\n", body_min.x(), body_min.y(), body_min.z(),
               body_max.x(), body_max.y(), body_max.z());
#endif
        // If the part of the body lies in this sub-domain, add it
        if ((body_min.x() <= subhi.x() && sublo.x() <= body_max.x()) &&
            (body_min.y() <= subhi.y() && sublo.y() <= body_max.y()) &&
            (body_min.z() <= subhi.z() && sublo.z() <= body_max.z())) {
            status = distributed::GLOBAL;
            // TODO cut off the shapes that don't affect this sub-domain?
        }
    }

    // Makes space for shapes
    ddm->body_shape_start.push_back(0);
    ddm->body_shape_count.push_back(0);

    ddm->comm_status.push_back(status);
    ddm->global_id.push_back(newbody->GetGid());

    newbody->SetId(data_manager->num_rigid_bodies);
    bodylist.push_back(newbody);

    ddm->gid_to_localid[newbody->GetGid()] = newbody->GetId();

    data_manager->num_rigid_bodies++;
    newbody->SetSystem(this);  // TODO Syncs collision model // TODO collision add expensive

    // actual data is set in UpdateBodies().
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
    data_manager->host_data.collide_rigid.push_back(true);

    // Let derived classes reserve space for specific material surface data
    ChSystemParallelSMC::AddMaterialSurfaceData(newbody);
}

void ChSystemDistributed::AddBody(std::shared_ptr<ChBody> newbody) {
    // Regardless of whether the body is on this rank,
    // increment the global id counter to maintain unique
    // global ids over all ranks.
    newbody->SetGid(num_bodies_global * rank_digits + my_rank);
    num_bodies_global++;
    distributed::COMM_STATUS status = domain->GetBodyRegion(newbody);  // TODO expensive

    // Check for collision with this sub-domain
    if (newbody->GetBodyFixed()) {
        ChVector<double> body_min;
        ChVector<double> body_max;
        ChVector<double> sublo(domain->GetSubLo());
        ChVector<double> subhi(domain->GetSubHi());

        newbody->GetCollisionModel()->GetAABB(body_min, body_max);

#ifdef DistrDebug
        printf("AABB: Min: %.3f %.3f %.3f  Max: %.3f %.3f %.3f\n", body_min.x(), body_min.y(), body_min.z(),
               body_max.x(), body_max.y(), body_max.z());
#endif
        // If the part of the body lies in this sub-domain, add it
        if ((body_min.x() <= subhi.x() && sublo.x() <= body_max.x()) &&
            (body_min.y() <= subhi.y() && sublo.y() <= body_max.y()) &&
            (body_min.z() <= subhi.z() && sublo.z() <= body_max.z())) {
            status = distributed::GLOBAL;
            // TODO cut off the shapes that don't affect this sub-domain?
        }
    }

    if (status == distributed::UNOWNED_UP || status == distributed::UNOWNED_DOWN) {
#ifdef DistrDebug
        GetLog() << "Not adding GID: " << newbody->GetGid() << " on Rank: " << my_rank << "\n";
#endif
        return;
    }
    // Makes space for shapes TODO Does this work for mid-simulation add by user?
    ddm->body_shape_start.push_back(0);
    ddm->body_shape_count.push_back(0);  // TODO these two lines were moved from above

    ddm->comm_status.push_back(status);
    ddm->global_id.push_back(newbody->GetGid());
#ifdef DistrDebug
    switch (status) {
        // Shared up
        case distributed::SHARED_UP:
            GetLog() << "Adding shared up";
            break;

        // Shared down
        case distributed::SHARED_DOWN:
            GetLog() << "Adding shared down";
            break;

        // Owned
        case distributed::OWNED:
            GetLog() << "Adding owned";
            break;

        // Ghost up
        case distributed::GHOST_UP:
            GetLog() << "Adding ghost up";
            break;

        // Ghost down
        case distributed::GHOST_DOWN:
            GetLog() << "Adding ghost down";
            break;

        // Global
        case distributed::GLOBAL:
            GetLog() << "Adding global";
            break;

        // Not involved with this rank
        default:
            break;
    }

    GetLog() << " GID: " << newbody->GetGid() << " on Rank: " << my_rank << "\n";
#endif

    newbody->SetId(data_manager->num_rigid_bodies);
    bodylist.push_back(newbody);

    ddm->gid_to_localid[newbody->GetGid()] = newbody->GetId();

    data_manager->num_rigid_bodies++;
    newbody->SetSystem(this);  // TODO Syncs collision model // TODO collision add expensive

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

    newbody->SetSystem(this);  // Calls collisionsystem::add? TODO

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
    bodylist[index]->SetCollide(false);  // Note: this calls collisionsystem::remove
    if (index < ddm->first_empty)
        ddm->first_empty = index;
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
    ddm->gid_to_localid.erase(body->GetGid());
}

// Used to end the program on an error and print a message.
void ChSystemDistributed::ErrorAbort(std::string msg) {
    if (my_rank == 0)
        GetLog() << msg << '\n';
    std::abort();
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
#ifdef DistrProfile
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
#endif

double ChSystemDistributed::GetLowestZ(uint* local_id) {
    double min = 0;
    for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
        if (ddm->comm_status[i] != distributed::EMPTY && data_manager->host_data.pos_rigid[i][2] < min) {
            min = data_manager->host_data.pos_rigid[i][2];
            *local_id = i;
            GetLog() << "SubZero: " << i << " Rank: "
                     << "\n";
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
