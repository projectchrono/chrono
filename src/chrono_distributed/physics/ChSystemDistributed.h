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

#pragma once

#include <mpi.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "chrono/physics/ChBody.h"

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/comm/ChCommDistributed.h"
#include "chrono_distributed/other_types.h"
#include "chrono_distributed/physics/ChDomainDistributed.h"

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

namespace chrono {
typedef struct CosimForce {
    uint gid;
    int owner_rank;
    double force[3];
} CosimForce;

typedef struct CosimDispl {
    double vertices[9];
	uint gid;
} CosimDispl;

class ChDomainDistributed;
class ChCommDistributed;
class ChDataManagerDistr;

/// This is the main user interface for Chrono::Distributed
/// Add bodies and set all settings through the systems
/// The simulation runs on all ranks given in the world parameter
class CH_DISTR_API ChSystemDistributed : public ChSystemParallelSMC {
    friend class ChCommDistributed;

  public:
    /// world specifies on which MPI_Comm the simulation should run.
    ChSystemDistributed(MPI_Comm world, double ghost_layer, unsigned int max_objects);
    virtual ~ChSystemDistributed();

    int GetNumRanks() { return num_ranks; }
    int GetMyRank() { return my_rank; }

    double GetGhostLayer() { return ghost_layer; }

    /// A running count of the number of global bodies for
    /// identification purposes
    int GetNumBodiesGlobal() { return num_bodies_global; }
    int IncrementGID() { return num_bodies_global++; }
    bool InSub(ChVector<double> pos);

    void AddBody(std::shared_ptr<ChBody> newbody) override;
    void AddBodyTrust(std::shared_ptr<ChBody> newbody);
    void RemoveBody(std::shared_ptr<ChBody> body) override;
    virtual bool Integrate_Y() override;
    virtual void UpdateRigidBodies() override;

    void RemoveBodyExchange(int index);
    ChDomainDistributed* GetDomain() { return domain; }
    ChCommDistributed* GetComm() { return comm; }

    /// Prints msg to the user and ends execution with an MPI error
    void ErrorAbort(std::string msg);

    void PrintBodyStatus();
    void PrintShapeData();
#ifdef DistrProfile
    void PrintEfficiency();
#endif
    MPI_Comm GetMPIWorld() { return world; }

    ChDistributedDataManager* ddm;

    char node_name[50];

    double GetLowestZ(uint* gid);
    void CheckIds();

    // Co-Simulation
    /// User functions for setting the range of GIDs that correspond to co-simulation triangles
    void SetFirstCosimGID(uint gid);
    void SetLastCosimGID(uint gid);

    /// Tells the simulation to send all forces on co-simulation bodies to the master rank
    /// Output: GIDS <- array of global IDS reporting force; forces <- array of corresponding forces
    /// Call on all ranks
    void CollectCosimForces(uint* GIDs, real3* forces);

    /// Updates the positions of all cosimulation bodies in the system
    /// Call on all ranks?TODO
    void DistributedCosimPositions(uint* GID, ChVector<double>* vertices);

  protected:
    // MPI
    int num_ranks;
    int my_rank;
	
    double ghost_layer;

    unsigned int num_bodies_global;

    // World of MPI ranks for the simulation
    MPI_Comm world;

    // Class for domain decomposition
    ChDomainDistributed* domain;

    // Class for MPI communication
    ChCommDistributed* comm;

    void AddBodyExchange(std::shared_ptr<ChBody> newbody, distributed::COMM_STATUS status);

    // Co-simulation
	MPI_Datatype CosimForceType;
	MPI_Datatype CosimDisplType;
};

} /* namespace chrono */
