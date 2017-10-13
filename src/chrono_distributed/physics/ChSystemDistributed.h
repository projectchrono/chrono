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
    ChSystemDistributed(MPI_Comm world, double ghost_layer, unsigned int max_objects, std::string debug_file);
    virtual ~ChSystemDistributed();

    int GetNumRanks() { return num_ranks; }
    int GetMyRank() { return my_rank; }

    double GetGhostLayer() { return ghost_layer; }

    /// A running count of the number of global bodies for
    /// identification purposes
    int GetNumBodiesGlobal() { return num_bodies_global; }

    void AddBody(std::shared_ptr<ChBody> newbody) override;
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
    /// Writes out the positions and velocities of all bodies in the system
    /// to a csv format file in in filedir. Prepends the MPI rank to
    /// the beginning of the file.
    void WriteCSV(std::string filedir, std::string filename);

    MPI_Comm GetMPIWorld() { return world; }

    ChDistributedDataManager* ddm;

    char* node_name;

#ifdef DistrDebug
    std::ofstream debug_stream;
#endif
    double GetLowestZ(uint* gid);
    void CheckIds();

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
};

} /* namespace chrono */
