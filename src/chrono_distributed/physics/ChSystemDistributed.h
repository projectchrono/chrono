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
#include <string>
#include <memory>

#include "chrono/physics/ChBody.h"

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/comm/ChCommDistributed.h"
#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/other_types.h"

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "ChDomainDistributed.h"

namespace chrono {

class ChDomainDistributed;
class ChCommDistributed;
class ChDataManagerDistr;

class CH_DISTR_API ChSystemDistributed : public ChSystemParallelDEM {

	friend class ChCommDistributed;

public:
	ChSystemDistributed(MPI_Comm world, double ghost_layer, unsigned int max_objects);
	virtual ~ChSystemDistributed();

	int GetNumRanks() {return num_ranks;}
	int GetMyRank() {return my_rank;}

	double GetGhostLayer() {return ghost_layer;}

	/// A running count of the number of global bodies for
	/// identification purposes
	int GetNumBodiesGlobal() {return num_bodies_global;}


	void AddBody(std::shared_ptr<ChBody> newbody) override;
	virtual bool Integrate_Y() override;
    virtual void UpdateRigidBodies() override;

	ChDomainDistributed* GetDomain() {return domain;}
	ChCommDistributed* GetComm() {return comm;}
	void ErrorAbort(std::string msg);
	void PrintBodyStatus();
	void WriteCSV(int fileCounter);

	MPI_Comm GetMPIWorld() {return world;}

	ChDistributedDataManager *ddm;

protected:
	// MPI
	int num_ranks;
	int my_rank;

	double ghost_layer;

	unsigned int num_bodies_global;

	// World of MPI ranks for the simulation
	MPI_Comm world;

	// Class for domain decomposition
	ChDomainDistributed *domain;

	// Class for MPI communication
	ChCommDistributed *comm;

    void AddBodyExchange(std::shared_ptr<ChBody> newbody, distributed::COMM_STATUS status);
};

} /* namespace chrono */
