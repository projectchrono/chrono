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

#ifndef CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_
#define CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_

#include <mpi.h>
#include <string>
#include <memory>

#include "chrono/physics/ChBody.h"

#include "chrono_distributed/ChApiDistributed.h"
#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/comm/ChCommDistr.h"
#include "chrono_distributed/ChDistributedDataManager.h"
#include "chrono_distributed/other_types.h"

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

namespace chrono {

class ChDomainDistr;
class ChCommDistr;
class ChDataManagerDistr;

class CH_DISTR_API ChSystemDistr : public ChSystemParallelDEM {

	friend class ChCommDistr;

public:
	ChSystemDistr(MPI_Comm world, double ghost_layer, unsigned int max_objects);
	virtual ~ChSystemDistr();

	int GetNumRanks() {return num_ranks;}
	int GetMyRank() {return my_rank;}

	double GetGhostLayer() {return ghost_layer;}

	// A running count of the number of global bodies for
	// identification purposes
	int GetNumBodiesGlobal() {return num_bodies_global;}


	void AddBody(std::shared_ptr<ChBody> newbody) override;
	virtual bool Integrate_Y() override;
    virtual void UpdateRigidBodies() override;

	ChDomainDistr* GetDomain() {return domain;}
	ChCommDistr* GetComm() {return comm;}
	void ErrorAbort(std::string msg);
	void PrintBodyStatus();

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
	ChDomainDistr *domain;

	// Class for MPI communication
	ChCommDistr *comm;

    void AddBodyExchange(std::shared_ptr<ChBody> newbody, distributed::COMM_STATUS status);
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_ */
