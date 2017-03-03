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

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/ChSystemParallel.h"

namespace chrono {


enum COMM_STATUS {
	EMPTY = 0,
	OWNED,
	GHOST_UP,
	GHOST_DOWN,
	SHARED_UP,
	SHARED_DOWN,
	UNOWNED_UP,
	UNOWNED_DOWN
};

enum MESSAGE_TYPE {
	EXCHANGE,
	UPDATE,
	TRANSFER
};


class ChDomainDistr;
class ChCommDistr;
class ChDataManagerDistr;

class CH_DISTR_API ChSystemDistr : public ChSystemParallelDEM {

public:
	ChSystemDistr(MPI_Comm world, double ghost_layer, unsigned int max_objects);
	virtual ~ChSystemDistr();

	int GetRanks() {return num_ranks;}
	int GetMyRank() {return my_rank;}

	ChDomainDistr* GetDomain() {return domain;}
	
	// Set the distance from the subdomain within which a body will be kept as a ghost
	void SetGhostLayer(double thickness) { if (ghost_layer > 0) ghost_layer = thickness; }
	double GetGhostLayer() {return ghost_layer;}

	void AddBody(std::shared_ptr<ChBody> newbody) override;

	virtual bool Integrate_Y() override;
    virtual void UpdateRigidBodies() override;

	void ErrorAbort(std::string msg);
	void PrintBodyStatus();

	// A running count of the number of global bodies for
	// identification purposes
	unsigned int num_bodies_global;

	double ghost_layer;

	// MPI
	int num_ranks;
	int my_rank;

	// World of MPI ranks for the simulation
	MPI_Comm world;

	// Class for domain decomposition
	ChDomainDistr *domain;

	// Class for MPI communication
	ChCommDistr *comm;

	ChDistributedDataManager *ddm;
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_ */
