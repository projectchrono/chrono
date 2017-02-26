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
#include "chrono_distributed/ChDataManagerDistr.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

namespace chrono {

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
	
	ChDataManagerDistr* GetDataManager() {return data_manager;}

	// Set the distance from the subdomain within which a body will be kept as a ghost
	void SetGhostLayer(double thickness) { if (ghost_layer > 0) ghost_layer = thickness; }
	double GetGhostLayer() {return ghost_layer;}

	void AddBody(std::shared_ptr<ChBody> newbody) override;

	int DoStepDynamics(double m_step); // TODO yeah...
    virtual bool Integrate_Y() override;
	void SetDomainImpl(ChDomainDistr *dom);
	void ErrorAbort(std::string msg);
	void PrintBodyStatus();

	// A running count of the number of global bodies for
	// identification purposes
	unsigned int num_bodies_global;

	double ghost_layer;

	// MPI
	int num_ranks;
	int my_rank;

	// TODO
	ChDataManagerDistr* data_manager;

	// World of MPI ranks for the simulation
	MPI_Comm world;

	// Class for domain decomposition
	ChDomainDistr *domain;

	// Class for MPI communication
	ChCommDistr *comm;

	// A body whose center is this far from the subdomain will be kept as a ghost.

};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_ */
