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

#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/comm/ChCommDistr.h"
//#include "chrono_distributed/collision/ChCollisionSystemDistr.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/ChSettings.h"
#include "chrono_parallel/collision/ChCollisionModelParallel.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/other_types.h"

namespace chrono {

class ChDomainDistr;
class ChCommDistr;

class ChSystemDistr : public ChSystemParallel {

public:
	ChSystemDistr(MPI_Comm world, double ghost_layer, int max_objects);
	virtual ~ChSystemDistr();

	int GetRanks() {return num_ranks;}
	int GetMyRank() {return my_rank;}

	std::shared_ptr<ChDomainDistr> GetDomain() {return domain;}
	
	ChParallelDataManager* GetDataManager() {return data_manager;}

	// Set the distance from the subdomain within which a body will be kept as a ghost
	void SetGhostLayer(double thickness) { if (ghost_layer > 0) ghost_layer = thickness; }
	double GetGhostLayer() {return ghost_layer;}

	// Reads in body data from a file for start up.
	// Format:
	// TODO:
	void ReadBodies(std::string filename);

	virtual void AddBody(std::shared_ptr<ChBody> newbody); // TODO: Needs to overwrite Chrono::Parallel
	void RemoveBody(int global_id);//TODO

	int Integrate_Y(); //TODO
	void SetDomainImpl(std::shared_ptr<ChDomainDistr> dom);
	void ErrorAbort(std::string msg);

	// A running count of the number of global bodies for
	// identification purposes
	int num_bodies_global;

	double ghost_layer;

	// MPI
	int num_ranks;
	int my_rank;

	// TODO
	ChParallelDataManager *data_manager;


	// World of MPI ranks for the simulation
	MPI_Comm world;

	// Class for domain decomposition
	std::shared_ptr<ChDomainDistr> domain;

	// Class for MPI communication
	std::shared_ptr<ChCommDistr> comm;

	// Class for collision detection
	collision::ChCollisionSystem *collision_system;

	// A body whose center is this far from the subdomain will be kept as a ghost.

};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_ */
