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

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"

#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/comm/ChCommDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"
#include "chrono_distributed/cosimulation/ChCosimulationDistr.h"
#include "chrono_distributed/collision/ChCollisionSystemDistr.h"
#include "chrono_distributed/collision/ChDataManagerDistr.h"

namespace chrono {

class ChDomainDistr;
class ChCommDistr;
class ChCosimulationDistr;
class ChDataManagerDistr;
class ChCollisionSystemDistr;


class ChSystemDistr : public ChSystem {

public:
	ChSystemDistr(MPI_Comm world, unsigned int max_local = 16000, unsigned int max_ghost = 10000);
	virtual ~ChSystemDistr();

	int GetRanks() {return num_ranks;}
	int GetMyRank() {return my_rank;}

	// Get counts of bodies
	int GetNumLocal() {return num_local;}
	int GetNumShared() {return num_shared;}
	int GetNumGhost() {return num_ghost;}
	int GetNumTotal() {return num_total;}
	int GetMaxLocal() {return max_local;}
	int GetMaxShared() {return max_shared;}
	int GetMaxGhost() {return max_ghost;}

	ChDomainDistr* GetDomain() {return domain;}
	ChCollisionSystemDistr* GetCollisionSystem() {return collision_system;}

	// Set how often MPI ranks communicate exchanges
	void SetExchangeFreq(int exchange_every) { if (exchange_every > 0) this->exchange_every = exchange_every; }

	// Set the distance from the subdomain within which a body will be kept as a ghost
	void SetGhostLayer(double thickness) { if (ghost_layer > 0) ghost_layer = thickness; }
	double GetGhostLayer() {return ghost_layer;}

	// Reads in body data from a file for start up.
	// Format:
	// TODO:
	void ReadBodies(std::string filename);

	void AddBody(std::shared_ptr<ChBody> newbody);
	void RemoveBody(int global_id);

	void SetDomainImpl(ChDomainDistr *dom);
	void ErrorAbort(std::string msg);

protected:
	// MPI
	int num_ranks;
	int my_rank;

	// Number of local bodies.
	int num_local;

	// Number of shared bodies.
	// ie bodies that are integrated on this rank, but exist as
	// a ghost on another rank.
	int num_shared;

	// Number of ghost bodies.
	int num_ghost;

	// Number of total bodies across the global domain.
	int num_total;

	int max_local;
	int max_shared;
	int max_ghost;

	// World of MPI ranks for the simulation
	MPI_Comm world;

	// Class for domain decomposition
	ChDomainDistr *domain;

	// Class for MPI communication
	ChCommDistr *comm;

	// Class for sending and receiving cosimulation data
	ChCosimulationDistr *cosim;

	// Class for collision detection
	ChCollisionSystemDistr *collision_system;

	// TODO
	ChDataManagerDistr *data;

	// Number of timesteps between exchanges (1 ==> exchange every timestep)
	int exchange_every;

	// A body whose center is this far from the subdomain will be kept as a ghost.
	double ghost_layer;
};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_ */
