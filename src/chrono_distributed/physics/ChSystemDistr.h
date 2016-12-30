/*
 * ChSystemDistr.h
 *
 *  Created on: Dec 28, 2016
 *      Author: nic
 */

#ifndef CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_
#define CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_

#include <mpi.h>
#include <string>

#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/comm/ChCommDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"

namespace chrono {

class ChSystemDistr {
public:
	ChSystemDistr(MPI_Comm world, unsigned int max_local = 16000, unsigned int max_ghost = 10000);
	virtual ~ChSystemDistr();

	int GetRanks() {return num_ranks;}
	int GetMyRank() {return my_rank;}

	void SetDt(double dt) { if (dt > 0) this->dt = dt; }
	double GetDt() {return dt;}
	double GetCurrTimestep() {return num_timestep;}
	double GetCurrSimTime() {return sim_time;}
	int GetNumLocal() {return num_local;}
	int GetNumShared() {return num_shared;}
	int GetNumGhost() {return num_ghost;}
	int GetNumTotal() {return num_total;}
	void SetExchangeFreq(int exchange_every) { if (exchange_every > 0) this->exchange_every = exchange_every; }

	void SetGhostLayer(double thickness) { if (ghost_layer > 0) ghost_layer = thickness; }
	double GetGhostLayer() {return ghost_layer;}
	void SplitDomain() { domain->SplitDomain(); }
	void SetDomain(double xlo, double xhi, double ylo, double yhi, double zlo, double zhi) { domain->SetDomain(xlo,xhi,ylo,yhi,zlo,zhi); }

	// Reads in body data from a file for start up.
	// Format:
	// TODO:
	void ReadBodies(std::string filename);

	int AddBody(ChBody *body);
	int RemoveBody();//TODO?

	// Advances the simulation by the specified number of steps
	void DoTimesteps(int timesteps);
	void DoTimestep();

	MPI_Comm world;
	ChDomainDistr * domain;
	ChCommDistr * comm;

protected:
	// MPI
	int num_ranks;
	int my_rank;

	int max_local;
	int max_shared;
	int max_ghost;

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

	// Lists of local and ghost bodies
	ChBodyDistr **local_bodylist;
	ChBodyDistr **shared_bodylist;
	ChBodyDistr **ghost_bodylist;

	// Length of a timestep
	double dt;

	// Total elapsed simulation time
	double sim_time;

	// Number of the current timestep
	int num_timestep;

	// Number of timesteps between exchanges (1 ==> exchange every timestep)
	int exchange_every;

	// A body whose center is this far from the subdomain will be kept as a ghost.
	double ghost_layer;

};

} /* namespace chrono */

#endif /* CHRONO_DISTRIBUTED_PHYSICS_CHSYSTEMDISTR_H_ */
