/*
 * ChSystemDistr.cpp
 *
 *  Created on: Dec 28, 2016
 *      Author: nic
 */

#include "ChSystemDistr.h"
#include <mpi.h>

namespace chrono {
namespace chrono_distributed {

ChSystemDistr::ChSystemDistr(MPI_Comm world, unsigned int max_local = 16000, unsigned int max_ghost = 10000) {

	this->world = world;

	MPI_Comm_size(world, &num_ranks);
	MPI_Comm_rank(world, &my_rank);

	domain = new ChDomainDistr(this);
	comm = new ChCommDistr(this);

	this->max_local = max_local;
	this->max_ghost = max_ghost;
	max_shared = max_ghost;

	num_local = 0;
	num_shared = 0;
	num_ghost = 0;
	num_total = 0;

	// The bodies lists are initialized to null
	local_bodylist = new ChBody* [max_local]();
	shared_bodylist = new ChBody* [max_shared]();
	ghost_bodylist = new ChBody* [max_ghost]();

	dt = 0.01;
	num_timestep = 0;
	sim_time = 0;

	exchange_every = 10;

	ghost_layer = 0.1;
}

ChSystemDistr::~ChSystemDistr() {}

// Read in body data at the beginning of a simulation.
// Format:
// TODO:
void ChSystemDistr::ReadBodies(std::string filename)
{

}

// Advances the simulation by the specified number of steps
void ChSystemDistr::DoTimesteps(int timesteps)
{
	for (int i = 0; i < timesteps; i++)
	{
		DoTimestep();
	}

	MPI_Barrier(world);
}

void ChSystemDistr::DoTimestep()
{
	// Todo: collision/force calculation
	// Send ghost forces
	// recv shared forces
	// integrate local
	// integrate shared


	// Exchange
	if (num_timestep % exchange_every == 0)
	{
		MPI_Barrier(world);
		comm->Exchange();
	}
}

}
} /* namespace chrono */
