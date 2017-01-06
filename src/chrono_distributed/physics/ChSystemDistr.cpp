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

#include <mpi.h>
#include <string>
#include <memory>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/physics/ChBodyDistr.h"
#include "chrono_distributed/collision/ChCollisionSystemDistr.h"
#include "chrono_distributed/collision/ChDataManagerDistr.h"

namespace chrono {

ChSystemDistr::ChSystemDistr(MPI_Comm world, unsigned int max_local, unsigned int max_ghost) :
		ChSystem()
{
	this->world = world;

	MPI_Comm_size(world, &num_ranks);
	MPI_Comm_rank(world, &my_rank);

	domain = NULL;
	comm = new ChCommDistr(this);
	cosim = new ChCosimulationDistr(this);
	collision_system = new ChCollisionSystemDistr(this);
	data = new ChDataManagerDistr(this);

	this->max_local = max_local;
	this->max_ghost = max_ghost;
	max_shared = max_ghost;

	num_local = 0;
	num_shared = 0;
	num_ghost = 0;
	num_total = 0;

	exchange_every = 10;

	ghost_layer = 0.1;
}

ChSystemDistr::~ChSystemDistr()
{
	delete comm;
	delete cosim;
	delete data;
}

// Read in body data at the beginning of a simulation.
// Format:
// TODO:
void ChSystemDistr::ReadBodies(std::string filename)
{

}

void ChSystemDistr::SetDomainImpl(ChDomainDistr *dom)
{
	if (domain) ErrorAbort("Domain implementation cannot be set more than once.");
	domain = dom;
}

void ChSystemDistr::AddBody(std::shared_ptr<ChBody> newbody)
{
}

void ChSystemDistr::RemoveBody(int global_id)
{
}

// Used to end the program on an error and print a message.
void ChSystemDistr::ErrorAbort(std::string msg)
{
	if (my_rank == 0) GetLog() << msg << '\n';
	MPI_Abort(world, MPI_ERR_OTHER);
}


} /* namespace chrono */
