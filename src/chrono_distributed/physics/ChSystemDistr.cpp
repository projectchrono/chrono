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
#include <numeric>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/collision/ChCCollisionSystem.h"

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/physics/ChDomainDistrLong.h"
#include "chrono_distributed/collision/ChDataManagerDistr.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/other_types.h"

namespace chrono {

ChSystemDistr::ChSystemDistr(MPI_Comm world, double ghost_layer, unsigned int max_objects) :
		ChSystemParallelDEM(max_objects)
{
	this->world = world;
	MPI_Comm_size(world, &num_ranks);
	MPI_Comm_rank(world, &my_rank);

	data_manager = new ChDataManagerDistr(this); // TODO Change?
	domain = 0;
	comm = std::make_shared<ChCommDistr>((std::shared_ptr<ChSystemDistr>) this);
	collision_system = new collision::ChCollisionSystemParallel(data_manager); //TODO change?

	this->ghost_layer = ghost_layer;
	this->num_bodies_global = 0;
	
	// collision_system_type = COLLSYS_PARALLEL; // Change to distributed???
}

//TODO *******
bool ChSystemDistr::Integrate_Y() {
	this->ChSystemParallel::Integrate_Y();

	//TODO: comm

    return 1;
}


ChSystemDistr::~ChSystemDistr()
{}


void ChSystemDistr::SetDomainImpl(std::shared_ptr<ChDomainDistr> dom)
{
	if (domain) ErrorAbort("Domain implementation cannot be set more than once.");
	domain = dom;
}


//TODO Change
void ChSystemDistr::AddBody(std::shared_ptr<ChBody> newbody)
{
	newbody->SetIdentifier(num_bodies_global);
	num_bodies_global++;

	// If local
	if (domain->InSub(newbody))
	{
		newbody->SetId(0); // Set as local
	}

	// If ghost
	else if (domain->InGhost(newbody))
	{
		newbody->SetId(1); // Set as ghost
	}

	newbody->SetSystem(this);
	bodylist.push_back(newbody);
	data_manager->num_rigid_bodies++;

	// actual data is set in UpdateBodies().
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
	data_manager->host_data.collide_rigid.push_back(true);

	// Let derived classes reserve space for specific material surface data
	// AddMaterialSurfaceData(newbody);
}


// Used to end the program on an error and print a message.
void ChSystemDistr::ErrorAbort(std::string msg)
{
	if (my_rank == 0) GetLog() << msg << '\n';
	MPI_Abort(world, MPI_ERR_OTHER);
}
} /* namespace chrono */
