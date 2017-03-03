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

#include "chrono/physics/ChBody.h"
#include "chrono/collision/ChCCollisionSystem.h"

#include "chrono_distributed/physics/ChSystemDistr.h"
#include "chrono_distributed/physics/ChDomainDistr.h"
#include "chrono_distributed/ChDistributedDataManager.h"

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/other_types.h"

using namespace chrono;

ChSystemDistr::ChSystemDistr(MPI_Comm world, double ghost_layer, unsigned int max_objects)
{
	this->world = world;
	MPI_Comm_size(world, &num_ranks);
	MPI_Comm_rank(world, &my_rank);


	//my_rank = 1;
	//num_ranks = 2;



	domain = new ChDomainDistr(this);
	comm = new ChCommDistr(this);

	this->ghost_layer = ghost_layer;
	this->num_bodies_global = 0;
}

//TODO *******
bool ChSystemDistr::Integrate_Y()
{
	bool ret = ChSystemParallelDEM::Integrate_Y();

	comm->Exchange();
	return ret;
}

void ChSystemDistr::UpdateRigidBodies()
{
	this->ChSystemParallel::UpdateRigidBodies();

#pragma omp parallel for
    for (int i = 0; i < bodylist.size(); i++)
    {
    	ddm->global_id[i] = bodylist[i]->GetGid();
    }
}


ChSystemDistr::~ChSystemDistr()
{
	delete domain;
	delete comm;
}


// TODO: BUG: inconsistent view between ranks ********************************************
void ChSystemDistr::AddBody(std::shared_ptr<ChBody> newbody)
{
	// Regardless of whether the body is on this rank,
	// increment the global id counter to maintain unique
	// global ids.
	newbody->SetGid(num_bodies_global);
	num_bodies_global++;

	int status = domain->GetCommStatus(newbody);
	if (status == chrono::UNOWNED_UP || status == chrono::UNOWNED_DOWN)
	{
		GetLog() << "Not adding GID: " << newbody->GetGid() << " on Rank: " << my_rank << "\n";
		return;
	}

	ddm->comm_status.push_back(status);
	ddm->global_id.push_back(num_bodies_global - 1);

	// DEBUGGING -----------------------------------
	switch(status)
	{

	// Shared up
	case chrono::SHARED_UP:
		GetLog() << "Adding shared up";
		break;

	// Shared down
	case chrono::SHARED_DOWN:
		GetLog() << "Adding shared down";
		break;

	// Owned
	case chrono::OWNED:
		GetLog() << "Adding owned";
		break;

	// Ghost up
	case chrono::GHOST_UP:
		GetLog() << "Adding ghost";
		break;

	// Ghost down
	case chrono::GHOST_DOWN:
		GetLog() << "Adding ghost";
		break;

	// Not involved with this rank
	default:
		break;
	}

	GetLog() << " GID: " << newbody->GetGid() << " on Rank: " << my_rank << "\n";
	//-------------------------------------------------



	newbody->SetId(data_manager->num_rigid_bodies);
	bodylist.push_back(newbody);
	data_manager->num_rigid_bodies++;
	newbody->SetSystem(this);

	// actual data is set in UpdateBodies().
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
    data_manager->host_data.collide_rigid.push_back(true);
	// Let derived classes reserve space for specific material surface data
	ChSystemParallelDEM::AddMaterialSurfaceData(newbody);
}


// Used to end the program on an error and print a message.
void ChSystemDistr::ErrorAbort(std::string msg)
{
	if (my_rank == 0) GetLog() << msg << '\n';
	MPI_Abort(world, MPI_ERR_OTHER);
}


void ChSystemDistr::PrintBodyStatus()
{
	GetLog() << "Rank: " << my_rank << "\n";
	GetLog() << "Bodylist:\n";
	std::vector<std::shared_ptr<ChBody>>::iterator bl_itr = bodylist.begin();
	for (; bl_itr != bodylist.end(); bl_itr++)
	{
		ChVector<double> pos = (*bl_itr)->GetPos();
		ChVector<double> vel = (*bl_itr)->GetPos_dt();

		fprintf(stdout, "Global ID: %d Pos: %.2f,%.2f,%.2f\n",
				(*bl_itr)->GetGid(), pos.x(), pos.y(), pos.z());
	}

	GetLog() << "Data Manager:\n";
	for (int i = 0; i < data_manager->num_rigid_bodies; i++)
	{
		int status = ddm->comm_status[i];
		unsigned int gid = ddm->global_id[i];

		if (status != chrono::EMPTY)
		{
			if (status == chrono::SHARED_DOWN)
			{
				GetLog() << "Global ID: " << gid << " Shared down";
			}
			else if (status == chrono::SHARED_UP)
			{
				GetLog() << "Global ID: " << gid << " Shared up";
			}
			else if (status == chrono::GHOST_UP)
			{
				GetLog() << "Global ID: " << gid << " Ghost up";
			}
			else if (status == chrono::GHOST_DOWN)
			{
				GetLog() << "Global ID: " << gid << " Ghost down";
			}
			else if (status == chrono::OWNED)
			{
				GetLog() << "Global ID: " << gid << " Owned";
			}
			else
			{
				GetLog() << "ERROR: Global ID: " << gid << " Undefined comm_status\n";
			}

			real3 pos = data_manager->host_data.pos_rigid[i];
			fprintf(stdout, " Pos: %.2f,%.2f,%.2f\n",pos.x, pos.y, pos.z);
		}
	}
}
