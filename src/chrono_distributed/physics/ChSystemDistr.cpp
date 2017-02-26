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
#include "chrono_distributed/ChDataManagerDistr.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/other_types.h"

using namespace chrono;

ChSystemDistr::ChSystemDistr(MPI_Comm world, double ghost_layer, unsigned int max_objects)
{
	// ChSystem
	new(this) ChSystem();

	// ChSystemParallel
	data_manager = new ChDataManagerDistr(this); // Altered

	descriptor = std::make_shared<ChSystemDescriptorParallel>(data_manager);
	contact_container = std::make_shared<ChContactContainerParallel>(data_manager);
	collision_system = std::make_shared<collision::ChCollisionSystemParallel>(data_manager);

	collision_system_type = CollisionSystemType::COLLSYS_PARALLEL;
	counter = 0;
	timer_accumulator.resize(10, 0);
	cd_accumulator.resize(10, 0);
	frame_threads = 0;
	frame_bins = 0;
	old_timer = 0;
	old_timer_cd = 0;
	detect_optimal_threads = false;
	detect_optimal_bins = false;
	current_threads = 2;

	data_manager->system_timer.AddTimer("step");
	data_manager->system_timer.AddTimer("update");
	data_manager->system_timer.AddTimer("collision");
	data_manager->system_timer.AddTimer("collision_broad");
	data_manager->system_timer.AddTimer("collision_narrow");
	data_manager->system_timer.AddTimer("solver");

	data_manager->system_timer.AddTimer("ChIterativeSolverParallel_Solve");
	data_manager->system_timer.AddTimer("ChIterativeSolverParallel_Setup");
	data_manager->system_timer.AddTimer("ChIterativeSolverParallel_Stab");
	data_manager->system_timer.AddTimer("ChIterativeSolverParallel_M");

#ifdef LOGGINGENABLED
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToFile, "false");
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%datetime{%h:%m:%s:%g} %msg");
#endif

	// ChSystemParallelDEM
	solver_speed = std::make_shared<ChIterativeSolverParallelDEM>(data_manager);

	data_manager->settings.collision.collision_envelope = 0;

	// Set this so that the CD can check what type of system it is (needed for narrowphase)
	data_manager->settings.system_type = SystemType::SYSTEM_DEM;

	data_manager->system_timer.AddTimer("ChIterativeSolverParallelDEM_ProcessContact");

	// ChSystemDistr
	this->world = world;
	MPI_Comm_size(world, &num_ranks);
	MPI_Comm_rank(world, &my_rank);

	domain = new ChDomainDistr(this);
	comm = new ChCommDistr(this);

	this->ghost_layer = ghost_layer;
	this->num_bodies_global = 0;

}


int ChSystemDistr::DoStepDynamics(double m_step) {
    step = m_step;
    return ChSystemDistr::Integrate_Y();
}

//TODO *******
bool ChSystemDistr::Integrate_Y() {
	GetLog() << "Integrate_Y\n";
	bool ret = ChSystemParallelDEM::Integrate_Y();


	comm->Exchange();
	return ret;
}


ChSystemDistr::~ChSystemDistr()
{
	delete data_manager;
	delete domain;
	delete comm;
}


void ChSystemDistr::SetDomainImpl(ChDomainDistr *dom)
{
	if (domain) ErrorAbort("Domain implementation cannot be set more than once.");
	domain = dom;
}


//TODO Change
void ChSystemDistr::AddBody(std::shared_ptr<ChBody> newbody)
{
	// Regardless of whether the body is on this rank,
	// increment the global id counter to maintain unique
	// global ids.
	newbody->SetGid(num_bodies_global);
	num_bodies_global++;

	switch(domain->GetCommStatus(newbody))
	{

	// Not involved with this rank
	case 0:
		GetLog() << "Adding NO\n";
		return;
		break;

	// Shared up
	case 1:
		GetLog() << "Adding shared up\n";
		newbody->SetIdentifier(1); // Set as shared up
		data_manager->comm_status.push_back(communication::SHARED_UP);
		data_manager->global_id.push_back(num_bodies_global-1);
		break;

	// Shared down
	case 2:
		GetLog() << "Adding shared down\n";
		newbody->SetIdentifier(2); // Set as shared down
		data_manager->comm_status.push_back(communication::SHARED_DOWN);
		data_manager->global_id.push_back(num_bodies_global-1);
		break;

	// Owned
	case 3:
		GetLog() << "Adding owned\n";
		newbody->SetIdentifier(3); // Set as owned
		data_manager->comm_status.push_back(communication::OWNED);
		data_manager->global_id.push_back(num_bodies_global-1);
		break;

	// Ghost up
	case 4:
		GetLog() << "Adding ghost\n";
		newbody->SetIdentifier(4); // Set as ghost
		data_manager->comm_status.push_back(communication::GHOST);
		data_manager->global_id.push_back(num_bodies_global-1);
		break;

	// Ghost down
	case 5:
		GetLog() << "Adding ghost\n";
		newbody->SetIdentifier(4); // Set as ghost
		data_manager->comm_status.push_back(communication::GHOST);
		data_manager->global_id.push_back(num_bodies_global-1);
		break;

	default:
		GetLog() << "ERROR: Global ID: " << newbody->GetGid() << "Undefined comm status ChSystemDistr::AddBody.\n";
	}


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
		if (data_manager->host_data.active_rigid[i])
		{
			if (data_manager->comm_status[i] == communication::SHARED_DOWN || data_manager->comm_status[i] == communication::SHARED_UP)
			{
				GetLog() << "Global ID: " << data_manager->global_id[i] << " Shared";
			}
			else if (data_manager->comm_status[i] == communication::GHOST)
			{
				GetLog() << "Global ID: " << data_manager->global_id[i] << " Ghost";
			}
			else if (data_manager->comm_status[i] == communication::OWNED)
			{
				GetLog() << "Global ID: " << data_manager->global_id[i] << " Owned";
			}
			else
			{
				GetLog() << "ERROR: Global ID: " << data_manager->global_id[i] << " Undefined comm_status\n";
			}

			real3 pos = data_manager->host_data.pos_rigid[i];
			fprintf(stdout, " Pos: %.2f,%.2f,%.2f\n",pos.x, pos.y, pos.z);
		}
	}
}
