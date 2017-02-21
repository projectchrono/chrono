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

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/real3.h"
#include "chrono_parallel/math/other_types.h"

namespace chrono {

ChSystemDistr::ChSystemDistr(MPI_Comm world, double ghost_layer, int max_objects) :
		ChSystemParallel(max_objects)
{
	this->world = world;
	MPI_Comm_size(world, &num_ranks);
	MPI_Comm_rank(world, &my_rank);

	data_manager = new ChParallelDataManager(); // TODO Change?
	domain = 0;
	comm = std::make_shared<ChCommDistr>((std::shared_ptr<ChSystemDistr>) this);
	collision_system = new collision::ChCollisionSystemParallel(data_manager); //TODO change?

	this->ghost_layer = ghost_layer;
	this->num_bodies_global = 0;
	
	collision_system_type = COLLSYS_PARALLEL; // Change to distributed???
}

//TODO *******
int ChSystemDistr::Integrate_Y() {
    LOG(INFO) << "ChSystemDistr::Integrate_Y() Time: " << ChTime;
    // Get the pointer for the system descriptor and store it into the data manager
    data_manager->system_descriptor = this->descriptor;
    data_manager->body_list = &this->bodylist;
    data_manager->link_list = &this->linklist;
    data_manager->other_physics_list = &this->otherphysicslist;

    data_manager->system_timer.Reset();
    data_manager->system_timer.start("step");

    Setup();

    data_manager->system_timer.start("update");
    Update();
    data_manager->system_timer.stop("update");

    data_manager->system_timer.start("collision");
    collision_system->Run();
    collision_system->ReportContacts(this->contact_container.get());

    for (size_t ic = 0; ic < collision_callbacks.size(); ic++) {
        collision_callbacks[ic]->PerformCustomCollision(this);
    }

    data_manager->system_timer.stop("collision");

    data_manager->system_timer.start("solver");
    ((ChIterativeSolverParallel*)(solver_speed))->RunTimeStep();
    data_manager->system_timer.stop("solver");

    data_manager->system_timer.start("update");

    // Iterate over the active bilateral constraints and store their Lagrange
    // multiplier.
    std::vector<ChConstraint*>& mconstraints = descriptor->GetConstraintsList();
    for (int index = 0; index < data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        mconstraints[cntr]->Set_l_i(data_manager->host_data.gamma[data_manager->num_unilaterals + index]);
    }

    // Update the constraint reactions.
    double factor = 1 / this->GetStep();
    for (int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsFetch_react(factor);
    }
    for (int ip = 0; ip < otherphysicslist.size(); ++ip) {
        otherphysicslist[ip]->ConstraintsFetch_react(factor);
    }
    contact_container->ConstraintsFetch_react(factor);

    // Scatter the states to the Chrono objects (bodies and shafts) and update
    // all physics items at the end of the step.
    DynamicVector<real>& velocities = data_manager->host_data.v;
    custom_vector<real3>& pos_pointer = data_manager->host_data.pos_rigid;
    custom_vector<quaternion>& rot_pointer = data_manager->host_data.rot_rigid;

#pragma omp parallel for
    for (int i = 0; i < bodylist.size(); i++) {
        if (data_manager->host_data.active_rigid[i] == true) {
            bodylist[i]->Variables().Get_qb().SetElement(0, 0, velocities[i * 6 + 0]);
            bodylist[i]->Variables().Get_qb().SetElement(1, 0, velocities[i * 6 + 1]);
            bodylist[i]->Variables().Get_qb().SetElement(2, 0, velocities[i * 6 + 2]);
            bodylist[i]->Variables().Get_qb().SetElement(3, 0, velocities[i * 6 + 3]);
            bodylist[i]->Variables().Get_qb().SetElement(4, 0, velocities[i * 6 + 4]);
            bodylist[i]->Variables().Get_qb().SetElement(5, 0, velocities[i * 6 + 5]);

            bodylist[i]->VariablesQbIncrementPosition(this->GetStep());
            bodylist[i]->VariablesQbSetSpeed(this->GetStep());

            bodylist[i]->Update(ChTime);

            // GetBodyContactTorqueupdate the position and rotation vectors
            pos_pointer[i] = (real3(bodylist[i]->GetPos().x, bodylist[i]->GetPos().y, bodylist[i]->GetPos().z));
            rot_pointer[i] = (quaternion(bodylist[i]->GetRot().e0, bodylist[i]->GetRot().e1, bodylist[i]->GetRot().e2,
                                         bodylist[i]->GetRot().e3));
        }
    }

    ////#pragma omp parallel for
    /*
    for (int i = 0; i < data_manager->num_shafts; i++) {
        if (!data_manager->host_data.shaft_active[i])
            continue;

        shaftlist[i]->Variables().Get_qb().SetElementN(0, velocities[data_manager->num_rigid_bodies * 6 + i]);
        shaftlist[i]->VariablesQbIncrementPosition(GetStep());
        shaftlist[i]->VariablesQbSetSpeed(GetStep());
        shaftlist[i]->Update(ChTime);
    }

    for (int i = 0; i < otherphysicslist.size(); i++) {
        otherphysicslist[i]->Update(ChTime);
    }

    data_manager->node_container->UpdatePosition(ChTime);
    data_manager->fea_container->UpdatePosition(ChTime);
    */
    data_manager->system_timer.stop("update");

    //=============================================================================================
    ChTime += GetStep();
    data_manager->system_timer.stop("step");
    if (data_manager->settings.perform_thread_tuning) {
        RecomputeThreads();
    }

    return 1;
}


ChSystemDistr::~ChSystemDistr()
{}

// Read in body data at the beginning of a simulation.
// Format:
// TODO:
void ChSystemDistr::ReadBodies(std::string filename)
{
}

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
