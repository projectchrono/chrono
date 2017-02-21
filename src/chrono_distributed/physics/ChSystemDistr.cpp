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

	data_manager = new ChParallelDataManager();
	domain = 0;
	comm = std::make_shared<ChCommDistr>((std::shared_ptr<ChSystemDistr>) this);
	collision_system = new collision::ChCollisionSystemParallel(data_manager); //TODO

	this->ghost_layer = ghost_layer;
	this->num_bodies_global = 0;

	descriptor = new ChSystemDescriptorParallel(data_manager);
	contact_container = std::make_shared<ChContactContainerParallel>(data_manager);

	collision_system_type = COLLSYS_PARALLEL; // Change to distributed???
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
////////////////////////////////////////////////////


    solver_speed = new ChIterativeSolverParallelDEM(data_manager);

    data_manager->settings.collision.collision_envelope = 0;

    // Set this so that the CD can check what type of system it is (needed for narrowphase)
    data_manager->settings.system_type = SYSTEM_DEM;

    data_manager->system_timer.AddTimer("ChIterativeSolverParallelDEM_ProcessContact");
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

void ChSystemDistr::RecomputeThreads() {
#ifdef CHRONO_OMP_FOUND
    timer_accumulator.insert(timer_accumulator.begin(), data_manager->system_timer.GetTime("step"));
    timer_accumulator.pop_back();

    double sum_of_elems = std::accumulate(timer_accumulator.begin(), timer_accumulator.end(), 0.0);

    if (frame_threads == 50 && detect_optimal_threads == false) {
        frame_threads = 0;
        if (current_threads + 2 < data_manager->settings.max_threads) {
            detect_optimal_threads = true;
            old_timer = sum_of_elems / 10.0;
            current_threads += 2;
            omp_set_num_threads(current_threads);

            LOG(TRACE) << "current threads increased to " << current_threads;

        } else {
            current_threads = data_manager->settings.max_threads;
            omp_set_num_threads(data_manager->settings.max_threads);

            LOG(TRACE) << "current threads increased to " << current_threads;
        }
    } else if (frame_threads == 10 && detect_optimal_threads) {
        double current_timer = sum_of_elems / 10.0;
        detect_optimal_threads = false;
        frame_threads = 0;
        if (old_timer < current_timer) {
            current_threads -= 2;
            omp_set_num_threads(current_threads);
            LOG(TRACE) << "current threads reduced back to " << current_threads;
        }
    }

    if (current_threads < data_manager->settings.min_threads) {
        current_threads = data_manager->settings.min_threads;
        omp_set_num_threads(data_manager->settings.min_threads);
    }
    frame_threads++;
#endif
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


//TODO Cases
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

//
// Reset forces for all variables
//
void ChSystemDistr::ClearForceVariables() {
#pragma omp parallel for
    for (int i = 0; i < data_manager->num_rigid_bodies; i++) {
        bodylist[i]->VariablesFbReset();
    }
}

void ChSystemDistr::RemoveBody(int global_id)
{
}

void ChSystemDistr::Update() {
    LOG(INFO) << "ChSystemDistr::Update()";
    // Clear the forces for all variables
    ClearForceVariables();

    // Allocate space for the velocities and forces for all objects
    data_manager->host_data.v.resize(data_manager->num_dof);
    data_manager->host_data.hf.resize(data_manager->num_dof);

    // Clear system-wide vectors for bilateral constraints
    data_manager->host_data.bilateral_mapping.clear();
    data_manager->host_data.bilateral_type.clear();

    this->descriptor->BeginInsertion();
    //UpdateLinks();
    //UpdateOtherPhysics();
    UpdateRigidBodies();//TODO
    //UpdateShafts();
    //Update3DOFBodies();
    descriptor->EndInsertion();

    //UpdateBilaterals();
}
//**********************







//
// Update all bodies in the system and populate system-wide state and force
// vectors. Note that visualization assets are not updated.
//
void ChSystemDistr::UpdateRigidBodies() {
	custom_vector<real3>& position = data_manager->host_data.pos_rigid;
	custom_vector<quaternion>& rotation = data_manager->host_data.rot_rigid;
	custom_vector<char>& active = data_manager->host_data.active_rigid;
	custom_vector<char>& collide = data_manager->host_data.collide_rigid;

#pragma omp parallel for
	for (int i = 0; i < bodylist.size(); i++) {
		bodylist[i]->Update(ChTime, false);
		bodylist[i]->VariablesFbLoadForces(GetStep());
		bodylist[i]->VariablesQbLoadSpeed();

		ChMatrix<>& body_qb = bodylist[i]->Variables().Get_qb();
		ChMatrix<>& body_fb = bodylist[i]->Variables().Get_fb();
		ChVector<>& body_pos = bodylist[i]->GetPos();
		ChQuaternion<>& body_rot = bodylist[i]->GetRot();

		data_manager->host_data.v[i * 6 + 0] = body_qb.GetElementN(0);
		data_manager->host_data.v[i * 6 + 1] = body_qb.GetElementN(1);
		data_manager->host_data.v[i * 6 + 2] = body_qb.GetElementN(2);
		data_manager->host_data.v[i * 6 + 3] = body_qb.GetElementN(3);
		data_manager->host_data.v[i * 6 + 4] = body_qb.GetElementN(4);
		data_manager->host_data.v[i * 6 + 5] = body_qb.GetElementN(5);

		data_manager->host_data.hf[i * 6 + 0] = body_fb.ElementN(0);
		data_manager->host_data.hf[i * 6 + 1] = body_fb.ElementN(1);
		data_manager->host_data.hf[i * 6 + 2] = body_fb.ElementN(2);
		data_manager->host_data.hf[i * 6 + 3] = body_fb.ElementN(3);
		data_manager->host_data.hf[i * 6 + 4] = body_fb.ElementN(4);
		data_manager->host_data.hf[i * 6 + 5] = body_fb.ElementN(5);

		position[i] = real3(body_pos.x, body_pos.y, body_pos.z);
		rotation[i] = quaternion(body_rot.e0, body_rot.e1, body_rot.e2, body_rot.e3);

		active[i] = bodylist[i]->IsActive();
		collide[i] = bodylist[i]->GetCollide();

		// Let derived classes set the specific material surface data.
		// UpdateMaterialSurfaceData(i, bodylist[i].get());

		bodylist[i]->GetCollisionModel()->SyncPosition();
	}
}


// Used to end the program on an error and print a message.
void ChSystemDistr::ErrorAbort(std::string msg)
{
	if (my_rank == 0) GetLog() << msg << '\n';
	MPI_Abort(world, MPI_ERR_OTHER);
}

void ChSystemDistr::PrintStepStats() {
    data_manager->system_timer.PrintReport();
}

int ChSystemDistr::GetNumContacts() {
    return data_manager->num_rigid_contacts + data_manager->num_rigid_fluid_contacts + data_manager->num_fluid_contacts;
}

/// Gets the time (in seconds) spent for computing the time step
double ChSystemDistr::GetTimerStep() {
    return data_manager->system_timer.GetTime("step");
}

/// Gets the fraction of time (in seconds) for the solution of the problem, within the time step
double ChSystemDistr::GetTimerSolver() {
    return data_manager->system_timer.GetTime("solver");
}
/// Gets the fraction of time (in seconds) for finding collisions, within the time step
double ChSystemDistr::GetTimerCollisionBroad() {
    return data_manager->system_timer.GetTime("collision_broad");
}
/// Gets the fraction of time (in seconds) for finding collisions, within the time step
double ChSystemDistr::GetTimerCollisionNarrow() {
    return data_manager->system_timer.GetTime("collision_narrow");
}
/// Gets the fraction of time (in seconds) for updating auxiliary data, within the time step
double ChSystemDistr::GetTimerUpdate() {
    return data_manager->system_timer.GetTime("update");
}

/// Gets the total time for the collision detection step
double ChSystemDistr::GetTimerCollision() {
    return data_manager->system_timer.GetTime("collision");
}

settings_container* ChSystemDistr::GetSettings() {
    return &(data_manager->settings);
}

ChBody* ChSystemDistr::NewBody() {
    if (collision_system_type == COLLSYS_PARALLEL)
        return new ChBody(new collision::ChCollisionModelParallel, ChMaterialSurfaceBase::DEM);

    return new ChBody(ChMaterialSurfaceBase::DEM);
}


void ChSystemDistr::AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) {
    assert(newbody->GetContactMethod() == ChMaterialSurfaceBase::DEM);

    // Reserve space for material properties for the specified body. Not that the
    // actual data is set in UpdateMaterialProperties().
    data_manager->host_data.mu.push_back(0);
    data_manager->host_data.cohesion_data.push_back(0);
    data_manager->host_data.adhesionMultDMT_data.push_back(0);

    data_manager->host_data.mass_rigid.push_back(0);

    if (data_manager->settings.solver.use_material_properties) {
        data_manager->host_data.elastic_moduli.push_back(real2(0, 0));
        data_manager->host_data.cr.push_back(0);
    } else {
        data_manager->host_data.dem_coeffs.push_back(real4(0, 0, 0, 0));
    }

    if (data_manager->settings.solver.tangential_displ_mode == ChSystemDEM::TangentialDisplacementModel::MultiStep) {
        for (int i = 0; i < max_shear; i++) {
            data_manager->host_data.shear_neigh.push_back(vec3(-1, -1, -1));
            data_manager->host_data.shear_disp.push_back(real3(0, 0, 0));
        }
    }
}

void ChSystemDistr::UpdateMaterialSurfaceData(int index, ChBody* body) {
    custom_vector<real>& mass = data_manager->host_data.mass_rigid;
    custom_vector<real2>& elastic_moduli = data_manager->host_data.elastic_moduli;
    custom_vector<real>& adhesion = data_manager->host_data.cohesion_data;
    custom_vector<real>& adhesionMult = data_manager->host_data.adhesionMultDMT_data;
    custom_vector<real>& mu = data_manager->host_data.mu;
    custom_vector<real>& cr = data_manager->host_data.cr;
    custom_vector<real4>& dem_coeffs = data_manager->host_data.dem_coeffs;

    // Since this function is called in a parallel for loop, we must access the
    // material properties in a thread-safe manner (we cannot use the function
    // ChBody::GetMaterialSurfaceDEM since that returns a copy of the reference
    // counted shared pointer).
    std::shared_ptr<ChMaterialSurfaceBase>& mat = body->GetMaterialSurfaceBase();
    ChMaterialSurfaceDEM* mat_ptr = static_cast<ChMaterialSurfaceDEM*>(mat.get());

    mass[index] = body->GetMass();
    mu[index] = mat_ptr->GetSfriction();
    adhesion[index] = mat_ptr->GetAdhesion();
    adhesionMult[index] = mat_ptr->GetAdhesionMultDMT();

    if (data_manager->settings.solver.use_material_properties) {
        elastic_moduli[index] = real2(mat_ptr->GetYoungModulus(), mat_ptr->GetPoissonRatio());
        cr[index] = mat_ptr->GetRestitution();
    } else {
        dem_coeffs[index] = real4(mat_ptr->GetKn(), mat_ptr->GetKt(), mat_ptr->GetGn(), mat_ptr->GetGt());
    }
}

void ChSystemDistr::Setup() {
    // First, invoke the base class method

	//ChSystemParallel::Setup();
    LOG(INFO) << "ChSystemParallel::Setup()";
    // Cache the integration step size and calculate the tolerance at impulse level.
    data_manager->settings.step_size = step;
    data_manager->settings.solver.tol_speed = step * data_manager->settings.solver.tolerance;
    data_manager->settings.gravity = real3(G_acc.x, G_acc.y, G_acc.z);

    // Calculate the total number of degrees of freedom (6 per rigid body and 1
    // for each shaft element).
    data_manager->num_dof = data_manager->num_rigid_bodies * 6 + data_manager->num_shafts +
                            data_manager->num_fluid_bodies * 3 + data_manager->num_fea_nodes * 3;

    // Set variables that are stored in the ChSystem class
    nbodies = data_manager->num_rigid_bodies;
    nlinks = 0;
    nphysicsitems = 0;
    ncoords = 0;
    ndoc = 0;
    nsysvars = 0;
    ncoords_w = 0;
    ndoc_w = 0;
    nsysvars_w = 0;
    ndof = data_manager->num_dof;
    ndoc_w_C = 0;
    ndoc_w_D = 0;
    ncontacts =
        data_manager->num_rigid_contacts + data_manager->num_rigid_fluid_contacts + data_manager->num_fluid_contacts;
    nbodies_sleep = 0;
    nbodies_fixed = 0;



    // Ensure that the collision envelope is zero.
    data_manager->settings.collision.collision_envelope = 0;
}

real3 ChSystemDistr::GetBodyContactForce(uint body_id) const {
    int index = data_manager->host_data.ct_body_map[body_id];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_force[index];
}

real3 ChSystemDistr::GetBodyContactTorque(uint body_id) const {
    int index = data_manager->host_data.ct_body_map[body_id];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_torque[index];
}


} /* namespace chrono */
