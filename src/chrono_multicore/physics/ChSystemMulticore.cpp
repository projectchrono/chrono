// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================
//
// Description: The definition of a multicore ChSystem, pretty much everything is
// done manually instead of using the functions used in ChSystem. This is to
// handle the different data structures present in the multicore implementation
//
// =============================================================================

#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChShaftsGearbox.h"
#include "chrono/physics/ChShaftsGearboxAngled.h"
#include "chrono/physics/ChShaftsPlanetary.h"

#include "chrono/multicore_math/matrix.h"

#include "chrono_multicore/ChConfigMulticore.h"
#include "chrono_multicore/collision/ChCollisionSystemChronoMulticore.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSolverMulticore.h"
#include "chrono_multicore/solver/ChSystemDescriptorMulticore.h"

#include <numeric>


namespace chrono {

ChSystemMulticore::ChSystemMulticore() : ChSystem() {
    data_manager = new ChMulticoreDataManager();

    descriptor = chrono_types::make_shared<ChSystemDescriptorMulticore>(data_manager);

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
    data_manager->system_timer.AddTimer("advance");

    data_manager->system_timer.AddTimer("collision");
    data_manager->system_timer.AddTimer("collision_broad");
    data_manager->system_timer.AddTimer("collision_narrow");

    data_manager->system_timer.AddTimer("ChIterativeSolverMulticore_Solve");
    data_manager->system_timer.AddTimer("ChIterativeSolverMulticore_Setup");
    data_manager->system_timer.AddTimer("ChIterativeSolverMulticore_Matrices");
    data_manager->system_timer.AddTimer("ChIterativeSolverMulticore_Stab");
}

ChSystemMulticore::ChSystemMulticore(const ChSystemMulticore& other) : ChSystem(other) {
    //// TODO
}

ChSystemMulticore::~ChSystemMulticore() {
    delete data_manager;
}

bool ChSystemMulticore::Integrate_Y() {
    ResetTimers();
    timer_step.start();  // time elapsed for step (for RTF calculation)

    // Store system data in the data manager
    data_manager->system_descriptor = this->descriptor;
    data_manager->body_list = &assembly.bodylist;
    data_manager->link_list = &assembly.linklist;
    data_manager->other_physics_list = &assembly.otherphysicslist;

    data_manager->system_timer.Reset();
    data_manager->system_timer.start("step");

    Setup();

    data_manager->system_timer.start("update");
    Update();
    data_manager->system_timer.stop("update");

    data_manager->system_timer.start("collision");
    if (collision_system) {
        collision_system->PreProcess();
        collision_system->Run();
        collision_system->PostProcess();
        collision_system->ReportContacts(this->contact_container.get());
        for (size_t ic = 0; ic < collision_callbacks.size(); ic++) {
            collision_callbacks[ic]->OnCustomCollision(this);
        }
    }
    data_manager->system_timer.stop("collision");

    data_manager->system_timer.start("advance");
    std::static_pointer_cast<ChIterativeSolverMulticore>(solver)->RunTimeStep();
    data_manager->system_timer.stop("advance");

    data_manager->system_timer.start("update");

    // Iterate over the active bilateral constraints and store their Lagrange
    // multiplier.
    std::vector<ChConstraint*>& mconstraints = descriptor->GetConstraintsList();
    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        mconstraints[cntr]->Set_l_i(data_manager->host_data.gamma[data_manager->num_unilaterals + index]);
    }

    // Update the constraint reactions.
    double factor = 1 / this->GetStep();
    for (auto& link : assembly.linklist) {
        link->ConstraintsFetch_react(factor);
    }
    for (auto& item : assembly.otherphysicslist) {
        item->ConstraintsFetch_react(factor);
    }
    contact_container->ConstraintsFetch_react(factor);

    // Scatter the states to the Chrono objects (bodies and shafts) and update
    // all physics items at the end of the step.
    DynamicVector<real>& velocities = data_manager->host_data.v;
    custom_vector<real3>& pos_pointer = data_manager->host_data.pos_rigid;
    custom_vector<quaternion>& rot_pointer = data_manager->host_data.rot_rigid;

#pragma omp parallel for
    for (int i = 0; i < assembly.bodylist.size(); i++) {
        if (data_manager->host_data.active_rigid[i] != 0) {
            auto& body = assembly.bodylist[i];
            body->Variables().Get_qb()(0) = velocities[i * 6 + 0];
            body->Variables().Get_qb()(1) = velocities[i * 6 + 1];
            body->Variables().Get_qb()(2) = velocities[i * 6 + 2];
            body->Variables().Get_qb()(3) = velocities[i * 6 + 3];
            body->Variables().Get_qb()(4) = velocities[i * 6 + 4];
            body->Variables().Get_qb()(5) = velocities[i * 6 + 5];

            body->VariablesQbIncrementPosition(this->GetStep());
            body->VariablesQbSetSpeed(this->GetStep());

            body->Update(ch_time);

            // update the position and rotation vectors
            pos_pointer[i] = (real3(body->GetPos().x(), body->GetPos().y(), body->GetPos().z()));
            rot_pointer[i] =
                (quaternion(body->GetRot().e0(), body->GetRot().e1(), body->GetRot().e2(), body->GetRot().e3()));
        }
    }

    uint offset = data_manager->num_rigid_bodies * 6;
    ////#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_shafts; i++) {
        if (data_manager->host_data.shaft_active[i] != 0) {
            auto& shaft = assembly.shaftlist[i];
            shaft->Variables().Get_qb()(0) = velocities[offset + i];
            shaft->VariablesQbIncrementPosition(GetStep());
            shaft->VariablesQbSetSpeed(GetStep());
            shaft->Update(ch_time);
        }
    }

    offset += data_manager->num_shafts;
    for (int i = 0; i < (signed)data_manager->num_linmotors; i++) {
        linmotorlist[i]->Variables().Get_qb()(0) = velocities[offset + i];
        linmotorlist[i]->VariablesQbIncrementPosition(GetStep());
        linmotorlist[i]->VariablesQbSetSpeed(GetStep());
        linmotorlist[i]->Update(ch_time, true);
    }

    offset += data_manager->num_linmotors;
    for (int i = 0; i < (signed)data_manager->num_rotmotors; i++) {
        rotmotorlist[i]->Variables().Get_qb()(0) = velocities[offset + i];
        rotmotorlist[i]->VariablesQbIncrementPosition(GetStep());
        rotmotorlist[i]->VariablesQbSetSpeed(GetStep());
        rotmotorlist[i]->Update(ch_time, true);
    }

    for (int i = 0; i < assembly.otherphysicslist.size(); i++) {
        assembly.otherphysicslist[i]->Update(ch_time);
    }

    data_manager->node_container->UpdatePosition(ch_time);
    data_manager->system_timer.stop("update");

    //=============================================================================================
    ch_time += GetStep();
    data_manager->system_timer.stop("step");
    if (data_manager->settings.perform_thread_tuning) {
        RecomputeThreads();
    }

    timer_step.stop();  // time elapsed for step (for RTF calculation)

    return true;
}

// Add the specified body to the system.
// A unique identifier is assigned to each body for indexing purposes.
// Space is allocated in system-wide vectors for data corresponding to the
// body.
void ChSystemMulticore::AddBody(std::shared_ptr<ChBody> newbody) {
    // This is only need because bilaterals need to know what bodies to
    // refer to. Not used by contacts
    newbody->SetId(data_manager->num_rigid_bodies);

    assembly.bodylist.push_back(newbody);
    data_manager->num_rigid_bodies++;

    // Set the system for the body.  Note that this will also add the body's
    // collision shapes to the collision system if not already done.
    newbody->SetSystem(this);

    // Reserve space for this body in the system-wide vectors. Note that the
    // actual data is set in UpdateBodies().
    data_manager->host_data.pos_rigid.push_back(real3());
    data_manager->host_data.rot_rigid.push_back(quaternion());
    data_manager->host_data.active_rigid.push_back(true);
    data_manager->host_data.collide_rigid.push_back(true);

    // Let derived classes reserve space for specific material surface data
    AddMaterialSurfaceData(newbody);
}

// Add the specified shaft to the system.
// A unique identifier is assigned to each shaft for indexing purposes.
// Space is allocated in system-wide vectors for data corresponding to the shaft.
void ChSystemMulticore::AddShaft(std::shared_ptr<ChShaft> shaft) {
    shaft->SetId(data_manager->num_shafts);
    shaft->SetSystem(this);

    assembly.shaftlist.push_back(shaft);
    data_manager->num_shafts++;

    // Reserve space for this shaft in the system-wide vectors. Not that the
    // actual data is set in UpdateShafts().
    data_manager->host_data.shaft_rot.push_back(0);
    data_manager->host_data.shaft_inr.push_back(0);
    data_manager->host_data.shaft_active.push_back(true);
}

void ChSystemMulticore::AddLink(std::shared_ptr<ChLinkBase> link) {
    if (link->GetDOF() == 1) {
        if (auto mot = std::dynamic_pointer_cast<ChLinkMotorLinearSpeed>(link)) {
            linmotorlist.push_back(mot.get());
            data_manager->num_linmotors++;
            data_manager->num_motors++;
        }
        if (auto mot = std::dynamic_pointer_cast<ChLinkMotorRotationSpeed>(link)) {
            rotmotorlist.push_back(mot.get());
            data_manager->num_rotmotors++;
            data_manager->num_motors++;
        }
    }

    ChSystem::AddLink(link);
}

// Add physics items, other than bodies, shafts, or links, to the system.
// Note that no test is performed to check if the item was already added.
void ChSystemMulticore::AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> newitem) {
    newitem->SetSystem(this);
    assembly.otherphysicslist.push_back(newitem);

    ////if (newitem->GetCollide()) {
    ////    newitem->AddCollisionModelsToSystem();
    ////}
}

//
// Reset forces for all variables
//
void ChSystemMulticore::ClearForceVariables() {
#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_rigid_bodies; i++) {
        assembly.bodylist[i]->VariablesFbReset();
    }

    ////#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_shafts; i++) {
        assembly.shaftlist[i]->VariablesFbReset();
    }

    for (int i = 0; i < (signed)data_manager->num_linmotors; i++) {
        linmotorlist[i]->VariablesFbReset();
    }

    for (int i = 0; i < (signed)data_manager->num_rotmotors; i++) {
        rotmotorlist[i]->VariablesFbReset();
    }
}

//
// Update all items in the system. The following order of operations is important:
// 1. Clear the force vectors by calling VariablesFbReset for all objects
// 2. Compute link constraint forces
// 3. Update other physics items (other than shafts)
// 4. Update bodies (these introduce state variables)
// 5. Update shafts (these introduce state variables)
// 6. Update motor links with states (these introduce state variables)
// 7. Update 3DOF onjects (these introduce state variables)
// 8. Process bilateral constraints
//
void ChSystemMulticore::Update() {
    // Clear the forces for all variables
    ClearForceVariables();

    // Allocate space for the velocities and forces for all objects
    data_manager->host_data.v.resize(data_manager->num_dof);
    data_manager->host_data.hf.resize(data_manager->num_dof);

    // Clear system-wide vectors for bilateral constraints
    data_manager->host_data.bilateral_mapping.clear();
    data_manager->host_data.bilateral_type.clear();

    this->descriptor->BeginInsertion();
    UpdateLinks();
    UpdateOtherPhysics();
    UpdateRigidBodies();
    UpdateShafts();
    UpdateMotorLinks();
    Update3DOFBodies();
    descriptor->EndInsertion();

    UpdateBilaterals();
}

//
// Update all bodies in the system and populate system-wide state and force
// vectors. Note that visualization assets are not updated.
//
void ChSystemMulticore::UpdateRigidBodies() {
    custom_vector<real3>& position = data_manager->host_data.pos_rigid;
    custom_vector<quaternion>& rotation = data_manager->host_data.rot_rigid;
    custom_vector<char>& active = data_manager->host_data.active_rigid;
    custom_vector<char>& collide = data_manager->host_data.collide_rigid;

#pragma omp parallel for
    for (int i = 0; i < assembly.bodylist.size(); i++) {
        auto& body = assembly.bodylist[i];

        body->Update(ch_time, false);
        body->VariablesFbLoadForces(GetStep());
        body->VariablesQbLoadSpeed();

        ChVectorRef body_qb = body->Variables().Get_qb();
        ChVectorRef body_fb = body->Variables().Get_fb();
        ChVector<>& body_pos = body->GetPos();
        ChQuaternion<>& body_rot = body->GetRot();

        data_manager->host_data.v[i * 6 + 0] = body_qb(0);
        data_manager->host_data.v[i * 6 + 1] = body_qb(1);
        data_manager->host_data.v[i * 6 + 2] = body_qb(2);
        data_manager->host_data.v[i * 6 + 3] = body_qb(3);
        data_manager->host_data.v[i * 6 + 4] = body_qb(4);
        data_manager->host_data.v[i * 6 + 5] = body_qb(5);

        data_manager->host_data.hf[i * 6 + 0] = body_fb(0);
        data_manager->host_data.hf[i * 6 + 1] = body_fb(1);
        data_manager->host_data.hf[i * 6 + 2] = body_fb(2);
        data_manager->host_data.hf[i * 6 + 3] = body_fb(3);
        data_manager->host_data.hf[i * 6 + 4] = body_fb(4);
        data_manager->host_data.hf[i * 6 + 5] = body_fb(5);

        position[i] = real3(body_pos.x(), body_pos.y(), body_pos.z());
        rotation[i] = quaternion(body_rot.e0(), body_rot.e1(), body_rot.e2(), body_rot.e3());

        active[i] = body->IsActive();
        collide[i] = body->GetCollide();

        // Let derived classes set the specific material surface data.
        UpdateMaterialSurfaceData(i, body.get());

        // Synchronize collision model (if any)
        if (body->GetCollisionModel())
            body->GetCollisionModel()->SyncPosition();
    }
}

//
// Update all shaft elements in the system and populate system-wide state and
// force vectors. Note that visualization assets are not updated.
//
void ChSystemMulticore::UpdateShafts() {
    real* shaft_rot = data_manager->host_data.shaft_rot.data();
    real* shaft_inr = data_manager->host_data.shaft_inr.data();
    char* shaft_active = data_manager->host_data.shaft_active.data();

    uint offset = data_manager->num_rigid_bodies * 6;
    ////#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_shafts; i++) {
        auto& shaft = assembly.shaftlist[i];

        shaft->Update(ch_time, false);
        shaft->VariablesFbLoadForces(GetStep());
        shaft->VariablesQbLoadSpeed();

        shaft_rot[i] = shaft->GetPos();
        shaft_inr[i] = shaft->Variables().GetInvInertia();
        shaft_active[i] = shaft->IsActive();

        data_manager->host_data.v[offset + i] = shaft->Variables().Get_qb()(0);
        data_manager->host_data.hf[offset + i] = shaft->Variables().Get_fb()(0);
    }
}

//
// Update all motor links that introduce *exactly* one variable.
// TODO: extend this to links with more than one variable.
//
void ChSystemMulticore::UpdateMotorLinks() {
    uint offset = data_manager->num_rigid_bodies * 6 + data_manager->num_shafts;
    for (uint i = 0; i < data_manager->num_linmotors; i++) {
        linmotorlist[i]->Update(ch_time, false);
        linmotorlist[i]->VariablesFbLoadForces(GetStep());
        linmotorlist[i]->VariablesQbLoadSpeed();
        data_manager->host_data.v[offset + i] = linmotorlist[i]->Variables().Get_qb()(0);
        data_manager->host_data.hf[offset + i] = linmotorlist[i]->Variables().Get_fb()(0);
    }
    offset += data_manager->num_linmotors;
    for (uint i = 0; i < data_manager->num_rotmotors; i++) {
        rotmotorlist[i]->Update(ch_time, false);
        rotmotorlist[i]->VariablesFbLoadForces(GetStep());
        rotmotorlist[i]->VariablesQbLoadSpeed();
        data_manager->host_data.v[offset + i] = rotmotorlist[i]->Variables().Get_qb()(0);
        data_manager->host_data.hf[offset + i] = rotmotorlist[i]->Variables().Get_fb()(0);
    }
}

//
// Update all fluid nodes
//
void ChSystemMulticore::Update3DOFBodies() {
    data_manager->node_container->Update3DOF(ch_time);
}

//
// Update all links in the system and set the type of the associated constraints
// to BODY_BODY. Note that visualization assets are not updated.
//
void ChSystemMulticore::UpdateLinks() {
    double oostep = 1 / GetStep();
    real clamp_speed = data_manager->settings.solver.bilateral_clamp_speed;
    bool clamp = data_manager->settings.solver.clamp_bilaterals;

    for (int i = 0; i < assembly.linklist.size(); i++) {
        auto& link = assembly.linklist[i];

        link->Update(ch_time, false);
        link->ConstraintsBiReset();
        link->ConstraintsBiLoad_C(oostep, clamp_speed, clamp);
        link->ConstraintsBiLoad_Ct(1);
        link->ConstraintsFbLoadForces(GetStep());
        link->ConstraintsLoadJacobians();

        link->InjectConstraints(*descriptor);

        for (int j = 0; j < link->GetDOC_c(); j++)
            data_manager->host_data.bilateral_type.push_back(BilateralType::BODY_BODY);
    }
}

//
// This utility function returns the type of constraints associated with the
// specified physics item. Return UNKNOWN if the item has no associated
// bilateral constraints or if it is unsupported.
//
BilateralType GetBilateralType(ChPhysicsItem* item) {
    if (item->GetDOC_c() == 0)
        return BilateralType::UNKNOWN;

    if (dynamic_cast<ChShaftsCouple*>(item))
        return BilateralType::SHAFT_SHAFT;

    if (dynamic_cast<ChShaftsPlanetary*>(item))
        return BilateralType::SHAFT_SHAFT_SHAFT;

    if (dynamic_cast<ChShaftsGearbox*>(item) || dynamic_cast<ChShaftsGearboxAngled*>(item))
        return BilateralType::SHAFT_SHAFT_BODY;

    if (dynamic_cast<ChShaftsBody*>(item))
        return BilateralType::SHAFT_BODY;

    // Debug check - do we ignore any constraints?
    assert(item->GetDOC_c() == 0);

    return BilateralType::UNKNOWN;
}

//
// Update other physics items in the system and set the type of the associated
// constraints.
// Notes:
// - ChShaft elements have already been excluded (as these are treated separately)
// - allow all items to include body forces (required e.g. ChShaftsTorqueBase)
// - no support for any items that introduce additional state variables
// - only include constraints from items of supported type (see GetBilateralType above)
// - visualization assets are not updated
//
void ChSystemMulticore::UpdateOtherPhysics() {
    double oostep = 1 / GetStep();
    real clamp_speed = data_manager->settings.solver.bilateral_clamp_speed;
    bool clamp = data_manager->settings.solver.clamp_bilaterals;

    for (int i = 0; i < assembly.otherphysicslist.size(); i++) {
        auto& item = assembly.otherphysicslist[i];

        item->Update(ch_time, false);
        item->ConstraintsBiReset();
        item->ConstraintsBiLoad_C(oostep, clamp_speed, clamp);
        item->ConstraintsBiLoad_Ct(1);
        item->ConstraintsFbLoadForces(GetStep());
        item->ConstraintsLoadJacobians();
        item->VariablesFbLoadForces(GetStep());
        item->VariablesQbLoadSpeed();

        BilateralType type = GetBilateralType(item.get());

        if (type == BilateralType::UNKNOWN)
            continue;

        item->InjectConstraints(*descriptor);

        for (int j = 0; j < item->GetDOC_c(); j++)
            data_manager->host_data.bilateral_type.push_back(type);
    }
}

//
// Collect indexes of all active bilateral constraints and calculate number of
// non-zero entries in the constraint Jacobian.
//
void ChSystemMulticore::UpdateBilaterals() {
    data_manager->nnz_bilaterals = 0;
    std::vector<ChConstraint*>& mconstraints = descriptor->GetConstraintsList();

    for (uint ic = 0; ic < mconstraints.size(); ic++) {
        if (mconstraints[ic]->IsActive()) {
            data_manager->host_data.bilateral_mapping.push_back(ic);
            switch (data_manager->host_data.bilateral_type[ic]) {
                case BilateralType::BODY_BODY:
                    data_manager->nnz_bilaterals += 12;
                    break;
                case BilateralType::SHAFT_SHAFT:
                    data_manager->nnz_bilaterals += 2;
                    break;
                case BilateralType::SHAFT_SHAFT_SHAFT:
                    data_manager->nnz_bilaterals += 3;
                    break;
                case BilateralType::SHAFT_BODY:
                    data_manager->nnz_bilaterals += 7;
                    break;
                case BilateralType::SHAFT_SHAFT_BODY:
                    data_manager->nnz_bilaterals += 8;
                    break;
            }
        }
    }
    // Set the number of currently active bilateral constraints.
    data_manager->num_bilaterals = (uint)data_manager->host_data.bilateral_mapping.size();
}

//
// Prepare simulation of the next step.  This function is called after
// the system update and before collision detection. A derived class can
// override this function, but it should invoke this default implementation.
//
void ChSystemMulticore::Setup() {
    // Cache the integration step size and calculate the tolerance at impulse level.
    data_manager->settings.step_size = step;
    data_manager->settings.solver.tol_speed = step * data_manager->settings.solver.tolerance;
    data_manager->settings.gravity = real3(G_acc.x(), G_acc.y(), G_acc.z());

    // Calculate the total number of degrees of freedom (6 per rigid body, 1 per shaft, 1 per motor).
    data_manager->num_dof = data_manager->num_rigid_bodies * 6 + data_manager->num_shafts + data_manager->num_motors +
                            data_manager->num_fluid_bodies * 3;

    // Set variables that are stored in the ChSystem class
    assembly.nbodies = data_manager->num_rigid_bodies;
    assembly.nlinks = 0;
    assembly.nphysicsitems = 0;
    ncoords = 0;
    ndoc = 0;
    nsysvars = 0;
    ncoords_w = 0;
    ndoc_w = 0;
    nsysvars_w = 0;
    ndof = data_manager->num_dof;
    ndoc_w_C = 0;
    ndoc_w_D = 0;
    if (data_manager->cd_data)
        ncontacts = data_manager->cd_data->num_rigid_contacts + data_manager->cd_data->num_rigid_fluid_contacts +
                    data_manager->cd_data->num_fluid_contacts;
    assembly.nbodies_sleep = 0;
    assembly.nbodies_fixed = 0;
}

void ChSystemMulticore::RecomputeThreads() {
#ifdef _OPENMP
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
        } else {
            current_threads = data_manager->settings.max_threads;
            omp_set_num_threads(data_manager->settings.max_threads);
        }
    } else if (frame_threads == 10 && detect_optimal_threads) {
        double current_timer = sum_of_elems / 10.0;
        detect_optimal_threads = false;
        frame_threads = 0;
        if (old_timer < current_timer) {
            current_threads -= 2;
            omp_set_num_threads(current_threads);
        }
    }

    if (current_threads < data_manager->settings.min_threads) {
        current_threads = data_manager->settings.min_threads;
        omp_set_num_threads(data_manager->settings.min_threads);
    }
    frame_threads++;
#endif
}

void ChSystemMulticore::SetCollisionSystemType(ChCollisionSystem::Type type) {
    assert(assembly.GetNbodies() == 0);

    if (type != ChCollisionSystem::Type::MULTICORE) {
        std::cout << "Only the Chrono multicore collision detection system is supported!" << std::endl;
        std::cout << "Creating a collision system of type ChCollisionSystemMulticore" << std::endl;
    }

    collision_system = chrono_types::make_shared<ChCollisionSystemChronoMulticore>(data_manager);
    collision_system->SetSystem(this);
}

// Calculate the (linearized) bilateral constraint violations and store them in
// the provided vector. Return the maximum constraint violation.
double ChSystemMulticore::CalculateConstraintViolation(std::vector<double>& cvec) {
    std::vector<ChConstraint*>& mconstraints = descriptor->GetConstraintsList();
    cvec.resize(data_manager->num_bilaterals);
    double max_c = 0;

    for (int index = 0; index < (signed)data_manager->num_bilaterals; index++) {
        int cntr = data_manager->host_data.bilateral_mapping[index];
        cvec[index] = mconstraints[cntr]->Compute_c_i();
        double abs_c = std::abs(cvec[index]);
        if (abs_c > max_c)
            max_c = abs_c;
    }

    return max_c;
}

void ChSystemMulticore::PrintStepStats() {
    data_manager->system_timer.PrintReport();
}

unsigned int ChSystemMulticore::GetNumBodies() {
    return data_manager->num_rigid_bodies + data_manager->num_fluid_bodies;
}

unsigned int ChSystemMulticore::GetNumShafts() {
    return data_manager->num_shafts;
}

unsigned int ChSystemMulticore::GetNumContacts() {
    if (!data_manager->cd_data)
        return 0;

    return data_manager->cd_data->num_rigid_contacts + data_manager->cd_data->num_rigid_fluid_contacts +
           data_manager->cd_data->num_fluid_contacts;
}

unsigned int ChSystemMulticore::GetNumBilaterals() {
    return data_manager->num_bilaterals;
}

// -------------------------------------------------------------

double ChSystemMulticore::GetTimerStep() const {
    return data_manager->system_timer.GetTime("step");
}

double ChSystemMulticore::GetTimerAdvance() const {
    return data_manager->system_timer.GetTime("advance");
}

double ChSystemMulticore::GetTimerUpdate() const {
    return data_manager->system_timer.GetTime("update");
}

double ChSystemMulticore::GetTimerLSsolve() const {
    return data_manager->system_timer.GetTime("ChIterativeSolverMulticore_Solve");
}

double ChSystemMulticore::GetTimerLSsetup() const {
    return data_manager->system_timer.GetTime("ChIterativeSolverMulticore_Setup");
}

double ChSystemMulticore::GetTimerJacobian() const {
    return data_manager->system_timer.GetTime("ChIterativeSolverMulticore_Matrices");
}

double ChSystemMulticore::GetTimerCollision() const {
    return data_manager->system_timer.GetTime("collision");
}

settings_container* ChSystemMulticore::GetSettings() {
    return &(data_manager->settings);
}

// -------------------------------------------------------------

void ChSystemMulticore::SetNumThreads(int num_threads_chrono, int num_threads_collision, int num_threads_eigen) {
    ChSystem::SetNumThreads(num_threads_chrono, num_threads_chrono, num_threads_eigen);

#ifdef _OPENMP
    int max_avail_threads = omp_get_num_procs();

    if (num_threads_chrono > max_avail_threads) {
        std::cout << "WARNING! Requested number of threads (" << num_threads_chrono << ") ";
        std::cout << "larger than maximum available (" << max_avail_threads << ")" << std::endl;
    }
    omp_set_num_threads(num_threads_chrono);
#else
    std::cout << "WARNING! OpenMP not enabled" << std::endl;
#endif
}

void ChSystemMulticore::EnableThreadTuning(int min_threads, int max_threads) {
#ifdef _OPENMP
    data_manager->settings.perform_thread_tuning = true;
    data_manager->settings.min_threads = min_threads;
    data_manager->settings.max_threads = max_threads;
    omp_set_num_threads(min_threads);
#else
    std::cout << "WARNING! OpenMP not enabled" << std::endl;
#endif
}

// -------------------------------------------------------------

void ChSystemMulticore::SetMaterialCompositionStrategy(std::unique_ptr<ChMaterialCompositionStrategy>&& strategy) {
    data_manager->composition_strategy = std::move(strategy);
}

// -------------------------------------------------------------

ChVector<> ChSystemMulticore::GetBodyAppliedForce(ChBody* body) {
    auto h = data_manager->settings.step_size;
    auto fx = data_manager->host_data.hf[body->GetId() * 6 + 0] / h;
    auto fy = data_manager->host_data.hf[body->GetId() * 6 + 1] / h;
    auto fz = data_manager->host_data.hf[body->GetId() * 6 + 2] / h;
    return ChVector<>((double)fx, (double)fy, (double)fz);
}

ChVector<> ChSystemMulticore::GetBodyAppliedTorque(ChBody* body) {
    auto h = data_manager->settings.step_size;
    auto tx = data_manager->host_data.hf[body->GetId() * 6 + 3] / h;
    auto ty = data_manager->host_data.hf[body->GetId() * 6 + 4] / h;
    auto tz = data_manager->host_data.hf[body->GetId() * 6 + 5] / h;
    return ChVector<>((double)tx, (double)ty, (double)tz);
}

}  // end namespace chrono
