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
// Description: The definition of a parallel ChSystem, pretty much everything is
// done manually instead of using the functions used in ChSystem. This is to
// handle the different data structures present in the parallel implementation
//
// =============================================================================

#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/physics/ChShaftsGearbox.h"
#include "chrono/physics/ChShaftsGearboxAngled.h"
#include "chrono/physics/ChShaftsPlanetary.h"

#include "chrono/fea/ChElementTetra_4.h"
#include "chrono/fea/ChNodeFEAxyz.h"

#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChCollisionModelParallel.h"
#include "chrono_parallel/collision/ChCollisionSystemBulletParallel.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/math/matrix.h"  // for quaternion, real4
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"

#include <numeric>

using namespace chrono::collision;

#ifdef LOGGINGENABLED
INITIALIZE_EASYLOGGINGPP
#endif

namespace chrono {

ChSystemParallel::ChSystemParallel() : ChSystem() {
    data_manager = new ChParallelDataManager();

    descriptor = std::make_shared<ChSystemDescriptorParallel>(data_manager);
    contact_container = std::make_shared<ChContactContainerParallel>(data_manager);
    contact_container->SetSystem(this);
    collision_system = std::make_shared<ChCollisionSystemParallel>(data_manager);

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
    data_manager->system_timer.AddTimer("advance");

    data_manager->system_timer.AddTimer("collision");
    data_manager->system_timer.AddTimer("collision_broad");
    data_manager->system_timer.AddTimer("collision_narrow");

    data_manager->system_timer.AddTimer("ChIterativeSolverParallel_Solve");
    data_manager->system_timer.AddTimer("ChIterativeSolverParallel_Setup");
    data_manager->system_timer.AddTimer("ChIterativeSolverParallel_Matrices");
    data_manager->system_timer.AddTimer("ChIterativeSolverParallel_Stab");

#ifdef LOGGINGENABLED
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToFile, "false");
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%datetime{%h:%m:%s:%g} %msg");
#endif
}

ChSystemParallel::ChSystemParallel(const ChSystemParallel& other) : ChSystem(other) {
    //// TODO
}

ChSystemParallel::~ChSystemParallel() {
    delete data_manager;
}

bool ChSystemParallel::Integrate_Y() {
    LOG(INFO) << "ChSystemParallel::Integrate_Y() Time: " << ChTime;
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
        collision_callbacks[ic]->OnCustomCollision(this);
    }
    data_manager->system_timer.stop("collision");

    data_manager->system_timer.start("advance");
    std::static_pointer_cast<ChIterativeSolverParallel>(solver_speed)->RunTimeStep();
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
        if (data_manager->host_data.active_rigid[i] != 0) {
            bodylist[i]->Variables().Get_qb().SetElement(0, 0, velocities[i * 6 + 0]);
            bodylist[i]->Variables().Get_qb().SetElement(1, 0, velocities[i * 6 + 1]);
            bodylist[i]->Variables().Get_qb().SetElement(2, 0, velocities[i * 6 + 2]);
            bodylist[i]->Variables().Get_qb().SetElement(3, 0, velocities[i * 6 + 3]);
            bodylist[i]->Variables().Get_qb().SetElement(4, 0, velocities[i * 6 + 4]);
            bodylist[i]->Variables().Get_qb().SetElement(5, 0, velocities[i * 6 + 5]);

            bodylist[i]->VariablesQbIncrementPosition(this->GetStep());
            bodylist[i]->VariablesQbSetSpeed(this->GetStep());

            bodylist[i]->Update(ChTime);

            // update the position and rotation vectors
            pos_pointer[i] = (real3(bodylist[i]->GetPos().x(), bodylist[i]->GetPos().y(), bodylist[i]->GetPos().z()));
            rot_pointer[i] = (quaternion(bodylist[i]->GetRot().e0(), bodylist[i]->GetRot().e1(),
                                         bodylist[i]->GetRot().e2(), bodylist[i]->GetRot().e3()));
        }
    }

    uint offset = data_manager->num_rigid_bodies * 6;
    ////#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_shafts; i++) {
        if (!data_manager->host_data.shaft_active[i])
            continue;

        shaftlist[i]->Variables().Get_qb().SetElementN(0, velocities[offset + i]);
        shaftlist[i]->VariablesQbIncrementPosition(GetStep());
        shaftlist[i]->VariablesQbSetSpeed(GetStep());
        shaftlist[i]->Update(ChTime);
    }

    offset += data_manager->num_shafts;
    for (int i = 0; i < (signed)data_manager->num_linmotors; i++) {
        linmotorlist[i]->Variables().Get_qb().SetElementN(0, velocities[offset + i]);
        linmotorlist[i]->VariablesQbIncrementPosition(GetStep());
        linmotorlist[i]->VariablesQbSetSpeed(GetStep());
        linmotorlist[i]->Update(ChTime, true);
    }

    offset += data_manager->num_linmotors;
    for (int i = 0; i < (signed)data_manager->num_rotmotors; i++) {
        rotmotorlist[i]->Variables().Get_qb().SetElementN(0, velocities[offset + i]);
        rotmotorlist[i]->VariablesQbIncrementPosition(GetStep());
        rotmotorlist[i]->VariablesQbSetSpeed(GetStep());
        rotmotorlist[i]->Update(ChTime, true);
    }

    for (int i = 0; i < otherphysicslist.size(); i++) {
        otherphysicslist[i]->Update(ChTime);
    }

    data_manager->node_container->UpdatePosition(ChTime);
    data_manager->fea_container->UpdatePosition(ChTime);
    data_manager->system_timer.stop("update");

    //=============================================================================================
    ChTime += GetStep();
    data_manager->system_timer.stop("step");
    if (data_manager->settings.perform_thread_tuning) {
        RecomputeThreads();
    }

    return true;
}

//
// Add the specified body to the system.
// A unique identifier is assigned to each body for indexing purposes.
// Space is allocated in system-wide vectors for data corresponding to the
// body.
//

void ChSystemParallel::AddBody(std::shared_ptr<ChBody> newbody) {
    // This is only need because bilaterals need to know what bodies to
    // refer to. Not used by contacts
    newbody->SetId(data_manager->num_rigid_bodies);

    bodylist.push_back(newbody);
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

void ChSystemParallel::AddLink(std::shared_ptr<ChLinkBase> link) {
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

//
// Add physics items, other than bodies or links, to the system.
// We keep track separately of ChShaft elements which are maintained in their
// own list (shaftlist).  All other items are stored in otherphysicslist.
//
// Note that no test is performed to check if the item was already added.
//
// Ideally, the function AddShaft() would be an override of a ChSystem
// virtual function and the vector shaftlist would be maintained by the base
// class ChSystem.  For now, users must use AddOtherPhysicsItem in order to
// properly account for the variables of a shaft elelement in ChSystem::Setup().
//

void ChSystemParallel::AddOtherPhysicsItem(std::shared_ptr<ChPhysicsItem> newitem) {
    if (auto shaft = std::dynamic_pointer_cast<ChShaft>(newitem)) {
        AddShaft(shaft);
    } else {
        newitem->SetSystem(this);
        otherphysicslist.push_back(newitem);

        if (newitem->GetCollide()) {
            newitem->AddCollisionModelsToSystem();
        }
    }
}

//
// Add the specified shaft to the system.
// A unique identifier is assigned to each shaft for indexing purposes.
// Space is allocated in system-wide vectors for data corresponding to the
// shaft.
//
// Currently, this function is private to prevent the user from directly calling
// it and instead force them to use AddOtherPhysicsItem().  See comment above.
// Eventually, this should be an override of a virtual function declared by ChSystem.
//

void ChSystemParallel::AddShaft(std::shared_ptr<ChShaft> shaft) {
    shaft->SetId(data_manager->num_shafts);
    shaft->SetSystem(this);

    shaftlist.push_back(shaft.get());
    data_manager->num_shafts++;

    // Reserve space for this shaft in the system-wide vectors. Not that the
    // actual data is set in UpdateShafts().
    data_manager->host_data.shaft_rot.push_back(0);
    data_manager->host_data.shaft_inr.push_back(0);
    data_manager->host_data.shaft_active.push_back(true);
}

//
// Add a ChMesh to the system
// The mesh is passed to the FEM container where it gets added to the system
// Mesh gets blown up into different data structures, connectivity and nodes are preserved
// Adding multiple meshes isn't a problem
void ChSystemParallel::AddMesh(std::shared_ptr<fea::ChMesh> mesh) {
    uint num_nodes = mesh->GetNnodes();
    uint num_elements = mesh->GetNelements();

    std::vector<real3> positions(num_nodes);
    std::vector<real3> velocities(num_nodes);

    uint current_nodes = data_manager->num_fea_nodes;

    for (int i = 0; i < (signed)num_nodes; i++) {
        if (auto node = std::dynamic_pointer_cast<fea::ChNodeFEAxyz>(mesh->GetNode(i))) {
            positions[i] = real3(node->GetPos().x(), node->GetPos().y(), node->GetPos().z());
            velocities[i] = real3(node->GetPos_dt().x(), node->GetPos_dt().y(), node->GetPos_dt().z());
            // Offset the element index by the current number of nodes at the start
            node->SetIndex(i);

            // printf("%d [%f %f %f]\n", i + current_nodes, node->GetPos().x(), node->GetPos().y(), node->GetPos().z());
        }
    }

    auto container = std::static_pointer_cast<ChFEAContainer>(data_manager->fea_container);

    std::vector<uvec4> elements(num_elements);

    for (int i = 0; i < (signed)num_elements; i++) {
        if (auto tet = std::dynamic_pointer_cast<fea::ChElementTetra_4>(mesh->GetElement(i))) {
            uvec4 elem;

            elem.x = tet->GetNodeN(0)->GetIndex();  //
            elem.y = tet->GetNodeN(1)->GetIndex();  //
            elem.z = tet->GetNodeN(2)->GetIndex();  //
            elem.w = tet->GetNodeN(3)->GetIndex();  //

            real3 c1, c2, c3;
            c1 = positions[elem.y] - positions[elem.x];
            c2 = positions[elem.z] - positions[elem.x];
            c3 = positions[elem.w] - positions[elem.x];

            if (Determinant(Mat33(c1, c2, c3)) < 0) {
                Swap(elem.x, elem.y);
                // printf("swapped!\n");
            }

            // elem = Sort(elem);

            // printf("%d %d %d %d \n", elem.x(), elem.y(), elem.z(), elem.w);
            // Offset once we have swapped
            elem.x += current_nodes;
            elem.y += current_nodes;
            elem.z += current_nodes;
            elem.w += current_nodes;

            elements[i] = elem;
        }
    }
    container->AddNodes(positions, velocities);
    container->AddElements(elements);
}

//
// Reset forces for all variables
//
void ChSystemParallel::ClearForceVariables() {
#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_rigid_bodies; i++) {
        bodylist[i]->VariablesFbReset();
    }

    ////#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_shafts; i++) {
        shaftlist[i]->VariablesFbReset();
    }

    for (int i = 0; i < data_manager->num_linmotors; i++) {
        linmotorlist[i]->VariablesFbReset();
    }

    for (int i = 0; i < data_manager->num_rotmotors; i++) {
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
void ChSystemParallel::Update() {
    LOG(INFO) << "ChSystemParallel::Update()";
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
void ChSystemParallel::UpdateRigidBodies() {
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

        position[i] = real3(body_pos.x(), body_pos.y(), body_pos.z());
        rotation[i] = quaternion(body_rot.e0(), body_rot.e1(), body_rot.e2(), body_rot.e3());

        active[i] = bodylist[i]->IsActive();
        collide[i] = bodylist[i]->GetCollide();

        // Let derived classes set the specific material surface data.
        UpdateMaterialSurfaceData(i, bodylist[i].get());

        bodylist[i]->GetCollisionModel()->SyncPosition();
    }
}

//
// Update all shaft elements in the system and populate system-wide state and
// force vectors. Note that visualization assets are not updated.
//
void ChSystemParallel::UpdateShafts() {
    real* shaft_rot = data_manager->host_data.shaft_rot.data();
    real* shaft_inr = data_manager->host_data.shaft_inr.data();
    char* shaft_active = data_manager->host_data.shaft_active.data();

    ////#pragma omp parallel for
    for (int i = 0; i < (signed)data_manager->num_shafts; i++) {
        shaftlist[i]->Update(ChTime, false);
        shaftlist[i]->VariablesFbLoadForces(GetStep());
        shaftlist[i]->VariablesQbLoadSpeed();

        shaft_rot[i] = shaftlist[i]->GetPos();
        shaft_inr[i] = shaftlist[i]->Variables().GetInvInertia();
        shaft_active[i] = shaftlist[i]->IsActive();

        data_manager->host_data.v[data_manager->num_rigid_bodies * 6 + i] =
            shaftlist[i]->Variables().Get_qb().GetElementN(0);
        data_manager->host_data.hf[data_manager->num_rigid_bodies * 6 + i] =
            shaftlist[i]->Variables().Get_fb().GetElementN(0);
    }
}

//
// Update all motor links that introduce *exactly* one variable.
// TODO: extend this to links with more than one variable.
//
void ChSystemParallel::UpdateMotorLinks() {
    int offset = data_manager->num_rigid_bodies * 6 + data_manager->num_shafts;
    for (int i = 0; i < data_manager->num_linmotors; i++) {
        linmotorlist[i]->Update(ChTime, false);
        linmotorlist[i]->VariablesFbLoadForces(GetStep());
        linmotorlist[i]->VariablesQbLoadSpeed();
        data_manager->host_data.v[offset + i] = linmotorlist[i]->Variables().Get_qb().GetElementN(0);
        data_manager->host_data.hf[offset + i] = linmotorlist[i]->Variables().Get_fb().GetElementN(0);
    }
    offset += data_manager->num_linmotors;
    for (int i = 0; i < data_manager->num_rotmotors; i++) {
        rotmotorlist[i]->Update(ChTime, false);
        rotmotorlist[i]->VariablesFbLoadForces(GetStep());
        rotmotorlist[i]->VariablesQbLoadSpeed();
        data_manager->host_data.v[offset + i] = rotmotorlist[i]->Variables().Get_qb().GetElementN(0);
        data_manager->host_data.hf[offset + i] = rotmotorlist[i]->Variables().Get_fb().GetElementN(0);
    }
}

//
// Update all fluid nodes
// currently a stub
void ChSystemParallel::Update3DOFBodies() {
    data_manager->node_container->Update(ChTime);
    data_manager->fea_container->Update(ChTime);
}

//
// Update all links in the system and set the type of the associated constraints
// to BODY_BODY. Note that visualization assets are not updated.
//
void ChSystemParallel::UpdateLinks() {
    double oostep = 1 / GetStep();
    real clamp_speed = data_manager->settings.solver.bilateral_clamp_speed;
    bool clamp = data_manager->settings.solver.clamp_bilaterals;

    for (int i = 0; i < linklist.size(); i++) {
        linklist[i]->Update(ChTime, false);
        linklist[i]->ConstraintsBiReset();
        linklist[i]->ConstraintsBiLoad_C(oostep, clamp_speed, clamp);
        linklist[i]->ConstraintsBiLoad_Ct(1);
        linklist[i]->ConstraintsFbLoadForces(GetStep());
        linklist[i]->ConstraintsLoadJacobians();

        linklist[i]->InjectConstraints(*descriptor);

        for (int j = 0; j < linklist[i]->GetDOC_c(); j++)
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
void ChSystemParallel::UpdateOtherPhysics() {
    double oostep = 1 / GetStep();
    real clamp_speed = data_manager->settings.solver.bilateral_clamp_speed;
    bool clamp = data_manager->settings.solver.clamp_bilaterals;

    for (int i = 0; i < otherphysicslist.size(); i++) {
        otherphysicslist[i]->Update(ChTime, false);
        otherphysicslist[i]->ConstraintsBiReset();
        otherphysicslist[i]->ConstraintsBiLoad_C(oostep, clamp_speed, clamp);
        otherphysicslist[i]->ConstraintsBiLoad_Ct(1);
        otherphysicslist[i]->ConstraintsFbLoadForces(GetStep());
        otherphysicslist[i]->ConstraintsLoadJacobians();
        otherphysicslist[i]->VariablesFbLoadForces(GetStep());
        otherphysicslist[i]->VariablesQbLoadSpeed();

        BilateralType type = GetBilateralType(otherphysicslist[i].get());

        if (type == BilateralType::UNKNOWN)
            continue;

        otherphysicslist[i]->InjectConstraints(*descriptor);

        for (int j = 0; j < otherphysicslist[i]->GetDOC_c(); j++)
            data_manager->host_data.bilateral_type.push_back(type);
    }
}

//
// Collect indexes of all active bilateral constraints and calculate number of
// non-zero entries in the constraint Jacobian.
//
void ChSystemParallel::UpdateBilaterals() {
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
void ChSystemParallel::Setup() {
    LOG(INFO) << "ChSystemParallel::Setup()";
    // Cache the integration step size and calculate the tolerance at impulse level.
    data_manager->settings.step_size = step;
    data_manager->settings.solver.tol_speed = step * data_manager->settings.solver.tolerance;
    data_manager->settings.gravity = real3(G_acc.x(), G_acc.y(), G_acc.z());

    // Calculate the total number of degrees of freedom (6 per rigid body, 1 per shaft, 1 per motor).
    data_manager->num_dof = data_manager->num_rigid_bodies * 6 + data_manager->num_shafts + data_manager->num_motors +
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
}

void ChSystemParallel::RecomputeThreads() {
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

void ChSystemParallel::ChangeCollisionSystem(CollisionSystemType type) {
    assert(GetNbodies() == 0);

    collision_system_type = type;

    switch (type) {
        case CollisionSystemType::COLLSYS_PARALLEL:
            collision_system = std::make_shared<ChCollisionSystemParallel>(data_manager);
            break;
        case CollisionSystemType::COLLSYS_BULLET_PARALLEL:
            collision_system = std::make_shared<ChCollisionSystemBulletParallel>(data_manager);
            break;
    }
}

void ChSystemParallel::SetLoggingLevel(LoggingLevel level, bool state) {
#ifdef LOGGINGENABLED

    std::string value = state ? "true" : "false";

    switch (level) {
        case LoggingLevel::LOG_NONE:
            el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");
            break;
        case LoggingLevel::LOG_INFO:
            el::Loggers::reconfigureAllLoggers(el::Level::Info, el::ConfigurationType::ToStandardOutput, value);
            break;
        case LoggingLevel::LOG_TRACE:
            el::Loggers::reconfigureAllLoggers(el::Level::Trace, el::ConfigurationType::ToStandardOutput, value);
            break;
        case LoggingLevel::LOG_WARNING:
            el::Loggers::reconfigureAllLoggers(el::Level::Warning, el::ConfigurationType::ToStandardOutput, value);
            break;
        case LoggingLevel::LOG_ERROR:
            el::Loggers::reconfigureAllLoggers(el::Level::Error, el::ConfigurationType::ToStandardOutput, value);
            break;
    }
#endif
}

// Calculate the current body AABB (union of the AABB of their collision shapes).
void ChSystemParallel::CalculateBodyAABB() {
    if (collision_system_type != CollisionSystemType::COLLSYS_PARALLEL)
        return;

    // Readability replacements
    auto& s_min = data_manager->host_data.aabb_min;
    auto& s_max = data_manager->host_data.aabb_max;
    auto& id_rigid = data_manager->shape_data.id_rigid;
    auto& offset = data_manager->measures.collision.global_origin;

    // Initialize body AABB to inverted boxes
    custom_vector<real3> b_min(data_manager->num_rigid_bodies, real3(C_LARGE_REAL));
    custom_vector<real3> b_max(data_manager->num_rigid_bodies, real3(-C_LARGE_REAL));

    // Loop over all shapes and update the AABB of the associated body
    //// TODO: can be done in parallel using Thrust
    for (uint is = 0; is < data_manager->num_rigid_shapes; is++) {
        uint ib = id_rigid[is];
        b_min[ib] = real3(Min(b_min[ib].x, s_min[ib].x + offset.x), Min(b_min[ib].y, s_min[ib].y + offset.y),
                          Min(b_min[ib].z, s_min[ib].z + offset.z));
        b_max[ib] = real3(Max(b_max[ib].x, s_max[ib].x + offset.x), Max(b_max[ib].y, s_max[ib].y + offset.y),
                          Max(b_max[ib].z, s_max[ib].z + offset.z));
    }

    // Loop over all bodies and set the AABB of its collision model
    for (auto b : Get_bodylist()) {
        uint ib = b->GetId();
        std::static_pointer_cast<ChCollisionModelParallel>(b->GetCollisionModel())->aabb_min = b_min[ib];
        std::static_pointer_cast<ChCollisionModelParallel>(b->GetCollisionModel())->aabb_max = b_max[ib];
    }
}

// Calculate the (linearized) bilateral constraint violations and store them in
// the provided vector. Return the maximum constraint violation.
double ChSystemParallel::CalculateConstraintViolation(std::vector<double>& cvec) {
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

void ChSystemParallel::PrintStepStats() {
    data_manager->system_timer.PrintReport();
}

unsigned int ChSystemParallel::GetNumBodies() {
    return data_manager->num_rigid_bodies + data_manager->num_fluid_bodies;
}

unsigned int ChSystemParallel::GetNumShafts() {
    return data_manager->num_shafts;
}

unsigned int ChSystemParallel::GetNumContacts() {
    return data_manager->num_rigid_contacts + data_manager->num_rigid_fluid_contacts + data_manager->num_fluid_contacts;
}

unsigned int ChSystemParallel::GetNumBilaterals() {
    return data_manager->num_bilaterals;
}

// -------------------------------------------------------------

double ChSystemParallel::GetTimerStep() const {
    return data_manager->system_timer.GetTime("step");
}

double ChSystemParallel::GetTimerAdvance() const {
    return data_manager->system_timer.GetTime("advance");
}

double ChSystemParallel::GetTimerUpdate() const {
    return data_manager->system_timer.GetTime("update");
}

double ChSystemParallel::GetTimerSolver() const {
    return data_manager->system_timer.GetTime("ChIterativeSolverParallel_Solve");
}

double ChSystemParallel::GetTimerSetup() const {
    return data_manager->system_timer.GetTime("ChIterativeSolverParallel_Setup");
}

double ChSystemParallel::GetTimerJacobian() const {
    return data_manager->system_timer.GetTime("ChIterativeSolverParallel_Matrices");
}

double ChSystemParallel::GetTimerCollision() const {
    return data_manager->system_timer.GetTime("collision");
}

settings_container* ChSystemParallel::GetSettings() {
    return &(data_manager->settings);
}

// -------------------------------------------------------------

void ChSystemParallel::SetMaterialCompositionStrategy(std::unique_ptr<ChMaterialCompositionStrategy<real>>&& strategy) {
    data_manager->composition_strategy = std::move(strategy);
}

}  // end namespace chrono
