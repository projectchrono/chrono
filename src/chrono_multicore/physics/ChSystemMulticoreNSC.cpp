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

#include "chrono_multicore/ChConfigMulticore.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChSolverMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"
#include "chrono_multicore/collision/ChContactContainerMulticoreNSC.h"
#include "chrono_multicore/collision/ChCollisionSystemBulletMulticore.h"

using namespace chrono;

ChSystemMulticoreNSC::ChSystemMulticoreNSC() : ChSystemMulticore() {
    contact_container = chrono_types::make_shared<ChContactContainerMulticoreNSC>(data_manager);
    contact_container->SetSystem(this);

    solver = chrono_types::make_shared<ChIterativeSolverMulticoreNSC>(data_manager);

    // Set this so that the CD can check what type of system it is (needed for narrowphase)
    data_manager->settings.system_type = SystemType::SYSTEM_NSC;

    data_manager->system_timer.AddTimer("ChSolverMulticore_solverA");
    data_manager->system_timer.AddTimer("ChSolverMulticore_solverB");
    data_manager->system_timer.AddTimer("ChSolverMulticore_solverC");
    data_manager->system_timer.AddTimer("ChSolverMulticore_solverD");
    data_manager->system_timer.AddTimer("ChSolverMulticore_solverE");
    data_manager->system_timer.AddTimer("ChSolverMulticore_solverF");
    data_manager->system_timer.AddTimer("ChSolverMulticore_solverG");
    data_manager->system_timer.AddTimer("ChSolverMulticore_Project");
    data_manager->system_timer.AddTimer("ChSolverMulticore_Solve");
    data_manager->system_timer.AddTimer("ShurProduct");
    data_manager->system_timer.AddTimer("ChIterativeSolverMulticore_D");
    data_manager->system_timer.AddTimer("ChIterativeSolverMulticore_E");
    data_manager->system_timer.AddTimer("ChIterativeSolverMulticore_R");
    data_manager->system_timer.AddTimer("ChIterativeSolverMulticore_N");
}

ChSystemMulticoreNSC::ChSystemMulticoreNSC(const ChSystemMulticoreNSC& other) : ChSystemMulticore(other) {
    //// TODO
}

void ChSystemMulticoreNSC::ChangeSolverType(SolverType type) {
    std::static_pointer_cast<ChIterativeSolverMulticoreNSC>(solver)->ChangeSolverType(type);
}

void ChSystemMulticoreNSC::Add3DOFContainer(std::shared_ptr<Ch3DOFContainer> container) {
    data_manager->node_container = container;

    //// TODO: remove this
    ////data_manager->cd_data->p_kernel_radius = container->kernel_radius;
    ////data_manager->cd_data->p_collision_envelope = container->collision_envelope;
    ////data_manager->cd_data->p_collision_family = container->family;

    container->SetSystem(this);
    container->data_manager = data_manager;
}

void ChSystemMulticoreNSC::SetContactContainer(collision::ChCollisionSystemType type) {
    contact_container = chrono_types::make_shared<ChContactContainerMulticoreNSC>(data_manager);
    contact_container->SetSystem(this);
}

void ChSystemMulticoreNSC::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerMulticoreNSC>(container))
        ChSystem::SetContactContainer(container);
}

void ChSystemMulticoreNSC::AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) {
    // Reserve space for material properties for the specified body.
    // Notes:
    //  - the actual data is set in UpdateMaterialProperties()
    //  - coefficients of sliding friction are only needed for fluid-rigid and FEA-rigid contacts;
    //    for now, we store a single value per body (corresponding to the first collision shape,
    //    if any, in the associated collision model)
    data_manager->host_data.sliding_friction.push_back(0);
    data_manager->host_data.cohesion.push_back(0);
}

void ChSystemMulticoreNSC::UpdateMaterialSurfaceData(int index, ChBody* body) {
    custom_vector<float>& friction = data_manager->host_data.sliding_friction;
    custom_vector<float>& cohesion = data_manager->host_data.cohesion;

    if (body->GetCollisionModel() && body->GetCollisionModel()->GetNumShapes() > 0) {
        auto mat =
            std::static_pointer_cast<ChMaterialSurfaceNSC>(body->GetCollisionModel()->GetShape(0)->GetMaterial());
        friction[index] = mat->GetKfriction();
        cohesion[index] = mat->GetCohesion();
    }
}

void ChSystemMulticoreNSC::CalculateContactForces() {
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_rigid_dof = data_manager->num_rigid_bodies * 6;
    uint num_contacts = data_manager->cd_data->num_rigid_contacts;
    DynamicVector<real>& Fc = data_manager->host_data.Fc;

    data_manager->Fc_current = true;

    if (num_contacts == 0) {
        Fc.resize(6 * data_manager->num_rigid_bodies);
        Fc = 0;
        return;
    }

    const SubMatrixType& D_u = blaze::submatrix(data_manager->host_data.D, 0, 0, num_rigid_dof, num_unilaterals);
    DynamicVector<real> gamma_u = blaze::subvector(data_manager->host_data.gamma, 0, num_unilaterals);
    Fc = D_u * gamma_u / data_manager->settings.step_size;
}

real3 ChSystemMulticoreNSC::GetBodyContactForce(uint body_id) const {
    assert(data_manager->Fc_current);
    return real3(data_manager->host_data.Fc[body_id * 6 + 0], data_manager->host_data.Fc[body_id * 6 + 1],
                 data_manager->host_data.Fc[body_id * 6 + 2]);
}

real3 ChSystemMulticoreNSC::GetBodyContactTorque(uint body_id) const {
    assert(data_manager->Fc_current);
    return real3(data_manager->host_data.Fc[body_id * 6 + 3], data_manager->host_data.Fc[body_id * 6 + 4],
                 data_manager->host_data.Fc[body_id * 6 + 5]);
}

static inline chrono::ChVector<real> ToChVector(const real3& a) {
    return chrono::ChVector<real>(a.x, a.y, a.z);
}

void ChSystemMulticoreNSC::SolveSystem() {
    data_manager->system_timer.Reset();
    data_manager->system_timer.start("step");

    Setup();

    data_manager->system_timer.start("update");
    Update();
    data_manager->system_timer.stop("update");

    data_manager->system_timer.start("collision");
    collision_system->Run();
    collision_system->ReportContacts(this->contact_container.get());
    data_manager->system_timer.stop("collision");
    data_manager->system_timer.start("advance");
    std::static_pointer_cast<ChIterativeSolverMulticoreNSC>(solver)->RunTimeStep();
    data_manager->system_timer.stop("advance");
    data_manager->system_timer.stop("step");
}

void ChSystemMulticoreNSC::AssembleSystem() {
    //// TODO: load colliding shape information in icontact? Not really needed here.

    Setup();

    collision_system->Run();
    collision_system->ReportContacts(contact_container.get());
    ChSystem::Update();
    contact_container->BeginAddContact();
    chrono::collision::ChCollisionInfo icontact;
    for (int i = 0; i < (signed)data_manager->cd_data->num_rigid_contacts; i++) {
        vec2 cd_pair = data_manager->cd_data->bids_rigid_rigid[i];
        icontact.modelA = Get_bodylist()[cd_pair.x]->GetCollisionModel().get();
        icontact.modelB = Get_bodylist()[cd_pair.y]->GetCollisionModel().get();
        icontact.vN = ToChVector(data_manager->cd_data->norm_rigid_rigid[i]);
        icontact.vpA =
            ToChVector(data_manager->cd_data->cpta_rigid_rigid[i] + data_manager->host_data.pos_rigid[cd_pair.x]);
        icontact.vpB =
            ToChVector(data_manager->cd_data->cptb_rigid_rigid[i] + data_manager->host_data.pos_rigid[cd_pair.y]);
        icontact.distance = data_manager->cd_data->dpth_rigid_rigid[i];
        icontact.eff_radius = data_manager->cd_data->erad_rigid_rigid[i];
        contact_container->AddContact(icontact);
    }
    contact_container->EndAddContact();

    // Reset sparse representation accumulators.
    for (auto& link : Get_linklist()) {
        link->ConstraintsBiReset();
    }
    for (auto& body : Get_bodylist()) {
        body->VariablesFbReset();
    }
    contact_container->ConstraintsBiReset();

    // Fill in the sparse system representation by looping over all links, bodies,
    // and other physics items.
    double F_factor = step;
    double K_factor = step * step;
    double R_factor = step;
    double M_factor = 1;
    double Ct_factor = 1;
    double C_factor = 1 / step;

    for (auto& link : Get_linklist()) {
        link->ConstraintsBiLoad_C(C_factor, max_penetration_recovery_speed, true);
        link->ConstraintsBiLoad_Ct(Ct_factor);
        link->VariablesQbLoadSpeed();
        link->VariablesFbIncrementMq();
        link->ConstraintsLoadJacobians();
        link->ConstraintsFbLoadForces(F_factor);
    }

    for (int ip = 0; ip < Get_bodylist().size(); ++ip) {
        std::shared_ptr<ChBody> Bpointer = Get_bodylist()[ip];

        Bpointer->VariablesFbLoadForces(F_factor);
        Bpointer->VariablesQbLoadSpeed();
        Bpointer->VariablesFbIncrementMq();
    }

    for (auto& item : Get_otherphysicslist()) {
        item->VariablesFbLoadForces(F_factor);
        item->VariablesQbLoadSpeed();
        item->VariablesFbIncrementMq();
        item->ConstraintsBiLoad_C(C_factor, max_penetration_recovery_speed, true);
        item->ConstraintsBiLoad_Ct(Ct_factor);
        item->ConstraintsLoadJacobians();
        item->KRMmatricesLoad(K_factor, R_factor, M_factor);
        item->ConstraintsFbLoadForces(F_factor);
    }

    contact_container->ConstraintsBiLoad_C(C_factor, max_penetration_recovery_speed, true);
    contact_container->ConstraintsFbLoadForces(F_factor);
    contact_container->ConstraintsLoadJacobians();

    // Inject all variables and constraints into the system descriptor.
    descriptor->BeginInsertion();
    for (auto& body : Get_bodylist()) {
        body->InjectVariables(*descriptor);
    }
    for (auto& link : Get_linklist()) {
        link->InjectConstraints(*descriptor);
    }
    contact_container->InjectConstraints(*descriptor);
    descriptor->EndInsertion();
}

void ChSystemMulticoreNSC::Initialize() {
    // Mpm update is special because it computes the number of nodes that we have
    // data_manager->node_container->ComputeDOF();

    Setup();

    data_manager->system_timer.start("update");
    Update();
    data_manager->system_timer.stop("update");

    data_manager->system_timer.start("collision");
    collision_system->PreProcess();
    collision_system->Run();
    collision_system->PostProcess();
    collision_system->ReportContacts(this->contact_container.get());
    data_manager->system_timer.stop("collision");

    data_manager->node_container->Initialize();
}
