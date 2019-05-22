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

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChSolverParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono_parallel/collision/ChCollisionSystemParallel.h"
#include "chrono_parallel/collision/ChCollisionSystemBulletParallel.h"

using namespace chrono;

ChSystemParallelNSC::ChSystemParallelNSC() : ChSystemParallel() {
    solver_speed = std::make_shared<ChIterativeSolverParallelNSC>(data_manager);

    // Set this so that the CD can check what type of system it is (needed for narrowphase)
    data_manager->settings.system_type = SystemType::SYSTEM_NSC;

    data_manager->system_timer.AddTimer("ChSolverParallel_solverA");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverB");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverC");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverD");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverE");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverF");
    data_manager->system_timer.AddTimer("ChSolverParallel_solverG");
    data_manager->system_timer.AddTimer("ChSolverParallel_Project");
    data_manager->system_timer.AddTimer("ChSolverParallel_Solve");
    data_manager->system_timer.AddTimer("ShurProduct");
    data_manager->system_timer.AddTimer("ChIterativeSolverParallel_D");
    data_manager->system_timer.AddTimer("ChIterativeSolverParallel_E");
    data_manager->system_timer.AddTimer("ChIterativeSolverParallel_R");
    data_manager->system_timer.AddTimer("ChIterativeSolverParallel_N");
}

ChSystemParallelNSC::ChSystemParallelNSC(const ChSystemParallelNSC& other) : ChSystemParallel(other) {
    //// TODO
}

ChBody* ChSystemParallelNSC::NewBody() {
    if (collision_system_type == CollisionSystemType::COLLSYS_PARALLEL)
        return new ChBody(std::make_shared<collision::ChCollisionModelParallel>(), ChMaterialSurface::NSC);

    return new ChBody(ChMaterialSurface::NSC);
}

void ChSystemParallelNSC::ChangeSolverType(SolverType type) {
    std::static_pointer_cast<ChIterativeSolverParallelNSC>(solver_speed)->ChangeSolverType(type);
}

ChBodyAuxRef* ChSystemParallelNSC::NewBodyAuxRef() {
    if (collision_system_type == CollisionSystemType::COLLSYS_PARALLEL)
        return new ChBodyAuxRef(std::make_shared<collision::ChCollisionModelParallel>(), ChMaterialSurface::NSC);

    return new ChBodyAuxRef(ChMaterialSurface::NSC);
}

void ChSystemParallelNSC::Add3DOFContainer(std::shared_ptr<Ch3DOFContainer> container) {
    if (auto fea_container = std::dynamic_pointer_cast<ChFEAContainer>(container)) {
        data_manager->fea_container = fea_container;
    } else {
        data_manager->node_container = container;
    }

    container->SetSystem(this);
    container->data_manager = data_manager;
}

void ChSystemParallelNSC::AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) {
    assert(newbody->GetContactMethod() == ChMaterialSurface::NSC);

    // Reserve space for material properties for the specified body. Not that the
    // actual data is set in UpdateMaterialProperties().
    data_manager->host_data.fric_data.push_back(real3(0));
    data_manager->host_data.cohesion_data.push_back(0);
    data_manager->host_data.compliance_data.push_back(real4(0));
}

void ChSystemParallelNSC::UpdateMaterialSurfaceData(int index, ChBody* body) {
    custom_vector<real>& cohesion = data_manager->host_data.cohesion_data;
    custom_vector<real3>& friction = data_manager->host_data.fric_data;
    custom_vector<real4>& compliance = data_manager->host_data.compliance_data;

    // Since this function is called in a parallel for loop, we must access the
    // material properties in a thread-safe manner (we cannot use the function
    // ChBody::GetMaterialSurfaceNSC since that returns a copy of the reference
    // counted shared pointer).
    std::shared_ptr<ChMaterialSurface>& mat = body->GetMaterialSurface();
    ChMaterialSurfaceNSC* mat_ptr = static_cast<ChMaterialSurfaceNSC*>(mat.get());

    friction[index] = real3(mat_ptr->GetKfriction(), mat_ptr->GetRollingFriction(), mat_ptr->GetSpinningFriction());
    cohesion[index] = mat_ptr->GetCohesion();
    compliance[index] = real4(mat_ptr->GetCompliance(), mat_ptr->GetComplianceT(), mat_ptr->GetComplianceRolling(),
                              mat_ptr->GetComplianceSpinning());
}

void ChSystemParallelNSC::CalculateContactForces() {
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_rigid_dof = data_manager->num_rigid_bodies * 6;
    uint num_contacts = data_manager->num_rigid_contacts;
    DynamicVector<real>& Fc = data_manager->host_data.Fc;

    data_manager->Fc_current = true;

    if (num_contacts == 0) {
        Fc.resize(6 * data_manager->num_rigid_bodies);
        Fc = 0;
        return;
    }

    LOG(INFO) << "ChSystemParallelNSC::CalculateContactForces() ";

    const SubMatrixType& D_u = blaze::submatrix(data_manager->host_data.D, 0, 0, num_rigid_dof, num_unilaterals);
    DynamicVector<real> gamma_u = blaze::subvector(data_manager->host_data.gamma, 0, num_unilaterals);
    Fc = D_u * gamma_u / data_manager->settings.step_size;
}

real3 ChSystemParallelNSC::GetBodyContactForce(uint body_id) const {
    assert(data_manager->Fc_current);
    return real3(data_manager->host_data.Fc[body_id * 6 + 0], data_manager->host_data.Fc[body_id * 6 + 1],
                 data_manager->host_data.Fc[body_id * 6 + 2]);
}

real3 ChSystemParallelNSC::GetBodyContactTorque(uint body_id) const {
    assert(data_manager->Fc_current);
    return real3(data_manager->host_data.Fc[body_id * 6 + 3], data_manager->host_data.Fc[body_id * 6 + 4],
                 data_manager->host_data.Fc[body_id * 6 + 5]);
}

static inline chrono::ChVector<real> ToChVector(const real3& a) {
    return chrono::ChVector<real>(a.x, a.y, a.z);
}

void ChSystemParallelNSC::SolveSystem() {
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
    std::static_pointer_cast<ChIterativeSolverParallelNSC>(solver_speed)->RunTimeStep();
    data_manager->system_timer.stop("advance");
    data_manager->system_timer.stop("step");
}

void ChSystemParallelNSC::AssembleSystem() {
    Setup();

    collision_system->Run();
    collision_system->ReportContacts(contact_container.get());
    ChSystem::Update();
    contact_container->BeginAddContact();
    chrono::collision::ChCollisionInfo icontact;
    for (int i = 0; i < (signed)data_manager->num_rigid_contacts; i++) {
        vec2 cd_pair = data_manager->host_data.bids_rigid_rigid[i];
        icontact.modelA = bodylist[cd_pair.x]->GetCollisionModel().get();
        icontact.modelB = bodylist[cd_pair.y]->GetCollisionModel().get();
        icontact.vN = ToChVector(data_manager->host_data.norm_rigid_rigid[i]);
        icontact.vpA =
            ToChVector(data_manager->host_data.cpta_rigid_rigid[i] + data_manager->host_data.pos_rigid[cd_pair.x]);
        icontact.vpB =
            ToChVector(data_manager->host_data.cptb_rigid_rigid[i] + data_manager->host_data.pos_rigid[cd_pair.y]);
        icontact.distance = data_manager->host_data.dpth_rigid_rigid[i];
        icontact.eff_radius = data_manager->host_data.erad_rigid_rigid[i];
        contact_container->AddContact(icontact);
    }
    contact_container->EndAddContact();

    // Reset sparse representation accumulators.
    for (int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->ConstraintsBiReset();
    }
    for (int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->VariablesFbReset();
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

    for (int ip = 0; ip < linklist.size(); ++ip) {
        std::shared_ptr<ChLinkBase> Lpointer = linklist[ip];

        Lpointer->ConstraintsBiLoad_C(C_factor, max_penetration_recovery_speed, true);
        Lpointer->ConstraintsBiLoad_Ct(Ct_factor);
        Lpointer->VariablesQbLoadSpeed();
        Lpointer->VariablesFbIncrementMq();
        Lpointer->ConstraintsLoadJacobians();
        Lpointer->ConstraintsFbLoadForces(F_factor);
    }

    for (int ip = 0; ip < bodylist.size(); ++ip) {
        std::shared_ptr<ChBody> Bpointer = bodylist[ip];

        Bpointer->VariablesFbLoadForces(F_factor);
        Bpointer->VariablesQbLoadSpeed();
        Bpointer->VariablesFbIncrementMq();
    }

    for (int ip = 0; ip < otherphysicslist.size(); ++ip) {
        std::shared_ptr<ChPhysicsItem> PHpointer = otherphysicslist[ip];

        PHpointer->VariablesFbLoadForces(F_factor);
        PHpointer->VariablesQbLoadSpeed();
        PHpointer->VariablesFbIncrementMq();
        PHpointer->ConstraintsBiLoad_C(C_factor, max_penetration_recovery_speed, true);
        PHpointer->ConstraintsBiLoad_Ct(Ct_factor);
        PHpointer->ConstraintsLoadJacobians();
        PHpointer->KRMmatricesLoad(K_factor, R_factor, M_factor);
        PHpointer->ConstraintsFbLoadForces(F_factor);
    }

    contact_container->ConstraintsBiLoad_C(C_factor, max_penetration_recovery_speed, true);
    contact_container->ConstraintsFbLoadForces(F_factor);
    contact_container->ConstraintsLoadJacobians();

    // Inject all variables and constraints into the system descriptor.
    descriptor->BeginInsertion();
    for (int ip = 0; ip < bodylist.size(); ++ip) {
        bodylist[ip]->InjectVariables(*descriptor);
    }
    for (int ip = 0; ip < linklist.size(); ++ip) {
        linklist[ip]->InjectConstraints(*descriptor);
    }
    contact_container->InjectConstraints(*descriptor);
    descriptor->EndInsertion();
}

void ChSystemParallelNSC::Initialize() {
    // Mpm update is special because it computes the number of nodes that we have
    // data_manager->node_container->ComputeDOF();

    Setup();

    data_manager->fea_container->Initialize();

    data_manager->system_timer.start("update");
    Update();
    data_manager->system_timer.stop("update");

    data_manager->system_timer.start("collision");
    collision_system->Run();
    collision_system->ReportContacts(this->contact_container.get());
    data_manager->system_timer.stop("collision");

    data_manager->node_container->Initialize();
}
