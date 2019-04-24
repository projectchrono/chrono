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
// Authors: Radu Serban, Hammad Mazhar
// =============================================================================

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

using namespace chrono;
using namespace chrono::collision;

ChSystemParallelSMC::ChSystemParallelSMC() : ChSystemParallel() {
    solver_speed = std::make_shared<ChIterativeSolverParallelSMC>(data_manager);

    data_manager->settings.collision.collision_envelope = 0;

    // Set this so that the CD can check what type of system it is (needed for narrowphase)
    data_manager->settings.system_type = SystemType::SYSTEM_SMC;

    data_manager->system_timer.AddTimer("ChIterativeSolverParallelSMC_ProcessContact");
}

ChSystemParallelSMC::ChSystemParallelSMC(const ChSystemParallelSMC& other) : ChSystemParallel(other) {
    //// TODO
}

ChBody* ChSystemParallelSMC::NewBody() {
    if (collision_system_type == CollisionSystemType::COLLSYS_PARALLEL)
        return new ChBody(std::make_shared<collision::ChCollisionModelParallel>(), ChMaterialSurface::SMC);

    return new ChBody(ChMaterialSurface::SMC);
}

ChBodyAuxRef* ChSystemParallelSMC::NewBodyAuxRef() {
    if (collision_system_type == CollisionSystemType::COLLSYS_PARALLEL)
        return new ChBodyAuxRef(std::make_shared<collision::ChCollisionModelParallel>(), ChMaterialSurface::SMC);

    return new ChBodyAuxRef(ChMaterialSurface::SMC);
}

void ChSystemParallelSMC::AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) {
    assert(newbody->GetContactMethod() == ChMaterialSurface::SMC);

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
        data_manager->host_data.smc_coeffs.push_back(real4(0, 0, 0, 0));
    }

    if (data_manager->settings.solver.tangential_displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
        for (int i = 0; i < max_shear; i++) {
            data_manager->host_data.shear_neigh.push_back(vec3(-1, -1, -1));
            data_manager->host_data.shear_disp.push_back(real3(0, 0, 0));
        }
    }
}

void ChSystemParallelSMC::UpdateMaterialSurfaceData(int index, ChBody* body) {
    custom_vector<real>& mass = data_manager->host_data.mass_rigid;
    custom_vector<real2>& elastic_moduli = data_manager->host_data.elastic_moduli;
    custom_vector<real>& adhesion = data_manager->host_data.cohesion_data;
    custom_vector<real>& adhesionMult = data_manager->host_data.adhesionMultDMT_data;
    custom_vector<real>& mu = data_manager->host_data.mu;
    custom_vector<real>& cr = data_manager->host_data.cr;
    custom_vector<real4>& smc_coeffs = data_manager->host_data.smc_coeffs;

    // Since this function is called in a parallel for loop, we must access the
    // material properties in a thread-safe manner (we cannot use the function
    // ChBody::GetMaterialSurfaceSMC since that returns a copy of the reference
    // counted shared pointer).
    std::shared_ptr<ChMaterialSurface>& mat = body->GetMaterialSurface();
    ChMaterialSurfaceSMC* mat_ptr = static_cast<ChMaterialSurfaceSMC*>(mat.get());

    mass[index] = body->GetMass();
    mu[index] = mat_ptr->GetSfriction();
    adhesion[index] = mat_ptr->GetAdhesion();
    adhesionMult[index] = mat_ptr->GetAdhesionMultDMT();

    if (data_manager->settings.solver.use_material_properties) {
        elastic_moduli[index] = real2(mat_ptr->GetYoungModulus(), mat_ptr->GetPoissonRatio());
        cr[index] = mat_ptr->GetRestitution();
    } else {
        smc_coeffs[index] = real4(mat_ptr->GetKn(), mat_ptr->GetKt(), mat_ptr->GetGn(), mat_ptr->GetGt());
    }
}

void ChSystemParallelSMC::Setup() {
    // First, invoke the base class method
    ChSystemParallel::Setup();

    // Ensure that the collision envelope is zero.
    data_manager->settings.collision.collision_envelope = 0;
}

void ChSystemParallelSMC::ChangeCollisionSystem(CollisionSystemType type) {
    ChSystemParallel::ChangeCollisionSystem(type);
    data_manager->settings.collision.collision_envelope = 0;
}

real3 ChSystemParallelSMC::GetBodyContactForce(uint body_id) const {
    int index = data_manager->host_data.ct_body_map[body_id];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_force[index];
}

real3 ChSystemParallelSMC::GetBodyContactTorque(uint body_id) const {
    int index = data_manager->host_data.ct_body_map[body_id];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_torque[index];
}

void ChSystemParallelSMC::PrintStepStats() {
    double timer_solver_stab = data_manager->system_timer.GetTime("ChIterativeSolverParallel_Stab");

    std::cout << std::endl;
    std::cout << "System Information" << std::endl;
    std::cout << "------------------" << std::endl;
    std::cout << "  Number of bodies     " << GetNumBodies() << std::endl;
    std::cout << "  Number of contacts   " << GetNumContacts() << std::endl;
    std::cout << "  Number of bilaterals " << GetNumBilaterals() << std::endl;
    std::cout << std::endl;
    std::cout << "Timing Information" << std::endl;
    std::cout << "------------------" << std::endl;
    std::cout << "Simulation time        " << GetTimerStep() << std::endl;
    std::cout << "  Collision detection  " << GetTimerCollision() << std::endl;
    std::cout << "    broad phase        " << GetTimerCollisionBroad() << std::endl;
    std::cout << "    narrow phase       " << GetTimerCollisionNarrow() << std::endl;
    std::cout << "  Update               " << GetTimerUpdate() << std::endl;
    std::cout << "  Advance              " << GetTimerAdvance() << std::endl;
    std::cout << "    contact force calc " << GetTimerProcessContact() << std::endl;
    std::cout << "    setup              " << GetTimerSetup() << std::endl;
    std::cout << "    solve              " << GetTimerSolver() << std::endl;
    std::cout << "    stabilization      " << timer_solver_stab << std::endl;
    std::cout << std::endl;
}
