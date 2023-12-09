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

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/solver/ChIterativeSolverMulticore.h"
#include "chrono_multicore/collision/ChContactContainerMulticoreSMC.h"

using namespace chrono;

ChSystemMulticoreSMC::ChSystemMulticoreSMC() : ChSystemMulticore() {
    contact_container = chrono_types::make_shared<ChContactContainerMulticoreSMC>(data_manager);
    contact_container->SetSystem(this);

    solver = chrono_types::make_shared<ChIterativeSolverMulticoreSMC>(data_manager);

    data_manager->settings.collision.collision_envelope = 0;

    // Set this so that the CD can check what type of system it is (needed for narrowphase)
    data_manager->settings.system_type = SystemType::SYSTEM_SMC;

    data_manager->system_timer.AddTimer("ChIterativeSolverMulticoreSMC_ProcessContact");
}

ChSystemMulticoreSMC::ChSystemMulticoreSMC(const ChSystemMulticoreSMC& other) : ChSystemMulticore(other) {
    //// TODO
}

void ChSystemMulticoreSMC::AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) {
    data_manager->host_data.mass_rigid.push_back(0);

    if (data_manager->settings.solver.tangential_displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
        for (int i = 0; i < max_shear; i++) {
            data_manager->host_data.shear_neigh.push_back(vec3(-1, -1, -1));
            data_manager->host_data.shear_disp.push_back(real3(0, 0, 0));
            data_manager->host_data.contact_relvel_init.push_back(0);
            data_manager->host_data.contact_duration.push_back(0);
        }
    }
}

void ChSystemMulticoreSMC::UpdateMaterialSurfaceData(int index, ChBody* body) {
    data_manager->host_data.mass_rigid[index] = body->GetMass();
}

void ChSystemMulticoreSMC::Setup() {
    // First, invoke the base class method
    ChSystemMulticore::Setup();

    // Ensure that the collision envelope is zero.
    data_manager->settings.collision.collision_envelope = 0;
}

void ChSystemMulticoreSMC::SetCollisionSystemType(ChCollisionSystem::Type type) {
    ChSystemMulticore::SetCollisionSystemType(type);
    data_manager->settings.collision.collision_envelope = 0;
}

void ChSystemMulticoreSMC::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerMulticoreSMC>(container))
        ChSystem::SetContactContainer(container);
}

real3 ChSystemMulticoreSMC::GetBodyContactForce(uint body_id) const {
    int index = data_manager->host_data.ct_body_map[body_id];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_force[index];
}

real3 ChSystemMulticoreSMC::GetBodyContactTorque(uint body_id) const {
    int index = data_manager->host_data.ct_body_map[body_id];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_torque[index];
}

void ChSystemMulticoreSMC::PrintStepStats() {
    double timer_solver_stab = data_manager->system_timer.GetTime("ChIterativeSolverMulticore_Stab");

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
    std::cout << "    LS setup           " << GetTimerLSsetup() << std::endl;
    std::cout << "    LS solve           " << GetTimerLSsolve() << std::endl;
    std::cout << "    stabilization      " << timer_solver_stab << std::endl;
    std::cout << std::endl;
}
