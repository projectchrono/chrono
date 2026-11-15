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

ChSystemMulticoreSMC::ChSystemMulticoreSMC(const std::string& name) : ChSystemMulticore(name) {
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

    // The contact history slots of this body (MultiStep tangential displacement model) are allocated in Setup.
}

void ChSystemMulticoreSMC::UpdateMaterialSurfaceData(int index, ChBody* body) {
    data_manager->host_data.mass_rigid[index] = body->GetMass();
}

void ChSystemMulticoreSMC::Setup() {
    // First, invoke the base class method
    ChSystemMulticore::Setup();

    // Ensure that the collision envelope is zero.
    data_manager->settings.collision.collision_envelope = 0;

    // With the MultiStep tangential displacement model, every body has max_shear contact history slots, whenever it
    // was added. Slots added here are empty, and slots in use are never moved, since bodies are only appended.
    // With any other model, discard the stored history, so that a later switch to MultiStep starts with no history.
    auto& hdata = data_manager->host_data;
    if (data_manager->settings.solver.tangential_displ_mode == ChSystemSMC::TangentialDisplacementModel::MultiStep) {
        size_t num_slots = (size_t)max_shear * data_manager->num_rigid_bodies;
        hdata.shear_neigh.resize(num_slots, vec3(-1, -1, -1));
        hdata.shear_disp.resize(num_slots, real3(0, 0, 0));
        hdata.contact_relvel_init.resize(num_slots, 0);
        hdata.contact_duration.resize(num_slots, 0);
    } else if (!hdata.shear_neigh.empty()) {
        hdata.shear_neigh.clear();
        hdata.shear_disp.clear();
        hdata.contact_relvel_init.clear();
        hdata.contact_duration.clear();
    }
}

void ChSystemMulticoreSMC::SetCollisionSystemType(ChCollisionSystem::Type type) {
    ChSystemMulticore::SetCollisionSystemType(type);
    data_manager->settings.collision.collision_envelope = 0;
}

void ChSystemMulticoreSMC::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerMulticoreSMC>(container))
        ChSystem::SetContactContainer(container);
}

real3 ChSystemMulticoreSMC::GetBodyContactForce(std::shared_ptr<ChBody> body) const {
    int index = data_manager->host_data.ct_body_map[body->GetIndex()];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_force[index];
}

real3 ChSystemMulticoreSMC::GetBodyContactTorque(std::shared_ptr<ChBody> body) const {
    int index = data_manager->host_data.ct_body_map[body->GetIndex()];

    if (index == -1)
        return real3(0);

    return data_manager->host_data.ct_body_torque[index];
}

void ChSystemMulticoreSMC::PrintStepStats() {
    double timer_solver_stab = data_manager->system_timer.GetTime("ChIterativeSolverMulticore_Stab");

    std::cout << std::endl;
    std::cout << "System Information" << std::endl;
    std::cout << "------------------" << std::endl;
    std::cout << "  Number of bodies     " << GetNumBodiesActive() << std::endl;
    std::cout << "  Number of contacts   " << GetNumContacts() << std::endl;
    std::cout << "  Number of bilaterals " << GetNumConstraintsUnilateral() << std::endl;
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
