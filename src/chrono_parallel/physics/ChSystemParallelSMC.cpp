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
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono_parallel/collision/ChContactContainerParallelSMC.h"
#include "chrono_parallel/ChParallelDefines.h"
#include <math.h> 

using namespace chrono;
using namespace chrono::collision;

ChSystemParallelSMC::ChSystemParallelSMC() : ChSystemParallel() {
    contact_container = chrono_types::make_shared<ChContactContainerParallelSMC>(data_manager);
    contact_container->SetSystem(this);

    solver = chrono_types::make_shared<ChIterativeSolverParallelSMC>(data_manager);

    data_manager->settings.collision.collision_envelope = 0;

    // Set this so that the CD can check what type of system it is (needed for narrowphase)
    data_manager->settings.system_type = SystemType::SYSTEM_SMC;

    data_manager->system_timer.AddTimer("ChIterativeSolverParallelSMC_ProcessContact");
}

ChSystemParallelSMC::ChSystemParallelSMC(const ChSystemParallelSMC& other) : ChSystemParallel(other) {
    //// TODO
}



void ChSystemParallelSMC::AddMaterialSurfaceData(std::shared_ptr<ChBody> newbody) {
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

void ChSystemParallelSMC::UpdateMaterialSurfaceData(int index, ChBody* body) {
    data_manager->host_data.mass_rigid[index] = body->GetMass();
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
    std::cout<<"size:"<<data_manager->host_data.ct_body_map.size()<<std::endl;
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

bool ChSystemParallelSMC::Par_Gran_Outhelper(ChSystemParallelSMC* system, const std::string& filename) {
    // Create the CSV stream.
    chrono::utils::CSV_writer csv(" ");
    std::string tab("    ");

    csv << "id,mass,pos_x,pos_y,pos_z,v_x,v_y,v_z,fx,fy,fz,KE"<<std::endl;
    for (auto body : system->Get_bodylist()) {
        ChVector<double>& pos = body->GetPos();
        ChVector<double>& pos_dt = body->GetPos_dt();

        real3 cont_f;
        int index = data_manager->host_data.ct_body_map[body->GetId()];
        if (index == -1)
            cont_f = real3(0);
        else
            cont_f = data_manager->host_data.ct_body_force[index];
        csv<<body->GetId()<<","<<body->GetMass()<<","<<(float)pos.x()<<","<<(float)pos.y()<<","<< (float)pos.z()<<","<<(float)pos_dt.x()<<","<<(float)pos_dt.y()<<","<<(float)pos_dt.z()<<","<<(double)cont_f.x<<","<<(double)cont_f.y<<","<<(double)(cont_f.z)<<",";
        csv<<body->GetMass()*0.5*((float)pos_dt.x()*(float)pos_dt.x()+(float)pos_dt.y()*(float)pos_dt.y()+(float)pos_dt.z()*(float)pos_dt.z());
        csv<<std::endl;
        
    }
    csv.write_to_file(filename);

    return true;
}


float ChSystemParallelSMC::GetTotKineticEnergy(ChSystemParallelSMC* system) {
    // Create the CSV stream.
    float total_KE = 0;

    for (auto body : system->Get_bodylist()) {
        ChVector<double>& pos_dt = body->GetPos_dt();

        float indiv_KE = body->GetMass()*0.5*((float)pos_dt.x()*(float)pos_dt.x()+(float)pos_dt.y()*(float)pos_dt.y()+(float)pos_dt.z()*(float)pos_dt.z());

        total_KE = total_KE+indiv_KE;
    }

    return total_KE;
}

bool ChSystemParallelSMC::Par_Gran_Cont_Outhelper(ChSystemParallelSMC* system, const std::string& filename) {
    // Create the CSV stream.
    std::shared_ptr<ChIterativeSolverParallelSMC> solver_1 = std::dynamic_pointer_cast<ChIterativeSolverParallelSMC>(solver);
    custom_vector<int> ext_body_id=solver_1->get_ext_body_id();
    custom_vector<real3> ext_body_force=solver_1->get_ext_body_force();
     
    chrono::utils::CSV_writer csv(" ");
    std::string tab("    ");

    csv << "id_1,id_2,contact_x,contact_y,contact_z,contact_mag"<<std::endl;

    for(int i = 0; i<ext_body_force.size()/2;i++){
        csv<<ext_body_id[i*2]<<","<<ext_body_id[i*2+1]<<","<< (float)ext_body_force[i*2].x<<","<<(float)ext_body_force[i*2].y<<","<<(float)ext_body_force[i*2].z<<",";
        double n_mag = (float)ext_body_force[i*2].x * (float)ext_body_force[i*2].x + (float)ext_body_force[i*2].y * (float)ext_body_force[i*2].y + (float)ext_body_force[i*2].z * (float)ext_body_force[i*2].z;
        n_mag = sqrt(n_mag);
        csv<<n_mag<<std::endl;
    }

    csv.write_to_file(filename);

    return true;
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
    std::cout << "    LS setup           " << GetTimerLSsetup() << std::endl;
    std::cout << "    LS solve           " << GetTimerLSsolve() << std::endl;
    std::cout << "    stabilization      " << timer_solver_stab << std::endl;
    std::cout << std::endl;
}
