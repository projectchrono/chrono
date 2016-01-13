
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE

ChMPMContainer::ChMPMContainer(ChSystemParallelMPM* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->Add3DOFContainer(this);
}
ChMPMContainer::~ChMPMContainer() {}

void ChMPMContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

    pos_fluid.insert(pos_fluid.end(), positions.begin(), positions.end());
    vel_fluid.insert(vel_fluid.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_fluid.resize(pos_fluid.size());
    data_manager->num_fluid_bodies = pos_fluid.size();
}
void ChMPMContainer::Update(double ChTime) {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;
    real3 g_acc = data_manager->settings.gravity;
    real3 h_gravity = data_manager->settings.step_size * data_manager->settings.fluid.mass * g_acc;
    for (int i = 0; i < num_fluid_bodies; i++) {
        // This was moved to after fluid collision detection
        // real3 vel = vel_fluid[i];
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = vel.x;
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = vel.y;
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = vel.z;

        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = h_gravity.z;
    }
}

void ChMPMContainer::UpdatePosition(double ChTime) {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;

    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

    for (int i = 0; i < num_fluid_bodies; i++) {
        real3 vel;
        int original_index = data_manager->host_data.particle_indices_3dof[i];
        // these are sorted so we have to unsort them
        vel.x = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0];
        vel.y = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1];
        vel.z = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2];

        real speed = Length(vel);
        if (speed > data_manager->settings.fluid.max_velocity) {
            vel = vel * data_manager->settings.fluid.max_velocity / speed;
        }
        vel_fluid[original_index] = vel;
        pos_fluid[original_index] += vel * data_manager->settings.step_size;
    }
}
void ChMPMContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;

    real inv_mass = 1.0 / data_manager->settings.mpm.mass;
    for (int i = 0; i < num_fluid_bodies; i++) {
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
        M_inv.finalize(offset + i * 3 + 2);
    }
}
void ChMPMContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;

    real mpm_mass = data_manager->settings.mpm.mass;
    for (int i = 0; i < num_fluid_bodies; i++) {
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, mpm_mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, mpm_mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, mpm_mass);
        M.finalize(offset + i * 3 + 2);
    }
}
}  // END_OF_NAMESPACE____

/////////////////////
