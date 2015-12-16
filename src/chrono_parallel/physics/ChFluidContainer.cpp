
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/ChFluidContainer.h>

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE

ChFluidContainer::ChFluidContainer(ChSystemParallelDVI* physics_system) {
    system = physics_system;
    system->AddFluid(this);
}

ChFluidContainer::~ChFluidContainer() {}

ChFluidContainer::ChFluidContainer(const ChFluidContainer& other) : ChPhysicsItem(other) {
    this->system = other.system;
}

ChFluidContainer& ChFluidContainer::operator=(const ChFluidContainer& other) {
    if (&other == this)
        return *this;

    ChPhysicsItem::operator=(other);
    return *this;
}

void ChFluidContainer::AddFluid(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_fluid = system->data_manager->host_data.pos_fluid;
    custom_vector<real3>& vel_fluid = system->data_manager->host_data.vel_fluid;

    pos_fluid.insert(pos_fluid.end(), positions.begin(), positions.end());
    vel_fluid.insert(vel_fluid.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_fluid.resize(pos_fluid.size());
    system->data_manager->num_fluid_bodies = pos_fluid.size();
}
void ChFluidContainer::Update(double ChTime) {
    uint num_fluid_bodies = system->data_manager->num_fluid_bodies;
    uint num_rigid_bodies = system->data_manager->num_rigid_bodies;
    uint num_shafts = system->data_manager->num_shafts;
    custom_vector<real3>& pos_fluid = system->data_manager->host_data.pos_fluid;
    custom_vector<real3>& vel_fluid = system->data_manager->host_data.vel_fluid;
    ChVector<> g_acc = system->Get_G_acc();
    real3 h_gravity = system->GetStep() * system->data_manager->settings.fluid.mass * real3(g_acc.x, g_acc.y, g_acc.z);
    for (int i = 0; i < num_fluid_bodies; i++) {
        real3 vel = vel_fluid[i];
        system->data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = vel.x;
        system->data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = vel.y;
        system->data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = vel.z;

        system->data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = h_gravity.x;
        system->data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = h_gravity.y;
        system->data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = h_gravity.z;
    }
}

void ChFluidContainer::UpdatePosition(double ChTime) {
    uint num_fluid_bodies = system->data_manager->num_fluid_bodies;
    uint num_rigid_bodies = system->data_manager->num_rigid_bodies;
    uint num_shafts = system->data_manager->num_shafts;

    custom_vector<real3>& pos_fluid = system->data_manager->host_data.pos_fluid;
    custom_vector<real3>& vel_fluid = system->data_manager->host_data.vel_fluid;

    for (int i = 0; i < num_fluid_bodies; i++) {
        real3 vel;
        vel.x = system->data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0];
        vel.y = system->data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1];
        vel.z = system->data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2];

        real speed = Length(vel);
        if (speed > system->data_manager->settings.fluid.max_velocity) {
            vel = vel * system->data_manager->settings.fluid.max_velocity / speed;
        }
        vel_fluid[i] = vel;
        pos_fluid[i] += vel * system->GetStep();
    }
}

real3 ChFluidContainer::GetPos(int i) {
    return system->data_manager->host_data.pos_fluid[i];
}
void ChFluidContainer::SetPos(const int& i, const real3& mpos) {
    system->data_manager->host_data.pos_fluid[i] = mpos;
}

real3 ChFluidContainer::GetPos_dt(int i) {
    return system->data_manager->host_data.vel_fluid[i];
}
void ChFluidContainer::SetPos_dt(const int& i, const real3& mposdt) {
    system->data_manager->host_data.vel_fluid[i] = mposdt;
}

}  // END_OF_NAMESPACE____

/////////////////////
