
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

Ch3DOFContainer::Ch3DOFContainer() {
    kernel_radius = .04;
    collision_envelope = 0;
    contact_recovery_speed = 1;
    contact_cohesion = 0;
    contact_mu = 0;
    max_velocity = 20;
}

Ch3DOFContainer::~Ch3DOFContainer() {}

Ch3DOFContainer::Ch3DOFContainer(const Ch3DOFContainer& other) : ChPhysicsItem(other) {
    this->data_manager = other.data_manager;
}

Ch3DOFContainer& Ch3DOFContainer::operator=(const Ch3DOFContainer& other) {
    if (&other == this)
        return *this;

    ChPhysicsItem::operator=(other);
    return *this;
}

real3 Ch3DOFContainer::GetPos(int i) {
    return data_manager->host_data.pos_3dof[i];
}
void Ch3DOFContainer::SetPos(const int& i, const real3& mpos) {
    data_manager->host_data.pos_3dof[i] = mpos;
}

real3 Ch3DOFContainer::GetPos_dt(int i) {
    return data_manager->host_data.vel_3dof[i];
}
void Ch3DOFContainer::SetPos_dt(const int& i, const real3& mposdt) {
    data_manager->host_data.vel_3dof[i] = mposdt;
}

}  // END_OF_NAMESPACE____

/////////////////////
