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
// Definitions for all 3DOF type containers for Chrono::Multicore.
//
// =============================================================================

#include <cstdlib>
#include <algorithm>
#include <cmath>

#include "chrono/physics/ChParticleCloud.h"
#include "chrono/assets/ChVisualShapeSphere.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono_multicore/physics/Ch3DOFContainer.h"
#include "chrono_multicore/ChDataManager.h"

namespace chrono {

// -----------------------------------------------------------------------------

class ChMulticoreVisualizationCloud : public ChParticleCloud {
  public:
    ChMulticoreVisualizationCloud(ChMulticoreDataManager* data_manager) : dm(data_manager) {
        EnableCollision(false);
    }
    virtual bool IsActive() const override { return true; }
    virtual size_t GetNumParticles() const override { return dm->num_particles; }
    virtual const ChVector3d& GetParticlePos(unsigned int n) const override {
        const auto& p = dm->host_data.pos_3dof[n];
        tmp = ChVector3d(p.x, p.y, p.z);
        return tmp;
    }
    virtual const ChVector3d& GetParticleVel(unsigned int n) const override {
        const auto& v = dm->host_data.vel_3dof[n];
        tmp = ChVector3d(v.x, v.y, v.z);
        return tmp;
    }
    virtual void Update(double time, bool update_assets) override {}

    ChMulticoreDataManager* dm;
    mutable ChVector3d tmp;
};

// -----------------------------------------------------------------------------

Ch3DOFContainer::Ch3DOFContainer()
    : data_manager(nullptr),
      kernel_radius(.04),
      collision_envelope(0),
      contact_recovery_speed(10),
      contact_cohesion(0),
      contact_compliance(0),
      contact_mu(0),
      max_velocity(20),
      num_particle_contacts(0),
      num_particles(0),
      num_rigid_bodies(0),
      num_rigid_particle_contacts(0),
      num_unilaterals(0),
      num_bilaterals(0),
      num_shafts(0),
      num_motors(0),
      alpha(0) {
    family.x = 1;
    family.y = 0x7FFF;
}

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

real3 Ch3DOFContainer::GetPosDt(int i) {
    return data_manager->host_data.vel_3dof[i];
}
void Ch3DOFContainer::SetPosDt(const int& i, const real3& mposdt) {
    data_manager->host_data.vel_3dof[i] = mposdt;
}

void Ch3DOFContainer::CreateVisualization(double radius, const ChColor& color) {
    // Create the visualization particle cloud
    m_cloud = chrono_types::make_shared<ChMulticoreVisualizationCloud>(data_manager);
    m_cloud->SetName("3DOF_particles");
    auto sphere = chrono_types::make_shared<ChVisualShapeSphere>(radius);
    sphere->SetColor(color);
    m_cloud->AddVisualShape(sphere);

    // Add the visualization cloud to the containing system
    system->Add(m_cloud);
}

void Ch3DOFContainer::Setup3DOF(int start_constraint) {
    start_row = start_constraint;
    if (data_manager) {
        if (data_manager->cd_data) {
            num_particle_contacts = data_manager->cd_data->num_particle_contacts;
            num_rigid_particle_contacts = data_manager->cd_data->num_rigid_particle_contacts;
        } else {
            num_particle_contacts = 0;
            num_rigid_particle_contacts = 0;
        }
        num_particles = data_manager->num_particles;
        num_rigid_bodies = data_manager->num_rigid_bodies;
        num_unilaterals = data_manager->num_unilaterals;
        num_bilaterals = data_manager->num_bilaterals;
        num_shafts = data_manager->num_shafts;
        num_motors = data_manager->num_motors;
    }
}

void Ch3DOFContainer::SetFamily(short mfamily, short mask_no_collision) {
    family.x = (1 << mfamily);
    family.y &= ~(1 << mask_no_collision);
}

}  // end namespace chrono
