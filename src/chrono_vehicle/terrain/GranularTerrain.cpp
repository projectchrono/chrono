// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Granular terrain model.
// This class implements a rectangular patch of granular terrain.
// Optionally, a moving patch feature can be enable so that the patch is
// relocated (currently only in the positive X direction) based on the position
// of a user-specified body.
// Boundary conditions (model of a container bin) are imposed through a custom
// collision detection object.
//
// Reference frame is ISO (X forward, Y left, Z up).
// All units SI.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/GranularTerrain.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// Safety inflation factor for inter-particle initial separation.
const double safety_factor = 1.001;

// Offset particles from bottom boundary by offset_factor * radius.
const double offset_factor = 3;

// -----------------------------------------------------------------------------
// Default constructor.
// -----------------------------------------------------------------------------
GranularTerrain::GranularTerrain(ChSystem* system)
    : m_start_id(1000000),
      m_num_particles(0),
      m_rough_surface(false),
      m_vis_enabled(false),
      m_moving_patch(false),
      m_friction(0.9f),
      m_restitution(0.0f),
      m_cohesion(0.0f),
      m_young_modulus(2e5f),
      m_poisson_ratio(0.3f),
      m_kn(2e5f),
      m_gn(40),
      m_kt(2e5f),
      m_gt(20) {
    // Create the ground body and add it to the system.
    m_ground = std::shared_ptr<ChBody>(system->NewBody());
    m_ground->SetName("ground");
    m_ground->SetPos(ChVector<>(0, 0, 0));
    m_ground->SetBodyFixed(true);
    m_ground->SetCollide(false);
    system->AddBody(m_ground);

    // Create the default color asset
    m_color = std::make_shared<ChColorAsset>();
    m_color->SetColor(ChColor(1, 1, 1));
    m_ground->AddAsset(m_color);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GranularTerrain::SetContactMaterialProperties(float young_modulus, float poisson_ratio) {
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

void GranularTerrain::SetContactMaterialCoefficients(float kn, float gn, float kt, float gt) {
    m_kn = kn;
    m_gn = gn;
    m_kt = kt;
    m_gt = gt;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GranularTerrain::EnableMovingPatch(std::shared_ptr<ChBody> body, double buffer_distance, double shift_distance) {
    m_body = body;
    m_buffer_distance = buffer_distance;
    m_shift_distance = shift_distance;

    // Enable moving patch
    m_moving_patch = true;
}

// -----------------------------------------------------------------------------
// Custom collision callback
// -----------------------------------------------------------------------------
class BoundaryContact : public ChSystem::CustomCollisionCallback {
  public:
    BoundaryContact(GranularTerrain* terrain) : m_terrain(terrain) {}
    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    void CheckBottom(ChBody* body, const ChVector<>& center);
    void CheckLeft(ChBody* body, const ChVector<>& center);
    void CheckRight(ChBody* body, const ChVector<>& center);
    void CheckFront(ChBody* body, const ChVector<>& center);
    void CheckRear(ChBody* body, const ChVector<>& center);

    GranularTerrain* m_terrain;
};

void BoundaryContact::OnCustomCollision(ChSystem* system) {
    auto bodylist = system->Get_bodylist();
    for (size_t i = 0; i < bodylist->size(); ++i) {
        auto body = (*bodylist)[i].get();
        auto center = body->GetPos();
        if (body->GetIdentifier() > m_terrain->m_start_id) {
            CheckBottom(body, center);
            CheckLeft(body, center);
            CheckRight(body, center);
            CheckFront(body, center);
            CheckRear(body, center);
        }
    }
}

void BoundaryContact::CheckBottom(ChBody* body, const ChVector<>& center) {
    double dist = center.z() - m_terrain->m_bottom;

    if (dist > m_terrain->m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(0, 0, 1);
    contact.vpA = ChVector<>(center.x(), center.y(), m_terrain->m_bottom);
    contact.vpB = ChVector<>(center.x(), center.y(), center.z() - m_terrain->m_radius);
    contact.distance = dist - m_terrain->m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

void BoundaryContact::CheckLeft(ChBody* body, const ChVector<>& center) {
    double dist = m_terrain->m_left - center.y();

    if (dist > m_terrain->m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(0, -1, 0);
    contact.vpA = ChVector<>(center.x(), m_terrain->m_left, center.z());
    contact.vpB = ChVector<>(center.x(), center.y() + m_terrain->m_radius, center.z());
    contact.distance = dist - m_terrain->m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

void BoundaryContact::CheckRight(ChBody* body, const ChVector<>& center) {
    double dist = center.y() - m_terrain->m_right;

    if (dist > m_terrain->m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(0, 1, 0);
    contact.vpA = ChVector<>(center.x(), m_terrain->m_right, center.z());
    contact.vpB = ChVector<>(center.x(), center.y() - m_terrain->m_radius, center.z());
    contact.distance = dist - m_terrain->m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

void BoundaryContact::CheckFront(ChBody* body, const ChVector<>& center) {
    double dist = m_terrain->m_front - center.x();

    if (dist > m_terrain->m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(-1, 0, 0);
    contact.vpA = ChVector<>(m_terrain->m_front, center.y(), center.z());
    contact.vpB = ChVector<>(center.x() + m_terrain->m_radius, center.y(), center.z());
    contact.distance = dist - m_terrain->m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

void BoundaryContact::CheckRear(ChBody* body, const ChVector<>& center) {
    double dist = center.x() - m_terrain->m_rear;

    if (dist > m_terrain->m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(1, 0, 0);
    contact.vpA = ChVector<>(m_terrain->m_rear, center.y(), center.z());
    contact.vpB = ChVector<>(center.x() - m_terrain->m_radius, center.y(), center.z());
    contact.distance = dist - m_terrain->m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

// -----------------------------------------------------------------------------
// Initialize the granular terrain patch
// -----------------------------------------------------------------------------
void GranularTerrain::Initialize(const ChVector<>& center,
                                 double length,
                                 double width,
                                 unsigned int num_particles,
                                 double radius,
                                 double density) {
    m_length = length;
    m_width = width;
    m_radius = radius;

    // Set boundary locations.
    m_front = center.x() + length / 2;
    m_rear = center.x() - length / 2;
    m_left = center.y() + width / 2;
    m_right = center.y() - width / 2;
    m_bottom = center.z();

    // Move the ground body at patch location.
    m_ground->SetPos(center);

    // If enabled, create particles fixed to ground.
    if (m_rough_surface) {
        m_ground->GetCollisionModel()->ClearModel();
        double d = 4 * radius;
        int nx = (int)std::floor(0.5 * length / d);
        int ny = (int)std::floor(0.5 * width / d);
        for (int ix = -nx; ix <= nx; ix++) {
            for (int iy = -ny; iy <= ny; iy++) {
                utils::AddSphereGeometry(m_ground.get(), radius, ChVector<>(ix * d, iy * d, radius));
            }
        }
        m_ground->GetCollisionModel()->BuildModel();
        m_ground->SetCollide(true);
    }

    // Create the contact material.
    std::shared_ptr<ChMaterialSurface> mat;

    // Set contact material properties for the ground (container) body.
    switch (m_ground->GetContactMethod()) {
        case ChMaterialSurface::NSC: {
            auto matNSC = std::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(m_friction);
            matNSC->SetRestitution(m_restitution);
            matNSC->SetCohesion(m_cohesion);
            mat = matNSC;
            break;
        }
        case ChMaterialSurface::SMC: {
            auto matSMC = std::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(m_friction);
            matSMC->SetRestitution(m_restitution);
            matSMC->SetYoungModulus(m_young_modulus);
            matSMC->SetAdhesion(m_cohesion);
            matSMC->SetPoissonRatio(m_poisson_ratio);
            matSMC->SetKn(m_kn);
            matSMC->SetGn(m_gn);
            matSMC->SetKt(m_kt);
            matSMC->SetGt(m_gt);
            mat = matSMC;
            break;
        }
    }

    // Set contact material for ground body.
    m_ground->SetMaterialSurface(mat);

    // Set the ground body identifier.
    m_ground->SetIdentifier(m_start_id);

    // Create a particle generator and a mixture entirely made out of spheres.
    // Set the starting value for particle body identifiers.
    utils::Generator generator(m_ground->GetSystem());
    generator.setBodyIdentifier(m_start_id + 1);
    std::shared_ptr<utils::MixtureIngredient> m1 = generator.AddMixtureIngredient(utils::SPHERE, 1.0);
    m1->setDefaultMaterial(mat);
    m1->setDefaultDensity(density);
    m1->setDefaultSize(radius);

    // Create particles, in layers, until exceeding the specified number.
    double r = safety_factor * radius;
    ChVector<> layer_hdims(length / 2 - r, width / 2 - r, 0);
    ChVector<> layer_center = center;
    layer_center.z() += offset_factor * r;

    while (m_num_particles < num_particles) {
        if (m_verbose)
            std::cout << "Create layer at height: " << layer_center.z() << std::endl;
        generator.createObjectsBox(utils::POISSON_DISK, 2 * r, layer_center, layer_hdims);
        layer_center.z() += 2 * r;
        m_num_particles = generator.getTotalNumBodies();
    }

    // If enabled, create visualization assets for the boundaries.
    if (m_vis_enabled) {
        auto box = std::make_shared<ChBoxShape>();
        double hthick = 0.05;
        box->GetBoxGeometry().Size = ChVector<>(length / 2, width / 2, hthick);
        box->Pos = ChVector<>(0, 0, -hthick);
        m_ground->AddAsset(box);
    }

    // Register the custom collision callback for boundary conditions.
    auto cb = new BoundaryContact(this);
    m_ground->GetSystem()->RegisterCustomCollisionCallback(cb);
}

void GranularTerrain::Synchronize(double time) {
    if (!m_moving_patch)
        return;

    // Check distance from monitored body to front boundary.
    double dist = m_front - m_body->GetPos().x();
    if (dist >= m_buffer_distance)
        return;

    // Shift ground body.
    m_ground->SetPos(m_ground->GetPos() + ChVector<>(m_shift_distance, 0, 0));

    // Shift rear boundary.
    m_rear += m_shift_distance;

    // Count particles that must be relocated.
    unsigned int num_moved_particles = 0;
    auto bodylist = m_ground->GetSystem()->Get_bodylist();
    for (auto body : *bodylist) {
        if (body->GetIdentifier() > m_start_id && body->GetPos().x() - m_radius < m_rear) {
            num_moved_particles++;
        }
    }

    // Create a Poisson Disk sampler and generate points in layers within the relocation volume.
    std::vector<ChVector<>> new_points;
    double r = safety_factor * m_radius;
    utils::PDSampler<> sampler(2 * r);
    ChVector<> layer_hdims(m_shift_distance / 2 - r, m_width / 2 - r, 0);
    ChVector<> layer_center(m_front + m_shift_distance / 2, (m_left + m_right) / 2, m_bottom + offset_factor * r);
    while (new_points.size() < num_moved_particles) {
        auto points = sampler.SampleBox(layer_center, layer_hdims);
        new_points.insert(new_points.end(), points.begin(), points.end());
        layer_center.z() += 2 * r;
    }

    // Relocate particles at their new locations.
    size_t ip = 0;
    for (auto body : *bodylist) {
        if (body->GetIdentifier() > m_start_id && body->GetPos().x() - m_radius < m_rear) {
            body->SetPos(new_points[ip++]);
        }
    }

    // Shift front boundary.
    m_front += m_shift_distance;

    if (m_verbose) {
        std::cout << "Move patch at time " << time << std::endl;
        std::cout << "   moved " << num_moved_particles << " particles" << std::endl;
        std::cout << "   rear: " << m_rear << "  front: " << m_front << std::endl;
    }
}

double GranularTerrain::GetHeight(double x, double y) const {
    auto bodylist = m_ground->GetSystem()->Get_bodylist();
    double highest = m_bottom;
    for (auto body : *bodylist) {
        if (body->GetIdentifier() > m_start_id && body->GetPos().z() > highest)
            highest = body->GetPos().z();
    }
    return highest + m_radius;
}

}  // end namespace vehicle
}  // end namespace chrono
