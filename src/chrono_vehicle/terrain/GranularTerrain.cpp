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
//
// TODO:
//   - re-enable collision envelope for lateral boundaries.
//   - currently disabled due to a bug in Chrono::Parallel where cohesion forces
//     are applied even when the penetration depth is positive!
//   - as a result, if envelope is considered, particles stick to the lateral
//     boundaries...
//   - for now, also make sure the envelope is not too large:  it's still used
//     for the bottom boundary and, if present, for the collision with the
//     ground-fixed spheres
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
// Note: this should be at least 3, to accommodate the case of rough surface.
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
      m_moved(false),
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
void GranularTerrain::EnableRoughSurface(int num_spheres_x, int num_spheres_y) {
    m_nx = num_spheres_x;
    m_ny = num_spheres_y;

    // Enable rough surface
    m_rough_surface = true;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void GranularTerrain::EnableMovingPatch(std::shared_ptr<ChBody> body,
                                        double buffer_distance,
                                        double shift_distance,
                                        const ChVector<>& init_vel) {
    m_body = body;
    m_buffer_distance = buffer_distance;
    m_shift_distance = shift_distance;
    m_init_part_vel = init_vel;

    // Enable moving patch
    m_moving_patch = true;
}

// -----------------------------------------------------------------------------
// Custom collision callback
// -----------------------------------------------------------------------------
class BoundaryContact : public ChSystem::CustomCollisionCallback {
  public:
    BoundaryContact(GranularTerrain* terrain) : m_terrain(terrain), m_radius(terrain->m_radius) {}
    virtual void OnCustomCollision(ChSystem* system) override;

  private:
    void CheckBottom(ChBody* body, const ChVector<>& center);
    void CheckLeft(ChBody* body, const ChVector<>& center);
    void CheckRight(ChBody* body, const ChVector<>& center);
    void CheckFront(ChBody* body, const ChVector<>& center);
    void CheckRear(ChBody* body, const ChVector<>& center);

    void CheckFixedSpheres(ChBody* body, const ChVector<>& center);
    void CheckFixedSphere(ChBody* body, const ChVector<>& center, const ChVector<>& s_center);

    GranularTerrain* m_terrain;
    double m_radius;
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
            if (m_terrain->m_rough_surface)
                CheckFixedSpheres(body, center);
        }
    }
}

// Check contact between granular material and ground-fixed spheres.
void BoundaryContact::CheckFixedSpheres(ChBody* body, const ChVector<>& center) {
    // No contact if particle above layer of fixed spheres
    double dist = center.z() - m_terrain->m_bottom;
    if (dist > 3 * m_radius)
        return;

    // Current patch length and width
    double length = m_terrain->m_front - m_terrain->m_rear;
    double width = m_terrain->m_left - m_terrain->m_right;

    // Identify 4 potential collisions with fixed spheres
    double x_pos = center.x() - m_terrain->m_rear;
    double y_pos = center.y() - m_terrain->m_right;
    int ix = (int)std::floor(x_pos / m_terrain->m_sep_x);
    int iy = (int)std::floor(y_pos / m_terrain->m_sep_y);

    double x_m = m_terrain->m_rear + ix * m_terrain->m_sep_x;
    double x_p = x_m + m_terrain->m_sep_x;
    double y_m = m_terrain->m_right + iy * m_terrain->m_sep_y;
    double y_p = y_m + m_terrain->m_sep_y;

    ////double eps = 1e-6;
    ////if (center.x() < x_m - eps || center.x() > x_p + eps) {
    ////    std::cout << "X problem  " << center.x() << " " << x_m << " " << x_p << std::endl;
    ////}
    ////if (center.y() < y_m - eps || center.y() > y_p + eps) {
    ////    std::cout << "Y problem  " << center.y() << " " << y_m << " " << y_p << std::endl;
    ////}

    // Check potential collisions
    CheckFixedSphere(body, center, ChVector<>(x_m, y_m, m_terrain->m_bottom + m_radius));
    CheckFixedSphere(body, center, ChVector<>(x_m, y_p, m_terrain->m_bottom + m_radius));
    CheckFixedSphere(body, center, ChVector<>(x_p, y_m, m_terrain->m_bottom + m_radius));
    CheckFixedSphere(body, center, ChVector<>(x_p, y_p, m_terrain->m_bottom + m_radius));
}

// Check collision between the specified particle and a ground-fixed sphere centered at s_center.
void BoundaryContact::CheckFixedSphere(ChBody* body, const ChVector<>& center, const ChVector<>& s_center) {
    ChVector<> delta = center - s_center;
    double dist2 = delta.Length2();
    double rad_sum = 2 * m_radius + 2 * m_terrain->m_envelope;

    if (dist2 >= rad_sum * rad_sum || dist2 < 1e-12)
        return;

    collision::ChCollisionInfo contact;
    double dist = std::sqrt(dist2);
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = delta / dist;
    contact.vpA = s_center + contact.vN * m_radius;
    contact.vpB = center - contact.vN * m_radius;
    contact.distance = dist - 2 * m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

// Check contact between granular material and bottom boundary.
void BoundaryContact::CheckBottom(ChBody* body, const ChVector<>& center) {
    double dist = center.z() - m_terrain->m_bottom;

    if (dist > m_radius + 2 * m_terrain->m_envelope)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(0, 0, 1);
    contact.vpA = ChVector<>(center.x(), center.y(), m_terrain->m_bottom);
    contact.vpB = ChVector<>(center.x(), center.y(), center.z() - m_radius);
    contact.distance = dist - m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

// Check contact between granular material and left boundary.
void BoundaryContact::CheckLeft(ChBody* body, const ChVector<>& center) {
    double dist = m_terrain->m_left - center.y();

    ////if (dist > m_radius + 2 * m_terrain->m_envelope)
    if (dist > m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(0, -1, 0);
    contact.vpA = ChVector<>(center.x(), m_terrain->m_left, center.z());
    contact.vpB = ChVector<>(center.x(), center.y() + m_radius, center.z());
    contact.distance = dist - m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

// Check contact between granular material and right boundary.
void BoundaryContact::CheckRight(ChBody* body, const ChVector<>& center) {
    double dist = center.y() - m_terrain->m_right;

    ////if (dist > m_radius + 2 * m_terrain->m_envelope)
    if (dist > m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(0, 1, 0);
    contact.vpA = ChVector<>(center.x(), m_terrain->m_right, center.z());
    contact.vpB = ChVector<>(center.x(), center.y() - m_radius, center.z());
    contact.distance = dist - m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

// Check contact between granular material and front boundary.
void BoundaryContact::CheckFront(ChBody* body, const ChVector<>& center) {
    double dist = m_terrain->m_front - center.x();

    ////if (dist > m_radius + 2 * m_terrain->m_envelope)
    if (dist > m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(-1, 0, 0);
    contact.vpA = ChVector<>(m_terrain->m_front, center.y(), center.z());
    contact.vpB = ChVector<>(center.x() + m_radius, center.y(), center.z());
    contact.distance = dist - m_radius;

    body->GetSystem()->GetContactContainer()->AddContact(contact);
}

// Check contact between granular material and rear boundary.
void BoundaryContact::CheckRear(ChBody* body, const ChVector<>& center) {
    double dist = center.x() - m_terrain->m_rear;

    ////if (dist > m_radius + 2 * m_terrain->m_envelope)
    if (dist > m_radius)
        return;

    collision::ChCollisionInfo contact;
    contact.modelA = m_terrain->m_ground->GetCollisionModel().get();
    contact.modelB = body->GetCollisionModel().get();
    contact.vN = ChVector<>(1, 0, 0);
    contact.vpA = ChVector<>(m_terrain->m_rear, center.y(), center.z());
    contact.vpB = ChVector<>(center.x() - m_radius, center.y(), center.z());
    contact.distance = dist - m_radius;

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
                                 double density,
                                 const ChVector<>& init_vel) {
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
        m_sep_x = length / (m_nx - 1);
        m_sep_y = width / (m_ny - 1);
        double x_pos = -0.5 * length;
        for (int ix = 0; ix < m_nx; ix++) {
            double y_pos = -0.5 * width;
            for (int iy = 0; iy < m_ny; iy++) {
                auto sphere = std::make_shared<ChSphereShape>();
                sphere->GetSphereGeometry().rad = radius;
                sphere->Pos = ChVector<>(x_pos, y_pos, radius);
                m_ground->AddAsset(sphere);

                ////auto box = std::make_shared<ChBoxShape>();
                ////double hside = radius / std::sqrt(2.0);
                ////box->GetBoxGeometry().Size = ChVector<>(hside, hside, hside);
                ////box->Pos = ChVector<>(x_pos, y_pos, radius);
                ////m_ground->AddAsset(box);

                y_pos += m_sep_y;
            }
            x_pos += m_sep_x;
        }

        if (m_verbose) {
            std::cout << "Enable rough surface." << std::endl;
            std::cout << "   X direction (" << m_nx << ") separation: " << m_sep_x << std::endl;
            std::cout << "   Y direction (" << m_ny << ") separation: " << m_sep_y << std::endl;
        }
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
            m_envelope = 0;  // collision envelope reset to 0
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
        generator.createObjectsBox(utils::POISSON_DISK, 2 * r, layer_center, layer_hdims, init_vel);
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
    m_moved = false;

    if (!m_moving_patch)
        return;

    // Check distance from monitored body to front boundary.
    double dist = m_front - m_body->GetFrame_REF_to_abs().GetPos().x();
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
            body->SetPos_dt(m_init_part_vel);
        }
    }

    // Shift front boundary.
    m_front += m_shift_distance;

    m_moved = true;

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

float GranularTerrain::GetCoefficientFriction(double x, double y) const {
    return m_friction_fun ? (*m_friction_fun)(x, y) : m_friction;
}

}  // end namespace vehicle
}  // end namespace chrono
