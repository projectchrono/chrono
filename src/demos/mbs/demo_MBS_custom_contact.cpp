// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Demo illustrating custom contact generation between two bodies.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/assets/ChVisualSystem.h"
#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

using namespace chrono;

// -----------------------------------------------------------------------------

// Example of a class for overriding the internal Chrono contact force generation.
// This class can use one of 3 different approaches:
//    ADD_ALL_CONTACTS:
//       Add contacts (to the system's contact container), one for each identified pair of collision points.
//       NOTES: - this is equivalent to what Chrono would do automatically
//    ADD_AVERAGE_CONTACT:
//       Create and add (to the system's contact container) a single custom contact, averaging all identified
//       pairs of collision points.
//       NOTES: - this is *not* a robust method
//              - used here for *illustration* purposes only
//              - for simplicity, use contact materials from the first shape in each collision model
//    ADD_CONTACT_LOADS
//       Calculate and add contact forces (as external loads) for each identified pair of collision points.
//       NOTES: - for illustration only, here we only generate normal contact forces
//              - use a Hertz force model
//
// In all cases, use the NarrowphaseCallback interface (implemented in OnNarrowphase) to intercept all collisions
// identified during the narrowphase of the collision detection phase. Cache the collisions involving the specified
// bodies and indicate that Chrono should not generate contact forces for them (return 'false' from OnNarrowphase).
// For all other collisions, let Chrono generate contacts (return 'true' from OnNarrowphase).
//
// ADD_ALL_CONTACTS and ADD_AVERAGE_CONTACT use the CustomCollisionCallback interface (implemented in OnCustomCollision)
// to process the cached collisions in order to add to the system's contact container one or more contacts. Chrono will
// then calculate and apply contact loads based on these contacts.
//
// ADD_CONTACT_LOADS uses the ChLoadContainer interface (implemented in Setup) to process the cached collisions in order
// to calculate contact forces and add them as loads (in a load container). Chrono will then add these as external loads
// to the two bodies.

class CustomContact : public ChCollisionSystem::NarrowphaseCallback,  // intercept collisions
                      public ChSystem::CustomCollisionCallback,       // generate contacts
                      public ChLoadContainer                          // apply contact loads
{
  public:
    enum class Method { ADD_ALL_CONTACTS, ADD_AVERAGE_CONTACT, ADD_CONTACT_LOADS };

    CustomContact(std::shared_ptr<ChBody> body1, std::shared_ptr<ChBody> body2, Method method)
        : body1(body1), body2(body2), method(method) {}

    // Clear the list of cached collisions.
    // Must be called at each step, before integration.
    void Reset() { collisions.clear(); }

    // Get the number of cached collisions at the current step.
    size_t GetNumCollisions() const { return collisions.size(); }

    // Implement NarrowphaseCallback interface.
    // This function is called during collision detection for each identified pair of collision points.
    virtual bool OnNarrowphase(ChCollisionInfo& cinfo) override {
        // If this is a collision between the given bodies, cache it and indicate that no contacts should be generated
        if ((body1.get() == cinfo.modelA->GetContactable() && body2.get() == cinfo.modelB->GetContactable()) ||
            (body1.get() == cinfo.modelB->GetContactable() && body2.get() == cinfo.modelA->GetContactable())) {
            if (cinfo.distance < 0) {
                collisions.push_back(cinfo);
            }
            return false;
        }
        // Let Chrono deal with all other collisions
        return true;
    }

    // Implement CustomCollisionCallback interface
    virtual void OnCustomCollision(ChSystem* sys) override {
        // Process current list of collisions between the tagged bodies
        if (collisions.empty())
            return;

        // Generate contacts between tagged bodies:
        //

        switch (method) {
            case Method::ADD_ALL_CONTACTS: {
                // Add a contact for each detected collision pair
                for (const auto& collision : collisions)
                    sys->GetContactContainer()->AddContact(collision);

                break;
            }

            case Method::ADD_AVERAGE_CONTACT: {
                // Get contact materials
                auto matA = collisions[0].shapeA->GetMaterial();
                auto matB = collisions[0].shapeB->GetMaterial();

                // Calculate one equivalent collision (average everything)
                auto num_collisions = collisions.size();

                ChCollisionInfo cinfo_avg;
                cinfo_avg.modelA = collisions[0].modelA;
                cinfo_avg.modelB = collisions[0].modelB;
                cinfo_avg.shapeA = nullptr;
                cinfo_avg.shapeB = nullptr;

                cinfo_avg.vpA = VNULL;
                cinfo_avg.vpB = VNULL;
                cinfo_avg.vN = VNULL;
                cinfo_avg.distance = 0;

                for (const auto& cinfo : collisions) {
                    assert(cinfo.distance < 0);

                    if (body1.get() == cinfo.modelA->GetContactable()) {
                        cinfo_avg.vpA += cinfo.vpA;
                        cinfo_avg.vpB += cinfo.vpB;
                        cinfo_avg.vN += cinfo.vN;
                    } else {
                        cinfo_avg.vpA += cinfo.vpB;
                        cinfo_avg.vpB += cinfo.vpA;
                        cinfo_avg.vN -= cinfo.vN;
                    }
                    cinfo_avg.distance += cinfo.distance;
                }

                cinfo_avg.distance /= num_collisions;
                cinfo_avg.vpA /= num_collisions;
                cinfo_avg.vpB /= num_collisions;
                cinfo_avg.vN /= num_collisions;
                cinfo_avg.vN.Normalize();

                // Add a single equivalent contact
                sys->GetContactContainer()->AddContact(cinfo_avg, matA, matB);

                break;
            }
        }
    }

    // Implement ChLoadContainer interface.
    // The Setup function is called at each step, *after* collision detection.
    virtual void Setup() override {
        if (method != Method::ADD_CONTACT_LOADS)
            return;

        // Reset the load list for this load container
        GetLoadList().clear();

        // Composite material properties
        double E_eff = 2e6;
        double G_eff = 2e6;
        double mu_eff = 0.8;
        double cr_eff = 0.01;
        double eff_radius = 0.5;

        // Generate contact forces between the two bodies using the cached collision information
        for (const auto& cinfo : collisions) {
            // Extract collision information
            auto delta = -cinfo.distance;
            auto normal_dir = cinfo.vN;
            auto p1 = cinfo.vpA;
            auto p2 = cinfo.vpB;
            auto objA = cinfo.modelA->GetContactable();
            auto objB = cinfo.modelB->GetContactable();
            auto vel1 = objA->GetContactPointSpeed(p1);
            auto vel2 = objB->GetContactPointSpeed(p2);

            // Calculate relative normal and tangential velocities
            ChVector3d relvel = vel2 - vel1;
            double relvel_n_mag = relvel.Dot(normal_dir);
            ChVector3d relvel_n = relvel_n_mag * normal_dir;
            ChVector3d relvel_t = relvel - relvel_n;
            double relvel_t_mag = relvel_t.Length();

            // Hertz contact
            double eff_mass = objA->GetContactableMass() * objB->GetContactableMass() /
                              (objA->GetContactableMass() + objB->GetContactableMass());
            double sqrt_Rd = std::sqrt(eff_radius * delta);
            double Sn = 2 * E_eff * sqrt_Rd;
            double St = 8 * G_eff * sqrt_Rd;
            double loge = std::log(cr_eff);
            double beta = loge / std::sqrt(loge * loge + CH_PI * CH_PI);
            double kn = CH_2_3 * Sn;
            double kt = St;
            double gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);
            double gt = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(St * eff_mass);

            // Tangential displacement
            auto delta_t = relvel_t_mag * GetChTime();

            // Normal and tangential contact forces
            double forceN = kn * delta - gn * relvel_n_mag;
            double forceT = kt * delta_t + gt * relvel_t_mag;

            // If the resulting normal contact force is negative, the two shapes are moving
            // away from each other so fast that no contact force is generated.
            if (forceN < 0)
                return;

            // Coulomb law
            forceT = std::min<double>(forceT, mu_eff * std::abs(forceN));

            // Accumulate normal and tangential forces
            ChVector3d force = forceN * normal_dir;
            if (relvel_t_mag >= 1e-4)
                force -= (forceT / relvel_t_mag) * relvel_t;

            // Add contact forces to load container
            Add(chrono_types::make_shared<ChLoadBodyForce>(body1, -force, false, cinfo.vpA, false));
            Add(chrono_types::make_shared<ChLoadBodyForce>(body2, +force, false, cinfo.vpA, false));
        }

        // Perform a full update of the load container
        ChLoadContainer::Update(ChTime, UpdateFlags::UPDATE_ALL);
    }

  private:
    Method method;
    std::shared_ptr<ChBody> body1;
    std::shared_ptr<ChBody> body2;
    std::vector<ChCollisionInfo> collisions;
};

// -----------------------------------------------------------------------------

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2026 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create the system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create a material (will be used by both collision shapes)
    ChContactMaterialData mat_data;
    mat_data.Y = 2e6;
    mat_data.mu = 0.4f;
    mat_data.cr = 0.1f;
    auto mat = mat_data.CreateMaterial(ChContactMethod::SMC);

    // Custom body tags
    int tag_ball = 1;
    int tag_plate = 2;

    // Create the mesh object
    double radius = 1;
    double mass = 300;
    ChMatrix33d inertia;
    auto sphere = utils::ChBodyGeometry::SphereShape(VNULL, radius, 0);
    inertia = mass * ChSphere::CalcGyration(radius);

    auto ball = chrono_types::make_shared<ChBody>();
    ball->SetTag(tag_ball);
    ball->SetMass(mass);
    ball->SetInertia(inertia);
    ball->SetPos(ChVector3d(0, 0, 2));
    ball->SetFixed(false);
    ball->EnableCollision(true);
    sys.AddBody(ball);
    utils::AddTriangleMeshGeometry(ball.get(), mat, GetChronoDataFile("models/sphere.obj"));
    ball->GetVisualShape(0)->SetColor(ChColor(0.1f, 0.5f, 0.9f));

    // Create the bottom plate body (fixed to ground)
    auto plate = chrono_types::make_shared<ChBody>();
    plate->SetTag(tag_plate);
    plate->SetFixed(true);
    plate->EnableCollision(true);
    sys.AddBody(plate);
    utils::AddBoxGeometry(plate.get(), mat, ChVector3d(4, 4, 0.4));
    plate->GetVisualShape(0)->SetColor(ChColor(0.5f, 0.1f, 0.1f));

    // Create container walls (fixed to ground)
    auto container = chrono_types::make_shared<ChBody>();
    container->SetFixed(true);
    container->EnableCollision(true);
    sys.AddBody(container);
    utils::AddBoxContainer(container, mat,                           //
                           ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT),  //
                           ChVector3d(4, 4, 1), 0.2,                 //
                           ChVector3i(2, 2, 0),                      //
                           true);

    // Register callback for narrowphase processing and contact generation.
    // Add callback object to system (as a ChLoadContainer).
    // This will be used to override Chrono only for the ball-plate interaction.
    auto custom_contact =
        chrono_types::make_shared<CustomContact>(ball, plate, CustomContact::Method::ADD_CONTACT_LOADS);
    sys.GetCollisionSystem()->RegisterNarrowphaseCallback(custom_contact);
    sys.RegisterCustomCollisionCallback(custom_contact);
    sys.Add(custom_contact);

    // Create the run-time visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif
    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<irrlicht::ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(1280, 800);
            vis_irr->SetWindowTitle("Custom contact");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->SetCameraVertical(CameraVerticalDir::Z);
            vis_irr->AddCamera(ChVector3d(0, -4, 2));
            vis_irr->AddTypicalLights();
            vis_irr->EnableShadows();
            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("Custom contact");
            vis_vsg->SetWindowSize(1280, 800);
            vis_vsg->SetWindowPosition(100, 100);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->SetCameraVertical(CameraVerticalDir::Z);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->AddCamera(ChVector3d(0, -6, 3));
            vis_vsg->Initialize();
            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    double time_step = 1e-5;
    double render_fps = 100;
    int render_frame = 0;
    ChRealtimeStepTimer rt_timer;
    while (vis->Run()) {
        if (sys.GetChTime() > render_frame / render_fps) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();
            render_frame++;
        }
        custom_contact->Reset();
        sys.DoStepDynamics(time_step);
        rt_timer.Spin(time_step);
    }

    return 0;
}
