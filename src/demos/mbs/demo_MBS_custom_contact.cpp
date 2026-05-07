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
// Authors: Radu Serban, Abhinov Koutharapu
// =============================================================================
//
// Demo illustrating custom contact generation between two bodies.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

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

    CustomContact(std::shared_ptr<ChBody> body, std::shared_ptr<ChBody> ground, Method method);

    // Clear the list of cached collisions.
    // Must be called at each step, before integration.
    void Reset();

    // Get the number of cached collisions at the current step.
    size_t GetNumCollisions() const;

    // Implement NarrowphaseCallback interface.
    // This function is called during collision detection for each identified pair of collision points.
    virtual bool OnNarrowphase(ChCollisionInfo& cinfo) override;

    // Implement CustomCollisionCallback interface
    virtual void OnCustomCollision(ChSystem* sys) override;

    // Implement ChLoadContainer interface.
    // The Setup function is called at each step, *after* collision detection.
    virtual void Setup() override;

  private:
    // Collision pairs associated with one collision shape
    typedef std::vector<ChCollisionInfo> CollisionSet;

    Method method;
    std::shared_ptr<ChBody> body;
    std::shared_ptr<ChBody> ground;
    size_t num_shapes;                                // number of body collision shapes
    std::vector<ChCollisionShape*> collision_shapes;  // body collision shapes
    std::vector<CollisionSet> collision_sets;         // collision sets for each collision shape
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
    int tag_dumbbell = 1;
    int tag_plate = 2;

    // Create the dumbbell object
    double radius = 1.0;
    double offset = radius + 0.2;
    double sphere_mass = 300;
    ChMatrix33d sphere_inertia = sphere_mass * ChSphere::CalcGyration(radius);
    CompositeInertia composite_inertia;
    composite_inertia.AddComponent(ChFramed(ChVector3d(-offset, 0, 0), QUNIT), sphere_mass, sphere_inertia);
    composite_inertia.AddComponent(ChFramed(ChVector3d(+offset, 0, 0), QUNIT), sphere_mass, sphere_inertia);

    auto dumbbell = chrono_types::make_shared<ChBody>();
    dumbbell->SetName("dumbbell");
    dumbbell->SetTag(tag_dumbbell);
    dumbbell->SetMass(composite_inertia.GetMass());
    dumbbell->SetInertia(composite_inertia.GetInertia());
    dumbbell->SetPos(ChVector3d(0, 0, 2));
    dumbbell->SetFixed(false);
    dumbbell->EnableCollision(true);
    sys.AddBody(dumbbell);
    utils::AddTriangleMeshGeometry(dumbbell.get(), mat, GetChronoDataFile("models/sphere.obj"), "L_SPHERE", ChVector3d(-offset, 0, 0));
    utils::AddTriangleMeshGeometry(dumbbell.get(), mat, GetChronoDataFile("models/sphere.obj"), "R_SPHERE", ChVector3d(+offset, 0, 0));
    auto cylinder = chrono_types::make_shared<ChVisualShapeCylinder>(0.2, 2 * offset);
    dumbbell->AddVisualShape(cylinder, ChFramed(VNULL, Q_ROTATE_X_TO_Z));
    dumbbell->GetVisualShape(0)->SetColor(ChColor(0.1f, 0.5f, 0.9f));
    dumbbell->GetVisualShape(1)->SetColor(ChColor(0.1f, 0.5f, 0.9f));
    dumbbell->GetVisualShape(2)->SetColor(ChColor(0.1f, 0.5f, 0.9f));

    // Create the bottom plate body (fixed to ground)
    double x_size = 2 * (radius + offset) + 2;
    double y_size = 2 * radius + 2;

    auto plate = chrono_types::make_shared<ChBody>();
    plate->SetName("plate");
    plate->SetTag(tag_plate);
    plate->SetFixed(true);
    plate->EnableCollision(true);
    sys.AddBody(plate);
    utils::AddBoxGeometry(plate.get(), mat, ChVector3d(x_size, y_size, 0.4));
    plate->GetVisualShape(0)->SetColor(ChColor(0.5f, 0.1f, 0.1f));

    // Create container walls (fixed to ground)
    auto container = chrono_types::make_shared<ChBody>();
    container->SetFixed(true);
    container->EnableCollision(true);
    sys.AddBody(container);
    utils::AddBoxContainer(container, mat,                           //
                           ChFrame<>(ChVector3d(0, 0, 0.5), QUNIT),  //
                           ChVector3d(x_size, y_size, radius), 0.2,  //
                           ChVector3i(2, 2, 0),                      //
                           true);

    // Register callback for narrowphase processing and contact generation.
    // Add callback object to system (as a ChLoadContainer).
    // This will be used to override Chrono only for the dumbbell-plate interaction.
    auto custom_contact = chrono_types::make_shared<CustomContact>(dumbbell, plate, CustomContact::Method::ADD_CONTACT_LOADS);
    sys.GetCollisionSystem()->RegisterNarrowphaseCallback(custom_contact);
    sys.RegisterCustomCollisionCallback(custom_contact);
    sys.Add(custom_contact);

    // Create the run-time visualization system
    auto vis = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("Custom contact");
    vis->SetWindowSize(1280, 800);
    vis->SetWindowPosition(100, 100);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetCameraAngleDeg(40.0);
    vis->AddCamera(ChVector3d(0, -6, 3));
    vis->Initialize();
    vis->SetContactNormalsVisibility(true);

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

// -----------------------------------------------------------------------------
// Implementation of CustomContact methods

CustomContact::CustomContact(std::shared_ptr<ChBody> body, std::shared_ptr<ChBody> ground, Method method) : body(body), ground(ground), method(method) {
    const auto& shape_instances = body->GetCollisionModel()->GetShapeInstances();
    for (const auto& shape_instance : shape_instances) {
        collision_shapes.push_back(shape_instance.shape.get());
    }
    num_shapes = shape_instances.size();
    collision_sets.resize(num_shapes);
}

void CustomContact::Reset() {
    for (auto& set : collision_sets)
        set.clear();
}

size_t CustomContact::GetNumCollisions() const {
    size_t num_collisions = 0;
    for (const auto& set : collision_sets)
        num_collisions += set.size();
    return num_collisions;
}

bool CustomContact::OnNarrowphase(ChCollisionInfo& cinfo) {
    // If the body is the first model involved in this collision and the ground is the second one, cache this collision
    if (body.get() == cinfo.modelA->GetContactable() && ground.get() == cinfo.modelB->GetContactable()) {
        // Do not cache collisions for which the two shapes are not penetrated
        if (cinfo.distance < 0) {
            // Compare against the colliding shape in `modelA` or its parent (if part of a compound)
            auto shapeA = cinfo.shapeA->GetParentShape() ? cinfo.shapeA->GetParentShape() : cinfo.shapeA;
            // Append to collision set corresponding to the body collision shape involved in this collision
            for (size_t i = 0; i < num_shapes; i++) {
                if (collision_shapes[i] == shapeA) {
                    collision_sets[i].push_back(cinfo);
                }
            }
        }
        // Indicate that Chrono should not process this collision (whether or not the shapes are penetrated)
        return false;
    }

    // If the body is the second model involved in this collision and the ground is the first one, cache this collision
    // Make sure to swap the collisions models and collision shapes inside the collision info structure
    if (body.get() == cinfo.modelB->GetContactable() && ground.get() == cinfo.modelA->GetContactable()) {
        // Do not cache collisions for which the two shapes are not penetrated
        if (cinfo.distance < 0) {
            // Compare against the colliding shape or its parent (if part of a compound)
            auto shapeB = cinfo.shapeB->GetParentShape() ? cinfo.shapeB->GetParentShape() : cinfo.shapeB;
            // Append to collision set corresponding to the body collision shape involved in this collision
            for (size_t i = 0; i < num_shapes; i++) {
                if (collision_shapes[i] == shapeB) {
                    // Append the "swapped" collision
                    ChCollisionInfo cinfoS = cinfo;
                    cinfoS.SwapModels();
                    collision_sets[i].push_back(cinfoS);
                }
            }
        }
        // Indicate that Chrono should not process this collision (whether or not the shapes are penetrated)
        return false;
    }

    // Let Chrono deal with all other collisions (not between the body and ground)
    return true;
}

void CustomContact::OnCustomCollision(ChSystem* sys) {
    if (method == Method::ADD_CONTACT_LOADS || GetNumCollisions() == 0)
        return;

    // Generate contacts between tagged bodies
    switch (method) {
        case Method::ADD_ALL_CONTACTS: {
            // Add a contact for each detected collision pair
            for (const auto& set : collision_sets) {
                for (const auto& cinfo : set)
                    sys->GetContactContainer()->AddContact(cinfo);
            }

            break;
        }

        case Method::ADD_AVERAGE_CONTACT: {
            // Calculate an average contact from a given collision set
            auto calculate_average_contact = [&](const CollisionSet& set) {
                if (set.empty())
                    return;

                auto num_collisions = set.size();

                auto matA = set[0].shapeA->GetMaterial();
                auto matB = set[0].shapeB->GetMaterial();

                // Calculate one equivalent collision (average everything)
                // Note: no need to specify collision shapes, as contact materials are explicitly specified
                ChCollisionInfo cinfo_avg;
                cinfo_avg.modelA = set[0].modelA;
                cinfo_avg.modelB = set[0].modelB;
                cinfo_avg.shapeA = nullptr;
                cinfo_avg.shapeB = nullptr;

                cinfo_avg.vpA = VNULL;
                cinfo_avg.vpB = VNULL;
                cinfo_avg.vN = VNULL;
                cinfo_avg.distance = 0;

                for (const auto& cinfo : set) {
                    assert(cinfo.distance < 0);

                    if (body.get() == cinfo.modelA->GetContactable()) {
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
            };

            // Generate an average contact for each collision set
            for (const auto& set : collision_sets)
                calculate_average_contact(set);

            break;
        }
    }
}

void CustomContact::Setup() {
    if (method != Method::ADD_CONTACT_LOADS)
        return;

    // Reset the load list for this load container
    GetLoadList().clear();

    if (GetNumCollisions() == 0)
        return;

    // Composite material properties
    double E_eff = 2e6;
    double G_eff = 2e6;
    double mu_eff = 0.8;
    double cr_eff = 0.01;
    double eff_radius = 0.5;

    // Generate contact forces between the two bodies based on a given set of collisions
    auto process_loads = [&](const CollisionSet& set) {
        for (const auto& cinfo : set) {
            // Extract collision information
            auto delta = -cinfo.distance;
            auto normal_dir = cinfo.vN;
            auto pA = cinfo.vpA;
            auto pB = cinfo.vpB;
            auto objA = cinfo.modelA->GetContactable();
            auto objB = cinfo.modelB->GetContactable();
            auto vel1 = objA->GetContactPointSpeed(pA);
            auto vel2 = objB->GetContactPointSpeed(pB);

            // Calculate relative normal and tangential velocities
            ChVector3d relvel = vel2 - vel1;
            double relvel_n_mag = relvel.Dot(normal_dir);
            ChVector3d relvel_n = relvel_n_mag * normal_dir;
            ChVector3d relvel_t = relvel - relvel_n;
            double relvel_t_mag = relvel_t.Length();

            // Hertz contact
            double eff_mass = objA->GetContactableMass() * objB->GetContactableMass() / (objA->GetContactableMass() + objB->GetContactableMass());
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
            Add(chrono_types::make_shared<ChLoadBodyForce>(body, -force, false, cinfo.vpA, false));
            Add(chrono_types::make_shared<ChLoadBodyForce>(ground, +force, false, cinfo.vpA, false));
        }
    };

    // Add contact loads for each collision set
    for (const auto& set : collision_sets)
        process_loads(set);

    // Perform a full update of the load container
    ChLoadContainer::Update(ChTime, UpdateFlags::UPDATE_ALL);
}
