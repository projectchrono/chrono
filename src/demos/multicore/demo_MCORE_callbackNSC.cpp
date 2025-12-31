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
// Chrono demonstration of using contact callbacks for non-smooth contacts
// (complementarity-based) in Chrono::Multicore.
//
// The global reference frame has Y up.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono/assets/ChVisualSystem.h"
#include "chrono_vsg/ChVisualSystemVSG.h"

using namespace chrono;
using namespace chrono::vsg3d;

// -----------------------------------------------------------------------------
// Callback class for contact reporting
// -----------------------------------------------------------------------------
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(std::shared_ptr<ChBody> obj1, std::shared_ptr<ChBody> obj2) : m_obj1(obj1), m_obj2(obj2) {}

  private:
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 double distance,
                                 double eff_radius,
                                 const ChVector3d& cforce,
                                 const ChVector3d& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB,
                                 int cnstr_offset) override {
        // Check if contact involves obj1
        if (modA == m_obj1.get()) {
            printf("  A contact on Obj 1 at pos: %7.3f  %7.3f  %7.3f", pA.x(), pA.y(), pA.z());
        } else if (modB == m_obj1.get()) {
            printf("  B contact on Obj 1 at pos: %7.3f  %7.3f  %7.3f", pB.x(), pB.y(), pB.z());
        }

        // Check if contact involves obj2
        if (modA == m_obj2.get()) {
            printf("  A contact on Obj 2 at pos: %7.3f  %7.3f  %7.3f", pA.x(), pA.y(), pA.z());
        } else if (modB == m_obj2.get()) {
            printf("  B contact on Obj 2 at pos: %7.3f  %7.3f  %7.3f", pB.x(), pB.y(), pB.z());
        }

        const ChVector3d& nrm = plane_coord.GetAxisX();
        printf("  nrm: %7.3f, %7.3f  %7.3f", nrm.x(), nrm.y(), nrm.z());
        printf("  frc: %7.3f  %7.3f  %7.3f", cforce.x(), cforce.y(), cforce.z());
        printf("  trq: %7.3f, %7.3f  %7.3f", ctorque.x(), ctorque.y(), ctorque.z());
        printf("  penetration: %8.4f   eff. radius: %7.3f\n", distance, eff_radius);

        return true;
    }

    std::shared_ptr<ChBody> m_obj1;
    std::shared_ptr<ChBody> m_obj2;
};

// -----------------------------------------------------------------------------
// Callback class for modifying composite material
// -----------------------------------------------------------------------------
class ContactMaterial : public ChContactContainer::AddContactCallback {
  public:
    virtual void OnAddContact(const ChCollisionInfo& contactinfo, ChContactMaterialComposite* const material) override {
        // Downcast to appropriate composite material type
        auto mat = static_cast<ChContactMaterialCompositeNSC* const>(material);

        // Set different friction for left/right halfs
        float friction = (contactinfo.vpA.z() > 0) ? 0.8f : 0.3f;
        mat->static_friction = friction;
        mat->sliding_friction = friction;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Parameters
    float friction = 0.6f;

    // Create the sys
    ChSystemMulticoreNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, -10, 0));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    sys.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    sys.GetSettings()->solver.max_iteration_sliding = 500;
    sys.GetSettings()->solver.contact_recovery_speed = 0;
    sys.GetSettings()->collision.collision_envelope = 0.005;
    sys.ChangeSolverType(SolverType::BB);

    // Create a contact material, shared among all bodies
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetFriction(friction);

    // Add bodies
    auto container = chrono_types::make_shared<ChBody>();
    sys.Add(container);
    container->SetPos(ChVector3d(0, 0, 0));
    container->SetFixed(true);

    container->EnableCollision(true);
    utils::AddBoxGeometry(container.get(), material, ChVector3d(8, 1, 8), ChVector3d(0, -0.5, 0));

    auto obj1 = chrono_types::make_shared<ChBody>();
    obj1->SetMass(10);
    obj1->SetInertiaXX(ChVector3d(1, 1, 1));
    obj1->SetPos(ChVector3d(-1, 0.21, -1));
    obj1->SetPosDt(ChVector3d(5, 0, 0));

    obj1->EnableCollision(true);
    utils::AddCapsuleGeometry(obj1.get(), material, 0.2, 0.4, ChVector3d(0), QuatFromAngleZ(CH_PI_2));

    sys.AddBody(obj1);

    auto obj2 = chrono_types::make_shared<ChBody>();
    obj2->SetMass(10);
    obj2->SetInertiaXX(ChVector3d(1, 1, 1));
    obj2->SetPos(ChVector3d(-1, 0.21, +1));
    obj2->SetPosDt(ChVector3d(5, 0, 0));

    obj2->EnableCollision(true);
    utils::AddCapsuleGeometry(obj2.get(), material, 0.2, 0.4, ChVector3d(0), QuatFromAngleZ(CH_PI_2));

    sys.AddBody(obj2);

    // Create the visualization window
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowTitle("NSC callbacks (Chrono::Multicore)");
    vis->SetCameraVertical(CameraVerticalDir::Y);
    vis->AddCamera(ChVector3d(4, 4, -5), ChVector3d(0, 0, 0));
    vis->SetWindowSize(1280, 720);
    vis->SetBackgroundColor(ChColor(0.8f, 0.85f, 0.9f));
    vis->EnableSkyBox();
    vis->SetCameraAngleDeg(40.0);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis->EnableShadows();
    vis->Initialize();

    // Callback for contact reporting
    auto creporter = chrono_types::make_shared<ContactReporter>(obj1, obj2);

    // Callback for contact addition
    auto cmaterial = chrono_types::make_shared<ContactMaterial>();
    sys.GetContactContainer()->RegisterAddContactCallback(cmaterial);

    // Simulate sys
    while (vis->Run()) {
        sys.DoStepDynamics(1e-3);
        vis->Render();

        // Process contacts
        std::cout << sys.GetChTime() << "  " << sys.GetNumContacts() << std::endl;

        if (sys.GetNumContacts() > 0) {
            // Force calculation of cumulative contact forces for all bodies in sys
            // (required for an NSC sys before invoking GetContactForce/GetContactTorque)
            sys.CalculateContactForces();

            sys.GetContactContainer()->ReportAllContacts(creporter);

            // Cumulative contact force and torque on objects (as applied to COM)
            ChVector3d frc1 = obj1->GetContactForce();
            ChVector3d trq1 = obj1->GetContactTorque();
            printf("  Obj 1 contact force at COM: %7.3f  %7.3f  %7.3f", frc1.x(), frc1.y(), frc1.z());
            printf("  contact torque at COM: %7.3f  %7.3f  %7.3f\n", trq1.x(), trq1.y(), trq1.z());
            ChVector3d frc2 = obj2->GetContactForce();
            ChVector3d trq2 = obj2->GetContactTorque();
            printf("  Obj 2 contact force at COM: %7.3f  %7.3f  %7.3f", frc2.x(), frc2.y(), frc2.z());
            printf("  contact torque at COM: %7.3f  %7.3f  %7.3f\n", trq2.x(), trq2.y(), trq2.z());
        }
    }

    return 0;
}
