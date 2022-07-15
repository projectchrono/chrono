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
// Chrono demonstration of using contact callbacks for smooth contacts
// (penalty-based) in Chrono::Multicore.
//
// The global reference frame has Y up.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"

#include "chrono_opengl/ChOpenGLWindow.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Callback class for contact reporting
// -----------------------------------------------------------------------------
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(std::shared_ptr<ChBody> obj1, std::shared_ptr<ChBody> obj2) : m_obj1(obj1), m_obj2(obj2) {}

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
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

        const ChVector<>& nrm = plane_coord.Get_A_Xaxis();
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
    virtual void OnAddContact(const collision::ChCollisionInfo& contactinfo,
                              ChMaterialComposite* const material) override {
        // Downcast to appropriate composite material type
        auto mat = static_cast<ChMaterialCompositeSMC* const>(material);

        // Set different friction for left/right halfs
        float friction = (contactinfo.vpA.z() > 0) ? 0.3f : 0.8f;
        mat->mu_eff = friction;
    }
};

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Parameters
    float friction = 0.6f;

    // Create the system
    ChSystemMulticoreSMC system;
    system.Set_G_acc(ChVector<>(0, -10, 0));

    // Create a contact material, shared among all bodies
    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    material->SetFriction(friction);

    // Add bodies
    auto container = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->SetCollide(true);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), material, ChVector<>(4, 0.5, 4), ChVector<>(0, -0.5, 0));
    container->GetCollisionModel()->BuildModel();

    auto obj1 = std::shared_ptr<ChBody>(system.NewBody());
    obj1->SetMass(10);
    obj1->SetInertiaXX(ChVector<>(1, 1, 1));
    obj1->SetPos(ChVector<>(-1, 0.21, -1));
    obj1->SetPos_dt(ChVector<>(5, 0, 0));

    obj1->SetCollide(true);
    obj1->GetCollisionModel()->ClearModel();
    utils::AddCapsuleGeometry(obj1.get(), material, 0.2, 0.4, ChVector<>(0), Q_from_AngZ(CH_C_PI_2));
    obj1->GetCollisionModel()->BuildModel();

    system.AddBody(obj1);

    auto obj2 = std::shared_ptr<ChBody>(system.NewBody());
    obj2->SetMass(10);
    obj2->SetInertiaXX(ChVector<>(1, 1, 1));
    obj2->SetPos(ChVector<>(-1, 0.21, +1));
    obj2->SetPos_dt(ChVector<>(5, 0, 0));

    obj2->SetCollide(true);
    obj2->GetCollisionModel()->ClearModel();
    utils::AddCapsuleGeometry(obj2.get(), material, 0.2, 0.4, ChVector<>(0), Q_from_AngZ(CH_C_PI_2));
    obj2->GetCollisionModel()->BuildModel();

    system.AddBody(obj2);

    // Create the visualization window
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.AttachSystem(&system);
    gl_window.Initialize(1280, 720, "SMC callbacks");
    gl_window.SetCamera(ChVector<>(4, 4, -5), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    // Callback for contact reporting
    auto creporter = chrono_types::make_shared<ContactReporter>(obj1, obj2);

    // Callback for contact addition
    auto cmaterial = chrono_types::make_shared<ContactMaterial>();
    system.GetContactContainer()->RegisterAddContactCallback(cmaterial);

    // Simulate system
    while (gl_window.Active()) {
        gl_window.DoStepDynamics(1e-3);
        gl_window.Render();

        // Process contacts
        std::cout << system.GetChTime() << "  " << system.GetNcontacts() << std::endl;
        system.GetContactContainer()->ReportAllContacts(creporter);

        // Cumulative contact force and torque on objects (as applied to COM)
        ChVector<> frc1 = obj1->GetContactForce();
        ChVector<> trq1 = obj1->GetContactTorque();
        printf("  Obj 1 contact force at COM: %7.3f  %7.3f  %7.3f", frc1.x(), frc1.y(), frc1.z());
        printf("  contact torque at COM: %7.3f  %7.3f  %7.3f\n", trq1.x(), trq1.y(), trq1.z());
        ChVector<> frc2 = obj2->GetContactForce();
        ChVector<> trq2 = obj2->GetContactTorque();
        printf("  Obj 2 contact force at COM: %7.3f  %7.3f  %7.3f", frc2.x(), frc2.y(), frc2.z());
        printf("  contact torque at COM: %7.3f  %7.3f  %7.3f\n", trq2.x(), trq2.y(), trq2.z());
    }

    return 0;
}
