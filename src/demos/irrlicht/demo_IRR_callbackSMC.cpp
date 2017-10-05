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
// (penalty-based).
//
// The global reference frame has Y up.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Callback class for contact reporting
// -----------------------------------------------------------------------------
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(std::shared_ptr<ChBody> box1, std::shared_ptr<ChBody> box2) : m_box1(box1), m_box2(box2) {}

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        // Convert force in absolute frame
        ChVector<> frc = plane_coord * cforce;

        // Check if contact involves box1
        if (modA == m_box1.get()) {
            printf("  contact on Box 1 at pos: %7.3f  %7.3f  %7.3f", pA.x(), pA.y(), pA.z());
            printf("  frc: %7.3f  %7.3f  %7.3f\n", frc.x(), frc.y(), frc.z());
        } else if (modB == m_box1.get()) {
            printf("  contact on Box 1 at pos: %7.3f  %7.3f  %7.3f", pB.x(), pB.y(), pB.z());
            printf("  frc: %7.3f  %7.3f  %7.3f\n", frc.x(), frc.y(), frc.z());
        }

        // Check if contact involves box2
        if (modA == m_box2.get()) {
            printf("  contact on Box 2 at pos: %7.3f  %7.3f  %7.3f", pA.x(), pA.y(), pA.z());
            printf("  frc: %7.3f  %7.3f  %7.3f\n", frc.x(), frc.y(), frc.z());
        } else if (modB == m_box2.get()) {
            printf("  contact on Box 2 at pos: %7.3f  %7.3f  %7.3f", pB.x(), pB.y(), pB.z());
            printf("  frc: %7.3f  %7.3f  %7.3f\n", frc.x(), frc.y(), frc.z());
        }

        return true;
    }

    std::shared_ptr<ChBody> m_box1;
    std::shared_ptr<ChBody> m_box2;
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
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ----------------
    // Parameters
    // ----------------

    float friction = 0.6f;

    // -----------------
    // Create the system
    // -----------------

    ChSystemSMC system;
    system.Set_G_acc(ChVector<>(0, -10, 0));

    // Set solver settings
    system.SetMaxItersSolverSpeed(100);
    system.SetTol(0);
    system.SetTolForce(0);

    // ----------
    // Add bodies
    // ----------

    auto container = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->GetMaterialSurfaceSMC()->SetFriction(friction);

    container->SetCollide(true);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), ChVector<>(4, 0.5, 4), ChVector<>(0, -0.5, 0));
    container->GetCollisionModel()->BuildModel();

    container->AddAsset(std::make_shared<ChColorAsset>(ChColor(0.4f, 0.4f, 0.4f)));

    auto box1 = std::shared_ptr<ChBody>(system.NewBody());
    box1->SetMass(10);
    box1->SetInertiaXX(ChVector<>(1, 1, 1));
    box1->SetPos(ChVector<>(-1, 0.21, -1));
    box1->SetPos_dt(ChVector<>(5, 0, 0));

    box1->GetMaterialSurfaceSMC()->SetFriction(friction);

    box1->SetCollide(true);
    box1->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box1.get(), ChVector<>(0.4, 0.2, 0.1));
    box1->GetCollisionModel()->BuildModel();

    box1->AddAsset(std::make_shared<ChColorAsset>(ChColor(0.1f, 0.1f, 0.4f)));

    system.AddBody(box1);

    auto box2 = std::shared_ptr<ChBody>(system.NewBody());
    box2->SetMass(10);
    box2->SetInertiaXX(ChVector<>(1, 1, 1));
    box2->SetPos(ChVector<>(-1, 0.21, +1));
    box2->SetPos_dt(ChVector<>(5, 0, 0));

    box2->GetMaterialSurfaceSMC()->SetFriction(friction);

    box2->SetCollide(true);
    box2->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box2.get(), ChVector<>(0.4, 0.2, 0.1));
    box2->GetCollisionModel()->BuildModel();

    box2->AddAsset(std::make_shared<ChColorAsset>(ChColor(0.4f, 0.1f, 0.1f)));

    system.AddBody(box2);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    irrlicht::ChIrrApp application(&system, L"SMC callbacks", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(4, 4, -6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // ---------------
    // Simulate system
    // ---------------

    ContactReporter creporter(box1, box2);

    ContactMaterial cmaterial;
    system.GetContactContainer()->RegisterAddContactCallback(&cmaterial);

    application.SetTimestep(1e-3);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        application.DrawAll();
        irrlicht::ChIrrTools::drawGrid(application.GetVideoDriver(), 0.5, 0.5, 12, 12,
                                       ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));
        irrlicht::ChIrrTools::drawAllCOGs(system, application.GetVideoDriver(), 1.0);

        application.DoStep();
        application.EndScene();

        // Process contacts
        std::cout << system.GetChTime() << "  " << system.GetNcontacts() << std::endl;
        system.GetContactContainer()->ReportAllContacts(&creporter);

        // Cumulative contact force and torque on boxes (as applied to COM)
        ChVector<> frc1 = box1->GetContactForce();
        ChVector<> trq1 = box1->GetContactTorque();
        printf("  Box 1 contact force at COM: %7.3f  %7.3f  %7.3f", frc1.x(), frc1.y(), frc1.z());
        printf("  contact torque at COM: %7.3f  %7.3f  %7.3f\n", trq1.x(), trq1.y(), trq1.z());
        ChVector<> frc2 = box2->GetContactForce();
        ChVector<> trq2 = box2->GetContactTorque();
        printf("  Box 2 contact force at COM: %7.3f  %7.3f  %7.3f", frc2.x(), frc2.y(), frc2.z());
        printf("  contact torque at COM: %7.3f  %7.3f  %7.3f\n", trq2.x(), trq2.y(), trq2.z());
    }

    return 0;
}
