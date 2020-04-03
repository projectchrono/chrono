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
// (complementarity-based).
//
// The global reference frame has Y up.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Callback class for contact reporting
// -----------------------------------------------------------------------------
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(std::shared_ptr<ChBody> box) : m_box(box) {}

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
        if (modA == m_box.get()) {
            printf("  %6.3f  %6.3f  %6.3f\n", pA.x(), pA.y(), pA.z());
        } else if (modB == m_box.get()) {
            printf("  %6.3f  %6.3f  %6.3f\n", pB.x(), pB.y(), pB.z());
        }
        return true;
    }

    std::shared_ptr<ChBody> m_box;
};

// -----------------------------------------------------------------------------
// Callback class for modifying composite material
// -----------------------------------------------------------------------------
class ContactMaterial : public ChContactContainer::AddContactCallback {
  public:
    virtual void OnAddContact(const collision::ChCollisionInfo& contactinfo,
                              ChMaterialComposite* const material) override {
        // Downcast to appropriate composite material type
        auto mat = static_cast<ChMaterialCompositeNSC* const>(material);

        // Set different friction for left/right halfs
        float friction = (contactinfo.vpA.z() > 0) ? 0.3f : 0.8f;
        mat->static_friction = friction;
        mat->sliding_friction = friction;
    }
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ----------------
    // Parameters
    // ----------------

    float friction = 0.6f;
    double collision_envelope = .001;

    // -----------------
    // Create the system
    // -----------------

    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, -10, 0));

    // Set solver settings
    system.SetSolverMaxIterations(100);
    system.SetMaxPenetrationRecoverySpeed(1e8);
    system.SetSolverForceTolerance(0);

    // --------------------------------------------------
    // Create a contact material, shared among all bodies
    // --------------------------------------------------

    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    material->SetFriction(friction);

    // ----------
    // Add bodies
    // ----------

    auto container = chrono_types::make_shared<ChBody>();
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->SetCollide(true);
    container->GetCollisionModel()->SetEnvelope(collision_envelope);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), material, ChVector<>(4, 0.5, 4), ChVector<>(0, -0.5, 0));
    container->GetCollisionModel()->BuildModel();

    container->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.4f, 0.4f, 0.4f)));

    auto box1 = chrono_types::make_shared<ChBody>();
    box1->SetMass(10);
    box1->SetInertiaXX(ChVector<>(1, 1, 1));
    box1->SetPos(ChVector<>(-1, 0.21, -1));
    box1->SetPos_dt(ChVector<>(5, 0, 0));

    box1->SetCollide(true);
    box1->GetCollisionModel()->SetEnvelope(collision_envelope);
    box1->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box1.get(), material, ChVector<>(0.4, 0.2, 0.1));
    box1->GetCollisionModel()->BuildModel();

    box1->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.1f, 0.1f, 0.4f)));

    system.AddBody(box1);

    auto box2 = std::shared_ptr<ChBody>(system.NewBody());
    box2->SetMass(10);
    box2->SetInertiaXX(ChVector<>(1, 1, 1));
    box2->SetPos(ChVector<>(-1, 0.21, +1));
    box2->SetPos_dt(ChVector<>(5, 0, 0));

    box2->SetCollide(true);
    box2->GetCollisionModel()->SetEnvelope(collision_envelope);
    box2->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box2.get(), material, ChVector<>(0.4, 0.2, 0.1));
    box2->GetCollisionModel()->BuildModel();

    box2->AddAsset(chrono_types::make_shared<ChColorAsset>(ChColor(0.4f, 0.1f, 0.1f)));

    system.AddBody(box2);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    irrlicht::ChIrrApp application(&system, L"NSC callbacks", irr::core::dimension2d<irr::u32>(800, 600), false, true);
    irrlicht::ChIrrWizard::add_typical_Logo(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Sky(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Lights(application.GetDevice());
    irrlicht::ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(4, 4, -6));

    application.AssetBindAll();
    application.AssetUpdateAll();

    // ---------------
    // Simulate system
    // ---------------

    ContactReporter creporter(box1);

    ContactMaterial cmaterial;
    system.GetContactContainer()->RegisterAddContactCallback(&cmaterial);

    application.SetTimestep(1e-3);

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
        application.DrawAll();
        irrlicht::ChIrrTools::drawGrid(application.GetVideoDriver(), 0.5, 0.5, 12, 12,
                                       ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));
        application.DoStep();
        application.EndScene();

        // Process contacts
        std::cout << system.GetChTime() << "  " << system.GetNcontacts() << std::endl;
        system.GetContactContainer()->ReportAllContacts(&creporter);
    }

    return 0;
}
