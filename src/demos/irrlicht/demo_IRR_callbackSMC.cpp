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

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

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
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        // Check if contact involves box1
        if (modA == m_box1.get()) {
            printf("  A contact on Box 1 at pos: %7.3f  %7.3f  %7.3f", pA.x(), pA.y(), pA.z());
        } else if (modB == m_box1.get()) {
            printf("  B contact on Box 1 at pos: %7.3f  %7.3f  %7.3f", pB.x(), pB.y(), pB.z());
        }

        // Check if contact involves box2
        if (modA == m_box2.get()) {
            printf("  A contact on Box 2 at pos: %7.3f  %7.3f  %7.3f", pA.x(), pA.y(), pA.z());
        } else if (modB == m_box2.get()) {
            printf("  B contact on Box 2 at pos: %7.3f  %7.3f  %7.3f", pB.x(), pB.y(), pB.z());
        }

        const ChVector<>& nrm = plane_coord.Get_A_Xaxis();
        printf("  nrm: %7.3f, %7.3f  %7.3f", nrm.x(), nrm.y(), nrm.z());
        printf("  frc: %7.3f  %7.3f  %7.3f", cforce.x(), cforce.y(), cforce.z());
        printf("  trq: %7.3f, %7.3f  %7.3f", ctorque.x(), ctorque.y(), ctorque.z());
        printf("  penetration: %8.4f   eff. radius: %7.3f\n", distance, eff_radius);

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
    system.SetSolverMaxIterations(100);
    system.SetSolverForceTolerance(0);

    // Change default collision effective radius of curvature
    ////collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(1);
    
    // --------------------------------------------------
    // Create a contact material, shared among all bodies
    // --------------------------------------------------

    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    material->SetFriction(friction);

    // ----------
    // Add bodies
    // ----------

    auto container = std::shared_ptr<ChBody>(system.NewBody());
    system.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->SetCollide(true);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), material, ChVector<>(4, 0.5, 4), ChVector<>(0, -0.5, 0));
    container->GetCollisionModel()->BuildModel();
    container->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.4f));

    auto box1 = std::shared_ptr<ChBody>(system.NewBody());
    box1->SetMass(10);
    box1->SetInertiaXX(ChVector<>(1, 1, 1));
    box1->SetPos(ChVector<>(-1, 0.21, -1));
    box1->SetPos_dt(ChVector<>(5, 0, 0));

    box1->SetCollide(true);
    box1->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box1.get(), material, ChVector<>(0.4, 0.2, 0.1));
    box1->GetCollisionModel()->BuildModel();
    box1->GetVisualShape(0)->SetColor(ChColor(0.1f, 0.1f, 0.4f));

    system.AddBody(box1);

    auto box2 = std::shared_ptr<ChBody>(system.NewBody());
    box2->SetMass(10);
    box2->SetInertiaXX(ChVector<>(1, 1, 1));
    box2->SetPos(ChVector<>(-1, 0.21, +1));
    box2->SetPos_dt(ChVector<>(5, 0, 0));

    box2->SetCollide(true);
    box2->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box2.get(), material, ChVector<>(0.4, 0.2, 0.1));
    box2->GetCollisionModel()->BuildModel();
    box2->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.1f, 0.1f));

    system.AddBody(box2);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    system.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("SMC callbacks");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(4, 4, -6));
    vis->AddTypicalLights();

    // ---------------
    // Simulate system
    // ---------------

    auto creporter = chrono_types::make_shared<ContactReporter>(box1, box2);

    auto cmaterial = chrono_types::make_shared<ContactMaterial>();
    system.GetContactContainer()->RegisterAddContactCallback(cmaterial);

    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        irrlicht::tools::drawGrid(vis.get(), 0.5, 0.5, 12, 12,
                                  ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));
        irrlicht::tools::drawAllCOGs(vis.get(), 1.0);

        system.DoStepDynamics(1e-3);
        vis->EndScene();

        // Process contacts
        std::cout << system.GetChTime() << "  " << system.GetNcontacts() << std::endl;
        system.GetContactContainer()->ReportAllContacts(creporter);

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
