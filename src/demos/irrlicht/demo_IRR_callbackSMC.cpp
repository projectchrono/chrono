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

// -----------------------------------------------------------------------------
// Class for overriding composite material laws
// -----------------------------------------------------------------------------
class CompsiteMaterial : public ChMaterialCompositionStrategy {
  public:
    virtual float CombineFriction(float a1, float a2) const { return 0.5 * (a1 + a2); }
};

// -----------------------------------------------------------------------------
// Class for overriding the default SMC contact force calculation
// -----------------------------------------------------------------------------
class ContactForce : public ChSystemSMC::ChContactForceSMC {
  public:
    // Demonstration only.
    virtual ChVector<> CalculateForce(
        const ChSystemSMC& sys,             ///< containing sys
        const ChVector<>& normal_dir,       ///< normal contact direction (expressed in global frame)
        const ChVector<>& p1,               ///< most penetrated point on obj1 (expressed in global frame)
        const ChVector<>& p2,               ///< most penetrated point on obj2 (expressed in global frame)
        const ChVector<>& vel1,             ///< velocity of contact point on obj1 (expressed in global frame)
        const ChVector<>& vel2,             ///< velocity of contact point on obj2 (expressed in global frame)
        const ChMaterialCompositeSMC& mat,  ///< composite material for contact pair
        double delta,                       ///< overlap in normal direction
        double eff_radius,                  ///< effective radius of curvature at contact
        double mass1,                       ///< mass of obj1
        double mass2                        ///< mass of obj2
    ) const override {
        // Relative velocity at contact
        ChVector<> relvel = vel2 - vel1;
        double relvel_n_mag = relvel.Dot(normal_dir);
        ChVector<> relvel_n = relvel_n_mag * normal_dir;
        ChVector<> relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();

        // Calculate effective mass
        double eff_mass = mass1 * mass2 / (mass1 + mass2);
        
        // Calculate the magnitudes of the normal and tangential contact forces
        double kn = mat.kn;
        double kt = mat.kt;
        double gn = eff_mass * mat.gn;
        double gt = eff_mass * mat.gt;

        // Tangential displacement (magnitude)
        double dT = sys.GetStep();
        double delta_t = relvel_t_mag * dT;

        double forceN = kn * delta - gn * relvel_n_mag;
        double forceT = kt * delta_t + gt * relvel_t_mag;

        // Coulomb law
        forceT = std::min<double>(forceT, mat.mu_eff * std::abs(forceN));

        // Accumulate normal and tangential forces
        ChVector<> force = forceN * normal_dir;
        if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
            force -= (forceT / relvel_t_mag) * relvel_t;

        return force;
    }
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // ----------------
    // Parameters
    // ----------------

    float friction = 0.6f;

    // -----------------
    // Create the sys
    // -----------------

    ChSystemSMC sys;
    sys.Set_G_acc(ChVector<>(0, -10, 0));

    // Set solver settings
    sys.SetSolverMaxIterations(100);
    sys.SetSolverForceTolerance(0);

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

    auto container = std::shared_ptr<ChBody>(sys.NewBody());
    sys.Add(container);
    container->SetPos(ChVector<>(0, 0, 0));
    container->SetBodyFixed(true);
    container->SetIdentifier(-1);

    container->SetCollide(true);
    container->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(container.get(), material, ChVector<>(4, 0.5, 4), ChVector<>(0, -0.5, 0));
    container->GetCollisionModel()->BuildModel();
    container->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.4f));

    auto box1 = std::shared_ptr<ChBody>(sys.NewBody());
    box1->SetMass(10);
    box1->SetInertiaXX(ChVector<>(1, 1, 1));
    box1->SetPos(ChVector<>(-1, 0.21, -1));
    box1->SetPos_dt(ChVector<>(5, 0, 0));

    box1->SetCollide(true);
    box1->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box1.get(), material, ChVector<>(0.4, 0.2, 0.1));
    box1->GetCollisionModel()->BuildModel();
    box1->GetVisualShape(0)->SetColor(ChColor(0.1f, 0.1f, 0.4f));

    sys.AddBody(box1);

    auto box2 = std::shared_ptr<ChBody>(sys.NewBody());
    box2->SetMass(10);
    box2->SetInertiaXX(ChVector<>(1, 1, 1));
    box2->SetPos(ChVector<>(-1, 0.21, +1));
    box2->SetPos_dt(ChVector<>(5, 0, 0));

    box2->SetCollide(true);
    box2->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(box2.get(), material, ChVector<>(0.4, 0.2, 0.1));
    box2->GetCollisionModel()->BuildModel();
    box2->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.1f, 0.1f));

    sys.AddBody(box2);

    // -------------------------------
    // Create the visualization window
    // -------------------------------

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("SMC callbacks");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(4, 4, -6));
    vis->AddTypicalLights();

    // ------------------------------------------------------------------------------
    // Use various user supplied callbacks to override default Chrono implementations
    // ------------------------------------------------------------------------------

    // User-defined SMC contact force calculation
    auto cforce = chrono_types::make_unique<ContactForce>();
    sys.SetContactForceAlgorithm(std::move(cforce));

    // User-defined composite coefficent of friction
    auto cmat = chrono_types::make_unique<ChMaterialCompositionStrategy>();
    sys.SetMaterialCompositionStrategy(std::move(cmat));

    // OVerride material properties at each new contact
    auto cmaterial = chrono_types::make_shared<ContactMaterial>();
    sys.GetContactContainer()->RegisterAddContactCallback(cmaterial);

    // User-defined callback for contact reporting
    auto creporter = chrono_types::make_shared<ContactReporter>(box1, box2);

    // ---------------
    // Simulate sys
    // ---------------

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        irrlicht::tools::drawGrid(vis.get(), 0.5, 0.5, 12, 12,
                                  ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));
        irrlicht::tools::drawAllCOGs(vis.get(), 1.0);

        sys.DoStepDynamics(1e-3);
        vis->EndScene();

        // Process contacts
        std::cout << sys.GetChTime() << "  " << sys.GetNcontacts() << std::endl;
        sys.GetContactContainer()->ReportAllContacts(creporter);

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
