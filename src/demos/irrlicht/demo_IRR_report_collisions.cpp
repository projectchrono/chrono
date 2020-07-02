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
// Demonstration of using a callback for reporting collisions.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;

// Callback class for contact processing.
class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager(ChSystem* system) : m_system(system) {}

    // Return the current total number of contacts experienced by the specified body.
    unsigned int GetNcontacts(std::shared_ptr<ChBody> body) const {
        auto search = m_bcontacts.find(body.get());
        return (search == m_bcontacts.end()) ? 0 : search->second;
    }

    // Process all contacts at current time.
    // Reset the hash map and invoke the callback for each collision.
    void Process() {
        m_bcontacts.clear();
        std::shared_ptr<ContactManager> shared_this(this, [](ContactManager*) {});
        m_system->GetContactContainer()->ReportAllContacts(shared_this);
    }

  private:
    // Keep track of the number of contacts experienced by each body.
    // Maintain a hash map with body as key and current number of contacts as value.
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        auto bodyA = static_cast<ChBody*>(modA);
        auto searchA = m_bcontacts.find(bodyA);
        if (searchA == m_bcontacts.end())
            m_bcontacts.insert(std::make_pair(bodyA, 1));
        else
            searchA->second++;

        auto bodyB = static_cast<ChBody*>(modB);
        auto searchB = m_bcontacts.find(bodyB);
        if (searchB == m_bcontacts.end())
            m_bcontacts.insert(std::make_pair(bodyB, 1));
        else
            searchB->second++;

        return true;
    }

    ChSystem* m_system;
    std::unordered_map<ChBody*, unsigned int> m_bcontacts;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create the system.
    ChSystemNSC system;
    system.SetSolverType(ChSolver::Type::PSOR);
    system.SetSolverMaxIterations(20);

    // Create the Irrlicht application.
    ChIrrApp application(&system, L"Number of collisions", irr::core::dimension2d<irr::u32>(800, 600), false);
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), irr::core::vector3df(0, 14, -20));

    // Create a contact material shared by all collision shapes
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Creeate a container fixed to ground (invisible).
    auto container = chrono_types::make_shared<ChBody>();
    container->SetBodyFixed(true);
    container->SetCollide(true);
    container->GetCollisionModel()->ClearModel();
    container->GetCollisionModel()->AddBox(mat, 20, 1, 20, ChVector<>(0, -10, 0));
    container->GetCollisionModel()->AddBox(mat, 1, 40, 20, ChVector<>(-11, 0, 0));
    container->GetCollisionModel()->AddBox(mat, 1, 40, 20, ChVector<>(11, 0, 0));
    container->GetCollisionModel()->AddBox(mat, 20, 40, 1, ChVector<>(0, 0, -11));
    container->GetCollisionModel()->AddBox(mat, 20, 40, 1, ChVector<>(0, 0, 11));
    container->GetCollisionModel()->BuildModel();
    system.AddBody(container);

    // Create falling rigid bodies with different shapes and mark one of each type.
    std::shared_ptr<ChBody> my_sphere;
    std::shared_ptr<ChBody> my_box;
    std::shared_ptr<ChBody> my_cylinder;
    for (int bi = 0; bi < 20; bi++) {
        auto sphere = chrono_types::make_shared<ChBodyEasySphere>(1.1, 1000, true, true, mat);
        system.Add(sphere);
        sphere->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        if (bi == 0) {
            sphere->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("bluwhite.png")));
            my_sphere = sphere;
        }

        auto box = chrono_types::make_shared<ChBodyEasyBox>(1.5, 1.5, 1.5, 100, true, true, mat);
        system.Add(box);
        box->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        if (bi == 0) {
            box->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("cubetexture_bluwhite.png")));
            my_box = box;
        }

        auto cylinder = chrono_types::make_shared<ChBodyEasyCylinder>(0.75, 0.5, 100, true, true, mat);
        system.Add(cylinder);
        cylinder->SetPos(ChVector<>(-5 + ChRandom() * 10, 4 + bi * 0.05, -5 + ChRandom() * 10));
        if (bi == 0) {
            cylinder->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("pinkwhite.png")));
            my_cylinder = cylinder;
        }
    }

    // Complete visualization asset construction.
    application.AssetBindAll();
    application.AssetUpdateAll();

    // Create the contact manager.
    ContactManager manager(&system);

    // Simulation loop.
    application.SetTimestep(0.02);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        // Render scene.
        application.DrawAll();

        // Advance dynamics.
        application.DoStep();

        // Process current collisions and report number of contacts on a few bodies.
        manager.Process();
        std::cout << "Time: " << system.GetChTime();
        std::cout << "   container: " << manager.GetNcontacts(container);
        std::cout << "   my_sphere: " << manager.GetNcontacts(my_sphere);
        std::cout << "   my_box: " << manager.GetNcontacts(my_box);
        std::cout << "   my_cylinder: " << manager.GetNcontacts(my_cylinder);
        std::cout << std::endl;

        application.EndScene();
    }

    return 0;
}
