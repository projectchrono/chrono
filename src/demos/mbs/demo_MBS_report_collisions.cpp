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
#include "chrono/core/ChRandom.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

// Callback class for contact processing.
class ContactManager : public ChContactContainer::ReportContactCallback {
  public:
    ContactManager(ChSystem* sys) : m_system(sys) {}

    // Return the current total number of contacts experienced by the specified body.
    unsigned int GetNumContacts(std::shared_ptr<ChBody> body) const {
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
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 double distance,
                                 double eff_radius,
                                 const ChVector3d& cforce,
                                 const ChVector3d& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB,
                                 int constraint_offset) override {
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
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create the sys.
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.GetSolver()->AsIterative()->SetMaxIterations(20);

    // Create a contact material shared by all collision shapes
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // Creeate a container fixed to ground (invisible).
    auto container = chrono_types::make_shared<ChBody>();
    container->SetFixed(true);
    container->EnableCollision(true);
    auto shape1 = chrono_types::make_shared<ChCollisionShapeBox>(mat, 40, 2, 40);
    auto shape2 = chrono_types::make_shared<ChCollisionShapeBox>(mat, 2, 80, 40);
    auto shape3 = chrono_types::make_shared<ChCollisionShapeBox>(mat, 2, 80, 40);
    auto shape4 = chrono_types::make_shared<ChCollisionShapeBox>(mat, 40, 80, 2);
    auto shape5 = chrono_types::make_shared<ChCollisionShapeBox>(mat, 40, 80, 2);
    container->AddCollisionShape(shape1, ChFrame<>(ChVector3d(0, -10, 0), QUNIT));
    container->AddCollisionShape(shape2, ChFrame<>(ChVector3d(-11, 0, 0), QUNIT));
    container->AddCollisionShape(shape3, ChFrame<>(ChVector3d(11, 0, 0), QUNIT));
    container->AddCollisionShape(shape4, ChFrame<>(ChVector3d(0, 0, -11), QUNIT));
    container->AddCollisionShape(shape5, ChFrame<>(ChVector3d(0, 0, 11), QUNIT));
    sys.AddBody(container);

    // Create falling rigid bodies with different shapes and mark one of each type.
    std::shared_ptr<ChBody> my_sphere;
    std::shared_ptr<ChBody> my_box;
    std::shared_ptr<ChBody> my_cylinder;
    for (int bi = 0; bi < 20; bi++) {
        auto sphere = chrono_types::make_shared<ChBodyEasySphere>(1.1, 1000, true, true, mat);
        sys.Add(sphere);
        sphere->SetPos(ChVector3d(-5 + ChRandom::Get() * 10, 4 + bi * 0.05, -5 + ChRandom::Get() * 10));
        if (bi == 0) {
            sphere->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
            my_sphere = sphere;
        }

        auto box = chrono_types::make_shared<ChBodyEasyBox>(1.5, 1.5, 1.5, 100, true, true, mat);
        sys.Add(box);
        box->SetPos(ChVector3d(-5 + ChRandom::Get() * 10, 4 + bi * 0.05, -5 + ChRandom::Get() * 10));
        if (bi == 0) {
            box->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/cubetexture_bluewhite.png"));
            my_box = box;
        }

        auto cylinder = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, 0.75, 0.5, 100, true, true, mat);
        sys.Add(cylinder);
        cylinder->SetPos(ChVector3d(-5 + ChRandom::Get() * 10, 4 + bi * 0.05, -5 + ChRandom::Get() * 10));
        if (bi == 0) {
            cylinder->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/pinkwhite.png"));
            my_cylinder = cylinder;
        }
    }

    // Create the Irrlicht visualization sys
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Number of collisions");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 14, -20));
    vis->AddTypicalLights();

    // Create the contact manager.
    ContactManager manager(&sys);

    // Simulation loop.
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(0.02);

        // Process current collisions and report number of contacts on a few bodies.
        manager.Process();
        std::cout << "Time: " << sys.GetChTime();
        std::cout << "   container: " << manager.GetNumContacts(container);
        std::cout << "   my_sphere: " << manager.GetNumContacts(my_sphere);
        std::cout << "   my_box: " << manager.GetNumContacts(my_box);
        std::cout << "   my_cylinder: " << manager.GetNumContacts(my_cylinder);
        std::cout << std::endl;
    }

    return 0;
}
