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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Demo code about
// - loading a .py file saved with the Chrono SolidWorks add-in
// - simulating the system with Irrlicht run-time visualization
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"

#include "chrono_parsers/ChParserPython.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::parsers;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Cache current path to Chrono data files
    auto data_path = GetChronoDataPath();

    // Create a Chrono system
    ChSystemNSC sys;

    // Set the collision margins.
    // This is expecially important for very large or very small objects!
    // Do this before creating shapes.
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    ChCollisionModel::SetDefaultSuggestedMargin(0.001);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create a Chrono Python engine to parse files created using the Chrono SolidWorks add-in
    ChPythonEngine my_python;

    std::cout << "\n\n Check availability of PyChrono in the system\n";
    try {
        my_python.Run("import pychrono as chrono");
    } catch (std::exception) {
        std::cout << "PyChrono must be available in the system to run this demo." << std::endl;
        return 0;
    }

    try {
        // This is the instruction that loads the .py (as saved from SolidWorks) and fills the system.
        // In this example, we load a mechanical system that represents a (quite simplified and approximated) clock
        // escapement, that has been modeled in SolidWorks and saved using the Chrono SolidWorks add-in.
        my_python.ImportSolidWorksSystem(GetChronoDataFile("solidworks/swiss_escapement.py"), sys);
    } catch (std::exception myerror) {
        std::cerr << myerror.what() << std::endl;
    }

    // At this point, the ChSystem has been populated with objects and assets loaded from the .py files.
    // It is now possible to fetch single items, modify them, add constraints between them, or else add new physics
    // items to the ChSystem.

    // Log the names of all items inserted in the system
    std::cout << "SYSTEM ITEMS:" << std::endl;
    sys.ShowHierarchy(std::cout);

    // Restore path to Chrono data files (modified by the Python importer)
    SetChronoDataPath(data_path);

    for (auto body : sys.GetBodies()) {
        std::cout << "item:" << typeid(body).name() << std::endl;
    }
    for (auto link : sys.GetLinks()) {
        std::cout << "item:" << typeid(link).name() << std::endl;
    }
    for (auto& mesh : sys.GetMeshes()) {
        std::cout << "item:" << typeid(mesh).name() << std::endl;
    }
    for (auto ph : sys.GetOtherPhysicsItems()) {
        std::cout << "item:" << typeid(ph).name() << std::endl;
    }

    // Fetch some bodies, given their names, and apply forces, constraints, etc.
    std::shared_ptr<ChPhysicsItem> myitemE = sys.Search("escape_wheel-1");
    std::shared_ptr<ChPhysicsItem> myitemA = sys.Search("truss-1");
    std::shared_ptr<ChPhysicsItem> myitemB = sys.Search("balance-1");
    std::shared_ptr<ChPhysicsItem> myitemC = sys.Search("anchor-1");
    auto mescape_wheel = std::dynamic_pointer_cast<ChBody>(myitemE);
    auto mtruss = std::dynamic_pointer_cast<ChBody>(myitemA);
    auto mbalance = std::dynamic_pointer_cast<ChBody>(myitemB);
    auto manchor = std::dynamic_pointer_cast<ChBody>(myitemC);

    // Create a contact material with zero friction which will be shared by all parts
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0);

    if (mescape_wheel && mtruss && mbalance && manchor) {
        // Set a constant torque to escape wheel, in a very simple way
        mescape_wheel->EmptyAccumulators();
        mescape_wheel->AccumulateTorque(ChVector3d(0, -0.03, 0), false);

        // Add a torsional spring
        std::shared_ptr<ChLinkLockFree> mspring(new ChLinkLockFree);
        mspring->Initialize(mtruss, mbalance, ChFrame<>());  // origin does not matter, it's only torque
        mspring->ForceRy().SetSpringCoefficient(0.24);
        mspring->ForceRy().SetActive(1);
        sys.Add(mspring);

        // Set an initial angular velocity to the balance:
        mbalance->SetAngVelParent(ChVector3d(0, 5, 0));

        // Set no friction in all parts
        assert(mbalance->GetCollisionModel());
        assert(mescape_wheel->GetCollisionModel());
        assert(manchor->GetCollisionModel());
        mbalance->GetCollisionModel()->SetAllShapesMaterial(mat);
        mescape_wheel->GetCollisionModel()->SetAllShapesMaterial(mat);
        manchor->GetCollisionModel()->SetAllShapesMaterial(mat);
    } else
        std::cerr << std::endl
                  << std::endl
                  << "ERROR: cannot find one or more objects from their names in the Chrono system!\n"
                  << std::endl;

    // Irrlicht run-time visualization
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Solidworks import demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 0.25, 0.25), ChVector3d(0, 0, -0.1));
    vis->AddTypicalLights();
    //vis->AddLightWithShadow(ChVector3d(-0.5, 0.5, 0.0), ChVector3d(0, 0, 0), 1, 0.2, 1.2, 30, 512,
    //                        ChColor(1.0f, 0.9f, 0.9f));
    //vis->AddLightWithShadow(ChVector3d(+0.5, 0.5, 0.5), ChVector3d(0, 0, 0), 1, 0.2, 1.2, 30, 512,
    //                        ChColor(0.6f, 0.8f, 1.0f));
    vis->EnableShadows();

    // Simulation loop
    sys.SetMaxPenetrationRecoverySpeed(0.002);  // low stabilization value because objects are small!

    double timestep = 0.002;
    ChRealtimeStepTimer realtime_timer;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
