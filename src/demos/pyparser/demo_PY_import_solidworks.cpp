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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - loading a SolidWorks .py file saved with the Chrono::Engine add-in,
// - showing the system in Irrlicht.
//
// =============================================================================

#include "chrono_pyparser/ChPython.h"
#include "chrono/core/ChRealtimeStep.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::collision;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC sys;

    // Set the collision margins. This is expecially important for
    // very large or very small objects! Do this before creating shapes.
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    ChCollisionModel::SetDefaultSuggestedMargin(0.001);

    //
    // LOAD THE SYSTEM
    //

    // The Python engine. This is necessary in order to parse the files that
    // have been saved using the SolidWorks add-in for Chrono::Engine.

    ChPythonEngine my_python;

    try {
        // This is the instruction that loads the .py (as saved from SolidWorks) and
        // fills the system.
        //   In this example, we load a mechanical system that represents
        // a (quite simplified & approximated) clock escapement, that has been
        // modeled in SolidWorks and saved using the Chrono Add-in for SolidWorks.

        my_python.ImportSolidWorksSystem(GetChronoDataFile("solid_works/swiss_escapement").c_str(),
                                         sys);  // note, don't type the .py suffix in filename..

    } catch (const ChException& myerror) {
        GetLog() << myerror.what();
    }

    // From this point, your ChSystem has been populated with objects and
    // assets load from the .py files. So you can proceed and fetch
    // single items, modify them, or add constraints between them, etc.
    // For example you can add other bodies, etc.

    // Log out all the names of the items inserted in the system:
    GetLog() << "SYSTEM ITEMS: \n";
    sys.ShowHierarchy(GetLog());

    for (auto body : sys.Get_bodylist()) {
        GetLog() << "item:" << typeid(body).name() << "\n";
    }
    for (auto link : sys.Get_linklist()) {
        GetLog() << "item:" << typeid(link).name() << "\n";
    }
    for (auto& mesh : sys.Get_meshlist()) {
        GetLog() << "item:" << typeid(mesh).name() << "\n";
    }
    for (auto ph : sys.Get_otherphysicslist()) {
        GetLog() << "item:" << typeid(ph).name() << "\n";
    }

    // Fetch some bodies, given their names, and apply forces/constraints/etc
    std::shared_ptr<ChPhysicsItem> myitemE = sys.Search("escape_wheel-1");
    std::shared_ptr<ChPhysicsItem> myitemA = sys.Search("truss-1");
    std::shared_ptr<ChPhysicsItem> myitemB = sys.Search("balance-1");
    std::shared_ptr<ChPhysicsItem> myitemC = sys.Search("anchor-1");
    auto mescape_wheel = std::dynamic_pointer_cast<ChBody>(myitemE);
    auto mtruss = std::dynamic_pointer_cast<ChBody>(myitemA);
    auto mbalance = std::dynamic_pointer_cast<ChBody>(myitemB);
    auto manchor = std::dynamic_pointer_cast<ChBody>(myitemC);

    // Create a contact material with zero friction which will be shared by all parts
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mat->SetFriction(0);

    if (mescape_wheel && mtruss && mbalance && manchor) {
        // Set a constant torque to escape wheel, in a very simple way:
        mescape_wheel->Empty_forces_accumulators();
        mescape_wheel->Accumulate_torque(ChVector<>(0, -0.03, 0), false);

        // Add a torsional spring
        std::shared_ptr<ChLinkLockFree> mspring(new ChLinkLockFree);
        mspring->Initialize(mtruss, mbalance, CSYSNORM);  // origin does not matter, it's only torque
        mspring->GetForce_Ry().SetK(0.24);
        mspring->GetForce_Ry().SetActive(1);
        sys.Add(mspring);

        // Set an initial angular velocity to the balance:
        mbalance->SetWvel_par(ChVector<>(0, 5, 0));

        // Set no friction in all parts
        mbalance->GetCollisionModel()->SetAllShapesMaterial(mat);
        mescape_wheel->GetCollisionModel()->SetAllShapesMaterial(mat);
        manchor->GetCollisionModel()->SetAllShapesMaterial(mat);
    } else
        GetLog() << "\n\nERROR: cannot find one or more objects from their names in the Chrono system!\n\n";

    //
    // THE VISUALIZATION
    //

    // Now, suppose one is interested in showing an animation of
    // the simulated system. There are different options, for instance
    // one could use the unit_POSTPROCESS approach for rendering in
    // POVray, or you can open an Irrlicht 3D realtime view and show
    // it, as in the following example code:


    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Collision visualization demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0.25, 0.25), ChVector<>(0, 0, -0.1));
    vis->AddTypicalLights();
    vis->AddLightWithShadow(ChVector<>(-0.5, 0.5, 0.0), ChVector<>(0, 0, 0), 1, 0.2, 1.2, 30, 512, ChColor(1.0f, 0.9f, 0.9f));
    vis->AddLightWithShadow(ChVector<>(+0.5, 0.5, 0.5), ChVector<>(0, 0, 0), 1, 0.2, 1.2, 30, 512, ChColor(0.6f, 0.8f, 1.0f));
    vis->EnableShadows();

    //
    // THE SIMULATION LOOP
    //

    // set a low stabilization value because objects are small!
    sys.SetMaxPenetrationRecoverySpeed(0.002);

    // Simulation loop
    double timestep = 0.002;
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();
        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
