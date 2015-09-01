//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about
//
//     - loading a SilidWorks .py file saved with
//       the Chrono::Engine add-in,
//     - showing the system in Irrlicht.
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_python/ChPython.h"

#include <irrlicht.h>

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::collision;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystem mphysicalSystem;

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
                                         mphysicalSystem);  // note, don't type the .py suffix in filename..

    } catch (ChException myerror) {
        GetLog() << myerror.what();
    }

    // From this point, your ChSystem has been populated with objects and
    // assets load from the .py files. So you can proceed and fetch
    // single items, modify them, or add constraints between them, etc.
    // For example you can add other bodies, etc.

    // Log out all the names of the items inserted in the system:
    GetLog() << "SYSTEM ITEMS: \n";
    mphysicalSystem.ShowHierarchy(GetLog());

    ChSystem::IteratorPhysicsItems myiter = mphysicalSystem.IterBeginPhysicsItems();
    while (myiter.HasItem()) {
        GetLog() << "item:" << (*myiter)->GetRTTI()->GetName() << "\n";
        ++myiter;
    }

    // Fetch some bodies, given their names,
    // and apply forces/constraints/etc
    ChSharedPtr<ChPhysicsItem> myitemE = mphysicalSystem.Search("escape_wheel^escapement-1");
    ChSharedPtr<ChPhysicsItem> myitemA = mphysicalSystem.Search("truss^escapement-1");
    ChSharedPtr<ChPhysicsItem> myitemB = mphysicalSystem.Search("balance^escapement-1");
    ChSharedPtr<ChPhysicsItem> myitemC = mphysicalSystem.Search("anchor^escapement-1");
    if ((!myitemE.IsNull() && myitemE.IsType<ChBody>()) && (!myitemA.IsNull() && myitemA.IsType<ChBody>()) &&
        (!myitemB.IsNull() && myitemB.IsType<ChBody>()) && (!myitemC.IsNull() && myitemC.IsType<ChBody>())) {
        // Downcast to specialized type (after we checked with IsType<...> this is safe)
        ChSharedPtr<ChBody> mescape_wheel = myitemE.DynamicCastTo<ChBody>();
        ChSharedPtr<ChBody> mtruss = myitemA.DynamicCastTo<ChBody>();
        ChSharedPtr<ChBody> mbalance = myitemB.DynamicCastTo<ChBody>();
        ChSharedPtr<ChBody> manchor = myitemC.DynamicCastTo<ChBody>();

        // Set a constant torque to escape wheel, in a
        // very simple way:
        mescape_wheel->Set_Scr_torque(ChVector<>(0, -0.03, 0));

        // Add a torsional spring
        ChSharedPtr<ChLinkLockFree> mspring(new ChLinkLockFree);
        mspring->Initialize(mtruss, mbalance, CSYSNORM);  // origin does not matter, it's only torque
        mspring->GetForce_Ry()->Set_K(0.24);
        mspring->GetForce_Ry()->Set_active(1);
        mphysicalSystem.Add(mspring);

        // Set an initial angular velocity to the balance:
        mbalance->SetWvel_par(ChVector<>(0, 5, 0));

        // Set no friction in all parts
        mbalance->GetMaterialSurface()->SetFriction(0);
        mescape_wheel->GetMaterialSurface()->SetFriction(0);
        manchor->GetMaterialSurface()->SetFriction(0);
    } else
        GetLog() << "Error: cannot one or more objects from their names in the C::E system! \n\n";

    //
    // THE VISUALIZATION
    //

    // Now, suppose one is interested in showing an animation of
    // the simulated system. There are different options, for instance
    // one could use the unit_POSTPROCESSING approach for rendering in
    // POVray, or you can open an Irrlicht 3D realtime view and show
    // it, as in the following example code:

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Import a SolidWorks system", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalCamera(vector3df(0, 0.25, 0.25), vector3df(0, 0, -0.1));
    application.AddLightWithShadow(vector3df(-0.5, 0.5, 0), vector3df(0, 0, 0), 1, 0.2, 1.2, 30, 512,
                                   video::SColorf(1, 0.9, 0.9));
    application.AddLightWithShadow(vector3df(0.5, 0.5, 0.5), vector3df(0, 0, 0), 1, 0.2, 1.2, 30, 512,
                                   video::SColorf(0.6, 0.8, 1));

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
    // for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)

    application.AddShadowAll();

    //
    // THE SIMULATION LOOP
    //

    // set a low stabilization value because objects are small!
    application.GetSystem()->SetMaxPenetrationRecoverySpeed(0.002);

    application.SetStepManage(true);
    application.SetTimestep(0.002);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
