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
// Demo code about using paths for defining trajectories
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkTrajectory.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Paths", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 4, -6));

    // This means that contactforces will be shown in Irrlicht application
    application.SetSymbolscale(0.2);
    application.SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_NORMALS);

    //
    // EXAMPLE 1:
    //

    // Create a ChBody that contains the trajectory (a floor, fixed body)

    auto mfloor = std::make_shared<ChBodyEasyBox>(3, 0.2, 3, 1000);
    mfloor->SetBodyFixed(true);
    // mfloor->SetRot(Q_from_AngAxis(0.1,VECT_Z));
    application.GetSystem()->Add(mfloor);

    // Create a ChLinePath geometry, and insert sub-paths:
    auto mpath = std::make_shared<ChLinePath>();
    ChLineSegment mseg1(ChVector<>(1, 2, 0), ChVector<>(2, 2, 0));
    mpath->AddSubLine(mseg1);
    ChLineArc marc1(ChCoordsys<>(ChVector<>(2, 2.5, 0)), 0.5, -CH_C_PI_2, CH_C_PI_2, true);
    mpath->AddSubLine(marc1);
    ChLineSegment mseg2(ChVector<>(2, 3, 0), ChVector<>(1, 3, 0));
    mpath->AddSubLine(mseg2);
    ChLineArc marc2(ChCoordsys<>(ChVector<>(1, 2.5, 0)), 0.5, CH_C_PI_2, -CH_C_PI_2, true);
    mpath->AddSubLine(marc2);
    mpath->Set_closed(true);

    // Create a ChLineShape, a visualization asset for lines.
    // The ChLinePath is a special type of ChLine and it can be visualized.
    auto mpathasset = std::make_shared<ChLineShape>();
    mpathasset->SetLineGeometry(mpath);
    mfloor->AddAsset(mpathasset);

    // Create a body that will follow the trajectory

    auto mpendulum = std::make_shared<ChBodyEasyBox>(0.1, 1, 0.1, 1000);
    mpendulum->SetPos(ChVector<>(1, 1.5, 0));
    application.GetSystem()->Add(mpendulum);

    // The trajectory constraint:

    auto mtrajectory = std::make_shared<ChLinkTrajectory>();

    // Define which parts are connected (the trajectory is considered in the 2nd body).
    mtrajectory->Initialize(mpendulum,              // body1 that follows the trajectory
                            mfloor,                 // body2 that 'owns' the trajectory
                            ChVector<>(0, 0.5, 0),  // point on body1 that will follow the trajectory
                            mpath                   // the trajectory (reuse the one already added to body2 as asset)
                            );

    // Optionally, set a function that gets the curvilinear
    // abscyssa s of the line, as a function of time s(t). By default it was simply  s=t.
    auto mspacefx = std::make_shared<ChFunction_Ramp>(0, 0.5);
    mtrajectory->Set_space_fx(mspacefx);

    application.GetSystem()->Add(mtrajectory);

    //
    // EXAMPLE 2:
    //

    // Create a ChBody that contains the trajectory

    auto mwheel = std::make_shared<ChBody>();
    mwheel->SetPos(ChVector<>(-3, 2, 0));
    application.GetSystem()->Add(mwheel);

    // Create a motor that spins the wheel
    auto motor = std::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(mwheel, mfloor, ChFrame<>(ChVector<>(-3, 2, 0)));
    motor->SetSpeedFunction(std::make_shared<ChFunction_Const>(CH_C_PI / 4.0));
    mphysicalSystem.AddLink(motor);

    // Create a ChLinePath geometry, and insert sub-paths:
    auto mglyph = std::make_shared<ChLinePath>();
    ChLineSegment ms1(ChVector<>(-0.5, -0.5, 0), ChVector<>(0.5, -0.5, 0));
    mglyph->AddSubLine(ms1);
    ChLineArc ma1(ChCoordsys<>(ChVector<>(0.5, 0, 0)), 0.5, -CH_C_PI_2, CH_C_PI_2, true);
    mglyph->AddSubLine(ma1);
    ChLineSegment ms2(ChVector<>(0.5, 0.5, 0), ChVector<>(-0.5, 0.5, 0));
    mglyph->AddSubLine(ms2);
    ChLineArc ma2(ChCoordsys<>(ChVector<>(-0.5, 0, 0)), 0.5, CH_C_PI_2, -CH_C_PI_2, true);
    mglyph->AddSubLine(ma2);
    mglyph->SetPathDuration(1);
    mglyph->Set_closed(true);

    // Create a ChLineShape, a visualization asset for lines.
    // The ChLinePath is a special type of ChLine and it can be visualized.
    auto mglyphasset = std::make_shared<ChLineShape>();
    mglyphasset->SetLineGeometry(mglyph);
    mwheel->AddAsset(mglyphasset);

    // Create a body that will slide on the glyph

    auto mpendulum2 = std::make_shared<ChBodyEasyBox>(0.1, 1, 0.1, 1000);
    mpendulum2->SetPos(ChVector<>(-3, 1, 0));
    application.GetSystem()->Add(mpendulum2);

    // The glyph constraint:

    auto mglyphconstraint = std::make_shared<ChLinkPointSpline>();

    // Define which parts are connected (the trajectory is considered in the 2nd body).
    mglyphconstraint->Initialize(mpendulum2,  // body1 that follows the trajectory
                                 mwheel,      // body2 that 'owns' the trajectory
                                 true,
                                 ChCoordsys<>(ChVector<>(0, 0.5, 0)),  // point on body1 that will follow the trajectory
                                 ChCoordsys<>());

    mglyphconstraint->Set_trajectory_line(mglyph);

    application.GetSystem()->Add(mglyphconstraint);



    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
