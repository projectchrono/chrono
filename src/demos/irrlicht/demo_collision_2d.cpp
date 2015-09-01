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
//     - using paths for 2d collisions
//
///////////////////////////////////////////////////

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkTrajectory.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace geometry;

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

    // Create the truss:

    ChSharedPtr<ChBody> mfloor(new ChBody);
    mfloor->SetBodyFixed(true);
    application.GetSystem()->Add(mfloor);

/*
    // Create a ChBody that contains a 2D convex collision shape:

    ChSharedPtr<ChBody> mcoin(new ChBody());
    mcoin->SetPos(ChVector<>(5.5, 1, 0));
    application.GetSystem()->Add(mcoin);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order.
    // Note that the end of one segment must match the beginning of the other. 
    // For arcs, the beginning corresponds to angle1, the second to angle2.
    ChSharedPtr<ChLinePath> mpathcoin(new ChLinePath);
    ChLineArc     marcol1 (ChCoordsys<>(ChVector<>(0, 0, 0)), 0.5, 0, CH_C_PI);
    ChLineSegment msegcol1(ChVector<>(-0.5, 0, 0), ChVector<>(0.5, 0, 0));
    mpathcoin->AddSubLine(marcol1);
    mpathcoin->AddSubLine(msegcol1);

    // Add the collision shape to the body
    mcoin->GetCollisionModel()->SetSafeMargin(0.1);
    mcoin->SetCollide(true);
    mcoin->GetCollisionModel()->ClearModel();
    mcoin->GetCollisionModel()->Add2Dpath(*mpathcoin.get_ptr(), VNULL, ChMatrix33<>(1), 0.03); // 0.03 thickness
    mcoin->GetCollisionModel()->BuildModel();

    // For visualization:create a ChLineShape, a visualization asset for lines.
    ChSharedPtr<ChLineShape> mcoinasset(new ChLineShape);
    mcoinasset->SetLineGeometry(mpathcoin);
    mcoin->AddAsset(mcoinasset);
    

    // Create a ChBody that contains a 2D concave collision shape:

    ChSharedPtr<ChBody> mhole(new ChBody());
    mhole->SetPos(ChVector<>(4, 0, 0));
    mhole->SetBodyFixed(true);
    application.GetSystem()->Add(mhole);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order.
    // Note that one arc here is concave: this means that the 'counterclockwise' flag of that 
    // arc must be enabled. It will flow from angle1 to angle2 in counterclockwise way: this 
    // means 'concave' because for 2d shapes, the 'solid' part of the shape is always ON THE RIGHT
    // side of the curvilinear abscyssa.
    ChSharedPtr<ChLinePath> mpathhole(new ChLinePath);
    ChLineArc     marcol2(ChCoordsys<>(ChVector<>(0, 0, 0)), 1, -CH_C_PI, 0, true); // true = ccw arc = concave
    ChLineSegment msegcol2(ChVector<>(1, 0, 0), ChVector<>(2, 0.04, 0));
    mpathhole->AddSubLine(marcol2);
    mpathhole->AddSubLine(msegcol2);
    mpathhole->Set_closed(false);

    // Add the collision shape to the body
    mcoin->GetCollisionModel()->SetSafeMargin(0.1);
    mhole->SetCollide(true);
    mhole->GetCollisionModel()->ClearModel();
    mhole->GetCollisionModel()->Add2Dpath(*mpathhole.get_ptr(), VNULL, ChMatrix33<>(1), 0.03); // 0.01 thickness
    mhole->GetCollisionModel()->BuildModel();

    // Create a ChLineShape, a visualization asset for lines.
    ChSharedPtr<ChLineShape> mholeasset(new ChLineShape);
    mholeasset->SetLineGeometry(mpathhole);
    mhole->AddAsset(mholeasset);

*/
    //
    // EXAMPLE 2: a Geneva wheel
    //

    
    // Geneva wheel geometry data:
    int nstations = 5;
    double R = 1;
    double Ri = 0.5;
    double wi = 0.1;
    double Li = 0.8;
    ChVector<> geneva_center(-0, 0, 0);
    // compute aux data:
    double beta = (CH_C_2PI/(double)nstations); // angle width of station
    double gamma = 2*(CH_C_PI_2 - beta/2); 
    double B = R*tan(beta/2);
    ChVector<> crank_center = ChVector<>(B, R, 0) + geneva_center;
    
    // Create the rotating Gnevawheel:
    ChSharedPtr<ChBody> mgenevawheel(new ChBody());
    mgenevawheel->SetPos(geneva_center);
    mgenevawheel->SetWvel_loc(ChVector<>(0,0,-0.08));
    application.GetSystem()->Add(mgenevawheel);

    // Create a ChLinePath geometry that represents the 2D shape of the Geneva wheel.
    // It can be made of ChLineArc or ChLineSegment sub-lines. These must be added in clockwise
    // order, and the end of sub-item i must be coincident with beginning of sub-line i+1.
    ChSharedPtr<ChLinePath> mpathwheel(new ChLinePath);
    
    for (int i=0; i<nstations; ++i) {
        double alpha = -i*beta; // phase of current station
        ChVector<> p1 (-B+Ri, R, 0);
        ChVector<> p2 (-wi/2, R, 0);
        ChVector<> p3 (-wi/2, R-Li, 0);
        ChVector<> p4 ( wi/2, R-Li, 0);
        ChVector<> p5 ( wi/2, R, 0);
        ChVector<> p6 ( B-Ri, R, 0);
        ChVector<> p7 ( B, R, 0);
        ChMatrix33<> mm;
        mm.Set_A_AngAxis(alpha, VECT_Z);
        p1 = mm * p1;
        p2 = mm * p2;
        p3 = mm * p3;
        p4 = mm * p4;
        p5 = mm * p5;
        p6 = mm * p6;
        p7 = mm * p7;
        ChLineSegment mseg1(p1,p2);
        ChLineSegment mseg2(p2,p3);
        ChLineSegment mseg3(p3,p4);
        ChLineSegment mseg4(p4,p5);
        ChLineSegment mseg5(p5,p6);
        mpathwheel->AddSubLine(mseg1);
        mpathwheel->AddSubLine(mseg2);
        mpathwheel->AddSubLine(mseg3);
        mpathwheel->AddSubLine(mseg4);
        mpathwheel->AddSubLine(mseg5);
        double a1 = alpha+CH_C_PI;
        double a2 = alpha+CH_C_PI+gamma;
        ChLineArc marc0(ChCoordsys<>(p7), Ri, a1, a2, true); // ccw arc bacause concave
        mpathwheel->AddSubLine(marc0);
    }

    // Add the collision shape to the body
    mgenevawheel->GetCollisionModel()->SetSafeMargin(0.02);
    mgenevawheel->SetCollide(true);
    mgenevawheel->GetCollisionModel()->ClearModel();
    mgenevawheel->GetCollisionModel()->Add2Dpath(*mpathwheel.get_ptr());
    mgenevawheel->GetCollisionModel()->BuildModel();

    // Create a ChLineShape, a visualization asset for lines.
    ChSharedPtr<ChLineShape> mwheelasset(new ChLineShape);
    mwheelasset->SetLineGeometry(mpathwheel);
    mgenevawheel->AddAsset(mwheelasset);
    
    // Revolute constraint 
    ChSharedPtr<ChLinkLockRevolute> mrevolute (new ChLinkLockRevolute);
    mrevolute->Initialize(mgenevawheel, mfloor, ChCoordsys<>(geneva_center));
    application.GetSystem()->Add(mrevolute);


    // Create the crank:

    ChSharedPtr<ChBody> mcrank(new ChBody());
    mcrank->SetPos(crank_center);
    application.GetSystem()->Add(mcrank);

     // Create a ChLinePath geometry, and insert sub-paths in clockwise order:
    ChSharedPtr<ChLinePath> mpathcrankpin(new ChLinePath);
    ChLineArc mpin(ChCoordsys<>(ChVector<>(-B, 0, 0)), wi/2-0.005, CH_C_2PI, 0);
    mpathcrankpin->AddSubLine(mpin);

    ChSharedPtr<ChLinePath> mpathcrankstopper(new ChLinePath);
    ChLineArc     mstopperarc(ChCoordsys<>(ChVector<>(0, 0, 0)), Ri-0.005, CH_C_PI-gamma/2, -CH_C_PI+gamma/2);
    ChLineSegment mstopperve1(mstopperarc.GetEndB(),ChVector<>(0, 0, 0));
    ChLineSegment mstopperve2(ChVector<>(0, 0, 0), mstopperarc.GetEndA());
    mpathcrankstopper->AddSubLine(mstopperarc);
    mpathcrankstopper->AddSubLine(mstopperve1);
    mpathcrankstopper->AddSubLine(mstopperve2);

    // Add the collision shape to the body
    mcrank->GetCollisionModel()->SetSafeMargin(0.02);
    mcrank->SetCollide(true);
    mcrank->GetCollisionModel()->ClearModel();
    mcrank->GetCollisionModel()->Add2Dpath(*mpathcrankpin.get_ptr());
    mcrank->GetCollisionModel()->Add2Dpath(*mpathcrankstopper.get_ptr());
    mcrank->GetCollisionModel()->BuildModel();

    // Create a ChLineShape, a visualization asset for lines.
    ChSharedPtr<ChLineShape> mcrankasset(new ChLineShape);
    mcrankasset->SetLineGeometry(mpathcrankpin);
    mcrank->AddAsset(mcrankasset);
    ChSharedPtr<ChLineShape> mcrankasset2(new ChLineShape);
    mcrankasset2->SetLineGeometry(mpathcrankstopper);
    mcrank->AddAsset(mcrankasset2);

    // .. an engine between crank and truss
    ChSharedPtr<ChLinkEngine> my_crankmotor(new ChLinkEngine);
    my_crankmotor->Initialize(mcrank, mfloor, ChCoordsys<>(crank_center));
    my_crankmotor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    if (ChSharedPtr<ChFunction_Const> mfun = my_crankmotor->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
        mfun->Set_yconst(CH_C_PI / 8.0);  
    application.GetSystem()->AddLink(my_crankmotor);



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
