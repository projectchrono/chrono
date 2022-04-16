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
// Demo code about using paths for 2d collisions
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkTrajectory.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
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
    ChSystemNSC sys;

    // Contact material (shared among all collision shapes)
    auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    //
    // EXAMPLE 1:
    //

    // Create the truss:
    auto mfloor = chrono_types::make_shared<ChBody>();
    mfloor->SetBodyFixed(true);
    sys.Add(mfloor);

    // Create a ChBody that contains a 2D convex collision shape:

    auto mcoin = chrono_types::make_shared<ChBody>();
    mcoin->SetPos(ChVector<>(5.5, 1, 0));
    sys.Add(mcoin);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order.
    // Note that the end of one segment must match the beginning of the other.
    // For arcs, the beginning corresponds to angle1, the second to angle2.
    auto mpathcoin = chrono_types::make_shared<ChLinePath>();
    ChLineArc marcol1(ChCoordsys<>(ChVector<>(0, 0, 0)), 0.5, 0, CH_C_PI);
    ChLineSegment msegcol1(ChVector<>(-0.5, 0, 0), ChVector<>(0.5, 0, 0));
    mpathcoin->AddSubLine(marcol1);
    mpathcoin->AddSubLine(msegcol1);

    // Add the collision shape to the body
    mcoin->GetCollisionModel()->SetSafeMargin(0.1);
    mcoin->SetCollide(true);
    mcoin->GetCollisionModel()->ClearModel();
    mcoin->GetCollisionModel()->Add2Dpath(mat, mpathcoin, VNULL, ChMatrix33<>(1), 0.03);  // 0.03 thickness
    mcoin->GetCollisionModel()->BuildModel();

    // For visualization:create a ChLineShape, a visualization asset for lines.
    auto mcoinasset = chrono_types::make_shared<ChLineShape>();
    mcoinasset->SetLineGeometry(mpathcoin);
    mcoin->AddVisualShape(mcoinasset);

    // Create a ChBody that contains a 2D concave collision shape:

    auto mhole = chrono_types::make_shared<ChBody>();
    mhole->SetPos(ChVector<>(4, 0, 0));
    mhole->SetBodyFixed(true);
    sys.Add(mhole);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order.
    // Note that one arc here is concave: this means that the 'counterclockwise' flag of that
    // arc must be enabled. It will flow from angle1 to angle2 in counterclockwise way: this
    // means 'concave' because for 2d shapes, the 'solid' part of the shape is always ON THE RIGHT
    // side of the curvilinear abscyssa.
    auto mpathhole = chrono_types::make_shared<ChLinePath>();
    ChLineArc marcol2(ChCoordsys<>(ChVector<>(0, 0, 0)), 1, -CH_C_PI, 0, true);  // true = ccw arc = concave
    ChLineSegment msegcol2(ChVector<>(1, 0, 0), ChVector<>(2, 0.04, 0));
    mpathhole->AddSubLine(marcol2);
    mpathhole->AddSubLine(msegcol2);
    mpathhole->Set_closed(false);

    // Add the collision shape to the body
    mcoin->GetCollisionModel()->SetSafeMargin(0.1);
    mhole->SetCollide(true);
    mhole->GetCollisionModel()->ClearModel();
    mhole->GetCollisionModel()->Add2Dpath(mat, mpathhole, VNULL, ChMatrix33<>(1), 0.03);  // 0.01 thickness
    mhole->GetCollisionModel()->BuildModel();

    // Create a ChLineShape, a visualization asset for lines.
    auto mholeasset = chrono_types::make_shared<ChLineShape>();
    mholeasset->SetLineGeometry(mpathhole);
    mhole->AddVisualShape(mholeasset);

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
    double beta = (CH_C_2PI / (double)nstations);  // angle width of station
    double gamma = 2 * (CH_C_PI_2 - beta / 2);
    double B = R * tan(beta / 2);
    ChVector<> crank_center = ChVector<>(B, R, 0) + geneva_center;

    // Create the rotating Genevawheel:
    auto mgenevawheel = chrono_types::make_shared<ChBody>();
    mgenevawheel->SetPos(geneva_center);
    mgenevawheel->SetWvel_loc(ChVector<>(0, 0, -0.08));
    sys.Add(mgenevawheel);

    // Create a ChLinePath geometry that represents the 2D shape of the Geneva wheel.
    // It can be made of ChLineArc or ChLineSegment sub-lines. These must be added in clockwise
    // order, and the end of sub-item i must be coincident with beginning of sub-line i+1.
    auto mpathwheel = chrono_types::make_shared<ChLinePath>();

    for (int i = 0; i < nstations; ++i) {
        double alpha = -i * beta;  // phase of current station
        ChVector<> p1(-B + Ri, R, 0);
        ChVector<> p2(-wi / 2, R, 0);
        ChVector<> p3(-wi / 2, R - Li, 0);
        ChVector<> p4(wi / 2, R - Li, 0);
        ChVector<> p5(wi / 2, R, 0);
        ChVector<> p6(B - Ri, R, 0);
        ChVector<> p7(B, R, 0);
        ChMatrix33<> mm(alpha, VECT_Z);
        p1 = mm * p1;
        p2 = mm * p2;
        p3 = mm * p3;
        p4 = mm * p4;
        p5 = mm * p5;
        p6 = mm * p6;
        p7 = mm * p7;
        ChLineSegment mseg1(p1, p2);
        ChLineSegment mseg2(p2, p3);
        ChLineSegment mseg3(p3, p4);
        ChLineSegment mseg4(p4, p5);
        ChLineSegment mseg5(p5, p6);
        mpathwheel->AddSubLine(mseg1);
        mpathwheel->AddSubLine(mseg2);
        mpathwheel->AddSubLine(mseg3);
        mpathwheel->AddSubLine(mseg4);
        mpathwheel->AddSubLine(mseg5);
        double a1 = alpha + CH_C_PI;
        double a2 = alpha + CH_C_PI + gamma;
        ChLineArc marc0(ChCoordsys<>(p7), Ri, a1, a2, true);  // ccw arc bacause concave
        mpathwheel->AddSubLine(marc0);
    }

    // Add the collision shape to the body
    mgenevawheel->GetCollisionModel()->SetSafeMargin(0.02);
    mgenevawheel->SetCollide(true);
    mgenevawheel->GetCollisionModel()->ClearModel();
    mgenevawheel->GetCollisionModel()->Add2Dpath(mat, mpathwheel);
    mgenevawheel->GetCollisionModel()->BuildModel();

    // Create a ChLineShape, a visualization asset for lines.
    auto mwheelasset = chrono_types::make_shared<ChLineShape>();
    mwheelasset->SetLineGeometry(mpathwheel);
    mgenevawheel->AddVisualShape(mwheelasset);

    // Revolute constraint
    auto mrevolute = chrono_types::make_shared<ChLinkLockRevolute>();
    mrevolute->Initialize(mgenevawheel, mfloor, ChCoordsys<>(geneva_center));
    sys.Add(mrevolute);

    // Create the crank:

    auto mcrank = chrono_types::make_shared<ChBody>();
    mcrank->SetPos(crank_center);
    sys.Add(mcrank);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order:
    auto mpathcrankpin = chrono_types::make_shared<ChLinePath>();
    ChLineArc mpin(ChCoordsys<>(ChVector<>(-B, 0, 0)), wi / 2 - 0.005, CH_C_2PI, 0);
    mpathcrankpin->AddSubLine(mpin);

    auto mpathcrankstopper = chrono_types::make_shared<ChLinePath>();
    ChLineArc mstopperarc(ChCoordsys<>(ChVector<>(0, 0, 0)), Ri - 0.005, CH_C_PI - gamma / 2, -CH_C_PI + gamma / 2);
    ChLineSegment mstopperve1(mstopperarc.GetEndB(), ChVector<>(0, 0, 0));
    ChLineSegment mstopperve2(ChVector<>(0, 0, 0), mstopperarc.GetEndA());
    mpathcrankstopper->AddSubLine(mstopperarc);
    mpathcrankstopper->AddSubLine(mstopperve1);
    mpathcrankstopper->AddSubLine(mstopperve2);

    // Add the collision shape to the body
    mcrank->GetCollisionModel()->SetSafeMargin(0.02);
    mcrank->SetCollide(true);
    mcrank->GetCollisionModel()->ClearModel();
    mcrank->GetCollisionModel()->Add2Dpath(mat, mpathcrankpin);
    mcrank->GetCollisionModel()->Add2Dpath(mat, mpathcrankstopper);
    mcrank->GetCollisionModel()->BuildModel();

    // Create a ChLineShape, a visualization asset for lines.
    auto mcrankasset = chrono_types::make_shared<ChLineShape>();
    mcrankasset->SetLineGeometry(mpathcrankpin);
    mcrank->AddVisualShape(mcrankasset);
    auto mcrankasset2 = chrono_types::make_shared<ChLineShape>();
    mcrankasset2->SetLineGeometry(mpathcrankstopper);
    mcrank->AddVisualShape(mcrankasset2);

    // .. a motor between crank and truss
    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_motor->Initialize(mcrank, mfloor, ChFrame<>(crank_center));
    my_motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 8.0));
    sys.AddLink(my_motor);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    sys.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Paths");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 4, -6));
    vis->AddTypicalLights();

    // This means that contactforces will be shown in Irrlicht application
    vis->SetSymbolScale(0.2);
    vis->EnableContactDrawing(IrrContactsDrawMode::CONTACT_NORMALS);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();

        sys.DoStepDynamics(0.01);
    }

    return 0;
}
