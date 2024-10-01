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
#include "chrono/physics/ChLinkLockTrajectory.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Contact material (shared among all collision shapes)
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // EXAMPLE 1:

    // Create the truss:
    auto mfloor = chrono_types::make_shared<ChBody>();
    mfloor->SetFixed(true);
    sys.Add(mfloor);

    // Create a ChBody that contains a 2D convex collision shape:

    auto mcoin = chrono_types::make_shared<ChBody>();
    mcoin->SetPos(ChVector3d(5.5, 1, 0));
    sys.Add(mcoin);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order.
    // Note that the end of one segment must match the beginning of the other.
    // For arcs, the beginning corresponds to angle1, the second to angle2.
    auto mpathcoin = chrono_types::make_shared<ChLinePath>();
    ChLineArc marcol1(ChCoordsys<>(ChVector3d(0, 0, 0)), 0.5, 0, CH_PI);
    ChLineSegment msegcol1(ChVector3d(-0.5, 0, 0), ChVector3d(0.5, 0, 0));
    mpathcoin->AddSubLine(marcol1);
    mpathcoin->AddSubLine(msegcol1);

    // Add the collision shape to the body
    mcoin->EnableCollision(true);
    auto coin_coll = chrono_types::make_shared<ChCollisionShapePath2D>(mat, mpathcoin);
    mcoin->AddCollisionShape(coin_coll, ChFrame<>());
    mcoin->GetCollisionModel()->SetSafeMargin(0.1f);

    // For visualization:create a ChVisualShapeLine, a visualization asset for lines.
    auto mcoinasset = chrono_types::make_shared<ChVisualShapeLine>();
    mcoinasset->SetLineGeometry(mpathcoin);
    mcoin->AddVisualShape(mcoinasset);

    // Create a ChBody that contains a 2D concave collision shape:

    auto mhole = chrono_types::make_shared<ChBody>();
    mhole->SetPos(ChVector3d(4, 0, 0));
    mhole->SetFixed(true);
    sys.Add(mhole);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order.
    // Note that one arc here is concave: this means that the 'counterclockwise' flag of that
    // arc must be enabled. It will flow from angle1 to angle2 in counterclockwise way: this
    // means 'concave' because for 2d shapes, the 'solid' part of the shape is always ON THE RIGHT
    // side of the curvilinear abscyssa.
    auto mpathhole = chrono_types::make_shared<ChLinePath>();
    ChLineArc marcol2(ChCoordsys<>(ChVector3d(0, 0, 0)), 1, -CH_PI, 0, true);  // true = ccw arc = concave
    ChLineSegment msegcol2(ChVector3d(1, 0, 0), ChVector3d(2, 0.04, 0));
    mpathhole->AddSubLine(marcol2);
    mpathhole->AddSubLine(msegcol2);
    mpathhole->SetClosed(false);

    // Add the collision shape to the body
    mhole->EnableCollision(true);
    auto hole_coll = chrono_types::make_shared<ChCollisionShapePath2D>(mat, mpathhole);
    mhole->AddCollisionShape(hole_coll, ChFrame<>());
    mhole->GetCollisionModel()->SetSafeMargin(0.1f);

    // Create a ChVisualShapeLine, a visualization asset for lines.
    auto mholeasset = chrono_types::make_shared<ChVisualShapeLine>();
    mholeasset->SetLineGeometry(mpathhole);
    mhole->AddVisualShape(mholeasset);

    // EXAMPLE 2: a Geneva wheel

    // Geneva wheel geometry data:
    int nstations = 5;
    double R = 1;
    double Ri = 0.5;
    double wi = 0.1;
    double Li = 0.8;
    ChVector3d geneva_center(-0, 0, 0);
    // compute aux data:
    double beta = (CH_2PI / (double)nstations);  // angle width of station
    double gamma = 2 * (CH_PI_2 - beta / 2);
    double B = R * std::tan(beta / 2);
    ChVector3d crank_center = ChVector3d(B, R, 0) + geneva_center;

    // Create the rotating Genevawheel:
    auto mgenevawheel = chrono_types::make_shared<ChBody>();
    mgenevawheel->SetPos(geneva_center);
    mgenevawheel->SetAngVelLocal(ChVector3d(0, 0, -0.08));
    sys.Add(mgenevawheel);

    // Create a ChLinePath geometry that represents the 2D shape of the Geneva wheel.
    // It can be made of ChLineArc or ChLineSegment sub-lines. These must be added in clockwise
    // order, and the end of sub-item i must be coincident with beginning of sub-line i+1.
    auto mpathwheel = chrono_types::make_shared<ChLinePath>();

    for (int i = 0; i < nstations; ++i) {
        double alpha = -i * beta;  // phase of current station
        ChVector3d p1(-B + Ri, R, 0);
        ChVector3d p2(-wi / 2, R, 0);
        ChVector3d p3(-wi / 2, R - Li, 0);
        ChVector3d p4(wi / 2, R - Li, 0);
        ChVector3d p5(wi / 2, R, 0);
        ChVector3d p6(B - Ri, R, 0);
        ChVector3d p7(B, R, 0);
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
        double a1 = alpha + CH_PI;
        double a2 = alpha + CH_PI + gamma;
        ChLineArc marc0(ChCoordsys<>(p7), Ri, a1, a2, true);  // ccw arc bacause concave
        mpathwheel->AddSubLine(marc0);
    }

    // Add the collision shape to the body
    mgenevawheel->EnableCollision(true);
    auto genevawheel_coll = chrono_types::make_shared<ChCollisionShapePath2D>(mat, mpathwheel);
    mgenevawheel->AddCollisionShape(genevawheel_coll, ChFrame<>());
    mgenevawheel->GetCollisionModel()->SetSafeMargin(0.02f);

    // Create a ChVisualShapeLine, a visualization asset for lines.
    auto mwheelasset = chrono_types::make_shared<ChVisualShapeLine>();
    mwheelasset->SetLineGeometry(mpathwheel);
    mgenevawheel->AddVisualShape(mwheelasset);

    // Revolute constraint
    auto mrevolute = chrono_types::make_shared<ChLinkLockRevolute>();
    mrevolute->Initialize(mgenevawheel, mfloor, ChFrame<>(geneva_center));
    sys.Add(mrevolute);

    // Create the crank:

    auto mcrank = chrono_types::make_shared<ChBody>();
    mcrank->SetPos(crank_center);
    sys.Add(mcrank);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order:
    auto mpathcrankpin = chrono_types::make_shared<ChLinePath>();
    ChLineArc mpin(ChCoordsys<>(ChVector3d(-B, 0, 0)), wi / 2 - 0.005, CH_2PI, 0);
    mpathcrankpin->AddSubLine(mpin);

    auto mpathcrankstopper = chrono_types::make_shared<ChLinePath>();
    ChLineArc mstopperarc(ChCoordsys<>(ChVector3d(0, 0, 0)), Ri - 0.005, CH_PI - gamma / 2, -CH_PI + gamma / 2);
    ChLineSegment mstopperve1(mstopperarc.GetEndB(), ChVector3d(0, 0, 0));
    ChLineSegment mstopperve2(ChVector3d(0, 0, 0), mstopperarc.GetEndA());
    mpathcrankstopper->AddSubLine(mstopperarc);
    mpathcrankstopper->AddSubLine(mstopperve1);
    mpathcrankstopper->AddSubLine(mstopperve2);

    // Add the collision shape to the body
    mcrank->EnableCollision(true);
    auto crankpin_coll = chrono_types::make_shared<ChCollisionShapePath2D>(mat, mpathcrankpin);
    auto crankstopper_coll = chrono_types::make_shared<ChCollisionShapePath2D>(mat, mpathcrankstopper);
    mcrank->AddCollisionShape(crankpin_coll, ChFrame<>());
    mcrank->AddCollisionShape(crankstopper_coll, ChFrame<>());
    mcrank->GetCollisionModel()->SetSafeMargin(0.02f);

    // Create a ChVisualShapeLine, a visualization asset for lines.
    auto mcrankasset = chrono_types::make_shared<ChVisualShapeLine>();
    mcrankasset->SetLineGeometry(mpathcrankpin);
    mcrank->AddVisualShape(mcrankasset);

    auto mcrankasset2 = chrono_types::make_shared<ChVisualShapeLine>();
    mcrankasset2->SetLineGeometry(mpathcrankstopper);
    mcrank->AddVisualShape(mcrankasset2);

    // .. a motor between crank and truss
    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_motor->Initialize(mcrank, mfloor, ChFrame<>(crank_center));
    my_motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(CH_PI / 8.0));
    sys.AddLink(my_motor);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Paths");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 4, -6));
    vis->AddTypicalLights();

    // This means that contactforces will be shown in Irrlicht application
    vis->SetSymbolScale(0.2);
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(0.01);
    }

    return 0;
}
