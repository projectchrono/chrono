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
#include "chrono/core/ChRealtimeStep.h"

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
    auto contact_mat = chrono_types::make_shared<ChContactMaterialNSC>();

    // EXAMPLE 1:

    // Create the truss:
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sys.Add(ground);

    // Create a ChBody that contains a 2D convex collision shape:

    auto coin = chrono_types::make_shared<ChBody>();
    coin->SetPos(ChVector3d(5.5, 1, 0));
    sys.Add(coin);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order.
    // Note that the end of one segment must match the beginning of the other.
    // For arcs, the beginning corresponds to angle1, the second to angle2.
    auto coin_path = chrono_types::make_shared<ChLinePath>();
    ChLineArc marcol1(ChCoordsys<>(ChVector3d(0, 0, 0)), 0.5, 0, CH_PI);
    ChLineSegment msegcol1(ChVector3d(-0.5, 0, 0), ChVector3d(0.5, 0, 0));
    coin_path->AddSubLine(marcol1);
    coin_path->AddSubLine(msegcol1);

    // Add the collision shape to the body
    coin->EnableCollision(true);
    auto coin_coll = chrono_types::make_shared<ChCollisionShapePath2D>(contact_mat, coin_path);
    coin->AddCollisionShape(coin_coll, ChFrame<>());
    coin->GetCollisionModel()->SetSafeMargin(0.1f);

    // For visualization:create a ChVisualShapeLine, a visualization asset for lines.
    auto coin_asset = chrono_types::make_shared<ChVisualShapeLine>();
    coin_asset->SetLineGeometry(coin_path);
    coin->AddVisualShape(coin_asset);

    // Create a ChBody that contains a 2D concave collision shape:

    auto hole = chrono_types::make_shared<ChBody>();
    hole->SetPos(ChVector3d(4, 0, 0));
    hole->SetFixed(true);
    sys.Add(hole);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order.
    // Note that one arc here is concave: this means that the 'counterclockwise' flag of that
    // arc must be enabled. It will flow from angle1 to angle2 in counterclockwise way: this
    // means 'concave' because for 2d shapes, the 'solid' part of the shape is always ON THE RIGHT
    // side of the curvilinear abscyssa.
    auto hole_path = chrono_types::make_shared<ChLinePath>();
    ChLineArc arc_hole(ChCoordsys<>(ChVector3d(0, 0, 0)), 1, -CH_PI, 0, true);  // true = ccw arc = concave
    ChLineSegment seg_hole(ChVector3d(1, 0, 0), ChVector3d(2, 0.04, 0));
    hole_path->AddSubLine(arc_hole);
    hole_path->AddSubLine(seg_hole);
    hole_path->SetClosed(false);

    // Add the collision shape to the body
    hole->EnableCollision(true);
    auto hole_coll = chrono_types::make_shared<ChCollisionShapePath2D>(contact_mat, hole_path);
    hole->AddCollisionShape(hole_coll, ChFrame<>());
    hole->GetCollisionModel()->SetSafeMargin(0.1f);

    // Create a ChVisualShapeLine, a visualization asset for lines.
    auto hole_asset = chrono_types::make_shared<ChVisualShapeLine>();
    hole_asset->SetLineGeometry(hole_path);
    hole->AddVisualShape(hole_asset);

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
    auto geneva_wheel = chrono_types::make_shared<ChBody>();
    geneva_wheel->SetPos(geneva_center);
    geneva_wheel->SetAngVelLocal(ChVector3d(0, 0, -0.08));
    sys.Add(geneva_wheel);

    // Create a ChLinePath geometry that represents the 2D shape of the Geneva wheel.
    // It can be made of ChLineArc or ChLineSegment sub-lines. These must be added in clockwise
    // order, and the end of sub-item i must be coincident with beginning of sub-line i+1.
    auto wheel_path = chrono_types::make_shared<ChLinePath>();

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
        ChLineSegment seg1(p1, p2);
        ChLineSegment seg2(p2, p3);
        ChLineSegment seg3(p3, p4);
        ChLineSegment seg4(p4, p5);
        ChLineSegment seg5(p5, p6);
        wheel_path->AddSubLine(seg1);
        wheel_path->AddSubLine(seg2);
        wheel_path->AddSubLine(seg3);
        wheel_path->AddSubLine(seg4);
        wheel_path->AddSubLine(seg5);
        double a1 = alpha + CH_PI;
        double a2 = alpha + CH_PI + gamma;
        ChLineArc arc0(ChCoordsys<>(p7), Ri, a1, a2, true);  // ccw arc bacause concave
        wheel_path->AddSubLine(arc0);
    }

    // Add the collision shape to the body
    geneva_wheel->EnableCollision(true);
    auto genevawheel_coll = chrono_types::make_shared<ChCollisionShapePath2D>(contact_mat, wheel_path);
    geneva_wheel->AddCollisionShape(genevawheel_coll, ChFrame<>());
    geneva_wheel->GetCollisionModel()->SetSafeMargin(0.02f);

    // Create a ChVisualShapeLine, a visualization asset for lines.
    auto wheel_asset = chrono_types::make_shared<ChVisualShapeLine>();
    wheel_asset->SetLineGeometry(wheel_path);
    geneva_wheel->AddVisualShape(wheel_asset);

    // Revolute constraint
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(geneva_wheel, ground, ChFrame<>(geneva_center));
    sys.Add(revolute);

    // Create the crank:

    auto crank = chrono_types::make_shared<ChBody>();
    crank->SetPos(crank_center);
    sys.Add(crank);

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order
    auto crankpin_path = chrono_types::make_shared<ChLinePath>();
    ChLineArc pin(ChCoordsys<>(ChVector3d(-B, 0, 0)), wi / 2 - 0.005, CH_2PI, 0);
    crankpin_path->AddSubLine(pin);

    auto crankstopper_path = chrono_types::make_shared<ChLinePath>();
    ChLineArc stopper_arc(ChCoordsys<>(ChVector3d(0, 0, 0)), Ri - 0.005, CH_PI - gamma / 2, -CH_PI + gamma / 2);
    ChLineSegment stopperve1(stopper_arc.GetEndB(), ChVector3d(0, 0, 0));
    ChLineSegment stopperve2(ChVector3d(0, 0, 0), stopper_arc.GetEndA());
    crankstopper_path->AddSubLine(stopper_arc);
    crankstopper_path->AddSubLine(stopperve1);
    crankstopper_path->AddSubLine(stopperve2);

    // Add the collision shape to the body
    crank->EnableCollision(true);
    auto crankpin_coll = chrono_types::make_shared<ChCollisionShapePath2D>(contact_mat, crankpin_path);
    auto crankstopper_coll = chrono_types::make_shared<ChCollisionShapePath2D>(contact_mat, crankstopper_path);
    crank->AddCollisionShape(crankpin_coll, ChFrame<>());
    crank->AddCollisionShape(crankstopper_coll, ChFrame<>());
    crank->GetCollisionModel()->SetSafeMargin(0.02f);

    // Create a ChVisualShapeLine, a visualization asset for lines
    auto crank_asset = chrono_types::make_shared<ChVisualShapeLine>();
    crank_asset->SetLineGeometry(crankpin_path);
    crank->AddVisualShape(crank_asset);

    auto crank_asset2 = chrono_types::make_shared<ChVisualShapeLine>();
    crank_asset2->SetLineGeometry(crankstopper_path);
    crank->AddVisualShape(crank_asset2);

    // Motor between crank and truss
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(crank, ground, ChFrame<>(crank_center));
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(CH_PI / 8.0));
    sys.AddLink(motor);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("2D collision");
    vis->SetBackgroundColor(ChColor(0.1f, 0.2f, 0.3f));
    vis->Initialize();
    vis->AddLogo();
    vis->AddCamera(ChVector3d(0, 4, -6));
    vis->AddTypicalLights();

    // This means that contactforces will be shown in Irrlicht application
    vis->SetSymbolScale(0.2);
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

    // Simulation loop
    double step = 1e-3;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(step);
    }

    return 0;
}
