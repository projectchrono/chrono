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
//   Show how to use the OpenCASCADE features
//   implemented in the unit_CASCADE:
//
//   - use the ChCascadeBodyEasy to create a 2D shape with proper mass & inertia
//   - use the ChCascadeBodyEasy to define a 2D collision profile optimized for smooth arc contacts
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_cascade/ChCascadeBodyEasy.h"
#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChVisualShapeCascade.h"
#include "chrono/assets/ChVisualSystem.h"

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::cascade;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
#if !defined(CHRONO_IRRLICHT) && !defined(CHRONO_VSG)
    std::cerr << "Configure chrono with VSG or Irrlicht to run this example!" << std::endl;
    return 1;
#endif

    // Check for valid visualization system
    if (vis_type == ChVisualSystem::Type::NONE) {
        std::cout << "Configure chrono with VSG or Irrlicht to run this example!" << std::endl;
        return 1;
    }
    // Create a Chrono physical system: all bodies and constraints
    // will be handled by this ChSystemNSC object.
    ChSystemNSC sys;
    sys.SetGravityY();
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Collision tolerances.
    // note: for 2D-2D contact problems, only the margin tolerance is used (that acts outward as the envelope)
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0);
    ChCollisionModel::SetDefaultSuggestedMargin(0.1);

    // Contact material (shared among all collision shapes)
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetFriction(0.05f);

    // Create the truss:
    auto floor = chrono_types::make_shared<ChBody>();
    floor->SetFixed(true);
    sys.Add(floor);

    //
    // Create the Geneva wheel
    //

    // Geneva wheel geometry data:
    int nstations = 5;
    double R = 1;
    double Ri = 0.5;
    double wi = 0.1;
    double Li = 0.55;
    ChVector3d geneva_center(-0, 0, 0);
    // compute aux data:
    double beta = (CH_2PI / (double)nstations);  // angle width of station
    double gamma = 2 * (CH_PI_2 - beta / 2);
    double B = R * std::tan(beta / 2);
    ChVector3d crank_center = ChVector3d(B, R, 0) + geneva_center;

    // Create a ChLinePath geometry that represents the 2D shape of the Geneva wheel.
    // It can be made of ChLineArc or ChLineSegment sub-lines. These must be added in clockwise
    // order, and the end of sub-item i must be coincident with beginning of sub-line i+1.
    auto pathwheel = chrono_types::make_shared<ChLinePath>();

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
        ChLineArc mseg3(ChCoordsys<>((p3 + p4) * 0.5), wi / 2, alpha + CH_PI, alpha + CH_2PI, true);
        ChLineSegment mseg4(p4, p5);
        ChLineSegment mseg5(p5, p6);
        pathwheel->AddSubLine(mseg1);
        pathwheel->AddSubLine(mseg2);
        pathwheel->AddSubLine(mseg3);
        pathwheel->AddSubLine(mseg4);
        pathwheel->AddSubLine(mseg5);
        double a1 = alpha + CH_PI;
        double a2 = alpha + CH_PI + gamma;
        ChLineArc marc0(ChCoordsys<>(p7), Ri, a1, a2, true);  // ccw arc because concave
        pathwheel->AddSubLine(marc0);
    }

    // Create the rotating Geneva wheel using the ChCascadeBodyEasyProfile  body, this will automate
    // some useful operations:
    // - it computes a triangulation of face surrounded by profile (and holes, if any) as a visualizer asset
    // - it computes the mass and the inertia tensor given the profile
    // - it moves the COG in the computed position
    // - it adds a 2D collision profile optimized for 2D-2D collision scenarios, with smooth arc collisions.
    // Might throw exception if some wire path not on XY plane, or not closed.

    auto genevawheel = chrono_types::make_shared<ChCascadeBodyEasyProfile>(
        std::vector<std::shared_ptr<::chrono::ChLinePath>>{pathwheel},      // wire(s) containing the face
        std::vector<std::shared_ptr<::chrono::ChLinePath>>{},               // wire(s) telling holes (empty here)
        0.05,                                                               // the thickness
        1000,                                                               // the density
        chrono_types::make_shared<ChCascadeTriangulate>(0.01, false, 0.2),  // finer than default
        true,                                                               // enable 2D collision
        material                                                            // contact material
    );
    genevawheel->GetVisualShape(0)->SetColor(ChColor(0.57f, 0.4f, 0.57f));
    for (auto& shape : genevawheel->GetVisualModel()->GetShapeInstances())
        shape.shape->SetColor(ChColor(0.57f, 0.4f, 0.57f));

    genevawheel->SetFrameRefToAbs(ChFrame<>(geneva_center));
    genevawheel->SetAngVelLocal(ChVector3d(0, 0, -0.08));
    sys.Add(genevawheel);

    // Do you need an additional profile at a different Z depht?
    // If so, use the AddProfile() function. It also updates the mass, COG position, collision shapes, etc.
    // Might throw exception if path not on XY plane, or not closed.
    auto mpathcam = chrono_types::make_shared<ChLinePath>();
    auto mcamline1 = chrono_types::make_shared<ChLineArc>(ChCoordsys<>(ChVector3d(0, 0, -0.10)), R * 0.3, 1.5,
                                                          CH_PI);  // CH_2PI, 2.5);
    auto mcamline2 = chrono_types::make_shared<ChLineSegment>(mcamline1->GetEndB(), mcamline1->GetEndA());
    mpathcam->AddSubLine(mcamline1);
    mpathcam->AddSubLine(mcamline2);
    genevawheel->AddProfile({mpathcam},                                         // wire(s) containing the face
                            {},                                                 // wire(s) telling holes (empty here)
                            0.09, 1000,                                         // thickness, density
                            chrono_types::make_shared<ChCascadeTriangulate>(),  // tdefault tesselation parameters
                            true,                                               // enable 2D collision on the profile
                            material                                            // contact material
    );

    for (auto& shape : genevawheel->GetVisualModel()->GetShapeInstances())
        shape.shape->SetColor(ChColor(0.57f, 0.4f, 0.57f));

    // Revolute constraint
    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute->Initialize(genevawheel, floor, ChFrame<>(geneva_center));
    sys.Add(revolute);

    //
    // Create the crank:
    //

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order:
    auto pathcrankpin = chrono_types::make_shared<ChLinePath>();
    ChLineArc mpin(ChCoordsys<>(ChVector3d(-B, 0, 0)), wi / 2 - 0.005, CH_2PI, 0);
    pathcrankpin->AddSubLine(mpin);

    auto pathcrankstopper = chrono_types::make_shared<ChLinePath>();
    ChLineArc stopperarc(ChCoordsys<>(ChVector3d(0, 0, 0)), Ri - 0.003, CH_PI - gamma / 2, -CH_PI + gamma / 2);
    ChLineSegment mstopperve1(stopperarc.GetEndB(), ChVector3d(0, 0, 0));
    ChLineSegment mstopperve2(ChVector3d(0, 0, 0), stopperarc.GetEndA());
    pathcrankstopper->AddSubLine(stopperarc);
    pathcrankstopper->AddSubLine(mstopperve1);
    pathcrankstopper->AddSubLine(mstopperve2);

    // Use the ChCascadeBodyEasyProfile to define a 2D profile, again:
    auto crank = chrono_types::make_shared<ChCascadeBodyEasyProfile>(
        std::vector<std::shared_ptr<::chrono::ChLinePath>>{pathcrankstopper,
                                                           pathcrankpin},   // wire(s) containing the face
        std::vector<std::shared_ptr<::chrono::ChLinePath>>{},               // wire(s) telling holes (empty here)
        0.05,                                                               // the thickness
        1000,                                                               // the density
        chrono_types::make_shared<ChCascadeTriangulate>(0.01, false, 0.2),  // finer than default
        true,                                                               // enable 2D collision
        material                                                            // contact material
    );

    // Do you need an additional profile at a different Z depht?
    // If so, use the AddProfile() function. It also updates the mass, COG position, collision shapes, etc.
    // Might throw exception if path not on XY plane, or not closed.
    auto mpathbackplate = chrono_types::make_shared<ChLinePath>();
    auto mbackplate = chrono_types::make_shared<ChLineArc>(ChCoordsys<>(ChVector3d(0, 0, 0.06)), Ri * 1.3, CH_2PI, 0);
    mpathbackplate->AddSubLine(mbackplate);
    crank->AddProfile({mpathbackplate},  // wire(s) containing the face
                      {},                // wire(s) telling holes (empty here)
                      0.05, 1000,        // thickness, density
                      chrono_types::make_shared<ChCascadeTriangulate>(0.01, false, 0.2),  // finer than default
                      true,                                                               // enable 2D collision
                      material                                                            // contact material
    );

    crank->SetFrameRefToAbs(
        ChFrame<>(crank_center));  // the REF is the coordinate where the path has been defined, the COG maybe elsewhere
    sys.Add(crank);

    for (auto& shape : crank->GetVisualModel()->GetShapeInstances())
        shape.shape->SetColor(ChColor(0.57f, 0.57f, 0.4f));

    // Should you later change some geometry, do something like this:
    // mbackplate->radius = Ri * 2;
    // crank->UpdateCollisionAndVisualizationShapes();

    // Add a motor between crank and truss
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(crank, floor, ChFrame<>(crank_center));
    motor->SetSpeedFunction(chrono_types::make_shared<ChFunctionConst>(CH_PI / 8.0));
    sys.AddLink(motor);

    //
    // The follower
    //

    // Create a simple follower falling on the little cam

    auto followerwire = chrono_types::make_shared<ChLinePath>();
    double Z_layer_2 = -0.10;
    ChLineSegment msfoll1(ChVector3d(-0.1, R * 0.3 + 0.1, Z_layer_2), ChVector3d(-0.1, R * 0.3, Z_layer_2));
    ChLineSegment msfoll2(msfoll1.GetEndB(), ChVector3d(-2 * R, R * 0.3, Z_layer_2));
    ChLineSegment msfoll3(msfoll2.GetEndB(), ChVector3d(-2 * R, R * 0.3 + 0.1, Z_layer_2));
    ChLineSegment msfoll4(msfoll3.GetEndB(), msfoll1.GetEndA());

    followerwire->AddSubLine(msfoll1);
    followerwire->AddSubLine(msfoll2);
    followerwire->AddSubLine(msfoll3);
    followerwire->AddSubLine(msfoll4);

    auto follower = chrono_types::make_shared<ChCascadeBodyEasyProfile>(
        std::vector<std::shared_ptr<::chrono::ChLinePath>>{followerwire},   // wire(s) containing the face
        std::vector<std::shared_ptr<::chrono::ChLinePath>>{},               // wire(s) telling holes (empty here)
        0.09,                                                               // the thickness
        1000,                                                               // the density
        chrono_types::make_shared<ChCascadeTriangulate>(0.01, false, 0.2),  // finer than default
        true,                                                               // enable 2D collision
        material                                                            // contact material
    );
    for (auto& shape : follower->GetVisualModel()->GetShapeInstances())
        shape.shape->SetColor(ChColor(0.4f, 0.57f, 0.57f));
    sys.Add(follower);

    // Revolute constraint
    auto revolute2 = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute2->Initialize(follower, floor, ChFrame<>(ChVector3d(-1.4 * R, R * 0.3 + 0.05, Z_layer_2)));
    sys.Add(revolute2);

    // Create the run-time visualization system
    std::shared_ptr<ChVisualSystem> vis;

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(1024, 768);
            vis_irr->SetWindowTitle("Use 2D profiles with OpenCASCADE for mass, inertia, meshing");
            vis_irr->SetCameraVertical(CameraVerticalDir::Y);
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector3d(0.2, 0.2, -4.3));
            vis_irr->AddTypicalLights();

            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowSize(1024, 768);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetWindowTitle("Use 2D profiles with OpenCASCADE for mass, inertia, meshing");
            vis_vsg->AddCamera(ChVector3d(0.2, 0.2, -4.3));
            vis_vsg->SetLightIntensity(0.8f);
            vis_vsg->SetLightDirection(-CH_PI_2, -CH_PI_4);
            vis_vsg->EnableShadows();
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double timestep = 0.01;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(timestep);
        realtime_timer.Spin(timestep);
    }

    return 0;
}
