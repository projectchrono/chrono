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

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::cascade;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::IRRLICHT;

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system: all bodies and constraints
    // will be handled by this ChSystemNSC object.
    ChSystemNSC sys;

    // Collision tolerances.
    // note: for 2D-2D contact problems, only the margin tolerance is used (that acts outward as the envelope)
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0);
    ChCollisionModel::SetDefaultSuggestedMargin(0.1);

    // Contact material (shared among all collision shapes)
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    material->SetFriction(0.05f);

    // Create the truss:
    auto mfloor = chrono_types::make_shared<ChBody>();
    mfloor->SetBodyFixed(true);
    sys.Add(mfloor);

    //
    // Create the Geneva wheel
    //

    // Geneva wheel geometry data:
    int nstations = 5;
    double R = 1;
    double Ri = 0.5;
    double wi = 0.1;
    double Li = 0.55;
    ChVector<> geneva_center(-0, 0, 0);
    // compute aux data:
    double beta = (CH_C_2PI / (double)nstations);  // angle width of station
    double gamma = 2 * (CH_C_PI_2 - beta / 2);
    double B = R * tan(beta / 2);
    ChVector<> crank_center = ChVector<>(B, R, 0) + geneva_center;

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
        ChLineArc mseg3(ChCoordsys<>((p3 + p4) * 0.5), wi / 2, alpha + CH_C_PI, alpha + CH_C_2PI, true);
        ChLineSegment mseg4(p4, p5);
        ChLineSegment mseg5(p5, p6);
        mpathwheel->AddSubLine(mseg1);
        mpathwheel->AddSubLine(mseg2);
        mpathwheel->AddSubLine(mseg3);
        mpathwheel->AddSubLine(mseg4);
        mpathwheel->AddSubLine(mseg5);
        double a1 = alpha + CH_C_PI;
        double a2 = alpha + CH_C_PI + gamma;
        ChLineArc marc0(ChCoordsys<>(p7), Ri, a1, a2, true);  // ccw arc because concave
        mpathwheel->AddSubLine(marc0);
    }

    // Create the rotating Geneva wheel using the ChCascadeBodyEasyProfile  body, this will automate
    // some useful operations:
    // - it computes a triangulation of face surrounded by profile (and holes, if any) as a visualizer asset
    // - it computes the mass and the inertia tensor given the profile
    // - it moves the COG in the computed position
    // - it adds a 2D collision profile optimized for 2D-2D collision scenarios, with smooth arc collisions.
    // Might throw exception if some wire path not on XY plane, or not closed.

    auto mgenevawheel = chrono_types::make_shared<ChCascadeBodyEasyProfile>(
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>>{mpathwheel},  // wire(s) containing the face
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>>{},            // wire(s) telling holes (empty here)
        0.05,                                                                      // the thickness
        1000,                                                                      // the density
        chrono_types::make_shared<ChCascadeTriangulate>(0.01, false, 0.2),         // finer than default
        true,                                                                      // enable 2D collision
        material                                                                   // contact material
    );
    mgenevawheel->SetFrame_REF_to_abs(ChFrame<>(geneva_center));
    mgenevawheel->SetWvel_loc(ChVector<>(0, 0, -0.08));
    sys.Add(mgenevawheel);

    // Do you need an additional profile at a different Z depht?
    // If so, use the AddProfile() function. It also updates the mass, COG position, collision shapes, etc.
    // Might throw exception if path not on XY plane, or not closed.
    auto mpathcam = chrono_types::make_shared<ChLinePath>();
    auto mcamline1 = chrono_types::make_shared<ChLineArc>(ChCoordsys<>(ChVector<>(0, 0, -0.10)), R * 0.3, 1.5,
                                                          CH_C_PI);  // CH_C_2PI, 2.5);
    auto mcamline2 = chrono_types::make_shared<ChLineSegment>(mcamline1->GetEndB(), mcamline1->GetEndA());
    mpathcam->AddSubLine(mcamline1);
    mpathcam->AddSubLine(mcamline2);
    mgenevawheel->AddProfile({mpathcam},                                         // wire(s) containing the face
                             {},                                                 // wire(s) telling holes (empty here)
                             0.09, 1000,                                         // thickness, density
                             chrono_types::make_shared<ChCascadeTriangulate>(),  // tdefault tesselation parameters
                             true,                                               // enable 2D collision on the profile
                             material                                            // contact material
    );

    // Revolute constraint
    auto mrevolute = chrono_types::make_shared<ChLinkLockRevolute>();
    mrevolute->Initialize(mgenevawheel, mfloor, ChCoordsys<>(geneva_center));
    sys.Add(mrevolute);

    //
    // Create the crank:
    //

    // Create a ChLinePath geometry, and insert sub-paths in clockwise order:
    auto mpathcrankpin = chrono_types::make_shared<ChLinePath>();
    ChLineArc mpin(ChCoordsys<>(ChVector<>(-B, 0, 0)), wi / 2 - 0.005, CH_C_2PI, 0);
    mpathcrankpin->AddSubLine(mpin);

    auto mpathcrankstopper = chrono_types::make_shared<ChLinePath>();
    ChLineArc mstopperarc(ChCoordsys<>(ChVector<>(0, 0, 0)), Ri - 0.003, CH_C_PI - gamma / 2, -CH_C_PI + gamma / 2);
    ChLineSegment mstopperve1(mstopperarc.GetEndB(), ChVector<>(0, 0, 0));
    ChLineSegment mstopperve2(ChVector<>(0, 0, 0), mstopperarc.GetEndA());
    mpathcrankstopper->AddSubLine(mstopperarc);
    mpathcrankstopper->AddSubLine(mstopperve1);
    mpathcrankstopper->AddSubLine(mstopperve2);

    // Use the ChCascadeBodyEasyProfile to define a 2D profile, again:
    auto mcrank = chrono_types::make_shared<ChCascadeBodyEasyProfile>(
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>>{mpathcrankstopper,
                                                                     mpathcrankpin},  // wire(s) containing the face
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>>{},     // wire(s) telling holes (empty here)
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
    auto mbackplate = chrono_types::make_shared<ChLineArc>(ChCoordsys<>(ChVector<>(0, 0, 0.06)), Ri * 1.3, CH_C_2PI, 0);
    mpathbackplate->AddSubLine(mbackplate);
    mcrank->AddProfile({mpathbackplate},  // wire(s) containing the face
                       {},                // wire(s) telling holes (empty here)
                       0.05, 1000,        // thickness, density
                       chrono_types::make_shared<ChCascadeTriangulate>(0.01, false, 0.2),  // finer than default
                       true,                                                               // enable 2D collision
                       material                                                            // contact material
    );

    mcrank->SetFrame_REF_to_abs(
        ChFrame<>(crank_center));  // the REF is the coordinate where the path has been defined, the COG maybe elsewhere
    sys.Add(mcrank);

    // Should you later change some geometry, do something like this:
    // mbackplate->radius = Ri * 2;
    // mcrank->UpdateCollisionAndVisualizationShapes();

    // Add a motor between crank and truss
    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_motor->Initialize(mcrank, mfloor, ChFrame<>(crank_center));
    my_motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 8.0));
    sys.AddLink(my_motor);

    //
    // The follower
    //

    // Create a simple follower falling on the little cam

    auto mfollowerwire = chrono_types::make_shared<ChLinePath>();
    double Z_layer_2 = -0.10;
    ChLineSegment msfoll1(ChVector<>(-0.1, R * 0.3 + 0.1, Z_layer_2), ChVector<>(-0.1, R * 0.3, Z_layer_2));
    ChLineSegment msfoll2(msfoll1.GetEndB(), ChVector<>(-2 * R, R * 0.3, Z_layer_2));
    ChLineSegment msfoll3(msfoll2.GetEndB(), ChVector<>(-2 * R, R * 0.3 + 0.1, Z_layer_2));
    ChLineSegment msfoll4(msfoll3.GetEndB(), msfoll1.GetEndA());

    mfollowerwire->AddSubLine(msfoll1);
    mfollowerwire->AddSubLine(msfoll2);
    mfollowerwire->AddSubLine(msfoll3);
    mfollowerwire->AddSubLine(msfoll4);

    auto mfollower = chrono_types::make_shared<ChCascadeBodyEasyProfile>(
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>>{mfollowerwire},  // wire(s) containing the face
        std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>>{},     // wire(s) telling holes (empty here)
        0.09,                                                               // the thickness
        1000,                                                               // the density
        chrono_types::make_shared<ChCascadeTriangulate>(0.01, false, 0.2),  // finer than default
        true,                                                               // enable 2D collision
        material                                                            // contact material
    );
    sys.Add(mfollower);

    // Revolute constraint
    auto mrevolute2 = chrono_types::make_shared<ChLinkLockRevolute>();
    mrevolute2->Initialize(mfollower, mfloor, ChCoordsys<>(ChVector<>(-1.4 * R, R * 0.3 + 0.05, Z_layer_2)));
    sys.Add(mrevolute2);

    // Create the run-time visualization system
    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->AttachSystem(&sys);
            vis_irr->SetWindowSize(1024, 768);
            vis_irr->SetWindowTitle("Use 2D profiles with OpenCASCADE for mass, inertia, meshing");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddCamera(ChVector<>(0.2, 0.2, -2.3));
            vis_irr->AddTypicalLights();
            vis_irr->AddLightWithShadow(ChVector<>(1.5, 5.5, -3.5), ChVector<>(0, 0, 0), 8.2, 2.2, 8.2, 40, 512,
                                    ChColor(0.8f, 0.8f, 0.8f));
            vis_irr->EnableShadows();

            vis = vis_irr;
#endif
            break;
        }
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowSize(1024, 768);
            vis_vsg->SetWindowTitle("Use 2D profiles with OpenCASCADE for mass, inertia, meshing");
            vis_vsg->AddCamera(ChVector<>(0.2, 0.2, -4.3));
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(0.01);
    }

    return 0;
}
