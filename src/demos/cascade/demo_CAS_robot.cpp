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
//   - load a 3D model saved in STEP format from a CAD
//   - select some sub assemblies from the STEP model
//   - make Chrono::Engine objects out of those parts
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_cascade/ChBodyEasyCascade.h"
#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeShapeAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/solver/ChSolverADMM.h"
#include "chrono/assets/ChColorAsset.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

// Use the namespace with OpenCascade stuff
using namespace cascade;

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC my_system;

    // Create a surface material to be used for collisions, if any
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0);

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Load a robot model from STEP file", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(core::vector3df(30, 100, 30), core::vector3df(30, -80, -30), 200, 130);
    application.AddTypicalCamera(core::vector3df(0.2f, 1.6f, 3.5f), core::vector3df(0.0f, 1.0f, 0.0f));

    //
    // Load a STEP file, containing a mechanism. The demo STEP file has been
    // created using a 3D CAD (in this case, SolidEdge v.18).
    //

    // Create the ChCascadeDoc, a container that loads the STEP model
    // and manages its subassemblies
    ChCascadeDoc mydoc;

    // load the STEP model using this command:
    bool load_ok = mydoc.Load_STEP(GetChronoDataFile("/cascade/IRB7600_23_500_m2000_rev1_01_decorated.stp").c_str());

    // print the contained shapes
    mydoc.Dump(GetLog());

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.001);

    //
    // Retrieve some sub shapes from the loaded model, using
    // the GetNamedShape() function, that can use path/subpath/subsubpath/part
    // syntax and * or ? wldcards, etc.
    //

    std::shared_ptr<ChBodyEasyCascade> mrigidBody_base;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody_turret;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody_bicep;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody_elbow;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody_forearm;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody_wrist;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody_hand;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody_cylinder;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody_rod;

    // Note, In most CADs the Y axis is horizontal, but we want it vertical.
    // So define a root transformation for rotating all the imported objects.
    ChQuaternion<> rotation1;
    rotation1.Q_from_AngAxis(-CH_C_PI / 2, VECT_X);  // 1: rotate 90� on X axis
    ChQuaternion<> rotation2;
    rotation2.Q_from_AngAxis(CH_C_PI, VECT_Y);            // 2: rotate 180� on vertical Y axis
    ChQuaternion<> tot_rotation = rotation2 % rotation1;  // rotate on 1 then on 2, using quaternion product
    ChFrameMoving<> root_frame(ChVector<>(0, 0, 0), tot_rotation);

    if (load_ok) {
        TopoDS_Shape shape_base;
        if (mydoc.GetNamedShape(shape_base, "Assem10/Assem8")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_base, 1000, true, false);
            mrigidBody_base = mbody;

            my_system.Add(mrigidBody_base);

            // The base is fixed to the ground
            mrigidBody_base->SetBodyFixed(true);

            mrigidBody_base->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape_turret;
        if (mydoc.GetNamedShape(shape_turret, "Assem10/Assem4")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_turret, 1000, true, false);
            mrigidBody_turret = mbody;
            my_system.Add(mrigidBody_turret);
            // Move the body as for global displacement/rotation
            mrigidBody_turret->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape_bicep;
        if (mydoc.GetNamedShape(shape_bicep, "Assem10/Assem1")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_bicep, 1000, true, false);
            mrigidBody_bicep = mbody;
            my_system.Add(mrigidBody_bicep);
            // Move the body as for global displacement/rotation
            mrigidBody_bicep->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape_elbow;
        if (mydoc.GetNamedShape(shape_elbow, "Assem10/Assem5")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_elbow, 1000, true, false);
            mrigidBody_elbow = mbody;
            my_system.Add(mrigidBody_elbow);
            // Move the body as for global displacement/rotation
            mrigidBody_elbow->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape_forearm;
        if (mydoc.GetNamedShape(shape_forearm, "Assem10/Assem7")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_forearm, 1000, true, false);
            mrigidBody_forearm = mbody;
            my_system.Add(mrigidBody_forearm);
            // Move the body as for global displacement/rotation
            mrigidBody_forearm->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape_wrist;
        if (mydoc.GetNamedShape(shape_wrist, "Assem10/Assem6")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_wrist, 1000, true, false);
            mrigidBody_wrist = mbody;
            my_system.Add(mrigidBody_wrist);
            // Move the body as for global displacement/rotation
            mrigidBody_wrist->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape_hand;
        if (mydoc.GetNamedShape(shape_hand, "Assem10/Assem9")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_hand, 1000, true, false);
            mrigidBody_hand = mbody;
            my_system.Add(mrigidBody_hand);
            // Move the body as for global displacement/rotation
            mrigidBody_hand->ConcatenatePreTransformation(root_frame);

            // also create a collision shape attached to the hand:
            auto mcube = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.08, 0.08, 1000, true, true, mysurfmaterial);
            mcube->SetCoord(ChCoordsys<>(ChVector<>(0.1,0,0)) >> mrigidBody_hand->GetCoord());
            my_system.Add(mcube);
            auto mcubelink = chrono_types::make_shared<ChLinkLockLock>();
            mcubelink->Initialize(mcube, mrigidBody_hand, mcube->GetCoord());
            my_system.Add(mcubelink);
           
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape_cylinder;
        if (mydoc.GetNamedShape(shape_cylinder, "Assem10/Assem3")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_cylinder, 1000, true, false);
            mrigidBody_cylinder = mbody;
            my_system.Add(mrigidBody_cylinder);
            // Move the body as for global displacement/rotation
            mrigidBody_cylinder->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape_rod;
        if (mydoc.GetNamedShape(shape_rod, "Assem10/Assem2")) {
            auto mbody = chrono_types::make_shared<ChBodyEasyCascade>(shape_rod, 1000, true, false);
            mrigidBody_rod = mbody;
            my_system.Add(mrigidBody_rod);
            // Move the body as for global displacement/rotation
            mrigidBody_rod->ConcatenatePreTransformation(root_frame);
        } else
            GetLog() << "Warning. Desired object not found in document \n";

    } else
        GetLog() << "Warning. Desired STEP file could not be opened/parsed \n";

    if (!mrigidBody_base || !mrigidBody_turret || !mrigidBody_bicep || !mrigidBody_elbow || !mrigidBody_forearm ||
        !mrigidBody_wrist || !mrigidBody_hand) {
        return 1;
    }

    // add color to created bodies
    for (auto mbody : my_system.Get_bodylist()) {
        auto mcol = chrono_types::make_shared<ChColorAsset>(1.0f,0.5f,0.0f);
        mbody->AddAsset(mcol);
    }

    // Create joints between two parts.
    // To understand where is the axis of the joint, we can exploit the fact
    // that in the STEP file that we prepared for this demo, we inserted some
    // objects called 'marker' and we placed them aligned to the shafts, so now
    // we can fetch them and get their position/rotation.

    TopoDS_Shape shape_marker;

    ChFrame<> frame_marker_base_turret;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem8/marker#1"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_base_turret);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    // Transform the abs coordinates of the marker because everything was rotated/moved by 'root_frame' :
    frame_marker_base_turret %= root_frame;

    std::shared_ptr<ChLinkLockRevolute> my_link1(new ChLinkLockRevolute);
    my_link1->Initialize(mrigidBody_base, mrigidBody_turret, frame_marker_base_turret.GetCoord());
    my_system.AddLink(my_link1);

    ChFrame<> frame_marker_turret_bicep;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem4/marker#2"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_turret_bicep);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    frame_marker_turret_bicep %= root_frame;

    std::shared_ptr<ChLinkLockRevolute> my_link2(new ChLinkLockRevolute);
    my_link2->Initialize(mrigidBody_turret, mrigidBody_bicep, frame_marker_turret_bicep.GetCoord());
    my_system.AddLink(my_link2);

    ChFrame<> frame_marker_bicep_elbow;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem1/marker#2"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_bicep_elbow);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    frame_marker_bicep_elbow %= root_frame;

    std::shared_ptr<ChLinkLockRevolute> my_link3(new ChLinkLockRevolute);
    my_link3->Initialize(mrigidBody_bicep, mrigidBody_elbow, frame_marker_bicep_elbow.GetCoord());
    my_system.AddLink(my_link3);

    ChFrame<> frame_marker_elbow_forearm;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem5/marker#2"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_elbow_forearm);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    frame_marker_elbow_forearm %= root_frame;

    std::shared_ptr<ChLinkLockRevolute> my_link4(new ChLinkLockRevolute);
    my_link4->Initialize(mrigidBody_elbow, mrigidBody_forearm, frame_marker_elbow_forearm.GetCoord());
    my_system.AddLink(my_link4);

    ChFrame<> frame_marker_forearm_wrist;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem7/marker#2"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_forearm_wrist);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    frame_marker_forearm_wrist %= root_frame;

    std::shared_ptr<ChLinkLockRevolute> my_link5(new ChLinkLockRevolute);
    my_link5->Initialize(mrigidBody_forearm, mrigidBody_wrist, frame_marker_forearm_wrist.GetCoord());
    my_system.AddLink(my_link5);

    ChFrame<> frame_marker_wrist_hand;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem6/marker#2"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_wrist_hand);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    frame_marker_wrist_hand %= root_frame;

    std::shared_ptr<ChLinkLockRevolute> my_link6(new ChLinkLockRevolute);
    my_link6->Initialize(mrigidBody_wrist, mrigidBody_hand, frame_marker_wrist_hand.GetCoord());
    my_system.AddLink(my_link6);

    ChFrame<> frame_marker_turret_cylinder;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem4/marker#3"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_turret_cylinder);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    frame_marker_turret_cylinder %= root_frame;

    std::shared_ptr<ChLinkLockRevolute> my_link7(new ChLinkLockRevolute);
    my_link7->Initialize(mrigidBody_turret, mrigidBody_cylinder, frame_marker_turret_cylinder.GetCoord());
    my_system.AddLink(my_link7);

    ChFrame<> frame_marker_cylinder_rod;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem3/marker#2"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_cylinder_rod);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    frame_marker_cylinder_rod %= root_frame;

    std::shared_ptr<ChLinkLockCylindrical> my_link8(new ChLinkLockCylindrical);
    my_link8->Initialize(mrigidBody_cylinder, mrigidBody_rod, frame_marker_cylinder_rod.GetCoord());
    my_system.AddLink(my_link8);

    ChFrame<> frame_marker_rod_bicep;
    if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem2/marker#2"))
        ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_rod_bicep);
    else
        GetLog() << "Warning. Desired marker not found in document \n";
    frame_marker_rod_bicep %= root_frame;

    std::shared_ptr<ChLinkLockCylindrical> my_link9(new ChLinkLockCylindrical);
    my_link9->Initialize(mrigidBody_rod, mrigidBody_bicep, frame_marker_rod_bicep.GetCoord());
    my_system.AddLink(my_link9);

    // Add a couple of markers for the 'lock' constraint between the hand and the
    // absolute reference: when we will move the marker in absolute reference, the
    // hand will follow it.

    std::shared_ptr<ChMarker> my_marker_hand(new ChMarker);
    std::shared_ptr<ChMarker> my_marker_move(new ChMarker);

    mrigidBody_hand->AddMarker(my_marker_hand);
    mrigidBody_base->AddMarker(my_marker_move);

    ChQuaternion<> rot_on_x;
    rot_on_x.Q_from_AngAxis(CH_C_PI / 2, VECT_X);
    ChFrame<> frame_marker_move = ChFrame<>(VNULL, rot_on_x) >> frame_marker_wrist_hand;

    my_marker_hand->Impose_Abs_Coord(frame_marker_wrist_hand.GetCoord());
    my_marker_move->Impose_Abs_Coord(frame_marker_move.GetCoord());

    std::shared_ptr<ChLinkLockLock> my_link_teacher(new ChLinkLockLock);
    my_link_teacher->Initialize(my_marker_hand, my_marker_move);
    my_system.AddLink(my_link_teacher);

    // Set motions for Z and Y coordinates of the 'my_link_teacher' marker,
    // so that the hand will follow it. To do so, we create four segments of
    // motion for Z coordinate and four for Y coordinate, we join them with
    // ChFunction_Sequence and we repeat sequence by ChFunction_Repeat

    std::shared_ptr<ChFunction_ConstAcc> motlaw_z1(new ChFunction_ConstAcc);
    motlaw_z1->Set_h(-0.7);
    motlaw_z1->Set_end(1);
    std::shared_ptr<ChFunction_Const> motlaw_z2(new ChFunction_Const);
    std::shared_ptr<ChFunction_ConstAcc> motlaw_z3(new ChFunction_ConstAcc);
    motlaw_z3->Set_h(0.7);
    motlaw_z3->Set_end(1);
    std::shared_ptr<ChFunction_Const> motlaw_z4(new ChFunction_Const);
    std::shared_ptr<ChFunction_Sequence> motlaw_z_seq(new ChFunction_Sequence);
    motlaw_z_seq->InsertFunct(motlaw_z1, 1, 1, true);
    motlaw_z_seq->InsertFunct(motlaw_z2, 1, 1, true);  // true = force c0 continuity, traslating fx
    motlaw_z_seq->InsertFunct(motlaw_z3, 1, 1, true);
    motlaw_z_seq->InsertFunct(motlaw_z4, 1, 1, true);
    std::shared_ptr<ChFunction_Repeat> motlaw_z(new ChFunction_Repeat);
    motlaw_z->Set_fa(motlaw_z_seq);
    motlaw_z->Set_window_length(4);

    std::shared_ptr<ChFunction_Const> motlaw_y1(new ChFunction_Const);
    std::shared_ptr<ChFunction_ConstAcc> motlaw_y2(new ChFunction_ConstAcc);
    motlaw_y2->Set_h(-0.6);
    motlaw_y2->Set_end(1);
    std::shared_ptr<ChFunction_Const> motlaw_y3(new ChFunction_Const);
    std::shared_ptr<ChFunction_ConstAcc> motlaw_y4(new ChFunction_ConstAcc);
    motlaw_y4->Set_h(0.6);
    motlaw_y4->Set_end(1);
    std::shared_ptr<ChFunction_Sequence> motlaw_y_seq(new ChFunction_Sequence);
    motlaw_y_seq->InsertFunct(motlaw_y1, 1, 1, true);
    motlaw_y_seq->InsertFunct(motlaw_y2, 1, 1, true);  // true = force c0 continuity, traslating fx
    motlaw_y_seq->InsertFunct(motlaw_y3, 1, 1, true);
    motlaw_y_seq->InsertFunct(motlaw_y4, 1, 1, true);
    std::shared_ptr<ChFunction_Repeat> motlaw_y(new ChFunction_Repeat);
    motlaw_y->Set_fa(motlaw_y_seq);
    motlaw_y->Set_window_length(4);

    my_marker_move->SetMotion_Z(motlaw_z);
    my_marker_move->SetMotion_Y(motlaw_y);

    // Create a large cube as a floor.

    std::shared_ptr<ChBodyEasyBox> mfloor(new ChBodyEasyBox(8, 1, 8, 1000, true, true, mysurfmaterial));
    mfloor->SetPos(ChVector<>(0, -0.5, 0));
    my_system.Add(mfloor);
    mfloor->SetBodyFixed(true);

    // Create a stack of boxes to be impacted  
    if (true) {
        double brick_h = 0.3;
        for (int ix = 0; ix < 3; ++ix)  
        for (int ib = 0; ib < 6; ++ib) {
            std::shared_ptr<ChBodyEasyBox> mcube(new ChBodyEasyBox(0.4, brick_h, 0.4, 1000, true, true, mysurfmaterial));
            mcube->SetPos(ChVector<>(-1.4, (0.5*brick_h)+ib*brick_h, -0.4 -0.5*ix));
            mcube->SetRot(Q_from_AngAxis(ib*0.1,VECT_Y));
            my_system.Add(mcube);
            auto mcol = chrono_types::make_shared<ChColorAsset>(0.5f+float(0.5*ChRandom()), 0.5f+float(0.5*ChRandom()), 0.5f+float(0.5*ChRandom()));
            mcube->AddAsset(mcol);
        }
    }

    std::shared_ptr<ChTexture> mtexture(new ChTexture(GetChronoDataFile("textures/blue.png").c_str()));
    mfloor->AddAsset(mtexture);

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    // Modify the settings of the solver.
    // By default, the solver might not have sufficient precision to keep the
    // robot joints 'mounted'. Expecially, the SOR, SSOR and other fixed point methods
    // cannot simulate well this robot problem because the mass of the last body in the
    // kinematic chain, i.e. the hand, is very low when compared to other bodies, so
    // the convergence of the solver would be bad when 'pulling the hand' as in this
    // 'teaching mode' IK.
    // So switch to a more precise solver, ex. BARZILAIBORWEIN

    
    my_system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    my_system.SetSolverMaxIterations(200);
    
    /*
    // Alternative: the ADMM solver offers higher precision and it can also support FEA + nonsmooth contacts 
    auto solver = chrono_types::make_shared<ChSolverADMM>(); //faster, if MKL enabled: chrono_types::make_shared<ChSolverPardisoMKL>());
    solver->EnableWarmStart(true);
    solver->SetMaxIterations(60);
    solver->SetRho(1);
    solver->SetSigma(1e-8);
    solver->SetStepAdjustPolicy(ChSolverADMM::AdmmStepType::BALANCED_UNSCALED);
    my_system.SetSolver(solver);
    */

    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, video::SColor(255, 140, 161, 192));

        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        // .. perform timestep simulation
        application.DoStep();

        // .. plot something on realtime view
        tools::drawChFunction(application.GetDevice(), motlaw_z.get(), 0, 10, -0.9, 0.2, 10, 400, 300, 80);
        tools::drawChFunction(application.GetDevice(), motlaw_y.get(), 0, 10, -0.9, 0.2, 10, 500, 300, 80);

        application.EndScene();
    }

    return 0;
}
