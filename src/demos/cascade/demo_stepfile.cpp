//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Show how to use the OpenCASCADE features
//   implemented in the unit_CASCADE:
//
//   - load a 3D model saved in STEP format from a CAD
//   - select some sub assemblies from the STEP model
//   - make Chrono::Engine objects out of those parts
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_cascade/ChBodyEasyCascade.h"
#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeShapeAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"


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

    // Create a ChronoENGINE physical system: all bodies and constraints
    // will be handled by this ChSystem object.
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Load a STEP model from file", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(30, 100, 30), core::vector3df(30, -80, -30), 200, 130);
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0.2f, 0.2f, -0.3f));


    //
    // Load a STEP file, containing a mechanism. The demo STEP file has been
    // created using a 3D CAD (in this case, SolidEdge v.18).
    //

    // Create the ChCascadeDoc, a container that loads the STEP model
    // and manages its subassembles
    ChCascadeDoc mydoc;

    // load the STEP model using this command:
    bool load_ok = mydoc.Load_STEP(GetChronoDataFile("cascade/assembly.stp").c_str());  // or specify abs.path: ("C:\\data\\cascade\\assembly.stp");

    // print the contained shapes
    mydoc.Dump(GetLog());


    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.001);

    // In most CADs the Y axis is horizontal, but we want it vertical.
    // So define a root transformation for rotating all the imported objects.
    ChQuaternion<> rotation1;
    rotation1.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));  // 1: rotate 90° on X axis
    ChQuaternion<> rotation2;
    rotation2.Q_from_AngAxis(CH_C_PI, ChVector<>(0, 1, 0));  // 2: rotate 180° on vertical Y axis
    ChQuaternion<> tot_rotation = rotation2 % rotation1;     // rotate on 1 then on 2, using quaternion product
    ChFrameMoving<> root_frame(ChVector<>(0, 0, 0), tot_rotation);

    // Retrieve some sub shapes from the loaded model, using
    // the GetNamedShape() function, that can use path/subpath/subsubpath/part
    // syntax and * or ? wildcards, etc.

    std::shared_ptr<ChBodyEasyCascade> mrigidBody1;
    std::shared_ptr<ChBodyEasyCascade> mrigidBody2;

    if (load_ok) {
       
        TopoDS_Shape shape1;
        if (mydoc.GetNamedShape(shape1, "Assem1/body1")) {
            
            std::shared_ptr<ChBodyEasyCascade> mbody1 (new ChBodyEasyCascade(shape1, 1000, false, true));
            my_system.Add(mbody1);
            
            mbody1->SetBodyFixed(true); 

            // Move the body as for global displacement/rotation (also mbody1 %= root_frame; )
            mbody1->ConcatenatePreTransformation(root_frame);

            mrigidBody1= mbody1;

        } else
            GetLog() << "Warning. Desired object not found in document \n";

        TopoDS_Shape shape2;
        if (mydoc.GetNamedShape(shape2, "Assem1/body2")) {

            std::shared_ptr<ChBodyEasyCascade> mbody2 (new ChBodyEasyCascade(shape2, 1000, false, true));
            my_system.Add(mbody2);
            
            // Move the body as for global displacement/rotation  (also mbody2 %= root_frame; )
            mbody2->ConcatenatePreTransformation(root_frame);

            mrigidBody2= mbody2;

        } else
            GetLog() << "Warning. Desired object not found in document \n";

    } else
        GetLog() << "Warning. Desired STEP file could not be opened/parsed \n";

    // Create a revolute joint between the two parts
    // as in a pendulum. We assume we already know in advance
    // the aboslute position of the joint (ex. we used measuring tools in the 3D CAD)
    ChVector<> measured_joint_pos_mm(0, 48, 120);
    double scale = 1. / 1000.;  // because we use meters instead of mm
    ChVector<> joint_pos =
        ((ChFrame<>)root_frame) * (measured_joint_pos_mm * scale);  // transform because we rotated everything

    if (mrigidBody1 && mrigidBody2) {
        std::shared_ptr<ChLinkLockRevolute> my_link(new ChLinkLockRevolute);
        my_link->Initialize(mrigidBody1, mrigidBody2, ChCoordsys<>(joint_pos));
        my_system.AddLink(my_link);
    }

    // Create a large cube as a floor.

    std::shared_ptr<ChBodyEasyBox> mfloor(new ChBodyEasyBox(1, 0.2, 1, 1000));
    mfloor->SetPos(ChVector<>(0,-0.3,0));
    mfloor->SetBodyFixed(true);
    application.GetSystem()->Add(mfloor);

    std::shared_ptr<ChColorAsset> mcolor(new ChColorAsset(0.3,0.3,0.8));
    mfloor->AddAsset(mcolor);



    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();


    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    application.SetTimestep(0.01);
    application.SetTryRealtime(true);

    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.GetVideoDriver()->beginScene(true, true, video::SColor(255, 140, 161, 192));

        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();

        application.DoStep();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
