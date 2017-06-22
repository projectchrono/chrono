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
// Demo code about using the assets system to create shapes that can be shown in
// the 3D postprocessing, by using POVray.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChAssetLevel.h"
#include "chrono/core/ChFileutils.h"

#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace postprocess;  // <- to keep things shorter

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystemNSC mphysicalSystem;

    /* Start example */
    /// [Example 1]

    // Create a ChBody, and attach some 'assets'
    // that define 3D shapes. These shapes can be shown
    // by Irrlicht or POV postprocessing, etc...
    // Note: these assets are independent from collision shapes!

    // Create a rigid body as usual, and add it
    // to the physical system:
    auto mfloor = std::make_shared<ChBody>();
    mfloor->SetBodyFixed(true);

    // Define a collision shape
    mfloor->GetCollisionModel()->ClearModel();
    mfloor->GetCollisionModel()->AddBox(10, 0.5, 10, ChVector<>(0, -1, 0));
    mfloor->GetCollisionModel()->BuildModel();
    mfloor->SetCollide(true);

    // Add body to system
    mphysicalSystem.Add(mfloor);

    // ==Asset== attach a 'box' shape.
    // Note that assets are managed via shared pointer, so they
    // can also be shared). Do not forget AddAsset() at the end!
    auto mboxfloor = std::make_shared<ChBoxShape>();
    mboxfloor->GetBoxGeometry().Pos = ChVector<>(0, -1, 0);
    mboxfloor->GetBoxGeometry().Size = ChVector<>(10, 0.5, 10);
    mfloor->AddAsset(mboxfloor);

    // ==Asset== attach color asset.
    auto mfloorcolor = std::make_shared<ChColorAsset>();
    mfloorcolor->SetColor(ChColor(0.3f, 0.3f, 0.6f));
    mfloor->AddAsset(mfloorcolor);

    
    /// [Example 1]
    /* End example */

    /* Start example */
    /// [Example 2]
    

    // Textures, colors, asset levels with transformations.
    // This section shows how to add more advanced types of assets
    // and how to group assets in ChAssetLevel containers.

    // Create the rigid body as usual (this won't move,
    // it is only for visualization tests)
    auto mbody = std::make_shared<ChBody>();
    mbody->SetBodyFixed(true);
    mphysicalSystem.Add(mbody);

    // ==Asset== Attach a 'sphere' shape
    auto msphere = std::make_shared<ChSphereShape>();
    msphere->GetSphereGeometry().rad = 0.5;
    msphere->GetSphereGeometry().center = ChVector<>(-1, 0, 0);
    mbody->AddAsset(msphere);

    // ==Asset== Attach also a 'box' shape
    auto mbox = std::make_shared<ChBoxShape>();
    mbox->GetBoxGeometry().Pos = ChVector<>(1, 0, 0);
    mbox->GetBoxGeometry().Size = ChVector<>(0.2, 0.5, 0.1);
    mbody->AddAsset(mbox);

    // ==Asset== Attach also a 'cylinder' shape
    auto mcyl = std::make_shared<ChCylinderShape>();
    mcyl->GetCylinderGeometry().p1 = ChVector<>(2, -0.2, 0);
    mcyl->GetCylinderGeometry().p2 = ChVector<>(2.2, 0.5, 0);
    mcyl->GetCylinderGeometry().rad = 0.3;
    mbody->AddAsset(mcyl);

    // ==Asset== Attach color. To set colors for all assets
    // in the same level, just add this:
    auto mvisual = std::make_shared<ChColorAsset>();
    mvisual->SetColor(ChColor(0.9f, 0.4f, 0.2f));
    mbody->AddAsset(mvisual);

    // ==Asset== Attach a level that contains other assets.
    // Note: a ChAssetLevel can define a rotation/translation respect to paren level,
    // Note: a ChAssetLevel can contain colors or textures: if any, they affect only objects in the level.
    auto mlevelA = std::make_shared<ChAssetLevel>();

    // ==Asset== Attach, in this level, a 'Wavefront mesh' asset,
    // referencing a .obj file:
    auto mobjmesh = std::make_shared<ChObjShapeFile>();
    mobjmesh->SetFilename(GetChronoDataFile("forklift_body.obj"));
    mlevelA->AddAsset(mobjmesh);

    // ==Asset== Attach also a texture, that will affect only the
    // assets in mlevelA:
    auto mtexture = std::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    mlevelA->AddAsset(mtexture);

    // Change the position of mlevelA, thus moving also its sub-assets:
    mlevelA->GetFrame().SetPos(ChVector<>(0, 0, 2));
    mbody->AddAsset(mlevelA);

    // ==Asset== Attach sub level, then add to it an array of sub-levels,
    // each rotated, and each containing a displaced box, thus making a
    // spiral of cubes
    auto mlevelB = std::make_shared<ChAssetLevel>();
    for (int j = 0; j < 20; j++) {
        // ==Asset== the sub sub level..
        auto mlevelC = std::make_shared<ChAssetLevel>();

        // ==Asset== the contained box..
        auto msmallbox = std::make_shared<ChBoxShape>();
        msmallbox->GetBoxGeometry().Pos = ChVector<>(0.4, 0, 0);
        msmallbox->GetBoxGeometry().Size = ChVector<>(0.1, 0.1, 0.01);
        mlevelC->AddAsset(msmallbox);

        ChQuaternion<> mrot;
        mrot.Q_from_AngAxis(j * 21 * CH_C_DEG_TO_RAD, ChVector<>(0, 1, 0));
        mlevelC->GetFrame().SetRot(mrot);
        mlevelC->GetFrame().SetPos(ChVector<>(0, j * 0.02, 0));

        mlevelB->AddAsset(mlevelC);
    }

    mbody->AddAsset(mlevelB);

    // ==Asset== Attach a video camera. This will be used by Irrlicht,
    // or POVray postprocessing, etc. Note that a camera can also be
    // put in a moving object.
    auto mcamera = std::make_shared<ChCamera>();
    mcamera->SetAngle(50);
    mcamera->SetPosition(ChVector<>(-3, 4, -5));
    mcamera->SetAimPoint(ChVector<>(0, 1, 0));
    mbody->AddAsset(mcamera);

    /// [Example 2]
    /* End example */

    /* Start example */
    /// [Example 3]

    // Create a ChParticleClones cluster, and attach 'assets'
    // that define a single "sample" 3D shape. This will be shown
    // N times in Irrlicht.

    // Create the ChParticleClones, populate it with some random particles,
    // and add it to physical system:
    auto mparticles = std::make_shared<ChParticlesClones>();

    // Note: coll. shape, if needed, must be specified before creating particles
    mparticles->GetCollisionModel()->ClearModel();
    mparticles->GetCollisionModel()->AddSphere(0.05);
    mparticles->GetCollisionModel()->BuildModel();
    mparticles->SetCollide(true);

    // Create the random particles
    for (int np = 0; np < 100; ++np)
        mparticles->AddParticle(ChCoordsys<>(ChVector<>(ChRandom() - 2, 1, ChRandom() -0.5)));

    // Do not forget to add the particle cluster to the system:
    mphysicalSystem.Add(mparticles);

    //  ==Asset== Attach a 'sphere' shape asset.. it will be used as a sample
    // shape to display all particles when rendering in 3D!
    auto mspherepart = std::make_shared<ChSphereShape>();
    mspherepart->GetSphereGeometry().rad = 0.05;
    mparticles->AddAsset(mspherepart);

    /// [Example 3]
    /* End example */

    /* Start example */
    /// [POV exporter]

    //
    // SETUP THE POSTPROCESSING
    //

    // Create an exporter to POVray !!!

    ChPovRay pov_exporter = ChPovRay(&mphysicalSystem);

    // Sets some file names for in-out processes.
    pov_exporter.SetTemplateFile(GetChronoDataFile("_template_POV.pov"));
    pov_exporter.SetOutputScriptFile("rendering_frames.pov");
    pov_exporter.SetOutputDataFilebase("my_state");
    pov_exporter.SetPictureFilebase("picture");

    // Even better: save the .dat files and the .bmp files
    // in two subdirectories, to avoid cluttering the current
    // directory...
    ChFileutils::MakeDirectory("output");
    ChFileutils::MakeDirectory("anim");

    pov_exporter.SetOutputDataFilebase("output/my_state");
    pov_exporter.SetPictureFilebase("anim/picture");

    // --Optional: modify default light
    pov_exporter.SetLight(ChVector<>(-3, 4, 2), ChColor(0.15f, 0.15f, 0.12f), false);

    // --Optional: add further POV commands, for example in this case:
    //     create an area light for soft shadows
    //     create a Grid object; Grid() parameters: step, linewidth, linecolor, planecolor
    //   Remember to use \ at the end of each line for a multiple-line string.
    pov_exporter.SetCustomPOVcommandsScript(
        " \
	light_source {   \
      <2, 10, -3>  \
	  color rgb<1.2,1.2,1.2> \
      area_light <4, 0, 0>, <0, 0, 4>, 8, 8 \
      adaptive 1 \
      jitter\
    } \
	object{ Grid(1,0.02, rgb<0.7,0.8,0.8>, rgbt<1,1,1,1>) rotate <0, 0, 90>  } \
    ");

    // --Optional: attach additional custom POV commands to some of the rigid bodies,
    //   using the ChPovRayAssetCustom asset. This asset for example projects a
    //   checkered texture to the floor. This POV specific asset won't be rendered
    //   by Irrlicht or other interfaces.
    auto mPOVcustom = std::make_shared<ChPovRayAssetCustom>();
    mPOVcustom->SetCommands((char*)"pigment { checker rgb<0.9,0.9,0.9>, rgb<0.75,0.8,0.8> }");
    mfloor->AddAsset(mPOVcustom);

    // IMPORTANT! Tell to the POVray exporter that
    // he must take care of converting the shapes of
    // all items!

    pov_exporter.AddAll();

    // (Optional: tell selectively which physical items you
    // want to render in the folllowing way...)
    //	pov_exporter.RemoveAll();
    //	pov_exporter.Add(mfloor);
    //	pov_exporter.Add(mbody);
    //	pov_exporter.Add(mparticles);

    /// [POV exporter]
    /* End example */

    /* Start example */
    /// [POV simulation]

    //
    // RUN THE SIMULATION AND SAVE THE POVray FILES AT EACH FRAME
    //

    // 1) Create the two .pov and .ini files for POV-Ray (this must be done
    //    only once at the beginning of the simulation).

    pov_exporter.ExportScript();

    while (mphysicalSystem.GetChTime() < 1.5) {
        mphysicalSystem.DoStepDynamics(0.01);

        GetLog() << "time= " << mphysicalSystem.GetChTime() << "\n";

        // 2) Create the incremental nnnn.dat and nnnn.pov files that will be load
        //    by the pov .ini script in POV-Ray (do this at each simulation timestep)
        pov_exporter.ExportData();
    }

    // That's all! If all worked ok, this python script should
    // have created a  "rendering_frames.pov.ini"  file that you can
    // load in POV-Ray, then when you press 'RUN' you will see that
    // POV-Ray will start rendering a short animation, saving the frames
    // in the directory 'anim'.

    /// [POV simulation]
    /* End example */

    return 0;
}
