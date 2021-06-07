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
// Demonstration of using the assets system to create shapes for Irrlicht
// visualization.
// NOTE:the same assets can be used for visualization with the OpenGL module,
// postprocessing with POVray, etc.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChLineNurbs.h"
#include "chrono/geometry/ChSurfaceNurbs.h"
#include "chrono/assets/ChSurfaceShape.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include <irrlicht.h>

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
    ChIrrApp application(&mphysicalSystem, L"Assets for Irrlicht visualization", core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 4, -6));

    //
    // EXAMPLE 1:
    //

    // Create a ChBody, and attach some 'assets' that define 3D shapes for visualization purposes.
    // Note: these assets are independent from collision shapes!

    // Create a rigid body as usual, and add it
    // to the physical system:
    auto mfloor = chrono_types::make_shared<ChBody>();
    mfloor->SetBodyFixed(true);

    // Contact material
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    // Define a collision shape
    mfloor->GetCollisionModel()->ClearModel();
    mfloor->GetCollisionModel()->AddBox(floor_mat, 10, 0.5, 10, ChVector<>(0, -1, 0));
    mfloor->GetCollisionModel()->BuildModel();
    mfloor->SetCollide(true);

    // Add body to system
    application.GetSystem()->Add(mfloor);

    // ==Asset== attach a 'box' shape.
    // Note that assets are managed via shared pointer, so they
    // can also be shared). Do not forget AddAsset() at the end!
    auto mboxfloor = chrono_types::make_shared<ChBoxShape>();
    mboxfloor->GetBoxGeometry().Pos = ChVector<>(0, -1, 0);
    mboxfloor->GetBoxGeometry().Size = ChVector<>(10, 0.5, 10);
    mfloor->AddAsset(mboxfloor);

    // ==Asset== attach color asset.
    auto mfloorcolor = chrono_types::make_shared<ChColorAsset>();
    mfloorcolor->SetColor(ChColor(0.3f, 0.3f, 0.6f));
    mfloor->AddAsset(mfloorcolor);

    // ==Asset== attach a 'path' shape populated with segments and arc fillets:
    auto mpathfloor = chrono_types::make_shared<ChPathShape>();
    ChLineSegment mseg1(ChVector<>(1, 2, 0), ChVector<>(1, 3, 0));
    mpathfloor->GetPathGeometry()->AddSubLine(mseg1);
    ChLineSegment mseg2(ChVector<>(1, 3, 0), ChVector<>(2, 3, 0));
    mpathfloor->GetPathGeometry()->AddSubLine(mseg2);
    ChLineArc marc1(ChCoordsys<>(ChVector<>(2, 3.5, 0)), 0.5, -CH_C_PI_2, CH_C_PI_2);
    mpathfloor->GetPathGeometry()->AddSubLine(marc1);
    mfloor->AddAsset(mpathfloor);

	// ==Asset== attach a 'nurbs line' shape:
	// (first you create the ChLineNurbs geometry, 
	// then you put it inside a ChLineShape asset)
	auto mnurbs = chrono_types::make_shared<ChLineNurbs>();
	std::vector<ChVector<>> controlpoints = { ChVector<>(1,2,-1), ChVector<>(1,3,-1), ChVector<>(1,3,-2), ChVector<>(1,4,-2) };
	mnurbs->SetupData(3, controlpoints);

	auto mnurbsasset = chrono_types::make_shared<ChLineShape>();
	mnurbsasset->SetLineGeometry(mnurbs);
	mfloor->AddAsset(mnurbsasset);


	// ==Asset== attach a 'nurbs surface' shape:
	// (first you create the ChSurfaceNurbs geometry, 
	// then you put it inside a ChSurfaceShape asset)
	auto msurf = chrono_types::make_shared<ChSurfaceNurbs>();
	ChMatrixDynamic<ChVector<>> surfpoints(4, 2); // u points, v points
	surfpoints(0, 0) = ChVector<>(1, 2, 3);
	surfpoints(1, 0) = ChVector<>(1, 3, 3);
	surfpoints(2, 0) = ChVector<>(2, 3, 3);
	surfpoints(3, 0) = ChVector<>(2, 4, 3);
	surfpoints(0, 1) = ChVector<>(1, 2, 1);
	surfpoints(1, 1) = ChVector<>(1, 3, 1);
	surfpoints(2, 1) = ChVector<>(3, 3, 1);
	surfpoints(3, 1) = ChVector<>(2, 4, 1);
	msurf->SetupData(3, 1, surfpoints);

	auto msurfasset = chrono_types::make_shared<ChSurfaceShape>();
	msurfasset->Pos = ChVector<>(3, -1, 3);
	msurfasset->SetSurfaceGeometry(msurf);
	msurfasset->SetWireframe(true);
	mfloor->AddAsset(msurfasset);

    //
    // EXAMPLE 2:
    //

    // Textures, colors, asset levels with transformations.
    // This section shows how to add more advanced types of assets
    // and how to group assets in ChAssetLevel containers.

    // Create the rigid body as usual (this won't move,
    // it is only for visualization tests)
    auto mbody = chrono_types::make_shared<ChBody>();
    mbody->SetBodyFixed(true);
    application.GetSystem()->Add(mbody);

    // ==Asset== Attach a 'sphere' shape
    auto msphere = chrono_types::make_shared<ChSphereShape>();
    msphere->GetSphereGeometry().rad = 0.5;
    msphere->GetSphereGeometry().center = ChVector<>(-1, 0, 0);
    mbody->AddAsset(msphere);

    // ==Asset== Attach also a 'box' shape
    auto mbox = chrono_types::make_shared<ChBoxShape>();
    mbox->GetBoxGeometry().Pos = ChVector<>(1, 1, 0);
    mbox->GetBoxGeometry().Size = ChVector<>(0.3, 0.5, 0.1);
    mbody->AddAsset(mbox);

    // ==Asset== Attach also a 'cylinder' shape
    auto mcyl = chrono_types::make_shared<ChCylinderShape>();
    mcyl->GetCylinderGeometry().p1 = ChVector<>(2, -0.2, 0);
    mcyl->GetCylinderGeometry().p2 = ChVector<>(2.2, 0.5, 0);
    mcyl->GetCylinderGeometry().rad = 0.3;
    mbody->AddAsset(mcyl);

    // ==Asset== Attach also a 'triangle mesh' shape
    auto mmesh = chrono_types::make_shared<ChTriangleMeshShape>();
    mmesh->GetMesh()->getCoordsVertices().push_back(ChVector<>(0, 1, 0));
    mmesh->GetMesh()->getCoordsVertices().push_back(ChVector<>(0, 1, 0.5));
    mmesh->GetMesh()->getCoordsVertices().push_back(ChVector<>(1, 1, 0));
    mmesh->GetMesh()->getIndicesVertexes().push_back(ChVector<int>(0, 1, 2));
    mbody->AddAsset(mmesh);

    // ==Asset== Attach color. To set colors for all assets
    // in the same level, just add this:
    auto mvisual = chrono_types::make_shared<ChColorAsset>();
    mvisual->SetColor(ChColor(0.9f, 0.4f, 0.2f));
    mbody->AddAsset(mvisual);

    // ==Asset== Attach a level that contains other assets.
    // Note: a ChAssetLevel can define a rotation/translation respect to paren level,
    // Note: a ChAssetLevel can contain colors or textures: if any, they affect only objects in the level.
    auto mlevelA = chrono_types::make_shared<ChAssetLevel>();

    // ==Asset== Attach, in this level, a 'Wavefront mesh' asset,
    // referencing a .obj file:
    auto mobjmesh = chrono_types::make_shared<ChObjShapeFile>();
    mobjmesh->SetFilename(GetChronoDataFile("models/forklift/body.obj"));
    mlevelA->AddAsset(mobjmesh);

    // ==Asset== Attach also a texture, that will affect only the
    // assets in mlevelA:
    auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/bluewhite.png"));
    mlevelA->AddAsset(mtexture);

    // Change the position of mlevelA, thus moving also its sub-assets:
    mlevelA->GetFrame().SetPos(ChVector<>(0, 0, 2));
    mbody->AddAsset(mlevelA);

    // ==Asset== Attach sub level, then add to it an array of sub-levels,
    // each rotated, and each containing a displaced box, thus making a
    // spiral of cubes
    auto mlevelB = chrono_types::make_shared<ChAssetLevel>();
    for (int j = 0; j < 20; j++) {
        // ==Asset== the sub sub level..
        auto mlevelC = chrono_types::make_shared<ChAssetLevel>();

        // ==Asset== the contained box..
        auto msmallbox = chrono_types::make_shared<ChBoxShape>();
        msmallbox->GetBoxGeometry().Pos = ChVector<>(0.4, 0, 0);
        msmallbox->GetBoxGeometry().Size = ChVector<>(0.1, 0.1, 0.01);
        mlevelC->AddAsset(msmallbox);

        auto boxcol = chrono_types::make_shared<ChColorAsset>(j * 0.05f, 1 - j * 0.05f, 0.0f);
        mlevelC->AddAsset(boxcol);

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
    auto mcamera = chrono_types::make_shared<ChCamera>();
    mcamera->SetAngle(50);
    mcamera->SetPosition(ChVector<>(-3, 4, -5));
    mcamera->SetAimPoint(ChVector<>(0, 1, 0));
    mbody->AddAsset(mcamera);

    //
    // EXAMPLE 3:
    //

    // Create a ChParticleClones cluster, and attach 'assets'
    // that define a single "sample" 3D shape. This will be shown
    // N times in Irrlicht.
    //***NOTE*** This crashes with Irrlicht 1.8 , it is ok with 1.7.x and 1.8.1 + ,

    // Create the ChParticleClones, populate it with some random particles,
    // and add it to physical system:
    auto mparticles = chrono_types::make_shared<ChParticlesClones>();

    // Note: coll. shape, if needed, must be specified before creating particles.
    // This will be shared among all particles in the ChParticlesClones.
    auto particle_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    mparticles->GetCollisionModel()->ClearModel();
    mparticles->GetCollisionModel()->AddSphere(particle_mat, 0.05);
    mparticles->GetCollisionModel()->BuildModel();
    mparticles->SetCollide(true);

    // Create the random particles
    for (int np = 0; np < 100; ++np)
        mparticles->AddParticle(ChCoordsys<>(ChVector<>(ChRandom() - 2, 1.5, ChRandom() + 2)));

    // Mass and inertia properties.
    // This will be shared among all particles in the ChParticlesClones.
    mparticles->SetMass(0.1);
    mparticles->SetInertiaXX(ChVector<>(0.001,0.001,0.001));

    // Do not forget to add the particle cluster to the system:
    application.GetSystem()->Add(mparticles);

    //  ==Asset== Attach a 'sphere' shape asset.. it will be used as a sample
    // shape to display all particles when rendering in 3D!
    auto mspherepart = chrono_types::make_shared<ChSphereShape>();
    mspherepart->GetSphereGeometry().rad = 0.05;
    mparticles->AddAsset(mspherepart);

    ChVector<> displ(1,0.0,0);
    std::vector< ChVector<> > mpoints;
    mpoints.push_back( ChVector<>(0.8,0.0,0.0)+displ );
    mpoints.push_back( ChVector<>(0.8,0.3,0.0)+displ );
    mpoints.push_back( ChVector<>(0.8,0.3,0.3)+displ );
    mpoints.push_back( ChVector<>(0.0,0.3,0.3)+displ );
    mpoints.push_back( ChVector<>(0.0,0.0,0.3)+displ );
    mpoints.push_back( ChVector<>(0.8,0.0,0.3)+displ );
    auto mhull = chrono_types::make_shared<ChBodyEasyConvexHullAuxRef>(
        mpoints, 1000, true, true, chrono_types::make_shared<ChMaterialSurfaceNSC>());
    //mhull->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(2,0.3,0)));
    //mhull->SetPos(ChVector<>(2,0.3,0));
    mhull->Move(ChVector<>(2,0.3,0));
    application.GetSystem()->Add(mhull);

    ////////////////////////

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
