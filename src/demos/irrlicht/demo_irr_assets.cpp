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
//     - using the assets system to create shapes
//       that can be shown in the Irrlicht 3D view.
//       This is a less invasive approach respect
//       to the previous Irrlicht demos. Also, the
//       same assets that you use for Irrlicht display
//       can be used for postprocessing such as POVray etc.
//
//	 CHRONO    
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
  
 
#include "physics/ChApidll.h" 
#include "physics/ChParticlesClones.h" 
#include "irrlicht_interface/ChIrrApp.h"
#include <irrlicht.h>


// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;




   
 
int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	// Create a Chrono::Engine physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Assets for Irrlicht visualization",core::dimension2d<u32>(800,600),false, true);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0,4,-6));

 
  

	//
	// EXAMPLE 1: 
	//

	// Create a ChBody, and attach some 'assets' 
	// that define 3D shapes. These shapes can be shown
	// by Irrlicht or POV postprocessing, etc...
	// Note: these assets are independent from collision shapes!

			// Create a rigid body as usual, and add it 
			// to the physical system:
	ChSharedPtr<ChBody> mfloor(new ChBody);
	mfloor->SetBodyFixed(true);

			// Define a collision shape 
	mfloor->GetCollisionModel()->ClearModel();
	mfloor->GetCollisionModel()->AddBox(10, 0.5, 10, ChVector<>(0,-1,0));
	mfloor->GetCollisionModel()->BuildModel();
	mfloor->SetCollide(true);

			// Add body to system
	application.GetSystem()->Add(mfloor);

			// ==Asset== attach a 'box' shape.
			// Note that assets are managed via shared pointer, so they 
			// can also be shared). Do not forget AddAsset() at the end!
	ChSharedPtr<ChBoxShape> mboxfloor(new ChBoxShape);
	mboxfloor->GetBoxGeometry().Pos = ChVector<>(0,-1,0);
	mboxfloor->GetBoxGeometry().Size = ChVector<>(10,0.5,10);
	mfloor->AddAsset(mboxfloor);

			// ==Asset== attach color asset. 
	ChSharedPtr<ChColorAsset> mfloorcolor(new ChColorAsset);
	mfloorcolor->SetColor(ChColor(0.3f,0.3f,0.6f));
	mfloor->AddAsset(mfloorcolor);

	/*
			//  ==Asset== IRRLICHT! Add a ChIrrNodeAsset so that Irrlicht will be able
			// to 'show' all the assets that we added to the body! 
			// OTHERWISE: use the application.AssetBind() function as at the end..
	ChSharedPtr<ChIrrNodeAsset> mirr_asset_floor(new ChIrrNodeAsset);
	mirr_asset_floor->Bind(mfloor, application);
	mfloor->AddAsset(mirr_asset_floor);
	*/


	//
	// EXAMPLE 2: 
	//

	// Textures, colors, asset levels with transformations.
	// This section shows how to add more advanced types of assets
	// and how to group assets in ChAssetLevel containers.

			// Create the rigid body as usual (this won't move,
			// it is only for visualization tests)
	ChSharedPtr<ChBody> mbody(new ChBody);
	mbody->SetBodyFixed(true);
	application.GetSystem()->Add(mbody);

			// ==Asset== Attach a 'sphere' shape  
	ChSharedPtr<ChSphereShape> msphere(new ChSphereShape);
	msphere->GetSphereGeometry().rad = 0.5;
	msphere->GetSphereGeometry().center = ChVector<>(-1,0,0);
	mbody->AddAsset(msphere);

			// ==Asset== Attach also a 'box' shape 
	ChSharedPtr<ChBoxShape> mbox(new ChBoxShape);
	mbox->GetBoxGeometry().Pos = ChVector<>(1,1,0);
	mbox->GetBoxGeometry().Size = ChVector<>(0.3,0.5,0.1);
	mbody->AddAsset(mbox);

			// ==Asset== Attach also a 'cylinder' shape 
	ChSharedPtr<ChCylinderShape> mcyl(new ChCylinderShape);
	mcyl->GetCylinderGeometry().p1  = ChVector<>(2,-0.2,0);
	mcyl->GetCylinderGeometry().p2  = ChVector<>(2.2,0.5,0);
	mcyl->GetCylinderGeometry().rad = 0.3;
	mbody->AddAsset(mcyl);

			// ==Asset== Attach also a 'triangle mesh' shape
	ChSharedPtr<ChTriangleMeshShape> mmesh(new ChTriangleMeshShape);
	mmesh->GetMesh().getCoordsVertices().push_back(ChVector<>(0,1,0));
	mmesh->GetMesh().getCoordsVertices().push_back(ChVector<>(0,1,0.5));
	mmesh->GetMesh().getCoordsVertices().push_back(ChVector<>(1,1,0));
	mmesh->GetMesh().getIndicesVertexes().push_back(ChVector<int>(0,1,2));
	mbody->AddAsset(mmesh);

			// ==Asset== Attach color. To set colors for all assets  
			// in the same level, just add this:
	ChSharedPtr<ChColorAsset> mvisual(new ChColorAsset);
	mvisual->SetColor(ChColor(0.9f,0.4f,0.2f));
	mbody->AddAsset(mvisual);
	
			// ==Asset== Attach a level that contains other assets.
			// Note: a ChAssetLevel can define a rotation/translation respect to paren level,
			// Note: a ChAssetLevel can contain colors or textures: if any, they affect only objects in the level.
	ChSharedPtr<ChAssetLevel> mlevelA(new ChAssetLevel);

				// ==Asset== Attach, in this level, a 'Wavefront mesh' asset, 
				// referencing a .obj file:
	ChSharedPtr<ChObjShapeFile> mobjmesh(new ChObjShapeFile);
	mobjmesh->SetFilename("../data/forklift_body.obj");
	mlevelA->AddAsset(mobjmesh);

				// ==Asset== Attach also a texture, that will affect only the 
				// assets in mlevelA:
	ChSharedPtr<ChTexture> mtexture(new ChTexture);
	mtexture->SetTextureFilename("../data/bluwhite.png");
	mlevelA->AddAsset(mtexture);
	
			// Change the position of mlevelA, thus moving also its sub-assets:
	mlevelA->GetFrame().SetPos(ChVector<>(0,0,2));
	mbody->AddAsset(mlevelA);

			// ==Asset== Attach sub level, then add to it an array of sub-levels, 
			// each rotated, and each containing a displaced box, thus making a 
			// spiral of cubes
	ChSharedPtr<ChAssetLevel> mlevelB(new ChAssetLevel);
	for (int j = 0; j<20; j++)
	{
				// ==Asset== the sub sub level..
		ChSharedPtr<ChAssetLevel> mlevelC(new ChAssetLevel);

					// ==Asset== the contained box..
		ChSharedPtr<ChBoxShape> msmallbox(new ChBoxShape);
		msmallbox->GetBoxGeometry().Pos = ChVector<>(0.4,0,0);
		msmallbox->GetBoxGeometry().Size = ChVector<>(0.1,0.1,0.01);
		mlevelC->AddAsset(msmallbox);

		ChQuaternion<> mrot;
		mrot.Q_from_AngAxis(j*21*CH_C_DEG_TO_RAD, ChVector<>(0,1,0));
		mlevelC->GetFrame().SetRot(mrot);
		mlevelC->GetFrame().SetPos(ChVector<>(0,j*0.02,0));

		mlevelB->AddAsset(mlevelC);
	}

	mbody->AddAsset(mlevelB);

			// ==Asset== Attach a video camera. This will be used by Irrlicht, 
			// or POVray postprocessing, etc. Note that a camera can also be 
			// put in a moving object.
	ChSharedPtr<ChCamera> mcamera(new ChCamera);
	mcamera->SetAngle(50);
	mcamera->SetPosition(ChVector<>(-3,4,-5));
	mcamera->SetAimPoint(ChVector<>(0,1,0));
	mbody->AddAsset(mcamera);

	/*
			//  ==Asset== IRRLICHT! Add a ChIrrNodeAsset so that Irrlicht will be able
			// to 'show' all the assets that we added to the body! 
			// OTHERWISE: use the application.AssetBind() function as at the end..
	ChSharedPtr<ChIrrNodeAsset> mirr_asset(new ChIrrNodeAsset);
	mirr_asset->Bind(mbody, application);
	mbody->AddAsset(mirr_asset);
	*/



	//
	// EXAMPLE 3: 
	//

	// Create a ChParticleClones cluster, and attach 'assets' 
	// that define a single "sample" 3D shape. This will be shown 
	// N times in Irrlicht.
	
			// Create the ChParticleClones, populate it with some random particles,
			// and add it to physical system:
	ChSharedPtr<ChParticlesClones> mparticles(new ChParticlesClones);

			// Note: coll. shape, if needed, must be specified before creating particles
	mparticles->GetCollisionModel()->ClearModel();
	mparticles->GetCollisionModel()->AddSphere(0.05);
	mparticles->GetCollisionModel()->BuildModel();
	mparticles->SetCollide(true);
		
			// Create the random particles 
	for (int np=0; np<100; ++np)
		mparticles->AddParticle(ChCoordsys<>(ChVector<>(ChRandom()-2,1.5,ChRandom()+2)));

			// Do not forget to add the particle cluster to the system:
	application.GetSystem()->Add(mparticles);

			//  ==Asset== Attach a 'sphere' shape asset.. it will be used as a sample 
			// shape to display all particles when rendering in 3D! 
	ChSharedPtr<ChSphereShape> mspherepart(new ChSphereShape);
	mspherepart->GetSphereGeometry().rad = 0.05;
	mparticles->AddAsset(mspherepart);

	/*
			//  ==Asset== IRRLICHT! Add a ChIrrNodeAsset so that Irrlicht will be able
			// to 'show' all the assets that we added to the body! 
			// OTHERWISE: use the application.AssetBind() function as at the end!
	ChSharedPtr<ChIrrNodeAsset> mirr_assetpart(new ChIrrNodeAsset);
	mirr_assetpart->Bind(mparticles, application);
	mparticles->AddAsset(mirr_assetpart);
	*/



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

	application.SetStepManage(true);
	application.SetTimestep(0.01);
	application.SetTryRealtime(true);

	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();
		
		application.DoStep();
			
		application.EndScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
