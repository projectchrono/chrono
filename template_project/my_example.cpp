//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

// A very simple example that can be used as template project for 
// a Chrono::Engine simulator with 3D view.
 
   
 
#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "physics/ChLinkMate.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "unit_IRRLICHT/ChIrrApp.h"
 


// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht

using namespace irr;
using namespace irr::core;
using namespace irr::scene; 
using namespace irr::video;
using namespace irr::io; 
using namespace irr::gui; 



int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"A simple project template",core::dimension2d<u32>(800,600),false); //screen dimensions

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(2,2,-5), core::vector3df(0,1,0));		//to change the position of camera
	//application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));
 

	//======================================================================

	// HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.
	//
	// An example: a pendulum.


	// 1-Create a floor that is fixed (that is used also to represent the absolute reference)

	ChSharedPtr<ChBodyEasyBox> floorBody(new ChBodyEasyBox( 10,2,10,  3000,	false, true));		//to create the floor, false -> doesn't represent a collide's surface
	floorBody->SetPos( ChVector<>(0,-2,0) );
	floorBody->SetBodyFixed(true);		

	mphysicalSystem.Add(floorBody);


	// 2-Create a pendulum 

	ChSharedPtr<ChBodyEasyBox> pendulumBody(new ChBodyEasyBox( 0.5,2,0.5,  3000, false, true));		//to create the floor, false -> doesn't represent a collide's surface
	pendulumBody->SetPos( ChVector<>(0, 3,0) );	
	pendulumBody->SetPos_dt( ChVector<>(1,0,0) );

	mphysicalSystem.Add(pendulumBody);


	// 3-Create a spherical constraint. 
	//   Here we'll use a ChLinkMateGeneric, but we could also use ChLinkLockSpherical

	ChSharedPtr<ChLinkMateGeneric> sphericalLink(new ChLinkMateGeneric(true,true,true, false,false,false)); // x,y,z,Rx,Ry,Rz constrains
	ChFrame<> link_position_abs(ChVector<>(0,4,0)); 

	sphericalLink->Initialize(	pendulumBody,			// the 1st body to connect
								floorBody,				// the 2nd body to connect
								false,					// the two following frames are in absolute, not relative, coords. 
								link_position_abs,		// the link reference attached to 1st body
								link_position_abs);		// the link reference attached to 2nd body

	mphysicalSystem.Add(sphericalLink);




	// optional, attach a RGB color asset to the floor, for better visualization
	ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset());
    mcolor->SetColor(ChColor(0.2, 0.25, 0.25));
	floorBody->AddAsset(mcolor);		

	// optional, attach a texture to the pendulum, for better visualization
	ChSharedPtr<ChTexture> mtexture(new ChTexture());
    mtexture->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));		//texture in ../data
	pendulumBody->AddAsset(mtexture);		



	//======================================================================
	

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes 
	application.AssetUpdateAll();


	// Adjust some settings:
	application.SetTimestep(0.005);
	application.SetTryRealtime(true);


	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	while (application.GetDevice()->run())
	{
		application.BeginScene();

		application.DrawAll();

		// This performs the integration timestep!
		application.DoStep();

		application.EndScene();
	}


	return 0;
}
  
