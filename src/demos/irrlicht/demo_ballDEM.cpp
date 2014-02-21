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
//     - collisions and contacts  with DEM
//  
//   CHRONO 
//   ------
//   Multibody dinamics engine
//  
///////////////////////////////////////////////////


#include "physics/ChApidll.h" 
#include "physics/ChSystemDEM.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChContactContainerDEM.h"

#include "collision/ChCModelBulletBody.h"

#include "lcp/ChLcpSolverDEM.h"

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


void AddWall(ChSharedBodyDEMPtr&  body,
             const ChVector<>&    dim,
             const ChVector<>&    loc)
{
	body->GetCollisionModel()->AddBox(dim.x, dim.y, dim.z, loc);

	ChSharedPtr<ChBoxShape> box(new ChBoxShape);
	box->GetBoxGeometry().Size = dim;
	box->GetBoxGeometry().Pos = loc;
	box->SetColor(ChColor(1,0,0));
	box->SetFading(0.6f);
	body->AddAsset(box);
}


int main(int argc, char* argv[])
{
	// Simulation parameters
	double gravity = -9.81;
	double time_step = 0.00001;
	double out_step = 2000 * time_step;

	// Parameters for the falling ball
	int             ballId = 100;
	double          radius = 1;
	double          mass = 1000;
	ChVector<>      pos(0, 2, 0);
	ChQuaternion<>  rot(1, 0, 0, 0);
	ChVector<>      init_vel(0, 0, 0);

	// Parameters for the containing bin
	int    binId = 200;
	double width = 2;
	double length = 2;
	double height = 1;
	double thickness = 0.1;

	// Initialize globals
	ChGlobals* GLOBAL_Vars = DLL_CreateGlobals();

	// Create the system
	ChSystemDEM msystem;

	msystem.Set_G_acc(ChVector<>(0, gravity, 0));

	// Create the Irrlicht visualization
	ChIrrApp application(&msystem, L"DEM demo",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0,3,-6));

	// Create a material (will be used by both objects)
	ChSharedPtr<ChMaterialSurfaceDEM> material;
	material = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	material->SetDissipationFactor(0.1f);
	material->SetFriction(0.4f);

	// Create the falling ball
	ChSharedPtr<ChBodyDEM> ball(new ChBodyDEM);

	ball->SetIdentifier(ballId);
	ball->SetMass(mass);
	ball->SetPos(pos);
	ball->SetRot(rot);
	ball->SetPos_dt(init_vel);
	ball->SetBodyFixed(false);
	ball->SetMaterialSurfaceDEM(material);

	ball->SetCollide(true);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(radius);
	ball->GetCollisionModel()->BuildModel();

	ball->SetInertiaXX(0.4*mass*radius*radius*ChVector<>(1,1,1));

	ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
	sphere->GetSphereGeometry().rad = radius;
	ball->AddAsset(sphere);

	ChSharedPtr<ChTexture> mtexture(new ChTexture);
	mtexture->SetTextureFilename("../data/bluwhite.png");
	ball->AddAsset(mtexture);

	msystem.AddBody(ball);

	// Create container
	ChSharedPtr<ChBodyDEM> bin(new ChBodyDEM);

	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0, 0, 0));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);
	bin->SetMaterialSurfaceDEM(material);

	bin->GetCollisionModel()->ClearModel();
	AddWall(bin, ChVector<>(width, thickness, length), ChVector<>(0, 0, 0));
	//AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(-width + thickness, height, 0));
	//AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(width - thickness, height, 0));
	//AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, -length + thickness));
	//AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, length - thickness));
	bin->GetCollisionModel()->BuildModel();

	msystem.AddBody(bin);


	// Complete asset construction
	application.AssetBindAll();
	application.AssetUpdateAll();

	// The soft-real-time cycle
	double time = 0.0;
	double out_time = 0.0;

	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();

		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.2, 0.2, 20,20, 
			ChCoordsys<>(ChVector<>(0,0,0), Q_from_AngX(CH_C_PI_2) ),
			video::SColor(255, 80,100,100), true);

		while (time < out_time) {
			msystem.DoStepDynamics(time_step);
			time += time_step;
		}
		out_time += out_step;

		application.EndScene();
	}


	DLL_DeleteGlobals();

	return 0;
}

