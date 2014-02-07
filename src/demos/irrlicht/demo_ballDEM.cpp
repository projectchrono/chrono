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
#include "physics/ChSystem.h"

#include "physics/ChBodyDEM.h"
#include "physics/ChContactContainerDEM.h"
#include "collision/ChCModelBulletBody.h"
#include "lcp/ChLcpSolverDEM.h"

#include "irrlicht_interface/ChIrrApp.h"
#include <irrlicht.h>

#define SPH_ID  33
#define BOX_ID  200

// Use the namespace of Chrono
using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;


void AddBoundaryBox(ChIrrApp& application,
                    double width,
                    double depth,
                    double height)
{
	ChVector<> hsize(width/2, height/2, depth/2);
	ChVector<> pos(0, -height/2, 0);
	ChSharedPtr<ChBodyDEM> body(new ChBodyDEM);

	body->SetIdentifier(BOX_ID);
	body->SetMass(1.0);
	body->SetBodyFixed(true);
	body->SetPos(pos);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	body->GetCollisionModel()->AddBox(hsize.x, hsize.y, hsize.z);
	body->GetCollisionModel()->BuildModel();

	ChSharedPtr<ChBoxShape> box(new ChBoxShape);
	box->GetBoxGeometry().Size = hsize;
	box->SetColor(ChColor(1,0,0));
	box->SetFading(0.6f);
	body->AddAsset(box);

	application.GetSystem()->Add(body);
}


void AddSphere(ChIrrApp& application, double radius, double mass, double height)
{
	ChVector<> pos(0.0, height, 0.0);

	ChSharedPtr<ChBodyDEM> body(new ChBodyDEM);
	body->SetIdentifier(SPH_ID);

	body->GetCollisionModel()->ClearModel();
	body->GetCollisionModel()->AddSphere(radius);
	body->GetCollisionModel()->BuildModel();
	body->SetCollide(true);

	body->SetPos(pos);
	body->SetInertiaXX((2.0/5.0)*mass*pow(radius,2)*ChVector<>(1,1,1));
	body->SetMass(mass);

	application.GetSystem()->Add(body);

	ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
	sphere->GetSphereGeometry().rad = radius;
	sphere->SetColor(ChColor(0.9f, 0.4f, 0.2f));
	body->AddAsset(sphere);
}


int main(int argc, char* argv[])
{
	// Fixed box parameters
	double D1 = 3.0;
	double D3 = 3.0;
	double th = 0.1;

	// Ball parameters
	double radius = 1.0;
	double mass = 1000;
	double height = 1.5;

	// Simulation and rendering time-step
	double time_step = 0.00001;
	double out_step = 2000 * time_step;
	double time = 0.0;
	double out_time = 0.0;

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	ChGlobals* GLOBAL_Vars = DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;
	mphysicalSystem.Set_G_acc(0.38*mphysicalSystem.Get_G_acc());

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Assets for Irrlicht visualization",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0,3,-6));
	//application.AddTypicalCamera(core::vector3df(0,0,-6));

	// Add fixed and moving bodies
	AddBoundaryBox(application, D1, D3, th);
	AddSphere(application, radius, mass, height);

	// Complete ...
	application.AssetBindAll();
	application.AssetUpdateAll();

	// Modify some setting of the physical system for the simulation, if you want
	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_DEM);

	// Use the DEM solver
	ChLcpSolverDEM* mysolver = new ChLcpSolverDEM;
	mphysicalSystem.ChangeLcpSolverSpeed(mysolver);
	mysolver->SetMaxIterations(0);

	// Use DEM contact
	// This takes care of the contact between the particles and with the wall
	ChContactContainerDEM* mycontainer = new ChContactContainerDEM;
	mphysicalSystem.ChangeContactContainer(mycontainer);

	// The soft-real-time cycle
	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();

		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.2, 0.2, 20,20, 
			ChCoordsys<>(ChVector<>(0,0,0), Q_from_AngX(CH_C_PI_2) ),
			video::SColor(255, 80,100,100), true);

		while (time < out_time) {
			mphysicalSystem.DoStepDynamics(time_step);
			time += time_step;
		}
		out_time += out_step;

		application.EndScene();
	}


	DLL_DeleteGlobals();

	// Note: we do not need to delete mysolver and mycontainer since these will be
	// freed in the ChSystem destructor.

	return 0;
}

