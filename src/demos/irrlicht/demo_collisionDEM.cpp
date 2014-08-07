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


#include "physics/ChSystemDEM.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChContactContainerDEM.h"

#include "collision/ChCModelBulletBody.h"

#include "lcp/ChLcpSolverDEM.h"

#include "unit_IRRLICHT/ChIrrApp.h"
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


void AddFallingItems(ChIrrApp& application)
{
	for (int ix = -2; ix < 3; ix++) {
		for (int iz = -2; iz < 3; iz++) {

			// Spheres
			{
				double mass = 1;
				double radius = 1.1;
				ChSharedPtr<ChBodyDEM> body(new ChBodyDEM);
				body->SetInertiaXX((2.0/5.0)*mass*pow(radius,2)*ChVector<>(1,1,1));
				body->SetMass(mass);
				body->SetPos(ChVector<>(4.0 * ix, 4.0, 4.0 * iz));

				body->GetCollisionModel()->ClearModel();
				body->GetCollisionModel()->AddSphere(radius);
				body->GetCollisionModel()->BuildModel();
				body->SetCollide(true);

				ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
				sphere->GetSphereGeometry().rad = radius;
				sphere->SetColor(ChColor(0.9f, 0.4f, 0.2f));
				body->AddAsset(sphere);

				application.GetSystem()->AddBody(body);
			}

			// Boxes
			{
				double mass = 1;
				ChVector<> hsize(0.75, 0.75, 0.75);
				ChSharedPtr<ChBodyDEM> body(new ChBodyDEM);

				body->SetMass(mass);
				body->SetPos(ChVector<>(4.0 * ix, 6.0, 4.0 * iz));
		
				body->GetCollisionModel()->ClearModel();
				body->GetCollisionModel()->AddBox(hsize.x, hsize.y, hsize.z);
				body->GetCollisionModel()->BuildModel();
				body->SetCollide(true);

				ChSharedPtr<ChBoxShape> box(new ChBoxShape);
				box->GetBoxGeometry().Size = hsize;
				box->SetColor(ChColor(0.4f, 0.9f, 0.2f));
				body->AddAsset(box);

				application.GetSystem()->AddBody(body);
			}
		}
	}
}

void AddContainerWall(ChSharedPtr<ChBodyDEM>& body,
                      const ChVector<>&       pos,
                      const ChVector<>&       size,
                      bool                    visible = true)
{
	ChVector<> hsize = 0.5 * size;

	body->GetCollisionModel()->AddBox(hsize.x, hsize.y, hsize.z, pos);

	if (visible) {
		ChSharedPtr<ChBoxShape> box(new ChBoxShape);
		box->GetBoxGeometry().Pos = pos;
		box->GetBoxGeometry().Size = hsize;
		box->SetColor(ChColor(1,0,0));
		box->SetFading(0.6f);
		body->AddAsset(box);
	}
}

void AddContainer(ChIrrApp& application)
{
	// The fixed body (5 walls)
	ChSharedPtr<ChBodyDEM> fixedBody(new ChBodyDEM);

	fixedBody->SetMass(1.0);
	fixedBody->SetBodyFixed(true);
	fixedBody->SetPos(ChVector<>());
	fixedBody->SetCollide(true);

	fixedBody->GetCollisionModel()->ClearModel();
	AddContainerWall(fixedBody, ChVector<>(  0,-5,  0), ChVector<>(20,     1, 20));
	AddContainerWall(fixedBody, ChVector<>(-10, 0,  0), ChVector<>( 1,    10, 20.99));
	AddContainerWall(fixedBody, ChVector<>( 10, 0,  0), ChVector<>( 1,    10, 20.99));
	AddContainerWall(fixedBody, ChVector<>(  0, 0,-10), ChVector<>(20.99, 10,  1), false);
	AddContainerWall(fixedBody, ChVector<>(  0, 0, 10), ChVector<>(20.99, 10,  1));
	fixedBody->GetCollisionModel()->BuildModel();

	application.GetSystem()->AddBody(fixedBody);

	// The rotating mixer body
	ChSharedPtr<ChBodyDEM> rotatingBody(new ChBodyDEM);

	rotatingBody->SetMass(10.0);
	rotatingBody->SetInertiaXX(ChVector<>(50,50,50));
	rotatingBody->SetPos(ChVector<>(0,-1.6,0));
	rotatingBody->SetCollide(true);

	ChVector<> hsize(5, 2.75, 0.5);

	rotatingBody->GetCollisionModel()->ClearModel();
	rotatingBody->GetCollisionModel()->AddBox(hsize.x, hsize.y, hsize.z);
	rotatingBody->GetCollisionModel()->BuildModel();

	ChSharedPtr<ChBoxShape> box(new ChBoxShape);
	box->GetBoxGeometry().Size = hsize;
	box->SetColor(ChColor(0,0,1));
	box->SetFading(0.6f);
	rotatingBody->AddAsset(box);

	application.GetSystem()->AddBody(rotatingBody);

	// An engine between the two
	ChSharedPtr<ChLinkEngine> my_motor(new ChLinkEngine);

        ChSharedPtr<ChBody> rotatingBody_(rotatingBody);
        ChSharedPtr<ChBody> fixedBody_(fixedBody);

	my_motor->Initialize(rotatingBody_,
	                     fixedBody_, 
	                     ChCoordsys<>(ChVector<>(0,0,0),
	                     Q_from_AngAxis(CH_C_PI_2, VECT_X)) );
	my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(my_motor->Get_spe_funct()))
		mfun->Set_yconst(CH_C_PI/2); // speed w=90°/s

	application.GetSystem()->AddLink(my_motor);
}


int main(int argc, char* argv[])
{
	// Simulation and rendering time-step
	double time_step = 1e-5;
	double out_step = 0.02;

	// Create a ChronoENGINE physical system
	ChSystemDEM mphysicalSystem;
	mphysicalSystem.Set_G_acc(0.38*mphysicalSystem.Get_G_acc());

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem,
	                     L"DEM collision demo",
	                     core::dimension2d<u32>(800,600),
	                     false,
	                     true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0,18,-20));

	// Add fixed and moving bodies
	AddContainer(application);
	AddFallingItems(application);

	// Complete asset specification: convert all assets to Irrlicht
	application.AssetBindAll();
	application.AssetUpdateAll();

	// The soft-real-time cycle
	double time = 0;
	double out_time = 0;
	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();

		while (time < out_time) {
			mphysicalSystem.DoStepDynamics(time_step);
			time += time_step;
		}
		out_time += out_step;

		application.EndScene();
	}


	return 0;
}

