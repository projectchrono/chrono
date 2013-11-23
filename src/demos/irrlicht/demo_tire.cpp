//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
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
//     - loading a .chulls file, with xyz points of 
//       cluters of convex hulls that define a complicate
//       concave shape. The shape is a wheel for tractors,
//       with large knobs, that has been decomposed using
//       demo_decomposition.exe from .obj shape to a .chull.  
// 
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
#include "physics/ChSystem.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "core/ChRealtimeStep.h"

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


ChBodySceneNode* create_wheel(ChVector<> mposition, ChIrrAppInterface& mapplication)
{
	ChCollisionModel::SetDefaultSuggestedEnvelope(0.005);
	ChCollisionModel::SetDefaultSuggestedMargin  (0.004);

	// the mesh for the visualization (independent from the collision shape)
	IAnimatedMesh*	tireMesh = mapplication.GetSceneManager()->getMesh("../data/tractor_wheel.obj");

	ChBodySceneNode* mrigidBody = (ChBodySceneNode*)addChBodySceneNode(
									mapplication.GetSystem(), mapplication.GetSceneManager(),
									tireMesh, // this mesh only for visualization
									50.0, 
									mposition  );

	mrigidBody->GetBody()->SetInertiaXX(ChVector<>(10,10,10));
	mrigidBody->GetBody()->SetFriction(0.5);

		// turn collision ON, otherwise no collide by default	
	mrigidBody->GetBody()->SetCollide(true);	 


		// Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
	mrigidBody->GetBody()->GetCollisionModel()->ClearModel();
		// Describe the (invisible) colliding shape by adding the 'carcass' decomposed shape and the 
		// 'knobs'. Since these decompositions are only for 1/15th of the wheel, use for() to pattern them.

	for (double mangle = 0; mangle < 360.; mangle+= (360./15.))
	{
		ChQuaternion<>myrot;
		ChStreamInAsciiFile myknobs("../data/tractor_wheel_knobs.chulls");
		ChStreamInAsciiFile myslice("../data/tractor_wheel_slice.chulls");
		myrot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
		ChMatrix33<> mm(myrot);
		mrigidBody->GetBody()->GetCollisionModel()->AddConvexHullsFromFile(myknobs, ChVector<>(0,0,0), mm);
		mrigidBody->GetBody()->GetCollisionModel()->AddConvexHullsFromFile(myslice, ChVector<>(0,0,0), mm);
		//break;
	}
	
		// Complete the description.
	mrigidBody->GetBody()->GetCollisionModel()->BuildModel();

	return mrigidBody;
}



void create_some_falling_items(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver)
{
	 // Make some pebbles, just for fun, under the wheel
	video::ITexture* cubeMap = driver->getTexture("../data/concrete.jpg");
	video::ITexture* rockMap = driver->getTexture("../data/rock.jpg");

	ChBodySceneNode* mrigidBody; 

	ChCollisionModel::SetDefaultSuggestedEnvelope(0.003);
	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);

	ChQuaternion<> rot;
	rot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);

	double bed_x = 0.6;
	double bed_z = 1;

	int n_pebbles = 30;
	for (int bi = 0; bi < n_pebbles; bi++) 
	{    
		double sphrad = 0.02 + 0.02*ChRandom();
		double sphdens = 1;
		double sphmass = (4/3)*CH_C_PI*pow(sphrad,3)*sphdens;
		double sphinertia = pow(sphrad,2) * sphmass;
		ChQuaternion<> randrot(ChRandom(),ChRandom(),ChRandom(),ChRandom());
		randrot.Normalize();

		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											&mphysicalSystem, msceneManager,
											(4/3)*CH_C_PI*pow(sphrad,3)*sphdens,
											ChVector<>(
												-0.5*bed_x+ChRandom()*bed_x, 
												0.01+ 0.04*((double)bi/(double)n_pebbles), 
												-0.5*bed_z+ChRandom()*bed_z),
											sphrad
											 );

		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
		mrigidBody->GetBody()->SetFriction(0.4f);
		mrigidBody->GetBody()->SetRot(randrot);
		//mrigidBody->addShadowVolumeSceneNode();
		mrigidBody->setMaterialTexture(0,	rockMap);

	} 


	// Create the a plane using body of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,-0.5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(10,1,10) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(0.5); 
	mrigidBody->setMaterialTexture(0,	cubeMap);

} 
     
  

 
   
 
int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&mphysicalSystem, L"Convex decomposed wheel",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(3.5f,2.5f,-2.4f));

 
	// Create some debris

	create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());
 

	// Create the wheel

	ChBodySceneNode* mwheelBody = create_wheel(ChVector<>(0,1,0), application);
 

	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	// This will help choosing an integration step which matches the
	// real-time step of the simulation, if possible.

	int nstep = 0;

	application.SetStepManage(true);
	application.SetTimestep(0.01);
	application.SetTryRealtime(true);

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();

		application.DoStep();
		

		application.GetVideoDriver()->endScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
