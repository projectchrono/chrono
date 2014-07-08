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
//     - conveyor belt primitive
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
#include "physics/ChBodyEasy.h"
#include "physics/ChConveyor.h"
#include "irrlicht_interface/ChIrrApp.h"

 


// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;
         
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 


// Static values valid through the entire program (bad
// programming practice, but enough for quick tests)

double STATIC_flow = 100;
double STATIC_speed = 2;
std::vector< ChSharedPtr<ChBody> > particlelist;


// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChIrrAppInterface* myapp)
			{
				// store pointer applicaiton
				application = myapp;

				// ..add a GUI slider to control particles flow
				scrollbar_flow = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 85, 650, 100), 0, 101);
				scrollbar_flow->setMax(300); 
				scrollbar_flow->setPos(150);
				text_flow = application->GetIGUIEnvironment()->addStaticText(
							L"Flow [particles/s]", rect<s32>(650,85,750,100), false);

				// ..add GUI slider to control the speed
				scrollbar_speed = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 125, 650, 140), 0, 102);
				scrollbar_speed->setMax(100); 
				scrollbar_speed->setPos(100);
				text_speed = application->GetIGUIEnvironment()->addStaticText(
								L"Conveyor speed [m/s]:", rect<s32>(650,125,750,140), false);
			}

	bool OnEvent(const SEvent& event)
			{

				// check if user moved the sliders with mouse..
				if (event.EventType == EET_GUI_EVENT)
				{
					s32 id = event.GUIEvent.Caller->getID();
					IGUIEnvironment* env = application->GetIGUIEnvironment();

					switch(event.GUIEvent.EventType)
					{
					case EGET_SCROLL_BAR_CHANGED:
							if (id == 101) // id of 'flow' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								STATIC_flow = (double)pos;
							}
							if (id == 102) // id of 'speed' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								STATIC_speed = (((double)pos)/100)*2;
							}
					break;
					}
					
				} 

				return false;
			}

private: 
	ChIrrAppInterface* application;

	IGUIScrollBar*  scrollbar_flow;
	IGUIStaticText* text_flow;
	IGUIScrollBar*  scrollbar_speed;
	IGUIStaticText* text_speed;
};



// Function that creates debris that fall on the conveyor belt, to be called at each dt

void create_debris(ChIrrApp& application, double dt, double particles_second)
{
	double xnozzlesize = 0.2;
	double znozzlesize = 0.56;
	double ynozzle = 0.2;

	double box_fraction = 0.3; // 30% cubes
	double cyl_fraction = 0.4; // 40% cylinders
	double sph_fraction = 1-box_fraction-cyl_fraction;

	double density = 1;
	double sphrad = 0.013;
	double sphmass = (4/3)*CH_C_PI*pow(sphrad,3)*density;
	double sphinertia = pow(sphrad,2) * sphmass;

	ChBodySceneNode* mrigidBody; 

	double exact_particles_dt = dt * particles_second;
	double particles_dt = floor(exact_particles_dt);
	double remaind = exact_particles_dt - particles_dt;
	if (remaind > ChRandom()) particles_dt +=1;

	video::ITexture* bluwhiteMap = application.GetVideoDriver()->getTexture("../data/bluwhite.png");
	video::ITexture* pinkwhiteMap = application.GetVideoDriver()->getTexture("../data/pinkwhite.png");

	for (int i = 0; i < particles_dt; i++)
	{
		double rand_fract = ChRandom();

		if (rand_fract < box_fraction)
		{
			ChSharedPtr<ChBodyEasySphere> mrigidBody (new ChBodyEasySphere(
											sphrad,		// size
											1,		    // density
											true,		// collide enable?
											true));		// visualization?
			mrigidBody->SetPos( ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize, ynozzle+i*0.005, -0.5*znozzlesize+ChRandom()*znozzlesize) );
			mrigidBody->SetFriction(0.2f);
			mrigidBody->SetImpactC(0.8f); 
			mrigidBody->AddAsset( ChSharedPtr<ChTexture>(new ChTexture("../data/bluwhite.png")) );

			application.GetSystem()->Add(mrigidBody);

			// This will make particle's visualization assets visible in Irrlicht:
			application.AssetBind(mrigidBody);
			application.AssetUpdate(mrigidBody);

			particlelist.push_back(mrigidBody);
		}

		if ((rand_fract > box_fraction) && 
			(rand_fract < box_fraction+cyl_fraction))
		{
			double xscale = 1.3*(1-0.8*ChRandom()); // for oddly-shaped boxes..
			double yscale = 1.3*(1-0.8*ChRandom());
			double zscale = 1.3*(1-0.8*ChRandom());

			ChSharedPtr<ChBodyEasyBox> mrigidBody (new ChBodyEasyBox(
											sphrad*2*xscale,
											sphrad*2*yscale,
											sphrad*2*zscale,	
											1,		    // density
											true,		// collide enable?
											true));		// visualization?
			mrigidBody->SetPos( ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize, ynozzle+i*0.005, -0.5*znozzlesize+ChRandom()*znozzlesize) );
			mrigidBody->SetFriction(0.4f);
			mrigidBody->AddAsset( ChSharedPtr<ChTexture>(new ChTexture("../data/cubetexture_bluwhite.png")) );

			application.GetSystem()->Add(mrigidBody);

			// This will make particle's visualization assets visible in Irrlicht:
			application.AssetBind(mrigidBody);
			application.AssetUpdate(mrigidBody);

			particlelist.push_back(mrigidBody);
		}

		if (rand_fract > box_fraction+cyl_fraction)
		{
			ChSharedPtr<ChBodyEasyCylinder> mrigidBody (new ChBodyEasyCylinder(
											sphrad,sphrad*2,	// rad, height
											1,		    // density
											true,		// collide enable?
											true));		// visualization?
			mrigidBody->SetPos( ChVector<>(-0.5*xnozzlesize+ChRandom()*xnozzlesize, ynozzle+i*0.005, -0.5*znozzlesize+ChRandom()*znozzlesize) );
			mrigidBody->SetFriction(0.2f);
			mrigidBody->AddAsset( ChSharedPtr<ChTexture>(new ChTexture("../data/pinkwhite.png")) );

			application.GetSystem()->Add(mrigidBody);

			// This will make particle's visualization assets visible in Irrlicht:
			application.AssetBind(mrigidBody);
			application.AssetUpdate(mrigidBody);

			particlelist.push_back(mrigidBody);
		}
	
	}

} 
     
  
// Function that deletes old debris (to avoid infinite creation that fills memory)

void purge_debris(ChIrrAppInterface& application, int nmaxparticles = 100)
{
	while (particlelist.size() > nmaxparticles)
	{
		application.GetSystem()->Remove(particlelist[0]);	// remove from physical simulation
		particlelist.erase(particlelist.begin());			// remove also from our particle list (will also automatically delete object thank to shared pointer)
	}
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
	ChIrrApp application(&mphysicalSystem, L"Conveyor belt",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(1.5,0.4,-1), core::vector3df(0.5,0,0) );


	// This is for GUI tweaking of system parameters..
	MyEventReceiver receiver(&application);
	  // note how to add the custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);


	// Set small collision envelopes for objects that will be created from now on..
	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
	ChCollisionModel::SetDefaultSuggestedMargin  (0.002);


	// Create two conveyor fences
	ChBodySceneNode* mfence1 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(0,0,-0.325),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(2,0.11,0.04) );
	mfence1->GetBody()->SetBodyFixed(true);
	mfence1->GetBody()->SetFriction(0.1);

	ChBodySceneNode* mfence2 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(0,0, 0.325),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(2,0.11,0.04) );
	mfence2->GetBody()->SetBodyFixed(true);
	mfence2->GetBody()->SetFriction(0.1);



	// Create the conveyor belt (this is a pure Chrono::Engine object, 
	// because an Irrlicht 'SceneNode' wrapper is not yet available, so it is invisible - no 3D preview)

	ChSharedPtr<ChConveyor> mconveyor (new ChConveyor(2, 0.05, 0.6));
	mconveyor->SetBodyFixed(true);
	mconveyor->SetFriction(0.35);
	mconveyor->SetConveyorSpeed(STATIC_speed);
	mconveyor->SetPos( ChVector<>(0, 0, 0) );

	mphysicalSystem.Add(mconveyor);


	// Create an Irrlicht 'directory' where debris will be put during the simulation loop
	ISceneNode* parent = application.GetSceneManager()->addEmptySceneNode();


	// 
	// THE SOFT-REAL-TIME CYCLE
	//
	
	application.SetStepManage(true);
	application.SetTimestep(0.005);

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();

		application.DoStep();

		if (!application.GetPaused())
		{
			// Continuosly create debris that fall on the conveyor belt
			create_debris(application, application.GetTimestep(), STATIC_flow);

			// Limit the max number of debris particles on the scene, deleting the oldest ones, for performance
			purge_debris (application, 300);

			// Maybe the user played with the slider and changed STATIC_speed...
		 	mconveyor->SetConveyorSpeed(STATIC_speed);
		}

		application.GetVideoDriver()->endScene();  
		
	}
	

 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
