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

///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - using the ChParticleEmitter to create flows
//       of random shapes 
//     - use a ChParticleRemover to remove particles outside a volume
//     - use a ChParticleProcessor to compute mass flow etc.
//     - use Irrlicht to display objects.
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
 
   
 
#include "physics/ChSystem.h"
#include "particlefactory/ChParticleEmitter.h"
#include "particlefactory/ChParticleRemover.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "unit_IRRLICHT/ChIrrApp.h"
 


// Use the main namespace of Chrono, and other chrono namespaces

using namespace chrono;
using namespace particlefactory;

// Use the main namespaces of Irrlicht
using namespace irr;
         
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 


   
 
int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Particle emitter, remover, processor",core::dimension2d<u32>(800,600),false);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,7,-10));

 
	//
	// CREATE THE SYSTEM OBJECTS
	// 

	// Create the floor:

	ChSharedPtr<ChBodyEasyBox> floorBody(new ChBodyEasyBox( 20,1,20,  1000,	true, true));
	floorBody->SetPos( ChVector<>(0,-5,0) );
	floorBody->SetBodyFixed(true);

	ChSharedPtr<ChTexture> mtexture(new ChTexture());
	mtexture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
	floorBody->AddAsset(mtexture);

	mphysicalSystem.Add(floorBody);


	// Create an emitter:

	ChParticleEmitter  emitter;

	// Ok, that object will take care of generating particle flows for you.
	// It accepts a lot of settings, for creating many different types of particle
	// flows, like fountains, outlets of various shapes etc. 
	// For instance, set the flow rate, etc:
	
	emitter.ParticlesPerSecond() = 20;

	emitter.SetUseParticleReservoir(true);
	emitter.ParticleReservoirAmount() = 200;
	
	// Our ChParticleEmitter object, among the main settings, it requires 
	// that you give him four 'randomizer' objects: one is in charge of 
	// generating random shapes, one is in charge of generating
	// random positions, one for random alignements, and one for random velocities.
	// In the following we need to instance such objects. (There are many ready-to-use
	// randomizer objects already available in chrono, but note that you could also
	// inherit your own class from these randomizers if the choice is not enough).


	// ---Initialize the randomizer for positions
	ChSharedPtr<ChRandomParticlePositionRectangleOutlet> emitter_positions (new ChRandomParticlePositionRectangleOutlet);
	emitter_positions->Outlet() = ChCoordsys<>( ChVector<>(0,3,0), Q_from_AngAxis(CH_C_PI_2,VECT_X) ); // center and alignment of the outlet
	emitter_positions->OutletWidth() = 3.0;  
	emitter_positions->OutletHeight() = 4.5; 
	emitter.SetParticlePositioner(emitter_positions);


	// ---Initialize the randomizer for alignments
	ChSharedPtr<ChRandomParticleAlignmentUniform> emitter_rotations (new ChRandomParticleAlignmentUniform);
	emitter.SetParticleAligner(emitter_rotations);
	

	// ---Initialize the randomizer for velocities, with statistical distribution
	ChSharedPtr<ChRandomParticleVelocityConstantDirection> mvelo(new ChRandomParticleVelocityConstantDirection);
	mvelo->SetDirection(-VECT_Y);
	mvelo->SetModulusDistribution(0.0);

	emitter.SetParticleVelocity(mvelo);


	// ---Initialize the randomizer for creations, with statistical distribution

	 // Create a ChRandomShapeCreator object (ex. here for sphere particles)
	ChSharedPtr<ChRandomShapeCreatorSpheres> mcreator_metal(new ChRandomShapeCreatorSpheres);
	mcreator_metal->SetDiameterDistribution( ChSmartPtr<ChMinMaxDistribution>  (new ChMinMaxDistribution(0.2, 0.6)) );
	mcreator_metal->SetDensityDistribution ( ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1600)) );

	 // Optional: define a callback to be exectuted at each creation of a sphere particle:
	class MyCreator_metal : public ChCallbackPostCreation
	{
		// Here do custom stuff on the just-created particle:
		public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator)
		{
			  // Ex.: attach some optional assets, ex for visualization 
			ChSharedPtr<ChColorAsset> mvisual(new ChColorAsset);
			mvisual->SetColor(ChColor(0.9f,(float)ChRandom(),0.0f));
			mbody->AddAsset(mvisual);
		}
	};
	MyCreator_metal* callback_metal = new MyCreator_metal;
	mcreator_metal->SetCallbackPostCreation(callback_metal);


	 // Create a ChRandomShapeCreator object (ex. here for box particles)
	ChSharedPtr<ChRandomShapeCreatorBoxes> mcreator_plastic(new ChRandomShapeCreatorBoxes);
	mcreator_plastic->SetXsizeDistribution      ( ChSmartPtr<ChZhangDistribution>   (new ChZhangDistribution(0.5,0.2)) ); // Zhang parameters: average val, min val. 
	mcreator_plastic->SetSizeRatioZDistribution ( ChSmartPtr<ChMinMaxDistribution>  (new ChMinMaxDistribution(0.2, 1.0)) );
	mcreator_plastic->SetSizeRatioYZDistribution( ChSmartPtr<ChMinMaxDistribution>  (new ChMinMaxDistribution(0.4, 1.0)) );
	mcreator_plastic->SetDensityDistribution    ( ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1000)) );

	 // Optional: define a callback to be exectuted at each creation of a box particle:
	class MyCreator_plastic : public ChCallbackPostCreation
	{
		// Here do custom stuff on the just-created particle:
		public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator)
		{
			  // Ex.: attach some optional assets, ex for visualization 
			ChSharedPtr<ChColorAsset> mvisual(new ChColorAsset);
			mvisual->SetColor(ChColor(0.0f,1.0f,(float)ChRandom()));
			mbody->AddAsset(mvisual);
		}
	};
	MyCreator_plastic* callback_plastic = new MyCreator_plastic;
	mcreator_plastic->SetCallbackPostCreation(callback_plastic);


	 // Create a parent ChRandomShapeCreator that 'mixes' the two generators above,
	 // mixing them with a given percentual:
	ChSharedPtr<ChRandomShapeCreatorFromFamilies> mcreatorTot(new ChRandomShapeCreatorFromFamilies);
	mcreatorTot->AddFamily(mcreator_metal,   0.4);	// 1st creator family, with percentual
	mcreatorTot->AddFamily(mcreator_plastic, 0.6);	// 2nd creator family, with percentual
	mcreatorTot->Setup();
	 // By default, percentuals are in terms of number of generated particles, 
	 // but you can optionally enforce percentuals in terms of masses:
	mcreatorTot->SetProbabilityMode(ChRandomShapeCreatorFromFamilies::MASS_PROBABILITY);
	 // Finally, tell to the emitter that it must use the 'mixer' above:
	emitter.SetParticleCreator(mcreatorTot);


	// --- Optional: what to do by default on ALL newly created particles? 
	//     A callback executed at each particle creation can be attached to the emitter.
	//     For example, we need that new particles will be bound to Irrlicht visualization:

	// a- define a class that implement your custom PostCreation method...
	class MyCreatorForAll : public ChCallbackPostCreation
	{
		public: virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator)
		{
			// Enable Irrlicht visualization for all particles
			airrlicht_application->AssetBind(mbody);
			airrlicht_application->AssetUpdate(mbody);

			// Other stuff, ex. disable gyroscopic forces for increased integrator stabilty
			mbody->SetNoGyroTorque(true);
		}
		irr::ChIrrApp* airrlicht_application;
	};
	// b- create the callback object...
	MyCreatorForAll* mcreation_callback = new MyCreatorForAll;
	// c- set callback own data that he might need...
	mcreation_callback->airrlicht_application = &application;
	// d- attach the callback to the emitter!
	emitter.SetCallbackPostCreation(mcreation_callback);



	// Create the remover, i.e. an object that takes care 
	// of removing particles that are inside or outside some volume.
	// The fact that particles are handled with shared pointers means that,
	// after they are removed from the ChSystem, they are also automatically
	// deleted if no one else is referencing them.

	ChParticleRemoverBox remover;
	remover.SetRemoveOutside(true);
	remover.GetBox().Pos = ChVector<>(0,0,0);
	remover.GetBox().SetLengths( ChVector<>(5,20,5) );

	// Test also a ChParticleProcessor configured as a
	// counter of particles that flow into a rectangle:
	//  -create the trigger:
	ChSharedPtr<ChParticleEventFlowInRectangle> rectangleflow (new ChParticleEventFlowInRectangle(8,8));
	rectangleflow->rectangle_csys = ChCoordsys<>( ChVector<>(0,2,0), Q_from_AngAxis(-CH_C_PI_2,VECT_X) ); // center and alignment of rectangle
	rectangleflow->margin = 1;
	//  -create the counter:
	ChSharedPtr<ChParticleProcessEventCount> counter (new ChParticleProcessEventCount);
	//  -create the processor and plug in the trigger and the counter:
	ChParticleProcessor processor_flowcount;
	processor_flowcount.SetEventTrigger(rectangleflow);
	processor_flowcount.SetParticleEventProcessor(counter);



	// Use this function for adding a ChIrrNodeAsset to all already created items (ex. the floor, etc.)
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes 
	application.AssetUpdateAll();


	// Modify some setting of the physical system for the simulation, if you want
	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	mphysicalSystem.SetIterLCPmaxItersSpeed(40);
	mphysicalSystem.SetIterLCPmaxItersStab(5);


	application.SetTimestep(0.02);

	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();
		
		// Continuosly create particle flow:
		emitter.EmitParticles(mphysicalSystem, application.GetTimestep());

		// Continuosly check if some particle must be removed:
		remover.ProcessParticles(mphysicalSystem);

		// Use the processor to count particle flow in the rectangle section:
		processor_flowcount.ProcessParticles(mphysicalSystem);
		GetLog() << "Particles being flown across rectangle:" << counter->counter << "\n";

		application.DoStep();	

		application.GetVideoDriver()->endScene();  
	}
	
	return 0;
}
  
