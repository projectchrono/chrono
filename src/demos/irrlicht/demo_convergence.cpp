///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - solver convergence with high stacks of
//       objects.
// 
//  
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
   
 
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeMINRES.h"
 #include "irrlicht_interface/ChPovTools.h"

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




void create_items(ChIrrAppInterface& application) 
{
	ChBodySceneNode* mrigidBody;

	// Create some spheres in a vertical stack

	video::ITexture* sphereMap = application.GetVideoDriver()->getTexture("../data/bluwhite.png");

	double sphrad = 0.2;
	double dens= 1000;
	double sphmass = dens * (4./3.) * CH_C_PI * pow(sphrad,3);
	double sphinertia = (2./5.) * sphmass * pow(sphrad,2);

	for (int bi = 0; bi < 10; bi++)  // N. of vert. bricks
	{ 
		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											application.GetSystem(), application.GetSceneManager(),
											sphmass,
											ChVector<>(0.5, sphrad+bi*(2*sphrad), 0.7),
											sphrad);
   
		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
		mrigidBody->GetBody()->SetFriction(0.2f); 
		mrigidBody->GetBody()->SetImpactC(0.0f); 
		mrigidBody->addShadowVolumeSceneNode();

		mrigidBody->setMaterialTexture(0,	sphereMap);
	}

	// Create the floor using a fixed rigid body of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											application.GetSystem(), application.GetSceneManager(),
											1.0,
											ChVector<>(0,-2,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(50,4,50) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(0.4f);

	video::ITexture* cubeMap = application.GetVideoDriver()->getTexture("../data/concrete.jpg");
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
	ChIrrAppInterface application(&mphysicalSystem, L"Critical cases for solver convergence",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,1.5,-3));

 
	// Create all the rigid bodies.

	create_items(application);
 


	// Modify some setting of the physical system for the simulation, if you want

	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN);
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	mphysicalSystem.SetIterLCPmaxItersSpeed(60);
	mphysicalSystem.SetIterLCPmaxItersStab(5);

 
	//mphysicalSystem.SetUseSleeping(true);

	application.SetStepManage(true);
	application.SetTimestep(0.001);

	// 
	// THE SOFT-REAL-TIME CYCLE
	//

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
  
