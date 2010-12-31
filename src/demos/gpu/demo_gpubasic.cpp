///////////////////////////////////////////////////
//
//   Demo code about  
// 
//     - using GPU solver - Tasora implementation.
//
//	 NOTE! this program should be copied
//   on multiple hosts of a cluster and executed 
//   using the launcher utility of the MPICH2 
//   toolchain (ex. mpiexec or wmpiexec.exe).
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

#include "unit_GPU/ChLcpIterativeSolverGPUsimple.h"		// <--
#include "unit_GPU/ChContactContainerGPUsimple.h"		// <--

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




void create_some_falling_items(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver)
{
	ChBodySceneNode* mrigidBody; 


	for (int bi = 0; bi < 29; bi++) 
	{    
		// Create a bunch of ChronoENGINE rigid bodies (spheres and
		// boxes) which will fall.

		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(-5+ChRandom()*10, 4+bi*0.05, -5+ChRandom()*10),
											1.1);
   
		mrigidBody->GetBody()->SetFriction(0.2); 
		mrigidBody->GetBody()->SetImpactC(1.0); 
		mrigidBody->addShadowVolumeSceneNode();


		video::ITexture* sphereMap = driver->getTexture("../data/bluwhite.png");
		mrigidBody->setMaterialTexture(0,	sphereMap);

		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(-5+ChRandom()*10, 4+bi*0.05, -5+ChRandom()*10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1.5,1.5,1.5) );

		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(1.2,1.2,1.2));
		mrigidBody->GetBody()->SetFriction(0.4);
		mrigidBody->addShadowVolumeSceneNode(); 


		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(-5+ChRandom()*10, 4+bi*0.05, -5+ChRandom()*10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1.5,0.5,1.5) );

		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(1.2,1.2,1.2));
		mrigidBody->GetBody()->SetFriction(0.4);
		mrigidBody->addShadowVolumeSceneNode();

		video::ITexture* cylinderMap = driver->getTexture("../data/pinkwhite.png");
		mrigidBody->setMaterialTexture(0,	cylinderMap);

	} 

	// Create the five walls of the rectangular container, using
	// fixed rigid bodies of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,-5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20,1,20) );
	mrigidBody->GetBody()->SetBodyFixed(true);

	video::ITexture* cubeMap = driver->getTexture("../data/concrete.jpg");
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(-10,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1,10,20.99) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(10,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1,10,20.99) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,0,-10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20.99,10,1) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,0, 10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20.99,10,1) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);
 
	// Add the rotating mixer 
	ChBodySceneNode* rotatingBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(0,-1.6,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(10,5.5,1) ); 
	rotatingBody->GetBody()->SetMass(100);
	rotatingBody->GetBody()->SetInertiaXX(ChVector<>(100,100,100));
	rotatingBody->GetBody()->SetFriction(0.4);
	rotatingBody->addShadowVolumeSceneNode();

	// .. an engine between mixer and truss	
	ChSharedPtr<ChLinkEngine> my_motor(new ChLinkEngine);
	my_motor->Initialize(rotatingBody->GetBody(), mrigidBody->GetBody(), 
				ChCoordsys<>(ChVector<>(0,0,0),
							 Q_from_AngAxis(CH_C_PI_2, VECT_X)) );
	my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	my_motor->Get_spe_funct()->Set_yconst(CH_C_PI/2.0); // speed w=90°/s
	mphysicalSystem.AddLink(my_motor);


	// Add also an oddly shaped object, loading from a mesh saved in '.X' fileformat.
	ChBodySceneNode* meshBody = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(&mphysicalSystem, msceneManager,
												1.0, ChVector<>(0,2,0),
												QUNIT, 
												"../data/rock.X" , 
												false,	// not static 
												true);	// true=convex; false=concave(do convex decomposition of concave mesh)
	meshBody->addShadowVolumeSceneNode();

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
	ChIrrAppInterface application(&mphysicalSystem, L"Collisions between objects - GPU",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,14,-20));

 
	// Create all the rigid bodies.

	create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());
 


	// Modify some setting of the physical system for the simulation, 
	// by plugging in some GPU-enabled components.
	//
	// In this example, we use the ChLcpIterativeCuda (implementation by A.Tasora)
	// for the solver (that must be used together with a ChContactContainerGPUsimple!) 
	// and no GPU for collision detection.
	// Note that currently the  ChLcpIterativeCuda  does not have fall-back
	// for the case of default contact data, so it _necessarily_ needs that
	// you also plug-in the ChContactContainerGPUsimple  contact container (it
	// creates and manages contact data that is better fit in the ChLcpIterativeCuda)
	// More advanced GPU components (ex. the H.Mazhar work) allow plugging in
	// also GPU collision detection that avoids bottlenecks in cd->solver data transfer.
	ChLcpSystemDescriptorGPU		newdescriptor(1000,1000*5, 1000);
	ChContactContainerGPUsimple		mGPUcontactcontainer;
	ChLcpIterativeSolverGPUsimple	mGPUsolverSpeed(&mGPUcontactcontainer, 50, false, 0, 0.2, 20000, 3000, 1000);
	ChLcpIterativeSolverGPUsimple	mGPUsolverPos(&mGPUcontactcontainer,  50, false, 0, 0.2, 20000, 3000, 1000);

	mphysicalSystem.ChangeContactContainer(&mGPUcontactcontainer);
	mphysicalSystem.ChangeLcpSolverSpeed(&mGPUsolverSpeed);
	mphysicalSystem.ChangeLcpSolverStab(&mGPUsolverPos);  // this is used only with Tasora timestepping, unneded for Anitescu
	mphysicalSystem.ChangeLcpSystemDescriptor(&newdescriptor);
	mphysicalSystem.SetIterLCPmaxItersSpeed(20);
	mGPUsolverSpeed.SetDt(0.02);
	mGPUsolverPos.SetDt(0.02);
	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();
		
		mphysicalSystem.DoStepDynamics( 0.02);
		

		application.GetVideoDriver()->endScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}