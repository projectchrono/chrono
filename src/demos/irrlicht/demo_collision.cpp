///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - collisions and contacts 
//     - use the easy ChBodySceneNode class for 
//       managing Irrlicht objects which encapsulates 
//       ChBody items.
// 
//       (This is just a possible method of integration 
//       of Chrono::Engine + Irrlicht: many others
//       are possible.)
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




void create_some_falling_items(ChSystem& mphysicalSystem, ISceneManager* msceneManager, IVideoDriver* driver)
{
	ChBodySceneNode* mrigidBody; 


	for (int bi = 0; bi < 29; bi++) 
	{    
		// Create a bunch of ChronoENGINE rigid bodies (spheres and
		// boxes) which will fall..
		// Falling bodies are Irrlicht nodes of the special class ChBodySceneNode, 
		// which encapsulates ChBody items).  
		// Note that ChBodySceneNode have collision turned ON by default (so if you
		// want to have them 'transparent' to collision detection, you may use
		// mrigidBody->GetBody()->SetCollide(false) if you need..) 

  
		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(-5+ChRandom()*10, 4+bi*0.05, -5+ChRandom()*10),
											1.1);
   
		mrigidBody->GetBody()->SetFriction(0.2f); 
		mrigidBody->GetBody()->SetImpactC(1.0f); 
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
		mrigidBody->GetBody()->SetFriction(0.4f);
		mrigidBody->addShadowVolumeSceneNode(); 


		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(-5+ChRandom()*10, 4+bi*0.05, -5+ChRandom()*10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1.5,0.5,1.5) );

		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(1.2,1.2,1.2));
		mrigidBody->GetBody()->SetFriction(0.4f);
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
 
/* 
	/// NOTE: Instead of creating five separate 'box' bodies to make
	/// the walls of the bowl, you could have used a single body
	/// made of five box shapes, which build a single collision description,
	/// as in the alternative approach:

		// optional: maybe that you want to add a mesh for Irrlicht 3D viewing, maybe a
		// mesh representing the 5 walls, and use it for this ChBodySceneNode
	IAnimatedMesh* wallsMesh = msceneManager->getMesh("../data/nice_box_with_walls.obj");

		// create a plain ChBodySceneNode (no colliding shape nor visualization mesh is used yet)
	mrigidBody = (ChBodySceneNode*)addChBodySceneNode(
											&mphysicalSystem, msceneManager,
											wallsMesh, 1.0, 
											ChVector<>(0,0,0)  );

		// set the ChBodySceneNode as fixed body, and turn collision ON, otherwise no collide by default
	mrigidBody->GetBody()->SetBodyFixed(true);	
	mrigidBody->GetBody()->SetCollide(true);	 
		
		// Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
	mrigidBody->GetBody()->GetCollisionModel()->ClearModel();
		// Describe the (invisible) colliding shape by adding five boxes (the walls and floor)
	mrigidBody->GetBody()->GetCollisionModel()->AddBox(20,1,20, &ChVector<>(  0,-10,  0)); 
	mrigidBody->GetBody()->GetCollisionModel()->AddBox(1,40,20, &ChVector<>(-11,  0,  0));
	mrigidBody->GetBody()->GetCollisionModel()->AddBox(1,40,20, &ChVector<>( 11,  0,  0));
	mrigidBody->GetBody()->GetCollisionModel()->AddBox(20,40,1, &ChVector<>(  0,  0,-11));
	mrigidBody->GetBody()->GetCollisionModel()->AddBox(20,40,1, &ChVector<>(  0,  0, 11));
		// Complete the description.
	mrigidBody->GetBody()->GetCollisionModel()->BuildModel();
 
 */ 

	// Add the rotating mixer 
	ChBodySceneNode* rotatingBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(0,-1.6,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(10,5.5,1) ); 
	rotatingBody->GetBody()->SetMass(10);
	rotatingBody->GetBody()->SetInertiaXX(ChVector<>(50,50,50));
	rotatingBody->GetBody()->SetFriction(0.4f);
	rotatingBody->addShadowVolumeSceneNode();

	// .. an engine between mixer and truss	
	ChSharedPtr<ChLinkEngine> my_motor(new ChLinkEngine);
	my_motor->Initialize(rotatingBody->GetBody(), mrigidBody->GetBody(), 
				ChCoordsys<>(ChVector<>(0,0,0),
							 Q_from_AngAxis(CH_C_PI_2, VECT_X)) );
	my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(my_motor->Get_spe_funct()))
		mfun->Set_yconst(CH_C_PI/2.0); // speed w=90°/s
	mphysicalSystem.AddLink(my_motor);


	// Add also an oddly shaped object, loading from a mesh saved in '.X' fileformat.
	ChBodySceneNode* meshBody = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(&mphysicalSystem, msceneManager,
												1.0, ChVector<>(0,2,0),
												QUNIT, 
												"../data/rock.X" , 
												false,	// not static 
												true);	// true=convex; false=concave(do convex decomposition of concave mesh)
	meshBody->addShadowVolumeSceneNode();

	/*
	// Add also a 'barrel' type object
	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBarrel(
											&mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(0,6,1),
											2, 4,
											-0.8, 0.8, 
											-0.5);
	mrigidBody->GetBody()->SetWvel_loc(ChVector<>(0.3,0,0));
	video::ITexture* barrellMap = driver->getTexture("../data/pinkwhite.png");
	mrigidBody->setMaterialTexture(0,	barrellMap);
	*/

	/*
	// Add also few spherical particles
	 ChParticlesSceneNode* mParticles = (ChParticlesSceneNode*)addChParticlesSceneNode_easySpheres(
												&mphysicalSystem, application.GetSceneManager(),
												0.8, // mass
												0.4 // radius
												);

	 for (int np = 0; np <10; np++) 
	 {
		 mParticles->GetParticles()->AddParticle(ChCoordsys<>(ChVector<>(-1,np,0), QUNIT));
	 }
	*/

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
	ChIrrAppInterface application(&mphysicalSystem, L"Collisions between objects",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,14,-20));

 
	// Create all the rigid bodies.

	create_some_falling_items(mphysicalSystem, application.GetSceneManager(), application.GetVideoDriver());
 


	// Modify some setting of the physical system for the simulation, if you want

	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	mphysicalSystem.SetIterLCPmaxItersSpeed(20);
	mphysicalSystem.SetIterLCPmaxItersStab(5);

 
	//mphysicalSystem.SetUseSleeping(true);

	application.SetStepManage(true);
	application.SetTimestep(0.02);

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
  
