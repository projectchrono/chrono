///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - meshless deformable material
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
#include "physics/ChMatterSPH.h"
#include "physics/ChNodeBody.h"
#include "physics/ChProximityContainerSPH.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "core/ChRealtimeStep.h"
#include "irrlicht_interface/ChParticlesSceneNode.h" //***TEST****
#include "physics/ChContinuumMaterial.h" //***TEST****

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




void create_some_falling_items(ChIrrAppInterface& mapp) 
{
	ChSystem* mphysicalSystem = mapp.GetSystem();
	ISceneManager* msceneManager = mapp.GetSceneManager();
	IVideoDriver* driver = mapp.GetVideoDriver();

	ChSharedPtr<ChMatterSPH> mymatter(new ChMatterSPH);
	double size_x = 3.;
	double size_y = 3.;
	double size_z = 1.;
	int samples_x = 12;
	int samples_y = 12;
	int samples_z = 5;
	double step_x = size_x/(samples_x-1);
	double step_y = size_y/(samples_y-1);
	double step_z = size_z/(samples_z-1);
	for (int iy = samples_y; iy >0 ; iy--)
		for (int ix = 0; ix < samples_x; ix++)  
			for (int iz = 0; iz < samples_z; iz++) 
			{
				ChVector<> pos (	iy*step_y,
									ix*step_x,	
									iz*step_z  );
				mymatter->AddNode(pos);
			}
	mymatter->SetCollide(true);
	mphysicalSystem->Add(mymatter);

	GetLog() << "Steps x y z " << step_x << "  " << step_y << "  " << step_z << "\n\n";
	for (int ip = 0; ip < mymatter->GetNnodes(); ip++)
	{
		ChNodeSPH* mnode = (ChNodeSPH*)&(mymatter->GetNode(ip));
		mnode->SetKernelRadius(0.45);
		mnode->SetCollisionRadius(0.01);
		mnode->SetMass(0.1);
	}


	mymatter->GetMaterial().Set_v(0.38);
	mymatter->GetMaterial().Set_E(250);
	mymatter->GetMaterial().Set_elastic_yeld(0.12);
	mymatter->GetMaterial().Set_flow_rate(100.5);
	mymatter->SetViscosity(0.3);


	ChBodySceneNode* mrigidBody; 

	for (int bi = 0; bi < 4; bi++) 
	{    
		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
											mphysicalSystem, msceneManager,
											1.0,
											ChVector<>(-5+ChRandom()*10, 4+bi*0.05, -5+ChRandom()*10),
											1.1);
   
		mrigidBody->GetBody()->SetFriction(0.2f); 
		mrigidBody->addShadowVolumeSceneNode();


		video::ITexture* sphereMap = driver->getTexture("../data/bluwhite.png");
		mrigidBody->setMaterialTexture(0,	sphereMap);
	} 



	// Create the five walls of the rectangular container, using
	// fixed rigid bodies of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,-5,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20,1,20) );
	mrigidBody->GetBody()->SetBodyFixed(true);

	video::ITexture* cubeMap = driver->getTexture("../data/concrete.jpg");
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(-10,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1,10,20.99) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);


	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(10,0,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(1,10,20.99) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,0,-10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20.99,10,1) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											mphysicalSystem, msceneManager,
											100.0,
											ChVector<>(0,0, 10),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20.99,10,1) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->setMaterialTexture(0,	cubeMap);
 

	// Join some nodes of SPH matter to a ChBody
	ChSharedPtr<ChIndexedNodes> mnodes = mymatter;
	for (int ij = 0; ij < 120; ij++)
	{
		ChSharedPtr<ChNodeBody> myjointnodebody(new ChNodeBody);
		myjointnodebody->Initialize(mnodes, 
									ij, 
									mrigidBody->GetBody());
		mphysicalSystem->Add(myjointnodebody);
	}


	/*
	// Add also few spherical particles
	 ChParticlesSceneNode* mParticles = (ChParticlesSceneNode*)addChParticlesSceneNode_easySpheres(
												mphysicalSystem, msceneManager,
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
	ChGlobals* GLOBAL_Vars = DLL_CreateGlobals();

	ChStrainTensor<> mstrain;
	ChStressTensor<> mstress;
mstrain.XX() = 2;
mstrain.YY() = 13;
mstrain.ZZ() = 1;
mstrain.XY() = 4;
mstrain.XZ() = 2;
mstrain.YZ() = 6;
GetLog() << "mstrain:\n" << mstrain << "\n\n";

ChContinuumMaterial mmat;
mmat.Set_E(100);
mmat.Set_v(0.4);
mmat.ComputeElasticStress(mstress,mstrain);
GetLog() << "mstress:\n" << mstress << "\n\n";

mmat.ComputeElasticStrain(mstrain,mstress);
GetLog() << "mstress:\n" << mstrain << "\n\n";


	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&mphysicalSystem, L"Meshless deformable material",core::dimension2d<u32>(800,600),false);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,3,-3));

 
	// Create all the rigid bodies.

	create_some_falling_items(application);
 


	// Modify some setting of the physical system for the simulation, if you want

	mphysicalSystem.SetIntegrationType(ChSystem::INT_ANITESCU); 

	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_GPU);

	mphysicalSystem.SetIterLCPmaxItersSpeed(20);
	mphysicalSystem.SetIterLCPmaxItersStab(5);
 
	mphysicalSystem.SetUseSleeping(true);
 
	// This takes care of the interaction between the SPH particles
	ChSharedPtr<ChProximityContainerSPH> my_sph_proximity(new ChProximityContainerSPH);
	mphysicalSystem.Add(my_sph_proximity);
	


	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	static int printed_prox = 0;

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();
		
		ChSystem::IteratorOtherPhysicsItems myiter = mphysicalSystem.IterBeginOtherPhysicsItems();
		while (myiter != mphysicalSystem.IterEndOtherPhysicsItems())
		{ 
			if (ChMatterSPH* mymatter = dynamic_cast<ChMatterSPH*> ( (*myiter).get_ptr() ) )
			{
				++printed_prox;
				if (printed_prox>15)
				{
					GetLog() << "T=" <<mphysicalSystem.GetChTime() << ", Nproximities=" << my_sph_proximity->GetNproximities() << "   Avg.prox-per-node: " << (double)my_sph_proximity->GetNproximities()/(double)(15*15*5)<< "\n";
					printed_prox = 0;
				}

				for (unsigned int ip = 0; ip < mymatter->GetNnodes(); ip++)
				{
					ChNodeSPH* mnode = (ChNodeSPH*)&(mymatter->GetNode(ip));
					ChVector<> mv = mnode->GetPos();
					float rad = (float)mnode->GetKernelRadius(); 
					core::vector3df mpos((irr::f32)mv.x, (irr::f32)mv.y, (irr::f32)mv.z);
					core::position2d<s32> spos = application.GetSceneManager()->getSceneCollisionManager()->getScreenCoordinatesFrom3DPosition(mpos);
					application.GetVideoDriver()->draw2DRectangle(video::SColor(100,200,200,230), 
									core::rect<s32>(spos.X-2, spos.Y-2, spos.X+2, spos.Y+2) );
					/*
					application.GetVideoDriver()->setTransform(video::ETS_WORLD, core::matrix4());
					application.GetVideoDriver()->draw3DBox( core::aabbox3d<f32>(
									(irr::f32)mv.x-rad ,(irr::f32)mv.y-rad , (irr::f32)mv.z-rad    , 
									(irr::f32)mv.x+rad ,(irr::f32)mv.y+rad , (irr::f32)mv.z+rad )   ,
									video::SColor(300,200,200,230) );
					*/
					
					/*
					double strain_scale =50;
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_X*mnode->p_strain.XX()* strain_scale), video::SColor(255,255,0,0),false);
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_Y*mnode->p_strain.YY()* strain_scale), video::SColor(255,0,255,0),false);
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_Z*mnode->p_strain.ZZ()* strain_scale), video::SColor(255,0,0,255),false);
					*/

					/*
					double stress_scale =0.03;
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_X*mnode->t_stress.XX()* stress_scale), video::SColor(100,255,0,0),false);
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_Y*mnode->t_stress.YY()* stress_scale), video::SColor(100,0,255,0),false);
					ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(VECT_Z*mnode->t_stress.ZZ()* stress_scale), video::SColor(100,0,0,255),false);
					*/
					//ChIrrTools::drawSegment(application.GetVideoDriver(), mnode->GetPos(), mnode->GetPos()+(mnode->UserForce * 0.1), video::SColor(100,0,0,0),false);
					
				}
			}
			++myiter;
		}

		

		mphysicalSystem.DoStepDynamics( 0.001);
		
		application.GetVideoDriver()->endScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
