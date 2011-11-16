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

// Static data used for this simple demo

std::vector<ChBodySceneNode*> mspheres;

double STATIC_COMPLIANCE = 0.1*   (10./1000.)/500; // as 1/K, in m/N. es: 10mm/500N




void create_items(ChIrrAppInterface& application) 
{
	ChBodySceneNode* mrigidBody;

	// Create some spheres in a vertical stack, and put them into 'parent' level

	video::ITexture* sphereMap = application.GetVideoDriver()->getTexture("../data/bluwhite.png");

	bool do_wall = false;
	bool do_stack = true;
	bool do_oddmass = true;
	bool do_spheres = true;
	bool do_heavyonside = true;
	

	double sphrad = 0.2;
	double dens= 1000;
	double sphmass = dens * (4./3.) * CH_C_PI * pow(sphrad,3);
	double sphinertia = (2./5.) * sphmass * pow(sphrad,2);

	if (do_stack)
	{
		int nbodies = 15;

		double totmass= 0;
		double level  = 0;
		double sphrad_base = 0.2;
		double oddfactor = 100;

		for (int bi = 0; bi < nbodies; bi++)  // N. of vert. bricks
		{ 
			double sphrad = sphrad_base;
			if (do_oddmass && bi==(nbodies-1))
				sphrad = sphrad*pow(oddfactor, 1./3.);
			double dens= 1000;
			double sphmass = dens * (4./3.) * CH_C_PI * pow(sphrad,3);
			double sphinertia = (2./5.) * sphmass * pow(sphrad,2);

			if (do_spheres)
				mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
												application.GetSystem(), application.GetSceneManager(),
												sphmass,
												ChVector<>(0.5, sphrad+level, 0.7),
												sphrad);
			else
				mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														application.GetSystem(), application.GetSceneManager(),
														sphmass,
														ChVector<>(0.5, sphrad+level, 0.7),
														ChQuaternion<>(1,0,0,0), 
														ChVector<>(sphrad,sphrad,sphrad) );
	   
			mrigidBody->GetBody()->SetInertiaXX(ChVector<>(sphinertia,sphinertia,sphinertia));
			mrigidBody->GetBody()->SetFriction(0.5f); 
			mrigidBody->GetBody()->SetImpactC(0.0f); 
			mrigidBody->addShadowVolumeSceneNode();

			mrigidBody->setMaterialTexture(0,	sphereMap);

			mspheres.push_back(mrigidBody);

			level   +=sphrad*2;
			totmass +=sphmass;
		}

		GetLog() << "Expected contact force at bottom F=" << (totmass *application.GetSystem()->Get_G_acc().y)  << "\n";
	}

	if (do_wall)
		for (int ai = 0; ai < 1; ai++)  // N. of walls
		{ 
			for (int bi = 0; bi < 10; bi++)  // N. of vert. bricks
			{ 
				for (int ui = 0; ui < 15; ui++)  // N. of hor. bricks
				{ 
					mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														application.GetSystem(), application.GetSceneManager(),
														0.8,
														ChVector<>(-8+ui*4.0+2*(bi%2),  1.0+bi*2.0, -5+ ai*6),
														ChQuaternion<>(1,0,0,0), 
														ChVector<>(3.96,2,4) );
					mrigidBody->GetBody()->SetFriction(0.4f);
					mrigidBody->setMaterialTexture(0,	sphereMap);
				}
			}
		}

	if (do_heavyonside)
	{
		double sphrad = 0.2;
		double dens= 1000;
		double sphmass = dens * (4./3.) * CH_C_PI * pow(sphrad,3);
		double sphinertia = (2./5.) * sphmass * pow(sphrad,2);

		double hfactor = 100;
		mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easySphere(
												application.GetSystem(), application.GetSceneManager(),
												sphmass*hfactor,
												ChVector<>(0.5, sphrad+0.1, -1),
												sphrad);
		mrigidBody->GetBody()->SetInertiaXX(ChVector<>(sphinertia*hfactor,sphinertia*hfactor,sphinertia*hfactor));
		mrigidBody->addShadowVolumeSceneNode();
		mrigidBody->setMaterialTexture(0,	sphereMap);

		GetLog() << "Expected contact deformation at side sphere=" << 
			(sphmass*hfactor *application.GetSystem()->Get_G_acc().y)*STATIC_COMPLIANCE  << "\n";
	}


	// Create the floor using a fixed rigid body of 'box' type:

	mrigidBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											application.GetSystem(), application.GetSceneManager(),
											1.0,
											ChVector<>(0,-2,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(50,4,50) );
	mrigidBody->GetBody()->SetBodyFixed(true);
	mrigidBody->GetBody()->SetFriction(0.6f);

	video::ITexture* cubeMap = application.GetVideoDriver()->getTexture("../data/concrete.jpg");
	mrigidBody->setMaterialTexture(0,	cubeMap);


	// Create rotating stuff
	ChBodySceneNode* rotatingBody = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											application.GetSystem(), application.GetSceneManager(),
											1.0,
											ChVector<>(0,12,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(10,2.5,1) ); 
	rotatingBody->GetBody()->SetMass(100);
	rotatingBody->GetBody()->SetInertiaXX(ChVector<>(500,500,500));
	rotatingBody->GetBody()->SetFriction(0.4f);
	rotatingBody->addShadowVolumeSceneNode();

	// .. an engine between mixer and truss	
	ChSharedPtr<ChLinkEngine> my_motor(new ChLinkEngine);
	my_motor->Initialize(rotatingBody->GetBody(), mrigidBody->GetBody(), 
				ChCoordsys<>(ChVector<>(0,15,0),
							 Q_from_AngAxis(CH_C_PI_2, VECT_X)) );
	my_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(my_motor->Get_spe_funct()))
		mfun->Set_yconst(CH_C_PI/2.0); // speed w=90°/s
	application.GetSystem()->AddLink(my_motor);

} 
     
  

// Function that forces all spheres in the 'parent' level to be on the same vertical
// axis, without needing any constraint (for simplifying the solver benchmark).

void align_spheres(ChIrrAppInterface& application)
{
	for (int i = 0; i< mspheres.size(); ++i)
	{
		ChBodySceneNode* body = mspheres[i];
		ChVector<> mpos = body->GetBody()->GetPos();
		mpos.x = 0.5;
		mpos.z = 0.7;
		body->GetBody()->SetPos(mpos);
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
	ChIrrAppInterface application(&mphysicalSystem, L"Critical cases for solver convergence",core::dimension2d<u32>(800,600),false,true);


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,1.5,-3));

 
	// Create all the rigid bodies.

	create_items(application);
 

	class MyContactCallback : public ChSystem::ChCustomCollisionPointCallback
	{
		public:	virtual void ContactCallback(
								const collision::ChCollisionInfo& mcontactinfo, ///< get info about contact (cannot change it)				
								ChMaterialCouple&  material )			  		///< you can modify this!	
		{
			// Set compliance (normal and tangential at once)
			material.compliance  = STATIC_COMPLIANCE; 
			material.complianceT = material.compliance ;
			material.dampingf = 0.2;
		};
		ChSystem* msystem;
	};

	MyContactCallback mycontact_callback;  // create the callback object
	mycontact_callback.msystem = &mphysicalSystem; // will be used by callback
	// Tell the system to use the callback above, per each created contact!
	mphysicalSystem.SetCustomCollisionPointCallback(&mycontact_callback);
 
	
	// Modify some setting of the physical system for the simulation, if you want

	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_BARZILAIBORWEIN);
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	mphysicalSystem.SetIterLCPmaxItersSpeed(60);
	mphysicalSystem.SetIterLCPmaxItersStab(5);
	mphysicalSystem.SetParallelThreadNumber(1);

	//mphysicalSystem.SetUseSleeping(true);

	application.SetStepManage(true);
	application.SetTimestep(0.01);
	application.SetPaused(true);

	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();
		
		align_spheres(application); // just to simplify test, on y axis only

		application.DoStep();
		
		application.GetVideoDriver()->endScene();  
	}
	
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
