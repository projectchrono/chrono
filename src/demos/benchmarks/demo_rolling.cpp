///////////////////////////////////////////////////
//      
//   Demo code about   
//   
//     - rolling, rolling friction, sliding friction 
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
#include <stdio.h>

// Use the namespace of Chrono
using namespace std;
using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

// Define the callback class for reporting all contacts via callback proxy
class _reporter_class : public chrono::ChReportContactCallback 
{
public:
	virtual bool ReportContactCallback (
					const ChVector<>& pA,				
					const ChVector<>& pB,				
					const ChMatrix33<>& plane_coord,	
					const double& distance,				
					const float& mfriction,			  	
					const ChVector<>& react_forces,		
					const ChVector<>& react_torques,	
					collision::ChCollisionModel* modA,	
					collision::ChCollisionModel* modB) 
	{
		//if (react_forces.x>0.00001)
		if (modA->GetPhysicsItem()->GetIdentifier()==6||modB->GetPhysicsItem()->GetIdentifier()==6)
		{	
			ChMatrix33<> localmatr(plane_coord);
			ChVector<> n1 = localmatr.Get_A_Xaxis();

			ChVector<> absreac= localmatr * react_forces;	//comment out localmatr to get relative forces!
		
			(*mfile) << modA->GetPhysicsItem()->GetChTime() << ", ";
			//(*mfile) << pA.x << ", ";
			//(*mfile) << pA.y << ", ";
			//(*mfile) << pA.z << ", ";
			//(*mfile) << n1.x << ", ";
			//(*mfile) << n1.y << ", ";
			//(*mfile) << n1.z << ", ";
			//(*mfile) << absreac.x << ", ";
			//(*mfile) << absreac.y << ", ";
			(*mfile) << absreac.x << ", ";
			(*mfile) << absreac.y << ", ";
			(*mfile) << absreac.z;
			
			//(*mfile) << absreac.z << ", ";
			(*mfile) << "\n";
		}
		//else
		//{
			//(*mfile) << "0" << " \n";
		//}
		return true; // to continue scanning contacts
	}
	// Data 
	ChStreamOutAsciiFile* mfile;
};

 
int main(int argc, char* argv[])
{ 

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	ChGlobals* GLOBAL_Vars = DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem; 

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&mphysicalSystem, L"Rolling test",core::dimension2d<u32>(800,600),false); 
	
	/* change the default path from ../data to the absolute path to
         * the ChronoEngine installation so we can find the data files */
        set_irrlicht_default_obj_dir(CHRONOENGINE_DATA "/");
 
	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo  (application.GetDevice(), CHRONOENGINE_DATA "/");
	ChIrrWizard::add_typical_Sky   (application.GetDevice(), CHRONOENGINE_DATA "/skybox/");
	ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(30.f, 200.f,  90.f), core::vector3df(30.f, 80.f, -60.f), 590,  400);
	//ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-15,14,-30), core::vector3df(0,5,0)); 
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,5,-7), core::vector3df(0,0,0));
	
	// 
	// HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO... 
	// 

	// Create all the rigid bodies.

	video::ITexture* cubeMap = application.GetVideoDriver()->getTexture(CHRONOENGINE_DATA "/cubetexture.png");
	video::ITexture* sphereMap = application.GetVideoDriver()->getTexture(CHRONOENGINE_DATA "/bluwhite.png");
	
	// General parameters
	double G_acc = -9.81;
	int filenumber = 2;
	double initVelX=2;

	// Friction parameters
	double frictionCoefficient0 = 0;
	double frictionCoefficient1 = .2;
	double frictionCoefficient2 = 1;
	
	// Plane parameters
	double planeThickness = .1;
	double planeLength = 100;
	double planeWidth = 5;

	// Sphere parameters
	double mradius = 1;
	double density = 1;
	//double mmass = (4./3.)*CH_C_PI*pow(mradius,3)*density; 
	double mmass = 1; 
	double minert = (2./5.)* mmass * pow(mradius,2);
	double groundMass = planeThickness*planeLength*planeWidth*density;
	double groundInertThickness = (1/12)*groundMass*(planeWidth*planeWidth+planeLength*planeLength);
	double groundInertWidth = (1/12)*groundMass*(planeThickness*planeThickness+planeLength*planeLength);
	double groundInertLength = (1/12)*groundMass*(planeThickness*planeThickness+planeWidth*planeWidth);

	// CREATE GROUND 0

	ChBodySceneNode* ground0 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
		&mphysicalSystem, application.GetSceneManager(),
		groundMass,
		ChVector<>(0,0,planeWidth*0),
		Q_from_AngAxis(0, VECT_Z), 
		ChVector<>(planeLength,planeThickness,planeWidth) 
		);
 
		ground0->GetBody()->SetBodyFixed(true); // the ground does not move!
		ground0->GetBody()->SetFriction(frictionCoefficient1);
		ground0->GetBody()->SetInertiaXX(ChVector<>(groundInertLength,groundInertThickness,groundInertWidth));
		ground0->GetBody()->SetRollingFriction(frictionCoefficient0);

	// CREATE SPHERE 0

	ChBodySceneNode* ball0 = (ChBodySceneNode*)addChBodySceneNode_easySphere(
		&mphysicalSystem, application.GetSceneManager(),
		mmass, // mass
		ChVector<>(0, mradius+planeThickness/2, planeWidth*0), // pos
		mradius, // radius
		20,  // hslices, for rendering
		15); // vslices, for rendering

		ball0->GetBody()->SetInertiaXX(ChVector<>(minert,minert,minert));
		ball0->GetBody()->SetPos_dt(ChVector<>(initVelX,0,0));
		ball0->GetBody()->SetFriction(frictionCoefficient1);
		ball0->GetBody()->SetRollingFriction(frictionCoefficient0);
		ball0->setMaterialTexture(0,	sphereMap);

	mphysicalSystem.Set_G_acc(ChVector<>(0,G_acc,0));	//set gravity!
  
	// Prepare the physical system for the simulation 
 
	//mphysicalSystem.SetIntegrationType(ChSystem::INT_ANITESCU);
	mphysicalSystem.SetIntegrationType(ChSystem::INT_TASORA); 

	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_SIMPLEX); //1
	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR); //2
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD); //3
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SYMMSOR); //4
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_JACOBI); //5
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_GPU);
	mphysicalSystem.SetUseSleeping(false);

	mphysicalSystem.SetMaxPenetrationRecoverySpeed(1.6); // used by Anitescu stepper only, default = 1.6
	mphysicalSystem.SetIterLCPmaxItersSpeed(40); //default is 40
	mphysicalSystem.SetIterLCPmaxItersStab(20); // unuseful for Anitescu, only Tasora uses this
	mphysicalSystem.SetIterLCPwarmStarting(false);
	mphysicalSystem.SetIterLCPomega(0.7);

	//
	// THE SOFT-REAL-TIME CYCLE
	//

	bool save_contact_data = true;
	int frame_number = 0; //for recording contacts at each point in time
	
	char filename_contact[100];
	char data_contact[100];
	sprintf(filename_contact, "contact%d.m", filenumber);
	ChStreamOutAsciiFile data_contacts(filename_contact);
	sprintf(data_contact, "contact%d", filenumber);
	data_contacts << data_contact << "_data = [\n";
	
	char filename_ball0[100];
	char data_ball0[100];
	sprintf(filename_ball0, "ball0%d.m", filenumber);
	ChStreamOutAsciiFile data_ballsim0(filename_ball0);
	sprintf(data_ball0, "ball0%d", filenumber);
	data_ballsim0 << data_ball0 << "_data = [\n";

	while(application.GetDevice()->run() && mphysicalSystem.GetChTime()<=3)
	{
		//mphysicalSystem.Set_G_acc(ChVector<>(0,G_acc_melanz,0));

		//OUTPUT POSITION FOR BALL 0
		data_ballsim0 << mphysicalSystem.GetChTime() << ", " << ball0->GetBody()->GetPos().x;
		data_ballsim0 << ", " << ball0->GetBody()->GetPos().y;
		data_ballsim0 << ", " << ball0->GetBody()->GetPos().z << ", ";

		//OUTPUT ANGULAR POSITION FOR BALL 0
		data_ballsim0 << ball0->GetBody()->GetA()->Get_A_Rxyz().x;
		data_ballsim0 << ", " << ball0->GetBody()->GetA()->Get_A_Rxyz().y;
		data_ballsim0 << ", " << ball0->GetBody()->GetA()->Get_A_Rxyz().z << ", ";

		//OUTPUT VELOCITY FOR BALL 0
		data_ballsim0 << ball0->GetBody()->GetPos_dt().x;
		data_ballsim0 << ", " << ball0->GetBody()->GetPos_dt().y;
		data_ballsim0 << ", " << ball0->GetBody()->GetPos_dt().z << ", ";

		//OUTPUT ACCELERATION FOR BALL 0
		data_ballsim0 << ball0->GetBody()->GetPos_dtdt().x;
		data_ballsim0 << ", " << ball0->GetBody()->GetPos_dtdt().y;
		data_ballsim0 << ", " << ball0->GetBody()->GetPos_dtdt().z << "\n";
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();

		mphysicalSystem.DoStepDynamics( 0.001);
 
		application.GetVideoDriver()->endScene();
		
		// save contacts to file
		if (save_contact_data)
		{
			
			//data_contacts << mphysicalSystem.GetChTime() << ", ";
			//char padnumber[100];
			//char filename[100];
			//sprintf(padnumber, "%d", (frame_number+10000));
			//sprintf(filename, "./data/contacts%s.dat", padnumber+1);
			//ChStreamOutAsciiFile data_contacts(filename);

			//GetLog() << " ++Saving contact data: " << filename <<"\n";
			//GetLog() << " Number of contacts: " << mphysicalSystem.GetNcontacts() <<"\n";

			_reporter_class my_contact_reporter;
			my_contact_reporter.mfile = &data_contacts;

			// scan all contacts 
			mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_contact_reporter);
			
			frame_number++;
		} 
	}

	data_ballsim0 << "];";
	data_contacts << "];"; 

	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();
	
	return 0;
}
