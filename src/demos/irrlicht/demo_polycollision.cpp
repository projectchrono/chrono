//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
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
//     - collisions and contacts 
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
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
    
  
 
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChBodySceneNode.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h" 
#include "unit_IRRLICHT/ChIrrAppInterface.h"
#include "core/ChRealtimeStep.h"
#include <stdio.h>

#include "unit_IRRLICHT/ChDisplayTools.h" 
#include "unit_IRRLICHT/ChIrrWizard.h" 
 
#include <irrlicht.h>

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
		//{	
			ChMatrix33<> localmatr(plane_coord);
			ChVector<> n1 = localmatr.Get_A_Xaxis();

			ChVector<> absreac= localmatr * react_forces;
			
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
		//}
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
	DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem; 

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&mphysicalSystem, L"Conservation of Momentum Test",core::dimension2d<u32>(800,600),false); 

 
	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo  (application.GetDevice());
	ChIrrWizard::add_typical_Sky   (application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(30.f, 200.f,  90.f), core::vector3df(30.f, 80.f, -60.f), 590,  400);
	//ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-15,14,-30), core::vector3df(0,5,0)); 
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,2,-10), core::vector3df(0,0,0));



	// 
	// HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO... 
	// 

	// Create all the rigid bodies.

	video::ITexture* cubeMap = application.GetVideoDriver()->getTexture("../data/cubetexture.png");

	double incline_degrees = 0;
	double frictionCoefficient = 0;
	//double G_acc = -9.81;
	double G_acc = 0;

	double incline = incline_degrees*CH_C_PI/180;
	double boxLength = 2;
	double boxHeight = 1;
	double boxWidth = 1;
	double planeThickness = .1;
	double planeLength = 5;
	double planeWidth = 5;

	double boxPosInitX=3;
	double L3 = (boxHeight+planeThickness)/2;
	double L2 = L3/cos(incline);
	double L1 = tan(incline)*boxPosInitX;
	double boxPosInitY=L1+L2;
	bool save_contact_data = true;
	
	double mradius = 1;
	double density = 1;
	double mmass = (4./3.)*CH_C_PI*pow(mradius,3)*density; 
	double minert = (2./5.)* mmass * pow(mradius,2);
	double groundMass = planeThickness*planeLength*planeWidth*density;
	double groundInertThickness = (1/12)*groundMass*(planeWidth*planeWidth+planeLength*planeLength);
	double groundInertWidth = (1/12)*groundMass*(planeThickness*planeThickness+planeLength*planeLength);
	double groundInertLength = (1/12)*groundMass*(planeThickness*planeThickness+planeWidth*planeWidth);

	double spin_degrees = -90;
	double spin = spin_degrees*CH_C_PI/180;
	
	// CREATE GROUND
	/*
	ChBodySceneNode* ground = (ChBodySceneNode*)addChBodySceneNode_easyBox(
		&mphysicalSystem, application.GetSceneManager(),
		groundMass,
		ChVector<>(0,0,0),
		Q_from_AngAxis(0, VECT_Z), 
		ChVector<>(planeLength,planeThickness,planeWidth) 
		);
 
		ground->GetBody()->SetBodyFixed(true); // the truss does not move!
		ground->GetBody()->SetFriction(frictionCoefficient);
		ground->GetBody()->SetInertiaXX(ChVector<>(groundInertLength,groundInertThickness,groundInertWidth));
		ground->setMaterialTexture(0,	cubeMap);
	*/
	
	ChBodySceneNode* poly = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(&mphysicalSystem, application.GetSceneManager(),
		mmass, ChVector<>(0,0,0),
		Q_from_AngAxis(0, VECT_X), 
		"../data/shuttle.obj" , 
		false,	// not static 
		true);	// true=convex; false=concave(do convex decomposition of concave mesh

		poly->GetBody()->SetBodyFixed(true);
		//poly->GetBody()->SetMass(mmass);
		poly->GetBody()->SetCollide(true);
		poly->GetBody()->SetRot(ChQuaternion<>(.7071,-.7071,0,0));
	
	/*
	ChBodySceneNode* poly2 = (ChBodySceneNode*)addChBodySceneNode_easyGenericMesh(&mphysicalSystem, application.GetSceneManager(),
		mmass, ChVector<>(5,0,0),
		Q_from_AngAxis(spin, VECT_Y), 
		"../data/octahedron.obj" , 
		false,	// not static 
		true);	// true=convex; false=concave(do convex decomposition of concave mesh

		//poly2->GetBody()->SetBodyFixed(true);
		//poly2->GetBody()->SetMass(mmass);
		poly2->GetBody()->SetCollide(true);
		poly2->GetBody()->SetPos_dt(ChVector<>(-2,0,0));
	*/
	/*
	ChBodySceneNode* box2 = (ChBodySceneNode*)addChBodySceneNode_easyBox(
		&mphysicalSystem, application.GetSceneManager(),
		mmass,
		ChVector<>(0,0,0),
		ChQuaternion<>(0.9239,0,0.3827,0)%ChQuaternion<>(0.9239,0,0,0.3827), 
		ChVector<>(mradius,mradius,mradius) 
		);

	ChBodySceneNode* box = (ChBodySceneNode*)addChBodySceneNode_easyBox(
		&mphysicalSystem, application.GetSceneManager(),
		mmass,
		ChVector<>(5,0,0),
		Q_from_NasaAngles(ChVector<>(0,0,0)), 
		ChVector<>(mradius,mradius,mradius) 
		);
		box->GetBody()->SetPos_dt(ChVector<>(-2,0,0));
		box->GetBody()->SetBodyFixed(true);
	*/
	/*
	ChBodySceneNode* ball2 = (ChBodySceneNode*)addChBodySceneNode_easySphere(
		&mphysicalSystem, application.GetSceneManager(),
		mmass, // mass
		ChVector<>(0, 0, 0), // pos
		mradius, // radius
		20,  // hslices, for rendering
		15); // vslices, for rendering
	*/
	// CREATE SPHERE
	/*
	video::ITexture* sphereMap = application.GetVideoDriver()->getTexture("../data/bluwhite.png");
	
	ChBodySceneNode* ball = (ChBodySceneNode*)addChBodySceneNode_easySphere(
		&mphysicalSystem, application.GetSceneManager(),
		mmass, // mass
		ChVector<>(5, 0, 0), // pos
		mradius, // radius
		20,  // hslices, for rendering
		15); // vslices, for rendering
	
	// set moment of inertia (more realisic than default 1,1,1).
	//ball->GetBody()->SetInertiaXX(ChVector<>(minert,minert,minert));
	ball->GetBody()->SetPos_dt(ChVector<>(-2,0,0));
	//ball->GetBody()->SetFriction(frictionCoefficient);
	//ball->GetBody()->SetRollingFriction(frictionCoefficient/100);
	//ball->GetBody()->SetImpactC(0);
	ball->setMaterialTexture(0,	sphereMap);
	*/
	mphysicalSystem.Set_G_acc(ChVector<>(0,G_acc,0));	//set gravity!


	// Prepare the physical system for the simulation 
 
	//mphysicalSystem.SetIntegrationType(ChSystem::INT_ANITESCU);
	mphysicalSystem.SetIntegrationType(ChSystem::INT_TASORA); 

	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_GPU);
	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);
	//mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
	mphysicalSystem.SetUseSleeping(false);

	mphysicalSystem.SetMaxPenetrationRecoverySpeed(1.6); // used by Anitescu stepper only, default = 1.6
	mphysicalSystem.SetIterLCPmaxItersSpeed(40); //default is 40
	mphysicalSystem.SetIterLCPmaxItersStab(20); // unuseful for Anitescu, only Tasora uses this
	mphysicalSystem.SetIterLCPwarmStarting(true);
	mphysicalSystem.SetIterLCPomega(0.7);
	//mphysicalSystem.SetIntegrtol(1e-300);

	//
	// THE SOFT-REAL-TIME CYCLE
	//
	//double time = 0;
	/*
	int filenumber = 1;
	char filename_vel[100];
	char filename_contact[100];
	char data_vel[100];
	char data_contact[100];
	sprintf(filename_vel, "velocity%d.m", filenumber);
	sprintf(filename_contact, "contact%d.m", filenumber);
	ChStreamOutAsciiFile data_contacts(filename_contact);
	ChStreamOutAsciiFile data_velocity(filename_vel);
	sprintf(data_vel, "vel%d", filenumber);
	sprintf(data_contact, "contact%d", filenumber);
	int frame_number = 0; //for recording contacts at each point in time
	data_velocity << data_vel << "_data = [\n";
	data_contacts << data_contact << "_data = [\n";
	*/
	//mphysicalSystem.SetEndTime(10);

	while(application.GetDevice()->run() /*&& mphysicalSystem.GetChTime()<=10*/)
	{
		/*
		//GetLog() << " Number of contacts: " << mphysicalSystem.GetNcontacts() <<"\n";
		data_velocity << mphysicalSystem.GetChTime() << ", " << ball->GetBody()->GetPos_dt().x;
		data_velocity << ", " << ball->GetBody()->GetPos_dt().y;
		data_velocity << ", " << ball->GetBody()->GetPos_dt().z;
		data_velocity << ", " << ball2->GetBody()->GetPos_dt().x;
		data_velocity << ", " << ball2->GetBody()->GetPos_dt().y;
		data_velocity << ", " << ball2->GetBody()->GetPos_dt().z << "\n";
		*/
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();
		
		//Draw grid
		ChIrrTools::drawGrid(application.GetVideoDriver(), 1, 1, 50,50, 
			ChCoordsys<>(ChVector<>(0,0,0), Q_from_AngX(CH_C_PI_2) ),
			video::SColor(255, 80,100,100), true);
		
		ChIrrTools::drawGrid(application.GetVideoDriver(), 1, 1, 50,50, 
			ChCoordsys<>(ChVector<>(0,0,0), Q_from_AngX(0) ),
			video::SColor(255, 80,100,100), true);
		
		mphysicalSystem.DoStepDynamics( 0.001);
 
		application.GetVideoDriver()->endScene();
		/*
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
		//time = time + mphysicalSystem.GetTimerStep(); 
		*/
	}
	//data_velocity << "];";
	//data_contacts << "];"; 
	//GetLog() << "Computational time: " << time <<"\n";
	return 0;
}
  
