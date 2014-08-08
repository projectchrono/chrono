#pragma once

#pragma unmanaged

#include <irrlicht.h>
#include "physics/ChSystem.h"
#include "physics/ChLinkSpring.h"
#include "unit_IRRLICHT/ChBodySceneNode.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h"
#include "unit_IRRLICHT/ChIrrWizard.h"
#include "unit_IRRLICHT/ChDisplayTools.h" 
#include "core/ChRealtimeStep.h"

using namespace chrono;
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;


/// Class for a simple 'one-degree of freedom oscillator' system
/// defined using Chrono::Engine body and spring.
/// 

class MyOscillatorSystem {
public:

	chrono::ChSystem physical_system;

	irr::scene::ChBodySceneNode* my_truss;
	irr::scene::ChBodySceneNode* my_body;
	chrono::ChSharedPtr<chrono::ChLinkSpring>    my_spring;

	double X0;	// initial condition position
	double V0;  // initial contition speed

		//
		// Create and initialize all data relative the the oscillatr system
		//
	MyOscillatorSystem(irr::IrrlichtDevice* mIrrDevice);

		// reset position and speed to initial state X0, V0
	void ResetToInitialState()
		{
			this->my_body->GetBody()->SetRot(QUNIT);
			this->my_body->GetBody()->SetPos( ChVector<>(X0, 0 , 0));
			this->my_body->GetBody()->SetRot_dt(QNULL);
			this->my_body->GetBody()->SetPos_dt( ChVector<>(V0, 0, 0));
			this->my_body->GetBody()->Update(); 
			this->my_spring->Update(0);
		}
};    


MyOscillatorSystem::MyOscillatorSystem(IrrlichtDevice* mIrrDevice)
	{	 
			X0  =  2;
			V0  =  0;

			this->physical_system.Set_G_acc(VNULL);

			this->my_truss = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&this->physical_system, mIrrDevice->getSceneManager(),
														1.0,
														ChVector<>(0, 0 ,0),
														QUNIT, 
														ChVector<>(0.1, 1, 1) );
				this->my_truss->GetBody()->SetBodyFixed(true);
				this->my_truss->GetBody()->SetCollide(false);
				
			this->my_body = (ChBodySceneNode*)addChBodySceneNode_easyBox(
														&this->physical_system, mIrrDevice->getSceneManager(),
														3.0,
														ChVector<>(X0, 0 ,0),
														QUNIT, 
														ChVector<>(1, 1, 1) );
				this->my_body->GetBody()->SetBodyFixed(false);
				this->my_truss->GetBody()->SetCollide(false);
				this->my_body->GetBody()->SetPos_dt( ChVector<>(V0, 0, 0) ); // set initial speed

			this->my_spring= ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
				this->my_spring->Initialize(my_truss->GetBody(), my_body->GetBody(), true, ChVector<>(0.0,0.0,0.0), ChVector<>(0.0,0.0,0.0));
				this->my_spring->Set_SpringK(300);
				this->my_spring->Set_SpringR(3);
				this->my_spring->Set_SpringRestLenght(X0*0.8);
				this->physical_system.AddLink(this->my_spring);

				//ChFunction_Jscript* motionfunct = new ChFunction_Jscript;
				//motionfunct->Set_Command("0");
				//this->my_spring->GetMarker1()->SetMotion_X(motionfunct);
	}

#pragma managed
