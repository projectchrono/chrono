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
//     - gear constraint (ChLinkGear) as a method
//       to impose a transmission ratio between two
//       shafts as they were connected by gears, without
//       the need of performing collision detection between
//       gear teeth geometries (which would be inefficient)
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
#include "unit_IRRLICHT/ChBodySceneNode.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h" 
#include "unit_IRRLICHT/ChIrrAppInterface.h"
#include "core/ChRealtimeStep.h"

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


   
     
 
int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&mphysicalSystem, L"Gears and pulleys",core::dimension2d<u32>(800,600),false); 


	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(12, 15,-20));

 

	// Create all the rigid bodies.

	double radA = 2; 
	double radB = 4; 

	// ...the truss
	ChBodySceneNode* mbody_truss = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(0,0, 3),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20,10,2) );
	mbody_truss->GetBody()->SetBodyFixed(true);
	mbody_truss->GetBody()->SetCollide(false);

  video::ITexture* cubeMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("cubetexture.png").c_str());
	mbody_truss->setMaterialTexture(0,	cubeMap);

	// ...the rotating bar support for the two epicycloidal wheels
	ChBodySceneNode* mbody_train = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(4,0, 0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(9,1.5,1.0) );
	mbody_train->GetBody()->SetCollide(false);


	// ...which must rotate respect to truss along Z axis, in 0,0,0,
	ChSharedPtr<ChLinkLockRevolute> link_revoluteTT(new ChLinkLockRevolute); 
	link_revoluteTT->Initialize(mbody_truss->GetBody(), mbody_train->GetBody(), 
					ChCoordsys<>(ChVector<>(0, 0, 0) , QUNIT ) );
	mphysicalSystem.AddLink(link_revoluteTT);


	// ...the first gear
	ChBodySceneNode* mbody_gearA = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(0,0,-1),
											chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X), 
											ChVector<>(radA*2 ,0.5, radA*2) );
	mbody_gearA->GetBody()->SetInertiaXX(ChVector<>(1.2,1.2,1.2));
	mbody_gearA->GetBody()->SetCollide(false);
	mbody_gearA->addShadowVolumeSceneNode();
  video::ITexture* cylinderMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("pinkwhite.png").c_str());
	mbody_gearA->setMaterialTexture(0,	cylinderMap);
			// for aesthetical reasons, also add a thin cylinder to show the shaft in Irrlicht: 
  IAnimatedMesh* axis_mesh = application.GetSceneManager()->getMesh(GetChronoDataFile("cylinder.obj").c_str());
	IAnimatedMeshSceneNode* axis_nodeA = application.GetSceneManager()->addAnimatedMeshSceneNode(axis_mesh, mbody_gearA);
	axis_nodeA->setScale( core::vector3df((irr::f32)0.4, 21, (irr::f32)0.4) );


	// ...impose rotation between the first gear and the fixed truss
	ChSharedPtr<ChLinkEngine> link_engine(new ChLinkEngine); 
		link_engine->Initialize(mbody_gearA->GetBody(), mbody_truss->GetBody(), 
					ChCoordsys<>(ChVector<>(0, 0, 0) , QUNIT ) );
		link_engine->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); // also works as revolute support
		link_engine->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
        if (ChSharedPtr<ChFunction_Const> mfun = link_engine->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
			mfun->Set_yconst(6); // rad/s  angular speed
	mphysicalSystem.AddLink(link_engine);

	// ...the second gear
	double interaxis12 = radA+radB;
	ChBodySceneNode* mbody_gearB = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(interaxis12,0,-1),
											chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X), 
											ChVector<>(radB*2, 0.4, radB*2) );
	mbody_gearB->GetBody()->SetInertiaXX(ChVector<>(1.2,1.2,1.2));
	mbody_gearB->GetBody()->SetCollide(false);
	mbody_gearB->addShadowVolumeSceneNode();
	mbody_gearB->setMaterialTexture(0,	cylinderMap);

	// ... the second gear is fixed to the rotating bar
	ChSharedPtr<ChLinkLockRevolute> link_revolute(new ChLinkLockRevolute); 
		link_revolute->Initialize(mbody_gearB->GetBody(), mbody_train->GetBody(), 
					ChCoordsys<>(ChVector<>(interaxis12, 0, 0) , QUNIT ) );
	mphysicalSystem.AddLink(link_revolute);


	// ...the gear constraint between the two wheels A and B. 
	//    As transmission ratio (=speed of wheel B / speed of wheel A) to enter in  Set_tau(), we
	//    could use whatever positive value we want: the ChLinkGear will compute the two radii of the
	//    wheels for its 'hidden' computations, given the distance between the two axes. However, since
	//    we already build two '3D cylinders' bodies -just for visualization reasons!- with radA and radB,
	//    we must enter Set_tau(radA/radB). 
	//    Also, note that the initial position of the constraint has no importance (simply use CSYSNORM),
	//    but we must set where the two axes are placed in the local coordinates of the two wheels, so
	//    we use Set_local_shaft1() and pass some local ChFrame. Note that, since the Z axis of that frame
	//    will be considered the axis of he wheel, we must rotate the frame 90° with Q_from_AngAxis(), because
	//    we created the wheel with addChBodySceneNode_easyCylinder() which created a cylinder with Y as axis.
	ChSharedPtr<ChLinkGear> link_gearAB(new ChLinkGear); 
		link_gearAB->Initialize(mbody_gearA->GetBody(), mbody_gearB->GetBody(), CSYSNORM );
		link_gearAB->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) );
		link_gearAB->Set_local_shaft2(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) );
		link_gearAB->Set_tau(radA/radB);
		link_gearAB->Set_checkphase(true);
	mphysicalSystem.AddLink(link_gearAB);


	// ...the gear constraint between the second wheel B and a large wheel C with inner teeth, that
	//    does not necessarily need to be created as a new body because it is the 'fixed' part of the 
	//    epicycloidal reducer, so, as wheel C, we will simply use the ground object 'mbody_truss'.
	double radC = 2*radB+radA;
	ChSharedPtr<ChLinkGear> link_gearBC(new ChLinkGear); 
		link_gearBC->Initialize(mbody_gearB->GetBody(), mbody_truss->GetBody(), CSYSNORM );
		link_gearBC->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) );
		link_gearBC->Set_local_shaft2(ChFrame<>(ChVector<>(0,0,-4), QUNIT) );
		link_gearBC->Set_tau(radB/radC);
		link_gearBC->Set_epicyclic(true); // <-- this means: use a wheel with internal teeth!
	mphysicalSystem.AddLink(link_gearBC);
 

	// ...the bevel gear at the side,
	double radD = 5;
	ChBodySceneNode* mbody_gearD = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(-10,0,-9),
											chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
											ChVector<>(radD*2, 0.8, radD*2) );
	mbody_gearD->GetBody()->SetCollide(false);
	mbody_gearD->setMaterialTexture(0,	cylinderMap);
			// (for aesthetical reasons, also add a thin cylinder to show the shaft in Irrlicht view:)
	IAnimatedMeshSceneNode* axis_nodeD = application.GetSceneManager()->addAnimatedMeshSceneNode(axis_mesh, mbody_gearD);
	axis_nodeD->setScale( core::vector3df((irr::f32)0.4, 18, (irr::f32)0.4) );

	// ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate 
	//     default ChLink creation coordys 90° on the Y vertical, since the revolute axis is the Z axis).
	ChSharedPtr<ChLinkLockRevolute> link_revoluteD(new ChLinkLockRevolute); 
		link_revoluteD->Initialize(mbody_gearD->GetBody(), mbody_truss->GetBody(), 
					ChCoordsys<>(ChVector<>(-10, 0, -9) , Q_from_AngAxis(CH_C_PI/2, VECT_Y) ) );
	mphysicalSystem.AddLink(link_revoluteD);

	// ... Let's make a 1:1 gear between wheel A and wheel D as a bevel gear: chrono::engine does not require 
	//     special info for this case -the position of the two shafts and the transmission ratio are enough-
	ChSharedPtr<ChLinkGear> link_gearAD(new ChLinkGear); 
		link_gearAD->Initialize(mbody_gearA->GetBody(), mbody_gearD->GetBody(), CSYSNORM );
		link_gearAD->Set_local_shaft1(ChFrame<>(ChVector<>(0,-7,0), chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) );
		link_gearAD->Set_local_shaft2(ChFrame<>(ChVector<>(0,-7,0), chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) );
		link_gearAD->Set_tau(1);
	mphysicalSystem.AddLink(link_gearAD);
	


		// ...the pulley at the side,
	double radE= 2;
	ChBodySceneNode* mbody_pulleyE = (ChBodySceneNode*)addChBodySceneNode_easyCylinder(
											&mphysicalSystem, application.GetSceneManager(),
											1.0,
											ChVector<>(-10,-11,-9),
											chrono::Q_from_AngAxis(CH_C_PI/2, VECT_Z), 
											ChVector<>(radE*2, 0.8, radE*2) );
	mbody_pulleyE->GetBody()->SetCollide(false);
	mbody_pulleyE->setMaterialTexture(0,	cylinderMap);

	// ... it is fixed to the truss using a revolute joint with horizontal axis (must rotate 
	//     default ChLink creation coordys 90° on the Y vertical, since the revolute axis is the Z axis).
	ChSharedPtr<ChLinkLockRevolute> link_revoluteE(new ChLinkLockRevolute); 
		link_revoluteE->Initialize(mbody_pulleyE->GetBody(), mbody_truss->GetBody(), 
					ChCoordsys<>(ChVector<>(-10, -11, -9) , Q_from_AngAxis(CH_C_PI/2, VECT_Y) ) );
	mphysicalSystem.AddLink(link_revoluteE);

	// ... Let's make a synchro belt constraint between pulley D and pulley E. The user must be
	//     sure that the two shafts are parallel in absolute space. Also, interaxial distance should not change.
	ChSharedPtr<ChLinkPulley> link_pulleyDE(new ChLinkPulley); 
		link_pulleyDE->Initialize(mbody_gearD->GetBody(), mbody_pulleyE->GetBody(), CSYSNORM );
		link_pulleyDE->Set_local_shaft1(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) );
		link_pulleyDE->Set_local_shaft2(ChFrame<>(VNULL, chrono::Q_from_AngAxis(-CH_C_PI/2, VECT_X)) );
		link_pulleyDE->Set_r1(radD);
		link_pulleyDE->Set_r2(radE);
		link_pulleyDE->Set_checkphase(true); // <- synchro belts don't tolerate slipping: this avoids slipping, even due to numerical errors.
	mphysicalSystem.AddLink(link_pulleyDE);




	// Prepare the physical system for the simulation

	mphysicalSystem.SetIntegrationType(ChSystem::INT_TASORA); 


	// 
	// THE SOFT-REAL-TIME CYCLE
	//
	application.SetStepManage(true);
	application.SetTimestep(0.01);
	application.SetTryRealtime(true);

	while(application.GetDevice()->run()) 
	{
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));

		application.DrawAll();

		// .. draw also some circle lines representing gears - just for aesthetical reasons

		ChIrrTools::drawCircle(application.GetVideoDriver(), link_gearBC->Get_r2(), 
			(link_gearBC->Get_local_shaft2() >> *link_gearBC->GetBody2()).GetCoord(), 
			video::SColor(255, 255,0,0), 50, true);
		ChIrrTools::drawCircle(application.GetVideoDriver(), link_gearAD->Get_r1(), 
			(link_gearAD->Get_local_shaft1() >> *link_gearAD->GetBody1()).GetCoord(), 
			video::SColor(255, 255,0,0), 30, true);
		ChIrrTools::drawCircle(application.GetVideoDriver(), link_gearAD->Get_r2(), 
			(link_gearAD->Get_local_shaft2() >> *link_gearAD->GetBody2()).GetCoord(), 
			video::SColor(255, 255,0,0), 30, true);

		ChIrrTools::drawCircle(application.GetVideoDriver(), 0.1, ChCoordsys<>( link_gearAB->GetMarker2()->GetAbsCoord().pos, QUNIT));
		ChIrrTools::drawCircle(application.GetVideoDriver(), 0.1, ChCoordsys<>( link_gearAD->GetMarker2()->GetAbsCoord().pos, QUNIT));
		ChIrrTools::drawCircle(application.GetVideoDriver(), 0.1, ChCoordsys<>( link_gearBC->GetMarker2()->GetAbsCoord().pos, QUNIT));

		// ..draw also some segments for a simplified representation of pulley
		ChIrrTools::drawSegment(application.GetVideoDriver(), 
			link_pulleyDE->Get_belt_up1(), 
			link_pulleyDE->Get_belt_up2(),
			video::SColor(255,   0,255,0), true);
		ChIrrTools::drawSegment(application.GetVideoDriver(), 
			link_pulleyDE->Get_belt_low1(), 
			link_pulleyDE->Get_belt_low2(),
			video::SColor(255,   0,255,0), true);

		// ADVANCE THE SIMULATION FOR ONE STEP

		application.DoStep();

		application.GetVideoDriver()->endScene();  
	}
	


	return 0;
}
  
