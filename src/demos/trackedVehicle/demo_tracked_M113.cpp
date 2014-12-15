//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
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
//     - modeling tracks with articulated shoes (as an example
//       of complex model with collisions and constraints)
//     - using different meshes for collision and visualization
//     - using clones of collision shapes
//     - using SetFamilyMaskNoCollisionWithFamily, SetFamily etc.
//       to avoid collisions between different families of bodies.
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
#include "geometry/ChCTriangleMeshSoup.h"

#include <irrlicht.h>



// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::geometry;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;



// First of all, define a class for the 'tank' (that is, a set of
// bodies and links which are grouped within this class; so it is 
// easier to manage data structures in this example).

class MySimpleTank {
public:
		// THE DATA

	double throttleL; // actual value 0...1 of gas throttle (left).
	double throttleR; // actual value 0...1 of gas throttle (right).
	double max_motor_speed;	 // the max rotation speed of the motor [rads/s]

		// The parts making the tank, as 3d Irrlicht scene nodes, each containing
		// the ChBody object
			// .. truss:
	ChBodySceneNode* truss;
			// .. right front suspension:
	ChBodySceneNode* wheelRF;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteRF;
			// .. left front suspension:
	ChBodySceneNode* wheelLF;
	ChSharedPtr<ChLinkLockRevolute> link_revoluteLF;
			// .. right back suspension:
	ChBodySceneNode* wheelRB;
	ChSharedPtr<ChLinkEngine> link_revoluteRB;
			// .. left back suspension:
	ChBodySceneNode* wheelLB;
	ChSharedPtr<ChLinkEngine> link_revoluteLB;

		
		// THE FUNCTIONS

		// Build and initialize the tank, creating all bodies corresponding to
		// the various parts and adding them to the physical system - also creating
		// and adding constraints to the system.
	MySimpleTank(ChSystem&  my_system,	///< the chrono::engine physical system 
				ISceneManager* msceneManager, ///< the Irrlicht scene manager for 3d shapes
				IVideoDriver* mdriver	///< the Irrlicht video driver
				)
			{
				throttleL = throttleR = 0; // initially, gas throttle is 0.
				max_motor_speed = 10;

				double my = 0.5; // left back hub pos
				double mx = 0;

				double shoelength = 0.2;
				double shoethickness = 0.06;
				double shoewidth = 0.3;
				double shoemass = 2;
				double radiustrack = 0.31;
				double wheeldiameter = 0.280*2;	
				int nwrap = 6;
				int ntiles = 7;
				double rlwidth = 1.20;
				double passo = (ntiles+1)*shoelength;

				ChVector<> cyl_displA(0,  0.075+0.02,0);
				ChVector<> cyl_displB(0, -0.075-0.02,0);
				double cyl_hthickness = 0.045;

				// --- The tank body --- 

        IAnimatedMesh*	bulldozer_bodyMesh = msceneManager->getMesh(GetChronoDataFile("bulldozerB10.obj").c_str());
				truss = (ChBodySceneNode*)addChBodySceneNode(
														&my_system, msceneManager, bulldozer_bodyMesh,
														350.0,
														ChVector<>(mx + passo/2, my + radiustrack , rlwidth/2),
														QUNIT);
				truss->GetBody()->SetInertiaXX(ChVector<>(13.8, 13.5, 10));
				truss->GetBody()->SetBodyFixed(false);
			    //truss->addShadowVolumeSceneNode();

				// --- Right Front suspension --- 

				// Load a triangle mesh for wheel visualization
        IAnimatedMesh* irmesh_wheel_view = msceneManager->getMesh(GetChronoDataFile("wheel_view.obj").c_str());

				// ..the tank right-front wheel
				wheelRF = (ChBodySceneNode*) addChBodySceneNode(
													    &my_system, msceneManager, irmesh_wheel_view,
														9.0,
														ChVector<>(mx + passo, my+ radiustrack , 0),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X) );
				
				wheelRF->GetBody()->GetCollisionModel()->ClearModel();
				wheelRF->GetBody()->GetCollisionModel()->AddCylinder(wheeldiameter/2,wheeldiameter/2, cyl_hthickness, cyl_displA);
				wheelRF->GetBody()->GetCollisionModel()->AddCylinder(wheeldiameter/2,wheeldiameter/2, cyl_hthickness, cyl_displB);
				wheelRF->GetBody()->GetCollisionModel()->BuildModel();			// Creates the collision model
				wheelRF->GetBody()->SetCollide(true);
				wheelRF->GetBody()->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
				wheelRF->GetBody()->SetFriction(1.0);
        video::ITexture* cylinderMap = mdriver->getTexture(GetChronoDataFile("bluwhite.png").c_str());
				wheelRF->setMaterialTexture(0,	cylinderMap);
				wheelRF->addShadowVolumeSceneNode();

				// .. create the revolute joint between the wheel and the truss
				link_revoluteRF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // right, front, upper, 1
				link_revoluteRF->Initialize(wheelRF->GetBody(), truss->GetBody(), 
					ChCoordsys<>(ChVector<>(mx + passo, my+ radiustrack , 0) , QUNIT ) );
				my_system.AddLink(link_revoluteRF);


				// --- Left Front suspension --- 

				// ..the tank left-front wheel

				wheelLF = (ChBodySceneNode*) addChBodySceneNode(
													    &my_system, msceneManager, irmesh_wheel_view,
														9.0,
														ChVector<>(mx + passo, my+ radiustrack , rlwidth),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X) );

				wheelLF->GetBody()->GetCollisionModel()->ClearModel();
				wheelLF->GetBody()->GetCollisionModel()->AddCylinder(wheeldiameter/2,wheeldiameter/2, cyl_hthickness, cyl_displA);
				wheelLF->GetBody()->GetCollisionModel()->AddCylinder(wheeldiameter/2,wheeldiameter/2, cyl_hthickness, cyl_displB);
				wheelLF->GetBody()->GetCollisionModel()->BuildModel();			// Creates the collision model
				wheelLF->GetBody()->SetCollide(true);
				wheelLF->GetBody()->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
				wheelLF->GetBody()->SetFriction(1.0);
				wheelLF->setMaterialTexture(0,	cylinderMap);
				wheelLF->addShadowVolumeSceneNode();

				// .. create the revolute joint between the wheel and the truss
				link_revoluteLF = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute); // left, front, upper, 1
				link_revoluteLF->Initialize(wheelLF->GetBody(), truss->GetBody(), 
					ChCoordsys<>(ChVector<>(mx + passo, my+ radiustrack , rlwidth) , QUNIT ) );
				my_system.AddLink(link_revoluteLF);


				// --- Right Back suspension --- 

				// ..the tank right-back wheel

				wheelRB = (ChBodySceneNode*) addChBodySceneNode(
													    &my_system, msceneManager, irmesh_wheel_view,
														9.0,
														ChVector<>(mx , my+ radiustrack , 0),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X) );
				
				wheelRB->GetBody()->GetCollisionModel()->ClearModel();
				wheelRB->GetBody()->GetCollisionModel()->AddCylinder(wheeldiameter/2,wheeldiameter/2, cyl_hthickness, cyl_displA);
				wheelRB->GetBody()->GetCollisionModel()->AddCylinder(wheeldiameter/2,wheeldiameter/2, cyl_hthickness, cyl_displB);
				wheelRB->GetBody()->GetCollisionModel()->BuildModel();			// Creates the collision model
				wheelRB->GetBody()->SetCollide(true);
				wheelRB->GetBody()->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
				wheelRB->GetBody()->SetFriction(1.0);
        cylinderMap = mdriver->getTexture(GetChronoDataFile("bluwhite.png").c_str());
				wheelRB->setMaterialTexture(0,	cylinderMap);
				wheelRB->addShadowVolumeSceneNode();

				// .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
				link_revoluteRB = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); // right, back, upper, 1
				link_revoluteRB->Initialize(wheelRB->GetBody(), truss->GetBody(), 
					ChCoordsys<>(ChVector<>(mx , my+ radiustrack , 0) , QUNIT ) );
				link_revoluteRB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
				my_system.AddLink(link_revoluteRB);


				// --- Left Back suspension --- 

				// ..the tank left-back wheel

				wheelLB = (ChBodySceneNode*) addChBodySceneNode(
													    &my_system, msceneManager, irmesh_wheel_view,
														9.0,
														ChVector<>(mx , my+ radiustrack , rlwidth),
														chrono::Q_from_AngAxis(CH_C_PI/2, VECT_X) );
				
				wheelLB->GetBody()->GetCollisionModel()->ClearModel();
				wheelLB->GetBody()->GetCollisionModel()->AddCylinder(wheeldiameter/2,wheeldiameter/2, cyl_hthickness, cyl_displA);
				wheelLB->GetBody()->GetCollisionModel()->AddCylinder(wheeldiameter/2,wheeldiameter/2, cyl_hthickness, cyl_displB);
				wheelLB->GetBody()->GetCollisionModel()->BuildModel();			// Creates the collision model
				wheelLB->GetBody()->SetCollide(true);
				wheelLB->GetBody()->SetInertiaXX(ChVector<>(1.2, 1.2, 1.2));
				wheelLB->GetBody()->SetFriction(1.0);
				wheelLB->setMaterialTexture(0,	cylinderMap);
				wheelLB->addShadowVolumeSceneNode();

				// .. create the motor joint between the wheel and the truss (simplified motor model: just impose speed..)
				link_revoluteLB = ChSharedPtr<ChLinkEngine>(new ChLinkEngine); // left, back, upper, 1
				link_revoluteLB->Initialize(wheelLB->GetBody(), truss->GetBody(), 
					ChCoordsys<>(ChVector<>(mx , my+ radiustrack , rlwidth) ,  QUNIT ) );
				link_revoluteLB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
				my_system.AddLink(link_revoluteLB);


				//--- TRACKS ---
				
				// Load a triangle mesh for visualization
        IAnimatedMesh* irmesh_shoe_view = msceneManager->getMesh(GetChronoDataFile("shoe_view.obj").c_str());
				
				// Load a triangle mesh for collision
        IAnimatedMesh* irmesh_shoe_collision = msceneManager->getMesh(GetChronoDataFile("shoe_collision.obj").c_str());
				ChTriangleMeshSoup temp_trianglemesh; 
				fillChTrimeshFromIrlichtMesh(&temp_trianglemesh, irmesh_shoe_collision->getMesh(0));


				chrono::ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
				chrono::ChVector<> joint_displacement(-shoelength*0.5,0,0); // position of shoe-shoe constraint, relative to COG.

				chrono::ChVector<> position;
				chrono::ChQuaternion<> rotation;
				
				

				for (int side = 0; side <2; side ++)
				{
					mx = 0;
					mx += shoelength;

					double mz = 0;

					if (side == 0)
						mz = 0;
					else
						mz = rlwidth;

					position.Set(mx, my, mz);
					rotation = QUNIT;

					// Create sample body (with empty collision shape; later create the collision model by adding the coll.shapes)
					ChBodySceneNode* firstBodyShoe = (ChBodySceneNode*)addChBodySceneNode(
														&my_system, msceneManager, 0,   
														shoemass, 
														position, 	
														rotation);
					// Creates the collision model with a (simplified) mesh
					ChVector<> pin_displacement = mesh_displacement + ChVector<>(0,0.05,0);
					firstBodyShoe->GetBody()->GetCollisionModel()->SetSafeMargin(0.004);	// inward safe margin
					firstBodyShoe->GetBody()->GetCollisionModel()->SetEnvelope(0.010);		// distance of the outward "collision envelope"
					firstBodyShoe->GetBody()->GetCollisionModel()->ClearModel();
					 // ...try to use a concave (simplified) mesh plus automatic convex decomposition?? ...
					 firstBodyShoe->GetBody()->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);	// not static, not convex
					 // .. or rather use few 'primitive' shapes to approximate with cubes/etc?? 
					 //firstBodyShoe->GetBody()->GetCollisionModel()->AddBox(shoelength/2, shoethickness/2, shoewidth/2, mesh_displacement);
					 //firstBodyShoe->GetBody()->GetCollisionModel()->AddBox(0.04, 0.02, 0.02, pin_displacement);
					firstBodyShoe->GetBody()->GetCollisionModel()->BuildModel();			// Creates the collision model
					firstBodyShoe->GetBody()->SetCollide(true);

					/*  // alternative: use box as a collision geometry (but wheels will slip..)
					ChBodySceneNode* firstBodyShoe =  (ChBodySceneNode*)addChBodySceneNode_easyBox(
															&my_system, msceneManager,
															shoemass,
															position,
															rotation, 
															ChVector<>(shoelength, 0.02, 0.2) );
					*/

					// Add a mesh for visualization purposes
					msceneManager->addAnimatedMeshSceneNode(irmesh_shoe_view, firstBodyShoe,-1, vector3dfCH(mesh_displacement));

					// Avoid creation of contacts with neighbouring shoes, using
					// a collision family (=3) that does not collide with itself 
					firstBodyShoe->GetBody()->GetCollisionModel()->SetFamily(3);
					firstBodyShoe->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);
	 
					// Other settings
					firstBodyShoe->GetBody()->SetInertiaXX(ChVector<>(0.1, 0.1, 0.1));

					ChBodySceneNode* previous_rigidBodyShoe = firstBodyShoe;

					for (int nshoe = 1; nshoe < ntiles; nshoe++)
					{
						mx += shoelength;
						position.Set(mx, my, mz);

						ChBodySceneNode* rigidBodyShoe = MakeShoe(previous_rigidBodyShoe, 
							firstBodyShoe, 
							position, rotation, 
							irmesh_shoe_view, 
							my_system, msceneManager, mdriver,
							mesh_displacement, joint_displacement, shoemass);

						previous_rigidBodyShoe = rigidBodyShoe;
					}
					for (int nshoe = 0; nshoe < nwrap; nshoe++)
					{
						double alpha = (CH_C_PI/((double)(nwrap-1.0)))*((double)nshoe);
						
						double lx = mx + shoelength + radiustrack * sin(alpha);
						double ly = my + radiustrack - radiustrack * cos(alpha);
						position.Set(lx, ly, mz);
						rotation = chrono::Q_from_AngAxis(alpha,ChVector<>(0,0,1));
						ChBodySceneNode* rigidBodyShoe = MakeShoe(previous_rigidBodyShoe, 
							firstBodyShoe, 
							position, rotation, 
							irmesh_shoe_view, 
							my_system, msceneManager, mdriver,
							mesh_displacement, joint_displacement, shoemass);

						previous_rigidBodyShoe = rigidBodyShoe;
					}
					for (int nshoe = (ntiles-1); nshoe >= 0; nshoe--)
					{
						position.Set(mx, my+2*radiustrack, mz);

						ChBodySceneNode* rigidBodyShoe = MakeShoe(previous_rigidBodyShoe, 
							firstBodyShoe, 
							position, rotation, 
							irmesh_shoe_view, 
							my_system, msceneManager, mdriver,
							mesh_displacement, joint_displacement, shoemass);

						previous_rigidBodyShoe = rigidBodyShoe;

						mx -= shoelength;
					}
					for (int nshoe = 0; nshoe < nwrap; nshoe++)
					{
						double alpha = CH_C_PI + (CH_C_PI/((double)(nwrap-1.0)))*((double)nshoe);
						
						double lx = mx + 0 + radiustrack * sin(alpha);
						double ly = my + radiustrack - radiustrack * cos(alpha);
						position.Set(lx, ly, mz);
						rotation = chrono::Q_from_AngAxis(alpha,ChVector<>(0,0,1));
						ChBodySceneNode* rigidBodyShoe = MakeShoe(previous_rigidBodyShoe, 
							firstBodyShoe, 
							position, rotation, 
							irmesh_shoe_view, 
							my_system, msceneManager, mdriver,
							mesh_displacement, joint_displacement, shoemass);

						previous_rigidBodyShoe = rigidBodyShoe;
					}
					// close track
					ChVector<> linkpos = firstBodyShoe->GetBody()->Point_Body2World(joint_displacement);
					ChSharedPtr<ChLinkLockRevolute> link_revolute_shoeshoe(new ChLinkLockRevolute); 
					link_revolute_shoeshoe->Initialize(firstBodyShoe->GetBody(), previous_rigidBodyShoe->GetBody(), 
								ChCoordsys<>( linkpos , QUNIT) );
					my_system.AddLink(link_revolute_shoeshoe);
				

				}

		
			} 

		// Delete the tank object, deleting also all bodies corresponding to
		// the various parts and removing them from the physical system.  Also
		// removes constraints from the system.
	~MySimpleTank()
			{
				ChSystem* mysystem = truss->GetBody()->GetSystem(); // trick to get the system here				
					// When a ChBodySceneNode is removed via ->remove() from Irrlicht 3D scene manager,
					// it is also automatically removed from the ChSystem (the ChSystem::RemoveBody() is
					// automatically called at Irrlicht node deletion - see ChBodySceneNode.h ).

					// For links, just remove them from the ChSystem using ChSystem::RemoveLink()
				mysystem->RemoveLink(link_revoluteRF);	
				mysystem->RemoveLink(link_revoluteLF);				
				mysystem->RemoveLink(link_revoluteRB);				
				mysystem->RemoveLink(link_revoluteLB);				

				truss->remove();
				wheelRF->remove();
				wheelLF->remove();
				wheelRB->remove();
				wheelLB->remove();
			}

		// Utility function to create quickly a track shoe connected to the previous one
	ChBodySceneNode* MakeShoe(ChBodySceneNode* previous_shoe, ///< will be linked with this one with revolute joint
				ChBodySceneNode* template_collision_shape, ///< collision geometry will be shared with this body, to save memory&cpu time.
				ChVector<> position, ChQuaternion<> rotation, /// pos. and rotation
				IAnimatedMesh* irmesh_shoe_view, ///< detailed mesh of the shoe
		        ChSystem&  my_system,	///< the chrono::engine physical system 
				ISceneManager* msceneManager, ///< the Irrlicht scene manager for 3d shapes
				IVideoDriver* mdriver,	///< the Irrlicht video driver
				chrono::ChVector<> mesh_displacement,  // since mesh origin is not in body center of mass
				chrono::ChVector<> joint_displacement, // position of shoe-shoe constraint, relative to COG.
				double shoemass)
	{
			// Create 'track shoe' body with increasing position and rotation, along the track
			ChBodySceneNode* rigidBodyShoe = (ChBodySceneNode*)addChBodySceneNode_easyClone(
												&my_system, msceneManager, 
												template_collision_shape, 
												position,rotation); 
			rigidBodyShoe->addShadowVolumeSceneNode();
 
			// Add a mesh for visualization purposes
			msceneManager->addAnimatedMeshSceneNode(irmesh_shoe_view, rigidBodyShoe,-1, vector3dfCH(mesh_displacement));

			// Avoid creation of contacts with neighbouring shoes 

			rigidBodyShoe->GetBody()->GetCollisionModel()->SetFamily(3);			
			rigidBodyShoe->GetBody()->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(3);

			// Other settings are already copied from   template_collision_shape, except for family and mask.

			// Create revolute constraint
			if (previous_shoe)
			{
				ChVector<> linkpos = rigidBodyShoe->GetBody()->Point_Body2World(joint_displacement);
				ChSharedPtr<ChLinkLockRevolute> link_revolute_shoeshoe(new ChLinkLockRevolute); 
				link_revolute_shoeshoe->Initialize(rigidBodyShoe->GetBody(), previous_shoe->GetBody(), 
							ChCoordsys<>( linkpos , QUNIT) );
				my_system.AddLink(link_revolute_shoeshoe);
			}

			return rigidBodyShoe;
	}

};



// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChIrrAppInterface* myapp,
					MySimpleTank* atank)
			{
				// store pointer application
				application = myapp;
				// store pointer to other stuff
				mtank  = atank;

				// ..add a GUI slider to control throttle left via mouse
				scrollbar_throttleL = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 20, 650, 35), 0, 101);
				scrollbar_throttleL->setMax(100); 
				scrollbar_throttleL->setPos(50);
				text_throttleL = application->GetIGUIEnvironment()->addStaticText(
							L"Left throttle ", rect<s32>(650,20,750,35), false);

				// ..add a GUI slider to control gas throttle right via mouse
				scrollbar_throttleR = application->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 45, 650, 60), 0, 102);
				scrollbar_throttleR->setMax(100); 
				scrollbar_throttleR->setPos(50);
				text_throttleR = application->GetIGUIEnvironment()->addStaticText(
							L"Right throttle", rect<s32>(650,45,750,60), false);
			}

	bool OnEvent(const SEvent& event)
			{

				// check if user moved the sliders with mouse..
				if (event.EventType == EET_GUI_EVENT)
				{
					s32 id = event.GUIEvent.Caller->getID();
					IGUIEnvironment* env = application->GetIGUIEnvironment();

					switch(event.GUIEvent.EventType)
					{
					case EGET_SCROLL_BAR_CHANGED:
							if (id == 101) // id of 'throttleL' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newthrottle =  ((double)(pos)-50)/50.0 ;
								this->mtank->throttleL=newthrottle;
                                if (ChSharedPtr<ChFunction_Const> mfun = mtank->link_revoluteLB->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
									mfun->Set_yconst(newthrottle*6);
								return true;
							}
							if (id == 102) // id of 'throttleR' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newthrottle =  ((double)(pos)-50)/50.0 ;
								this->mtank->throttleR=newthrottle;
                                if (ChSharedPtr<ChFunction_Const> mfun = mtank->link_revoluteRB->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
									mfun->Set_yconst(newthrottle*6);
								return true;
							}
					break;
					}
					
				} 

				return false;
			}

private:
	ChIrrAppInterface* application;
	MySimpleTank*    mtank;

	IGUIStaticText* text_throttleL;
	IGUIScrollBar*  scrollbar_throttleL;
	IGUIStaticText* text_throttleR;
	IGUIScrollBar*  scrollbar_throttleR;
};



//
// This is the program which is executed
//

int main(int argc, char* argv[])
{
	// 1- Create a ChronoENGINE physical system: all bodies and constraints
	//    will be handled by this ChSystem object.
	ChSystem my_system;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&my_system, L"Modeling a simplified   tank",core::dimension2d<u32>(800,600),false, true); 


	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,0,-6), core::vector3df(-2,2,0));


 

	// 2- Create the rigid bodies of the simpified tank suspension mechanical system
	//   maybe setting position/mass/inertias of
	//   their center of mass (COG) etc.
	
	// ..the world
	ChBodySceneNode* my_ground = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, application.GetSceneManager(),
											1.0,
											ChVector<>(0,-1,0),
											QUNIT, 
											ChVector<>(60,2,60) );
	my_ground->GetBody()->SetBodyFixed(true);
	my_ground->GetBody()->SetCollide(true);
	my_ground->GetBody()->SetFriction(1.0);
  video::ITexture* groundMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("blu.png").c_str());
	my_ground->setMaterialTexture(0,groundMap);

	// ..some obstacles on the ground:
	for (int i=0; i<50; i++)
	{
		ChBodySceneNode* my_obstacle = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, application.GetSceneManager(),
											3.0,
											ChVector<>(-6+6*ChRandom(),2+1*ChRandom(), 6*ChRandom()),
											Q_from_AngAxis(ChRandom()*CH_C_PI, VECT_Y), 
											ChVector<>(0.6*(1-0.4*ChRandom()),
											           0.08,
													   0.3*(1-0.4*ChRandom()) ) );
		my_obstacle->addShadowVolumeSceneNode();
	}


	// ..the tank (this class - see above - is a 'set' of bodies and links, automatically added at creation)
	MySimpleTank* mytank = new MySimpleTank(my_system, application.GetSceneManager(), application.GetVideoDriver());



	//
	// USER INTERFACE
	//
	 

	// Create some graphical-user-interface (GUI) items to show on the screen.
	// This requires an event receiver object.
	MyEventReceiver receiver(&application, mytank);
	  // note how to add the custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);


	//
	// SETTINGS 
	// 	

	my_system.SetIterLCPmaxItersSpeed(100); // the higher, the easier to keep the constraints 'mounted'.
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR); 



	//
	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
	//


	application.SetStepManage(true);
	application.SetTimestep(0.03);
	application.SetTryRealtime(true);

	while(application.GetDevice()->run())
	{ 
		// Irrlicht must prepare frame to draw
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
	
		// .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
		application.DrawAll();

		// .. draw also a grid (rotated so that it's horizontal)
		ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 30,30, 
			ChCoordsys<>(ChVector<>(0,0.01,0), Q_from_AngX(CH_C_PI_2) ),
			video::SColor(255, 60,60,60), true);

		// HERE CHRONO INTEGRATION IS PERFORMED: 
		
		application.DoStep();


		application.GetVideoDriver()->endScene(); 
	}


	if (mytank) delete mytank;

	return 0;
}


