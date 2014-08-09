//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Show how to use the OpenCASCADE features 
//   implemented in the unit_CASCADE:
//
//   - load a 3D model saved in STEP format from a CAD
//   - select some sub assemblies from the STEP model
//   - make Chrono::Engine objects out of those parts
//
//	 CHRONO    
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
  
 
#include "core/ChRealtimeStep.h"
#include "unit_CASCADE/ChCascadeDoc.h"
#include "unit_CASCADE/ChCascadeMeshTools.h"
#include "unit_CASCADE/ChIrrCascadeMeshTools.h"
#include "unit_CASCADE/ChIrrCascade.h"
#include "unit_IRRLICHT/ChBodySceneNode.h" 
#include "unit_IRRLICHT/ChIrrAppInterface.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h" 

#include <irrlicht.h>


// Use the namespace of Chrono
using namespace chrono;

// Use the main namespaces of Irlicht
using namespace irr; 
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

// Use the namespace with OpenCascade stuff
using namespace cascade; 


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
	ChIrrAppInterface application(&my_system, L"Load a STEP model from file",core::dimension2d<u32>(800,600),false, true, video::EDT_OPENGL); 

	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	//ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(30,100,30), core::vector3df(30,-80,-30),200,130);
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0.2,0.3,-0.5));


	//
	// Load a STEP file, containing a mechanism. The demo STEP file has been 
	// created using a 3D CAD (in this case, SolidEdge v.18).
	//

		// Create the ChCascadeDoc, a container that loads the STEP model 
		// and manages its subassembles
	ChCascadeDoc mydoc;

	ChBodySceneNodeAuxRef* mrigidBody1 = 0;
	ChBodySceneNodeAuxRef* mrigidBody2 = 0;


		// load the STEP model using this command:
	bool load_ok = mydoc.Load_STEP("..\\data\\cascade\\assembly.stp");

		// print the contained shapes
	mydoc.Dump(GetLog());

	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.001);

	// In most CADs the Y axis is horizontal, but we want it vertical.
	// So define a root transformation for rotating all the imported objects.
	ChQuaternion<> rotation1;
	rotation1.Q_from_AngAxis(-CH_C_PI/2, ChVector<>(1,0,0)); // 1: rotate 90° on X axis 
	ChQuaternion<> rotation2;
	rotation2.Q_from_AngAxis(CH_C_PI, ChVector<>(0,1,0));	 // 2: rotate 180° on vertical Y axis
	ChQuaternion<> tot_rotation = rotation2 % rotation1;  // rotate on 1 then on 2, using quaternion product
	ChFrameMoving<> root_frame( ChVector<>(0,0,0), tot_rotation); 
	
	if (load_ok)
	{
			// Retrieve some sub shapes from the loaded model, using
			// the GetNamedShape() function, that can use path/subpath/subsubpath/part 
			// syntax and * or ? wldcards, etc.

		TopoDS_Shape mshape;
		if (mydoc.GetNamedShape(mshape, "Assem1/body1" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody1 = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									mshape);

			mrigidBody1->GetBody()->SetBodyFixed(true);

				// Move the body as for global displacement/rotation (also mrigidBody1->GetBody() %= root_frame; )
			mrigidBody1->GetBody()->ConcatenatePreTransformation(root_frame);

				// Also add a collision shape based on the triangulation of the OpenCascade CAD model
			/*
			TopoDS_Shape relshape = mshape;
			relshape.Location( TopLoc_Location() );
			ChTriangleMesh temp_trianglemesh; 
			ChCascadeMeshTools::fillTriangleMeshFromCascade(temp_trianglemesh, relshape);
			mrigidBody1->GetBody()->GetCollisionModel()->ClearModel();
			mrigidBody1->GetBody()->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false); 
			mrigidBody1->GetBody()->GetCollisionModel()->BuildModel();
			mrigidBody1->GetBody()->SetCollide(true);
			*/
				// Move the COG body apart
			//ChFrame<> mfr(ChVector<>(0,0,0.04));
			//mrigidBody1->GetBody()->SetFrame_COG_to_REF(mfr);

		}
		else GetLog() << "Warning. Desired object not found in document \n";


		TopoDS_Shape bshape;
		if (mydoc.GetNamedShape(bshape, "Assem1/body2" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization
			mrigidBody2 = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									bshape);

				// Move the body as for global displacement/rotation  (also mrigidBody1->GetBody() %= root_frame; )
			mrigidBody2->GetBody()->ConcatenatePreTransformation(root_frame);

				// Also add a collision shape based on the triangulation of the OpenCascade CAD model
			/*
			TopoDS_Shape relshape = bshape;
			relshape.Location( TopLoc_Location() );
			ChTriangleMesh temp_trianglemesh; 
			ChCascadeMeshTools::fillTriangleMeshFromCascade(temp_trianglemesh, relshape);
			//fillChTrimeshFromIrlichtMesh(&temp_trianglemesh, mrigidBody->GetChildMesh()->getMesh());
			mrigidBody2->GetBody()->GetCollisionModel()->ClearModel();
			mrigidBody2->GetBody()->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false); 
			mrigidBody2->GetBody()->GetCollisionModel()->BuildModel();
			mrigidBody2->GetBody()->SetCollide(true);
			*/

		}
		else GetLog() << "Warning. Desired object not found in document \n";


	}
	else GetLog() << "Warning. Desired STEP file could not be opened/parsed \n";

	
	// Create a revolute joint between the two parts
	// as in a pendulum. We assume we already know in advance
	// the aboslute position of the joint (ex. we used measuring tools in the 3D CAD)
	ChVector<> measured_joint_pos_mm (0, 48, 120); 
	double scale = 1./1000.; // because we use meters instead of mm
	ChVector<> joint_pos =  ((ChFrame<>)root_frame) * (measured_joint_pos_mm*scale); // trasform because we rotated everything

	if (mrigidBody1 && mrigidBody2)
	{
		ChSharedPtr<ChLinkLockRevolute>  my_link(new ChLinkLockRevolute);
		ChSharedBodyPtr mb1 = mrigidBody1->GetBody();
		ChSharedBodyPtr mb2 = mrigidBody2->GetBody();
		my_link->Initialize(mb1, mb2, ChCoordsys<>(joint_pos));
		my_system.AddLink(my_link);
	}



		// Create a large cube as a floor.

	ChBodySceneNode* mfloor = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, application.GetSceneManager(),
											1000.0,
											ChVector<>(0,-0.6,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20,1,20) );
	mfloor->GetBody()->SetBodyFixed(true);
	mfloor->GetBody()->SetCollide(true);
	video::ITexture* cubeMap = application.GetVideoDriver()->getTexture(GetChronoDataFile("blu.png").c_str());
	mfloor->setMaterialTexture(0,	cubeMap);




	//
	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
	//
	
	application.SetStepManage(true);
	application.SetTimestep(0.01);
	application.SetTryRealtime(true);

	while(application.GetDevice()->run())
	{ 
		// Irrlicht must prepare frame to draw
		application.GetVideoDriver()->beginScene(true, true, video::SColor(255,140,161,192));

		// .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
		application.DrawAll();

		application.DoStep();

		application.GetVideoDriver()->endScene(); 
	}


	return 0;
}


