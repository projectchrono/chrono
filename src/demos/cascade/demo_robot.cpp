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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
  
 
#include "physics/ChApidll.h" 
#include "core/ChRealtimeStep.h"
#include "irrlicht_interface/ChIrrAppInterface.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "unit_CASCADE/ChCascadeDoc.h"
#include "unit_CASCADE/ChCascadeMeshTools.h"
#include "unit_CASCADE/ChIrrCascadeMeshTools.h"
#include "unit_CASCADE/ChIrrCascade.h"
#include "irrlicht_interface/ChBodySceneNode.h" 


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

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	ChGlobals* GLOBAL_Vars = DLL_CreateGlobals();

	// 1- Create a ChronoENGINE physical system: all bodies and constraints
	//    will be handled by this ChSystem object.
	ChSystem my_system;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrAppInterface application(&my_system, L"Load a robot model from STEP file",core::dimension2d<u32>(800,600),false, true, video::EDT_OPENGL); 

	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	//ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(30,100,30), core::vector3df(30,-80,-30),200,130);
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0.2,1.6,-3.5));


	//
	// Load a STEP file, containing a mechanism. The demo STEP file has been 
	// created using a 3D CAD (in this case, SolidEdge v.18).
	//

		// Create the ChCascadeDoc, a container that loads the STEP model 
		// and manages its subassembles
	ChCascadeDoc mydoc;

	ChBodySceneNodeAuxRef* mrigidBody_base	= 0;
	ChBodySceneNodeAuxRef* mrigidBody_turret = 0;
	ChBodySceneNodeAuxRef* mrigidBody_bicep = 0;
	ChBodySceneNodeAuxRef* mrigidBody_elbow = 0;
	ChBodySceneNodeAuxRef* mrigidBody_forearm = 0;
	ChBodySceneNodeAuxRef* mrigidBody_wrist = 0;
	ChBodySceneNodeAuxRef* mrigidBody_hand = 0;
	ChBodySceneNodeAuxRef* mrigidBody_cylinder = 0;
	ChBodySceneNodeAuxRef* mrigidBody_rod = 0;


		// load the STEP model using this command:
	bool load_ok = mydoc.Load_STEP("..\\data\\cascade\\IRB7600_23_500_m2000_rev1_01_decorated.stp");

		// print the contained shapes
	mydoc.Dump(GetLog());

	ChCollisionModel::SetDefaultSuggestedEnvelope(0.002);
	ChCollisionModel::SetDefaultSuggestedMargin(0.001);

	// In most CADs the Y axis is horizontal, but we want it vertical.
	// So define a root transformation for rotating all the imported objects.
	ChQuaternion<> rotation1;
	rotation1.Q_from_AngAxis(-CH_C_PI/2, VECT_X); // 1: rotate 90° on X axis 
	ChQuaternion<> rotation2;
	rotation2.Q_from_AngAxis(CH_C_PI, VECT_Y);	 // 2: rotate 180° on vertical Y axis
	ChQuaternion<> tot_rotation = rotation2 % rotation1;  // rotate on 1 then on 2, using quaternion product
	ChFrameMoving<> root_frame( ChVector<>(0,0,0), tot_rotation); 
	
	if (load_ok)
	{
			// Retrieve some sub shapes from the loaded model, using
			// the GetNamedShape() function, that can use path/subpath/subsubpath/part 
			// syntax and * or ? wldcards, etc.

		TopoDS_Shape shape_base;
		if (mydoc.GetNamedShape(shape_base, "Assem10/Assem8" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_base = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_base);

				// The base is fixed to the ground
			mrigidBody_base->GetBody()->SetBodyFixed(true);

				// Move the body as for global displacement/rotation by pre-transform its coords.
				// Note, it could be written also as   mrigidBody_base->GetBody() %= root_frame; 
			mrigidBody_base->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";


		TopoDS_Shape shape_turret;
		if (mydoc.GetNamedShape(shape_turret, "Assem10/Assem4" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_turret = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_turret);

				// Move the body as for global displacement/rotation
			mrigidBody_turret->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";


		TopoDS_Shape shape_bicep;
		if (mydoc.GetNamedShape(shape_bicep, "Assem10/Assem1" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_bicep = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_bicep);

				// Move the body as for global displacement/rotation
			mrigidBody_bicep->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";


		TopoDS_Shape shape_elbow;
		if (mydoc.GetNamedShape(shape_elbow, "Assem10/Assem5" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_elbow = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_elbow);

				// Move the body as for global displacement/rotation
			mrigidBody_elbow->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";

		
		TopoDS_Shape shape_forearm;
		if (mydoc.GetNamedShape(shape_forearm, "Assem10/Assem7" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_forearm = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_forearm);

				// Move the body as for global displacement/rotation
			mrigidBody_forearm->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";

		
		TopoDS_Shape shape_wrist;
		if (mydoc.GetNamedShape(shape_wrist, "Assem10/Assem6" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_wrist = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_wrist);

				// Move the body as for global displacement/rotation
			mrigidBody_wrist->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";


		TopoDS_Shape shape_hand;
		if (mydoc.GetNamedShape(shape_hand, "Assem10/Assem9" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_hand = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_hand);

			//mrigidBody_hand->GetBody()->SetBodyFixed(true);

				// Move the body as for global displacement/rotation
			mrigidBody_hand->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";


		TopoDS_Shape shape_cylinder;
		if (mydoc.GetNamedShape(shape_cylinder, "Assem10/Assem3" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_cylinder = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_cylinder);

				// Move the body as for global displacement/rotation
			mrigidBody_cylinder->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";


		TopoDS_Shape shape_rod;
		if (mydoc.GetNamedShape(shape_rod, "Assem10/Assem2" ))
		{
				// Add the shape to the Irrlicht system, to get also visualization.
			mrigidBody_rod = (ChBodySceneNodeAuxRef*)addChBodySceneNode_Cascade_C(
									&my_system, application.GetSceneManager(), 
									shape_rod);

				// Move the body as for global displacement/rotation
			mrigidBody_rod->GetBody()->ConcatenatePreTransformation(root_frame);
		}
		else GetLog() << "Warning. Desired object not found in document \n";


	}
	else GetLog() << "Warning. Desired STEP file could not be opened/parsed \n";



	if (!mrigidBody_base ||
		!mrigidBody_turret ||
		!mrigidBody_bicep ||
		!mrigidBody_elbow ||
		!mrigidBody_forearm ||
		!mrigidBody_wrist ||
		!mrigidBody_hand )
	{
		DLL_DeleteGlobals();
		return 0;
	}

	// Create joints between two parts. 
	// To understand where is the axis of the joint, we can exploit the fact
	// that in the STEP file that we prepared for this demo, we inserted some
	// objects called 'marker' and we placed them aligned to the shafts, so now
	// we can fetch them and get their position/rotation.
	
	TopoDS_Shape shape_marker;


	ChFrame<> frame_marker_base_turret;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem8/marker#1" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_base_turret);
	else GetLog() << "Warning. Desired marker not found in document \n";
		// Transform the abs coordinates of the marker because everything was rotated/moved by 'root_frame' :
	frame_marker_base_turret %= root_frame;

	ChSharedPtr<ChLinkLockRevolute>  my_link1(new ChLinkLockRevolute);
	ChSharedBodyPtr mb1 = mrigidBody_base->GetBody();
	ChSharedBodyPtr mb2 = mrigidBody_turret->GetBody();
	my_link1->Initialize(mb1, mb2, frame_marker_base_turret.GetCoord() );
	my_system.AddLink(my_link1);


	ChFrame<> frame_marker_turret_bicep;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem4/marker#2" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_turret_bicep);
	else GetLog() << "Warning. Desired marker not found in document \n";
	frame_marker_turret_bicep %= root_frame;

	ChSharedPtr<ChLinkLockRevolute>  my_link2(new ChLinkLockRevolute);
	mb1 = mrigidBody_turret->GetBody();
	mb2 = mrigidBody_bicep->GetBody();
	my_link2->Initialize(mb1, mb2, frame_marker_turret_bicep.GetCoord() );
	my_system.AddLink(my_link2);


	ChFrame<> frame_marker_bicep_elbow;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem1/marker#2" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_bicep_elbow);
	else GetLog() << "Warning. Desired marker not found in document \n";
	frame_marker_bicep_elbow %= root_frame;

	ChSharedPtr<ChLinkLockRevolute>  my_link3(new ChLinkLockRevolute);
	mb1 = mrigidBody_bicep->GetBody();
	mb2 = mrigidBody_elbow->GetBody();
	my_link3->Initialize(mb1, mb2, frame_marker_bicep_elbow.GetCoord() );
	my_system.AddLink(my_link3);
	

	ChFrame<> frame_marker_elbow_forearm;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem5/marker#2" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_elbow_forearm);
	else GetLog() << "Warning. Desired marker not found in document \n";
	frame_marker_elbow_forearm %= root_frame;
	
	ChSharedPtr<ChLinkLockRevolute>  my_link4(new ChLinkLockRevolute);
	mb1 = mrigidBody_elbow->GetBody();
	mb2 = mrigidBody_forearm->GetBody();
	my_link4->Initialize(mb1, mb2, frame_marker_elbow_forearm.GetCoord() );
	my_system.AddLink(my_link4);


	ChFrame<> frame_marker_forearm_wrist;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem7/marker#2" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_forearm_wrist);
	else GetLog() << "Warning. Desired marker not found in document \n";
	frame_marker_forearm_wrist %= root_frame;

	ChSharedPtr<ChLinkLockRevolute>  my_link5(new ChLinkLockRevolute);
	mb1 = mrigidBody_forearm->GetBody();
	mb2 = mrigidBody_wrist->GetBody();
	my_link5->Initialize(mb1, mb2, frame_marker_forearm_wrist.GetCoord() );
	my_system.AddLink(my_link5);


	ChFrame<> frame_marker_wrist_hand;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem6/marker#2" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_wrist_hand);
	else GetLog() << "Warning. Desired marker not found in document \n";
	frame_marker_wrist_hand %= root_frame;

	ChSharedPtr<ChLinkLockRevolute>  my_link6(new ChLinkLockRevolute);
	mb1 = mrigidBody_wrist->GetBody();
	mb2 = mrigidBody_hand->GetBody();
	my_link6->Initialize(mb1, mb2, frame_marker_wrist_hand.GetCoord() );
	my_system.AddLink(my_link6);


	ChFrame<> frame_marker_turret_cylinder;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem4/marker#3" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_turret_cylinder);
	else GetLog() << "Warning. Desired marker not found in document \n";
	frame_marker_turret_cylinder %= root_frame;

	ChSharedPtr<ChLinkLockRevolute>  my_link7(new ChLinkLockRevolute);
	mb1 = mrigidBody_turret->GetBody();
	mb2 = mrigidBody_cylinder->GetBody();
	my_link7->Initialize(mb1, mb2, frame_marker_turret_cylinder.GetCoord() );
	my_system.AddLink(my_link7);


	ChFrame<> frame_marker_cylinder_rod;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem3/marker#2" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_cylinder_rod);
	else GetLog() << "Warning. Desired marker not found in document \n";
	frame_marker_cylinder_rod %= root_frame;

	ChSharedPtr<ChLinkLockCylindrical>  my_link8(new ChLinkLockCylindrical);
	mb1 = mrigidBody_cylinder->GetBody();
	mb2 = mrigidBody_rod->GetBody();
	my_link8->Initialize(mb1, mb2, frame_marker_cylinder_rod.GetCoord() );
	my_system.AddLink(my_link8);


	ChFrame<> frame_marker_rod_bicep;
	if (mydoc.GetNamedShape(shape_marker, "Assem10/Assem2/marker#2" ))
		ChCascadeDoc::FromCascadeToChrono(shape_marker.Location(), frame_marker_rod_bicep);
	else GetLog() << "Warning. Desired marker not found in document \n";
	frame_marker_rod_bicep %= root_frame;

	ChSharedPtr<ChLinkLockCylindrical>  my_link9(new ChLinkLockCylindrical);
	mb1 = mrigidBody_rod->GetBody();
	mb2 = mrigidBody_bicep->GetBody();
	my_link9->Initialize(mb1, mb2, frame_marker_rod_bicep.GetCoord() );
	my_system.AddLink(my_link9);


	// Add a couple of markers for the 'lock' constraint between the hand and the 
	// absolute reference: when we will move the marker in absolute reference, the
	// hand will follow it.

	ChSharedMarkerPtr  my_marker_hand(new ChMarker);
	ChSharedMarkerPtr  my_marker_move(new ChMarker);

	mrigidBody_hand->GetBody()->AddMarker(my_marker_hand);
	mrigidBody_base->GetBody()->AddMarker(my_marker_move);

	ChQuaternion<> rot_on_x; rot_on_x.Q_from_AngAxis(CH_C_PI/2, VECT_X);
	ChFrame<> frame_marker_move = ChFrame<>(VNULL, rot_on_x) >> frame_marker_wrist_hand ;

	my_marker_hand->Impose_Abs_Coord( frame_marker_wrist_hand.GetCoord() );
	my_marker_move->Impose_Abs_Coord( frame_marker_move.GetCoord() );

	ChSharedPtr<ChLinkLockLock>  my_link_teacher(new ChLinkLockLock);
	my_link_teacher->Initialize(my_marker_hand, my_marker_move);
	my_system.AddLink(my_link_teacher);



	// Set motions for Z and Y coordinates of the 'my_link_teacher' marker,
	// so that the hand will follow it. To do so, we create four segments of
	// motion for Z coordinate and four for Y coordinate, we join them with 
	// ChFunction_Sequence and we repeat sequence by ChFunction_Repeat

	ChFunction_ConstAcc* motlaw_z1 = new ChFunction_ConstAcc();
	motlaw_z1->Set_h(-0.7);
	motlaw_z1->Set_end(1);
	ChFunction_Const*	 motlaw_z2 = new ChFunction_Const();
	ChFunction_ConstAcc* motlaw_z3 = new ChFunction_ConstAcc();
	motlaw_z3->Set_h( 0.7);
	motlaw_z3->Set_end(1);
	ChFunction_Const*	 motlaw_z4 = new ChFunction_Const();
	ChFunction_Sequence* motlaw_z_seq = new ChFunction_Sequence();
	motlaw_z_seq->InsertFunct(motlaw_z1, 1,  1, true); 
	motlaw_z_seq->InsertFunct(motlaw_z2, 1,  1, true);  // true = force c0 continuity, traslating fx
	motlaw_z_seq->InsertFunct(motlaw_z3, 1,  1, true);
	motlaw_z_seq->InsertFunct(motlaw_z4, 1,  1, true);
	ChFunction_Repeat* motlaw_z = new ChFunction_Repeat();
	motlaw_z->Set_fa(motlaw_z_seq);
	motlaw_z->Set_window_length(4);

	ChFunction_Const*	 motlaw_y1 = new ChFunction_Const();
	ChFunction_ConstAcc* motlaw_y2 = new ChFunction_ConstAcc();
	motlaw_y2->Set_h(-0.6);
	motlaw_y2->Set_end(1);
	ChFunction_Const*	 motlaw_y3 = new ChFunction_Const();
	ChFunction_ConstAcc* motlaw_y4 = new ChFunction_ConstAcc();
	motlaw_y4->Set_h(0.6);
	motlaw_y4->Set_end(1);
	ChFunction_Sequence* motlaw_y_seq = new ChFunction_Sequence();
	motlaw_y_seq->InsertFunct(motlaw_y1, 1,  1, true);
	motlaw_y_seq->InsertFunct(motlaw_y2, 1,  1, true);  // true = force c0 continuity, traslating fx
	motlaw_y_seq->InsertFunct(motlaw_y3, 1,  1, true);
	motlaw_y_seq->InsertFunct(motlaw_y4, 1,  1, true);
	ChFunction_Repeat* motlaw_y = new ChFunction_Repeat();
	motlaw_y->Set_fa(motlaw_y_seq);
	motlaw_y->Set_window_length(4);

	my_marker_move->SetMotion_Z(motlaw_z);
	my_marker_move->SetMotion_Y(motlaw_y);




	// Create a large cube as a floor.

	ChBodySceneNode* mfloor = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, application.GetSceneManager(),
											1000.0,
											ChVector<>(0,-0.6,0),
											ChQuaternion<>(1,0,0,0), 
											ChVector<>(20,1,20) );
	mfloor->GetBody()->SetBodyFixed(true);
	mfloor->GetBody()->SetCollide(true);
	video::ITexture* cubeMap = application.GetVideoDriver()->getTexture("../data/blu.png");
	mfloor->setMaterialTexture(0,	cubeMap);


	// Modify the settings of the solver.
	// By default, the solver might not have sufficient precision to keep the 
	// robot joints 'mounted'. Expecially, the SOR, SSOR and other fixed point methods
	// cannot simulate well this robot problem because the mass of the last body in the 
	// kinematic chain, i.e. the hand, is very low when compared to other bodies, so 
	// the convergence of the solver would be bad when 'pulling the hand' as in this 
	// 'teaching mode' IK.
	// So switch to a more precise solver; this LCP_ITERATIVE_PMINRES is fast
	// and precise (although it is not fit for frictional collisions):

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES);


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

		// .. perform timestep simulation
		application.DoStep();

		// .. plot something on realtime view
		ChIrrTools::drawChFunction(application.GetDevice(),motlaw_z, 0, 10, -0.9, 0.2);  


		application.GetVideoDriver()->endScene(); 
	}


	
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


