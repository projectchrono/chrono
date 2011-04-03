#ifndef CHIRRCASCADE_H
#define CHIRRCASCADE_H

//////////////////////////////////////////////////
//
//   ChIrrCascade.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Some functions to allow easy creation of
//   ChBodySceneNode C++ objects in Irrlicht+ChronoEngine+OpenCascade
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChIrrMeshTools.h"
#include "geometry/ChCTriangleMesh.h"
#include "ChIrrCascadeMeshTools.h"
#include "ChCascadeDoc.h"

namespace irr
{
namespace scene
{



/// Easy-to-use function which creates a ChBodySceneNode 
/// corresponding to a OpenCascade mesh (assuming TopoDS_Shape location is relative to body csys)
/// Version (A), with full parameters:

static
ISceneNode* addChBodySceneNode_Cascade_A(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   TopoDS_Shape& mshape, 
										   const chrono::ChVector<>& position = chrono::ChVector<>(0,0,0),	
										   const chrono::ChQuaternion<>& rotation = chrono::ChQuaternion<>(1,0,0,0),
										   double mmass = 1.0, 
										   chrono::ChVector<>& XXinertia = chrono::ChVector<>(1,1,1),
										   chrono::ChVector<>& XYinertia = chrono::ChVector<>(0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	if (!aparent)
		aparent = amanager->getRootSceneNode();

	scene::SMesh* mmesh = new scene::SMesh();
	video::SColor clr(255, 100,120,125);
	irr::scene::ChIrrCascadeMeshTools::fillIrrlichtMeshFromCascade(mmesh, mshape, 0.5);
	scene::SAnimatedMesh* amesh = new scene::SAnimatedMesh();
	amesh->addMesh(mmesh);
	mmesh->drop();

	// create a ChronoENGINE rigid body
	ChBodySceneNode* rigidBodyZ = new ChBodySceneNode(asystem, 
													amesh, 
													aparent,
													amanager,
													mid
													);

			// set some ChronoENGINE specific properties for the body...
	rigidBodyZ->GetBody()->SetPos(position);
	rigidBodyZ->GetBody()->SetRot(rotation);
	rigidBodyZ->GetBody()->SetMass(mmass);
	rigidBodyZ->GetBody()->SetInertiaXX(XXinertia);
	rigidBodyZ->GetBody()->SetInertiaXY(XYinertia); 
	rigidBodyZ->drop();

	amesh->drop();

	return rigidBodyZ;	
}



/// Super-easy-to-use function which creates a ChBodySceneNode 
/// corresponding to a OpenCascade mesh, where the position and rotation
/// of the reference is automatically set according to the OpenCascade shape:
/// Version (B), with auto computation of reference position and rotation:

static
ISceneNode* addChBodySceneNode_Cascade_B(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   TopoDS_Shape& mshape, 
										   double mmass = 1.0, 
										   chrono::ChVector<>& XXinertia = chrono::ChVector<>(1,1,1),
										   chrono::ChVector<>& XYinertia = chrono::ChVector<>(0,0,0),
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	TopLoc_Location mloc;
	mloc = mshape.Location();

	gp_XYZ mtr = mloc.Transformation().TranslationPart();
	chrono::GetLog() << " Addins shape with abs pos at: "<< mtr.X() <<" "<< mtr.Y() << " "<<mtr.Z() <<" \n";

	chrono::ChFrame<> mframe;
	chrono::cascade::ChCascadeDoc::FromCascadeToChrono(mloc, mframe);

	chrono::GetLog() << "  create obj at COG " << mframe.GetPos().x << " " << mframe.GetPos().y << " " << mframe.GetPos().z << " \n";

	TopoDS_Shape objshape = mshape;
	objshape.Location( TopLoc_Location() ); // Reset shape location to absolute csys (identity). 

	return addChBodySceneNode_Cascade_A(asystem, amanager, objshape, mframe.GetPos(), mframe.GetRot(), mmass, XXinertia, XYinertia, aparent, mid);
}




/// Super-Super-easy-to-use function which creates a ChBodySceneNode 
/// corresponding to a OpenCascade mesh, where the position and rotation
/// of the reference is automatically set according to the COG of the OpenCascade shape:
/// Version (C), with auto computation of reference position and rotation, and mass and inertia
///***TO DO*** - requires a new ChBodyWithCog for cog in different place than csys

static
ISceneNode* addChBodySceneNode_Cascade_C(chrono::ChSystem* asystem,
										   ISceneManager* amanager,
										   TopoDS_Shape& mshape, 
										   double density = 1000.0, 
										   ISceneNode* aparent=0, 
										   s32 mid=-1
										   )
{
	TopLoc_Location mloc;
	mloc = mshape.Location();
	chrono::ChFrame<> mframe;
	chrono::cascade::ChCascadeDoc::FromCascadeToChrono(mloc, mframe);

	chrono::ChVector<> mcog;
	chrono::ChVector<> minertiaXX;
	chrono::ChVector<> minertiaXY;
	double mvol;
	double mmass;
	chrono::cascade::ChCascadeDoc::GetVolumeProperties(mshape, density, mcog, minertiaXX, minertiaXY, mvol, mmass); 

	chrono::ChFrame<> cog_frame;
	cog_frame.SetPos(mcog);
	cog_frame.SetRot(mframe.GetRot());

	TopLoc_Location newloc;
	chrono::cascade::ChCascadeDoc::FromChronoToCascade(cog_frame, newloc);

	TopoDS_Shape objshape = mshape;
	objshape.Location( newloc ); // Reset shape location to COG csys. 

	return addChBodySceneNode_Cascade_A(asystem, amanager, objshape, cog_frame.GetPos(), cog_frame.GetRot(), mmass, minertiaXX, minertiaXY, aparent, mid);
}






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif // END of ChIrrCascade.h
