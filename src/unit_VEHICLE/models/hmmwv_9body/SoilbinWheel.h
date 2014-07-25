#ifndef SOILBINWHEEL_H
#define SOILBINWHEEL_H

///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - Create a wheel and its collision shape for the soilBin
//		- either a convex hull, or a cylinder
//
///////////////////////////////////////////////////

#include "assets/ChObjShapeFile.h"
#include "assets/ChAssetLevel.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChTexture.h"
#include "physics/CHapidll.h" 
#include "physics/CHsystem.h"

using namespace chrono;
// using namespace chrono::collision;

class SoilbinWheel {
public:
	//data
	// ChBodySceneNode* wheel;
	ChSharedPtr<ChBody> wheel;	// use irr assets to not screw up inertia definition
	ChSharedPtr<ChObjShapeFile> m_obj;
	ChSharedPtr<ChCylinderShape> mcyl;
	ChSharedPtr<ChAssetLevel> meshLevel;

	// Use Alsesandro's convex decomposition for C-D with the Trelleborg tire
	// can turn rigid body collision on/off, with enable_collision
	// NOTE: use this same constructor when you just want to use the wheelMeshFile for visualization, by 
	//		setting enable_collision to false (so the collision will be taken care of by the Terramechanics routine)
	SoilbinWheel(ChSystem& msys, 
		ChVector<>& mposition, const ChQuaternion<>& wheelRot,
		const ChQuaternion<>& wheelMeshrot,
		ChVector<>& inertia, double mass,
		const std::string wheelMeshFile,
		const double mu = 0.4, const double cohesion = 0.0,
		const bool enable_collision = true): wheel(new ChBody)
	{

		// the mesh for the visualization (independent from the collision shape)
		ChSharedPtr<ChObjShapeFile> m_obj(new ChObjShapeFile);
		m_obj->SetFilename(wheelMeshFile );
		// add a level between the body and irr asset, so it can be rotated relative to the rigid body
		ChSharedPtr<ChAssetLevel> meshLevel(new ChAssetLevel);
		meshLevel->AddAsset(m_obj);
		meshLevel->GetFrame().SetRot( wheelMeshrot );	// shouldn't have to rotate the rigid body, only the mesh!
													// allows for body inertia to be found relative to the global c-sys
		// add the asset's level to the body
		wheel->AddAsset(meshLevel);
		ChSharedPtr<ChTexture> mtexture(new ChTexture);
		mtexture->SetTextureFilename("../data/tire.png");
		meshLevel->AddAsset(mtexture);

		// now, rigid body info should be added directly to the body
		wheel->SetMass(mass);
		wheel->SetInertiaXX(inertia);
		// set material surface params (friction, cohesion)
		wheel->GetMaterialSurface()->SetFriction(mu);
		wheel->GetMaterialSurface()->SetCohesion(cohesion);
 
		// Clear model. The colliding shape description MUST be between  ClearModel() .. BuildModel() pair.
		wheel->GetCollisionModel()->ClearModel();


		// Describe the (invisible) colliding shape by adding the 'carcass' decomposed shape and the 
		// 'knobs'. Since these decompositions are only for 1/15th of the wheel, use for() to pattern them.
		for (double mangle = 0; mangle < 360.; mangle+= (360./15.))
		{
			ChQuaternion<>myrot;
			ChStreamInAsciiFile myknobs("../data/tractor_wheel_knobs.chulls");
			ChStreamInAsciiFile myslice("../tractor_wheel_slice.chulls");
			myrot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
			ChMatrix33<> mm(myrot);
			wheel->GetCollisionModel()->AddConvexHullsFromFile(myknobs,ChVector<>(0,0,0),mm);
			wheel->GetCollisionModel()->AddConvexHullsFromFile(myslice,ChVector<>(0,0,0),mm);
			//break;
		}
	
		// Complete the description.
		wheel->GetCollisionModel()->BuildModel();
		wheel->SetPos( mposition);
		// rotate the wheel
		wheel->SetRot( wheelRot);

		wheel->SetCollide(enable_collision);
		
		// finally, add the body to the system
		msys.Add(wheel);
	}

	// use a hollow cylinder as the wheel body. Note, the last input arg doesn't do anything
	// Just needed a different constructor
	SoilbinWheel(ChSystem& msys, 
		ChVector<>& mposition, const ChQuaternion<>& wheelRot,
		double mass, double cyl_width, double cyl_d_outer, double cyl_d_inner,
		const bool enable_collision,
		const std::string& wheelMeshFile="none",
		const ChQuaternion<>& meshRot = QUNIT,
		const double mu = 0.7, const double coh = 0.0): wheel(new ChBody)
	{
		double r2 = cyl_d_outer/2.0;	// outer radius
		double r1 = cyl_d_inner/2.0;	// inner radius
		double h = cyl_width;		// height

		// if you want a wheel mesh, the name will be different than "none"
		if( wheelMeshFile=="none" ) {
			// use a cylinder for the tm tests
			ChSharedPtr<ChCylinderShape> mcyl(new ChCylinderShape);
			mcyl->GetCylinderGeometry().p1 = ChVector<>(0, cyl_width/2.0, 0);
			mcyl->GetCylinderGeometry().p2 = ChVector<>(0, -cyl_width/2.0, 0);
			mcyl->GetCylinderGeometry().rad = r2;
			wheel->AddAsset(mcyl);
		} else {
			// use the input mesh specified as an asset
			ChSharedPtr<ChObjShapeFile> mesh_obj(new ChObjShapeFile);
			mesh_obj->SetFilename(wheelMeshFile);
			wheel->AddAsset(mesh_obj);
		}
		//  create the CD model using a cylinder in either case
		wheel->GetCollisionModel()->ClearModel();
		wheel->GetCollisionModel()->AddCylinder(r2,r2,h/2.0);
		wheel->GetCollisionModel()->BuildModel();

		// set inertia, friction coef, collide
		wheel->SetMass(mass);
		double iyy = (mass/2.0)*(r1*r1 + r2*r2);
		double ixx = (mass/12.0)*(3.0*(r2*r2+r1*r1) + h*h);
		ChVector<> inertia( ixx, iyy, ixx);
		wheel->SetInertiaXX(inertia);
		wheel->GetMaterialSurface()->SetFriction(mu);
		wheel->GetMaterialSurface()->SetCohesion(coh);

		GetLog() << "wheel CM, inertia = " << mposition << "\n" <<  inertia << "\n";

		// set the wheel ICs, collide on/off
		wheel->SetPos(mposition);
		wheel->SetRot(wheelRot);
		wheel->SetCollide(enable_collision);

		// add a rubber texture
		ChSharedPtr<ChTexture> mtexture(new ChTexture);
		mtexture->SetTextureFilename("../data/tire.png");
		wheel->AddAsset(mtexture);
		msys.Add(wheel);
	}

	// toggle visibility
	void toggleVisibility(bool isVisible)
	{
		m_obj->SetVisible(isVisible);
	}

	// return a reference to the body
	ChSharedPtr<ChBody>& GetBody() {
		return this->wheel;
	}

	~SoilbinWheel() {
		// ChSystem* msys = wheel->GetBody()->GetSystem();
		// remove the bodies, joints
		// wheel->remove();

		// NOTE: DO I NEED TO DELETE THE SHARED PTRS WITH IRR ASSETS?
	}
};

#endif