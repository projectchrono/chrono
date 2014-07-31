#ifndef CHWHEEL_H
#define CHWHEEL_H

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

class ChWheel {
public:
	//data
	// ChBodySceneNode* wheel;
	ChSharedPtr<ChBody> wheelBody;	// use irr assets to not screw up inertia definition
	ChSharedPtr<ChObjShapeFile> m_obj;
	ChSharedPtr<ChCylinderShape> mcyl;
	ChSharedPtr<ChAssetLevel> meshLevel;

	// use a hollow cylinder as the wheel body. Note, the last input arg doesn't do anything
	// Just needed a different constructor
	ChWheel(ChSystem& msys, 
		ChVector<>& mposition, const ChQuaternion<>& wheelRot,
		double mass, double cyl_width, double cyl_d_outer, double cyl_d_inner,
		const bool enable_collision,
		const std::string& wheelMeshFile="none",
		const ChQuaternion<>& meshRot = QUNIT,
		const double mu = 0.7, const double coh = 0.0): wheelBody(new ChBody)
	{
		wheelBody->SetName("wheel");
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
			wheelBody->AddAsset(mcyl);
		} else {
			// use the input mesh specified as an asset
			ChSharedPtr<ChObjShapeFile> mesh_obj(new ChObjShapeFile);
			mesh_obj->SetFilename(wheelMeshFile);
			ChSharedPtr<ChAssetLevel> wheel_level(new ChAssetLevel);
			wheel_level->AddAsset(mesh_obj);
			wheelBody->AddAsset(wheel_level);
		}
		//  create the CD model using a cylinder in either case
		wheelBody->GetCollisionModel()->ClearModel();
		wheelBody->GetCollisionModel()->AddCylinder(r2,r2,h/2.0);
		wheelBody->GetCollisionModel()->BuildModel();

		// set inertia, friction coef, collide
		wheelBody->SetMass(mass);
		double iyy = (mass/2.0)*(r1*r1 + r2*r2);
		double ixx = (mass/12.0)*(3.0*(r2*r2+r1*r1) + h*h);
		ChVector<> inertia( ixx, iyy, ixx);
		wheelBody->SetInertiaXX(inertia);
		wheelBody->GetMaterialSurface()->SetFriction(mu);
		wheelBody->GetMaterialSurface()->SetCohesion(coh);

		GetLog() << "wheel CM, inertia = " << mposition << "\n" <<  inertia << "\n";

		// set the wheel ICs, collide on/off
		wheelBody->SetPos(mposition);
		wheelBody->SetRot(wheelRot);
		wheelBody->SetCollide(enable_collision);

		// add a rubber texture
		ChSharedPtr<ChTexture> mtexture(new ChTexture);
		mtexture->SetTextureFilename("../data/bluwhite.png");
		wheelBody->AddAsset(mtexture);
		msys.Add(wheelBody);



		// DEBUG
		wheelBody->SetCollide(false);




	}

	// toggle visibility
	void toggleVisibility(bool isVisible)
	{
		m_obj->SetVisible(isVisible);
	}

	// return a reference to the body
	ChSharedPtr<ChBody>& GetBody() {
		return this->wheelBody;
	}

	~ChWheel() {
		// ChSystem* msys = wheel->GetBody()->GetSystem();
		// remove the bodies, joints
		// wheel->remove();

		// NOTE: DO I NEED TO DELETE THE SHARED PTRS WITH IRR ASSETS?
	}
};

#endif