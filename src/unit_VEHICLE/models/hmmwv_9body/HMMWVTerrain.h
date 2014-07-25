#ifndef HMMWVTERRAIN_H
#define HMMWVTERRAIN_H

// This will allow finer control of the terrain surface features for the HMMWV,
// including non-flat surface, obstacles, etc.
// NOTE: No Irrlicht assets in this class, it should work both with and without it
//		SO, can't create/delete visualization objects during run-time, at least not
//		directly in this class
// Author: Justin Madsen
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include <fstream>
#include <ostream>
#include <iostream>

namespace chrono{

// for easy output of ChVectors
std::ostream&  operator<<(std::ostream& os, const ChVector<>& v);


// create a test mechanism made up of 2 bodies
// a hub to connect to the wheel spindle and apply a torque through it
// a weight that moves vertically and horizontally w.r.t. the wheel spindle CM location
// spring/damper to apply a vertical load to the tire
// Purpose: only allow the tire to operate In-Plane, to simulate how a soil bin test mechaism works
class HMMWVTerrain {
public:
	// First, just create some simple terrain details (floor and some walls, that will not be shown)
	ChSharedPtr<ChBody> floor;
	ChSharedPtr<ChBody> wall1;
	ChSharedPtr<ChBody> wall2;
	ChSharedPtr<ChBody> wall3;
	ChSharedPtr<ChBody> wall4;

	// GUI-tweaked data

	// @brief create the terrain for the HMMWV
	// @param pos		where to put the center of the floor, should be slightly below the HMMWV chassis CM

	HMMWVTerrain(ChSystem& system, ChVector<> pos = ChVector<>(0.0,0.0,0.0),
		double terrainWidth = 100.0, double terrainLength =  100.0,
		const double muWall = 0.7, const double cohesionWall = 0.0,
		const bool collide=true): msys(&system), floor(new ChBody), wall1(new ChBody), wall2(new ChBody), wall3(new ChBody), wall4(new ChBody)
	{
		double wallThickness = std::min<double>(terrainWidth, terrainLength) / 20.0;	// wall width = 1/10 of min of bin dims
		double wallHeight = 1.0;
		// create the floor
		ChVector<> floordims( terrainWidth/2.0, wallThickness, terrainLength/2.0);
		floor->GetCollisionModel()->ClearModel();
		floor->GetCollisionModel()->AddBox(floordims.x, floordims.y, floordims.z); // , &pos);
		floor->GetMaterialSurface()->SetFriction(muWall);
		floor->GetMaterialSurface()->SetCohesion(cohesionWall);
		floor->GetCollisionModel()->BuildModel();
		floor->SetCollide(collide);
		floor->SetBodyFixed(true);
	//	floor->setMaterialTexture(0, cubeMap);
		floor->SetPos(pos);
		floor->SetInertiaXX(ChVector<>(1,1,1));
		floor->SetMass(10.0);
		system.Add(floor);

		// attach the visualization assets, floor
		ChSharedPtr<ChBoxShape> mboxfloor(new ChBoxShape);
		// mboxfloor->GetBoxGeometry().Pos = pos;
		mboxfloor->GetBoxGeometry().Size = floordims;
		floor->AddAsset(mboxfloor);	// add the box shape asset
		ChSharedPtr<ChVisualization> floorColor(new ChVisualization);
		floorColor->SetColor(ChColor(0.4,0.4,0.6));
		floor->AddAsset(floorColor);	// add the color asset

		// add the walls to the soilBin, w.r.t. width, length, height of bin
		// wall 1, sidewall 1
		wall1->GetCollisionModel()->ClearModel();
		ChVector<> wall1Pos = ChVector<>(pos.x -terrainWidth/2.0 - wallThickness/2.0, pos.y + wallHeight/2.0, pos.z);
		ChVector<> wall1dims(wallThickness,wallHeight/2.0+wallThickness,terrainLength/2.0-wallThickness/2.0);
		wall1->GetCollisionModel()->AddBox(wall1dims.x, wall1dims.y, wall1dims.z); // , &wall1Pos);
		wall1->GetMaterialSurface()->SetFriction(muWall);
		wall1->GetMaterialSurface()->SetCohesion(cohesionWall);
		wall1->GetCollisionModel()->BuildModel();
		wall1->SetBodyFixed(true);
		wall1->SetCollide(collide);
		// wall1->setMaterialTexture(0,	cubeMap);
		wall1->SetPos(wall1Pos);
		system.Add(wall1);

		// add the visualization assets, wall 1
		ChSharedPtr<ChBoxShape> mboxwall1(new ChBoxShape);
		// mboxwall1->GetBoxGeometry().Pos = wall1Pos;
		mboxwall1->GetBoxGeometry().Size = wall1dims;
		wall1->AddAsset(mboxwall1);	// add the box shape asset
		ChSharedPtr<ChTexture> wallTex(new ChTexture);
		wallTex->SetTextureFilename("../data/concrete.jpg");
		wall1->AddAsset(wallTex);	// add a texture asset

		// wall 2, sidewall 2
		ChVector<> wall2pos(pos.x + terrainWidth/2.0 + wallThickness/2.0, pos.y + wallHeight/2.0, pos.z);
		ChVector<> wall2dims(wallThickness,wallHeight/2.0+wallThickness,terrainLength/2.0-wallThickness/2.0);
		wall2->GetCollisionModel()->ClearModel();
		wall2->GetCollisionModel()->AddBox( wall2dims.x, wall2dims.y, wall2dims.z); // , &wall2pos);
		wall2->GetMaterialSurface()->SetFriction(muWall);
		wall2->GetMaterialSurface()->SetCohesion(cohesionWall);
		wall2->SetCollide(collide);
		wall2->GetCollisionModel()->BuildModel();
		wall2->SetBodyFixed(true);
		// wall2->setMaterialTexture(0,	rockMap);
		wall2->SetPos(wall2pos);
		wall2->AddAsset(wallTex);	// reuse the asset from wall1
		system.Add(wall2);

		// add the visualization assets, wall 2
		ChSharedPtr<ChBoxShape> mboxwall2(new ChBoxShape);
		mboxwall2->GetBoxGeometry().Size = wall2dims;
		// wall2->AddAsset(mboxwall2);


		// wall 3
		ChVector<> wall3pos(pos.x, pos.y + wallHeight/2.0, pos.z -terrainLength/2.0 - wallThickness/2.0);
		ChVector<> wall3dims(terrainWidth/2.0 + wallThickness, wallHeight/2.0+wallThickness, wallThickness);
		wall3->GetCollisionModel()->ClearModel();
		wall3->GetCollisionModel()->AddBox(wall3dims.x, wall3dims.y, wall3dims.z); // , &wall3pos);
		wall3->GetMaterialSurface()->SetFriction(muWall);
		wall3->GetMaterialSurface()->SetCohesion(cohesionWall);
		wall3->SetCollide(collide);
		wall3->GetCollisionModel()->BuildModel();
		wall3->SetBodyFixed(true);
		wall3->SetPos(wall3pos);
		wall3->AddAsset(wallTex);	// re-use the wall 1 texture
		system.Add(wall3);

		// add the visualizaiton assets, wall 3
		ChSharedPtr<ChBoxShape> mboxwall3(new ChBoxShape);
		mboxwall3->GetBoxGeometry().Size = wall3dims;
		wall3->AddAsset(mboxwall3);	// add the box asset

		// wall 4
		ChVector<> wall4pos(pos.x, pos.y + wallHeight/2.0, pos.z + terrainLength/2.0 + wallThickness/2.0);
		ChVector<> wall4dims(terrainWidth/2.0 + wallThickness, wallHeight/2.0+wallThickness, wallThickness);
		wall4->GetCollisionModel()->ClearModel();
		wall4->GetCollisionModel()->AddBox(wall4dims.x, wall4dims.y, wall4dims.z); // , &wall4pos);
		wall4->GetMaterialSurface()->SetFriction(muWall);
		wall4->GetMaterialSurface()->SetCohesion(cohesionWall);
		wall4->SetCollide(collide);
		wall4->GetCollisionModel()->BuildModel();
		wall4->SetBodyFixed(true);
		wall4->SetPos(wall4pos);
		wall4->AddAsset(wallTex);	// add the asset from wall 1
		system.Add(wall4);

		// add the visualization assets, wall 4
		ChSharedPtr<ChBoxShape> mboxwall4(new ChBoxShape);
		// mboxwall4->GetBoxGeometry().Pos = wall4pos;
		mboxwall4->GetBoxGeometry().Size = wall4dims;
		wall4->AddAsset(mboxwall4);
		ChSharedPtr<ChTexture> wall4tex(new ChTexture);
		wall4tex->SetTextureFilename("../data/rock.jpg");
		wall4->AddAsset(wall4tex);	// add a texture asset

	}

	// create a few big blocks as obstacles
	void create_some_obstacles(ChVector<> loc) {
		int num_obstacles = 6;
		double obs_mass = 50;
		double avg_width = 2.0;
		double avg_depth = 0.5;
		double height = 0.1;
		for (int i=0; i<num_obstacles; i++) {
			// put some blocks in the way; make them heavy
			ChSharedPtr<ChBody> obstacle(new ChBody);
			// position, size, orientation are random
			ChVector<> pos(loc.x + (ChRandom()-0.5)*5.0, loc.y + ChRandom(), loc.z + (ChRandom()-0.5)*5.0);
			ChQuaternion<> ori(ChRandom(), ChRandom(), ChRandom(), ChRandom() );
			ori.Normalize();
			// set the body info
			obstacle->SetPos(pos);
			obstacle->SetRot(ori);
			obstacle->SetMass(obs_mass);
			// using a rectangle shape. x = depth, z = width
			ChVector<> size(avg_depth*(ChRandom()+0.5), height, avg_width*(ChRandom()+0.5) );
			ChVector<> inertia(size.y*size.y+size.z*size.z, size.x*size.x+size.z*size.z, size.y*size.y+size.x*size.x );
			obstacle->SetInertiaXX(obs_mass/12.0 * inertia);
			// create shape for collision
			ChSharedPtr<ChBoxShape> box(new ChBoxShape);
			box->GetBoxGeometry().Pos = size;
			obstacle->AddAsset(box);
			// create the box collision shape
			obstacle->GetCollisionModel()->ClearModel();
			obstacle->GetCollisionModel()->AddBox(size.x,size.y,size.z);
			obstacle->GetCollisionModel()->BuildModel();
			// finally, add the body to the system
			obstacle->GetMaterialSurface()->SetFriction(0.8);
			msys->Add(obstacle);

		}
	}

	~HMMWVTerrain()
	{
		// remove the bodies
		// note, managed w/ shared ptrs, shouldn't need to worry about calling delete

		// remove joints
//		msys->RemoveLink(spring);
//		msys->Remove(torqueDriver);

	}

private:
	ChSystem* msys;

};


}	// namespace chrono{

#endif		// #ifndef HMMWVTERRAIN_H