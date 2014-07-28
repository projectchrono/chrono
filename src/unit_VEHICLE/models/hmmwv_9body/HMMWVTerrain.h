#ifndef HMMWVTERRAIN_H
#define HMMWVTERRAIN_H

// This will allow finer control of the terrain surface features for the HMMWV,
// including non-flat surface, obstacles, etc.
// Author: Justin Madsen
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include <fstream>
#include <ostream>
#include <iostream>
#include "physics/ChBodyEasy.h"

namespace chrono{

// for easy output of ChVectors
std::ostream&  operator<<(std::ostream& os, const ChVector<>& v);


// define the terrain for the HMMWV test model. Based on your choice of TireForceType, 
//	0 = rigid contact, 1 = PacTire, 2 = Justin's soft soil
// Turn on DVI contact for 0, else turn off contact
class HMMWVTerrain {
public:
	// rigid box, fixed to ground.
	ChSharedPtr<ChBody> floor;

	// @brief create the terrain for the HMMWV
	// @param pos		where to put the center of the top surface of the floor
	HMMWVTerrain(ChSystem& system, ChVector<> pos = ChVector<>(0.0,0.0,0.0),
		double terrainWidth = 100.0, double terrainLength =  100.0,
		const double muWall = 0.7, const double cohesionWall = 0.0,
		const bool collide=true): msys(&system)
	{
		// create the floor, 20 cm thick by default
		this->floor = ChSharedPtr<ChBodyEasyBox>(new ChBodyEasyBox(terrainWidth/2.0, terrainLength/2.0, 0.1,
			1000.0, true, true) );
		floor->GetMaterialSurface()->SetFriction(muWall);
		floor->GetMaterialSurface()->SetCohesion(cohesionWall);
		floor->SetBodyFixed(true);
	//	floor->setMaterialTexture(0, cubeMap);

		// move the center of the floor so the top-most surface is at the z-height specified by input 'pos'
		ChVector<> floorCenter = ChVector<>(pos);
		floorCenter.z -= 0.1;
		floor->SetPos(pos);
		floor->SetInertiaXX(ChVector<>(1,1,1));
		floor->SetMass(100.0);
		floor->SetName("ground");
		system.Add(floor);

		// add some color
		ChSharedPtr<ChVisualization> floorColor(new ChVisualization);
		floorColor->SetColor(ChColor(0.4,0.4,0.6));
		floor->AddAsset(floorColor);	// add the color asset

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
			// position, size, orientation are random
			ChVector<> pos(loc.x + (ChRandom()-0.5)*5.0, loc.y + ChRandom(), loc.z + (ChRandom()-0.5)*5.0);
			ChQuaternion<> ori(ChRandom(), ChRandom(), ChRandom(), ChRandom() );
			ori.Normalize();

			// using a rectangle shape. x = depth, z = width
			ChVector<> size(avg_depth*(ChRandom()+0.5), height, avg_width*(ChRandom()+0.5) );
			
			ChSharedPtr<ChBodyEasyBox> obstacle(new ChBodyEasyBox(size.x, size.y, size.z, 2000.0, true, true) );
			// set the body info
			obstacle->SetPos(pos);
			obstacle->SetRot(ori);
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