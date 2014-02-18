//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
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
//     - collisions and contacts  with DEM
//  
//   CHRONO 
//   ------
//   Multibody dinamics engine
//  
///////////////////////////////////////////////////


#include "physics/ChApidll.h" 
#include "physics/ChSystemDEM.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChContactContainerDEM.h"

#include "collision/ChCModelBulletBody.h"

#include "lcp/ChLcpSolverDEM.h"


using namespace chrono;


void AddWall(ChSharedBodyDEMPtr& body,
             const ChVector<>&    dim,
             const ChVector<>&    loc)
{
	body->GetCollisionModel()->AddBox(dim.x, dim.y, dim.z, loc);
}

void OutputSphere(ChStreamOutAsciiFile& file,
                  ChSystem&             sys,
                  double                time,
                  int                   bodyId)
{
	chrono::Vector bodyAngs;

	for (int i = 0; i < sys.Get_bodylist()->size(); ++i) {
		ChBody* abody = (ChBody*) sys.Get_bodylist()->at(i);
		assert(typeid(*abody) == typeid(ChBodyDEM));

		if (abody->GetIdentifier() == bodyId) {
			const ChVector<>& bodypos = abody->GetPos();
			bodyAngs = abody->GetRot().Q_to_NasaAngles();
			file << time << ", ";
			file << bodypos.x  << ", " << bodypos.y  << ", " << bodypos.z  << ", ";
			file << bodyAngs.x << ", " << bodyAngs.y << ", " << bodyAngs.z << "\n";
			std::cout << time << "  " << bodypos.y << std::endl;
		}
	}
}

int main(int argc, char* argv[])
{
	// Simulation parameters
	double gravity = -9.81;
	double time_step = .001;
	double time_end = 1;

	// Output to file
	bool output_all = false;
	double out_step = 0.01;
	ChStreamOutAsciiFile sph_file("sphere_pos.txt");

	// Parameters for the falling ball
	int             ballId = 100;
	double          radius = .5;
	double          mass = 1;
	ChVector<>      pos(0, 3, 0);
	ChQuaternion<>  rot(1, 0, 0, 0);
	ChVector<>      init_vel(0, 0, 0);

	// Parameters for the containing bin
	int    binId = 200;
	double width = 5;
	double length = 25;
	double height = 2;
	double thickness = .25;

	// Initialize globals
	ChGlobals* GLOBAL_Vars = DLL_CreateGlobals();

	// Create the system
	ChSystemDEM msystem;

	msystem.Set_G_acc(ChVector<>(0, gravity, 0));

	// Create a material (will be used by both objects)
	ChSharedPtr<ChMaterialSurfaceDEM> material;
	material = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	material->SetFriction(0.4f);

	// Create the falling ball
	ChSharedPtr<ChBodyDEM> ball(new ChBodyDEM);

	ball->SetMass(mass);
	ball->SetIdentifier(ballId);
	ball->SetPos(pos);
	ball->SetRot(rot);
	ball->SetPos_dt(init_vel);
	ball->SetBodyFixed(false);
	ball->SetMaterialSurfaceDEM(material);

	ball->SetCollide(true);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(radius);
	ball->GetCollisionModel()->BuildModel();

	ball->SetInertiaXX(0.4*mass*radius*radius*ChVector<>(1,1,1));

	msystem.AddBody(ball);

	// Create the containing bin
	ChSharedPtr<ChBodyDEM> bin(new ChBodyDEM);

	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0, 0, 0));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);
	bin->SetMaterialSurfaceDEM(material);

	bin->GetCollisionModel()->ClearModel();
	AddWall(bin, ChVector<>(width, thickness, length), ChVector<>(0, 0, 0));
	//AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(-width + thickness, height, 0));
	//AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(width - thickness, height, 0));
	//AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, -length + thickness));
	//AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, length - thickness));
	bin->GetCollisionModel()->BuildModel();

	msystem.AddBody(bin);

	// Perform the simulation
	double time = 0;
	double out_time = 0;

	while(time <= time_end) {
		if(output_all || time >= out_time) {
			OutputSphere(sph_file, msystem, time, ballId);
			out_time += out_step;
		}

		msystem.DoStepDynamics(time_step);
		time += time_step;
	}
	
	// Delete globals
	DLL_DeleteGlobals();

	// Note: we do not need to delete mysolver and mycontainer since these will be
	// freed in the ChSystem destructor.

	return 0;
}

