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


#include "physics/CHApidll.h" 
#include "physics/CHSystem.h"

#include "physics/CHbodyDEM.h"
#include "physics/CHcontactContainerDEM.h"
#include "collision/CHcModelBulletBody.h"
#include "lcp/CHlcpSolverDEM.h"

#define SPH_ID  33
#define BOX_ID  200

#define SPH_FILENAME "sph_pos.txt"
#define BOX_FILENAME "box_pos.txt"

// Use the namespace of Chrono
using namespace chrono;


void AddBoundaryBox(ChSystem& mphysicalSystem, 
                    double D1, 
                    double D2, 
                    double D3, 
                    double th,
                    ChStreamOutAsciiFile& file)
{
	double hw[5][3] = {{D1,th,D3},   {th,D2,D3},    { th,D2,D3},    {D1,D2,th},    {D1,D2, th}};
	double o[5][3]  = {{0, -th, 0},  {D1+th,D2,0},  {-D1-th,D2,0},  {0, D2,D3+th}, {0, D2,-D3-th}};

	for (int i = 0; i < 5; i++) {
		ChSharedPtr<ChBodyDEM> p0(new ChBodyDEM);
		p0->SetIdentifier(BOX_ID+i);
		p0->GetCollisionModel()->ClearModel();
		p0->GetCollisionModel()->AddBox(hw[i][0], hw[i][1], hw[i][2]);
		p0->GetCollisionModel()->BuildModel();
		p0->SetCollide(true);
		p0->SetMass(1.0);
		p0->SetBodyFixed(true);
		p0->SetPos(ChVector<>(o[i][0], o[i][1], o[i][2]));
		mphysicalSystem.Add(p0);
		file << o[i][0] << ", " << o[i][1] << ", " << o[i][2] << ", ";
		file << hw[i][0] << ", " << hw[i][1] << ", " << hw[i][2] << "\n";
	}
}


void AddSphere(ChSystem& mySys, double radius, double mass, double height)
{
	ChVector<> pos(0.0, height, 0.0);

	ChSharedPtr<ChBodyDEM> mybody(new ChBodyDEM);
	mybody->SetIdentifier(SPH_ID);

	mySys.Add(mybody);

	mybody->GetCollisionModel()->ClearModel();
	mybody->GetCollisionModel()->AddSphere(radius);
	mybody->GetCollisionModel()->BuildModel();
	mybody->SetCollide(true);

	mybody->SetPos(pos);
	mybody->SetInertiaXX((2.0/5.0)*mass*pow(radius,2)*ChVector<>(1,1,1));
	mybody->SetMass(mass);
}

void OutputSphere(ChStreamOutAsciiFile& file,
                  ChSystem&             sys,
                  double                time)
{
	chrono::Vector bodyAngs;

	std::list<ChPhysicsItem*>::iterator abody = sys.Get_otherphysicslist()->begin();

	while (abody != sys.Get_otherphysicslist()->end()) {
		ChBodyDEM* bbb = dynamic_cast<ChBodyDEM*>(*abody);

		if (bbb && bbb->GetIdentifier() == SPH_ID) {
			const ChVector<>& bodypos = bbb->GetPos();
			bodyAngs = bbb->GetRot().Q_to_NasaAngles();
			file << time << ", ";
			file << bodypos.x  << ", " << bodypos.y  << ", " << bodypos.z  << ", ";
			file << bodyAngs.x << ", " << bodyAngs.y << ", " << bodyAngs.z << "\n";
		}

		abody++;
	}

}


int main(int argc, char* argv[])
{
	// Fixed boundary parameters
	double D1 = 3.0;
	double D2 = 1.0;
	double D3 = 3.0;
	double th = 0.05;

	// Ball parameters
	double radius = 1.0;
	double mass = 1000;
	double height = 1.5;

	// Simulation times
	double time_step = 0.00001;
	double end_time = 6.0;

	// Output to file
	bool output = true;
	double out_step = 3000 * time_step;

	// Initialize counters
	double time = 0.0;
	double out_time = 0.0;

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	ChGlobals* GLOBAL_Vars = DLL_CreateGlobals();

	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;
	mphysicalSystem.Set_G_acc(0.38*mphysicalSystem.Get_G_acc());

	ChStreamOutAsciiFile box_file(BOX_FILENAME);
	ChStreamOutAsciiFile sph_file(SPH_FILENAME);

	// Add fixed bodies
	AddBoundaryBox(mphysicalSystem, D1/2, D2/2, D3/2, th/2, box_file);
	AddSphere(mphysicalSystem, radius, mass, height);

	// Modify some setting of the physical system for the simulation, if you want
	mphysicalSystem.SetLcpSolverType(ChSystem::LCP_DEM);

	// Use the DEM solver
	ChLcpSolverDEM* mysolver = new ChLcpSolverDEM;
	mphysicalSystem.ChangeLcpSolverSpeed(mysolver);
	mysolver->SetMaxIterations(0);

	// Use DEM contact
	// This takes care of the contact between the particles and with the wall
	ChContactContainerDEM* mycontainer = new ChContactContainerDEM;
	mphysicalSystem.ChangeContactContainer(mycontainer);

	// Loop until final time
	while(time < end_time)  {
		if(output && time >= out_time) {
			GetLog() << "Time= "<< time <<" bodies= "<< mphysicalSystem.GetNbodies() << " contacts= " << mphysicalSystem.GetNcontacts() << "\n";
			OutputSphere(sph_file, mphysicalSystem, time);
			out_time += out_step;
		}

		mphysicalSystem.DoStepDynamics(time_step);
		time += time_step;
	}
	
	DLL_DeleteGlobals();

	// Note: we do not need to delete mysolver and mycontainer since these will be
	// freed in the ChSystem destructor.

	return 0;
}

