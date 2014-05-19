#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"
#include "utils/creators.h"

using namespace chrono;
using namespace geometry;


void OutputFile(ChStreamOutAsciiFile& file,
                ChSystem*             sys,
                double                time)
{
	file << time << "     ";
	std::cout << time << "     ";

	for (int i = 0; i < sys->Get_bodylist()->size(); ++i) {
		ChBody* abody = (ChBody*) sys->Get_bodylist()->at(i);
		assert(typeid(*abody) == typeid(ChBodyDEM));

		if (abody->IsActive()) {
			const ChVector<>& bodypos = abody->GetPos();
			ChQuaternion<>& bodyRot = abody->GetRot();
			ChVector<> bodyAngs = bodyRot.Q_to_NasaAngles();
			file << bodypos.x  << "  " << bodypos.y  << "  " << bodypos.z  << "     ";
			file << bodyRot.e0 << "  " << bodyRot.e1 << "  " << bodyRot.e2 << "  " << bodyRot.e3 << "     ";
			file << bodyAngs.x << "  " << bodyAngs.y << "  " << bodyAngs.z << "       ";
			std::cout << bodypos.x << "  " << bodypos.y << "  " << bodypos.z << "      ";
		}
	}

	file << "\n";
	std::cout << std::endl;
}

int main(int argc, char* argv[])
{
	int threads = 8;

	// Simulation parameters
	double gravity = 9.81;
	double time_step = 0.001;
	double time_end = 4.0;
	double out_fps = 50;

	int max_iteration = 20;

	// Output
	ChStreamOutAsciiFile sph_file("../TEST_CAPSULE/capsule_pos_DEM.txt");
	const char* out_folder = "../TEST_CAPSULE/POVRAY_DEM";

	// Parameters for the falling box
	int             capsuleId = 100;
	double          radius = 0.1;
	double          hlen = 0.2;
	double          density = 2000;
	double          volume = 2 * CH_C_PI * radius * radius * (hlen + 2 * radius / 3);
	double          mass = density * volume;
	double          hlen1 = radius + hlen;
	ChVector<>      inertia = mass * ChVector<>((3 * radius * radius + hlen * hlen) / 12,
	                                            radius * radius / 2,
	                                            (3 * radius * radius + hlen * hlen) / 12);
	ChVector<>      initPos(1, -1, 4);
	ChVector<>      initVel(0,0,0);
	ChQuaternion<>  initRot(1,0,0,0);
	initRot.Q_from_AngAxis(CH_C_PI/3, ChVector<>(1, 0, 0));

	// Parameters for the containing bin
	int    binId = -200;
	double hDimX = 5;          // length in x direction
	double hDimY = 2;          // depth in y direction
	double hDimZ = 0.5;        // height in z direction
	double hThickness = 0.1;   // wall thickness

	// Create system
	ChSystemParallelDEM* msystem = new ChSystemParallelDEM();

	// Set number of threads.
	int max_threads = msystem->GetParallelThreadNumber();
	if (threads > max_threads)
		threads = max_threads;
	msystem->SetParallelThreadNumber(threads);
	omp_set_num_threads(threads);

	// Set gravitational acceleration
	msystem->Set_G_acc(ChVector<>(0, 0, -gravity));

	// Edit system settings
	msystem->SetMaxiter(max_iteration);
	msystem->SetIterLCPmaxItersSpeed(max_iteration);
	msystem->SetTol(1e-3);
	msystem->SetTolSpeeds(1e-3);
	msystem->SetStep(time_step);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBodyPerBin(100, 50);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);


	// Create the falling object
	ChSharedPtr<ChMaterialSurfaceDEM> capsuleMat(new ChMaterialSurfaceDEM);
	capsuleMat->SetYoungModulus(1e7f);
	capsuleMat->SetFriction(0.4f);
	capsuleMat->SetDissipationFactor(0.1f);

	ChSharedBodyDEMPtr capsule(new ChBodyDEM(new ChCollisionModelParallel));
	capsule->SetMaterialSurfaceDEM(capsuleMat);


	capsule->SetIdentifier(capsuleId);
	capsule->SetMass(mass);
	capsule->SetInertiaXX(inertia);
	capsule->SetPos(initPos);
	capsule->SetRot(initRot);
	capsule->SetPos_dt(initVel);
	capsule->SetBodyFixed(false);

	capsule->SetCollide(true);

	capsule->GetCollisionModel()->ClearModel();
	utils::AddCapsuleGeometry(capsule.get_ptr(), radius, hlen);
	capsule->GetCollisionModel()->BuildModel();

	msystem->AddBody(capsule);


	// Create the containing bin
	ChSharedPtr<ChMaterialSurfaceDEM> binMat(new ChMaterialSurfaceDEM);
	binMat->SetYoungModulus(1e7f);
	binMat->SetFriction(0.4f);
	binMat->SetDissipationFactor(0.1f);

/*
	utils::CreateBoxContainerDEM(msystem, binId, binMat, ChVector<>(hDimX, hDimY, hDimZ), hThickness);
*/

/*
	// A set of fixed spheres
	ChSharedBodyDEMPtr bin(new ChBodyDEM(new ChCollisionModelParallel));
	bin->SetMaterialSurfaceDEM(binMat);
	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0,0,0));
	bin->SetRot(ChQuaternion<>(1,0,0,0));
	bin->SetBodyFixed(true);
	bin->SetCollide(true);

	double spacing = 1.6;
	double bigR = 2;
	double offsetZ = -1;
	bin->GetCollisionModel()->ClearModel();
	for (int ix = -2; ix < 3; ix++) {
		for (int iy = -2; iy < 3; iy++) {
			ChVector<> pos(ix * spacing, iy * spacing, offsetZ);
			utils::AddSphereGeometry(bin.get_ptr(), bigR, pos);
		}
	}
	bin->GetCollisionModel()->BuildModel();

	msystem->AddBody(bin);
*/

	// A set of fixed capsules
	ChSharedBodyDEMPtr bin(new ChBodyDEM(new ChCollisionModelParallel));
	bin->SetMaterialSurfaceDEM(binMat);
	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0,0,0));
	bin->SetRot(ChQuaternion<>(1,0,0,0));
	bin->SetBodyFixed(true);
	bin->SetCollide(true);

	double spacing = 1.5;
	double bigR = 1;
	double bigH = 6;
	double offsetZ = -1;

	ChQuaternion<>  rot(1,0,0,0);
	rot.Q_from_AngAxis(CH_C_PI/6, ChVector<>(0, 0, 1));

	bin->GetCollisionModel()->ClearModel();
	for (int ix = -3; ix < 6; ix++) {
		ChVector<> pos(ix * spacing, 0, offsetZ);
		utils::AddCapsuleGeometry(bin.get_ptr(), bigR, bigH, pos, rot);
	}
	bin->GetCollisionModel()->BuildModel();

	msystem->AddBody(bin);



	// Perform the simulation
	int num_steps = std::ceil(time_end / time_step);
	int out_steps = std::ceil((1 / time_step) / out_fps);

	double time = 0;
	int out_frame = 0;
	char filename[100];

	for (int i = 0; i < num_steps; i++) {

		if (i % out_steps == 0) {
			sprintf(filename, "%s/data_%03d.dat", out_folder, out_frame);
			utils::WriteShapesPovray(msystem, filename);

			OutputFile(sph_file, msystem, time);

			out_frame++;
		}

		msystem->DoStepDynamics(time_step);
		time += time_step;
	}

	return 0;
}

