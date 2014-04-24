#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"
#include "utils/creators.h"

using namespace chrono;
using namespace geometry;


int main(int argc, char* argv[])
{
	int threads = 8;

	// Simulation parameters
	double gravity = 9.81;
	double time_step = 0.0001;
	double time_end = 10.0;
	double out_fps = 30;

	int max_iteration = 20;

	// Input / Output
	const std::string  obj_mesh_file("../TEST_MESH/box3d.obj");
	const std::string  mesh_name("box3d");
	const std::string  pov_mesh_file("../TEST_MESH/box3d.inc");
	const char* pov_out_folder = "../TEST_MESH/POVRAY_DEM";

	// Parameters for the falling body
	ChVector<>      hdims(1, 1, 1);
	double          density = 1000;
	double          volume = 8 * hdims.x * hdims.y * hdims.z;
	double          mass = density * volume;
	ChVector<>      inertia = mass/12 * ChVector<>(hdims.y * hdims.y + hdims.z * hdims.z,
	                                               hdims.x * hdims.x + hdims.z * hdims.z,
	                                               hdims.x * hdims.x + hdims.y * hdims.y);

	ChVector<>      initPos1( 1.5,  1.5, 4);
	ChVector<>      initPos2(-1.5, -1.5, 4);

	ChVector<>      initVel1(0,0,0);
	ChVector<>      initVel2(0,0,0);

	ChQuaternion<>  initRot1(1,0,0,0);
	initRot1.Q_from_AngAxis(PI/4, ChVector<>(1/sqrt(2.0), 1/sqrt(2.0), 0));

	ChQuaternion<>  initRot2(1,0,0,0);
	initRot2.Q_from_AngAxis(PI/4, ChVector<>(-1/sqrt(2.0), -1/sqrt(2.0), 0));

	// -------------
	// Create system
	// -------------

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

	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetTolerance(0);
	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetContactRecoverySpeed(1);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBodyPerBin(100, 50);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);

	// --------------------------
	// Create the falling objects
	// --------------------------

	ChSharedPtr<ChMaterialSurfaceDEM> bodyMat(new ChMaterialSurfaceDEM);
	bodyMat->SetYoungModulus(1e8f);
	bodyMat->SetFriction(0.5f);
	bodyMat->SetDissipationFactor(0.4f);

	ChSharedBodyDEMPtr body1(new ChBodyDEM(new ChCollisionModelParallel));
	body1->SetMaterialSurfaceDEM(bodyMat);
	body1->SetIdentifier(101);
	body1->SetMass(mass);
	body1->SetInertiaXX(inertia);
	body1->SetPos(initPos1);
	body1->SetRot(initRot1);
	body1->SetPos_dt(initVel1);
	body1->SetBodyFixed(false);
	body1->SetCollide(true);

	body1->GetCollisionModel()->ClearModel();
	utils::AddTriangleMeshGeometry(body1.get_ptr(), obj_mesh_file, mesh_name);
	body1->GetCollisionModel()->BuildModel();

	body1->SetInertiaXX(inertia);

	msystem->AddBody(body1);

	utils::WriteMeshPovray(obj_mesh_file, mesh_name, pov_mesh_file);


	ChSharedBodyDEMPtr body2(new ChBodyDEM(new ChCollisionModelParallel));
	body2->SetMaterialSurfaceDEM(bodyMat);
	body2->SetIdentifier(102);
	body2->SetMass(mass);
	body2->SetInertiaXX(inertia);
	body2->SetPos(initPos2);
	body2->SetRot(initRot2);
	body2->SetPos_dt(initVel2);
	body2->SetBodyFixed(false);
	body2->SetCollide(true);

	body2->GetCollisionModel()->ClearModel();
	utils::AddBoxGeometry(body2.get_ptr(), hdims);
	body2->GetCollisionModel()->BuildModel();

	body2->SetInertiaXX(inertia);

	msystem->AddBody(body2);


	// -------------------------
	// Create the fixed objects
	// -------------------------

	ChSharedPtr<ChMaterialSurfaceDEM> binMat(new ChMaterialSurfaceDEM);
	binMat->SetYoungModulus(1e8f);
	binMat->SetFriction(0.4f);
	binMat->SetDissipationFactor(0.1f);

	// A set of fixed spheres
	ChSharedBodyDEMPtr bin(new ChBodyDEM(new ChCollisionModelParallel));
	bin->SetMaterialSurfaceDEM(binMat);
	bin->SetIdentifier(-100);
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

	// ----------------------
	// Perform the simulation
	// ----------------------

	int num_steps = std::ceil(time_end / time_step);
	int out_steps = std::ceil((1 / time_step) / out_fps);

	double time = 0;
	int out_frame = 0;
	char filename[100];

	for (int i = 0; i < num_steps; i++) {

		if (i % out_steps == 0) {
			sprintf(filename, "%s/data_%03d.dat", pov_out_folder, out_frame);
			utils::WriteShapesPovray(msystem, filename);
			out_frame++;
		}

		msystem->DoStepDynamics(time_step);
		time += time_step;
	}

	return 0;
}

