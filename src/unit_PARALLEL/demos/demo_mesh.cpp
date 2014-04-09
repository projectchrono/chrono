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
	double time_step = 0.001;
	double time_end = 4.0;
	double out_fps = 25;

	int max_iteration = 20;

	// Input / Output
	const std::string  obj_mesh_file("../TEST_MESH/tetrahedron.obj");
	const std::string  mesh_name("tetrahedron");
	const std::string  pov_mesh_file("../TEST_MESH/tetrahedron.inc");
	const char* pov_out_folder = "../TEST_MESH/POVRAY";

	// Parameters for the falling body
	int             bodyId = 100;
	double          mass = 30;
	ChVector<>      inertia = ChVector<>(0.2, 0.2, 0.2);
	ChVector<>      initPos(1, -1, 4);
	ChVector<>      initVel(0,0,0);
	ChQuaternion<>  initRot(1,0,0,0);
	//initRot.Q_from_AngAxis(PI/3, ChVector<>(1, 0, 0));

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

	// -------------------------
	// Create the falling object
	// -------------------------

	ChSharedPtr<ChMaterialSurfaceDEM> bodyMat(new ChMaterialSurfaceDEM);
	bodyMat->SetYoungModulus(1e7f);
	bodyMat->SetFriction(0.4f);
	bodyMat->SetDissipationFactor(0.1f);

	ChSharedBodyDEMPtr body(new ChBodyDEM(new ChCollisionModelParallel));
	body->SetMaterialSurfaceDEM(bodyMat);
	body->SetIdentifier(bodyId);
	body->SetMass(mass);
	body->SetInertiaXX(inertia);
	body->SetPos(initPos);
	body->SetRot(initRot);
	body->SetPos_dt(initVel);
	body->SetBodyFixed(false);
	body->SetCollide(true);

	body->GetCollisionModel()->ClearModel();
	utils::AddTriangleMeshGeometry(body.get_ptr(), obj_mesh_file, mesh_name);
	body->GetCollisionModel()->BuildModel();

	body->SetInertiaXX(inertia);

	msystem->AddBody(body);

	utils::WriteMeshPovray(obj_mesh_file, mesh_name, pov_mesh_file);

	// -------------------------
	// Create the containing bin
	// -------------------------

	ChSharedPtr<ChMaterialSurfaceDEM> binMat(new ChMaterialSurfaceDEM);
	binMat->SetYoungModulus(1e7f);
	binMat->SetFriction(0.4f);
	binMat->SetDissipationFactor(0.1f);

	int binId = -200;

/*
	double hDimX = 5;          // length in x direction
	double hDimY = 2;          // depth in y direction
	double hDimZ = 0.5;        // height in z direction
	double hThickness = 0.1;   // wall thickness
	utils::CreateBoxContainerDEM(msystem, binId, binMat, ChVector<>(hDimX, hDimY, hDimZ), hThickness);
*/

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

/*
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
	rot.Q_from_AngAxis(PI/6, ChVector<>(0, 0, 1));

	bin->GetCollisionModel()->ClearModel();
	for (int ix = -3; ix < 6; ix++) {
		ChVector<> pos(ix * spacing, 0, offsetZ);
		utils::AddCapsuleGeometry(bin.get_ptr(), bigR, bigH, pos, rot);
	}
	bin->GetCollisionModel()->BuildModel();

	msystem->AddBody(bin);
*/

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

