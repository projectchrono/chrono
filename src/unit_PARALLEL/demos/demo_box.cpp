#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"
#include "utils/creators.h"

using namespace chrono;
using namespace geometry;

//// Comment this for DVI contact
#define DEM


void OutputFile(ChStreamOutAsciiFile& file,
                ChSystem*             sys,
                double                time)
{
	file << time << "     ";
	std::cout << time << "     ";

	for (int i = 0; i < sys->Get_bodylist()->size(); ++i) {
		ChBody* abody = (ChBody*) sys->Get_bodylist()->at(i);
#ifdef DEM
		assert(typeid(*abody) == typeid(ChBodyDEM));
#endif
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
	double time_step = 0.0005;
	double time_end = 0.5;
	double out_fps = 100;

	int max_iteration = 20;

	// Output
#ifdef DEM
	ChStreamOutAsciiFile sph_file("../TEST_BOX/box_pos_DEM.txt");
	const char* out_folder = "../TEST_BOX/POVRAY_DEM";
#else
	ChStreamOutAsciiFile sph_file("../TEST_BOX/box_pos_DVI.txt");
	const char* out_folder = "../TEST_BOX/POVRAY_DVI";
#endif

	// Parameters for the falling box
	int             boxId = 100;
	ChVector<>      hdims(0.3, 0.2, 0.1);
	double          density = 2000;
	double          volume = 8 * hdims.x * hdims.y * hdims.z;
	double          mass = density * volume;
	ChVector<>      inertia = mass/12 * ChVector<>(hdims.y * hdims.y + hdims.z * hdims.z,
	                                               hdims.x * hdims.x + hdims.z * hdims.z,
	                                               hdims.x * hdims.x + hdims.y * hdims.y);
	ChVector<>      initPos(1, -1, 0.5);
	ChVector<>      initVel(0,0,0);
	ChQuaternion<>  initRot(1,0,0,0);
	initRot.Q_from_AngAxis(PI/6, ChVector<>(1, 0, 0));

	// Parameters for the containing bin
	int    binId = -200;
	double hDimX = 5;          // length in x direction
	double hDimY = 2;          // depth in y direction
	double hDimZ = 0.5;        // height in z direction
	double hThickness = 0.1;   // wall thickness

	// Create system
#ifdef DEM
	ChSystemParallelDEM* msystem = new ChSystemParallelDEM();
#else
	ChSystemParallelDVI* msystem = new ChSystemParallelDVI();
#endif

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

#ifdef DEM
	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetTolerance(0);
	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetContactRecoverySpeed(1);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBodyPerBin(100, 50);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);
#else
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetTolerance(0);
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetCompliance(0);
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetContactRecoverySpeed(1);
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);

	//((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->SetCollisionEnvelope(0.01);
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBodyPerBin(100, 50);
#endif

	// Create the falling object
#ifdef DEM
	ChSharedPtr<ChMaterialSurfaceDEM> boxMat(new ChMaterialSurfaceDEM);
	boxMat->SetYoungModulus(2e5f);
	boxMat->SetFriction(0.4f);
	boxMat->SetDissipationFactor(0.6f);

	ChSharedBodyDEMPtr box(new ChBodyDEM(new ChCollisionModelParallel));
	box->SetMaterialSurfaceDEM(boxMat);
#else
	ChSharedPtr<ChMaterialSurface> boxMat(new ChMaterialSurface);
	boxMat->SetFriction(0.4f);

	ChSharedBodyPtr box(new ChBody(new ChCollisionModelParallel));
	box->SetMaterialSurface(boxMat);
#endif

	box->SetIdentifier(boxId);
	box->SetMass(mass);
	box->SetInertiaXX(inertia);
	box->SetPos(initPos);
	box->SetRot(initRot);
	box->SetPos_dt(initVel);
	box->SetBodyFixed(false);
	box->SetCollide(true);

	box->GetCollisionModel()->ClearModel();
	utils::AddBoxGeometry(box.get_ptr(), hdims);
	box->GetCollisionModel()->BuildModel();

	msystem->AddBody(box);


	// Create the containing bin
#ifdef DEM
	ChSharedPtr<ChMaterialSurfaceDEM> binMat(new ChMaterialSurfaceDEM);
	binMat->SetYoungModulus(2e5f);
	binMat->SetFriction(0.4f);
	binMat->SetDissipationFactor(0.6f);

	utils::CreateBoxContainerDEM(msystem, binId, binMat, ChVector<>(hDimX, hDimY, hDimZ), hThickness);
#else
	ChSharedPtr<ChMaterialSurface> binMat(new ChMaterialSurface);
	binMat->SetFriction(0.4f);

	utils::CreateBoxContainerDVI(msystem, binId, binMat, ChVector<>(hDimX, hDimY, hDimZ), hThickness);
#endif


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

