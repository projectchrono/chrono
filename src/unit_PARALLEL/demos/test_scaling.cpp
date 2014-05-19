#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"
#include "utils/generators.h"

using namespace chrono;

using std::cout;
using std::endl;


// =======================================================================
// Global problem definitions

int threads = 8;

// Simulation parameters
double gravity = -9.81;
double time_step = 0.0001;
double measure_interval = 0.2;
double time_end = 2;

int max_iteration = 20;

// Parameters for the mixture balls
double     radius = 0.1;
double     density = 2000;
double     vol = (4.0/3) * CH_C_PI * radius * radius * radius;
double     mass = density * vol;
ChVector<> inertia = 0.4 * mass * radius * radius * ChVector<>(1,1,1);

// Parameters for the containing bin
int    binId = -200;
double hDimX = 2;          // length in x direction
double hDimY = 2;          // depth in y direction
double hDimZ = 5;          // height in z direction
double hThickness = 0.1;   // wall thickness

// =======================================================================

void CreateObjects(ChSystemParallel* system)
{
	// Create a material for the ball mixture
	ChSharedPtr<ChMaterialSurfaceDEM> ballMixMat;
	ballMixMat = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	ballMixMat->SetYoungModulus(1e8f);
	ballMixMat->SetFriction(0.4f);
	ballMixMat->SetDissipationFactor(0.1f);

	// Create a mixture entirely made out of spheres
	utils::Generator gen(system);

	utils::MixtureIngredientPtr& m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
	m1->setDefaultMaterialDEM(ballMixMat);
	m1->setDefaultDensity(density);
	m1->setDefaultSize(radius);

	// Generate the objects
	gen.createObjectsBox(utils::POISSON_DISK,
	                     2.01 * radius,
	                     ChVector<>(0, 0, 2.5),
	                     ChVector<>(0.8 * hDimX, 0.8 * hDimY, 2));

	cout << "Number bodies generated: " << gen.getTotalNumBodies() << endl;
}

// =======================================================================


int main(int argc, char* argv[])
{

	// Create system
	ChSystemParallelDEM* msystem = new ChSystemParallelDEM();

	// Set number of threads.
	int max_threads = msystem->GetParallelThreadNumber();
	if (threads > max_threads)
		threads = max_threads;
	msystem->SetParallelThreadNumber(threads);
	omp_set_num_threads(threads);

	// Set gravitational acceleration
	msystem->Set_G_acc(ChVector<>(0, 0, gravity));

	// Edit system settings
	msystem->SetMaxiter(max_iteration);
	msystem->SetIterLCPmaxItersSpeed(max_iteration);
	msystem->SetTol(1e-3);
	msystem->SetTolSpeeds(1e-3);
	msystem->SetStep(time_step);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBodyPerBin(100, 50);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);

	// Create container bin.
	ChSharedPtr<ChMaterialSurfaceDEM> binMat;
	binMat = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	binMat->SetYoungModulus(2e6f);
	binMat->SetFriction(0.4f);
	binMat->SetDissipationFactor(0.6f);

	utils::CreateBoxContainerDEM(msystem, binId, binMat, ChVector<>(hDimX, hDimY, hDimZ), hThickness);

	// Create the granular material
	CreateObjects(msystem);


	// Number of steps
	int num_frames1 = std::ceil(measure_interval / time_step);
	int num_frames2 = std::ceil((time_end - 2 * measure_interval) / time_step);

	// Perform the simulation
	double timeA = 0;
	int ncA = 0;

	for (int i = 0; i < num_frames1; i++) {
		msystem->DoStepDynamics(time_step);
		timeA += msystem->GetTimerStep();
		ncA += msystem->GetNcontacts();
	}

	for (int i = 0; i < num_frames2; i++) {
		msystem->DoStepDynamics(time_step);
	}

	double timeB = 0;
	int ncB = 0;

	for (int i = 0; i < num_frames1; i++) {
		msystem->DoStepDynamics(time_step);
		timeB += msystem->GetTimerStep();
		ncB += msystem->GetNcontacts();
	}

	// Final stats
	cout << "==================================" << endl;
	cout << "Number of threads: " << threads << endl;
	cout << "Number of bodies: " << msystem->Get_bodylist()->size() << endl;
	cout << "Number of frames: " << num_frames1 << endl;
	cout << "Simulation time A: " << timeA << endl;
	cout << "Average num. contacts A: " << double(ncA) / num_frames1 << endl;
	cout << "Simulation time B: " << timeB << endl;
	cout << "Average num. contacts B: " << double(ncB) / num_frames1 << endl;


	msystem->gpu_data_manager->system_timer.PrintReport();

	return 0;
}

