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
// Initialize the random engine
std::random_device rd;
std::default_random_engine utils::rengine(rd());


// =======================================================================
// Global problem definitions

int threads = 8;

// Simulation parameters
double gravity = 9.81;
double time_step = 1e-4;
double time_end = 5;

int max_iteration = 20;

// Output
const char* out_folder = "../FOAM/POVRAY";
double out_fps = 50;

// Parameters for the granular material
int        Id_g = 1;
double     r_g = 0.05;
double     rho_g = 2000;

float      Y_g = 5e6;
float      mu_g = 0.3;
float      alpha_g = 0.4;
float      cohesion_g = 300;

// Parameters for the containing bin
int        binId = -200;
double     hDimX = 10;           // length in x direction
double     hDimY = 10;           // depth in y direction
double     hDimZ = 1;            // height in z direction
double     hThickness = 0.4;     // wall thickness

float      Y_c = 2e6;
float      mu_c = 0.3;
float      alpha_c = 0.6;
float      cohesion_c = 5;

// Particle generator
utils::Generator*  gen;

double     initVel = 5;         // initial particle velocity in negative X direction


// =======================================================================

int SpawnParticles()
{
	double dist = 2 * 0.99 * r_g;

	////gen->createObjectsBox(utils::POISSON_DISK,
	////                     dist,
	////                     ChVector<>(9, 0, 3),
	////                     ChVector<>(0, 1, 0.5),
	////                     ChVector<>(-initVel, 0, 0));
	gen->createObjectsCylinderX(utils::POISSON_DISK,
	                     dist,
	                     ChVector<>(9, 0, 3),
	                     0.5, 0,
	                     ChVector<>(-initVel, 0, 0));
	cout << "  total bodies: " << gen->getTotalNumBodies() << endl;

	return gen->getTotalNumBodies();
}


// ========================================================================
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

	// Create a material for the granular material
	ChSharedPtr<ChMaterialSurfaceDEM> mat_g;
	mat_g = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	mat_g->SetYoungModulus(Y_g);
	mat_g->SetFriction(mu_g);
	mat_g->SetDissipationFactor(alpha_g);
	mat_g->SetCohesion(cohesion_g);

	// Create a material for the container
	ChSharedPtr<ChMaterialSurfaceDEM> mat_c;
	mat_c = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	mat_c->SetYoungModulus(Y_c);
	mat_c->SetFriction(mu_c);
	mat_c->SetDissipationFactor(alpha_c);
	mat_c->SetCohesion(cohesion_c);

	// Create the containing bin
	utils::CreateBoxContainerDEM(msystem, binId, mat_c, ChVector<>(hDimX, hDimY, hDimZ), hThickness);

	// Create a mixture entirely made out of spheres
	double     vol_g = (4.0/3) * PI * r_g * r_g * r_g;
	double     mass_g = rho_g * vol_g;
	ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1,1,1);

	gen = new utils::Generator(msystem);

	utils::MixtureIngredientPtr& m1 = gen->AddMixtureIngredient(utils::SPHERE, 1.0);
	m1->setDefaultMaterialDEM(mat_g);
	m1->setDefaultDensity(rho_g);
	m1->setDefaultSize(r_g);

	gen->setBodyIdentifier(Id_g);

	// Number of steps
	int num_steps = std::ceil(time_end / time_step);
	int out_steps = std::ceil((1 / time_step) / out_fps);
	int gen_steps = std::ceil(2 * r_g / initVel / time_step);

	// Perform the simulation
	double time = 0;
	int sim_frame = 0;
	int out_frame = 0;
	double exec_time = 0;

	while (time < time_end) {
		if (sim_frame % gen_steps == 0) {
			SpawnParticles();
		}

		if (sim_frame % out_steps == 0) {
			char filename[100];
			sprintf(filename, "%s/data_%03d.dat", out_folder, out_frame);
			utils::WriteShapesPovray(msystem, filename);

			cout << " --------------------------------- Output frame:   " << out_frame << endl;
			cout << "                                   Sim frame:      " << sim_frame << endl;
			cout << "                                   Time:           " << time << endl;
			cout << "                                   Execution time: " << exec_time << endl;
			cout << "                                   Num. bodies:    " << msystem->Get_bodylist()->size() << endl;

			out_frame++;
		}

		msystem->DoStepDynamics(time_step);

		time += time_step;
		sim_frame++;
		exec_time += msystem->mtimer_step();
	}

	// Final stats
	cout << "==================================" << endl;
	cout << "Number of bodies: " << msystem->Get_bodylist()->size() << endl;
	cout << "Simulation time: " << exec_time << endl;
	cout << "Number of threads: " << threads << endl;

	return 0;
}

