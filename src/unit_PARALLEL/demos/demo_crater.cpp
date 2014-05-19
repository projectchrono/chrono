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
enum ProblemType {
	SETTLING,
	DROPPING
};

ProblemType problem = DROPPING;

// =======================================================================
// Global problem definitions

int threads = 8;

// Simulation parameters
double gravity = 9.81;
double time_step = 1e-5;
double time_settling_min = 0.1;
double time_settling_max = 0.8;
double time_dropping = 0.06;

int max_iteration = 20;

// Output
const char* out_folder = "../CRATER/POVRAY";
const char* height_file = "../CRATER/height.dat";
const char* checkpoint_file = "../CRATER/settled.dat";
double out_fps = 1200;

// Parameters for the granular material
int        Id_g = 1;
double     r_g = 1e-3 / 2;
double     rho_g = 2500;
double     vol_g = (4.0/3) * CH_C_PI * r_g * r_g * r_g;
double     mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1,1,1);

float      Y_g = 1e8;
float      mu_g = 0.3;
float      alpha_g = 0.1;

// Parameters for the falling ball
int        Id_b = 0;
double     R_b = 2.54e-2 / 2;
double     rho_b = 700;
double     vol_b = (4.0/3) * CH_C_PI * R_b * R_b * R_b;
double     mass_b = rho_b * vol_b;
ChVector<> inertia_b = 0.4 * mass_b * R_b * R_b * ChVector<>(1,1,1);

float      Y_b = 1e8;
float      mu_b = 0.3;
float      alpha_b = 0.1;

// Parameters for the containing bin
int        binId = -200;
double     hDimX = 4e-2;            // length in x direction
double     hDimY = 4e-2;            // depth in y direction
double     hDimZ = 7.5e-2;          // height in z direction
double     hThickness = 0.5e-2;     // wall thickness

float      Y_c = 2e6;
float      mu_c = 0.3;
float      alpha_c = 0.6;

// Number of layers and height of one layer for generator domain
int        numLayers = 10;
double     layerHeight = 1e-2;

// Drop height (above surface of settled granular material)
double h = 5e-2;

// =======================================================================

void AddWall(ChSharedBodyDEMPtr&  body,
             const ChVector<>&    loc,
             const ChVector<>&    dim)
{
	// Append to collision geometry
	body->GetCollisionModel()->AddBox(dim.x, dim.y, dim.z, loc);

	// Append to assets
	ChSharedPtr<ChBoxShape> box_shape = ChSharedPtr<ChAsset>(new ChBoxShape);
	box_shape->SetColor(ChColor(1, 0, 0));
	box_shape->Pos = loc;
	box_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
	box_shape->GetBoxGeometry().Size = dim;

	body->GetAssets().push_back(box_shape);
}

int CreateObjects(ChSystemParallel* system)
{
	// Create a material for the granular material
	ChSharedPtr<ChMaterialSurfaceDEM> mat_g;
	mat_g = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	mat_g->SetYoungModulus(Y_g);
	mat_g->SetFriction(mu_g);
	mat_g->SetDissipationFactor(alpha_g);

	// Create a material for the container
	ChSharedPtr<ChMaterialSurfaceDEM> mat_c;
	mat_c = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	mat_c->SetYoungModulus(Y_c);
	mat_c->SetFriction(mu_c);
	mat_c->SetDissipationFactor(alpha_c);

	// Create a mixture entirely made out of spheres
	utils::Generator gen(system);

	utils::MixtureIngredientPtr& m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
	m1->setDefaultMaterialDEM(mat_g);
	m1->setDefaultDensity(rho_g);
	m1->setDefaultSize(r_g);

	gen.setBodyIdentifier(Id_g);

	double r = 1.01 * r_g;

	for (int i = 0; i < numLayers; i++) {
		double center = r + layerHeight / 2 + i * (2 * r + layerHeight);
		gen.createObjectsBox(utils::POISSON_DISK,
		                     2 * r,
		                     ChVector<>(0, 0, center),
		                     ChVector<>(hDimX - r, hDimY - r, layerHeight/2));
		cout << "Layer " << i << "  total bodies: " << gen.getTotalNumBodies() << endl;
	}

	// Create the containing bin
	ChSharedBodyDEMPtr bin(new ChBodyDEM(new ChCollisionModelParallel));

	bin->SetMaterialSurfaceDEM(mat_c);

	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0, 0, 0));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);

	bin->GetCollisionModel()->ClearModel();
	AddWall(bin, ChVector<>(0, 0, -hThickness), ChVector<>(hDimX, hDimY, hThickness));
	AddWall(bin, ChVector<>(-hDimX-hThickness, 0, hDimZ), ChVector<>(hThickness, hDimY, hDimZ));
	AddWall(bin, ChVector<>( hDimX+hThickness, 0, hDimZ), ChVector<>(hThickness, hDimY, hDimZ));
	AddWall(bin, ChVector<>(0, -hDimY-hThickness, hDimZ), ChVector<>(hDimX, hThickness, hDimZ));
	AddWall(bin, ChVector<>(0,  hDimY+hThickness, hDimZ), ChVector<>(hDimX, hThickness, hDimZ));
	bin->GetCollisionModel()->BuildModel();

	system->AddBody(bin);

	return gen.getTotalNumBodies();
}


// =======================================================================
// Create the falling ball such that its bottom point is at the specified
// height and its downward initial velocity has the specified magnitude.

ChSharedBodyDEMPtr CreateFallingBall(ChSystemParallel* system, double z, double vz)
{
	// Create a material for the falling ball
	ChSharedPtr<ChMaterialSurfaceDEM> mat_b;
	mat_b = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	mat_b->SetYoungModulus(1e8f);
	mat_b->SetFriction(0.4f);
	mat_b->SetDissipationFactor(0.1f);

	// Create the falling ball, but do not add it to the system
	ChSharedBodyDEMPtr ball(new ChBodyDEM(new ChCollisionModelParallel));

	ball->SetMaterialSurfaceDEM(mat_b);

	ball->SetIdentifier(Id_b);
	ball->SetMass(mass_b);
	ball->SetInertiaXX(inertia_b);
	ball->SetPos(ChVector<>(0, 0, z + r_g + R_b));
	ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
	ball->SetPos_dt(ChVector<>(0, 0, -vz));
	ball->SetCollide(true);
	ball->SetBodyFixed(false);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(R_b);
	ball->GetCollisionModel()->BuildModel();

	ChSharedPtr<ChSphereShape> ball_shape = ChSharedPtr<ChAsset>(new ChSphereShape);
	ball_shape->SetColor(ChColor(0, 0, 1));
	ball_shape->GetSphereGeometry().rad = R_b;
	ball_shape->Pos = ChVector<>(0, 0, 0);
	ball_shape->Rot = ChQuaternion<>(1, 0, 0, 0);
	ball->GetAssets().push_back(ball_shape);

	system->AddBody(ball);

	return ball;
}


// ========================================================================
// These utility functions find the height of the highest or lowest sphere
// in the granular mix, respectively.  We only look at bodies whith stricty
// positive identifiers.

double FindHighest(ChSystem* sys)
{
	double highest = 0;
	for (int i = 0; i < sys->Get_bodylist()->size(); ++i) {
		ChBody* body = (ChBody*) sys->Get_bodylist()->at(i);
		if (body->GetIdentifier() > 0 && body->GetPos().z > highest)
			highest = body->GetPos().z;
	}
	return highest;
}

double FindLowest(ChSystem* sys)
{
	double lowest = 1000;
	for (int i = 0; i < sys->Get_bodylist()->size(); ++i) {
		ChBody* body = (ChBody*) sys->Get_bodylist()->at(i);
		if (body->GetIdentifier() > 0 && body->GetPos().z < lowest)
			lowest = body->GetPos().z;
	}
	return lowest;
}

// ========================================================================
// This utility function returns true if all bodies in the granular mix
// have a linear velocity whose magnitude is below the specified value.

bool CheckSettled(ChSystem* sys, double threshold)
{
	double t2 = threshold * threshold;
	for (int i = 0; i < sys->Get_bodylist()->size(); ++i) {
		ChBody* body = (ChBody*) sys->Get_bodylist()->at(i);
		if (body->GetIdentifier() > 0) {
			double vel2 = body->GetPos_dt().Length2();
			if (vel2 > t2)
				return false;
		}
	}

	return true;
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

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBodyPerBin(100, 50);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);


	// Depending on problem type:
	// - Select end simulation times
	// - Create granular material and container
	// - Create falling ball
	double time_end;
	ChSharedBodyDEMPtr ball;

	if (problem == SETTLING) {
		time_end = time_settling_max;

		// Create containing bin and the granular material at randomized initial positions
		CreateObjects(msystem);
	} else {
		time_end = time_dropping;

		// Create the granular material bodies and the container from the checkpoint file.
		cout << "Read checkpoint data from " << checkpoint_file;
		utils::ReadCheckpoint(msystem, checkpoint_file);
		cout << "  done.  Read " << msystem->Get_bodylist()->size() << " bodies." << endl;

		// Create the falling ball just above the granular material with a velocity
		// given by free fall from the specified height and starting at rest.
		double z = FindHighest(msystem);
		double vz = std::sqrt(2 * gravity * h);
		cout << "Create falling ball with center at" << z + R_b + r_g << " and velocity " << vz << endl;
		ball = CreateFallingBall(msystem, z, vz);
	}

	// Number of steps
	int num_steps = std::ceil(time_end / time_step);
	int out_steps = std::ceil((1 / time_step) / out_fps);

	// Zero velocity level for settling check (fraction of a grain radius per second)
	double zero_v = 0.1 * r_g;

	// Perform the simulation
	double time = 0;
	int sim_frame = 0;
	int out_frame = 0;
	int next_out_frame = 0;
	double exec_time = 0;
	ChStreamOutAsciiFile hfile(height_file);

	while (time < time_end) {
		if (sim_frame == next_out_frame) {
			char filename[100];
			sprintf(filename, "%s/data_%03d.dat", out_folder, out_frame);
			utils::WriteShapesPovray(msystem, filename);

			cout << " --------------------------------- Output frame:   " << out_frame << endl;
			cout << "                                   Sim frame:      " << sim_frame << endl;
			cout << "                                   Time:           " << time << endl;
			cout << "                                   Lowest point:   " << FindLowest(msystem) << endl;
			cout << "                                   Execution time: " << exec_time << endl;

			if (problem == DROPPING) {
				hfile << time << "  " << ball->GetPos().z << "\n";
				cout << "                                   Ball height:    " << ball->GetPos().z << endl;
			}

			out_frame++;
			next_out_frame += out_steps;
		}

		if (problem == SETTLING && time > time_settling_min && CheckSettled(msystem, zero_v)) {
			cout << "Granular material settled...  time = " << time << endl;
			break;
		}

		msystem->DoStepDynamics(time_step);

		time += time_step;
		sim_frame++;
		exec_time += msystem->GetTimerStep();
	}

	// Create a checkpoint from the last state
	if (problem == SETTLING)
		utils::WriteCheckpoint(msystem, checkpoint_file);

	// Final stats
	cout << "==================================" << endl;
	cout << "Number of bodies: " << msystem->Get_bodylist()->size() << endl;
	cout << "Lowest position: " << FindLowest(msystem) << endl;
	cout << "Simulation time: " << exec_time << endl;
	cout << "Number of threads: " << threads << endl;

	return 0;
}

