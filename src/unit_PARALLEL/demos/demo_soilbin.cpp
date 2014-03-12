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
	DROPPING,
	COMPLETE
};

ProblemType problem = SETTLING;

// =======================================================================
// Global problem definitions

int threads = 8;

// Simulation parameters
double gravity = -9.81;
double time_step = 0.0001;
double time_drop = 5;
double time_end = 10;

int max_iteration = 20;

// Output
const char* data_folder = "../SOILBIN";
const char* checkpoint_file = "../SOILBIN/settled.dat";
ChStreamOutAsciiFile sph_file("../SOILBIN/soilbin_pos.txt");
double out_fps = 30;

// Parameters for the mixture balls
double     radius = 0.1;
double     density = 2000;
double     vol = (4.0/3) * PI * radius * radius * radius;
double     mass = density * vol;
ChVector<> inertia = 0.4 * mass * radius * radius * ChVector<>(1,1,1);

// Parameters for the falling ball
double     dropHeight = 5;
int        ballId = 100;
double     radius1 = 0.5;
double     density1 = 2000;
double     volume1 = (4.0/3) * PI * radius1 * radius1 * radius1;
double     mass1 = density1 * volume1;
ChVector<> inertia1 = 0.4 * mass1 * radius1 * radius1 * ChVector<>(1,1,1);
ChVector<> initvel1(0, 0, 0);

// Parameters for the containing bin
int    binId = -200;
double hDimX = 5;          // length in x direction
double hDimY = 2;          // depth in y direction
double hDimZ = 2;          // height in z direction
double hThickness = 0.1;   // wall thickness

// =======================================================================

template <typename T>
void AddWall(T&                   body,
             const ChVector<>&    loc,
             const ChVector<>&    dim)
{
	// Append to collision geometry
	body->GetCollisionModel()->AddBox(dim.x, dim.y, dim.z, loc);

	// Append to assets
	ChSharedPtr<ChBoxShape> box_shape = ChSharedPtr<ChAsset>(new ChBoxShape);
	box_shape->SetColor(ChColor(1, 0, 0));
	box_shape->Pos = loc;
	box_shape->Rot = ChQuaternion<>(1,0,0,0);
	box_shape->GetBoxGeometry().Size = dim;

	body->GetAssets().push_back(box_shape);
}

void CreateObjects(ChSystemParallel* system)
{
	// Create a material for the ball mixture
	ChSharedPtr<ChMaterialSurfaceDEM> ballMixMat;
	ballMixMat = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	ballMixMat->SetYoungModulus(2e6f);
	ballMixMat->SetFriction(0.4f);
	ballMixMat->SetDissipationFactor(0.6f);

	// Create a material for the bin
	ChSharedPtr<ChMaterialSurfaceDEM> binMat;
	binMat = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	binMat->SetYoungModulus(2e6f);
	binMat->SetFriction(0.4f);
	binMat->SetDissipationFactor(0.6f);

	// Create a mixture entirely made out of spheres
	utils::Generator gen(system);

	utils::MixtureIngredientPtr& m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
	m1->setDefaultMaterialDEM(ballMixMat);
	m1->setDefaultDensity(density);
	m1->setDefaultSize(radius);

	gen.createObjectsBox(utils::POISSON_DISK,
	                     2.01 * radius,
	                     ChVector<>(0, 0, 2.5),
	                     ChVector<>(0.8 * hDimX, 0.8 * hDimY, 2));

	cout << "Number bodies generated: " << gen.getTotalNumBodies() << endl;

	// Create the containing bin
	ChSharedBodyDEMPtr bin(new ChBodyDEM(new ChCollisionModelParallel));

	bin->SetMaterialSurfaceDEM(binMat);

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
}

// =======================================================================

ChBody* CreateFallingBall()
{
	// Create a material for the falling ball
	ChSharedPtr<ChMaterialSurfaceDEM> ballMat;
	ballMat = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	ballMat->SetYoungModulus(2e6f);
	ballMat->SetFriction(0.4f);
	ballMat->SetDissipationFactor(0.6f);

	// Create the falling ball, but do not add it to the system
	ChBodyDEM* ball = new ChBodyDEM(new ChCollisionModelParallel);

	ball->SetMaterialSurfaceDEM(ballMat);

	ball->SetIdentifier(binId);
	ball->SetMass(mass1);
	ball->SetInertiaXX(inertia1);
	ball->SetPos(ChVector<>(0, 0, dropHeight));
	ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
	ball->SetPos_dt(initvel1);
	ball->SetCollide(true);
	ball->SetBodyFixed(false);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(radius1);
	ball->GetCollisionModel()->BuildModel();

	ChSharedPtr<ChSphereShape> ball_shape = ChSharedPtr<ChAsset>(new ChSphereShape);
	ball_shape->SetColor(ChColor(0, 0, 1));
	ball_shape->GetSphereGeometry().rad = radius1;
	ball_shape->Pos = ChVector<>(0,0,0);
	ball_shape->Rot = ChQuaternion<>(1,0,0,0);
	ball->GetAssets().push_back(ball_shape);

	return ball;
}

// =======================================================================

void OutputFile(ChStreamOutAsciiFile& file,
                ChSystem&             sys,
                double                time)
{
	chrono::Vector bodyAngs;

	file << time << "     ";
	cout << time << "     ";

	for (int i = 0; i < sys.Get_bodylist()->size(); ++i) {
		ChBody* abody = (ChBody*) sys.Get_bodylist()->at(i);
		assert(typeid(*abody) == typeid(ChBodyDEM));

		const ChVector<>& bodypos = abody->GetPos();
		bodyAngs = abody->GetRot().Q_to_NasaAngles();
		file << bodypos.x  << "  " << bodypos.y  << "  " << bodypos.z  << "  ";
		file << bodyAngs.x << "  " << bodyAngs.y << "  " << bodyAngs.z << "       ";
		cout << bodypos.x << "  " << bodypos.y << "  " << bodypos.z << "   |   ";
	}

	file << "\n";
	cout << endl;
}

double FindLowest(ChSystem& sys)
{
	double lowest = 1000;

	for (int i = 0; i < sys.Get_bodylist()->size(); ++i) {
		ChBody* abody = (ChBody*) sys.Get_bodylist()->at(i);
		if (abody->GetBodyFixed())
			continue;
		if (abody->GetPos().z < lowest)
			lowest = abody->GetPos().z;
	}
	return lowest;
}



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

	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetTolerance(0);
	((ChLcpSolverParallelDEM*) msystem->GetLcpSolverSpeed())->SetContactRecoverySpeed(1);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBodyPerBin(100, 50);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);

	// Always create the falling ball (not attached to system)
	ChBody* ball = CreateFallingBall();

	// Select start/end simulation times
	double Tstart;
	double Tend;
	bool dropped = false;

	switch (problem) {
	case SETTLING:
		Tstart = 0;
		Tend = time_drop;    // Simulate from 0 to time_drop
		dropped = true;      // Make sure we never drop the ball
		break;
	case DROPPING:
		Tstart = time_drop;
		Tend = time_end;     // Simulate from time_drop to time_end
		break;
	case COMPLETE:
		Tstart = 0;
		Tend = time_end;     // Simulate from 0 to time_end
		break;
	}

	// Create objects
	if (problem == DROPPING)
		utils::ReadCheckpoint(msystem, checkpoint_file);
	else
		CreateObjects(msystem);

	// Number of steps
	int num_steps = std::ceil((Tend - Tstart) / time_step);
	int out_steps = std::ceil((1 / time_step) / out_fps);

	// Perform the simulation
	double time = Tstart;
	int out_frame = 0;
	double exec_time = 0;

	for (int i = 0; i < num_steps; i++) {
		if (!dropped && time >= time_drop) {
			cout << " >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DROP BALL" << endl;
			msystem->AddBody(ChSharedPtr<ChBody>(ball));
			dropped = true;
		}

		if (i % out_steps == 0) {
			char filename[100];

			// Output for Renderman
			////sprintf(filename, "%s/RENDERMAN/data_%03d.dat", data_folder, out_frame);
			////utils::WriteShapesRender(msystem, filename);

			// Output for POV-Ray
			sprintf(filename, "%s/POVRAY/data_%03d.dat", data_folder, out_frame);
			utils::WriteShapesPovray(msystem, filename);

			// Stats
			cout << " --------------------------------- " << out_frame << "  " << time << "  " <<  endl;
			cout << "                                   " << FindLowest(*msystem) << endl;
			cout << "                                   " << exec_time << endl;

			//OutputFile(sph_file, *msystem, time);

			out_frame++;
		}

		msystem->DoStepDynamics(time_step);
		time += time_step;

		exec_time += msystem->mtimer_step();
	}

	// Create a checkpoint from the last state
	if (problem == SETTLING)
		utils::WriteCheckpoint(msystem, checkpoint_file);

	// Final stats
	cout << "==================================" << endl;
	cout << "Number of bodies: " << msystem->Get_bodylist()->size() << endl;
	cout << "Lowest position: " << FindLowest(*msystem) << endl;
	cout << "Simulation time: " << exec_time << endl;
	cout << "Number of threads: " << threads << endl;

	return 0;
}

