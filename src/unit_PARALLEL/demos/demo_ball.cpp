#include <stdio.h>
#include <vector>
#include <cmath>

#include "assets/ChSphereShape.h"
#include "assets/ChEllipsoidShape.h"
#include "assets/ChBoxShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChConeShape.h"

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"

using namespace chrono;
using namespace geometry;

//// Comment this for DVI contact
#define DEM

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

void OutputFile(ChStreamOutAsciiFile& file,
                ChSystem&             sys,
                double                time)
{
	chrono::Vector bodyAngs;

	file << time << "     ";
	std::cout << time << "     ";

	for (int i = 0; i < sys.Get_bodylist()->size(); ++i) {
		ChBody* abody = (ChBody*) sys.Get_bodylist()->at(i);
#ifdef DEM
		assert(typeid(*abody) == typeid(ChBodyDEM));
#endif
		const ChVector<>& bodypos = abody->GetPos();
		bodyAngs = abody->GetRot().Q_to_NasaAngles();
		file << bodypos.x  << "  " << bodypos.y  << "  " << bodypos.z  << "  ";
		file << bodyAngs.x << "  " << bodyAngs.y << "  " << bodyAngs.z << "       ";
		std::cout << bodypos.x << "  " << bodypos.y << "  " << bodypos.z << "   |   ";
	}

	file << "\n";
	std::cout << std::endl;
}

int main(int argc, char* argv[])
{
	int threads = 8;

	// Simulation parameters
	double gravity = -9.81;
	double time_step = 0.0001;
	double time_end = 1;
	int    num_steps = std::ceil(time_end / time_step);

	int max_iteration = 20;

	// Output
#ifdef DEM
	ChStreamOutAsciiFile sph_file("../sphere_pos_DEM.txt");
	char* data_folder = "./DEM";
#else
	ChStreamOutAsciiFile sph_file("../sphere_pos_DVI.txt");
	char* data_folder = "./DVI";
#endif
	double out_fps = 1000;
	int out_steps = std::ceil((1 / time_step) / out_fps);

	// Parameters for the falling ball
	int             ballId = 100;
	double          radius = 0.1;
	double          density = 2000;
	double          volume = (4.0/3) * CH_C_PI * radius * radius * radius;
	double          mass = density * volume;
	ChVector<>      inertia = 0.4 * mass * radius * radius * ChVector<>(1,1,1);
	ChVector<>      initPos(1, -1, 0.5);
	ChVector<>      initPos2(0.629447, -1.45809, 0.5);
	ChQuaternion<>  initRot(1,0,0,0);//(0.89, 0.4, -0.2, 0.088889);
	ChVector<>      initVel(0,0,0);//(0.75, 0.2, 0);

	// Parameters for the containing bin
	int    binId = -200;
	double hDimX = 5;          // length in x direction
	double hDimY = 2;          // depth in y direction
	double hDimZ = 0.5;        // height in z direction
	double hThickness = 0.1;   // wall thickness

	// Create system
#ifdef DEM
	ChSystemParallelDEM msystem;
#else
	ChSystemParallelDVI msystem;
#endif

	msystem.Set_G_acc(ChVector<>(0, 0, gravity));

	// Edit system settings
	msystem.SetParallelThreadNumber(threads);
	msystem.SetMaxiter(max_iteration);
	msystem.SetIterLCPmaxItersSpeed(max_iteration);
	msystem.SetTol(1e-3);
	msystem.SetTolSpeeds(1e-3);
	msystem.SetStep(time_step);

#ifdef DEM
	((ChLcpSolverParallelDEM*) msystem.GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
	((ChLcpSolverParallelDEM*) msystem.GetLcpSolverSpeed())->SetTolerance(0);
	((ChLcpSolverParallelDEM*) msystem.GetLcpSolverSpeed())->SetContactRecoverySpeed(1);

	((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->setBodyPerBin(100, 50);
#else
	((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
	((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetTolerance(0);
	((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetCompliance(0);
	((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetContactRecoverySpeed(1);
	((ChLcpSolverParallelDVI*) msystem.GetLcpSolverSpeed())->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);

	((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->SetCollisionEnvelope(radius * 0.05);
	((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->setBodyPerBin(100, 50);
#endif

	omp_set_num_threads(threads);

	// Create a material for the balls
#ifdef DEM
	ChSharedPtr<ChMaterialSurfaceDEM> ballMat(new ChMaterialSurfaceDEM);
	ballMat->SetYoungModulus(5e4f);
	ballMat->SetFriction(0.4f);
	ballMat->SetDissipationFactor(0.6f);
#else
	ChSharedPtr<ChMaterialSurface> ballMat(new ChMaterialSurface);
	ballMat->SetFriction(0.4f);
#endif

	// Create a material for the bin
#ifdef DEM
	ChSharedPtr<ChMaterialSurfaceDEM> binMat(new ChMaterialSurfaceDEM);
	binMat->SetYoungModulus(5e4f);
	binMat->SetFriction(0.4f);
	binMat->SetDissipationFactor(0.6f);
#else
	ChSharedPtr<ChMaterialSurface> binMat(new ChMaterialSurface);
	binMat->SetFriction(0.4f);
#endif

	// Create the falling ball
#ifdef DEM
	ChSharedBodyDEMPtr ball(new ChBodyDEM(new ChCollisionModelParallel));
	ball->SetMaterialSurfaceDEM(ballMat);
#else
	ChSharedBodyPtr ball(new ChBody(new ChCollisionModelParallel));
	ball->SetMaterialSurface(ballMat);
#endif

	ball->SetIdentifier(ballId);
	ball->SetMass(mass);
	ball->SetInertiaXX(inertia);
	ball->SetPos(initPos);
	ball->SetRot(initRot);
	ball->SetPos_dt(initVel);
	ball->SetBodyFixed(false);

	ball->SetCollide(true);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(radius);
	ball->GetCollisionModel()->BuildModel();

	ChSharedPtr<ChSphereShape> sphere_shape = ChSharedPtr<ChAsset>(new ChSphereShape);
	sphere_shape->SetColor(ChColor(1, 0, 0));
	sphere_shape->GetSphereGeometry().rad = radius;
	sphere_shape->Pos = ChVector<>(0,0,0);
	sphere_shape->Rot = ChQuaternion<>(1,0,0,0);
	ball->GetAssets().push_back(sphere_shape);

	msystem.AddBody(ball);

	// Create a second ball
#ifdef DEM
	ChSharedBodyDEMPtr ball2(new ChBodyDEM(new ChCollisionModelParallel));
	ball2->SetMaterialSurfaceDEM(ballMat);
#else
	ChSharedBodyPtr ball2(new ChBody(new ChCollisionModelParallel));
	ball2->SetMaterialSurface(ballMat);
#endif

	ball2->SetIdentifier(ballId);
	ball2->SetMass(mass);
	ball2->SetInertiaXX(inertia);
	ball2->SetPos(initPos2);
	ball2->SetRot(initRot);
	ball2->SetPos_dt(initVel);
	ball2->SetBodyFixed(false);

	ball2->SetCollide(true);

	ball2->GetCollisionModel()->ClearModel();
	ball2->GetCollisionModel()->AddSphere(radius);
	ball2->GetCollisionModel()->BuildModel();

	ball2->GetAssets().push_back(sphere_shape);

	msystem.AddBody(ball2);


	// Create the containing bin
#ifdef DEM
	ChSharedBodyDEMPtr bin(new ChBodyDEM(new ChCollisionModelParallel));
	bin->SetMaterialSurfaceDEM(binMat);
#else
	ChSharedBodyPtr bin(new ChBody(new ChCollisionModelParallel));
	bin->SetMaterialSurface(binMat);
#endif

	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0, 0, 0));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);

	bin->GetCollisionModel()->ClearModel();
	AddWall(bin, ChVector<>(0, 0, -hThickness), ChVector<>(hDimX, hDimY, hThickness));
	AddWall(bin, ChVector<>(-hDimX-hThickness, 0, hDimZ), ChVector<>(hThickness, hDimY, 2*hDimZ));
	AddWall(bin, ChVector<>( hDimX+hThickness, 0, hDimZ), ChVector<>(hThickness, hDimY, hDimZ));
	AddWall(bin, ChVector<>(0, -hDimY-hThickness, hDimZ), ChVector<>(hDimX, hThickness, 2*hDimZ));
	AddWall(bin, ChVector<>(0,  hDimY+hThickness, hDimZ), ChVector<>(hDimX, hThickness, hDimZ));
	bin->GetCollisionModel()->BuildModel();

	msystem.AddBody(bin);

	// Perform the simulation
	double time = 0;
	int out_frame = 0;
	char filename[100];

	for (int i = 0; i < num_steps; i++) {

		if (i % out_steps == 0) {
			////sprintf(filename, "%s/data_%03d.dat", data_folder, out_frame);
			////utils::WriteShapesRender(&msystem, filename);

			OutputFile(sph_file, msystem, time);

			out_frame++;
		}

		msystem.DoStepDynamics(time_step);
		time += time_step;
	}

	return 0;
}

