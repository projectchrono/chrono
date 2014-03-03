#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"
#include "utils/generators.h"

using namespace chrono;
using namespace geometry;

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
		assert(typeid(*abody) == typeid(ChBodyDEM));

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
	ChStreamOutAsciiFile sph_file("../soilbin_pos.txt");
	double out_fps = 1000;
	int out_steps = std::ceil((1 / time_step) / out_fps);

	// Parameters for the falling ball
	int             ballId = 100;
	double          radius = .1;
	double          density = 2000;
	double          volume = (4.0/3) * PI * radius * radius * radius;
	double          mass = density * volume;
	ChVector<>      inertia = 0.4 * mass * radius * radius * ChVector<>(1,1,1);

	// Parameters for the containing bin
	int    binId = -200;
	double hDimX = 5;          // length in x direction
	double hDimY = 2;          // depth in y direction
	double hDimZ = 0.5;        // height in z direction
	double hThickness = 0.1;   // wall thickness

	// Create system
	ChSystemParallelDEM* msystem = new ChSystemParallelDEM();

	msystem->Set_G_acc(ChVector<>(0, 0, gravity));

	// Edit system settings
	msystem->SetParallelThreadNumber(threads);
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

	omp_set_num_threads(threads);

	// Create a material for the balls
	ChSharedPtr<ChMaterialSurfaceDEM> ballMat;
	ballMat = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	ballMat->SetYoungModulus(5e4f);
	ballMat->SetFriction(0.4f);
	ballMat->SetDissipationFactor(0.6f);

	// Create a material for the bin
	ChSharedPtr<ChMaterialSurfaceDEM> binMat;
	binMat = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	binMat->SetYoungModulus(5e4f);
	binMat->SetFriction(0.4f);
	binMat->SetDissipationFactor(0.6f);

	// Create the falling balls (a mixture entirely made out of spheres)
/*
	utils::Generator gen(msystem);

	utils::MixtureIngredientPtr& m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
	m1->setDefaultMaterialDEM(ballMat);
	m1->setDefaultDensity(density);
	m1->setDefaultSize(radius);

	gen.createObjectsBox(utils::POISSON_DISK,
	                     4 * radius,
	                     ChVector<>(0, 0, 3 * hDimZ),
	                     ChVector<>(0.8 * hDimX, 0.8 * hDimY, hDimZ / 2));

	std::cout << "Number bodies: " << gen.getTotalNumBodies() << std::endl;
*/



	////utils::GridSampler<> gs(ChVector<>(0.1, 0.2, 0.3));
	////utils::PointVectorD points = gs.SampleBox(ChVector<>(0, 0, 2), ChVector<>(1, 2, 0));

	utils::PDSampler<> pd(0.2);
	utils::PointVectorD points = pd.SampleBox(ChVector<>(0, 0, 2), ChVector<>(1, 2, 0));

	std::cout << "Number points: " << points.size() << std::endl;

	for (int i = 0; i < 1; i++) {
		ChSharedBodyDEMPtr ball(new ChBodyDEM(new ChCollisionModelParallel));

		ball->SetMaterialSurfaceDEM(ballMat);

		ball->SetIdentifier(ballId);
		ball->SetMass(mass);
		ball->SetInertiaXX(inertia);
		ball->SetPos(points[i]);
		ball->SetBodyFixed(false);
		ball->SetRot(ChQuaternion<>(1,0,0,0));

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

		msystem->AddBody(ball);
	}



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
	AddWall(bin, ChVector<>(-hDimX-hThickness, 0, hDimZ), ChVector<>(hThickness, hDimY, 2*hDimZ));
	AddWall(bin, ChVector<>( hDimX+hThickness, 0, hDimZ), ChVector<>(hThickness, hDimY, hDimZ));
	AddWall(bin, ChVector<>(0, -hDimY-hThickness, hDimZ), ChVector<>(hDimX, hThickness, 2*hDimZ));
	AddWall(bin, ChVector<>(0,  hDimY+hThickness, hDimZ), ChVector<>(hDimX, hThickness, hDimZ));
	bin->GetCollisionModel()->BuildModel();

	msystem->AddBody(bin);

	// Perform the simulation
	double time = 0;
	int out_frame = 0;
	char filename[100];

	for (int i = 0; i < num_steps; i++) {
		if (i % out_steps == 0) {
			////sprintf(filename, "./DEM/data_%03d.dat", data_folder, out_frame);
			////utils::WriteShapesRender(msystem, filename);

			OutputFile(sph_file, *msystem, time);

			out_frame++;
		}

		msystem->DoStepDynamics(time_step);
		time += time_step;
	}

	return 0;
}

