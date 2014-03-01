#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"
#include "utils/generators.h"

using namespace chrono;
using namespace geometry;

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
	box_shape->Rot = ChQuaternion<>(1,0,0,0);
	box_shape->GetBoxGeometry().Size = dim;

	body->GetAssets().push_back(box_shape);
}


int main(int argc, char* argv[])
{
	// Simulation parameters
	int threads = 8;

	double gravity = -9.81;
	double time_step = 0.0001;
	double time_end = 1;
	int    num_steps = std::ceil(time_end / time_step);

	int max_iteration = 20;

	// Output
	char* data_folder = "./DEM";
	double out_fps = 60;
	int out_steps = std::ceil((1 / time_step) / out_fps);

	// Parameters for the falling ball
	int             ballId = 100;
	double          radius = .1;
	double          density = 2000;
	double          volume = (4.0/3) * PI * radius * radius * radius;
	double          mass = density * volume;
	ChVector<>      inertia = 0.4 * mass * radius * radius * ChVector<>(1,1,1);
	ChVector<>      init_vel(0, 0, 0);

	// Parameters for the containing bin
	int    binId = -200;
	double hDimX = 5;          // length in x direction
	double hDimY = 2;          // depth in y direction
	double hDimZ = 0.5;        // height in z direction
	double hThickness = 0.1;   // wall thickness

	// Create system
	ChSystemParallelDEM* msystem = new ChSystemParallelDEM();

	// Edit system settings
	msystem->SetParallelThreadNumber(threads);
	msystem->SetMaxiter(max_iteration);
	msystem->SetIterLCPmaxItersSpeed(max_iteration);
	msystem->SetTol(1e-3);
	msystem->SetTolSpeeds(1e-3);
	msystem->Set_G_acc(ChVector<>(0, 0, gravity));
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

	// Create the falling balls
	utils::PDSampler<> my_sampler(radius);
	utils::PointVectorD
		points = my_sampler.SampleBox(ChVector<>(0, 0, 2 * hDimZ), ChVector<>(1.8 * hDimX, 1.8 * hDimY, 0));

	for (int i = 0; i < points.size(); i++) {
		ChSharedBodyDEMPtr ball = ChSharedBodyDEMPtr(new ChBodyDEM(new ChCollisionModelParallel));
	
		ball->SetIdentifier(ballId);
		ball->SetMass(mass);
		ball->SetPos(points[i]);
		ball->SetPos_dt(init_vel);
		ball->SetBodyFixed(false);
		ball->SetMaterialSurfaceDEM(ballMat);
	
		ball->SetCollide(true);
		ball->GetCollisionModel()->SetFamily(-15);
	
		ball->GetCollisionModel()->ClearModel();
		ball->GetCollisionModel()->AddSphere(radius);
		ball->GetCollisionModel()->BuildModel();
	
		ball->SetInertiaXX(inertia);
	
		ChSharedPtr<ChSphereShape> sphere_shape = ChSharedPtr<ChAsset>(new ChSphereShape);
		sphere_shape->SetColor(ChColor(1, 0, 0));
		sphere_shape->GetSphereGeometry().rad = radius;
		sphere_shape->Pos = ChVector<>(0);
		sphere_shape->Rot = ChQuaternion<>(1,0,0,0);
		ball->GetAssets().push_back(sphere_shape);
	
		msystem->AddBody(ball);
	}


	// Create the containing bin
	ChSharedBodyDEMPtr bin = ChSharedBodyDEMPtr(new ChBodyDEM(new ChCollisionModelParallel));

	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0, 0, 0));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);
	bin->SetMaterialSurfaceDEM(binMat);

	bin->GetCollisionModel()->SetFamily(-20);
	bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(-20);

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
			sprintf(filename, "%s/data_%03d.dat", data_folder, out_frame);
			utils::WriteShapesRender(msystem, filename);

			for (int i = 0; i < msystem->Get_bodylist()->size(); ++i) {
				ChBody* abody = (ChBody*) msystem->Get_bodylist()->at(i);
				if (abody->GetIdentifier() == ballId) {
					ChVector<> pos = abody->GetPos();
					std::cout << "------ Frame: " << out_frame << " Time: " << time << " Height: " << pos.z << std::endl;
				}
			}

			out_frame++;
		}

		msystem->DoStepDynamics(time_step);
		time += time_step;
	}

	return 0;
}

