#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/input_output.h"

using namespace chrono;
using namespace geometry;


void AddWall(ChSharedBodyPtr&  body,
             const ChVector<>& dim,
             const ChVector<>& loc)
{
	// Append to collision geometry
	body->GetCollisionModel()->AddBox(dim.x, dim.y, dim.z, loc);

	// Append to assets
	ChSharedPtr<ChBoxShape> box_shape = ChSharedPtr<ChAsset>(new ChBoxShape);
	box_shape->SetColor(ChColor(1, 0, 0));
	box_shape->Pos = loc;
	box_shape->GetBoxGeometry().Size = dim;

	body->GetAssets().push_back(box_shape);
}


int main(int argc, char* argv[])
{
	// Simulation parameters
	int threads = 8;

	double gravity = -9.81;
	double time_step = .01;
	double time_end = 1;
	int    num_steps = std::ceil(time_end / time_step);

	int max_iteration = 20;

	// Output
	char* data_folder = "./DVI";
	double out_fps = 60;
	int out_steps = std::ceil((1 / time_step) / out_fps);

	// Parameters for the falling ball
	int             ballId = 100;
	double          radius = .5;
	double          mass = 1;
	ChVector<>      pos(0, 3, 0);
	ChQuaternion<>  rot(1, 0, 0, 0);
	ChVector<>      init_vel(0, 0, 0);

	// Parameters for the containing bin
	int    binId = -200;
	double width = 5;
	double length = 25;
	double height = 2;
	double thickness = .25;

	// Create system
	ChSystemParallelDVI* msystem = new ChSystemParallelDVI();

	// Create a material (will be used by both objects)
	ChSharedPtr<ChMaterialSurface> material;
	material = ChSharedPtr<ChMaterialSurface>(new ChMaterialSurface);
	material->SetFriction(0.4f);

	// Edit system settings
	msystem->SetParallelThreadNumber(threads);
	msystem->SetMaxiter(max_iteration);
	msystem->SetIterLCPmaxItersSpeed(max_iteration);
	msystem->SetTol(1e-3);
	msystem->SetTolSpeeds(1e-3);
	msystem->Set_G_acc(ChVector<>(0, gravity, 0));
	msystem->SetStep(time_step);

	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetTolerance(0);
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetCompliance(0);
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetContactRecoverySpeed(1);
	((ChLcpSolverParallelDVI*) msystem->GetLcpSolverSpeed())->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);

	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->SetCollisionEnvelope(radius * 0.05);
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel*) msystem->GetCollisionSystem())->setBodyPerBin(100, 50);

	omp_set_num_threads(threads);

	// Create the falling ball
	ChVector<>      loc_pos(0, 0, 0);
	ChQuaternion<>  loc_rot(1, 0, 0, 0);

	ChSharedBodyPtr ball = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));

	ball->SetIdentifier(ballId);
	ball->SetMass(mass);
	ball->SetPos(pos);
	ball->SetRot(rot);
	ball->SetPos_dt(init_vel);
	ball->SetBodyFixed(false);
	ball->SetMaterialSurface(material);

	ball->SetCollide(true);
	ball->GetCollisionModel()->SetFamily(-15);
	ball->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(-15);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(radius, loc_pos);
	ball->GetCollisionModel()->BuildModel();

	ball->SetInertiaXX(0.4*mass*radius*radius*ChVector<>(1,1,1));

	ChSharedPtr<ChSphereShape> sphere_shape = ChSharedPtr<ChAsset>(new ChSphereShape);
	sphere_shape->SetColor(ChColor(1, 0, 0));
	sphere_shape->GetSphereGeometry().rad = radius;
	sphere_shape->Pos = loc_pos;
	sphere_shape->Rot = loc_rot;
	ball->GetAssets().push_back(sphere_shape);

	msystem->AddBody(ball);

	// Create the containing bin
	ChSharedBodyPtr bin = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));

	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0, 0, 0));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);
	bin->SetMaterialSurface(material);

	bin->GetCollisionModel()->SetFamily(-20);
	bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(-20);

	bin->GetCollisionModel()->ClearModel();
	AddWall(bin, ChVector<>(width, thickness, length), ChVector<>(0, 0, 0));
	AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(-width + thickness, height, 0));
	AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(width - thickness, height, 0));
	AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, -length + thickness));
	AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, length - thickness));
	bin->GetCollisionModel()->BuildModel();

	msystem->AddBody(bin);

	// Perform the simulation
	double time = 0;
	int out_frame = 0;
	char filename[100];

	for (int i = 0; i < num_steps; i++) {

		if (i % out_steps == 0) {
			sprintf(filename, "%s/out_%04d.csv", data_folder, out_frame);
			WriteShapesRender(msystem, filename);

			for (int i = 0; i < msystem->Get_bodylist()->size(); ++i) {
				ChBody* abody = (ChBody*) msystem->Get_bodylist()->at(i);
				if (abody->GetIdentifier() == ballId) {
					ChVector<> pos = abody->GetPos();
					std::cout << "Frame: " << out_frame << " Time: " << time << " Height: " << pos.y << std::endl;
				}
			}

			out_frame++;
		}

		msystem->DoStepDynamics(time_step);
		time += time_step;
	}

	return 0;
}

