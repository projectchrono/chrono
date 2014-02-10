#include <vector>

#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativeAPGD.h"
#include "lcp/ChLcpIterativeBB.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeJacobi.h"

#include "core/ChLinearAlgebra.h"
#include "core/ChRealtimeStep.h"

#include "physics/ChApidll.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChContactContainer.h"

#include "physics/CHBodyDEM.h"
#include "physics/CHcontactContainerDEM.h"
#include "lcp/CHlcpSolverDEM.h"

#include "collision/ChCModelBullet.h"
#include "collision/CHcModelBulletBody.h"
#include "collision/ChCCollisionSystemBullet.h"

#include "assets/ChSphereShape.h"
#include "assets/ChBoxShape.h"

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"


using namespace chrono;
using namespace geometry;
using namespace std;


void AddWall(ChSharedBodyDEMPtr&  body,
             const ChVector<>&    dim,
             const ChVector<>&    loc)
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

	int max_iteration = 20;

	// Parameters for the falling ball
	int             ballId = 100;
	double          radius = .5;
	double          mass = 1;
	ChVector<>      pos(0, 3, 0);
	ChQuaternion<>  rot(1, 0, 0, 0);
	ChVector<>      init_vel(0, 0, 10);

	// Parameters for the containing bin
	int    binId = 200;
	double width = 5;           //width of area with particles
	double length = 25;         //length of area with particles
	double height = 2;          //height of the outer walls
	double thickness = .25;     //thickness of container walls

	// Create system
	ChSystemParallel * msystem = new ChSystemParallel;

	// Create a material (will be used by both objects)
	ChSharedPtr<ChMaterialSurfaceDEM> material;
	material = ChSharedPtr<ChMaterialSurfaceDEM>(new ChMaterialSurfaceDEM);
	material->SetFriction(0.4f);

	// Edit system settings
	msystem->SetIntegrationType(ChSystem::INT_ANITESCU);
	msystem->SetParallelThreadNumber(threads);
	msystem->SetMaxiter(max_iteration);
	msystem->SetIterLCPmaxItersSpeed(max_iteration);
	msystem->SetTol(1e-3);
	msystem->SetTolSpeeds(1e-3);
	msystem->Set_G_acc(ChVector<>(0, gravity, 0));
	msystem->SetStep(time_step);

	((ChLcpSolverParallel *) (msystem->GetLcpSolverSpeed()))->SetMaxIteration(max_iteration);
	((ChLcpSolverParallel *) (msystem->GetLcpSolverSpeed()))->SetTolerance(0);
	((ChLcpSolverParallel *) (msystem->GetLcpSolverSpeed()))->SetCompliance(0);
	((ChLcpSolverParallel *) (msystem->GetLcpSolverSpeed()))->SetContactRecoverySpeed(1);
	((ChLcpSolverParallel *) (msystem->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);

	((ChCollisionSystemParallel *) (msystem->GetCollisionSystem()))->SetCollisionEnvelope(radius * 0.05);
	((ChCollisionSystemParallel *) (msystem->GetCollisionSystem()))->setBinsPerAxis(I3(10, 10, 10));
	((ChCollisionSystemParallel *) (msystem->GetCollisionSystem()))->setBodyPerBin(100, 50);

	omp_set_num_threads(threads);

	// Create the falling ball
	ChVector<>      loc_pos(0, 0, 0);
	ChQuaternion<>  loc_rot(1, 0, 0, 0);

	ChSharedBodyDEMPtr ball = ChSharedBodyDEMPtr(new ChBodyDEM(new ChCollisionModelParallel));

	ball->SetIdentifier(ballId);
	ball->SetMass(mass);
	ball->SetPos(pos);
	ball->SetRot(rot);
	ball->SetBodyFixed(false);
	ball->SetMaterialSurfaceDEM(material);

	ball->SetCollide(true);
	ball->GetCollisionModel()->SetFamily(-15);
	ball->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(-15);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(radius, loc_pos);
	ball->GetCollisionModel()->BuildModel();

	//ball->SetPos_dt(init_vel);

	ChSharedPtr<ChSphereShape> sphere_shape = ChSharedPtr<ChAsset>(new ChSphereShape);
	sphere_shape->SetColor(ChColor(1, 0, 0));
	sphere_shape->GetSphereGeometry().rad = radius;
	sphere_shape->Pos = loc_pos;
	sphere_shape->Rot = loc_rot;
	ball->GetAssets().push_back(sphere_shape);

	msystem->AddBody(ball);

	// Create the containing bin
	ChSharedBodyDEMPtr bin = ChSharedBodyDEMPtr(new ChBodyDEM(new ChCollisionModelParallel));

	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(0, 0, 0));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);
	bin->SetMaterialSurfaceDEM(material);

	bin->GetCollisionModel()->SetFamily(-20);
	bin->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(-20);

	bin->GetCollisionModel()->ClearModel();
	AddWall(bin, ChVector<>(width, thickness, length), ChVector<>(0, 0, 0));
	//AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(-width + thickness, height, 0));
	//AddWall(bin, ChVector<>(thickness, height, length), ChVector<>(width - thickness, height, 0));
	//AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, -length + thickness));
	//AddWall(bin, ChVector<>(width, height, thickness), ChVector<>(0, height, length - thickness));
	ball->GetCollisionModel()->BuildModel();

	msystem->AddBody(bin);

	// Perform the simulation
	double time = 0;

	while (time < time_end) {

		// Walk the list of bodies in the system and output the ball position
		std::cout << "Bodies" << std::endl;
		for (int i = 0; i < msystem->Get_bodylist()->size(); ++i) {
			ChBody* abody = (ChBody*) msystem->Get_bodylist()->at(i);
			ChVector<> pos = abody->GetPos();
			std::cout << "  [" << abody->GetIdentifier() << "]   " << pos.x << "  " << pos.y << "  " << pos.z << std::endl;
		}

		std::cout << "Other" << std::endl;
		std::list<ChPhysicsItem*>::iterator it = msystem->Get_otherphysicslist()->begin();
		while (it != msystem->Get_otherphysicslist()->end()) {
			ChBodyDEM* abody = dynamic_cast<ChBodyDEM*>(*it);
			ChVector<> pos = abody->GetPos();
			std::cout << "  [" << abody->GetIdentifier() << "]   " << pos.x << "  " << pos.y << "  " << pos.z << std::endl;
		}


		msystem->DoStepDynamics(time_step);

		time += time_step;
	}

	return 0;
}

