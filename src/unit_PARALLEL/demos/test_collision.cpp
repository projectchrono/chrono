#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

using namespace chrono;
using namespace geometry;


#define DEM

void writeContactInfo(ChParallelDataManager* data_container)
{
	int    ncontacts = data_container->number_of_rigid_rigid;
	real3* pt1 = data_container->host_data.cpta_rigid_rigid.data();
	real3* pt2 = data_container->host_data.cptb_rigid_rigid.data();
	real3* nrm = data_container->host_data.norm_rigid_rigid.data();
	real*  depth = data_container->host_data.dpth_rigid_rigid.data();

	std::cout << "Number contacts: " << ncontacts << std::endl;
	for (int i = 0; i < ncontacts; i++) {
		std::cout << "Depth: " << depth[i] << std::endl;
		std::cout << "Point1: " << pt1[i].x << " " << pt1[i].y << " " << pt1[i].z << std::endl;
		std::cout << "Point2: " << pt2[i].x << " " << pt2[i].y << " " << pt2[i].z << std::endl;
		std::cout << "Normal: " << nrm[i].x << " " << nrm[i].y << " " << nrm[i].z << std::endl << std::endl;
	}
}


int main(int argc, char* argv[])
{
	int threads = 8;
	omp_set_num_threads(threads);

	ChVector<>     ball_pos(0.629447, -1.45809, 0.045702643641292666);
	ChQuaternion<> ball_rot(1, 0, 0, 0);
	real           ball_radius = 0.1;

	ChVector<>     box_pos(0, 0, -0.1);
	ChQuaternion<> box_rot(1, 0, 0, 0);
	ChVector<>     box_hdims(5, 2, 0.1);

	std::cout << "Box position: " << box_pos.x << "  " << box_pos.y << "  " << box_pos.z << std::endl;
	std::cout << "Box half-dims: " << box_hdims.x << "  " << box_hdims.y << "  " << box_hdims.z << std::endl << std::endl;

#ifdef DEM
	std::cout << "DEM" << std::endl << std::endl;
	ChSystemParallelDEM msystem;
#else
	std::cout << "DVI" << std::endl << std::endl;
	ChSystemParallelDVI msystem;
#endif


	((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);

	// Create the box body
#ifdef DEM
	ChSharedBodyDEMPtr bin(new ChBodyDEM(new ChCollisionModelParallel));
#else
	ChSharedBodyPtr bin(new ChBody(new ChCollisionModelParallel));
#endif
	bin->SetPos(box_pos);
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);

	bin->GetCollisionModel()->ClearModel();
	bin->GetCollisionModel()->AddBox(box_hdims.x, box_hdims.y, box_hdims.z);
	bin->GetCollisionModel()->BuildModel();

	msystem.AddBody(bin);


	// Create the ball body
#ifdef DEM
	ChSharedBodyDEMPtr ball(new ChBodyDEM(new ChCollisionModelParallel));
#else
	ChSharedBodyPtr ball(new ChBody(new ChCollisionModelParallel));
#endif
	ball->SetPos(ball_pos);
	ball->SetRot(ball_rot);
	ball->SetBodyFixed(false);
	ball->SetCollide(true);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(ball_radius);
	ball->GetCollisionModel()->BuildModel();

	msystem.AddBody(ball);


	// Perform the collision detection.
	msystem.Update();
	msystem.GetCollisionSystem()->Run();

	// Print contact information
	std::cout << "Ball radius: " << ball_radius << std::endl;
	std::cout << "Ball position: " << ball_pos.x << "  " << ball_pos.y << "  " << ball_pos.z << std::endl;
	writeContactInfo(msystem.gpu_data_manager);

	std::cout << " --------------------" << std::endl;

	// Move ball in horizontal plane, perform collision detection, print info
	ball_pos.x = 1;
	ball_pos.y = -1;
	ball->SetPos(ball_pos);

	msystem.Update();
	msystem.GetCollisionSystem()->Run();
	std::cout << "Ball radius: " << ball_radius << std::endl;
	std::cout << "Ball position: " << ball_pos.x << "  " << ball_pos.y << "  " << ball_pos.z << std::endl;
	writeContactInfo(msystem.gpu_data_manager);


	return 0;
}

