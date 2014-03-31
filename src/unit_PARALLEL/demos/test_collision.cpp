#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

using namespace chrono;
using namespace geometry;

using std::cout;
using std::endl;


#define DEM
#define NARROW_NEW


int counter = 0;

// ------------------------------------------------------------------------------

void writeContactInfo(ChParallelDataManager* data_container)
{
	int    ncontacts = data_container->number_of_rigid_rigid;

	real3* pt1 = data_container->host_data.cpta_rigid_rigid.data();
	real3* pt2 = data_container->host_data.cptb_rigid_rigid.data();
	real3* nrm = data_container->host_data.norm_rigid_rigid.data();
	real*  depth = data_container->host_data.dpth_rigid_rigid.data();
	real*  erad = data_container->host_data.erad_rigid_rigid.data();

	int2* bid = data_container->host_data.bids_rigid_rigid.data();

	cout << endl << "Number contacts: " << ncontacts << endl << endl;
	for (int i = 0; i < ncontacts; i++) {
		int body1 = bid[i].x;
		int body2 = bid[i].y;

		cout << body1 << " - " << body2 << endl;
#ifdef NARROW_NEW
		cout << "Radius: " << erad[i] << endl;
#endif
		cout << "Depth:  " << depth[i] << endl;
		cout << "Point1: " << pt1[i].x << " " << pt1[i].y << " " << pt1[i].z << endl;
		cout << "Point2: " << pt2[i].x << " " << pt2[i].y << " " << pt2[i].z << endl;
		cout << "Normal: " << nrm[i].x << " " << nrm[i].y << " " << nrm[i].z << endl << endl;
	}
}

// ------------------------------------------------------------------------------

void
createSphere(ChSystemParallel* sys,
             const ChVector<>& pos,
             const ChQuaternion<>& rot,
             double radius)
{
#ifdef DEM
	ChSharedBodyDEMPtr ball(new ChBodyDEM(new ChCollisionModelParallel));
#else
	ChSharedBodyPtr ball(new ChBody(new ChCollisionModelParallel));
#endif
	ball->SetPos(pos);
	ball->SetRot(rot);
	ball->SetBodyFixed(false);
	ball->SetCollide(true);

	ball->GetCollisionModel()->ClearModel();
	ball->GetCollisionModel()->AddSphere(radius);
	ball->GetCollisionModel()->BuildModel();

	sys->AddBody(ball);

	cout << counter++ << " [ball]" << endl;
	cout << "   radius: " << radius << endl;
	cout << "   pos:    " << pos.x << "  " << pos.y << "  " << pos.z << endl;
	cout << "   rot:    " << rot.e0 << "  " << rot.e1 << "  " << rot.e2 << "  " << rot.e3 << endl;
}

// ------------------------------------------------------------------------------

void
createCapsule(ChSystemParallel* sys,
              const ChVector<>& pos,
              const ChQuaternion<>& rot,
              double radius, double hlen)
{
#ifdef DEM
	ChSharedBodyDEMPtr capsule(new ChBodyDEM(new ChCollisionModelParallel));
#else
	ChSharedBodyPtr capsule(new ChBody(new ChCollisionModelParallel));
#endif
	capsule->SetPos(pos);
	capsule->SetRot(rot);
	capsule->SetBodyFixed(false);
	capsule->SetCollide(true);

	capsule->GetCollisionModel()->ClearModel();
	capsule->GetCollisionModel()->AddCapsule(radius, hlen);
	capsule->GetCollisionModel()->BuildModel();

	sys->AddBody(capsule);

	cout << counter++ << " [capsule]" << endl;
	cout << "   radius: " << radius << endl;
	cout << "   hlen:   " << hlen << endl;
	cout << "   pos:    " << pos.x << "  " << pos.y << "  " << pos.z << endl;
	cout << "   rot:    " << rot.e0 << "  " << rot.e1 << "  " << rot.e2 << "  " << rot.e3 << endl;
}

// ------------------------------------------------------------------------------

void
createBox(ChSystemParallel* sys,
          const ChVector<>& pos,
          const ChQuaternion<>& rot,
          const ChVector<>& hdims)
{
#ifdef DEM
	ChSharedBodyDEMPtr box(new ChBodyDEM(new ChCollisionModelParallel));
#else
	ChSharedBodyPtr box(new ChBody(new ChCollisionModelParallel));
#endif
	box->SetPos(pos);
	box->SetRot(rot);
	box->SetCollide(true);
	box->SetBodyFixed(false);

	box->GetCollisionModel()->ClearModel();
	box->GetCollisionModel()->AddBox(hdims.x, hdims.y, hdims.z);
	box->GetCollisionModel()->BuildModel();

	sys->AddBody(box);

	cout << counter++ << " [box]" << endl;
	cout << "   hdims:  " << hdims.x << "  " << hdims.y << "  " << hdims.z << endl;
	cout << "   pos:    " << pos.x << "  " << pos.y << "  " << pos.z << endl;
	cout << "   rot:    " << rot.e0 << "  " << rot.e1 << "  " << rot.e2 << "  " << rot.e3 << endl;
}

// ------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
	int threads = 8;
	omp_set_num_threads(threads);

	// Create system
#ifdef DEM
	cout << "DEM";
	ChSystemParallelDEM msystem;
#ifdef NARROW_NEW
	cout << "  R" << endl << endl;
	((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);
#else
	cout << "  MPR" << endl << endl;
#endif
#else
	cout << "DVI" << endl << endl;
	ChSystemParallelDVI msystem;
#endif


	//createBox(&msystem, ChVector<>(0, 0, -0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(5, 2, 0.1));
	//createSphere(&msystem, ChVector<>(0.629447, -1.45809, 0.045702643641292666), ChQuaternion<>(1, 0, 0, 0), 0.1);
	//createSphere(&msystem, ChVector<>(1, -1, 0.045702643641292666), ChQuaternion<>(1, 0, 0, 0), 0.1);

	//createSphere(&msystem, ChVector<>(0, 0, 7), ChQuaternion<>(1, 0, 0, 0), 1);
	//createSphere(&msystem, ChVector<>(0, 0, 4), ChQuaternion<>(1, 0, 0, 0), 2);

	//ChQuaternion<> rot;
	//rot.Q_from_AngAxis(PI/4, ChVector<>(1, 0, 0));
	//createCapsule(&msystem, ChVector<>(0, 0, 4), rot, 2, 1);

	//createCapsule(&msystem, ChVector<>(0, 0, 4), ChQuaternion<>(1, 0, 0, 0), 2, 1);
	//createBox(&msystem, ChVector<>(0, 0, 4), ChQuaternion<>(1, 0, 0, 0), ChVector<>(2, 3, 2));

	createBox(&msystem, ChVector<>(0, 0, -0.1), ChQuaternion<>(1, 0, 0, 0), ChVector<>(5, 2, 0.1));
	//createSphere(&msystem, ChVector<>(2, -1, 2), ChQuaternion<>(1, 0, 0, 0), 2.1);
	ChQuaternion<> rot;
	rot.Q_from_AngAxis(PI/2, ChVector<>(1, 0, 0));
	createCapsule(&msystem, ChVector<>(2, -1, 3), rot, 2.1, 1);




	// Perform the collision detection.
	msystem.Update();
	msystem.GetCollisionSystem()->Run();

	writeContactInfo(msystem.gpu_data_manager);

	return 0;
}

