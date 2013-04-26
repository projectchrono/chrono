#include "common.h"

vector<contact_dat> contact_cpu, contact_gpu;

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

real mass = 1;
real radius = .1;
real tolerance = 1e-6;
real envelope = 1e-3;
real margin = 0;
int max_bodies = 100;
bool dump_to_file = true;
using namespace chrono;
uint num_objects = 0;

#define GPU_BULLET 1

template<class T>
void DumpObjects(T* mSys, string filename) {
	ofstream ofile(filename.c_str());

	for (int i = 0; i < mSys->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) mSys->Get_bodylist()->at(i);
		if (abody->IsActive() == true) {
			ofile << abody->GetPos().x << "\t" << abody->GetPos().y << "\t" << abody->GetPos().z << "\t";
			ofile << abody->GetRot().e0 << "\t" << abody->GetRot().e1 << "\t" << abody->GetRot().e2 << "\t" << abody->GetRot().e3 << "\t";
			ofile << abody->GetPos_dt().x << "\t" << abody->GetPos_dt().y << "\t" << abody->GetPos_dt().z << "\t";
			ofile << abody->GetWvel_loc().x << "\t" << abody->GetWvel_loc().y << "\t" << abody->GetWvel_loc().z << endl;
		}
	}

}

void LoadObjects_CPU(ChSystem* mSys, string filename) {
	ifstream ifile(filename.c_str());
	ChSharedBodyPtr body;
	int counter = 0;
	Vector pos, pos_dt, wvel_loc;
	Quaternion quat;
	string temp;

	while (ifile.fail() == false && counter < max_bodies) {
		getline(ifile, temp);
		if (ifile.fail() == true) {
			break;
		}
		body = ChSharedBodyPtr(new ChBody);
		//cout << "CPU LOAD BODY: " << counter << endl;

		stringstream ss(temp);
		ss >> pos.x >> pos.y >> pos.z;
		ss >> quat.e0 >> quat.e1 >> quat.e2 >> quat.e3;
		ss >> pos_dt.x >> pos_dt.y >> pos_dt.z;
		ss >> wvel_loc.x >> wvel_loc.y >> wvel_loc.z;

		InitObject(body, mass, pos, quat, 1, 1, 0, true, false, 32, 17 + counter);
		body->SetIdentifier(counter);
		AddCollisionGeometry(body, SPHERE, Vector(radius, radius, radius), lpos, lquat);
		FinalizeObject(body, mSys);
		Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
		body->SetInertiaXX(inertia);
		body->SetPos_dt(pos_dt);
		body->SetWvel_loc(wvel_loc);
		body->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
		//body->GetCollisionModel()->SetDefaultSuggestedMargin(margin);
		//((ChModelBullet* ) body->GetCollisionModel())->GetBulletModel()->getCollisionShape()->setMargin(margin);
		counter++;
	}
	{
		ChQuaternion<> quat(1, 0, 0, 0);
		ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody);
		ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody);
		ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody);
		ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody);
		ChSharedBodyPtr BTM = ChSharedBodyPtr(new ChBody);
//
		float mWallMu = 1, container_width = 7.0, container_thickness = .25, container_height = 7.0, wscale = 1;
		InitObject(L, 100000, Vector(-container_width + container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(R, 100000, Vector(container_width - container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(F, 100000, Vector(0, 0, -container_width + container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(B, 100000, Vector(0, 0, container_width - container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(BTM, 100000, Vector(0, -container_height + container_thickness, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
//
		L->SetIdentifier(counter);
		R->SetIdentifier(counter + 1);
		F->SetIdentifier(counter + 2);
		B->SetIdentifier(counter + 3);
		BTM->SetIdentifier(counter + 4);

		AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
		AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
		AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
		AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
		AddCollisionGeometry(BTM, BOX, Vector(container_width * wscale, container_thickness, container_width), lpos, lquat);
//
		FinalizeObject(L, mSys);
		FinalizeObject(R, mSys);
		FinalizeObject(F, mSys);
		FinalizeObject(B, mSys);
		FinalizeObject(BTM, mSys);
//
		L->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
//		L->GetCollisionModel()->SetDefaultSuggestedMargin(margin);
		R->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
//		R->GetCollisionModel()->SetDefaultSuggestedMargin(margin);
		F->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
//		F->GetCollisionModel()->SetDefaultSuggestedMargin(margin);
		B->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
//		B->GetCollisionModel()->SetDefaultSuggestedMargin(margin);
		BTM->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
//		BTM->GetCollisionModel()->SetDefaultSuggestedMargin(margin);

	}

}

void LoadObjects_GPU(ChSystemGPU* mSys, string filename) {
	ifstream ifile(filename.c_str());
	ChSharedBodyGPUPtr body;
	int counter = 0;

	Vector pos, pos_dt, wvel_loc;
	Quaternion quat;
	string temp;
	while (ifile.fail() == false && counter < max_bodies) {
		getline(ifile, temp);
		if (ifile.fail() == true) {
			break;
		}
		body = ChSharedBodyGPUPtr(new ChBodyGPU);
#ifdef GPU_BULLET
		body->SetCollisionModelBullet();
#endif
		//cout << "GPU LOAD BODY: " << counter << endl;

		stringstream ss(temp);
		ss >> pos.x >> pos.y >> pos.z;
		ss >> quat.e0 >> quat.e1 >> quat.e2 >> quat.e3;
		ss >> pos_dt.x >> pos_dt.y >> pos_dt.z;
		ss >> wvel_loc.x >> wvel_loc.y >> wvel_loc.z;
		InitObject(body, mass, pos, quat, 1, 1, 0, true, false, 32, 17 + counter);
		AddCollisionGeometry(body, SPHERE, Vector(radius, radius, radius), lpos, lquat);
		FinalizeObject(body, mSys);
		Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
		body->SetInertiaXX(inertia);
		body->SetPos_dt(pos_dt);
		body->SetWvel_loc(wvel_loc);
#ifdef GPU_BULLET
		body->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
#endif
		counter++;
	}
	{
		ChQuaternion<> quat(1, 0, 0, 0);
		ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
		ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
		ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
		ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
		ChSharedBodyGPUPtr BTM = ChSharedBodyGPUPtr(new ChBodyGPU);
#ifdef GPU_BULLET
		L->SetCollisionModelBullet();
		R->SetCollisionModelBullet();
		F->SetCollisionModelBullet();
		B->SetCollisionModelBullet();
		BTM->SetCollisionModelBullet();
#endif
		float mWallMu = 1, container_width = 7.0, container_thickness = .25, container_height = 7.0, wscale = 1;
		InitObject(L, 100000, Vector(-container_width + container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(R, 100000, Vector(container_width - container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(F, 100000, Vector(0, 0, -container_width + container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(B, 100000, Vector(0, 0, container_width - container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(BTM, 100000, Vector(0, -container_height + container_thickness, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
//
		AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
		AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
		AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
		AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
		AddCollisionGeometry(BTM, BOX, Vector(container_width * wscale, container_thickness, container_width), lpos, lquat);
//
		FinalizeObject(L, mSys);
		FinalizeObject(R, mSys);
		FinalizeObject(F, mSys);
		FinalizeObject(B, mSys);
		FinalizeObject(BTM, mSys);

#ifdef GPU_BULLET

		L->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
		R->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
		F->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
		B->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
		BTM->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);

#endif
	}

}

template<class T>
void RunTimeStep(T* mSys, const int frame) {

}
void createGeometryCPU(ChSystem* mSys) {
	ChQuaternion<> quat(1, 0, 0, 0);
	ChSharedBodyPtr sphere;
	srand(1);
	for (int i = 0; i < 10000; i++) {
		sphere = ChSharedBodyPtr(new ChBody);
		real mass = 1;
		real radius = .1;
		//(rand() % 10000 / 1000.0 - 5)
		Vector pos = Vector((rand() % 10000 / 1000.0 - 5), (rand() % 10000 / 1000.0 - 4), (rand() % 10000 / 1000.0 - 5));
		InitObject(sphere, mass, pos, quat, 1, 1, 0, true, false, 32, 17 + i);
		AddCollisionGeometry(sphere, SPHERE, Vector(radius, radius, radius), lpos, lquat);
		Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);

		FinalizeObject(sphere, mSys);
		sphere->SetInertiaXX(inertia);
	}

	float mWallMu = 1, container_width = 7.0, container_thickness = .25, container_height = 7.0, wscale = 1;
	ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr BTM = ChSharedBodyPtr(new ChBody);

	InitObject(L, 100000, Vector(-container_width + container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(R, 100000, Vector(container_width - container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(F, 100000, Vector(0, 0, -container_width + container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(B, 100000, Vector(0, 0, container_width - container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(BTM, 100000, Vector(0, -container_height + container_thickness, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);

	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
	AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
	AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
	AddCollisionGeometry(BTM, BOX, Vector(container_width * wscale, container_thickness, container_width), lpos, lquat);

	FinalizeObject(L, mSys);
	FinalizeObject(R, mSys);
	FinalizeObject(F, mSys);
	FinalizeObject(B, mSys);
	FinalizeObject(BTM, mSys);

}

void createGeometryGPU(ChSystemGPU* mSys) {

	ChQuaternion<> quat(1, 0, 0, 0);
	ChSharedBodyGPUPtr sphere; //= ChSharedBodyGPUPtr(new ChBodyGPU);
	srand(1);
	for (int i = 0; i < 10000; i++) {
		real mass = 1;
		real radius = .1;
		sphere = ChSharedBodyGPUPtr(new ChBodyGPU);
		sphere->SetCollisionModelBullet();
		Vector pos = Vector((rand() % 10000 / 1000.0 - 5), (rand() % 10000 / 1000.0 - 4), (rand() % 10000 / 1000.0 - 5));
		InitObject(sphere, mass, pos, quat, 1, 1, 0, true, false, 32, 17 + i);
		AddCollisionGeometry(sphere, SPHERE, Vector(radius, radius, radius), lpos, quat);
		FinalizeObject(sphere, mSys);
		Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
		sphere->SetInertiaXX(inertia);
	}

	float mWallMu = 1, container_width = 7.0, container_thickness = .25, container_height = 7.0, wscale = 1;
	ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM = ChSharedBodyGPUPtr(new ChBodyGPU);

	L->SetCollisionModelBullet();
	R->SetCollisionModelBullet();
	F->SetCollisionModelBullet();
	B->SetCollisionModelBullet();
	BTM->SetCollisionModelBullet();

//
	InitObject(L, 100000, Vector(-container_width + container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(R, 100000, Vector(container_width - container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(F, 100000, Vector(0, 0, -container_width + container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(B, 100000, Vector(0, 0, container_width - container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(BTM, 100000, Vector(0, -container_height + container_thickness, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);

	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
	AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
	AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
	AddCollisionGeometry(BTM, BOX, Vector(container_width * wscale, container_thickness, container_width), lpos, lquat);

	FinalizeObject(L, mSys);
	FinalizeObject(R, mSys);
	FinalizeObject(F, mSys);
	FinalizeObject(B, mSys);
	FinalizeObject(BTM, mSys);

}

bool validate_ContactsBullet(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	cout << "validating contacts" << endl;
	btCollisionWorld* bt_collision_world_cpu = ((ChCollisionSystemBullet*) (system_cpu->GetCollisionSystem()))->GetBulletCollisionWorld();
	btCollisionWorld* bt_collision_world_gpu = ((ChCollisionSystemBulletGPU*) (system_gpu->GetCollisionSystem()))->GetBulletCollisionWorld();

	int numManifolds_cpu = bt_collision_world_cpu->getDispatcher()->getNumManifolds();
	int numManifolds_gpu = bt_collision_world_gpu->getDispatcher()->getNumManifolds();
	if (numManifolds_cpu != numManifolds_gpu) {
		cout << "NUM_MANIFOLDS ERROR" << endl;
		return false;
	}
	ChCollisionInfo icontact_cpu, icontact_gpu;

	for (int i = 0; i < numManifolds_cpu; i++) {
		btPersistentManifold* contactManifold_cpu = bt_collision_world_cpu->getDispatcher()->getManifoldByIndexInternal(i);
		btPersistentManifold* contactManifold_gpu = bt_collision_world_gpu->getDispatcher()->getManifoldByIndexInternal(i);

		btCollisionObject* obA_cpu = static_cast<btCollisionObject*>(contactManifold_cpu->getBody0());
		btCollisionObject* obB_cpu = static_cast<btCollisionObject*>(contactManifold_cpu->getBody1());

		btCollisionObject* obA_gpu = static_cast<btCollisionObject*>(contactManifold_gpu->getBody0());
		btCollisionObject* obB_gpu = static_cast<btCollisionObject*>(contactManifold_gpu->getBody1());

		contactManifold_cpu->refreshContactPoints(obA_cpu->getWorldTransform(), obB_cpu->getWorldTransform());
		contactManifold_gpu->refreshContactPoints(obA_gpu->getWorldTransform(), obB_cpu->getWorldTransform());

		icontact_cpu.modelA = (ChCollisionModel*) obA_cpu->getUserPointer();
		icontact_cpu.modelB = (ChCollisionModel*) obB_cpu->getUserPointer();

		icontact_gpu.modelA = (ChCollisionModel*) obA_gpu->getUserPointer();
		icontact_gpu.modelB = (ChCollisionModel*) obB_gpu->getUserPointer();

		double envelopeA_cpu = icontact_cpu.modelA->GetEnvelope();
		double envelopeB_cpu = icontact_cpu.modelB->GetEnvelope();

		double envelopeA_gpu = icontact_gpu.modelA->GetEnvelope();
		double envelopeB_gpu = icontact_gpu.modelB->GetEnvelope();

		double marginA_cpu = icontact_cpu.modelA->GetSafeMargin();
		double marginB_cpu = icontact_cpu.modelB->GetSafeMargin();

		double marginA_gpu = icontact_gpu.modelA->GetSafeMargin();
		double marginB_gpu = icontact_gpu.modelB->GetSafeMargin();

		int numContacts_cpu = contactManifold_cpu->getNumContacts();
		int numContacts_gpu = contactManifold_gpu->getNumContacts();
		if (numContacts_cpu != numContacts_gpu) {
			cout << "NUM_CONTACTS ERROR" << endl;
			return false;
		}

		for (int j = 0; j < numContacts_cpu; j++) {
			btManifoldPoint& pt_cpu = contactManifold_cpu->getContactPoint(j);
			btManifoldPoint& pt_gpu = contactManifold_gpu->getContactPoint(j);
			if (pt_cpu.getDistance() < marginA_cpu + marginB_cpu) // to discard "too far" constraints (the Bullet engine also has its threshold)
					{
				btVector3 ptA_diff = pt_cpu.getPositionWorldOnA() - pt_gpu.getPositionWorldOnA();
				btVector3 ptB_diff = pt_cpu.getPositionWorldOnB() - pt_gpu.getPositionWorldOnB();
				btVector3 Normaldiff = pt_cpu.m_normalWorldOnB - pt_gpu.m_normalWorldOnB;
				if (fabs(ptA_diff.x()) > tolerance) {
					cout << "ptA_diff x: [" << i << ", " << ptA_diff.x() << endl;
				}
				if (fabs(ptA_diff.y()) > tolerance) {
					cout << "ptA_diff y: [" << i << ", " << ptA_diff.y() << endl;
				}
				if (fabs(ptA_diff.z()) > tolerance) {
					cout << "ptA_diff z: [" << i << ", " << ptA_diff.z() << endl;
				}
				if (fabs(ptB_diff.x()) > tolerance) {
					cout << "ptB_diff x: [" << i << ", " << ptB_diff.x() << endl;
				}
				if (fabs(ptB_diff.y()) > tolerance) {
					cout << "ptB_diff y: [" << i << ", " << ptB_diff.y() << endl;
				}
				if (fabs(ptB_diff.z()) > tolerance) {
					cout << "ptB_diff z: [" << i << ", " << ptB_diff.z() << endl;
				}

				if (fabs(Normaldiff.x()) > tolerance) {
					cout << "Normaldiff x: [" << i << ", " << Normaldiff.x() << endl;
				}
				if (fabs(Normaldiff.y()) > tolerance) {
					cout << "Normaldiff y: [" << i << ", " << Normaldiff.y() << endl;
				}
				if (fabs(Normaldiff.z()) > tolerance) {
					cout << "Normaldiff z: [" << i << ", " << Normaldiff.z() << endl;
				}

			}

		}

	}
	cout << "validating contacts - PASSED" << endl;
	return true;
}

int main(int argc, char* argv[]) {

#ifndef __APPLE__
	feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

//#pragma omp single
//	{
//		omp_set_num_threads(4);
//	}
//cudaSetDevice(0);

	ChSystem * system_cpu = new ChSystem;
	{
		ChLcpSystemDescriptor *mdescriptor = new ChLcpSystemDescriptor();
		ChContactContainer *mcontactcontainer = new ChContactContainer();
		ChCollisionSystemBullet *mcollisionengine = new ChCollisionSystemBullet();
		//ChLcpIterativeJacobi *msolver = new ChLcpIterativeJacobi(120, false, 1e-4);
		ChLcpIterativeAPGD *msolver = new ChLcpIterativeAPGD();

		system_cpu->ChangeLcpSystemDescriptor(mdescriptor);
		system_cpu->ChangeContactContainer(mcontactcontainer);
		system_cpu->ChangeLcpSolverSpeed(msolver);
		system_cpu->ChangeCollisionSystem(mcollisionengine);
		system_cpu->SetIntegrationType(ChSystem::INT_ANITESCU);
		system_cpu->Set_G_acc(Vector(0, -9.80665, 0));
		system_cpu->SetStep(0.005);
		//createGeometryCPU(system_cpu);
		LoadObjects_CPU(system_cpu, "stack10000_bodies.txt");

		((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->SetMaxIterations(100);
		system_cpu->SetMaxiter(100);
		system_cpu->SetIterLCPmaxItersSpeed(100);
		system_cpu->SetTol(tolerance);
		system_cpu->SetTolSpeeds(tolerance);
		((ChLcpIterativeAPGD *) (system_cpu->GetLcpSolverSpeed()))->SetTolerance(tolerance);
		system_cpu->SetMaxPenetrationRecoverySpeed(.6);
	}

	ChSystemGPU * system_gpu = new ChSystemGPU;
	{
		ChLcpSystemDescriptorGPU *mdescriptor = new ChLcpSystemDescriptorGPU();
		ChContactContainerGPU *mcontactcontainer = new ChContactContainerGPU();
#ifdef GPU_BULLET
		ChCollisionSystemBulletGPU *mcollisionengine = new ChCollisionSystemBulletGPU();
#else
		ChCollisionSystemGPU *mcollisionengine = new ChCollisionSystemGPU();
#endif
		system_gpu->ChangeLcpSystemDescriptor(mdescriptor);
		system_gpu->ChangeContactContainer(mcontactcontainer);
		system_gpu->ChangeCollisionSystem(mcollisionengine);
		system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
		system_gpu->Set_G_acc(Vector(0, -9.80665, 0));
		system_gpu->SetStep(0.005);
		//-9.80665
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(100);
		system_gpu->SetMaxiter(100);
		system_gpu->SetIterLCPmaxItersSpeed(100);
		system_gpu->SetTol(tolerance);
		system_gpu->SetTolSpeeds(tolerance);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(tolerance);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(.6);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);
		((ChCollisionSystemGPU *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(envelope);

		//createGeometryGPU(system_gpu);
		//100_bodies
		LoadObjects_GPU(system_gpu, "stack10000_bodies.txt");
	}
//
//	ChOpenGLManager * window_manager = new ChOpenGLManager();
//	ChOpenGL openGLView(window_manager, system_cpu, 800, 600, 0, 0, "Test_Solvers");
//	//openGLView.AddSystem(system_cpu);
//	openGLView.SetCustomCallback(RunTimeStep);
//	openGLView.StartSpinning(window_manager);
//	window_manager->CallGlutMainLoop();

	int counter = 0;
	while (counter < 1) {

		RunTimeStep(system_cpu, counter);
		RunTimeStep(system_gpu, counter);

		validate_positions(system_cpu, system_gpu, tolerance);
		validate_rotations(system_cpu, system_gpu, tolerance);
		validate_velocities(system_cpu, system_gpu, tolerance);
		validate_omega(system_cpu, system_gpu, tolerance);
		cout << "CPU: =============" << endl;
		system_cpu->DoStepDynamics(.005);
		cout << "ITER: " << ((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->GetTotalIterations() << endl;
		cout << "Residual: " << ((ChLcpIterativeAPGD *) (system_cpu->GetLcpSolverSpeed()))->GetResidual() << endl;
		cout << "GPU: =============" << endl;
		system_gpu->DoStepDynamics(.005);
		cout << "ITER: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations() << endl;
		cout << "Residual: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetResidual() << endl;
		cout << "=============" << endl;

		//		validate_ContactsBullet(system_cpu, system_gpu);
		printContactsBullet(((ChCollisionSystemBullet*) (system_cpu->GetCollisionSystem()))->GetBulletCollisionWorld(), contact_cpu);
		printContactsGPU(system_gpu, contact_gpu);
		comparecontacts(contact_cpu, contact_gpu, tolerance);
		validate_positions(system_cpu, system_gpu, tolerance);
		validate_rotations(system_cpu, system_gpu, tolerance);
		validate_velocities(system_cpu, system_gpu, tolerance);
		validate_omega(system_cpu, system_gpu, tolerance);
		//validate_jacobians(system_cpu, system_gpu,tolerance);
		cout << "DONE: =============" << counter << endl;

//		btCollisionWorld* bt_collision_world_cpu = ((ChCollisionSystemBullet*) (system_cpu->GetCollisionSystem()))->GetBulletCollisionWorld();
//		btCollisionWorld* bt_collision_world_gpu = ((ChCollisionSystemBulletGPU*) (system_gpu->GetCollisionSystem()))->GetBulletCollisionWorld();
//		bt_collision_world_cpu->performDiscreteCollisionDetection();
//		bt_collision_world_gpu->performDiscreteCollisionDetection();
//		printContacts(bt_collision_world_cpu);
//		printContacts(bt_collision_world_gpu);

//		cout<<"CPUL: ============="<<endl;
//		((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->Dump_Lambda(cout);
//		cout<<"GPUL: ============="<<endl;
//		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->Dump_Lambda(cout);

//		cout << "Ncontacts: "<< system_gpu->GetNcontacts() << endl;
//		btCollisionWorld* bt_collision_world_gpu = ((ChCollisionSystemBulletGPU*) (system_gpu->GetCollisionSystem()))->GetBulletCollisionWorld();
//		cout << "GPUP: =============" << endl;
//		for (int i = 0; i < system_gpu->Get_bodylist()->size(); i++) {
//			ChBody* abody = (ChBody*) system_gpu->Get_bodylist()->at(i);
//			btVector3 pos = bt_collision_world_gpu->getCollisionObjectArray().at(i)->getWorldTransform().getOrigin();
//			cout << abody->GetPos().x-pos.x() << " " << abody->GetPos().y-pos.y() << " " << abody->GetPos().z-pos.z() << endl;
//		}
//
//		cout << "CPUCBP: =============" << endl;
//		btCollisionWorld* bt_collision_world_cpu = ((ChCollisionSystemBullet*) (system_cpu->GetCollisionSystem()))->GetBulletCollisionWorld();
//
//		for (int i = 0; i < bt_collision_world_cpu->getCollisionObjectArray().size(); i++) {
//			btVector3 pos = bt_collision_world_cpu->getCollisionObjectArray().at(i)->getWorldTransform().getOrigin();
//			cout << pos.x() << " " << pos.y() << " " << pos.z() << endl;
//		}
//
		counter++;

	}

//while (counter < 1000) {
//    system_gpu->DoStepDynamics(.005);
//    cout<<counter<<endl;
//    counter++;
//}
//    
//    
//	DumpObjects(system_gpu, "stack10000_bodies.txt");

	return 0;
}

