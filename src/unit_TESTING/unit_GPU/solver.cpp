#define THRUST_DEBUG 1

#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativeAPGD.h"
#include "lcp/ChLcpIterativeBB.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "core/ChLinearAlgebra.h"
#include "physics/ChSystemOpenMP.h"
#include "assets/ChSphereShape.h"
#include "physics/ChApidll.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "collision/ChCModelBullet.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "physics/ChContactContainer.h"
#include "unit_OPENGL/ChOpenGL.h"

#include "unit_GPU/ChSystemGPU.h"

// Remember to use the namespace 'chrono' because all classes
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
uint num_objects = 0;

template<class T>
void RunTimeStep(T* mSys, const int frame) {
}
void createGeometryCPU(ChSystem* mSys) {
	Vector lpos(0, 0, 0);
	ChQuaternion<> quat(1, 0, 0, 0);
	ChSharedBodyPtr sphere;
	srand(1);
	for (int i = 0; i < 2; i++) {
		sphere = ChSharedBodyPtr(new ChBody);
		real mass = 1;
		real radius = .3;
		//(rand() % 10000 / 1000.0 - 5)
		Vector pos = Vector(0,i, 0);
		InitObject(sphere, mass, pos, quat, 1, 1, 0, true, false, 32, 17 + i);
		AddCollisionGeometry(sphere, SPHERE, Vector(radius, radius, radius), lpos, quat);
		Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);

		FinalizeObject(sphere, mSys);
		sphere->SetInertiaXX(inertia);
		sphere->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
		sphere->GetCollisionModel()->SetDefaultSuggestedMargin(0);
	}

//	sphere = ChSharedBodyPtr(new ChBody);
////	sphere->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
////	sphere->GetCollisionModel()->SetDefaultSuggestedMargin(0);
//	InitObject(sphere, 1.0, Vector(0, 0, 0), quat, 1, 1, 0, true, false, 32, 17);
//	AddCollisionGeometry(sphere, SPHERE, Vector(.3, .3, .3), lpos, quat);
//	Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
//	FinalizeObject(sphere, mSys);
//	sphere->SetInertiaXX(inertia);
//	sphere->SetPos_dt(Vector(0,0,1));


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

	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(BTM, BOX, Vector(container_width * wscale, container_thickness, container_width), lpos, quat);

//	L->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
//	L->GetCollisionModel()->SetDefaultSuggestedMargin(0);
//
//	R->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
//	R->GetCollisionModel()->SetDefaultSuggestedMargin(0);
//
//	F->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
//	F->GetCollisionModel()->SetDefaultSuggestedMargin(0);
//
//	B->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
//	B->GetCollisionModel()->SetDefaultSuggestedMargin(0);
//
//	BTM->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
//	BTM->GetCollisionModel()->SetDefaultSuggestedMargin(0);

//	FinalizeObject(L, mSys);
//	FinalizeObject(R, mSys);
//	FinalizeObject(F, mSys);
//	FinalizeObject(B, mSys);
	FinalizeObject(BTM, mSys);

}

void createGeometryGPU(ChSystemGPU* mSys) {
	Vector lpos(0, 0, 0);
	ChQuaternion<> quat(1, 0, 0, 0);
	ChSharedBodyGPUPtr sphere; //= ChSharedBodyGPUPtr(new ChBodyGPU);
	srand(1);
	for (int i = 0; i < 2; i++) {
		real mass = 1;
		real radius = .3;
		sphere = ChSharedBodyGPUPtr(new ChBodyGPU);
		sphere->SetCollisionModelBullet();
		Vector pos = Vector(0,i, 0);
		InitObject(sphere, mass,pos , quat, 1, 1, 0, true, false, 32, 17 + i);
		AddCollisionGeometry(sphere, SPHERE, Vector(radius, radius, radius), lpos, quat);
		FinalizeObject(sphere, mSys);
		Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
		sphere->SetInertiaXX(inertia);
		sphere->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
		sphere->GetCollisionModel()->SetDefaultSuggestedMargin(0);
	}
//	sphere = ChSharedBodyGPUPtr(new ChBodyGPU);
//	sphere->SetCollisionModelBullet();

//	InitObject(sphere, 1.0, Vector(0, 0, 0), quat, 1, 1, 0, true, false, 32, 17);
//	AddCollisionGeometry(sphere, SPHERE, Vector(.3, .3, .3), lpos, quat);
//	FinalizeObject(sphere, mSys);
//	Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
//	sphere->SetInertiaXX(inertia);
//	sphere->SetPos_dt(Vector(0,0,1));

	float mWallMu = 1, container_width = 7.0, container_thickness = .25, container_height = 7.0, wscale = 1;
//	ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
//	ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
//	ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
//	ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM = ChSharedBodyGPUPtr(new ChBodyGPU);

//	L->SetCollisionModelBullet();
//	R->SetCollisionModelBullet();
//	F->SetCollisionModelBullet();
//	B->SetCollisionModelBullet();
	BTM->SetCollisionModelBullet();

//
//	InitObject(L, 100000, Vector(-container_width + container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
//	InitObject(R, 100000, Vector(container_width - container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
//	InitObject(F, 100000, Vector(0, 0, -container_width + container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
//	InitObject(B, 100000, Vector(0, 0, container_width - container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(BTM, 100000, Vector(0, -container_height + container_thickness, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);

//	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
//	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
//	AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
//	AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(BTM, BOX, Vector(container_width * wscale, container_thickness, container_width), lpos, quat);

	//FinalizeObject(L, mSys);
	//FinalizeObject(R, mSys);
	//FinalizeObject(F, mSys);
	//FinalizeObject(B, mSys);

	FinalizeObject(BTM, mSys);

}
bool validate(ChSystem* cpu_system, ChSystemGPU* gpu_system) {
	cout << "Nbodies: " << cpu_system->GetNbodiesTotal() << "\t" << gpu_system->GetNbodiesTotal() << endl;
	cout << "Ncontacts: " << cpu_system->GetNcontacts() << "\t" << gpu_system->GetNcontacts() << endl;



//	if (cpu_system->GetNbodiesTotal() != gpu_system->GetNbodiesTotal()) {
//		return false;
//	}
//
//	if(cpu_system->GetNcontacts()>0){exit(1);}
//	if (cpu_system->GetNcontacts() != gpu_system->GetNcontacts()) {
//		return false;
//	}

	vector<ChCollisionInfo> icontactlist_cpu, icontactlist_gpu;
	ChCollisionInfo icontact;
	{
		btCollisionWorld* bt_collision_world = ((ChCollisionSystemBullet*) (cpu_system->GetCollisionSystem()))->GetBulletCollisionWorld();
		//bt_collision_world->performDiscreteCollisionDetection();
		int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();

		for (int i = 0; i < numManifolds; i++) {
			btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
			btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
			contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

			icontact.modelA = (ChCollisionModel*) obA->getUserPointer();
			icontact.modelB = (ChCollisionModel*) obB->getUserPointer();

			double envelopeA = icontact.modelA->GetEnvelope();
			double envelopeB = icontact.modelB->GetEnvelope();

			double marginA = icontact.modelA->GetSafeMargin();
			double marginB = icontact.modelB->GetSafeMargin();

			int numContacts = contactManifold->getNumContacts();

			for (int j = 0; j < numContacts; j++) {
				btManifoldPoint& pt = contactManifold->getContactPoint(j);

				if (pt.getDistance() < marginA + marginB) {

					btVector3 ptA = pt.getPositionWorldOnA();
					btVector3 ptB = pt.getPositionWorldOnB();

					icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
					icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());

					icontact.vN.Set(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(), -pt.m_normalWorldOnB.getZ());
					icontact.vN.Normalize();

					double ptdist = pt.getDistance();

					icontact.vpA = icontact.vpA - icontact.vN * envelopeA;
					icontact.vpB = icontact.vpB + icontact.vN * envelopeB;
					icontact.distance = ptdist + envelopeA + envelopeB;

					icontact.reaction_cache = pt.reactions_cache;
					cout<<"dist_cpu "<<icontact.distance<<endl;
					//icontactlist_cpu.push_back(icontact);

				}
			}
		}
	}
	{
		btCollisionWorld* bt_collision_world = ((ChCollisionSystemBulletGPU*) (gpu_system->GetCollisionSystem()))->GetBulletCollisionWorld();
		//bt_collision_world->performDiscreteCollisionDetection();
		int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();

		for (int i = 0; i < numManifolds; i++) {
			btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
			btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
			contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

			icontact.modelA = (ChCollisionModel*) obA->getUserPointer();
			icontact.modelB = (ChCollisionModel*) obB->getUserPointer();

			double envelopeA = icontact.modelA->GetEnvelope();
			double envelopeB = icontact.modelB->GetEnvelope();

			double marginA = icontact.modelA->GetSafeMargin();
			double marginB = icontact.modelB->GetSafeMargin();

			int numContacts = contactManifold->getNumContacts();

			for (int j = 0; j < numContacts; j++) {
				btManifoldPoint& pt = contactManifold->getContactPoint(j);

				if (pt.getDistance() < marginA + marginB) {

					btVector3 ptA = pt.getPositionWorldOnA();
					btVector3 ptB = pt.getPositionWorldOnB();

					icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
					icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());

					icontact.vN.Set(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(), -pt.m_normalWorldOnB.getZ());
					icontact.vN.Normalize();

					double ptdist = pt.getDistance();

					icontact.vpA = icontact.vpA - icontact.vN * envelopeA;
					icontact.vpB = icontact.vpB + icontact.vN * envelopeB;
					icontact.distance = ptdist + envelopeA + envelopeB;
					cout<<"dist_gpu "<<icontact.distance<<endl;
					icontact.reaction_cache = pt.reactions_cache;

					//icontactlist_gpu.push_back(icontact);

				}
			}
		}
	}



//	real tolerance = 1e-6;
//cout<<icontactlist_cpu.size()<<" "<<icontactlist_gpu.size()<<endl;
//
//if(icontactlist_cpu.size()!=icontactlist_gpu.size()){return false;}
//
//	for (int i = 0; i < icontactlist_cpu.size(); i++) {
//		real distance = icontactlist_cpu[i].distance - icontactlist_gpu[i].distance;
//		Vector vpA = icontactlist_cpu[i].vpA - icontactlist_gpu[i].vpA;
//		Vector vpB = icontactlist_cpu[i].vpB - icontactlist_gpu[i].vpB;
//		Vector vN = icontactlist_cpu[i].vN - icontactlist_gpu[i].vN;
//
//		if (fabs(distance) > tolerance) {
//			cout << "Dist Fail " << i << endl;
//			return false;
//		}
////		if (fabs(vN.Length()) > tolerance) {
////			cout << "Norm Fail " << i << endl;
////			return false;
////		}
////		if (fabs(vpA.Length()) > tolerance) {
////			cout << "vpA Fail " << i << endl;
////			return false;
////		}
////		if (fabs(vpB.Length()) > tolerance) {
////			cout << "vpB Fail " << i << endl;
////			return false;
////		}
//	}

}

int main(int argc, char* argv[]) {
	feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
	//omp_set_num_threads(3);
	//cudaSetDevice(0);

	//cpu code
	ChSystem * system_cpu = new ChSystem;
	{
		ChLcpSystemDescriptor *mdescriptor = new ChLcpSystemDescriptor();
		ChContactContainer *mcontactcontainer = new ChContactContainer();
		ChCollisionSystemBullet *mcollisionengine = new ChCollisionSystemBullet();
		ChLcpIterativeAPGD *msolver = new ChLcpIterativeAPGD();

		system_cpu->ChangeLcpSystemDescriptor(mdescriptor);
		system_cpu->ChangeContactContainer(mcontactcontainer);
		system_cpu->ChangeLcpSolverSpeed(msolver);
		system_cpu->ChangeCollisionSystem(mcollisionengine);
		system_cpu->SetIntegrationType(ChSystem::INT_ANITESCU);
		system_cpu->Set_G_acc(Vector(0, -9.80665, 0));
		system_cpu->SetStep(0.01);
		createGeometryCPU(system_cpu);
		((ChLcpIterativeSolver *) (system_cpu->GetLcpSolverSpeed()))->SetMaxIterations(120);
		system_cpu->SetMaxPenetrationRecoverySpeed(.6);
	}

	//gpu code
	ChSystemGPU * system_gpu = new ChSystemGPU;
	{
		ChLcpSystemDescriptorGPU *mdescriptor = new ChLcpSystemDescriptorGPU();
		ChContactContainerGPU *mcontactcontainer = new ChContactContainerGPU();
		ChCollisionSystemBulletGPU *mcollisionengine = new ChCollisionSystemBulletGPU();
		//ChCollisionSystemGPU *mcollisionengine = new ChCollisionSystemGPU();

		system_gpu->ChangeLcpSystemDescriptor(mdescriptor);
		system_gpu->ChangeContactContainer(mcontactcontainer);
		system_gpu->ChangeCollisionSystem(mcollisionengine);
		system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
		system_gpu->Set_G_acc(Vector(0, -9.80665, 0));
		system_gpu->SetStep(0.01);

		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(120);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(1e-8);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(.6);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);
		//((ChCollisionSystemGPU *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(0.0);

		createGeometryGPU(system_gpu);
	}

	ChOpenGLManager * window_manager = new ChOpenGLManager();
	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
	openGLView.AddSystem(system_cpu);
	openGLView.SetCustomCallback(RunTimeStep);
	openGLView.StartSpinning(window_manager);
	window_manager->CallGlutMainLoop();

	int counter = 0;

	while (counter < 113) {

		RunTimeStep(system_cpu, counter);
		RunTimeStep(system_gpu, counter);

		system_cpu->DoStepDynamics(.01);
		system_gpu->DoStepDynamics(.01);
		cout << "Ncontacts: " << system_cpu->GetNcontacts() << "\t" << system_gpu->GetNcontacts() << endl;
//		cout<<"CPUR: ============="<<endl;
//		((ChLcpIterativeAPGD *) (system_cpu->GetLcpSolverSpeed()))->Dump_Rhs(cout);
//		cout<<"GPUR: ============="<<endl;
//		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->Dump_Rhs(cout);
//		cout<<"CPUL: ============="<<endl;
//		((ChLcpIterativeAPGD *) (system_cpu->GetLcpSolverSpeed()))->Dump_Lambda(cout);
//		cout<<"GPUL: ============="<<endl;
//		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->Dump_Lambda(cout);

		cout<<"CPUP: ============="<<endl;
		for(int i=0; i<system_cpu->Get_bodylist()->size(); i++){
			ChBody* abody = (ChBody*) system_cpu->Get_bodylist()->at(i);
			cout<<abody->GetPos().x<<" "<<abody->GetPos().y<<" "<<abody->GetPos().z<<" "<<endl;
		}
		cout<<"GPUP: ============="<<endl;
		for(int i=0; i<system_gpu->Get_bodylist()->size(); i++){
			ChBody* abody = (ChBody*) system_gpu->Get_bodylist()->at(i);
			cout<<abody->GetPos().x<<" "<<abody->GetPos().y<<" "<<abody->GetPos().z<<" "<<endl;
		}
		validate(system_cpu, system_gpu);

		cout<<"DONE: ============="<<counter<<endl;

//		cout<< ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetCurrentIteration();
//		cout<<" "<<((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetResidual()<<endl;

//
//		stringstream ss;
//		ss << "data/" << counter << ".xml";
//		//output.ExportData(ss.str());
//
//		cout << counter << endl;
		counter++;
	}
	return 0;
}

