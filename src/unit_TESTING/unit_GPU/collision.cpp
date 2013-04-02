#define THRUST_DEBUG 1

#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativePMINRES.h"
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
	for (int i = 0; i < 100; i++) {
		sphere = ChSharedBodyPtr(new ChBody);
		InitObject(sphere, 1.0, Vector((rand() % 10000 / 1000.0 - 5), (rand() % 10000 / 1000.0 - 5), (rand() % 10000 / 1000.0 - 5)), quat, 1, 1, 0, true, false, 32, 17 + i);
		AddCollisionGeometry(sphere, SPHERE, Vector(.3, .3, .3), lpos, quat);
		sphere->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
		sphere->GetCollisionModel()->SetDefaultSuggestedMargin(0);

		FinalizeObject(sphere, mSys);
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

	AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(BTM, BOX, Vector(container_width * wscale, container_thickness, container_width), lpos, quat);

	L->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
	L->GetCollisionModel()->SetDefaultSuggestedMargin(0);

	R->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
	R->GetCollisionModel()->SetDefaultSuggestedMargin(0);

	F->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
	F->GetCollisionModel()->SetDefaultSuggestedMargin(0);

	B->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
	B->GetCollisionModel()->SetDefaultSuggestedMargin(0);

	BTM->GetCollisionModel()->SetDefaultSuggestedEnvelope(0);
	BTM->GetCollisionModel()->SetDefaultSuggestedMargin(0);

	FinalizeObject(L, mSys);
	FinalizeObject(R, mSys);
	FinalizeObject(F, mSys);
	FinalizeObject(B, mSys);
	FinalizeObject(BTM, mSys);

}

void createGeometryGPU(ChSystemGPU* mSys) {
	Vector lpos(0, 0, 0);
	ChQuaternion<> quat(1, 0, 0, 0);
	ChSharedBodyGPUPtr sphere;
	for (int i = 0; i < 100; i++) {
		sphere = ChSharedBodyGPUPtr(new ChBodyGPU);
		InitObject(sphere, 1.0, Vector((rand() % 10000 / 1000.0 - 5), (rand() % 10000 / 1000.0 - 5), (rand() % 10000 / 1000.0 - 5)), quat, 1, 1, 0, true, false, 32, 17 + i);
		AddCollisionGeometry(sphere, SPHERE, Vector(.3, .3, .3), lpos, quat);
		FinalizeObject(sphere, mSys);
	}

	float mWallMu = 1, container_width = 7.0, container_thickness = .25, container_height = 7.0, wscale = 1;
	ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
	ChSharedBodyGPUPtr BTM = ChSharedBodyGPUPtr(new ChBodyGPU);

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

	FinalizeObject(L, mSys);
	FinalizeObject(R, mSys);
	FinalizeObject(F, mSys);
	FinalizeObject(B, mSys);
	FinalizeObject(BTM, mSys);

}
bool validate(ChSystem* cpu_system, ChSystemGPU* gpu_system) {
	cout << "Nbodies: " << cpu_system->GetNbodiesTotal() << "\t" << gpu_system->GetNbodiesTotal() << endl;
	cout << "Ncontacts: " << cpu_system->GetNcontacts() << "\t" << gpu_system->GetNcontacts() << endl;

	btBroadphasePairArray broadphase_array = ((ChCollisionSystemBullet *) (cpu_system->GetCollisionSystem()))->GetBulletCollisionWorld()->getPairCache()->getOverlappingPairArray();
	for (int i = 0; i < broadphase_array.size(); i++) {
		cout << broadphase_array[i].m_pProxy0->m_uniqueId << " ";
		cout << broadphase_array[i].m_pProxy1->m_uniqueId << endl;
	}
	vector<int2> pairs = ((ChCollisionSystemGPU *) (gpu_system->GetCollisionSystem()))->GetOverlappingPairs();
	for (int i = 0; i < pairs.size(); i++) {
		cout << pairs[i].x << " " << pairs[i].y << endl;

	}

}

int main(int argc, char* argv[]) {
	feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
	omp_set_num_threads(3);
//cudaSetDevice(0);

//cpu code
	ChSystem * system_cpu = new ChSystem;
	{
		ChLcpSystemDescriptor *mdescriptor = new ChLcpSystemDescriptor();
		ChContactContainer *mcontactcontainer = new ChContactContainer();
		ChCollisionSystemBullet *mcollisionengine = new ChCollisionSystemBullet();
		ChLcpIterativeJacobi *msolver = new ChLcpIterativeJacobi();

		system_cpu->ChangeLcpSystemDescriptor(mdescriptor);
		system_cpu->ChangeContactContainer(mcontactcontainer);
		system_cpu->ChangeLcpSolverSpeed(msolver);
		system_cpu->ChangeCollisionSystem(mcollisionengine);
		system_cpu->SetIntegrationType(ChSystem::INT_ANITESCU);
		system_cpu->Set_G_acc(Vector(0, -9.80665, 0));
		system_cpu->SetStep(0.01);
		createGeometryCPU(system_cpu);
		((ChLcpIterativeSolver *) (system_cpu->GetLcpSolverSpeed()))->SetMaxIterations(120);
	}

//gpu code
	ChSystemGPU * system_gpu = new ChSystemGPU;
	{
		ChLcpSystemDescriptorGPU *mdescriptor = new ChLcpSystemDescriptorGPU();
		ChContactContainerGPUsimple *mcontactcontainer = new ChContactContainerGPUsimple();
		ChCollisionSystemGPU *mcollisionengine = new ChCollisionSystemGPU();

		system_gpu->ChangeLcpSystemDescriptor(mdescriptor);
		system_gpu->ChangeContactContainer(mcontactcontainer);
		system_gpu->ChangeCollisionSystem(mcollisionengine);
		system_gpu->SetIntegrationType(ChSystem::INT_ANITESCU);
		system_gpu->Set_G_acc(Vector(0, -9.80665, 0));
		system_gpu->SetStep(0.01);
		createGeometryGPU(system_gpu);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(120);

	}

//	ChOpenGLManager * window_manager = new ChOpenGLManager();
//	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
//	openGLView.SetCustomCallback(RunTimeStep);
//	openGLView.StartSpinning(window_manager);
//	window_manager->CallGlutMainLoop();

	int counter = 0;

	while (counter < 2) {
		//RunTimeStepCPU(system_cpu, counter);
		//RunTimeStepGPU(system_gpu, counter);
		system_cpu->DoStepDynamics(.01);
		system_gpu->DoStepDynamics(.01);

		validate(system_cpu, system_gpu);

		stringstream ss;
		ss << "data/" << counter << ".xml";
		//output.ExportData(ss.str());

		cout << counter << endl;
		counter++;
	}
	return 0;
}

