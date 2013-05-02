#include "common.h"

vector<contact_dat> contact_cpu, contact_gpu;
real tolerance = 0;
real step_size = .001;
real solver_tolerance = 1e-5;
uint solver_iter = 100;
real mass = 1;
real radius = .1;
Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
int max_bodies = 10000;
real envelope = .01;
//#define GPU_BULLET 1

template<class T>
void RunTimeStep(T* mSys, const int frame) {

	cout << "Residual: " << ((ChLcpSolverGPU *) (mSys->GetLcpSolverSpeed()))->GetResidual() << endl;
	cout << "ITER: " << ((ChLcpSolverGPU *) (mSys->GetLcpSolverSpeed()))->GetTotalIterations() << endl;

}

template<class T>
void ReadLine(string temp, T & body, int counter) {
	Quaternion quat;
	Vector pos, pos_dt, wvel_loc;
	stringstream ss(temp);
	ss >> pos.x >> pos.y >> pos.z >> quat.e0 >> quat.e1 >> quat.e2 >> quat.e3 >> pos_dt.x >> pos_dt.y >> pos_dt.z >> wvel_loc.x >> wvel_loc.y >> wvel_loc.z;
	body->SetPos(pos);
	body->SetRot(quat);
	body->SetPos_dt(pos_dt);
	body->SetWvel_loc(wvel_loc);
	body->SetInertiaXX(inertia);
	body->SetIdentifier(counter);
}
template<class T, class U>
void GenerateContainer(T& L, T& R, T& F, T& B, T& BTM, U* mSys, int counter) {
	ChQuaternion<> quat(1, 0, 0, 0);
	float mWallMu = 1, container_width = 7.0, container_thickness = .25, container_height = 7.0, wscale = 1;
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

	L->SetIdentifier(counter);
	R->SetIdentifier(counter + 1);
	F->SetIdentifier(counter + 2);
	B->SetIdentifier(counter + 3);
	BTM->SetIdentifier(counter + 4);

}
void LoadObjects(ChSystem* mSys, string filename) {
	ifstream ifile(filename.c_str());
	ChSharedBodyPtr body;
	int counter = 0;

	string temp;
	while (ifile.fail() == false && counter < max_bodies) {
		getline(ifile, temp);
		if (ifile.fail() == true) {
			break;
		}
		body = ChSharedBodyPtr(new ChBody);
		InitObject(body, mass, Vector(0, 0, 0), Quaternion(1, 0, 0, 0), 1, 1, 0, true, false, 32, 17 + counter);
		AddCollisionGeometry(body, SPHERE, Vector(radius, radius, radius), lpos, lquat);
		FinalizeObject(body, mSys);
		ReadLine(temp, body, counter);
		body->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
		counter++;
	}

	ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr BTM = ChSharedBodyPtr(new ChBody);

	GenerateContainer(L, R, F, B, BTM, mSys, counter);

	L->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
	R->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
	F->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
	B->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
	BTM->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);

}
void LoadObjects(ChSystemGPU* mSys, string filename) {
	ifstream ifile(filename.c_str());
	ChSharedBodyGPUPtr body;
	int counter = 0;

	string temp;
	while (ifile.fail() == false && counter < max_bodies) {
		getline(ifile, temp);
		if (ifile.fail() == true) {
			break;
		}
		body = ChSharedBodyGPUPtr(new ChBodyGPU);
#ifdef GPU_BULLET
		body->SetCollisionModelBullet();
		//body->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
#endif
		InitObject(body, mass, Vector(0, 0, 0), Quaternion(1, 0, 0, 0), 1, 1, 0, true, false, 32, 17 + counter);
		AddCollisionGeometry(body, SPHERE, Vector(radius, radius, radius), lpos, lquat);
		FinalizeObject(body, mSys);
		ReadLine(temp, body, counter);
		counter++;
	}
	cout << "Done oading" << endl;
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
	//L->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
	//R->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
	//F->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
	//B->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
	//BTM->GetCollisionModel()->SetDefaultSuggestedEnvelope(envelope);
#endif

	GenerateContainer(L, R, F, B, BTM, mSys, counter);

}
int main(int argc, char* argv[]) {
	omp_set_num_threads(3);

	ChSystem * system_cpu = new ChSystem;
	{
		ChLcpSystemDescriptor *mdescriptor = new ChLcpSystemDescriptor();
		ChContactContainer *mcontactcontainer = new ChContactContainer();
		ChCollisionSystemBullet *mcollisionengine = new ChCollisionSystemBullet();
		ChLcpIterativeAPGD *msolver = new ChLcpIterativeAPGD(100, false, 1e-4);
		//ChLcpIterativeJacobi *msolver = new ChLcpIterativeJacobi(100, false, 1e-4);

		system_cpu->ChangeLcpSystemDescriptor(mdescriptor);
		system_cpu->ChangeContactContainer(mcontactcontainer);
		system_cpu->ChangeLcpSolverSpeed(msolver);
		system_cpu->ChangeCollisionSystem(mcollisionengine);
		system_cpu->SetIntegrationType(ChSystem::INT_ANITESCU);
		system_cpu->Set_G_acc(Vector(0, -9.80665, 0));
		system_cpu->SetStep(step_size);

		((ChLcpIterativeSolver *) (system_cpu->GetLcpSolverSpeed()))->SetMaxIterations(solver_iter);
		system_cpu->SetMaxiter(solver_iter);
		system_cpu->SetIterLCPmaxItersSpeed(solver_iter);
		system_cpu->SetTol(solver_tolerance);
		system_cpu->SetTolSpeeds(solver_tolerance);
		((ChLcpIterativeSolver *) (system_cpu->GetLcpSolverSpeed()))->SetTolerance(solver_tolerance);
		system_cpu->SetMaxPenetrationRecoverySpeed(.6);

		//createGeometryCPU(system_cpu);
		//LoadObjects(system_cpu, "stack10000_bodies.txt");
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
		system_gpu->SetStep(step_size);
		//-9.80665
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(solver_iter);
		system_gpu->SetMaxiter(solver_iter);
		system_gpu->SetIterLCPmaxItersSpeed(solver_iter);
		system_gpu->SetTol(solver_tolerance);
		system_gpu->SetTolSpeeds(solver_tolerance);

		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(solver_tolerance);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(.6);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);
		//((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(BLOCK_JACOBI);
		((ChCollisionSystemGPU *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(envelope);

		Quaternion quat(1,0,0,0);

		quat.Set(ChRandom(), ChRandom(), ChRandom(), ChRandom());
				quat.Normalize();
		real rad_a = .25;
		real rad_b = .25;
		real rad_c = 1;

		ChSharedBodyGPUPtr body;
		for (int k = 0; k < 1; k++) {
		for (int i = 0; i < 1; i++) {
			for (int j = 0; j < 2; j++) {

				body = ChSharedBodyGPUPtr(new ChBodyGPU);
				InitObject(body, mass, Vector(i/2.0, -1, j/2.0), quat, 1, 1, 0, true, false, 0, 1);
				AddCollisionGeometry(body, ELLIPSOID, Vector(rad_a, rad_b, rad_c), lpos, QUNIT);
				body->SetInertiaXX((1.0 / 5.0) * mass * ChVector<>(rad_b * rad_b + rad_c * rad_c, rad_c * rad_c + rad_a * rad_a, rad_a * rad_a + rad_b * rad_b));
				FinalizeObject(body, (ChSystemGPU*) system_gpu);
			}
		}
		}
		ChSharedBodyGPUPtr L = ChSharedBodyGPUPtr(new ChBodyGPU);
		ChSharedBodyGPUPtr R = ChSharedBodyGPUPtr(new ChBodyGPU);
		ChSharedBodyGPUPtr F = ChSharedBodyGPUPtr(new ChBodyGPU);
		ChSharedBodyGPUPtr B = ChSharedBodyGPUPtr(new ChBodyGPU);
		ChSharedBodyGPUPtr BTM = ChSharedBodyGPUPtr(new ChBodyGPU);

		//		L->SetCollisionModelBullet();
		//		R->SetCollisionModelBullet();
		//		F->SetCollisionModelBullet();
		//		B->SetCollisionModelBullet();
		//		BTM->SetCollisionModelBullet();

		float mWallMu = 1, container_width = 20.0, container_thickness = .25, container_height = 7.0, wscale = 1;
		InitObject(L, 100000, Vector(-container_width + container_thickness, 0, 0), QUNIT, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(R, 100000, Vector(container_width - container_thickness, 0, 0), QUNIT, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(F, 100000, Vector(0, 0, -container_width + container_thickness), QUNIT, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(B, 100000, Vector(0, 0, container_width - container_thickness), QUNIT, mWallMu, mWallMu, 0, true, true, -20, -20);
		InitObject(BTM, 100000, Vector(0, -container_height + container_thickness, 0), QUNIT, mWallMu, mWallMu, 0, true, true, -20, -20);

		AddCollisionGeometry(L, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
		AddCollisionGeometry(R, BOX, Vector(container_thickness, container_height, container_width), lpos, lquat);
		AddCollisionGeometry(F, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
		AddCollisionGeometry(B, BOX, Vector(container_width * wscale, container_height, container_thickness), lpos, lquat);
		AddCollisionGeometry(BTM, BOX, Vector(container_width * wscale, container_thickness, container_width), lpos, lquat);

		FinalizeObject(L, system_gpu);
		FinalizeObject(R, system_gpu);
		FinalizeObject(F, system_gpu);
		FinalizeObject(B, system_gpu);
		FinalizeObject(BTM, system_gpu);

		//createGeometryGPU(system_gpu);
		//LoadObjects(system_gpu, "stack10000_bodies.txt");
	}
	ChOpenGLManager * window_manager = new ChOpenGLManager();
	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
	//openGLView.AddSystem(system_cpu);
	openGLView.SetCustomCallback(RunTimeStep);
	openGLView.StartSpinning(window_manager);
	window_manager->CallGlutMainLoop();
	ChTimer<double> timer;
	timer.start();
	int counter = 0;
	while (counter < 20) {
		RunTimeStep(system_gpu, counter);
		system_gpu->DoStepDynamics(step_size);
		cout << "Residual: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetResidual() << endl;
		cout << "ITER: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations() << endl;
		cout << "DONE: =============" << counter << endl;

		counter++;
	}
	timer.stop();
	cout << "TOTAL TIME " << timer();

//	while (counter < 1) {
//
//		RunTimeStep(system_cpu, counter);
//		RunTimeStep(system_gpu, counter);
//
//		validate_positions(system_cpu, system_gpu, tolerance);
//		validate_rotations(system_cpu, system_gpu, tolerance);
//		//validate_velocities(system_cpu, system_gpu, tolerance);
//		//validate_omega(system_cpu, system_gpu, tolerance);
//		cout << "CPU: =============" << endl;
//		system_cpu->DoStepDynamics(step_size);
//		cout << "ITER: " << ((ChLcpIterativeSolver *) (system_cpu->GetLcpSolverSpeed()))->GetTotalIterations() << endl;
//		cout << "Residual: " << ((ChLcpIterativeAPGD *) (system_cpu->GetLcpSolverSpeed()))->GetResidual() << endl;
//		cout << "GPU: =============" << endl;
//		system_gpu->DoStepDynamics(step_size);
//		cout << "ITER: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations() << endl;
//		cout << "Residual: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetResidual() << endl;
//		cout << "=============" << endl;
//
//		//		validate_ContactsBullet(system_cpu, system_gpu);
//		printContactsCPU(system_cpu, contact_cpu);
//		printContactsGPU(system_gpu, contact_gpu);
//		comparecontacts(contact_cpu, contact_gpu, tolerance);
//		validate_positions(system_cpu, system_gpu, tolerance);
//		validate_rotations(system_cpu, system_gpu, tolerance);
//		//validate_velocities(system_cpu, system_gpu, tolerance);
//		//validate_omega(system_cpu, system_gpu, tolerance);
//		chrono::ChSparseMatrix M_cpu, M_gpu;
//		chrono::ChMatrixDynamic<double> F_cpu, F_gpu;
//
////		dump_matricies(system_cpu->GetLcpSystemDescriptor(), M_cpu, F_cpu);
////		dump_matricies(((ChLcpSystemDescriptorGPU*) (system_gpu->GetLcpSystemDescriptor())), M_gpu, F_gpu);
////
////		chrono::ChStreamOutAsciiFile file_M_cpu("dump_M_cpu.dat");
////		chrono::ChStreamOutAsciiFile file_M_gpu("dump_M_gpu.dat");
////		M_cpu.StreamOUTsparseMatlabFormat(file_M_cpu);
////		M_gpu.StreamOUTsparseMatlabFormat(file_M_gpu);
////
////		chrono::ChStreamOutAsciiFile file_F_cpu("dump_F_cpu.dat");
////		chrono::ChStreamOutAsciiFile file_F_gpu("dump_F_gpu.dat");
////		F_cpu.StreamOUTdenseMatlabFormat(file_F_cpu);
////		F_gpu.StreamOUTdenseMatlabFormat(file_F_gpu);
//
//		compare_variables(system_cpu->GetLcpSystemDescriptor(), system_gpu->GetLcpSystemDescriptor(), tolerance);
//
////		if (M_cpu == M_gpu) {
////			cout << "M equal!" << endl;
////		} else {
////			cout << "M NOT equal!" << endl;
////		}
//		cout << "DONE: =============" << counter << endl;
//
//		counter++;
//	}
}
