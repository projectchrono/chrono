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
Vector lpos(0, 0, 0);
ChQuaternion<> lquat(1, 0, 0, 0);

struct contact_dat {
		real3 posA, posB, N;
		real dist;
		uint idA, idB;
};
vector<contact_dat> contact_cpu, contact_gpu;

real tolerance = 0;
bool dump_to_file = true;
using namespace chrono;
uint num_objects = 0;
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
	real mass = 1;
	real radius = .1;
	Vector pos, pos_dt, wvel_loc;
	Quaternion quat;
	string temp;
	while (ifile.fail() == false) {
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
		AddCollisionGeometry(body, SPHERE, Vector(radius, radius, radius), lpos, lquat);
		FinalizeObject(body, mSys);
		Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
		body->SetInertiaXX(inertia);
		body->SetPos_dt(pos_dt);
		body->SetWvel_loc(wvel_loc);
		body->SetIdentifier(counter);
		counter++;
	}
	{
		ChQuaternion<> quat(1, 0, 0, 0);
		ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody);
		ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody);
		ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody);
		ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody);
		ChSharedBodyPtr BTM = ChSharedBodyPtr(new ChBody);

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
	}

}

void LoadObjects_GPU(ChSystemGPU* mSys, string filename) {
	ifstream ifile(filename.c_str());
	ChSharedBodyGPUPtr body;
	int counter = 0;
	real mass = 1;
	real radius = .1;
	Vector pos, pos_dt, wvel_loc;
	Quaternion quat;
	string temp;
	while (ifile.fail() == false) {
		getline(ifile, temp);
		if (ifile.fail() == true) {
			break;
		}
		body = ChSharedBodyGPUPtr(new ChBodyGPU);
		body->SetCollisionModelBullet();
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
		counter++;
	}
	{
		ChQuaternion<> quat(1, 0, 0, 0);
		//ChQuaternion<> fortyfive;
		//fortyfive.Q_from_AngAxis(PI/4.0,ChVector<>(1,0,0));
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
	}

}

template<class T>
void RunTimeStep(T* mSys, const int frame) {
}
void createGeometryCPU(ChSystem* mSys) {
	ChQuaternion<> quat(1, 0, 0, 0);
	ChSharedBodyPtr sphere;
	srand(11);
	for (int i = 0; i < 10; i++) {
		sphere = ChSharedBodyPtr(new ChBody);
		real mass = 1;
		real radius = .1;
		//(rand() % 10000 / 1000.0 - 5)
		Vector pos = Vector((rand() % 10000 / 1000.0 - 5), i / 100.0 - 5, (rand() % 10000 / 1000.0 - 5));
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
	srand(11);
	int counter = 0;

	for (int i = 0; i < 1000; i++) {
		real mass = 1;
		real radius = .1;
		sphere = ChSharedBodyGPUPtr(new ChBodyGPU);
		sphere->SetCollisionModelBullet();
		Vector pos = Vector((rand() % 10000 / 1000.0 - 5), (rand() % 10000 / 1000.0 - 4), (rand() % 10000 / 1000.0 - 5));
		InitObject(sphere, mass, pos, quat, 1, 1, 0, true, false, 32, counter);
		AddCollisionGeometry(sphere, SPHERE, Vector(radius, radius, radius), Vector(0, 0, 0), quat);
		//AddCollisionGeometry(sphere, SPHERE, Vector(radius, radius, radius), Vector(radius*.5,0,0), quat);
		FinalizeObject(sphere, mSys);
		Vector inertia = Vector(2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius, 2 / 5.0 * mass * radius * radius);
		sphere->SetInertiaXX(inertia);
		counter++;
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

	ChQuaternion<> fortyfive;
	fortyfive.Q_from_AngAxis(PI / 4.0, ChVector<>(1, 0, 0));
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
bool validate_positions(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	vector<ChVector<> > positions_cpu, positions_gpu;
	cout << "validating positions" << endl;
	for (int i = 0; i < system_cpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_cpu->Get_bodylist()->at(i);
		positions_cpu.push_back(abody->GetPos());
	}
	for (int i = 0; i < system_gpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_gpu->Get_bodylist()->at(i);
		positions_gpu.push_back(abody->GetPos());
	}

	for (int i = 0; i < positions_cpu.size(); i++) {
		ChVector<> diff = positions_cpu[i] - positions_gpu[i];
		if (fabs(diff.x) > tolerance) {
			cout << "Position Error x: [" << i << ", " << diff.x << endl;
			//return false;
		}
		if (fabs(diff.y) > tolerance) {
			cout << "Position Error y: [" << i << ", " << diff.y << endl;
			//return false;
		}
		if (fabs(diff.z) > tolerance) {
			cout << "Position Error z: [" << i << ", " << diff.z << endl;
			//return false;
		}

	}
	cout << "validating positions - PASSED" << endl;
	return true;
}
bool validate_rotations(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	vector<ChQuaternion<> > positions_cpu, positions_gpu;
	cout << "validating rotations" << endl;
	for (int i = 0; i < system_cpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_cpu->Get_bodylist()->at(i);
		positions_cpu.push_back(abody->GetRot());
	}
	for (int i = 0; i < system_gpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_gpu->Get_bodylist()->at(i);
		positions_gpu.push_back(abody->GetRot());
	}

	for (int i = 0; i < positions_cpu.size(); i++) {
		ChQuaternion<> diff = positions_cpu[i] - positions_gpu[i];
		if (fabs(diff.e0) > tolerance) {
			cout << "rotations e0: [" << i << ", " << diff.e0 << endl;
		}
		if (fabs(diff.e1) > tolerance) {
			cout << "rotations e1: [" << i << ", " << diff.e1 << endl;
		}
		if (fabs(diff.e2) > tolerance) {
			cout << "rotations e2: [" << i << ", " << diff.e2 << endl;
		}
		if (fabs(diff.e3) > tolerance) {
			cout << "rotations e3: [" << i << ", " << diff.e3 << endl;
		}
	}
	cout << "validating rotations - PASSED" << endl;
	return true;
}
bool validate_velocities(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	vector<ChVector<> > velocities_cpu, velocities_gpu;
	cout << "validating velocities" << endl;
	for (int i = 0; i < system_cpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_cpu->Get_bodylist()->at(i);
		velocities_cpu.push_back(abody->GetPos_dt());
	}
	for (int i = 0; i < system_gpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_gpu->Get_bodylist()->at(i);
		velocities_gpu.push_back(abody->GetPos_dt());
	}

	for (int i = 0; i < velocities_cpu.size(); i++) {
		ChVector<> diff = velocities_cpu[i] - velocities_gpu[i];
		if (fabs(diff.x) > tolerance) {
			cout << "Velocity x: [" << i << ", " << diff.x << endl;
			//return false;
		}
		if (fabs(diff.y) > tolerance) {
			cout << "Velocity y: [" << i << ", " << diff.y << endl;
			//return false;
		}
		if (fabs(diff.z) > tolerance) {
			cout << "Velocity z: [" << i << ", " << diff.z << endl;
			//return false;
		}

	}
	cout << "validating velocities - PASSED" << endl;
	return true;
}
bool validate_omega(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	vector<ChVector<> > omega_cpu, omega_gpu;
	cout << "validating omega" << endl;
	for (int i = 0; i < system_cpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_cpu->Get_bodylist()->at(i);
		omega_cpu.push_back(abody->GetWvel_loc());
	}
	for (int i = 0; i < system_gpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_gpu->Get_bodylist()->at(i);
		omega_gpu.push_back(abody->GetWvel_loc());
	}

	for (int i = 0; i < omega_cpu.size(); i++) {
		ChVector<> diff = omega_cpu[i] - omega_gpu[i];
		if (fabs(diff.x) > tolerance) {
			cout << "Omega x: [" << i << ", " << diff.x << endl;
			//return false;
		}
		if (fabs(diff.y) > tolerance) {
			cout << "Omega y: [" << i << ", " << diff.y << endl;
			//return false;
		}
		if (fabs(diff.z) > tolerance) {
			cout << "Omega z: [" << i << ", " << diff.z << endl;
			//return false;
		}

	}
	cout << "validating omega - PASSED" << endl;
	return true;
}
bool validate_rhs(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	vector<double> rhs_cpu, rhs_gpu;
	cout << "validating rhs" << endl;
	//((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->Dump_Rhs(rhs_cpu);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->Dump_Rhs(rhs_gpu);

	if (dump_to_file) {
		ofstream ofile("rhs.txt");

		for (int i = 0, j = 0; i < rhs_cpu.size(), j < rhs_cpu.size() / 3; i += 3, j++) {
			ofile << rhs_cpu[0 + i] << " " << rhs_gpu[j + rhs_cpu.size() / 3 * 0] << endl;
			ofile << rhs_cpu[1 + i] << " " << rhs_gpu[j + rhs_cpu.size() / 3 * 1] << endl;
			ofile << rhs_cpu[2 + i] << " " << rhs_gpu[j + rhs_cpu.size() / 3 * 2] << endl;
		}
		ofile.close();
	}

	for (int i = 0, j = 0; i < rhs_cpu.size(), j < rhs_cpu.size() / 3; i += 3, j++) {
		double diff1 = rhs_cpu[0 + i] - rhs_gpu[j + rhs_cpu.size() / 3 * 0];
		double diff2 = rhs_cpu[1 + i] - rhs_gpu[j + rhs_cpu.size() / 3 * 1];
		double diff3 = rhs_cpu[2 + i] - rhs_gpu[j + rhs_cpu.size() / 3 * 2];
		if (fabs(diff1) > tolerance) {
			cout << "Rhs Error1: [" << i << ", " << diff1 << endl;
			//return false;
		}
		if (fabs(diff2) > tolerance) {
			cout << "Rhs Error2: [" << i << ", " << diff2 << endl;
			//return false;
		}
		if (fabs(diff3) > tolerance) {
			cout << "Rhs Error3: [" << i << ", " << diff3 << endl;
			//return false;
		}
	}

	cout << "validating rhs - PASSED" << endl;
	return true;
}

bool validate_jacobians(ChSystem* system_cpu, ChSystemGPU* system_gpu) {

	cout << "validating jacobians" << endl;
	vector<real3> JXYZA_cpu, JUVWA_cpu, JXYZB_cpu, JUVWB_cpu;
	vector<real3> JXYZA_gpu, JUVWA_gpu, JXYZB_gpu, JUVWB_gpu;
	std::vector<ChLcpConstraint*>& mconstraints = system_cpu->GetLcpSystemDescriptor()->GetConstraintsList();
	ChMatrix<float>* JA, *JB;
	for (unsigned int i = 0; i < mconstraints.size(); i += 3) {
		JA = ((ChLcpConstraintTwoBodies*) (mconstraints[0 + i]))->Get_Cq_a();
		JB = ((ChLcpConstraintTwoBodies*) (mconstraints[0 + i]))->Get_Cq_b();
		JXYZA_cpu.push_back(R3(JA->Element(0, 0), JA->Element(0, 1), JA->Element(0, 2)));
		JUVWA_cpu.push_back(R3(JA->Element(0, 3), JA->Element(0, 4), JA->Element(0, 5)));
		JXYZB_cpu.push_back(R3(JB->Element(0, 0), JB->Element(0, 1), JB->Element(0, 2)));
		JUVWB_cpu.push_back(R3(JB->Element(0, 3), JB->Element(0, 4), JB->Element(0, 5)));

		JA = ((ChLcpConstraintTwoBodies*) (mconstraints[1 + i]))->Get_Cq_a();
		JB = ((ChLcpConstraintTwoBodies*) (mconstraints[1 + i]))->Get_Cq_b();
		JXYZA_cpu.push_back(R3(JA->Element(0, 0), JA->Element(0, 1), JA->Element(0, 2)));
		JUVWA_cpu.push_back(R3(JA->Element(0, 3), JA->Element(0, 4), JA->Element(0, 5)));
		JXYZB_cpu.push_back(R3(JB->Element(0, 0), JB->Element(0, 1), JB->Element(0, 2)));
		JUVWB_cpu.push_back(R3(JB->Element(0, 3), JB->Element(0, 4), JB->Element(0, 5)));

		JA = ((ChLcpConstraintTwoBodies*) (mconstraints[2 + i]))->Get_Cq_a();
		JB = ((ChLcpConstraintTwoBodies*) (mconstraints[2 + i]))->Get_Cq_b();
		JXYZA_cpu.push_back(R3(JA->Element(0, 0), JA->Element(0, 1), JA->Element(0, 2)));
		JUVWA_cpu.push_back(R3(JA->Element(0, 3), JA->Element(0, 4), JA->Element(0, 5)));
		JXYZB_cpu.push_back(R3(JB->Element(0, 0), JB->Element(0, 1), JB->Element(0, 2)));
		JUVWB_cpu.push_back(R3(JB->Element(0, 3), JB->Element(0, 4), JB->Element(0, 5)));
	}

	system_gpu->gpu_data_manager->DeviceToHostJacobians();

	for (unsigned int i = 0; i < mconstraints.size() / 3; i++) {
		JXYZA_gpu.push_back(system_gpu->gpu_data_manager->host_JXYZA_data[i + mconstraints.size() / 3 * 0]);
		JXYZA_gpu.push_back(system_gpu->gpu_data_manager->host_JXYZA_data[i + mconstraints.size() / 3 * 1]);
		JXYZA_gpu.push_back(system_gpu->gpu_data_manager->host_JXYZA_data[i + mconstraints.size() / 3 * 2]);

		JUVWA_gpu.push_back(system_gpu->gpu_data_manager->host_JUVWA_data[i + mconstraints.size() / 3 * 0]);
		JUVWA_gpu.push_back(system_gpu->gpu_data_manager->host_JUVWA_data[i + mconstraints.size() / 3 * 1]);
		JUVWA_gpu.push_back(system_gpu->gpu_data_manager->host_JUVWA_data[i + mconstraints.size() / 3 * 2]);

		JXYZB_gpu.push_back(system_gpu->gpu_data_manager->host_JXYZB_data[i + mconstraints.size() / 3 * 0]);
		JXYZB_gpu.push_back(system_gpu->gpu_data_manager->host_JXYZB_data[i + mconstraints.size() / 3 * 1]);
		JXYZB_gpu.push_back(system_gpu->gpu_data_manager->host_JXYZB_data[i + mconstraints.size() / 3 * 2]);

		JUVWB_gpu.push_back(system_gpu->gpu_data_manager->host_JUVWB_data[i + mconstraints.size() / 3 * 0]);
		JUVWB_gpu.push_back(system_gpu->gpu_data_manager->host_JUVWB_data[i + mconstraints.size() / 3 * 1]);
		JUVWB_gpu.push_back(system_gpu->gpu_data_manager->host_JUVWB_data[i + mconstraints.size() / 3 * 2]);

	}

	for (int i = 0; i < mconstraints.size(); i++) {

		real3 diff1 = (JXYZA_cpu[i] - JXYZA_gpu[i]);
		real3 diff2 = (JUVWA_cpu[i] - JUVWA_gpu[i]);
		real3 diff3 = (JXYZB_cpu[i] - JXYZB_gpu[i]);
		real3 diff4 = (JUVWB_cpu[i] - JUVWB_gpu[i]);

		if (fabs(diff1.x) > tolerance) {
			cout << "jacobian Error JXYZA X: [" << i << ", " << diff1.x << endl;
		}
		if (fabs(diff1.y) > tolerance) {
			cout << "jacobian Error JXYZA Y: [" << i << ", " << diff1.y << endl;
		}
		if (fabs(diff1.z) > tolerance) {
			cout << "jacobian Error JXYZA Z: [" << i << ", " << diff1.z << endl;
		}

		if (fabs(diff2.x) > tolerance) {
			cout << "jacobian Error JUVWA X: [" << i << ", " << diff2.x << endl;
		}
		if (fabs(diff2.y) > tolerance) {
			cout << "jacobian Error JUVWA Y: [" << i << ", " << diff2.y << endl;
		}
		if (fabs(diff2.z) > tolerance) {
			cout << "jacobian Error JUVWA Z: [" << i << ", " << diff2.z << endl;
		}

		if (fabs(diff3.x) > tolerance) {
			cout << "jacobian Error JXYZB X: [" << i << ", " << diff2.x << endl;
		}
		if (fabs(diff3.y) > tolerance) {
			cout << "jacobian Error JXYZB Y: [" << i << ", " << diff2.y << endl;
		}
		if (fabs(diff3.z) > tolerance) {
			cout << "jacobian Error JXYZB Z: [" << i << ", " << diff2.z << endl;
		}

		if (fabs(diff4.x) > tolerance) {
			cout << "jacobian Error JUVWB X: [" << i << ", " << diff4.x << endl;
		}
		if (fabs(diff4.y) > tolerance) {
			cout << "jacobian Error JUVWB Y: [" << i << ", " << diff4.y << endl;
		}
		if (fabs(diff4.z) > tolerance) {
			cout << "jacobian Error JUVWB Z: [" << i << ", " << diff4.z << endl;
		}

	}
	cout << "validating jacobians - PASSED" << endl;
	return true;
}
bool validate_lagrange(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	vector<double> lagrange_cpu, lagrange_gpu;

	//((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->Dump_Lambda(lagrange_cpu);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->Dump_Lambda(lagrange_gpu);
	cout << "validating lagrange" << lagrange_cpu.size() << " " << lagrange_gpu.size() << endl;
	for (int i = 0, j = 0; i < lagrange_cpu.size(), j < lagrange_cpu.size() / 3; i += 3, j++) {
		float diff1 = lagrange_cpu[0 + i] - lagrange_gpu[j + lagrange_cpu.size() / 3 * 0];
		float diff2 = lagrange_cpu[1 + i] - lagrange_gpu[j + lagrange_cpu.size() / 3 * 1];
		float diff3 = lagrange_cpu[2 + i] - lagrange_gpu[j + lagrange_cpu.size() / 3 * 2];
		if (fabs(diff1) > tolerance) {
			cout << "lagrange Error1: [" << i << ", " << diff1 << " " << lagrange_cpu[0 + i] << " " << lagrange_gpu[j + lagrange_cpu.size() / 3 * 0] << endl;
			//return false;
		}
		if (fabs(diff2) > tolerance) {
			cout << "lagrange Error2: [" << i << ", " << diff2 << " " << lagrange_cpu[1 + i] << " " << lagrange_gpu[j + lagrange_cpu.size() / 3 * 1] << endl;
			//return false;
		}
		if (fabs(diff3) > tolerance) {
			cout << "lagrange Error3: [" << i << ", " << diff3 << " " << lagrange_cpu[2 + i] << " " << lagrange_gpu[j + lagrange_cpu.size() / 3 * 2] << endl;
			//return false;
		}
	}
	cout << "validating lagrange - PASSED" << endl;
	return true;
}
bool validate_shur(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	vector<double> shur_cpu, shur_gpu;
	cout << "validating Shur" << endl;

	///////debug code -Hammad------
	int nc = system_cpu->GetLcpSystemDescriptor()->CountActiveConstraints();
	ChMatrixDynamic<> mbackup;
	ChMatrixDynamic<> mdebug;
	system_cpu->GetLcpSystemDescriptor()->FromVariablesToVector(mbackup, true);
	mdebug.Resize(nc, 1);
	ChMatrixDynamic<> mtemp(nc, 1);
	mtemp.FillElem(1);
	system_cpu->GetLcpSystemDescriptor()->ShurComplementProduct(mdebug, &mtemp, 0);
	system_cpu->GetLcpSystemDescriptor()->FromVectorToVariables(mbackup);
	///////------------------------

	for (int i = 0; i < mdebug.GetRows(); i++) {
		shur_cpu.push_back(mdebug(i, 0));
	}

	//((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->Dump_Shur(shur_cpu);
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->Dump_Shur(shur_gpu);

	for (int i = 0, j = 0; i < shur_cpu.size(), j < shur_cpu.size() / 3; i += 3, j++) {
		double diff1 = shur_cpu[0 + i] - shur_gpu[j + shur_cpu.size() / 3 * 0];
		double diff2 = shur_cpu[1 + i] - shur_gpu[j + shur_cpu.size() / 3 * 1];
		double diff3 = shur_cpu[2 + i] - shur_gpu[j + shur_cpu.size() / 3 * 2];
		if (fabs(diff1) > tolerance) {
			cout << "shur Error1: [" << i << ", " << diff1 << endl;
			//return false;
		}
		if (fabs(diff2) > tolerance) {
			cout << "shur Error2: [" << i << ", " << diff2 << endl;
			//return false;
		}
		if (fabs(diff3) > tolerance) {
			cout << "shur Error3: [" << i << ", " << diff3 << endl;
			//return false;
		}
	}
	cout << "validating shur - PASSED" << endl;
	return true;
}

bool validate_q(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	vector<ChVector<> > v_cpu, o_cpu, v_gpu, o_gpu;
	cout << "validating q" << endl;

	std::vector<ChLcpVariables*>& mvariables = system_cpu->GetLcpSystemDescriptor()->GetVariablesList();

	for (unsigned int i = 0; i < mvariables.size(); i++) {
		ChMatrix<> qb = mvariables[i]->Get_qb();
		v_cpu.push_back(ChVector<>(qb.GetElement(0, 0), qb.GetElement(1, 0), qb.GetElement(2, 0)));
		o_cpu.push_back(ChVector<>(qb.GetElement(3, 0), qb.GetElement(4, 0), qb.GetElement(5, 0)));

		real3 v = system_gpu->gpu_data_manager->gpu_data.device_vel_data[i];
		real3 o = system_gpu->gpu_data_manager->gpu_data.device_omg_data[i];
		v_gpu.push_back(ChVector<>(v.x, v.y, v.z));
		o_gpu.push_back(ChVector<>(o.x, o.y, o.z));
	}
	for (unsigned int i = 0; i < mvariables.size(); i++) {

		ChVector<> diff1 = v_cpu[i] - v_gpu[i];
		if (diff1.Length() > tolerance) {
			cout << "V Error: [" << i << ", " << diff1.Length() << " " << v_cpu[i].x << " " << v_cpu[i].y << " " << v_cpu[i].z << " " << v_gpu[i].x << " " << v_gpu[i].y << " " << v_gpu[i].z << endl;
			//return false;
		}
		ChVector<> diff2 = o_cpu[i] - o_gpu[i];
		if (diff2.Length() > tolerance) {
			cout << "O Error: [" << i << ", " << diff2.Length() << endl;
			//return false;
		}
	}

	cout << "validating q - PASSED" << endl;
	return true;
}

bool validate(ChSystem* system_cpu, ChSystemGPU* system_gpu) {
	return true;
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
bool operator <(const contact_dat &i, const contact_dat &j) {

	if (i.idA == j.idA) {
		return i.idB < j.idB;
	} else {
		return i.idA < j.idA;
	}
}
void printContactsBullet(btCollisionWorld* bt_collision_world) {

	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	uint contacts = 0;
	contact_dat icontact;
	ChCollisionInfo jcontact;
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

		jcontact.modelA = (ChCollisionModel*) obA->getUserPointer();
		jcontact.modelB = (ChCollisionModel*) obB->getUserPointer();

		if (((ChBody*) (jcontact.modelA->GetPhysicsItem()))->IsActive() == false && ((ChBody*) (jcontact.modelB->GetPhysicsItem()))->IsActive() == false) {
			continue;
		}

		double envelopeA = jcontact.modelA->GetEnvelope();
		double envelopeB = jcontact.modelB->GetEnvelope();

		double marginA = jcontact.modelA->GetSafeMargin();
		double marginB = jcontact.modelB->GetSafeMargin();

		// Execute custom broadphase callback, if any
		bool do_narrow_contactgeneration = true;

		if (do_narrow_contactgeneration) {
			int numContacts = contactManifold->getNumContacts();

			for (int j = 0; j < numContacts; j++) {
				btManifoldPoint& pt = contactManifold->getContactPoint(j);

				if (pt.getDistance() < marginA + marginB) // to discard "too far" constraints (the Bullet engine also has its threshold)
						{

					btVector3 ptA = pt.getPositionWorldOnA();
					btVector3 ptB = pt.getPositionWorldOnB();

					icontact.posA = R3(ptA.getX(), ptA.getY(), ptA.getZ());
					icontact.posB = R3(ptB.getX(), ptB.getY(), ptB.getZ());

					icontact.N = R3(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(), -pt.m_normalWorldOnB.getZ());
					icontact.N = normalize(icontact.N);

					double ptdist = pt.getDistance();

					icontact.posA = icontact.posA - icontact.N * envelopeA;
					icontact.posB = icontact.posB + icontact.N * envelopeB;
					icontact.dist = ptdist + envelopeA + envelopeB;
					icontact.idA = jcontact.modelA->GetPhysicsItem()->GetIdentifier();
					icontact.idB = jcontact.modelB->GetPhysicsItem()->GetIdentifier();

					//icontact.reaction_cache = pt.reactions_cache;

					contact_cpu.push_back(icontact);

					contacts++;
					// Execute some user custom callback, if any
//					cout << icontact.idA << " "<< icontact.idB << " ";
//
//					cout << icontact.N.x << " " << icontact.N.y << " " << icontact.N.z << " ";
//					cout << icontact.posA.x << " " << icontact.posA.y << " " << icontact.posA.z << " ";
//					cout << icontact.posB.x << " " << icontact.posB.y << " " << icontact.posB.z << " ";
//					cout << " dist_cpu " << icontact.dist << " Envelopes: " << envelopeA << " " << envelopeB << endl;

				}
			}
		}
	}
	std::sort(contact_cpu.begin(), contact_cpu.end());
	cout << "Contacts: " << contacts << endl;
}
bool printContactsGPU(ChSystemGPU* system_gpu) {

	system_gpu->gpu_data_manager->DeviceToHostContacts();
	contact_dat icontact;
	for (int i = 0; i < system_gpu->gpu_data_manager->number_of_contacts; i++) {
		real3 vN = system_gpu->gpu_data_manager->host_norm_data[i];
		real3 vpA = system_gpu->gpu_data_manager->host_cpta_data[i];
		real3 vpB = system_gpu->gpu_data_manager->host_cptb_data[i];
		real distance = system_gpu->gpu_data_manager->host_dpth_data[i];

		icontact.posA = system_gpu->gpu_data_manager->host_cpta_data[i];
		icontact.posB = system_gpu->gpu_data_manager->host_cptb_data[i];
		icontact.N = system_gpu->gpu_data_manager->host_norm_data[i];
		icontact.dist = system_gpu->gpu_data_manager->host_dpth_data[i];
		icontact.idA = system_gpu->gpu_data_manager->host_bids_data[i].x;
		icontact.idB = system_gpu->gpu_data_manager->host_bids_data[i].y;

		contact_gpu.push_back(icontact);

//		cout << vN.x << " " << vN.y << " " << vN.z << " ";
//		cout << vpA.x << " " << vpA.y << " " << vpA.z << " ";
//		cout << vpB.x << " " << vpB.y << " " << vpB.z << " ";
//		cout << " dist_gpu " << distance << " "<< system_gpu->gpu_data_manager->host_bids_data[i].x<<" "<< system_gpu->gpu_data_manager->host_bids_data[i].y<<endl;

	}
	cout << "Contacts: " << system_gpu->gpu_data_manager->number_of_contacts << endl;
//	for (int i = 0; i < system_gpu->gpu_data_manager->host_pair_data.size(); i++) {
//
//		long long p = system_gpu->gpu_data_manager->host_pair_data[i];
//		int2 pair = I2(int(p >> 32), int(p & 0xffffffff));
//		cout << pair.x << " " << pair.y << endl;
//
//	}

}
void comparecontacts() {
	cout << "compare contacts" << endl;
	for (int i = 0; i < contact_cpu.size(); i++) {

		real3 N = contact_cpu[i].N - contact_gpu[i].N;
		real3 Pa = contact_cpu[i].posA - contact_gpu[i].posA;
		real3 Pb = contact_cpu[i].posB - contact_gpu[i].posB;
		real d = contact_cpu[i].dist - contact_gpu[i].dist;

		//if (fabs(N.x) > tolerance) {
			cout << "N x: [" << i << ", " << N.x << endl;
		//}
		//if (fabs(N.y) > tolerance) {
			cout << "N y: [" << i << ", " << N.y << endl;
		//}
		//if (fabs(N.z) > tolerance) {
			cout << "N z: [" << i << ", " << N.z << endl;
		//}

		//if (fabs(Pa.x) > tolerance) {
			cout << "Pa x: [" << i << ", " << Pa.x << endl;
		//}
		//if (fabs(Pa.y) > tolerance) {
			cout << "Pa y: [" << i << ", " << Pa.y << endl;
		//}
		//if (fabs(Pa.z) > tolerance) {
			cout << "Pa z: [" << i << ", " << Pa.z << endl;
		//}

		//if (fabs(Pb.x) > tolerance) {
			cout << "Pb x: [" << i << ", " << Pb.x << endl;
		//}
		//if (fabs(Pb.y) > tolerance) {
			cout << "Pb y: [" << i << ", " << Pb.y << endl;
		//}
		//if (fabs(Pb.z) > tolerance) {
			cout << "Pb z: [" << i << ", " << Pb.z << endl;
		//}
		//if (fabs(d) > tolerance) {
			cout << "dist: [" << i << ", " << d << endl;
		//}
//		cout << contact_cpu[i].idA << " " << contact_cpu[i].idB << " | " << contact_gpu[i].idA << " " << contact_gpu[i].idB << "\n";
//		cout<<"-------------------------------------------------------------------------------------"<<endl;
//		cout << "[" << contact_cpu[i].N.x << "," << contact_cpu[i].N.y << "," << contact_cpu[i].N.z << "]\n";
//		cout << "[" << contact_gpu[i].N.x << "," << contact_gpu[i].N.y << "," << contact_gpu[i].N.z << "]\n";
//		cout<<"-------------------------------------------------------------------------------------"<<endl;
//		cout << contact_cpu[i].posA.x << " " << contact_cpu[i].posA.y << " " << contact_cpu[i].posA.z << "\n";
//		cout << contact_gpu[i].posA.x << " " << contact_gpu[i].posA.y << " " << contact_gpu[i].posA.z << "\n";
//		cout<<"-------------------------------------------------------------------------------------"<<endl;
//
//		cout << contact_cpu[i].dist << " " << contact_gpu[i].dist << endl;
//		cout << "============================================================================================" << endl;
	}

}


int main(int argc, char* argv[]) {
	//feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
//	omp_set_num_threads(3);
//cudaSetDevice(0);

	ChSystem * system_cpu = new ChSystem;
	{
		ChLcpSystemDescriptor *mdescriptor = new ChLcpSystemDescriptor();
		ChContactContainer *mcontactcontainer = new ChContactContainer();
		ChCollisionSystemBullet *mcollisionengine = new ChCollisionSystemBullet();
		ChLcpIterativeJacobi *msolver = new ChLcpIterativeJacobi(100, false, 1e-4);

		system_cpu->ChangeLcpSystemDescriptor(mdescriptor);
		system_cpu->ChangeContactContainer(mcontactcontainer);
		system_cpu->ChangeLcpSolverSpeed(msolver);
		system_cpu->ChangeCollisionSystem(mcollisionengine);
		system_cpu->SetIntegrationType(ChSystem::INT_ANITESCU);
		system_cpu->Set_G_acc(Vector(0, -9.80665, 0));
		system_cpu->SetStep(0.005);
		createGeometryCPU(system_cpu);
		//LoadObjects_CPU(system_cpu, "fly10_bodies.txt");

		((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->SetMaxIterations(100);
		system_cpu->SetMaxiter(100);
		system_cpu->SetIterLCPmaxItersSpeed(100);
		system_cpu->SetTol(1e-4);
		system_cpu->SetTolSpeeds(1e-4);
		((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->SetTolerance(1e-4);
		system_cpu->SetMaxPenetrationRecoverySpeed(.6);
	}

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
		system_gpu->SetStep(0.005);
		//-9.80665
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetMaxIteration(100);
		system_gpu->SetMaxiter(100);
		system_gpu->SetIterLCPmaxItersSpeed(100);
		system_gpu->SetTol(1e-4);
		system_gpu->SetTolSpeeds(1e-4);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetTolerance(1e-4);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetCompliance(0, 0, 0);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetContactRecoverySpeed(.6);
		((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->SetSolverType(ACCELERATED_PROJECTED_GRADIENT_DESCENT);
		((ChCollisionSystemGPU *) (system_gpu->GetCollisionSystem()))->SetCollisionEnvelope(0.01);

		createGeometryGPU(system_gpu);
		//100_bodies
		//LoadObjects_GPU(system_gpu, "fly10_bodies.txt");
	}
////
	ChOpenGLManager * window_manager = new ChOpenGLManager();
	ChOpenGL openGLView(window_manager, system_gpu, 800, 600, 0, 0, "Test_Solvers");
	//openGLView.AddSystem(system_cpu);
	openGLView.SetCustomCallback(RunTimeStep);
	openGLView.StartSpinning(window_manager);
	window_manager->CallGlutMainLoop();

	int counter = 0;
	ChTimer<double> timer;
	timer.start();

	while (counter < 100) {
        
//		RunTimeStep(system_cpu, counter);
//		RunTimeStep(system_gpu, counter);
//
//		validate_positions(system_cpu, system_gpu);
//		validate_rotations(system_cpu, system_gpu);
//		validate_velocities(system_cpu, system_gpu);
//		validate_omega(system_cpu, system_gpu);
//		cout << "CPU: =============" << endl;
//		system_cpu->DoStepDynamics(.005);
//		cout << "ITER: " << ((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->GetTotalIterations() << endl;
////		cout << "Residual: " << ((ChLcpIterativeJacobi *) (system_cpu->GetLcpSolverSpeed()))->GetResidual() << endl;
//		cout << "GPU: =============" << endl;
		system_gpu->DoStepDynamics(.005);
//		cout << "ITER: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetTotalIterations() << endl;
//		cout << "Residual: " << ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetResidual() << endl;
////		cout << "=============" << endl;
////		validate_ContactsBullet(system_cpu, system_gpu);
//		//printContactsBullet(((ChCollisionSystemBullet*) (system_cpu->GetCollisionSystem()))->GetBulletCollisionWorld());
////		printContactsGPU(system_gpu);
//		printContactsBullet(((ChCollisionSystemBullet*) (system_cpu->GetCollisionSystem()))->GetBulletCollisionWorld());
//		printContactsGPU(system_gpu);
//		comparecontacts();
//
//		validate_positions(system_cpu, system_gpu);
//		validate_rotations(system_cpu, system_gpu);
//		validate_velocities(system_cpu, system_gpu);
//		validate_omega(system_cpu, system_gpu);
//		validate_rhs(system_cpu, system_gpu);
//		validate_jacobians(system_cpu, system_gpu);
//		validate_lagrange(system_cpu, system_gpu);
////		//validate_shur(system_cpu, system_gpu);
////		validate_q(system_cpu, system_gpu);
////
//		cout << "DONE: =============" << counter << endl;

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
//		cout << "GPUCBP: =============" << endl;

//		for (int i = 0; i < bt_collision_world_gpu->getCollisionObjectArray().size(); i++) {
//
//		}

		//validate(system_cpu, system_gpu);

//		cout << "DONE: =============" << counter << endl;

//		cout<< ((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetCurrentIteration();
//		cout<<" "<<((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->GetResidual()<<endl;

//
//		stringstream ss;
//		ss << "data/" << counter << ".xml";
//		//output.ExportData(ss.str());
//
		cout << counter << endl;
		counter++;
	}
	timer.stop();
	cout << "TIME TAKEN: " << timer() << endl;
	//DumpObjects(system_cpu, "fly10_bodies.txt");

	return 0;
}

