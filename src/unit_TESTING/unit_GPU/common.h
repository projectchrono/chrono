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
#include "unit_GPU/ChLcpSystemDescriptorGPU.h"
using namespace chrono;
using namespace std;

Vector lpos(0, 0, 0);
ChQuaternion<> lquat(1, 0, 0, 0);

struct contact_dat {
	real3 posA, posB, N;
	real dist;
	uint idA, idB;
	real3 JXYZA_N, JXYZA_U, JXYZA_W;
	real3 JXYZB_N, JXYZB_U, JXYZB_W;
	real3 JUVWA_N, JUVWA_U, JUVWA_W;
	real3 JUVWB_N, JUVWB_U, JUVWB_W;
	real3 lagrange;
	real3 rhs;

};

bool operator <(const contact_dat &i, const contact_dat &j) {

	if (i.idA == j.idA) {
		return i.idB < j.idB;
	} else {
		return i.idA < j.idA;
	}
}
bool validate_real3(real3 A, real3 B, string value, int i, real tolerance) {
	real3 diff = A - B;

	if (fabs(diff.x) > tolerance) {
		cout << value << " X " << i << ", " << diff.x << " " << A.x << " " << B.x << endl;
	}
	if (fabs(diff.y) > tolerance) {
		cout << value << " Y " << i << ", " << diff.y << " " << A.y << " " << B.y << endl;
	}
	if (fabs(diff.z) > tolerance) {
		cout << value << " Z " << i << ", " << diff.z << " " << A.z << " " << B.z << endl;
	}

}

bool validate_positions(ChSystem* system_cpu, ChSystemGPU* system_gpu, real tolerance) {
	vector<real3> positions_cpu, positions_gpu;
	cout << "validating positions" << endl;
	for (int i = 0; i < system_cpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_cpu->Get_bodylist()->at(i);
		positions_cpu.push_back(R3(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z));
	}
	for (int i = 0; i < system_gpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_gpu->Get_bodylist()->at(i);
		positions_gpu.push_back(R3(abody->GetPos().x, abody->GetPos().y, abody->GetPos().z));
	}
	for (int i = 0; i < positions_cpu.size(); i++) {
		validate_real3(positions_cpu[i], positions_gpu[i], "Position ", i, tolerance);
	}
	cout << "validating positions - DONE" << endl;
	return true;
}
bool validate_rotations(ChSystem* system_cpu, ChSystemGPU* system_gpu, real tolerance) {
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
	cout << "validating rotations - DONE" << endl;
	return true;
}
bool validate_velocities(ChSystem* system_cpu, ChSystemGPU* system_gpu, real tolerance) {
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
		}
		if (fabs(diff.y) > tolerance) {
			cout << "Velocity y: [" << i << ", " << diff.y << endl;
		}
		if (fabs(diff.z) > tolerance) {
			cout << "Velocity z: [" << i << ", " << diff.z << endl;
		}
	}
	cout << "validating velocities - DONE" << endl;
	return true;
}

bool validate_omega(ChSystem* system_cpu, ChSystemGPU* system_gpu, real tolerance) {
	vector<real3> omega_cpu, omega_gpu;
	cout << "validating omega" << endl;
	for (int i = 0; i < system_cpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_cpu->Get_bodylist()->at(i);
		omega_cpu.push_back(R3(abody->GetWvel_loc().x, abody->GetWvel_loc().y, abody->GetWvel_loc().z));
	}
	for (int i = 0; i < system_gpu->Get_bodylist()->size(); i++) {
		ChBody* abody = (ChBody*) system_gpu->Get_bodylist()->at(i);
		omega_gpu.push_back(R3(abody->GetWvel_loc().x, abody->GetWvel_loc().y, abody->GetWvel_loc().z));
	}
	for (int i = 0; i < omega_cpu.size(); i++) {
		validate_real3(omega_cpu[i], omega_gpu[i], "Omega  ", i, tolerance);
	}
	cout << "validating omega - Done" << endl;
	return true;
}
bool validate_shur(ChSystem* system_cpu, ChSystemGPU* system_gpu, real tolerance) {
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
		}
		if (fabs(diff2) > tolerance) {
			cout << "shur Error2: [" << i << ", " << diff2 << endl;
		}
		if (fabs(diff3) > tolerance) {
			cout << "shur Error3: [" << i << ", " << diff3 << endl;
		}
	}
	cout << "validating shur - PASSED" << endl;
	return true;
}
void printContactsBullet(btCollisionWorld* bt_collision_world, vector<contact_dat> & contact_cpu) {

	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	int contacts = 0;
	contact_dat icontact;
	ChCollisionInfo jcontact;
	contact_cpu.clear();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		//contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

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

			}
		}
	}

	cout << "BContacts: " << contact_cpu.size() << " " << contacts << endl;
}
bool printContactsCPU(ChSystem* system_cpu, vector<contact_dat> & contact_cpu) {

	std::list<ChContact*> list_cpu = ((ChContactContainer*) (system_cpu->GetContactContainer()))->GetContactList();
	contact_dat icontact;
	std::vector<ChLcpConstraint*>& mconstraints = system_cpu->GetLcpSystemDescriptor()->GetConstraintsList();
	int i = 0;
	ChMatrix<double>* JA, *JB;
	real lagrange;

	vector<double> rhs_cpu;

	((ChLcpIterativeAPGD *) (system_cpu->GetLcpSolverSpeed()))->Dump_Rhs(rhs_cpu);
	//((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->Dump_Rhs(rhs_gpu);

	for (std::list<ChContact*>::iterator it = list_cpu.begin(); it != list_cpu.end(); ++it) {
		Vector Pa = (*it)->GetContactP1();
		Vector Pb = (*it)->GetContactP2();
		Vector N = (*it)->GetContactNormal();

		icontact.posA = R3(Pa.x, Pa.y, Pa.z);
		icontact.posB = R3(Pb.x, Pb.y, Pb.z);
		icontact.N = R3(N.x, N.y, N.z);
		icontact.dist = (*it)->GetContactDistance();
		icontact.idA = (*it)->GetModelA()->GetPhysicsItem()->GetIdentifier();
		icontact.idB = (*it)->GetModelB()->GetPhysicsItem()->GetIdentifier();

		JA = ((ChLcpConstraintTwoBodies*) (mconstraints[0 + i]))->Get_Cq_a();
		JB = ((ChLcpConstraintTwoBodies*) (mconstraints[0 + i]))->Get_Cq_b();
		lagrange = ((ChLcpConstraintTwoBodies*) (mconstraints[0 + i]))->Get_l_i();

		icontact.JXYZA_N = (R3(JA->Element(0, 0), JA->Element(0, 1), JA->Element(0, 2)));
		icontact.JUVWA_N = (R3(JA->Element(0, 3), JA->Element(0, 4), JA->Element(0, 5)));
		icontact.JXYZB_N = (R3(JB->Element(0, 0), JB->Element(0, 1), JB->Element(0, 2)));
		icontact.JUVWB_N = (R3(JB->Element(0, 3), JB->Element(0, 4), JB->Element(0, 5)));
		icontact.lagrange.x = lagrange;
		icontact.rhs.x = rhs_cpu[0 + i];

		JA = ((ChLcpConstraintTwoBodies*) (mconstraints[1 + i]))->Get_Cq_a();
		JB = ((ChLcpConstraintTwoBodies*) (mconstraints[1 + i]))->Get_Cq_b();
		lagrange = ((ChLcpConstraintTwoBodies*) (mconstraints[1 + i]))->Get_l_i();
		icontact.JXYZA_U = (R3(JA->Element(0, 0), JA->Element(0, 1), JA->Element(0, 2)));
		icontact.JUVWA_U = (R3(JA->Element(0, 3), JA->Element(0, 4), JA->Element(0, 5)));
		icontact.JXYZB_U = (R3(JB->Element(0, 0), JB->Element(0, 1), JB->Element(0, 2)));
		icontact.JUVWB_U = (R3(JB->Element(0, 3), JB->Element(0, 4), JB->Element(0, 5)));
		icontact.lagrange.y = lagrange;
		icontact.rhs.y = rhs_cpu[1 + i];

		JA = ((ChLcpConstraintTwoBodies*) (mconstraints[2 + i]))->Get_Cq_a();
		JB = ((ChLcpConstraintTwoBodies*) (mconstraints[2 + i]))->Get_Cq_b();
		lagrange = ((ChLcpConstraintTwoBodies*) (mconstraints[2 + i]))->Get_l_i();
		icontact.JXYZA_W = (R3(JA->Element(0, 0), JA->Element(0, 1), JA->Element(0, 2)));
		icontact.JUVWA_W = (R3(JA->Element(0, 3), JA->Element(0, 4), JA->Element(0, 5)));
		icontact.JXYZB_W = (R3(JB->Element(0, 0), JB->Element(0, 1), JB->Element(0, 2)));
		icontact.JUVWB_W = (R3(JB->Element(0, 3), JB->Element(0, 4), JB->Element(0, 5)));
		icontact.lagrange.z = lagrange;
		icontact.rhs.z = rhs_cpu[2 + i];

		i += 3;
		contact_cpu.push_back(icontact);
	}

	cout << "CContacts: " << contact_cpu.size() << endl;
}
bool printContactsGPU(ChSystemGPU* system_gpu, vector<contact_dat> & contact_gpu) {

	system_gpu->gpu_data_manager->DeviceToHostContacts();
	contact_dat icontact;
	system_gpu->gpu_data_manager->DeviceToHostJacobians();
	uint constraints = system_gpu->gpu_data_manager->number_of_contacts;
	vector<double> rhs_gpu;
	((ChLcpSolverGPU *) (system_gpu->GetLcpSolverSpeed()))->Dump_Rhs(rhs_gpu);
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

		icontact.JXYZA_N = (system_gpu->gpu_data_manager->host_JXYZA_data[i + constraints * 0]);
		icontact.JXYZA_U = (system_gpu->gpu_data_manager->host_JXYZA_data[i + constraints * 1]);
		icontact.JXYZA_W = (system_gpu->gpu_data_manager->host_JXYZA_data[i + constraints * 2]);

		icontact.JUVWA_N = (system_gpu->gpu_data_manager->host_JUVWA_data[i + constraints * 0]);
		icontact.JUVWA_U = (system_gpu->gpu_data_manager->host_JUVWA_data[i + constraints * 1]);
		icontact.JUVWA_W = (system_gpu->gpu_data_manager->host_JUVWA_data[i + constraints * 2]);

		icontact.JXYZB_N = (system_gpu->gpu_data_manager->host_JXYZB_data[i + constraints * 0]);
		icontact.JXYZB_U = (system_gpu->gpu_data_manager->host_JXYZB_data[i + constraints * 1]);
		icontact.JXYZB_W = (system_gpu->gpu_data_manager->host_JXYZB_data[i + constraints * 2]);

		icontact.JUVWB_N = (system_gpu->gpu_data_manager->host_JUVWB_data[i + constraints * 0]);
		icontact.JUVWB_U = (system_gpu->gpu_data_manager->host_JUVWB_data[i + constraints * 1]);
		icontact.JUVWB_W = (system_gpu->gpu_data_manager->host_JUVWB_data[i + constraints * 2]);

		icontact.lagrange.x = (system_gpu->gpu_data_manager->host_gam_data[i + constraints * 0]);
		icontact.lagrange.y = (system_gpu->gpu_data_manager->host_gam_data[i + constraints * 1]);
		icontact.lagrange.z = (system_gpu->gpu_data_manager->host_gam_data[i + constraints * 2]);

		icontact.rhs.x = rhs_gpu[i + constraints * 0];
		icontact.rhs.y = rhs_gpu[i + constraints * 1];
		icontact.rhs.z = rhs_gpu[i + constraints * 2];

		contact_gpu.push_back(icontact);
	}
	cout << "GContacts: " << contact_gpu.size() << endl;
//	for (int i = 0; i < system_gpu->gpu_data_manager->host_pair_data.size();
//			i++) {
//
//		long long p = system_gpu->gpu_data_manager->host_pair_data[i];
//		int2 pair = I2(int(p >> 32), int(p & 0xffffffff));
//		cout << pair.x << " " << pair.y << endl;
//
//	}

}

void comparecontacts(vector<contact_dat> & contact_cpu, vector<contact_dat> & contact_gpu, real tolerance) {
	cout << "validating contacts" << endl;
	std::sort(contact_cpu.begin(), contact_cpu.end());
	std::sort(contact_gpu.begin(), contact_gpu.end());
	for (int i = 0; i < contact_cpu.size(); i++) {

		real3 N = contact_cpu[i].N - contact_gpu[i].N;
		real3 Pa = contact_cpu[i].posA - contact_gpu[i].posA;
		real3 Pb = contact_cpu[i].posB - contact_gpu[i].posB;
		real d = contact_cpu[i].dist - contact_gpu[i].dist;

		validate_real3(contact_cpu[i].N, contact_gpu[i].N, "N ", i, tolerance);
		validate_real3(contact_cpu[i].posA, contact_gpu[i].posA, "Pa ", i, tolerance);
		validate_real3(contact_cpu[i].posB, contact_gpu[i].posB, "Pb  ", i, tolerance);

		if (fabs(d) > tolerance) {
			cout << "dist: [" << i << ", " << d <<" "<<contact_cpu[i].dist<<" "<<contact_gpu[i].dist<< endl;
		}

		validate_real3(contact_cpu[i].JXYZA_N, contact_gpu[i].JXYZA_N, "JXYZA_N ", i, tolerance);
		validate_real3(contact_cpu[i].JXYZA_U, contact_gpu[i].JXYZA_U, "JXYZA_U ", i, tolerance);
		validate_real3(contact_cpu[i].JXYZA_W, contact_gpu[i].JXYZA_W, "JXYZA_W ", i, tolerance);

		validate_real3(contact_cpu[i].JUVWA_N, contact_gpu[i].JUVWA_N, "JUVWA_N ", i, tolerance);
		validate_real3(contact_cpu[i].JUVWA_U, contact_gpu[i].JUVWA_U, "JUVWA_U ", i, tolerance);
		validate_real3(contact_cpu[i].JUVWA_W, contact_gpu[i].JUVWA_W, "JUVWA_W ", i, tolerance);

		validate_real3(contact_cpu[i].JXYZB_N, contact_gpu[i].JXYZB_N, "JXYZB_N ", i, tolerance);
		validate_real3(contact_cpu[i].JXYZB_U, contact_gpu[i].JXYZB_U, "JXYZB_U ", i, tolerance);
		validate_real3(contact_cpu[i].JXYZB_W, contact_gpu[i].JXYZB_W, "JXYZB_W ", i, tolerance);

		validate_real3(contact_cpu[i].JUVWB_N, contact_gpu[i].JUVWB_N, "JUVWB_N ", i, tolerance);
		validate_real3(contact_cpu[i].JUVWB_U, contact_gpu[i].JUVWB_U, "JUVWB_U ", i, tolerance);
		validate_real3(contact_cpu[i].JUVWB_W, contact_gpu[i].JUVWB_W, "JUVWB_W ", i, tolerance);

		validate_real3(contact_cpu[i].lagrange, contact_gpu[i].lagrange, "lagrange ", i, tolerance);
		validate_real3(contact_cpu[i].rhs, contact_gpu[i].rhs, "rhs ", i, tolerance);

		if (contact_cpu[i].idA != contact_gpu[i].idA || contact_cpu[i].idB != contact_gpu[i].idB) {
			cout << contact_cpu[i].idA << " " << contact_cpu[i].idB << " || " << contact_gpu[i].idA << " " << contact_gpu[i].idB << "\n";
		}
	}
	cout << "validating contacts - Done" << endl;
	contact_cpu.clear();
	contact_gpu.clear();

}
template<class T>
void dump_matricies(T* mdescriptor, chrono::ChSparseMatrix & mdM, chrono::ChMatrixDynamic<double> mdf) {
	chrono::ChSparseMatrix mdCq;
	chrono::ChSparseMatrix mdE;

	chrono::ChMatrixDynamic<double> mdb;
	chrono::ChMatrixDynamic<double> mdfric;
	mdescriptor->ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);
//	chrono::ChStreamOutAsciiFile file_M("dump_M.dat");
//	mdM.StreamOUTsparseMatlabFormat(file_M);
//	chrono::ChStreamOutAsciiFile file_Cq("dump_Cq.dat");
//	mdCq.StreamOUTsparseMatlabFormat(file_Cq);
//	chrono::ChStreamOutAsciiFile file_E("dump_E.dat");
//	mdE.StreamOUTsparseMatlabFormat(file_E);
//	chrono::ChStreamOutAsciiFile file_f("dump_f.dat");
//	mdf.StreamOUTdenseMatlabFormat(file_f);
//	chrono::ChStreamOutAsciiFile file_b("dump_b.dat");
//	mdb.StreamOUTdenseMatlabFormat(file_b);
//	chrono::ChStreamOutAsciiFile file_fric("dump_fric.dat");
//	mdfric.StreamOUTdenseMatlabFormat(file_fric);
}

void compare_variables(ChLcpSystemDescriptor* mdescriptor_cpu, ChLcpSystemDescriptor* mdescriptor_gpu, real tolerance) {
	std::vector<ChLcpVariables*> vvariables_cpu = mdescriptor_cpu->GetVariablesList();
	std::vector<ChLcpVariables*> vvariables_gpu = mdescriptor_gpu->GetVariablesList();
	chrono::ChMatrixDynamic<double> qb_vector_cpu;
	chrono::ChMatrixDynamic<double> qb_vector_gpu;

	qb_vector_cpu.Reset(vvariables_cpu.size() * 6, 1);
	qb_vector_gpu.Reset(vvariables_gpu.size() * 6, 1);

	//cout<<"VAR "<<vvariables_cpu.size() * 6<<" "<<vvariables_gpu.size() * 6<<endl;
	int s_q = 0;

	for (int i = 0; i < vvariables_cpu.size(); i++) {
		//cout<<s_q<<endl;
		//qb_vector_cpu.PasteMatrix(&vvariables_cpu[i]->Get_qb(), s_q, 0); // .. fills  'f'
		//qb_vector_gpu.PasteMatrix(&vvariables_gpu[i]->Get_qb(), s_q, 0); // .. fills  'f'

		real3 qb_v_cpu, qb_v_gpu;
		qb_v_cpu.x = vvariables_cpu[i]->Get_qb().Element(0, 0);
		qb_v_cpu.y = vvariables_cpu[i]->Get_qb().Element(1, 0);
		qb_v_cpu.z = vvariables_cpu[i]->Get_qb().Element(2, 0);

		qb_v_gpu.x = vvariables_gpu[i]->Get_qb().Element(0, 0);
		qb_v_gpu.y = vvariables_gpu[i]->Get_qb().Element(1, 0);
		qb_v_gpu.z = vvariables_gpu[i]->Get_qb().Element(2, 0);

		validate_real3(qb_v_cpu, qb_v_gpu, "QB_V  ", i, tolerance);
		real3 qb_r_cpu, qb_r_gpu;
		qb_r_cpu.x = vvariables_cpu[i]->Get_qb().Element(3, 0);
		qb_r_cpu.y = vvariables_cpu[i]->Get_qb().Element(4, 0);
		qb_r_cpu.z = vvariables_cpu[i]->Get_qb().Element(5, 0);

		qb_r_gpu.x = vvariables_gpu[i]->Get_qb().Element(3, 0);
		qb_r_gpu.y = vvariables_gpu[i]->Get_qb().Element(4, 0);
		qb_r_gpu.z = vvariables_gpu[i]->Get_qb().Element(5, 0);

		validate_real3(qb_r_cpu, qb_r_gpu, "QB_O  ", i, tolerance);

		s_q += 6;
	}
	//chrono::ChStreamOutAsciiFile file_qb_cpu("dump_qb_cpu.dat");
	//qb_vector_cpu.StreamOUTdenseMatlabFormat(file_qb_cpu);

	//chrono::ChStreamOutAsciiFile file_qb_gpu("dump_qb_gpu.dat");
	//qb_vector_gpu.StreamOUTdenseMatlabFormat(file_qb_gpu);
}

