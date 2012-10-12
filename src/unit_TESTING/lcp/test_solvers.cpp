#include "lcp/ChLcpVariablesGeneric.h"
#include "lcp/ChLcpVariablesBody.h"
#include "lcp/ChLcpConstraintTwoGeneric.h"
#include "lcp/ChLcpSystemDescriptor.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpSimplexSolver.h"
#include "core/ChLinearAlgebra.h"
#include "physics/ChSystem.h"
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
// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
#define CHBODYSHAREDPTR ChSharedBodyPtr
#define CHBODY ChBody
#define CHCOLLISIONSYS ChCollisionSystemBullet
#define CHLCPDESC ChLcpSystemDescriptor
#define CHCONTACTCONT ChContactContainer
#define CHSOLVER ChLcpIterativeJacobi
#define CHMODEL ChModelBullet
#define CHSYS ChSystem

void InitObject(ChSharedPtr<CHBODY> &body, double mass, ChVector<> pos, ChQuaternion<> rot, double sfric, double kfric, double restitution, bool collide, bool fixed, int family, int nocolwith) {
	body->SetMass(mass);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetCollide(collide);
	body->SetBodyFixed(fixed);
	body->SetImpactC(restitution);
	body->SetSfriction(sfric);
	body->SetKfriction(kfric);
	body->GetCollisionModel()->ClearModel();
	body->GetCollisionModel()->SetFamily(family);
	body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(nocolwith);
	//body->SetLimitSpeed(true);
	//body->SetUseSleeping(true);
}

void AddCollisionGeometry(ChSharedPtr<CHBODY> &body, ShapeType type, ChVector<> dim, ChVector<> lPos, ChQuaternion<> lRot) {
	ChMatrix33<> *rotation = new ChMatrix33<>(lRot);
	CHMODEL * model = (CHMODEL*) body->GetCollisionModel();
	if (type == SPHERE) {
		model->AddSphere(dim.x, &lPos);
	} else if (type == ELLIPSOID) {
		model->AddEllipsoid(dim.x, dim.y, dim.z, &lPos, rotation);
	} else if (type == BOX) {
		model->AddBox(dim.x, dim.y, dim.z, &lPos, rotation);
	} else if (type == CYLINDER) {
		model->AddCylinder(dim.x, dim.y, dim.z, &lPos, rotation);
	}
}
void FinalizeObject(ChSharedPtr<CHBODY> newbody, CHSYS * mSystem) {
	newbody->GetCollisionModel()->BuildModel();
	mSystem->AddBody(newbody);
}

void dump_matricies(ChLcpSystemDescriptor& mdescriptor) {

	chrono::ChSparseMatrix mdM;
	chrono::ChSparseMatrix mdCq;
	chrono::ChSparseMatrix mdE;
	chrono::ChMatrixDynamic<double> mdf;
	chrono::ChMatrixDynamic<double> mdb;
	chrono::ChMatrixDynamic<double> mdfric;
	mdescriptor.ConvertToMatrixForm(&mdCq, &mdM, &mdE, &mdf, &mdb, &mdfric);
	chrono::ChStreamOutAsciiFile file_M("dump_M.dat");
	mdM.StreamOUTsparseMatlabFormat(file_M);
	chrono::ChStreamOutAsciiFile file_Cq("dump_Cq.dat");
	mdCq.StreamOUTsparseMatlabFormat(file_Cq);
	chrono::ChStreamOutAsciiFile file_E("dump_E.dat");
	mdE.StreamOUTsparseMatlabFormat(file_E);
	chrono::ChStreamOutAsciiFile file_f("dump_f.dat");
	mdf.StreamOUTdenseMatlabFormat(file_f);
	chrono::ChStreamOutAsciiFile file_b("dump_b.dat");
	mdb.StreamOUTdenseMatlabFormat(file_b);
	chrono::ChStreamOutAsciiFile file_fric("dump_fric.dat");
	mdfric.StreamOUTdenseMatlabFormat(file_fric);
}

int main(int argc, char* argv[]) {

	ChSystem* mSys = new ChSystem();
	ChLcpSystemDescriptor *mdescriptor = new ChLcpSystemDescriptor();
	ChContactContainer *mcontactcontainer = new ChContactContainer();
	ChCollisionSystemBullet *mcollisionengine = new ChCollisionSystemBullet();
	ChLcpIterativeJacobi *msolver = new ChLcpIterativeJacobi();

	mSys->ChangeLcpSystemDescriptor(mdescriptor);
	mSys->ChangeContactContainer(mcontactcontainer);
	mSys->ChangeLcpSolverSpeed(msolver);
	mSys->ChangeCollisionSystem(mcollisionengine);

	mSys->SetIntegrationType(ChSystem::INT_ANITESCU);
	mSys->Set_G_acc(ChVector<>(0, -9.80665, 0));

	ChVector<> lpos(0, 0, 0);
		ChQuaternion<> quat(1, 0, 0, 0);
	ChSharedBodyPtr sphere;
	for (int i = 0; i < 1000; i++) {
		sphere = ChSharedBodyPtr(new ChBody);
		InitObject(sphere, 1.0, ChVector<>((rand() % 10000 / 1000.0 - 5) * 2, (rand() % 10000 / 1000.0 - 5) * 2, (rand() % 10000 / 1000.0 - 5) * 2), quat, 0, 0, 0, true, false, -1, i);
		AddCollisionGeometry(sphere, SPHERE, ChVector<>(.06, .06, .06), lpos, quat);
		FinalizeObject(sphere, mSys);
	}


	float mWallMu = 1, container_width = -7.0, container_thickness = .1, container_height = -7.0, wscale = 1;
	ChSharedBodyPtr L = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr R = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr F = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr B = ChSharedBodyPtr(new ChBody);
	ChSharedBodyPtr BTM = ChSharedBodyPtr(new ChBody);

	InitObject(L, 100000, ChVector<>(-container_width + container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(R, 100000, ChVector<>(container_width - container_thickness, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(F, 100000, ChVector<>(0, 0, -container_width + container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(B, 100000, ChVector<>(0, 0, container_width - container_thickness), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
	InitObject(BTM, 100000, ChVector<>(0, -container_height + container_thickness, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);

	AddCollisionGeometry(L, BOX, ChVector<>(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(R, BOX, ChVector<>(container_thickness, container_height, container_width), lpos, quat);
	AddCollisionGeometry(F, BOX, ChVector<>(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(B, BOX, ChVector<>(container_width * wscale, container_height, container_thickness), lpos, quat);
	AddCollisionGeometry(BTM, BOX, ChVector<>(container_width * wscale, container_thickness * .25, container_width), lpos, quat);

	FinalizeObject(L, mSys);
	FinalizeObject(R, mSys);
	FinalizeObject(F, mSys);
	FinalizeObject(B, mSys);
	FinalizeObject(BTM, mSys);
	mSys->DoStepDynamics(.01);


	//msolver->Solve(*mdescriptor,true);

	dump_matricies(*mdescriptor);

	//ChLcpIterativePMINRES msolver_krylov(20, false, 0.00001);
	//msolver_krylov.Solve(mdescriptor);

	return 0;
}

