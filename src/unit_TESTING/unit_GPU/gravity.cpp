#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <limits>
#include <time.h>
#include <cutil_inline.h>
#include <GL/freeglut.h>
#include "omp.h"
#include "unit_GPU/ChSystemGPU.h"
#include "unit_GPU/ChSystemMultiGPU.h"
#include "unit_GPU/ChLcpSolverGPU.h"
#include "unit_GPU/ChContactContainerGPUsimple.h"
#include "unit_GPU/ChCCollisionSystemGPU.h"
#include "unit_GPU/ChLcpSystemDescriptorGPU.h"
#include "unit_GPU/ChCCollisionModelGPU.h"
#include "unit_GPU/ChBodyGPU.h"
#include "unit_GPU/ChCuda.h"
#include "physics/ChApidll.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "collision/ChCModelBullet.h"
#include "collision/ChCCollisionSystemBullet.h"

using namespace chrono;
using namespace std;
#if defined( _WINDOWS )
#include <windows.h>
#define ISNAN(x) _isnan(x)
#else
#include <unistd.h>
#define ISNAN(x) isnan(x)
#endif

#define TOLERANCE  1e-2

#define CHBODYSHAREDPTR ChSharedBodyGPUPtr
#define CHBODY ChBodyGPU
#define CHCOLLISIONSYS ChCollisionSystemGPU
#define CHLCPDESC ChLcpSystemDescriptorGPU
#define CHCONTACTCONT ChContactContainerGPUsimple
#define CHSOLVER ChLcpSolverGPU
#define CHMODEL ChCollisionModelGPU
#define CHSYS ChSystemGPU

ChBODYSHAREDPTR FREE;
CHSYS *mSystem;

int frame_number=0;
float step_size=.001;
float current_time=0;


void InitObject(
		ChSharedPtr<ChBODY> &body,
		double mass,
		ChVector<> pos,
		ChQuaternion<> rot,
		double sfric,
		double kfric,
		double restitution,
		bool collide,
		bool fixed,
		int family,
		int nocolwith) {
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
void AddCollisionGeometry(ChSharedPtr<ChBODY> &body, ShapeType type, ChVector<> dim, ChVector<> lPos, ChQuaternion<> lRot) {
	ChMatrix33<> *rotation = new ChMatrix33<>(lRot);
	ChMODEL * model = (ChMODEL*) body->GetCollisionModel();
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
void FinalizeObject(ChSharedPtr<ChBODY> newbody) {
	newbody->GetCollisionModel()->BuildModel();
	mSystem->AddBody(newbody);
}
void DoTimeStep() {
	frame_number++;
	mSystem->DoStepDynamics(step_size);
	current_time += step_size;

	float correct = 9.80665 * current_time;
	float current = FREE->GetPos_dt().Length();
	//cout<< mSystem->GetChTime()<<endl;
	if (fabs(correct - current) > TOLERANCE) {
		cout << "FAIL at T=" << mSystem->GetChTime() << " correct: " << correct << " current: " << current << " " << fabs(correct - current) << endl;
		exit(1);
	}

}

int main(int argc, char* argv[]) {
mSystem = new CHSYS();
ChLCPDESC *mGPUDescriptor;
ChCONTACTCONT *mGPUContactContainer;
ChCOLLISIONSYS *mGPUCollisionEngine;
ChSOLVER *mGPUsolverSpeed;
	
mGPUDescriptor = (ChLCPDESC*) mSystem->GetLcpSystemDescriptor();
mGPUsolverSpeed = (ChSOLVER*) mSystem->GetLcpSolverSpeed();
mSystem->SetIntegrationType(ChSystem::INT_ANITESCU);
mSystem->Set_G_acc(ChVector<>(0, -9.80665, 0));
mSystem->SetStep(.001);
mGPUsolverSpeed->SetMaxIterations(1000);
mGPUsolverSpeed->SetOmega(.4);
mGPUsolverSpeed->SetTolerance(1e-8);
ChQuaternion<> quat(1, 0, 0, 0);
	ChVector<> lpos(0, 0, 0);
	FREE = ChBODYSHAREDPTR(new ChBODY);

	InitObject(FREE, 1.0, ChVector<>(0, 0, 0), quat, 0, 0, 0, false, false, -20, -20);
	AddCollisionGeometry(FREE, SPHERE, ChVector<>(1, 1, 1), lpos, quat);
	FinalizeObject(FREE);
	while (current_time<5) {
	DoTimeStep();
	}

	cout << "PASS" << endl;
	return 0;
}
