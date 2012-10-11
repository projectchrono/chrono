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


#define CHBODYSHAREDPTR ChSharedBodyGPUPtr
#define CHBODY ChBodyGPU
#define CHCOLLISIONSYS ChCollisionSystemGPU
#define CHLCPDESC ChLcpSystemDescriptorGPU
#define CHCONTACTCONT ChContactContainerGPUsimple
#define CHSOLVER ChLcpSolverGPU
#define CHMODEL ChCollisionModelGPU
#define CHSYS ChSystemGPU
/*
#define CHBODYSHAREDPTR ChSharedBodyPtr
#define CHBODY ChBody
#define CHCOLLISIONSYS ChCollisionSystemBullet
#define CHLCPDESC ChLcpSystemDescriptor
#define CHCONTACTCONT ChContactContainer
#define CHSOLVER ChLcpIterativeJacobi
#define CHMODEL ChModelBullet
#define CHSYS ChSystem
*/
CHSYS *mSystem;
CHLCPDESC *mGPUDescriptor;
CHCONTACTCONT *mGPUContactContainer;
CHCOLLISIONSYS *mGPUCollisionEngine;
CHSOLVER *mGPUsolverSpeed;

void InitObject(
		ChSharedPtr<CHBODY> &body,
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
void FinalizeObject(ChSharedPtr<CHBODY> newbody) {
	newbody->GetCollisionModel()->BuildModel();
	mSystem->AddBody(newbody);
}
