#include "unit_POSTPROCESS/ChMitsubaRender.h"
#include "physics/ChSystem.h"
#include "assets/ChSphereShape.h"
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "core/ChRealtimeStep.h"
#include "lcp/ChLcpIterativeSOR.h"
#include "lcp/ChLcpIterativeJacobi.h"
#include "collision/ChCModelBullet.h"
#include "collision/ChCCollisionSystemBullet.h"
#include "physics/ChContactContainer.h"
using namespace chrono;
using namespace chrono::collision;
using namespace postprocess;
using namespace geometry;
using namespace std;
using namespace tinyxml2;

#define PI 3.1415

#define CHBODYSHAREDPTR ChSharedBodyPtr
#define CHBODY ChBody
#define CHCOLLISIONSYS ChCollisionSystemBullet
#define CHLCPDESC ChLcpSystemDescriptor
#define CHCONTACTCONT ChContactContainer
#define CHSOLVER ChLcpIterativeJacobi
#define CHMODEL ChModelBullet
#define CHSYS ChSystem

void InitObject_(ChSharedPtr<CHBODY>& body,
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
    // body->SetLimitSpeed(true);
    // body->SetUseSleeping(true);
}

void AddCollisionGeometry_(ChSharedPtr<CHBODY>& body, int type, ChVector<> dim, ChVector<> lPos, ChQuaternion<> lRot) {
    CHMODEL* model = (CHMODEL*)body->GetCollisionModel();
    if (type == SPHERE) {
        model->AddSphere(dim.x, lPos);
    } else if (type == ELLIPSOID) {
        model->AddEllipsoid(dim.x, dim.y, dim.z, lPos, lRot);
    } else if (type == BOX) {
        model->AddBox(dim.x, dim.y, dim.z, lPos, lRot);
    } else if (type == CYLINDER) {
        model->AddCylinder(dim.x, dim.y, dim.z, lPos, lRot);
    }
}
void FinalizeObject_(ChSharedPtr<CHBODY> newbody, CHSYS* mSystem) {
    newbody->GetCollisionModel()->BuildModel();
    mSystem->AddBody(newbody);
}

int main() {
    ChSystem* mSys = new ChSystem;
    CHLCPDESC* mGPUDescriptor = new CHLCPDESC();
    CHCONTACTCONT* mGPUContactContainer = new CHCONTACTCONT();
    CHCOLLISIONSYS* mGPUCollisionEngine = new CHCOLLISIONSYS();
    CHSOLVER* mGPUsolverSpeed = new CHSOLVER();

    mSys->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);

    mSys->ChangeLcpSystemDescriptor(mGPUDescriptor);
    mSys->ChangeContactContainer(mGPUContactContainer);
    mSys->ChangeLcpSolverSpeed(mGPUsolverSpeed);
    mSys->ChangeCollisionSystem(mGPUCollisionEngine);

    mGPUDescriptor = (CHLCPDESC*)mSys->GetLcpSystemDescriptor();
    mGPUsolverSpeed = (CHSOLVER*)mSys->GetLcpSolverSpeed();

    // mGPUCollisionEngine = (CHCOLLISIONSYS*) mSystem->GetCollisionSystem();
    // mGPUContactContainer = (CHCONTACTCONT*) mSystem->GetContactContainer();

    mSys->SetIntegrationType(ChSystem::INT_ANITESCU);
    mSys->Set_G_acc(ChVector<>(0, -9.80665, 0));

    ChSharedBodyPtr sphere;
    ChVector<> lpos(0, 0, 0);
    ChQuaternion<> quat(1, 0, 0, 0);
    for (int i = 0; i < 1000; i++) {
        sphere = ChSharedBodyPtr(new ChBody);
        InitObject_(sphere, 1.0, ChVector<>((rand() % 10000 / 1000.0 - 5) * 2, (rand() % 10000 / 1000.0 - 5) * 2,
                                            (rand() % 10000 / 1000.0 - 5) * 2),
                    quat, 0, 0, 0, true, false, -1, i);
        AddCollisionGeometry_(sphere, SPHERE, ChVector<>(.06, .06, .06), lpos, quat);
        FinalizeObject_(sphere, mSys);
        ChSharedPtr<ChSphereShape> sphere_shape(new ChSphereShape);
        sphere_shape->SetColor(ChColor(1, 0, 0));
        sphere_shape->GetSphereGeometry().rad = .06;

        sphere->GetAssets().push_back(sphere_shape);
    }
    float mWallMu = 1;

    float container_R = -7.0, container_T = .1;

    CHBODYSHAREDPTR L = CHBODYSHAREDPTR(new CHBODY);
    CHBODYSHAREDPTR R = CHBODYSHAREDPTR(new CHBODY);
    CHBODYSHAREDPTR F = CHBODYSHAREDPTR(new CHBODY);
    CHBODYSHAREDPTR B = CHBODYSHAREDPTR(new CHBODY);
    CHBODYSHAREDPTR BTM = CHBODYSHAREDPTR(new CHBODY);
    CHBODYSHAREDPTR FREE = CHBODYSHAREDPTR(new CHBODY);
    ChQuaternion<> quat2(1, 0, 0, 0);
    quat2.Q_from_AngAxis(PI / 6.0, ChVector<>(1, 0, 0));
    InitObject_(L, 100000, ChVector<>(-container_R, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
    InitObject_(R, 100000, ChVector<>(container_R, 0, 0), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
    InitObject_(F, 100000, ChVector<>(0, 0, -container_R), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
    InitObject_(B, 100000, ChVector<>(0, 0, container_R), quat, mWallMu, mWallMu, 0, true, true, -20, -20);
    InitObject_(BTM, 1, ChVector<>(0, -container_R, 0), quat, mWallMu, mWallMu, 0, true, true, -1000, -20000);

    AddCollisionGeometry_(L, BOX, ChVector<>(container_T, container_R, container_R), lpos, quat);
    AddCollisionGeometry_(R, BOX, ChVector<>(container_T, container_R, container_R), lpos, quat);
    AddCollisionGeometry_(F, BOX, ChVector<>(container_R, container_R, container_T), lpos, quat);
    AddCollisionGeometry_(B, BOX, ChVector<>(container_R, container_R, container_T), lpos, quat);
    AddCollisionGeometry_(BTM, BOX, ChVector<>(container_R, container_T, container_R), lpos, quat);

    // FinalizeObject(L, mSys);
    // FinalizeObject(R, mSys);
    // FinalizeObject(F, mSys);
    // FinalizeObject(B, mSys);
    FinalizeObject_(BTM, mSys);

    ChMitsubaRender output(mSys);
    //
    output.SetIntegrator("photonmapper");
    output.SetIntegratorOption("integer", "maxDepth", "32");
    output.SetFilm("ldrfilm");
    output.SetFilmOption("integer", "height", "1200");
    output.SetFilmOption("integer", "width", "1920");

    output.camera_target = ChVector<>(0, 0, 0);
    output.camera_origin = ChVector<>(0, 0, -10);
    output.camera_up = ChVector<>(0, 1, 0);

    output.SetDataFolder("data");
    output.ExportScript("test.xml");
    mSys->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR_MULTITHREAD);
    mSys->SetIterLCPmaxItersSpeed(20);
    int file = 0;
    for (int i = 0; i < 100; i++) {
        mSys->DoStepDynamics(.1);
        stringstream ss;
        ss << "data/" << file << ".xml";
        output.ExportData(ss.str());
        file++;
        cout << "Frame: " << file << endl;
    }
}
