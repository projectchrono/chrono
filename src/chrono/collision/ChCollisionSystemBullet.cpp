// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/collision/ChCollisionSystemBullet.h"
#include "chrono/collision/ChCollisionModelBullet.h"
#include "chrono/collision/ChCollisionAlgorithmsBullet.h"
#include "chrono/collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"
#include "chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"

extern btScalar gContactBreakingThreshold;

namespace chrono {
namespace collision {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionSystemBullet)

ChCollisionSystemBullet::ChCollisionSystemBullet() {
    // btDefaultCollisionConstructionInfo conf_info(...); ***TODO***
    bt_collision_configuration = new btDefaultCollisionConfiguration();

#ifdef BT_USE_OPENMP
    bt_dispatcher = new btCollisionDispatcherMt(bt_collision_configuration);  // parallel version
    btSetTaskScheduler(btGetOpenMPTaskScheduler());
#else
    bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);  // serial version
#endif

    bt_broadphase = new btDbvtBroadphase();
    bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

    // custom collision for cylinder-sphere case, for improved precision   
    ////btCollisionAlgorithmCreateFunc* m_collision_sph_cyl = new btSphereCylinderCollisionAlgorithm::CreateFunc;
    ////btCollisionAlgorithmCreateFunc* m_collision_cyl_sph = new btSphereCylinderCollisionAlgorithm::CreateFunc;
    ////m_collision_cyl_sph->m_swapped = true;
    ////bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE, CYLINDER_SHAPE_PROXYTYPE, m_collision_sph_cyl);
    ////bt_dispatcher->registerCollisionCreateFunc(CYLINDER_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE, m_collision_cyl_sph);
    
    // custom collision for capsule-box case
    m_collision_capsule_box = new btCapsuleBoxCollisionAlgorithm::CreateFunc;
    m_collision_box_capsule = new btCapsuleBoxCollisionAlgorithm::CreateFunc;
    m_collision_box_capsule->m_swapped = true;
    bt_dispatcher->registerCollisionCreateFunc(CAPSULE_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, m_collision_capsule_box);
    bt_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, CAPSULE_SHAPE_PROXYTYPE, m_collision_box_capsule);

    // custom collision for cylshell-box case
    m_collision_cylshell_box = new btCylshellBoxCollisionAlgorithm::CreateFunc;
    m_collision_box_cylshell = new btCylshellBoxCollisionAlgorithm::CreateFunc;
    m_collision_box_cylshell->m_swapped = true;
    bt_dispatcher->registerCollisionCreateFunc(CYLSHELL_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, m_collision_cylshell_box);
    bt_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, CYLSHELL_SHAPE_PROXYTYPE, m_collision_box_cylshell);

    // custom collision for 2D arc-segment case
    m_collision_arc_seg = new btArcSegmentCollisionAlgorithm::CreateFunc;
    m_collision_seg_arc = new btArcSegmentCollisionAlgorithm::CreateFunc;
    m_collision_seg_arc->m_swapped = true;
    bt_dispatcher->registerCollisionCreateFunc(ARC_SHAPE_PROXYTYPE, SEGMENT_SHAPE_PROXYTYPE, m_collision_arc_seg);
    bt_dispatcher->registerCollisionCreateFunc(SEGMENT_SHAPE_PROXYTYPE, ARC_SHAPE_PROXYTYPE, m_collision_seg_arc);

    // custom collision for 2D arc-arc case
    m_collision_arc_arc = new btArcArcCollisionAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(ARC_SHAPE_PROXYTYPE, ARC_SHAPE_PROXYTYPE, m_collision_arc_arc);

    // custom collision for C::E triangles:
    m_collision_cetri_cetri = new btCEtriangleShapeCollisionAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(CE_TRIANGLE_SHAPE_PROXYTYPE, CE_TRIANGLE_SHAPE_PROXYTYPE,
                                               m_collision_cetri_cetri);

    // custom collision for point-point case (in point clouds, just never create point-point contacts)
    // btCollisionAlgorithmCreateFunc* m_collision_point_point = new btPointPointCollisionAlgorithm::CreateFunc;
    m_tmp_mem = btAlignedAlloc(sizeof(btEmptyAlgorithm::CreateFunc), 16);
    m_emptyCreateFunc = new (m_tmp_mem) btEmptyAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(POINT_SHAPE_PROXYTYPE, POINT_SHAPE_PROXYTYPE, m_emptyCreateFunc);
    bt_dispatcher->registerCollisionCreateFunc(POINT_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE,
                                               bt_collision_configuration->getCollisionAlgorithmCreateFunc(
                                                   SPHERE_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE));  // just for speedup
    bt_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, POINT_SHAPE_PROXYTYPE,
                                               bt_collision_configuration->getCollisionAlgorithmCreateFunc(
                                                   BOX_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE));  // just for speedup

    // custom collision for GIMPACT mesh case too
    btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);
}

ChCollisionSystemBullet::~ChCollisionSystemBullet() {
    delete bt_collision_world;
    delete bt_broadphase;
    delete bt_dispatcher;
    delete bt_collision_configuration;

    delete m_collision_capsule_box;
    delete m_collision_box_capsule;
    delete m_collision_cylshell_box;
    delete m_collision_box_cylshell;
    delete m_collision_arc_seg;
    delete m_collision_seg_arc;
    delete m_collision_arc_arc;
    delete m_collision_cetri_cetri;
    m_emptyCreateFunc->~btCollisionAlgorithmCreateFunc();
    btAlignedFree(m_tmp_mem);
}

void ChCollisionSystemBullet::SetNumThreads(int nthreads) {
#ifdef BT_USE_OPENMP
    btGetOpenMPTaskScheduler()->setNumThreads(nthreads);
#endif
}

void ChCollisionSystemBullet::Clear(void) {
    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        contactManifold->clearManifold();
    }
}

void ChCollisionSystemBullet::Add(ChCollisionModel* model) {
    auto model_bt = static_cast<ChCollisionModelBullet*>(model);
    if (model_bt->GetBulletModel()->getCollisionShape()) {
        model->SyncPosition();
        bt_collision_world->addCollisionObject(model_bt->GetBulletModel(), model_bt->GetFamilyGroup(),
                                               model_bt->GetFamilyMask());
    }
}

void ChCollisionSystemBullet::Remove(ChCollisionModel* model) {
    auto model_bt = static_cast<ChCollisionModelBullet*>(model);
    if (model_bt->GetBulletModel()->getCollisionShape()) {
        bt_collision_world->removeCollisionObject(model_bt->GetBulletModel());
    }
}

void ChCollisionSystemBullet::Run() {
    if (bt_collision_world) {
        bt_collision_world->performDiscreteCollisionDetection();
    }
	
	//int numPairs = bt_collision_world->getBroadphase()->getOverlappingPairCache()->getNumOverlappingPairs();	
	//GetLog() << "tot pairs: " << numPairs << "\n";
	
}

void ChCollisionSystemBullet::GetBoundingBox(ChVector<>& aabb_min, ChVector<>& aabb_max) const {
    btVector3 aabbMin;
    btVector3 aabbMax;
    bt_broadphase->getBroadphaseAabb(aabbMin, aabbMax);
    aabb_min = ChVector<>((double)aabbMin.x(), (double)aabbMin.y(), (double)aabbMin.z());
    aabb_max = ChVector<>((double)aabbMax.x(), (double)aabbMax.y(), (double)aabbMax.z());
}

void ChCollisionSystemBullet::ResetTimers() {
    bt_collision_world->timer_collision_broad.reset();
    bt_collision_world->timer_collision_narrow.reset();
}

double ChCollisionSystemBullet::GetTimerCollisionBroad() const {
    return bt_collision_world->timer_collision_broad();
}

double ChCollisionSystemBullet::GetTimerCollisionNarrow() const {
    return bt_collision_world->timer_collision_narrow();
}

void ChCollisionSystemBullet::ReportContacts(ChContactContainer* mcontactcontainer) {
    // This should remove all old contacts (or at least rewind the index)
    mcontactcontainer->BeginAddContact();

    // NOTE: Bullet does not provide information on radius of curvature at a contact point.
    // As such, for all Bullet-identified contacts, the default value will be used (SMC only).
    ChCollisionInfo icontact;

    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

        icontact.modelA = (ChCollisionModel*)obA->getUserPointer();
        icontact.modelB = (ChCollisionModel*)obB->getUserPointer();

        double envelopeA = icontact.modelA->GetEnvelope();
        double envelopeB = icontact.modelB->GetEnvelope();

        double marginA = icontact.modelA->GetSafeMargin();
        double marginB = icontact.modelB->GetSafeMargin();

        // Execute custom broadphase callback, if any
        bool do_narrow_contactgeneration = true;
        if (this->broad_callback)
            do_narrow_contactgeneration = this->broad_callback->OnBroadphase(icontact.modelA, icontact.modelB);

        if (do_narrow_contactgeneration) {
            int numContacts = contactManifold->getNumContacts();
            // GetLog() << "numContacts=" << numContacts << "\n";
            for (int j = 0; j < numContacts; j++) {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);

                // Discard "too far" constraints (the Bullet engine also has its threshold)
                if (pt.getDistance() < marginA + marginB) {
                    btVector3 ptA = pt.getPositionWorldOnA();
                    btVector3 ptB = pt.getPositionWorldOnB();

                    icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
                    icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());

                    icontact.vN.Set(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(),
                                    -pt.m_normalWorldOnB.getZ());
                    icontact.vN.Normalize();

                    double ptdist = pt.getDistance();

                    icontact.vpA = icontact.vpA - icontact.vN * envelopeA;
                    icontact.vpB = icontact.vpB + icontact.vN * envelopeB;
                    icontact.distance = ptdist + envelopeA + envelopeB;

                    icontact.reaction_cache = pt.reactions_cache;

                    bool compoundA = (obA->getCollisionShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);
                    bool compoundB = (obB->getCollisionShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);

                    int indexA = compoundA ? pt.m_index0 : 0;
                    int indexB = compoundB ? pt.m_index1 : 0;

                    icontact.shapeA = icontact.modelA->GetShape(indexA).get();
                    icontact.shapeB = icontact.modelB->GetShape(indexB).get();

                    // Execute some user custom callback, if any
                    bool add_contact = true;
                    if (this->narrow_callback)
                        add_contact = this->narrow_callback->OnNarrowphase(icontact);

                    // Add to contact container
                    if (add_contact) {
                        ////std::cout << " add indexA=" << indexA << " indexB=" << indexB << std::endl;
                        ////std::cout << "     typeA=" << icontact.shapeA->m_type << " typeB=" << icontact.shapeB->m_type
                        ////          << std::endl;
                        mcontactcontainer->AddContact(icontact);
                    }
                }
            }
        }

        // Uncomment this line to remove all points
        ////contactManifold->clearManifold();
    }
    mcontactcontainer->EndAddContact();
}

void ChCollisionSystemBullet::ReportProximities(ChProximityContainer* mproximitycontainer) {
    mproximitycontainer->BeginAddProximities();

    int numPairs = bt_collision_world->getBroadphase()->getOverlappingPairCache()->getNumOverlappingPairs();
    for (int i = 0; i < numPairs; i++) {
        btBroadphasePair mp =
            bt_collision_world->getBroadphase()->getOverlappingPairCache()->getOverlappingPairArray().at(i);

        btCollisionObject* obA = static_cast<btCollisionObject*>(mp.m_pProxy0->m_clientObject);
        btCollisionObject* obB = static_cast<btCollisionObject*>(mp.m_pProxy1->m_clientObject);

        ChCollisionModel* modelA = (ChCollisionModel*)obA->getUserPointer();
        ChCollisionModel* modelB = (ChCollisionModel*)obB->getUserPointer();

        // Add to proximity container
        mproximitycontainer->AddProximity(modelA, modelB);
    }
    mproximitycontainer->EndAddProximities();
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult) const {
    return RayHit(from, to, mresult, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChRayhitResult& mresult,
                                     short int filter_group,
                                     short int filter_mask) const {
    btVector3 btfrom((btScalar)from.x(), (btScalar)from.y(), (btScalar)from.z());
    btVector3 btto((btScalar)to.x(), (btScalar)to.y(), (btScalar)to.z());

    btCollisionWorld::ClosestRayResultCallback rayCallback(btfrom, btto);
    rayCallback.m_collisionFilterGroup = filter_group;
    rayCallback.m_collisionFilterMask = filter_mask;

    this->bt_collision_world->rayTest(btfrom, btto, rayCallback);

    if (rayCallback.hasHit()) {
        mresult.hitModel = (ChCollisionModel*)(rayCallback.m_collisionObject->getUserPointer());
        if (mresult.hitModel) {
            mresult.hit = true;
            mresult.abs_hitPoint.Set(rayCallback.m_hitPointWorld.x(), rayCallback.m_hitPointWorld.y(),
                                     rayCallback.m_hitPointWorld.z());
            mresult.abs_hitNormal.Set(rayCallback.m_hitNormalWorld.x(), rayCallback.m_hitNormalWorld.y(),
                                      rayCallback.m_hitNormalWorld.z());
            mresult.abs_hitNormal.Normalize();
            mresult.hit = true;
            mresult.dist_factor = rayCallback.m_closestHitFraction;
            mresult.abs_hitPoint = mresult.abs_hitPoint - mresult.abs_hitNormal * mresult.hitModel->GetEnvelope();
            return true;
        }
    }
    mresult.hit = false;
    return false;
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChCollisionModel* model,
                                     ChRayhitResult& mresult) const {
    return RayHit(from, to, model, mresult, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChCollisionModel* model,
                                     ChRayhitResult& mresult,
                                     short int filter_group,
                                     short int filter_mask) const {
    btVector3 btfrom((btScalar)from.x(), (btScalar)from.y(), (btScalar)from.z());
    btVector3 btto((btScalar)to.x(), (btScalar)to.y(), (btScalar)to.z());

    btCollisionWorld::AllHitsRayResultCallback rayCallback(btfrom, btto);
    rayCallback.m_collisionFilterGroup = filter_group;
    rayCallback.m_collisionFilterMask = filter_mask;

    this->bt_collision_world->rayTest(btfrom, btto, rayCallback);

    // Find the closest hit result on the specified model (if any)
    int hit = -1;
    btScalar fraction = 1;
    for (int i = 0; i < rayCallback.m_collisionObjects.size(); ++i) {
        if (rayCallback.m_collisionObjects[i]->getUserPointer() == model && rayCallback.m_hitFractions[i] < fraction) {
            hit = i;
            fraction = rayCallback.m_hitFractions[i];
        }
    }

    // Ray does not hit specified model
    if (hit == -1) {
        mresult.hit = false;
        return false;
    }

    // Return the closest hit on the specified model
    mresult.hit = true;
    mresult.hitModel = static_cast<ChCollisionModel*>(rayCallback.m_collisionObjects[hit]->getUserPointer());
    mresult.abs_hitPoint.Set(rayCallback.m_hitPointWorld[hit].x(), rayCallback.m_hitPointWorld[hit].y(),
                             rayCallback.m_hitPointWorld[hit].z());
    mresult.abs_hitNormal.Set(rayCallback.m_hitNormalWorld[hit].x(), rayCallback.m_hitNormalWorld[hit].y(),
                              rayCallback.m_hitNormalWorld[hit].z());
    mresult.abs_hitNormal.Normalize();
    mresult.dist_factor = fraction;
    mresult.abs_hitPoint = mresult.abs_hitPoint - mresult.abs_hitNormal * mresult.hitModel->GetEnvelope();
    return true;
}

void ChCollisionSystemBullet::SetContactBreakingThreshold(double threshold) {
    gContactBreakingThreshold = (btScalar)threshold;
}

}  // end namespace collision
}  // end namespace chrono
