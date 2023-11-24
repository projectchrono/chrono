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

#include <algorithm>

#include "chrono/physics/ChContactContainer.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/fea/ChMesh.h"

#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/collision/bullet/ChCollisionModelBullet.h"
#include "chrono/collision/bullet/ChCollisionAlgorithmsBullet.h"
#include "chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactCollisionAlgorithm.h"
#include "chrono/collision/bullet/BulletCollision/CollisionDispatch/cbtCollisionDispatcherMt.h"
#include "chrono/collision/bullet/LinearMath/cbtIDebugDraw.h"

extern cbtScalar gContactBreakingThreshold;

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChCollisionSystemBullet)
CH_UPCASTING(ChCollisionSystemBullet, ChCollisionSystem)

ChCollisionSystemBullet::ChCollisionSystemBullet() : m_debug_drawer(nullptr) {
    bt_collision_configuration = new cbtDefaultCollisionConfiguration();

#ifdef BT_USE_OPENMP
    bt_dispatcher = new cbtCollisionDispatcherMt(bt_collision_configuration);  // parallel version
    cbtSetTaskScheduler(cbtGetOpenMPTaskScheduler());
#else
    bt_dispatcher = new cbtCollisionDispatcher(bt_collision_configuration);  // serial version
#endif

    bt_broadphase = new cbtDbvtBroadphase();
    bt_collision_world = new cbtCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

    // custom collision for cylinder-sphere case, for improved precision
    ////cbtCollisionAlgorithmCreateFunc* m_collision_sph_cyl = new cbtSphereCylinderCollisionAlgorithm::CreateFunc;
    ////cbtCollisionAlgorithmCreateFunc* m_collision_cyl_sph = new cbtSphereCylinderCollisionAlgorithm::CreateFunc;
    ////m_collision_cyl_sph->m_swapped = true;
    ////bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE, CYLINDER_SHAPE_PROXYTYPE,
    /// m_collision_sph_cyl); /bt_dispatcher->registerCollisionCreateFunc(CYLINDER_SHAPE_PROXYTYPE,
    /// SPHERE_SHAPE_PROXYTYPE, m_collision_cyl_sph);

    // custom collision for capsule-box case
    m_collision_capsule_box = new cbtCapsuleBoxCollisionAlgorithm::CreateFunc;
    m_collision_box_capsule = new cbtCapsuleBoxCollisionAlgorithm::CreateFunc;
    m_collision_box_capsule->m_swapped = true;
    bt_dispatcher->registerCollisionCreateFunc(CAPSULE_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, m_collision_capsule_box);
    bt_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, CAPSULE_SHAPE_PROXYTYPE, m_collision_box_capsule);

    // custom collision for cylshell-box case
    m_collision_cylshell_box = new cbtCylshellBoxCollisionAlgorithm::CreateFunc;
    m_collision_box_cylshell = new cbtCylshellBoxCollisionAlgorithm::CreateFunc;
    m_collision_box_cylshell->m_swapped = true;
    bt_dispatcher->registerCollisionCreateFunc(CYLSHELL_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE, m_collision_cylshell_box);
    bt_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, CYLSHELL_SHAPE_PROXYTYPE, m_collision_box_cylshell);

    // custom collision for 2D arc-segment case
    m_collision_arc_seg = new cbtArcSegmentCollisionAlgorithm::CreateFunc;
    m_collision_seg_arc = new cbtArcSegmentCollisionAlgorithm::CreateFunc;
    m_collision_seg_arc->m_swapped = true;
    bt_dispatcher->registerCollisionCreateFunc(ARC_SHAPE_PROXYTYPE, SEGMENT_SHAPE_PROXYTYPE, m_collision_arc_seg);
    bt_dispatcher->registerCollisionCreateFunc(SEGMENT_SHAPE_PROXYTYPE, ARC_SHAPE_PROXYTYPE, m_collision_seg_arc);

    // custom collision for 2D arc-arc case
    m_collision_arc_arc = new cbtArcArcCollisionAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(ARC_SHAPE_PROXYTYPE, ARC_SHAPE_PROXYTYPE, m_collision_arc_arc);

    // custom collision for C::E triangles:
    m_collision_cetri_cetri = new cbtCEtriangleShapeCollisionAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(CE_TRIANGLE_SHAPE_PROXYTYPE, CE_TRIANGLE_SHAPE_PROXYTYPE,
                                               m_collision_cetri_cetri);

    // custom collision for point-point case (in point clouds, just never create point-point contacts)
    // cbtCollisionAlgorithmCreateFunc* m_collision_point_point = new cbtPointPointCollisionAlgorithm::CreateFunc;
    m_tmp_mem = cbtAlignedAlloc(sizeof(cbtEmptyAlgorithm::CreateFunc), 16);
    m_emptyCreateFunc = new (m_tmp_mem) cbtEmptyAlgorithm::CreateFunc;
    bt_dispatcher->registerCollisionCreateFunc(POINT_SHAPE_PROXYTYPE, POINT_SHAPE_PROXYTYPE, m_emptyCreateFunc);
    bt_dispatcher->registerCollisionCreateFunc(POINT_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE,
                                               bt_collision_configuration->getCollisionAlgorithmCreateFunc(
                                                   SPHERE_SHAPE_PROXYTYPE, BOX_SHAPE_PROXYTYPE));  // just for speedup
    bt_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE, POINT_SHAPE_PROXYTYPE,
                                               bt_collision_configuration->getCollisionAlgorithmCreateFunc(
                                                   BOX_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE));  // just for speedup

    // custom collision for GIMPACT mesh case too
    cbtGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);
}

ChCollisionSystemBullet::~ChCollisionSystemBullet() {
    delete bt_collision_world;
    delete bt_broadphase;
    delete bt_dispatcher;
    delete bt_collision_configuration;

    delete m_debug_drawer;

    delete m_collision_capsule_box;
    delete m_collision_box_capsule;
    delete m_collision_cylshell_box;
    delete m_collision_box_cylshell;
    delete m_collision_arc_seg;
    delete m_collision_seg_arc;
    delete m_collision_arc_arc;
    delete m_collision_cetri_cetri;
    m_emptyCreateFunc->~cbtCollisionAlgorithmCreateFunc();
    cbtAlignedFree(m_tmp_mem);
}

void ChCollisionSystemBullet::SetNumThreads(int nthreads) {
#ifdef BT_USE_OPENMP
    cbtGetOpenMPTaskScheduler()->setNumThreads(nthreads);
#endif
}

void ChCollisionSystemBullet::Add(std::shared_ptr<ChCollisionModel> model) {
    if (model->HasImplementation())
        return;

    auto bt_model = chrono_types::make_shared<ChCollisionModelBullet>(model.get());
    bt_model->Populate();
    if (bt_model->GetBulletObject()->getCollisionShape()) {
        model->SyncPosition();
        bt_collision_world->addCollisionObject(bt_model->bt_collision_object.get(),  //
                                               bt_model->model->GetFamilyGroup(),    //
                                               bt_model->model->GetFamilyMask());
    }
    bt_models.push_back(bt_model);
}

void ChCollisionSystemBullet::Clear() {
    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        cbtPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        contactManifold->clearManifold();
    }
    bt_models.clear();
}

void ChCollisionSystemBullet::Remove(std::shared_ptr<ChCollisionModel> model) {
    if (!model->HasImplementation())
        return;

    auto bt_model = (ChCollisionModelBullet*)model->GetImplementation();
    Remove(bt_model);
    model->RemoveImplementation();
}

void ChCollisionSystemBullet::Remove(ChCollisionModelBullet* bt_model) {
    if (bt_model->GetBulletObject()->getCollisionShape()) {
        bt_collision_world->removeCollisionObject(bt_model->GetBulletObject());
    }

    auto pos = std::find_if(bt_models.begin(), bt_models.end(),                                                    //
                            [bt_model](std::shared_ptr<ChCollisionModelBullet> x) { return x.get() == bt_model; }  //
    );
    if (pos != bt_models.end())
        bt_models.erase(pos);
    else
        std::cout << "ChCollisionSystemBullet::Remove - Cannot find specified model!" << std::endl;
}

void ChCollisionSystemBullet::Run() {
    if (bt_collision_world) {
        bt_collision_world->performDiscreteCollisionDetection();
    }
}

geometry::ChAABB ChCollisionSystemBullet::GetBoundingBox() const {
    cbtVector3 aabbMin;
    cbtVector3 aabbMax;
    bt_broadphase->getBroadphaseAabb(aabbMin, aabbMax);

    return geometry::ChAABB(ChVector<>((double)aabbMin.x(), (double)aabbMin.y(), (double)aabbMin.z()),
                            ChVector<>((double)aabbMax.x(), (double)aabbMax.y(), (double)aabbMax.z()));
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
        cbtPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        const cbtCollisionObject* obA = contactManifold->getBody0();
        const cbtCollisionObject* obB = contactManifold->getBody1();
        contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

        auto bt_modelA = (ChCollisionModelBullet*)obA->getUserPointer();
        auto bt_modelB = (ChCollisionModelBullet*)obB->getUserPointer();

        icontact.modelA = bt_modelA->model;
        icontact.modelB = bt_modelB->model;

        double envelopeA = icontact.modelA->GetEnvelope();
        double envelopeB = icontact.modelB->GetEnvelope();

        double marginA = icontact.modelA->GetSafeMargin();
        double marginB = icontact.modelB->GetSafeMargin();

        // Execute custom broadphase callback, if any
        bool do_narrow_contactgeneration = true;
        if (broad_callback)
            do_narrow_contactgeneration = broad_callback->OnBroadphase(icontact.modelA, icontact.modelB);

        if (do_narrow_contactgeneration) {
            int numContacts = contactManifold->getNumContacts();
            // GetLog() << "numContacts=" << numContacts << "\n";
            for (int j = 0; j < numContacts; j++) {
                cbtManifoldPoint& pt = contactManifold->getContactPoint(j);

                // Discard "too far" constraints (the Bullet engine also has its threshold)
                if (pt.getDistance() < marginA + marginB) {
                    cbtVector3 ptA = pt.getPositionWorldOnA();
                    cbtVector3 ptB = pt.getPositionWorldOnB();

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

                    icontact.shapeA = bt_modelA->m_shapes[indexA].get();
                    icontact.shapeB = bt_modelB->m_shapes[indexB].get();

                    // Execute some user custom callback, if any
                    bool add_contact = true;
                    if (this->narrow_callback)
                        add_contact = this->narrow_callback->OnNarrowphase(icontact);

                    // Add to contact container
                    if (add_contact) {
                        ////std::cout << " add indexA=" << indexA << " indexB=" << indexB << std::endl;
                        ////std::cout << "     typeA=" << icontact.shapeA->m_type << " typeB=" <<
                        /// icontact.shapeB->m_type << std::endl;

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
        cbtBroadphasePair mp =
            bt_collision_world->getBroadphase()->getOverlappingPairCache()->getOverlappingPairArray().at(i);

        cbtCollisionObject* obA = static_cast<cbtCollisionObject*>(mp.m_pProxy0->m_clientObject);
        cbtCollisionObject* obB = static_cast<cbtCollisionObject*>(mp.m_pProxy1->m_clientObject);

        auto bt_modelA = (ChCollisionModelBullet*)obA->getUserPointer();
        auto bt_modelB = (ChCollisionModelBullet*)obB->getUserPointer();

        ChCollisionModel* modelA = bt_modelA->model;
        ChCollisionModel* modelB = bt_modelB->model;

        // Add to proximity container
        mproximitycontainer->AddProximity(modelA, modelB);
    }
    mproximitycontainer->EndAddProximities();
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& result) const {
    return RayHit(from, to, result, cbtBroadphaseProxy::DefaultFilter, cbtBroadphaseProxy::AllFilter);
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChRayhitResult& result,
                                     short int filter_group,
                                     short int filter_mask) const {
    cbtVector3 btfrom((cbtScalar)from.x(), (cbtScalar)from.y(), (cbtScalar)from.z());
    cbtVector3 btto((cbtScalar)to.x(), (cbtScalar)to.y(), (cbtScalar)to.z());

    cbtCollisionWorld::ClosestRayResultCallback rayCallback(btfrom, btto);
    rayCallback.m_collisionFilterGroup = filter_group;
    rayCallback.m_collisionFilterMask = filter_mask;

    this->bt_collision_world->rayTest(btfrom, btto, rayCallback);

    if (rayCallback.hasHit()) {
        auto bt_model = static_cast<ChCollisionModelBullet*>(rayCallback.m_collisionObject->getUserPointer());
        result.hitModel = bt_model->model;
        if (result.hitModel) {
            result.hit = true;
            result.abs_hitPoint.Set(rayCallback.m_hitPointWorld.x(), rayCallback.m_hitPointWorld.y(),
                                    rayCallback.m_hitPointWorld.z());
            result.abs_hitNormal.Set(rayCallback.m_hitNormalWorld.x(), rayCallback.m_hitNormalWorld.y(),
                                     rayCallback.m_hitNormalWorld.z());
            result.abs_hitNormal.Normalize();
            result.dist_factor = rayCallback.m_closestHitFraction;
            result.abs_hitPoint = result.abs_hitPoint - result.abs_hitNormal * result.hitModel->GetEnvelope();
            return true;
        }
    }
    result.hit = false;
    return false;
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChCollisionModel* model,
                                     ChRayhitResult& result) const {
    return RayHit(from, to, model, result, cbtBroadphaseProxy::DefaultFilter, cbtBroadphaseProxy::AllFilter);
}

bool ChCollisionSystemBullet::RayHit(const ChVector<>& from,
                                     const ChVector<>& to,
                                     ChCollisionModel* model,
                                     ChRayhitResult& result,
                                     short int filter_group,
                                     short int filter_mask) const {
    cbtVector3 btfrom((cbtScalar)from.x(), (cbtScalar)from.y(), (cbtScalar)from.z());
    cbtVector3 btto((cbtScalar)to.x(), (cbtScalar)to.y(), (cbtScalar)to.z());

    cbtCollisionWorld::AllHitsRayResultCallback rayCallback(btfrom, btto);
    rayCallback.m_collisionFilterGroup = filter_group;
    rayCallback.m_collisionFilterMask = filter_mask;

    this->bt_collision_world->rayTest(btfrom, btto, rayCallback);

    // Find the closest hit result on the specified model (if any)
    int hit = -1;
    cbtScalar fraction = 1;
    for (int i = 0; i < rayCallback.m_collisionObjects.size(); ++i) {
        auto bt_model = static_cast<ChCollisionModelBullet*> (rayCallback.m_collisionObject->getUserPointer());
        if (bt_model->model == model && rayCallback.m_hitFractions[i] < fraction) {
            hit = i;
            fraction = rayCallback.m_hitFractions[i];
        }
    }

    // Ray does not hit specified model
    if (hit == -1) {
        result.hit = false;
        return false;
    }

    // Return the closest hit on the specified model
    auto bt_model = static_cast<ChCollisionModelBullet*>(rayCallback.m_collisionObjects[hit]->getUserPointer());
    result.hit = true;
    result.hitModel = bt_model->model;
    result.abs_hitPoint.Set(rayCallback.m_hitPointWorld[hit].x(), rayCallback.m_hitPointWorld[hit].y(),
                            rayCallback.m_hitPointWorld[hit].z());
    result.abs_hitNormal.Set(rayCallback.m_hitNormalWorld[hit].x(), rayCallback.m_hitNormalWorld[hit].y(),
                             rayCallback.m_hitNormalWorld[hit].z());
    result.abs_hitNormal.Normalize();
    result.dist_factor = fraction;
    result.abs_hitPoint = result.abs_hitPoint - result.abs_hitNormal * result.hitModel->GetEnvelope();
    return true;
}

void ChCollisionSystemBullet::SetContactBreakingThreshold(double threshold) {
    gContactBreakingThreshold = (cbtScalar)threshold;
}

class ChDebugDrawer : public cbtIDebugDraw {
  public:
    explicit ChDebugDrawer(ChCollisionSystem::VisualizationCallback* vis) : m_debugMode(0), m_vis(vis) {}

    ~ChDebugDrawer() override {}

    void drawLine(const cbtVector3& from, const cbtVector3& to, const cbtVector3& color) override {
        m_vis->DrawLine(ChVector<>(from.x(), from.y(), from.z()), ChVector<>(to.x(), to.y(), to.z()),
                        ChColor(color.x(), color.y(), color.z()));
    }

    void drawContactPoint(const cbtVector3& PointOnB,
                          const cbtVector3& normalOnB,
                          cbtScalar distance,
                          int lifeTime,
                          const cbtVector3& color) override {
        cbtVector3 from = PointOnB;
        cbtVector3 to = PointOnB + m_vis->GetNormalScale() * normalOnB;
        m_vis->DrawLine(ChVector<>(from.x(), from.y(), from.z()), ChVector<>(to.x(), to.y(), to.z()),
                        ChColor(color.x(), color.y(), color.z()));
    }

    void reportErrorWarning(const char* warningString) override {}
    void draw3dText(const cbtVector3& location, const char* textString) override {}

    void setDebugMode(int debugMode) override { m_debugMode |= debugMode; }

    int getDebugMode() const override { return m_debugMode; }

  private:
    int m_debugMode;
    ChCollisionSystem::VisualizationCallback* m_vis;
};

void ChCollisionSystemBullet::RegisterVisualizationCallback(std::shared_ptr<VisualizationCallback> callback) {
    ChCollisionSystem::RegisterVisualizationCallback(callback);

    m_debug_drawer = new ChDebugDrawer(vis_callback.get());
    m_debug_drawer->setDebugMode(cbtIDebugDraw::DBG_NoDebug);
    bt_collision_world->setDebugDrawer(m_debug_drawer);
}

void ChCollisionSystemBullet::Visualize(int flags) {
    if (!vis_callback || flags == VisualizationModes::VIS_None)
        return;

    if (flags & VisualizationModes::VIS_Shapes)
        m_debug_drawer->setDebugMode(cbtIDebugDraw::DBG_DrawWireframe);
    if (flags & VisualizationModes::VIS_Aabb)
        m_debug_drawer->setDebugMode(cbtIDebugDraw::DBG_DrawAabb);
    if (flags & VisualizationModes::VIS_Contacts)
        m_debug_drawer->setDebugMode(cbtIDebugDraw::DBG_DrawContactPoints);

    bt_collision_world->debugDrawWorld();
}

void ChCollisionSystemBullet::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCollisionSystemBullet>();
    // serialize parent class
    ChCollisionSystem::ArchiveOut(marchive);
}

void ChCollisionSystemBullet::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCollisionSystemBullet>();
    // deserialize parent class
    ChCollisionSystem::ArchiveIn(marchive);
}

}  // end namespace chrono
