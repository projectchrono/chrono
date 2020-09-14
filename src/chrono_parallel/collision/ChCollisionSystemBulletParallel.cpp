// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Hammad Mazhar
// =============================================================================
//
// Based on the regular bullet collision system, some modifications made to
// store contacts in the parallel data structures
//
// =============================================================================

#include "chrono/collision/ChCollisionModelBullet.h"

#include "chrono_parallel/collision/ChCollisionSystemBulletParallel.h"
#include "chrono_parallel/ChDataManager.h"

#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChContactContainerParallel.h"
#include "chrono_parallel/collision/ChCollisionModelParallel.h"
#include "chrono_parallel/collision/ChDataStructures.h"

#include "chrono/collision/bullet/BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"

namespace chrono {
namespace collision {

/*
 void defaultChronoNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, btDispatcherInfo&
 dispatchInfo)
 {
 btCollisionDispatcher::defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
 if (broad_callback)
 broad_callback(collisionPair, dispatcher, dispatchInfo);
 }
*/

ChCollisionSystemBulletParallel::ChCollisionSystemBulletParallel(ChParallelDataManager* dc)
    : data_manager(dc) {
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

    // register custom collision for GIMPACT mesh case too
    btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);

    counter = 0;
}

ChCollisionSystemBulletParallel::~ChCollisionSystemBulletParallel() {
    if (bt_collision_world)
        delete bt_collision_world;
    if (bt_broadphase)
        delete bt_broadphase;
    if (bt_dispatcher)
        delete bt_dispatcher;
    if (bt_collision_configuration)
        delete bt_collision_configuration;
}

void ChCollisionSystemBulletParallel::SetNumThreads(int nthreads) {
#ifdef BT_USE_OPENMP
    btGetOpenMPTaskScheduler()->setNumThreads(nthreads);
#endif
}

void ChCollisionSystemBulletParallel::Clear(void) {
    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        contactManifold->clearManifold();
    }
}

void ChCollisionSystemBulletParallel::Add(ChCollisionModel* model) {
    ChCollisionModelBullet* bmodel = static_cast<ChCollisionModelBullet*>(model);
    if (bmodel->GetBulletModel()->getCollisionShape()) {
        bmodel->SyncPosition();
        bmodel->GetBulletModel()->setCompanionId(counter);
        bt_collision_world->addCollisionObject(bmodel->GetBulletModel(), bmodel->GetFamilyGroup(),
                                               bmodel->GetFamilyMask());
        counter++;
        data_manager->num_rigid_shapes++;
    }
}

void ChCollisionSystemBulletParallel::Remove(ChCollisionModel* model) {
    ChCollisionModelBullet* bmodel = static_cast<ChCollisionModelBullet*>(model);
    if (bmodel->GetBulletModel()->getCollisionShape()) {
        bt_collision_world->removeCollisionObject(bmodel->GetBulletModel());
    }
}

void ChCollisionSystemBulletParallel::Run() {
    if (bt_collision_world) {
        bt_collision_world->performDiscreteCollisionDetection();
    }
}

void ChCollisionSystemBulletParallel::GetBoundingBox(ChVector<>& aabb_min, ChVector<>& aabb_max) const {
    btVector3 aabbMin;
    btVector3 aabbMax;
    bt_broadphase->getBroadphaseAabb(aabbMin, aabbMax);
    aabb_min = ChVector<>((double)aabbMin.x(), (double)aabbMin.y(), (double)aabbMin.z());
    aabb_max = ChVector<>((double)aabbMax.x(), (double)aabbMax.y(), (double)aabbMax.z());
}

void ChCollisionSystemBulletParallel::ResetTimers() {
    bt_collision_world->timer_collision_broad.reset();
    bt_collision_world->timer_collision_narrow.reset();
}

double ChCollisionSystemBulletParallel::GetTimerCollisionBroad() const {
    return bt_collision_world->timer_collision_broad();
}

double ChCollisionSystemBulletParallel::GetTimerCollisionNarrow() const {
    return bt_collision_world->timer_collision_narrow();
}

void ChCollisionSystemBulletParallel::ReportContacts(ChContactContainer* mcontactcontainer) {
    data_manager->system_timer.start("collision_narrow");
    data_manager->host_data.norm_rigid_rigid.clear();
    data_manager->host_data.cpta_rigid_rigid.clear();
    data_manager->host_data.cptb_rigid_rigid.clear();
    data_manager->host_data.dpth_rigid_rigid.clear();
    data_manager->host_data.erad_rigid_rigid.clear();
    data_manager->host_data.bids_rigid_rigid.clear();
    data_manager->num_rigid_contacts = 0;
    // mcontactcontainer->BeginAddContact();

    // NOTE: Bullet does not provide information on radius of curvature at a contact point.
    // As such, for all Bullet-identified contacts, the default value will be used (SMC only). 
    ChCollisionInfo icontact;

    int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = contactManifold->getBody0();
        const btCollisionObject* obB = contactManifold->getBody1();
        if (obB->getCompanionId() < obA->getCompanionId()) {
            auto tmp = obA;
            obA = obB;
            obB = tmp;
        }
        contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

        icontact.modelA = (ChCollisionModel*)obA->getUserPointer();
        icontact.modelB = (ChCollisionModel*)obB->getUserPointer();

        double envelopeA = icontact.modelA->GetEnvelope();
        double envelopeB = icontact.modelB->GetEnvelope();

        double marginA = icontact.modelA->GetSafeMargin();
        double marginB = icontact.modelB->GetSafeMargin();

        bool activeA = ((ChBody*)(icontact.modelA->GetPhysicsItem()))->IsActive();
        bool activeB = ((ChBody*)(icontact.modelB->GetPhysicsItem()))->IsActive();

        if (activeA == 0 && activeB == 0) {
            continue;
        }

        // Execute custom broadphase callback, if any
        bool do_narrow_contactgeneration = true;
        if (this->broad_callback)
            do_narrow_contactgeneration = this->broad_callback->OnBroadphase(icontact.modelA, icontact.modelB);

        if (do_narrow_contactgeneration) {
            int numContacts = contactManifold->getNumContacts();

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

                    if (add_contact) {
                        ////std::cout << " add indexA=" << indexA << " indexB=" << indexB << std::endl;
                        ////std::cout << "     typeA=" << icontact.shapeA->m_type << " typeB=" << icontact.shapeB->m_type
                        ////          << std::endl;

                        data_manager->host_data.norm_rigid_rigid.push_back(
                            real3(icontact.vN.x(), icontact.vN.y(), icontact.vN.z()));
                        data_manager->host_data.cpta_rigid_rigid.push_back(
                            real3(icontact.vpA.x(), icontact.vpA.y(), icontact.vpA.z()));
                        data_manager->host_data.cptb_rigid_rigid.push_back(
                            real3(icontact.vpB.x(), icontact.vpB.y(), icontact.vpB.z()));
                        data_manager->host_data.dpth_rigid_rigid.push_back(icontact.distance);
                        data_manager->host_data.erad_rigid_rigid.push_back(icontact.eff_radius);
                        data_manager->host_data.bids_rigid_rigid.push_back(
                            I2(obA->getCompanionId(), obB->getCompanionId()));
                        data_manager->num_rigid_contacts++;
                    }
                }
            }
        }
        // you can un-comment out this line, and then all points are removed
        // contactManifold->clearManifold();
    }

    // mcontactcontainer->EndAddContact();
    data_manager->system_timer.stop("collision_narrow");
}

}  // end namespace collision
}  // end namespace chrono
