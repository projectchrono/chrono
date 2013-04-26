//////////////////////////////////////////////////
//
//   ChCCollisionSystemBullet.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCCollisionSystemBulletGPU.h"

#include "collision/ChCModelBullet.h"
#include "physics/ChBody.h"
#include "../ChCudaDefines.h"
#include "../ChLcpSystemDescriptorGPU.h"
#include "../../collision/ChCCollisionSystem.h"
#include "../../physics/ChProximityContainerBase.h"

namespace chrono {
namespace collision {

/*
 void defaultChronoNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, btDispatcherInfo& dispatchInfo)
 {
 btCollisionDispatcher::defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
 if (broad_callback)
 broad_callback(collisionPair, dispatcher, dispatchInfo);
 }
 */

ChCollisionSystemBulletGPU::ChCollisionSystemBulletGPU(unsigned int max_objects, double scene_size) {
	// btDefaultCollisionConstructionInfo conf_info(...); ***TODO***
	bt_collision_configuration = new btDefaultCollisionConfiguration();
	bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);

	//***OLD***

	btScalar sscene_size = (btScalar) scene_size;
	btVector3 worldAabbMin(-sscene_size, -sscene_size, -sscene_size);
	btVector3 worldAabbMax(sscene_size, sscene_size, sscene_size);
	bt_broadphase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, true); // true for disabling raycast accelerator

	//***NEW***
	//bt_broadphase = new btDbvtBroadphase();

	bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

	// custom collision for sphere-sphere case ***OBSOLETE***
	//bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,new btSphereSphereCollisionAlgorithm::CreateFunc);

	// register custom collision for GIMPACT mesh case too
	btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);

	counter = 0;

}

ChCollisionSystemBulletGPU::~ChCollisionSystemBulletGPU() {
	if (bt_collision_world)
		delete bt_collision_world;
	if (bt_broadphase)
		delete bt_broadphase;
	if (bt_dispatcher)
		delete bt_dispatcher;
	if (bt_collision_configuration)
		delete bt_collision_configuration;
}

void ChCollisionSystemBulletGPU::Clear(void) {
	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		contactManifold->clearManifold();
	}
}

void ChCollisionSystemBulletGPU::Add(ChCollisionModel* model) {
	if (((ChModelBullet*) model)->GetBulletModel()->getCollisionShape()) {
		model->SyncPosition();
		((ChModelBullet*) model)->GetBulletModel()->setCompanionId(counter);
		bt_collision_world->addCollisionObject(((ChModelBullet*) model)->GetBulletModel(), ((ChModelBullet*) model)->GetFamilyGroup(), ((ChModelBullet*) model)->GetFamilyMask());
		counter++;
		data_container->number_of_models++;
	}
}

void ChCollisionSystemBulletGPU::Remove(ChCollisionModel* model) {
	if (((ChModelBullet*) model)->GetBulletModel()->getCollisionShape()) {
		bt_collision_world->removeCollisionObject(((ChModelBullet*) model)->GetBulletModel());
	}
}

void ChCollisionSystemBulletGPU::Run() {
	if (bt_collision_world) {
		bt_collision_world->performDiscreteCollisionDetection();
	}

}
void ChCollisionSystemBulletGPU::ReportContacts(ChContactContainerBase* mcontactcontainer) {

	data_container->host_norm_data.clear();
	data_container->host_cpta_data.clear();
	data_container->host_cptb_data.clear();
	data_container->host_dpth_data.clear();
	data_container->host_bids_data.clear();
	data_container->number_of_contacts = 0;
	mcontactcontainer->BeginAddContact();
	ChCollisionInfo icontact;

	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold = bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

		icontact.modelA = (ChCollisionModel*) obA->getUserPointer();
		icontact.modelB = (ChCollisionModel*) obB->getUserPointer();

		double envelopeA = icontact.modelA->GetEnvelope();
		double envelopeB = icontact.modelB->GetEnvelope();

		double marginA = icontact.modelA->GetSafeMargin();
		double marginB = icontact.modelB->GetSafeMargin();

		bool activeA = ((ChBody*) (icontact.modelA->GetPhysicsItem()))->IsActive();
		bool activeB = ((ChBody*) (icontact.modelB->GetPhysicsItem()))->IsActive();

		if (activeA == 0 && activeB == 0) {
			continue;
		}

		// Execute custom broadphase callback, if any
		bool do_narrow_contactgeneration = true;
		if (this->broad_callback)
			do_narrow_contactgeneration = this->broad_callback->BroadCallback(icontact.modelA, icontact.modelB);

		if (do_narrow_contactgeneration) {
			int numContacts = contactManifold->getNumContacts();

			for (int j = 0; j < numContacts; j++) {
				btManifoldPoint& pt = contactManifold->getContactPoint(j);

				if (pt.getDistance() < marginA + marginB) // to discard "too far" constraints (the Bullet engine also has its threshold)
						{

					btVector3 ptA = pt.getPositionWorldOnA();
					btVector3 ptB = pt.getPositionWorldOnB();

					icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
					icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());

					icontact.vN.Set(-pt.m_normalWorldOnB.getX(), -pt.m_normalWorldOnB.getY(), -pt.m_normalWorldOnB.getZ());
					icontact.vN.Normalize();

					double ptdist = pt.getDistance();

					icontact.vpA = icontact.vpA - icontact.vN * envelopeA;
					icontact.vpB = icontact.vpB + icontact.vN * envelopeB;
					icontact.distance = ptdist + envelopeA + envelopeB;

					icontact.reaction_cache = pt.reactions_cache;

					// Execute some user custom callback, if any
					if (this->narrow_callback)
						this->narrow_callback->NarrowCallback(icontact);

					// Add to contact container
					mcontactcontainer->AddContact(icontact);

					data_container->host_norm_data.push_back(R3(icontact.vN.x, icontact.vN.y, icontact.vN.z));
					data_container->host_cpta_data.push_back(R3(icontact.vpA.x, icontact.vpA.y, icontact.vpA.z));
					data_container->host_cptb_data.push_back(R3(icontact.vpB.x, icontact.vpB.y, icontact.vpB.z));
					data_container->host_dpth_data.push_back(icontact.distance);
					data_container->host_bids_data.push_back(I2(obA->getCompanionId(), obB->getCompanionId()));
					data_container->number_of_contacts++;


				}

			}
		}
		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();
	}

	data_container->HostToDeviceContacts();
	mcontactcontainer->EndAddContact();

}
} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

