//////////////////////////////////////////////////
//  
//   ChCCollisionSystemBullet.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
   
 
#include "collision/ChCCollisionSystemBullet.h"
#include "collision/ChCModelBullet.h"
#include "collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"
#include "physics/ChBody.h"
#include "physics/ChContactContainerBase.h"
#include "physics/ChProximityContainerBase.h"
#include "LinearMath/btPoolAllocator.h"

namespace chrono 
{
namespace collision 
{


/*
void defaultChronoNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, btDispatcherInfo& dispatchInfo)
{
	btCollisionDispatcher::defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
	if (broad_callback)
		broad_callback(collisionPair, dispatcher, dispatchInfo);
}
*/

ChCollisionSystemBullet::ChCollisionSystemBullet(unsigned int max_objects, double scene_size)
{
	// btDefaultCollisionConstructionInfo conf_info(...); ***TODO***
	bt_collision_configuration = new btDefaultCollisionConfiguration(); 
	bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);  
	
	  //***OLD***
	
	btScalar sscene_size = (btScalar)scene_size;
	 btVector3	worldAabbMin(-sscene_size,-sscene_size,-sscene_size);
	 btVector3	worldAabbMax(sscene_size,sscene_size,sscene_size);
	bt_broadphase = new bt32BitAxisSweep3(worldAabbMin,worldAabbMax, max_objects, 0, true); // true for disabling raycast accelerator
	
	  //***NEW***
	//bt_broadphase = new btDbvtBroadphase();



	bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

	// custom collision for sphere-sphere case ***OBSOLETE***
	//bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,new btSphereSphereCollisionAlgorithm::CreateFunc);

	// register custom collision for GIMPACT mesh case too
	btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);
}


ChCollisionSystemBullet::~ChCollisionSystemBullet()
{
	if(bt_collision_world) delete bt_collision_world;
	if(bt_broadphase) delete bt_broadphase;
	if(bt_dispatcher) delete bt_dispatcher; 
	if(bt_collision_configuration) delete bt_collision_configuration;
}

void ChCollisionSystemBullet::Clear(void)
{
	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		contactManifold->clearManifold();
	}
}   
				


void ChCollisionSystemBullet::Add(ChCollisionModel* model)
{
	if (((ChModelBullet*)model)->GetBulletModel()->getCollisionShape())
	{
		model->SyncPosition();
		bt_collision_world->addCollisionObject(((ChModelBullet*)model)->GetBulletModel(),
			((ChModelBullet*)model)->GetFamilyGroup(),
			((ChModelBullet*)model)->GetFamilyMask());
	}
}
		 		
void ChCollisionSystemBullet::Remove(ChCollisionModel* model)
{
	if (((ChModelBullet*)model)->GetBulletModel()->getCollisionShape())
	{
		bt_collision_world->removeCollisionObject(((ChModelBullet*)model)->GetBulletModel());
	}
}


void ChCollisionSystemBullet::Run()
{
	if (bt_collision_world)
	{
		bt_collision_world->performDiscreteCollisionDetection(); 
	}
}


void ChCollisionSystemBullet::ReportContacts(ChContactContainerBase* mcontactcontainer)
{
	// This should remove all old contacts (or at least rewind the index)
	mcontactcontainer->BeginAddContact();

	ChCollisionInfo icontact;

	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		contactManifold->refreshContactPoints(obA->getWorldTransform(),obB->getWorldTransform());
	 
		icontact.modelA = (ChCollisionModel*)obA->getUserPointer();
		icontact.modelB = (ChCollisionModel*)obB->getUserPointer();

		double envelopeA = icontact.modelA->GetEnvelope();
		double envelopeB = icontact.modelB->GetEnvelope();

		// Execute custom broadphase callback, if any
		bool do_narrow_contactgeneration = true;
		if (this->broad_callback)
			do_narrow_contactgeneration = this->broad_callback->BroadCallback(icontact.modelA, icontact.modelB);

		if (do_narrow_contactgeneration)
		{
			int numContacts = contactManifold->getNumContacts();

			for (int j=0;j<numContacts;j++)
			{
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
 
				btVector3 ptA = pt.getPositionWorldOnA();
				btVector3 ptB = pt.getPositionWorldOnB(); 
				
				icontact.vpA.Set(ptA.getX(), ptA.getY(), ptA.getZ());
				icontact.vpB.Set(ptB.getX(), ptB.getY(), ptB.getZ());
				
				icontact.vN.Set( -pt.m_normalWorldOnB.getX(), 
								 -pt.m_normalWorldOnB.getY(),
								 -pt.m_normalWorldOnB.getZ());
				icontact.vN.Normalize(); 

				double ptdist = pt.getDistance();

				icontact.vpA = icontact.vpA - icontact.vN*envelopeA;
				icontact.vpB = icontact.vpB + icontact.vN*envelopeB;
				icontact.distance = ptdist + envelopeA + envelopeB;	

				icontact.reaction_cache = pt.reactions_cache;

				// Execute some user custom callback, if any
				if (this->narrow_callback)
					this->narrow_callback->NarrowCallback(icontact);

				// Add to contact container
				mcontactcontainer->AddContact(icontact); 

			}
		}

		//you can un-comment out this line, and then all points are removed
		//contactManifold->clearManifold();	
	}
	mcontactcontainer->EndAddContact();
}


void ChCollisionSystemBullet::ReportProximities(ChProximityContainerBase* mproximitycontainer)
{
	mproximitycontainer->BeginAddProximities();

	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
		contactManifold->refreshContactPoints(obA->getWorldTransform(),obB->getWorldTransform());
	 
		ChCollisionModel* modelA = (ChCollisionModel*)obA->getUserPointer();
		ChCollisionModel* modelB = (ChCollisionModel*)obB->getUserPointer();

		// Add to proximity container
		mproximitycontainer->AddProximity(modelA, modelB);
	}
	mproximitycontainer->EndAddProximities();
}




bool ChCollisionSystemBullet::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult)
{
	btVector3 btfrom((btScalar)from.x, (btScalar)from.y, (btScalar)from.z);
	btVector3 btto  ((btScalar)to.x,   (btScalar)to.y,   (btScalar)to.z);

	btCollisionWorld::ClosestRayResultCallback rayCallback(btfrom,btto);

	this->bt_collision_world->rayTest(btfrom, btto, rayCallback);

	if (rayCallback.hasHit())
	{
		mresult.hitModel = (ChCollisionModel*)(rayCallback.m_collisionObject->getUserPointer());
		if (mresult.hitModel)
		{
			mresult.hit = true;
			mresult.abs_hitPoint.Set(rayCallback.m_hitPointWorld.x(),rayCallback.m_hitPointWorld.y(),rayCallback.m_hitPointWorld.z());
			mresult.abs_hitNormal.Set(rayCallback.m_hitNormalWorld.x(),rayCallback.m_hitNormalWorld.y(),rayCallback.m_hitNormalWorld.z());
			mresult.abs_hitNormal.Normalize();
			mresult.hit = true;
			mresult.dist_factor = rayCallback.m_closestHitFraction;
			return true;
		}
	}
	mresult.hit = false;
	return false;
}





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


