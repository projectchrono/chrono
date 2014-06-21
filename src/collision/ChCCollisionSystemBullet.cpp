//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//  
//   ChCCollisionSystemBullet.cpp
//
// ------------------------------------------------
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
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"


extern btScalar gContactBreakingThreshold;


namespace chrono 
{
namespace collision 
{



// Utility class that we use to override the default cylinder-sphere collision
// case, because the default behavior in Bullet was using the GJK algorithm, that 
// gives not 100% precise results if the cylinder is much larger than the sphere:
class btSphereCylinderCollisionAlgorithm : public btActivatingCollisionAlgorithm
{
	bool	m_ownManifold;
	btPersistentManifold*	m_manifoldPtr;
	bool	m_isSwapped;
	
public:
	btSphereCylinderCollisionAlgorithm(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* col0,btCollisionObject* col1, bool isSwapped)
		: btActivatingCollisionAlgorithm(ci,col0,col1),
			m_ownManifold(false),
			m_manifoldPtr(mf),
			m_isSwapped(isSwapped)
			{
				if (!m_manifoldPtr)
				{
					m_manifoldPtr = m_dispatcher->getNewManifold(col0,col1);
					m_ownManifold = true;
				}
			}

	btSphereCylinderCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
		: btActivatingCollisionAlgorithm(ci) {}

	virtual void processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
	{
		(void)dispatchInfo;

		if (!m_manifoldPtr)
			return;

		btCollisionObject* sphereObj = m_isSwapped? body1 : body0;
		btCollisionObject* cylObj = m_isSwapped? body0 : body1;

		resultOut->setPersistentManifold(m_manifoldPtr);

		btSphereShape* sphere0 = (btSphereShape*)sphereObj->getCollisionShape();
		btCylinderShape* cylinder = (btCylinderShape*)cylObj->getCollisionShape();

		const btTransform&	m44T = cylObj->getWorldTransform();
		btVector3 diff = m44T.invXform(sphereObj->getWorldTransform().getOrigin()); //col0->getWorldTransform().getOrigin()-  col1->getWorldTransform().getOrigin();
		btScalar radius0 = sphere0->getRadius();
		btScalar radius1 = cylinder->getHalfExtentsWithMargin().getX(); //cylinder->getRadius();
		btScalar H1 = cylinder->getHalfExtentsWithMargin().getY();

		btVector3 r1 = diff; 
		r1.setY(0);

		btScalar y1 = diff.y();

		btScalar r1_len = r1.length();

		btVector3 pos1;
		btVector3 normalOnSurfaceB(1,0,0);
		btScalar dist;

		// Case A
		if ((y1 <= H1)&&(y1 >= -H1))
		{

			///iff distance positive, don't generate a new contact
			if ( r1_len > (radius0+radius1))
			{
				resultOut->refreshContactPoints();
				return;
			}
			///distance (negative means penetration)
			dist = r1_len - (radius0+radius1);

			btVector3 localnormalOnSurfaceB;
			if (r1_len > SIMD_EPSILON)
			{
				localnormalOnSurfaceB = r1 / r1_len;
				normalOnSurfaceB =  m44T.getBasis() * localnormalOnSurfaceB;
			}
			///point on B (worldspace)
			pos1 = m44T(btVector3(0, y1,0)) + radius1* normalOnSurfaceB;
		}
		else
		{
			btScalar side = 1;
			if (y1 < -H1)
				side = -1;

			if (r1_len > radius1)
			{
				// case B
				btVector3 pos_loc = r1.normalized() * radius1 + btVector3(0, H1*side, 0);
				pos1 = m44T(pos_loc);
				btVector3 d = sphereObj->getWorldTransform().getOrigin() - pos1;
				normalOnSurfaceB = d.normalized();
				dist = d.length() - radius0;
			}
			else
			{
				// case C
				normalOnSurfaceB = m44T.getBasis() * btVector3(0, 1*side, 0);
				btVector3 pos_loc = r1 + btVector3(0, H1*side, 0);
				pos1 = m44T(pos_loc);
				dist = side*(y1 - H1) - radius0;
			}
		}
		/// report a contact. internally this will be kept persistent, and contact reduction is done
		resultOut->addContactPoint(normalOnSurfaceB,pos1,dist);

		resultOut->refreshContactPoints();
	}

	virtual btScalar calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
	{
		//not yet
		return btScalar(1.);
	}

	virtual	void	getAllContactManifolds(btManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}
	
	virtual ~btSphereCylinderCollisionAlgorithm()
		{
			if (m_ownManifold)
			{
				if (m_manifoldPtr)
					m_dispatcher->releaseManifold(m_manifoldPtr);
			}
		}

	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btSphereCylinderCollisionAlgorithm));
			if (!m_swapped)
			{
				return new(mem) btSphereCylinderCollisionAlgorithm(0,ci,body0,body1,false);
			} else
			{
				return new(mem) btSphereCylinderCollisionAlgorithm(0,ci,body0,body1,true);
			}
		}
	};

};



////////////////////////////////////
////////////////////////////////////



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

	// custom collision for sphere-sphere case ***OBSOLETE*** // already registered by btDefaultCollisionConfiguration
	//bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,new btSphereSphereCollisionAlgorithm::CreateFunc); 
	
	// custom collision for cylinder-sphere case, for improved precision
	bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE, CYLINDER_SHAPE_PROXYTYPE,new btSphereCylinderCollisionAlgorithm::CreateFunc);
	bt_dispatcher->registerCollisionCreateFunc(CYLINDER_SHAPE_PROXYTYPE, SPHERE_SHAPE_PROXYTYPE,new btSphereCylinderCollisionAlgorithm::CreateFunc);

	// custom collision for GIMPACT mesh case too
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
		
		double marginA = icontact.modelA->GetSafeMargin();
		double marginB = icontact.modelB->GetSafeMargin();

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

				if (pt.getDistance() < marginA+marginB) // to discard "too far" constraints (the Bullet engine also has its threshold)
				{
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



void ChCollisionSystemBullet::SetContactBreakingThreshold(double threshold)
{
	gContactBreakingThreshold = (btScalar)threshold;
}


} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


