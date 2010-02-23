//////////////////////////////////////////////////
//  
//   ChCCollisionSystemGPU.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#ifndef CH_NOCUDA 

 
#include "ChCModelGPU.h"
#include "ChCCollisionSystem.h"
#include <cutil_inline.h>
//#include "collision/gimpact/GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"
#include "physics/ChBody.h"
#include "LinearMath/btPoolAllocator.h"
#include "ChCCollisionSystemGPU.h"
#include "physics/ChContactContainerBase.h"  //-ALEX

extern "C" void cudaCollisions(float *x,float *y,float *z,float *r, chrono::collision::ChCollisionSystemGPU::contact_Data *&contactdata,int &numcontacts, int numbodies, float cellSize, float limits[3][2]);

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

ChCollisionSystemGPU::ChCollisionSystemGPU(unsigned int max_objects, double scene_size)
{
	x_data=new float[max_objects];
	y_data=new float[max_objects];
	z_data=new float[max_objects];
	r_data=new float[max_objects];
	reactionCache = new float[6];
	reactionCache[0]=reactionCache[1]=reactionCache[2]=reactionCache[3]=reactionCache[4]=reactionCache[5]=0.f;
	maxSize = 0.f;
	num_Bodies = 0;
	contact_Data *contactdata = NULL;
	contactdata= (contact_Data*) malloc(sizeof(contact_Data)*2000000); //***ALEX why allocation was commented out?***
	numContacts = 0;
	colModels = new ChModelGPU*[max_objects];
	cutilSafeCall(cudaSetDevice(0));
	//OLD STUFF BELOW
	/*
	bt_collision_configuration = new btDefaultCollisionConfiguration(); // (0,0,0,  2); //max_pairs); ***TODO***
	bt_dispatcher = new btCollisionDispatcher(bt_collision_configuration);  
	btScalar sscene_size = (btScalar)scene_size;
	 btVector3	worldAabbMin(-sscene_size,-sscene_size,-sscene_size);
	 btVector3	worldAabbMax(sscene_size,sscene_size,sscene_size);
	bt_broadphase = new bt32BitAxisSweep3(worldAabbMin,worldAabbMax, max_objects);
	bt_collision_world = new btCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);

	// custom collision for sphere-sphere case
	bt_dispatcher->registerCollisionCreateFunc(SPHERE_SHAPE_PROXYTYPE,SPHERE_SHAPE_PROXYTYPE,new btSphereSphereCollisionAlgorithm::CreateFunc);
	
	// custom collision for GIMPACT mesh case
	btGImpactCollisionAlgorithm::registerAlgorithm(bt_dispatcher);
	*/
}


ChCollisionSystemGPU::~ChCollisionSystemGPU()
{
	delete x_data;
	delete y_data;
	delete z_data;
	delete r_data;
	delete reactionCache;
	for(int i=0; i<num_Bodies; i++)
	{
		delete colModels[i];
	}
	delete [] colModels;
	free(contactdata);
	//OLD STUFF BELOW
	/*
	if(bt_collision_world) delete bt_collision_world;
	if(bt_broadphase) delete bt_broadphase;
	if(bt_dispatcher) delete bt_dispatcher; 
	if(bt_collision_configuration) delete bt_collision_configuration;
	*/
}

void ChCollisionSystemGPU::Clear(void)
{
	//OLD STUFF BELOW
	/*
	int numManifolds = bt_collision_world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  bt_collision_world->getDispatcher()->getManifoldByIndexInternal(i);
		contactManifold->clearManifold();
	}
	*/
}   
				


void ChCollisionSystemGPU::Add(ChCollisionModel* model)
{
	colModels[num_Bodies]=(chrono::collision::ChModelGPU*)model;
	x_data[num_Bodies]=((chrono::collision::ChModelGPU*)model)->GetXPos();
	y_data[num_Bodies]=((chrono::collision::ChModelGPU*)model)->GetYPos();
	z_data[num_Bodies]=((chrono::collision::ChModelGPU*)model)->GetZPos();
	r_data[num_Bodies]=(((chrono::collision::ChModelGPU*)model)->GetRad())+(model->GetEnvelope());


	if(num_Bodies==0)
	{
		maxSize=(((chrono::collision::ChModelGPU*)model)->GetRad())+(model->GetEnvelope());
		limits[0][0]=((chrono::collision::ChModelGPU*)model)->GetXPos();
		limits[0][1]=((chrono::collision::ChModelGPU*)model)->GetXPos();
		limits[1][0]=((chrono::collision::ChModelGPU*)model)->GetYPos();
		limits[1][1]=((chrono::collision::ChModelGPU*)model)->GetYPos();
		limits[2][0]=((chrono::collision::ChModelGPU*)model)->GetZPos();
		limits[2][1]=((chrono::collision::ChModelGPU*)model)->GetZPos();
	}
	else
	{	
		if((((chrono::collision::ChModelGPU*)model)->GetRad())+(model->GetEnvelope())>maxSize)
			maxSize=(((chrono::collision::ChModelGPU*)model)->GetRad())+(model->GetEnvelope());
		
		if(((chrono::collision::ChModelGPU*)model)->GetXPos()<limits[0][0])
			limits[0][0]=((chrono::collision::ChModelGPU*)model)->GetXPos();
		else if(((chrono::collision::ChModelGPU*)model)->GetXPos()>limits[0][1])
			limits[0][1]=((chrono::collision::ChModelGPU*)model)->GetXPos();
		
		if(((chrono::collision::ChModelGPU*)model)->GetYPos()<limits[1][0])
			limits[1][0]=((chrono::collision::ChModelGPU*)model)->GetYPos();
		else if(((chrono::collision::ChModelGPU*)model)->GetYPos()>limits[1][1])
			limits[1][1]=((chrono::collision::ChModelGPU*)model)->GetYPos();
		
		if(((chrono::collision::ChModelGPU*)model)->GetZPos()<limits[2][0])
			limits[2][0]=((chrono::collision::ChModelGPU*)model)->GetZPos();
		else if(((chrono::collision::ChModelGPU*)model)->GetZPos()>limits[2][1])
			limits[2][1]=((chrono::collision::ChModelGPU*)model)->GetZPos();
	}
	num_Bodies++;

	//OLD STUFF BELOW
	/*
	if (((ChModelBullet*)model)->GetBulletModel()->getCollisionShape())
	{
		model->SyncPosition();
		bt_collision_world->addCollisionObject(((ChModelBullet*)model)->GetBulletModel());
	}
	*/
}
				
void ChCollisionSystemGPU::Remove(ChCollisionModel* model)
{
	//HOW DO I REMOVE???

	//OLD STUFF BELOW
	/*
	if (((ChModelBullet*)model)->GetBulletModel()->getCollisionShape())
	{
		bt_collision_world->removeCollisionObject(((ChModelBullet*)model)->GetBulletModel());
	}
	*/
}
				 

void ChCollisionSystemGPU::Run()
{
	updateDataStructures();
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Perform the Collision Detection%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//printf("%f \n", *x_data);
	//cudaThreadExit();
	//cutilSafeCall(cudaSetDevice(0));
	
	cudaCollisions(x_data,y_data,z_data,r_data,contactdata,numContacts, num_Bodies,10*.015, limits);
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Post Process the Collisions which were found%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
}

void ChCollisionSystemGPU::ReportContacts(ChContactContainerBase* mcontactcontainer)
{
	// This should remove all old contacts (or at least rewind the index) -ALEX
	this->contact_container->BeginAddContact();

	// Use this structure to communicate with contact container -ALEX
	ChCollisionInfo icontact;

	//cudaThreadExit();
//	cutilSafeCall(cudaSetDevice(1));
	double ptdist;
	double envelopeA;
	double envelopeB;
	chrono::ChVector<> vpA;
	chrono::ChVector<> vpB;
	chrono::ChVector<> vpN;
	ChBody* mbodyA;
	ChBody* mbodyB;
	for(int i=0; i<numContacts; i++)
	{
		mbodyA=colModels[contactdata[i].objectIdA]->GetBody();
		mbodyB=colModels[contactdata[i].objectIdB]->GetBody();

		envelopeA = mbodyA->GetCollisionModel()->GetEnvelope();
		envelopeB = mbodyB->GetCollisionModel()->GetEnvelope();

		if (!(mbodyA->GetBodyFixed() && mbodyB->GetBodyFixed()))
		{
			vpA.x= contactdata[i].positionWorldOnA[0];
			vpA.y= contactdata[i].positionWorldOnA[1];
			vpA.z= contactdata[i].positionWorldOnA[2];
			vpB.x= contactdata[i].positionWorldOnB[0];
			vpB.y= contactdata[i].positionWorldOnB[1];
			vpB.z= contactdata[i].positionWorldOnB[2];
			vpN.x= -contactdata[i].normalWorldOnB[0];
			vpN.y= -contactdata[i].normalWorldOnB[1];
			vpN.z= -contactdata[i].normalWorldOnB[2];
			
			vpN.Normalize(); 
			ptdist = contactdata[i].distance;

			vpA = vpA - vpN*envelopeA;
			vpB = vpB + vpN*envelopeB;
			ptdist = ptdist + envelopeA + envelopeB;	

			icontact.modelA = mbodyA->GetCollisionModel();
			icontact.modelB = mbodyB->GetCollisionModel();
			icontact.vpA = vpA;
			icontact.vpB = vpB;
			icontact.vN  = vpN;
			icontact.distance = ptdist;
			icontact.reaction_cache = reactionCache;

			// Execute some user custom callback, if any  -ALEX
			if (this->narrow_callback)
				this->narrow_callback->NarrowCallback(icontact);

			// Add to contact container   -ALEX
			mcontactcontainer->AddContact(icontact);

			// NOTE:            -ALEX
			// Check if this->contact_container is some specialized type such as the new
			// ChContactContainerGPU, maybe he also offers some advanced more efficient
			// AddContact(..) function to add contacts to the the container, for instance like a
			// batch AddContactBuffer(..) function that copies directly from GPU coll.detect. buffer
			// into some dedicated GPU contact buffer in the container, without needing
			// to pass through the cpu. [This should be done in future.]
			// Anyway, all containers should implement the basic AddContact(icontact) as fall-back.
		}
	}
	//free(contactdata); //???***ALEX*** Toby, do not deallocate here - it may be reused with a repeated call to this function-

	// This is needed after all additions, because some contact
	// container might implement an optimization of the contact pool memory.  -ALEX
	this->contact_container->EndAddContact(); 
}



bool ChCollisionSystemGPU::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult)
{
	//OLD STUFF BELOW
	/*
	btVector3 btfrom((btScalar)from.x, (btScalar)from.y, (btScalar)from.z);
	btVector3 btto  ((btScalar)to.x,   (btScalar)to.y,   (btScalar)to.z);

	btCollisionWorld::ClosestRayResultCallback rayCallback(btfrom,btto);

	this->bt_collision_world->rayTest(btfrom, btto, rayCallback);

	if (rayCallback.HasHit())
	{
		mresult.hitBody = (ChBody*)(rayCallback.m_collisionObject->getUserPointer());
		if (mresult.hitBody)
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
	*/
	return false;
}

void ChCollisionSystemGPU::updateDataStructures()
{
	for( int i=0; i<num_Bodies; i++)
	{
		chrono::collision::ChModelGPU* body = colModels[i];
		x_data[i]=body->GetXPos();
		y_data[i]=body->GetYPos();
		z_data[i]=body->GetZPos();
		r_data[i]=(body->GetRad())+(body->GetEnvelope());

		if(((body->GetRad())+(body->GetEnvelope()))>maxSize)
			maxSize=(body->GetRad())+(body->GetEnvelope());
		
		if(body->GetXPos()<limits[0][0])
			limits[0][0]=body->GetXPos();
		else if(body->GetXPos()>limits[0][1])
			limits[0][1]=body->GetXPos();
		
		if(body->GetYPos()<limits[1][0])
			limits[1][0]=body->GetYPos();
		else if(body->GetYPos()>limits[1][1])
			limits[1][1]=body->GetYPos();
		
		if(body->GetZPos()<limits[2][0])
			limits[2][0]=body->GetZPos();
		else if(body->GetZPos()>limits[2][1])
			limits[2][1]=body->GetZPos();
	}
}





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif  // end of ! CH_NOCUDA
