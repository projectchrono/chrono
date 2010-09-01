//////////////////////////////////////////////////
//  
//   ChCCollisionSystemGPU.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#ifdef CH_UNIT_CUDA 


#include "ChCModelGPUBody.h"
#include "ChCCollisionSystem.h"
#include <cutil_inline.h>
#include "physics/ChBody.h"
#include "LinearMath/btPoolAllocator.h"
#include "ChCCollisionSystemGPU.h"
#include "physics/ChContactContainerBase.h" 
#include "physics/ChProximityContainerBase.h" 
#include <time.h>
#include <iostream>
using namespace std;

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
			unsigned int max_spheres = 1000100;
			mGPUColl.mData.resize(max_spheres);
			
			reactionCache = new float[6];
			reactionCache[0]=reactionCache[1]=reactionCache[2]=reactionCache[3]=reactionCache[4]=reactionCache[5]=0.f;
			maxSize = 0.f;
			num_Bodies = 0;
			num_Spheres = 0;
			contact_Data *contactdata = NULL;
			//contactdata= (contact_Data*) malloc(sizeof(contact_Data)*2000000);
			numContacts = 0;
			colModels = new ChModelGPUBody*[max_objects];
			mGPUColl.bodyID.resize(max_spheres);
			mGPUColl.noCollWith.resize(max_spheres);
			mGPUColl.colFam.resize(max_spheres);

			
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
			mGPUColl.mData.clear();
			delete reactionCache;
			mGPUColl.bodyID.clear();
			mGPUColl.noCollWith.clear();
			mGPUColl.colFam.clear();
			for(int i=0; i<num_Bodies; i++)
			{
				delete colModels[i];
			}
			delete [] colModels;
			contactdata.clear();
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
			//***NOT SUPPORTED*** ***TO DO***

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
			colModels[num_Bodies]=(chrono::collision::ChModelGPUBody*)model;
			int spheresToAdd = ((chrono::collision::ChModelGPUBody*)model)->GetNSpheres();
			num_Spheres+=spheresToAdd;
			num_Bodies++;
		}

		void ChCollisionSystemGPU::Remove(ChCollisionModel* model)
		{
			//***NOT SUPPORTED*** ***TO DO***
		}


		void ChCollisionSystemGPU::Run()
		{
			updateDataStructures();
			mGPUColl.mParam.mNumBodies=num_Spheres;
			mGPUColl.mBinSize_=maxSize*4;
			
			//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Perform the Collision Detection%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			//printf("maxR: %f\nx: %f, %f\ny: %f, %f\nz: %f, %f\n", maxSize, mGPUColl.limits[0][0], mGPUColl.limits[0][1], mGPUColl.limits[1][0], mGPUColl.limits[1][1], mGPUColl.limits[2][0], mGPUColl.limits[2][1]);
			//printf("Start GPU CD\n");
			mGPUColl.cudaCollisions(contactdata,numContacts);//2*maxSize OR 0.1
			//printf("End GPU CD\nnumContacts: %i\n", numContacts);
			//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Post Process the Collisions which were found%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			//int times= clock();
			
			//cudaThreadExit(); ???
			//cutilSafeCall(cudaSetDevice(1));  ???
		}


		void ChCollisionSystemGPU::ReportContacts(ChContactContainerBase* mcontactcontainer)
		{
			// This should remove all old contacts (or at least rewind the index) -ALEX
			mcontactcontainer->BeginAddContact();

			// Use this structure to communicate with contact container -ALEX
			ChCollisionInfo icontact;

			double ptdist;
			double envelopeA;
			double envelopeB;
			chrono::ChVector<> vpA;
			chrono::ChVector<> vpB;
			chrono::ChVector<> vpN;
			ChBody* mbodyA;
			ChBody* mbodyB;
			ChModelGPUBody* colModelA;
			ChModelGPUBody* colModelB;
			//bool AcollWithB;
			//bool BcollWithA;
			for(int i=0; i<numContacts; i++)
			{
				//printf("ncontacts: %i\nIDA: %i\nIDB: %i\n", numContacts, contactdata[i].objectIdA, contactdata[i].objectIdB);
				colModelA = colModels[contactdata[i].objectIdA];
				colModelB = colModels[contactdata[i].objectIdB];

				mbodyA=colModels[contactdata[i].objectIdA]->GetBody();
				mbodyB=colModels[contactdata[i].objectIdB]->GetBody();

				//AcollWithB = colModelA->GetFamilyMaskDoesCollisionWithFamily(colModelB->GetFamily());
				//BcollWithA = colModelB->GetFamilyMaskDoesCollisionWithFamily(colModelA->GetFamily());

				//if(AcollWithB && BcollWithA)
				//{
				if (!(mbodyA->GetBodyFixed() && mbodyB->GetBodyFixed()))
				{
					envelopeA = colModelA->GetEnvelope();
					envelopeB = colModelB->GetEnvelope();

					vpA.x= contactdata[i].positionWorldOnA[0];
					vpA.y= contactdata[i].positionWorldOnA[1];
					vpA.z= contactdata[i].positionWorldOnA[2];

					vpB.x= contactdata[i].positionWorldOnB[0];
					vpB.y= contactdata[i].positionWorldOnB[1];
					vpB.z= contactdata[i].positionWorldOnB[2];

					vpN.x= -contactdata[i].normalWorldOnB[0];
					vpN.y= -contactdata[i].normalWorldOnB[1];
					vpN.z= -contactdata[i].normalWorldOnB[2];

					//vpN.Normalize(); 
					ptdist = contactdata[i].distance;

					vpA = vpA - vpN*envelopeA;
					vpB = vpB + vpN*envelopeB;
					ptdist = ptdist + envelopeA + envelopeB;	

					icontact.modelA = mbodyA->GetCollisionModel();
					icontact.modelB = mbodyB->GetCollisionModel();
					icontact.vpA = vpA;
					icontact.vpB = vpB;
					icontact.vN  = vpN;
					icontact.distance =ptdist;
					icontact.reaction_cache = reactionCache;

					// Execute some user custom callback, if any  -ALEX
					if (this->narrow_callback)
						this->narrow_callback->NarrowCallback(icontact);

					// Add to contact container   -ALEX
					mcontactcontainer->AddContact(icontact);

					// NOTE:            -ALEX
					// Check if mcontactcontainer is some specialized type such as a future
					// ChContactContainerGPU, maybe it also offers some advanced more efficient
					// AddContact(..) function to add contacts to the the container, for instance like a
					// batch AddContactBuffer(..) function that copies directly from GPU coll.detect. buffer
					// into some dedicated GPU contact buffer in the container, without needing
					// to pass through the cpu. [This should be done in future.]
					// Anyway, all containers should implement the basic AddContact(icontact) as fall-back.
				}
				
			}
			contactdata.clear();

			// This is needed after all additions, because some contact
			// container might implement an optimization of the contact pool memory.  -ALEX
			mcontactcontainer->EndAddContact(); 
		}


		void ChCollisionSystemGPU::ReportProximities(ChProximityContainerBase* mproximitycontainer)
		{
			mproximitycontainer->BeginAddProximities();
			
			//***NOT SUPPORTED*** ***TO DO*** 
			// Should loop over all broadphase pairs and report them:
			//   ... mproximitycontainer->AddProximity(modelA, modelB);

			mproximitycontainer->EndAddProximities();
		}



		bool ChCollisionSystemGPU::RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult)
		{
			//***NOT SUPPORTED*** ***TO DO***
			return false;
		}


		void ChCollisionSystemGPU::updateDataStructures()
		{
			int index = 0;
			float xm, xM, ym, yM, zm, zM;
			ChCoordsys<> bodyCoord;
			ChVector<> localPos;
			ChVector<> globalPos;
			float size;
			chrono::collision::ChModelGPUBody* body;
			int spheresToAdd;
			int fam;
			int noC;

			for( int i=0; i<num_Bodies; i++)
			{
				body = colModels[i];
				//update all spheres in this body
				spheresToAdd = body->GetNSpheres();
				bodyCoord = body->GetBody()->GetCoord();
				fam = body->GetFamily();
				noC = body->GetNoCollFamily();

				for(int j=0; j<spheresToAdd; j++)
				{
					//update the collision masks
					mGPUColl.colFam[index] = fam;
					mGPUColl.noCollWith[index] = noC;

					localPos = body->GetSpherePos(j);
					globalPos = bodyCoord.TrasformLocalToParent(localPos);
					size = (body->GetSphereR(j))+(body->GetEnvelope());
					mGPUColl.mData[index].x = globalPos.x;
					mGPUColl.mData[index].y = globalPos.y;
					mGPUColl.mData[index].z = globalPos.z;
					mGPUColl.mData[index].w = size;
					mGPUColl.bodyID[index] = i;//body->GetBody()->GetIdentifier();

					if(index==0)
					{
						xm = globalPos.x;
						xM = globalPos.x;
						ym = globalPos.y;
						yM = globalPos.y;
						zm = globalPos.z;
						zM = globalPos.z;
					}

					if(size>maxSize)
						maxSize = size;

					if(globalPos.x<xm)
						xm = globalPos.x;
					else if(globalPos.x>xM)
						xM = globalPos.x;

					if(globalPos.y<ym)
						ym = globalPos.y;
					else if(globalPos.y>yM)
						yM = globalPos.y;

					if(globalPos.z<zm)
						zm = globalPos.z;
					else if(globalPos.z>zM)
						zM = globalPos.z;

					mGPUColl.limits[0][0] = xm;
					mGPUColl.limits[0][1] = xM;
					mGPUColl.limits[1][0] = ym;
					mGPUColl.limits[1][1] = yM;
					mGPUColl.limits[2][0] = zm;
					mGPUColl.limits[2][1] = zM;

					index++;
				}
			}
		}





	} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____



#endif  // end of ! CH_UNIT_CUDA
