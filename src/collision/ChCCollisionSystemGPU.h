#ifndef CHC_COLLISIONSYSTEMGPU_H
#define CHC_COLLISIONSYSTEMGPU_H

#ifndef CH_NOCUDA 

//////////////////////////////////////////////////
//  
//   ChCCollisionSystemGPU.h
//
//   Header for class for collision engine based on
//   spatial subdivision method, performed on GPU.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChCCollisionSystem.h"
#include "ChCModelGPU.h"
//#include "collision/bullet/btBulletCollisionCommon.h" 


namespace chrono 
{
namespace collision 
{


///
/// Class for collision engine based on the spatial subdivision method.
/// Contains both the broadphase and the narrow phase methods.
/// 

class ChCollisionSystemGPU : public ChCollisionSystem
{
  public:

	ChCollisionSystemGPU(unsigned int max_objects = 16000, double scene_size = 500);
	virtual ~ChCollisionSystemGPU();

					/// Clears all data instanced by this algorithm
					/// if any (like persistent contact manifolds)
    virtual void Clear(void);

					/// Adds a collision model to the collision
					/// engine (custom data may be allocated).
    virtual void Add(ChCollisionModel* model);

					/// Removes a collision model from the collision
					/// engine (custom data may be deallocated).
    virtual void Remove(ChCollisionModel* model);

					/// Removes all collision models from the collision
					/// engine (custom data may be deallocated).
    //virtual void RemoveAll();

					/// Run the algorithm and finds all the contacts.
	virtual void Run();

					/// After the Run() has completed, you can call this function to
					/// fill a 'contact container', that is an object inherited from class 
					/// ChContactContainerBase. For instance ChSystem, after each Run()
					/// collision detection, calls this method multiple times for all contact containers in the system,
					/// The basic behavior of the implementation is the following: collision system 
					/// will call in sequence the functions BeginAddContact(), AddContact() (x n times), 
					/// EndAddContact() of the contact container.
	virtual void ReportContacts(ChContactContainerBase* mcontactcontainer);

					/// After the Run() has completed, you can call this function to
					/// fill a 'proximity container' (container of narrow phase pairs), that is 
					/// an object inherited from class ChProximityContainerBase. For instance ChSystem, after each Run()
					/// collision detection, calls this method multiple times for all proximity containers in the system,
					/// The basic behavior of the implementation is  the following: collision system 
					/// will call in sequence the functions BeginAddProximities(), AddProximity() (x n times), 
					/// EndAddProximities() of the proximity container.
	virtual void ReportProximities(ChProximityContainerBase* mproximitycontainer) {}; //***TO DO***


					/// Perform a raycast (ray-hit test with the collision models).
	virtual bool RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult);

	struct contact_Data{
		//float localPointA[3];
		//float localPointB[3];
		float positionWorldOnB[3];
		float positionWorldOnA[3];
		float normalWorldOnB[3];
		float distance;
		//float lateralFrictionDir1[3];
		//float lateralFrictionDir2[3];
		int objectIdA;
		int objectIdB;
	};

private:
					/// Update data structures to pass into GPU for collision detection
	void updateDataStructures();

private:
	float	*x_data,*y_data,*z_data,*r_data;
	float *reactionCache;
	float maxSize;
	float limits[3][2];
	int num_Bodies;
	int numContacts;
	contact_Data *contactdata;
	chrono::collision::ChModelGPU **colModels;
	
};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif  // end of ! CH_NOCUDA

#endif

