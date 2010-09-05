#ifndef CHC_COLLISIONSYSTEMBULLET_H
#define CHC_COLLISIONSYSTEMBULLET_H

//////////////////////////////////////////////////
//  
//   ChCCollisionSystemBullet.h
//
//   Header for class for collision engine based on
//   the 'Bullet' library.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "collision/ChCCollisionSystem.h"
#include "collision/bullet/btBulletCollisionCommon.h" 


namespace chrono 
{
namespace collision 
{


///
/// Class for collision engine based on the 'Bullet' library.
/// Contains either the broadphase and the narrow phase Bullet
/// methods.
/// 

class ChCollisionSystemBullet : public ChCollisionSystem
{
  public:

	ChCollisionSystemBullet(unsigned int max_objects = 16000, double scene_size = 500);
	virtual ~ChCollisionSystemBullet();

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
					/// (Contacts will be managed by the Bullet persistent contact cache).
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
	virtual void ReportProximities(ChProximityContainerBase* mproximitycontainer);


					/// Perform a raycast (ray-hit test with the collision models).
	virtual bool RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult);

					// For Bullet related stuff
	btCollisionWorld* GetBulletCollisionWorld() {return bt_collision_world;}

private:
	btCollisionConfiguration* bt_collision_configuration;
	btCollisionDispatcher*  bt_dispatcher;
	btBroadphaseInterface*	bt_broadphase;
	btCollisionWorld*		bt_collision_world; 

};






} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif
