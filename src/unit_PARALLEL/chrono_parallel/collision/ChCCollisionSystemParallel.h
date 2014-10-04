#ifndef CHC_COLLISIONSYSTEMGPUA_H
#define CHC_COLLISIONSYSTEMGPUA_H
//////////////////////////////////////////////////
//
//   ChCCollisionSystemGPU.h
//
//   Header for class for collision engine based on
//   spatial subdivision method, performed on GPU.
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "physics/ChProximityContainerBase.h"
#include "physics/ChBody.h"

#include "collision/ChCCollisionSystem.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChLcpSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChContactContainerParallel.h"
#include "chrono_parallel/collision/ChCCollisionModelParallel.h"
#include "chrono_parallel/collision/ChCAABBGenerator.h"
#include "chrono_parallel/collision/ChCNarrowphaseDispatch.h"
#include "chrono_parallel/collision/ChCBroadphase.h"

namespace chrono {

  class ChSystemParallel;  // forward declaration

namespace collision {
///
/// Class for collision engine based on the spatial subdivision method.
/// Contains both the broadphase and the narrow phase methods.
///

class CH_PARALLEL_API ChCollisionSystemParallel : public ChCollisionSystem {
 public:

   ChCollisionSystemParallel();
   virtual ~ChCollisionSystemParallel();

   /// Clears all data instanced by this algorithm
   /// if any (like persistent contact manifolds)
   virtual void Clear(void) {
   }

   /// Adds a collision model to the collision
   /// engine (custom data may be allocated).
   virtual void Add(ChCollisionModel *model);

   /// Removes a collision model from the collision
   /// engine (custom data may be deallocated).
   virtual void Remove(ChCollisionModel *model);

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
   /// EndAddContact() of the contact container. But if a special container (say, GPU enabled)
   /// is passed, a more rapid buffer copy might be performed)
   virtual void ReportContacts(ChContactContainerBase *mcontactcontainer) {
   }

   /// After the Run() has completed, you can call this function to
   /// fill a 'proximity container' (container of narrow phase pairs), that is
   /// an object inherited from class ChProximityContainerBase. For instance ChSystem, after each Run()
   /// collision detection, calls this method multiple times for all proximity containers in the system,
   /// The basic behavior of the implementation is  the following: collision system
   /// will call in sequence the functions BeginAddProximities(), AddProximity() (x n times),
   /// EndAddProximities() of the proximity container. But if a special container (say, GPU enabled)
   /// is passed, a more rapid buffer copy might be performed)
   virtual void ReportProximities(ChProximityContainerBase *mproximitycontainer) {
   }

   /// Perform a raycast (ray-hit test with the collision models).
   virtual bool RayHit(const ChVector<> &from,
                       const ChVector<> &to,
                       ChRayhitResult &mresult) {
      return false;
   }

   void SetCollisionEnvelope(const real &envelope) {
      data_container->settings.collision.collision_envelope = envelope;
   }
   real GetCollisionEnvelope() {
      return data_container->settings.collision.collision_envelope;
   }

   std::vector<int2> GetOverlappingPairs();
   void GetOverlappingAABB(custom_vector<bool> &active_id,
                           real3 Amin,
                           real3 Amax);

   void setBinsPerAxis(int3 binsPerAxis) {
      broadphase->setBinsPerAxis(binsPerAxis);
   }
   void setBodyPerBin(int max,
                      int min) {
      data_container->settings.collision.min_body_per_bin = min;
      data_container->settings.collision.max_body_per_bin = max;
      broadphase->setBodyPerBin(max, min);

   }
   void SetAABB(real3 aabbmin,
                real3 aabbmax) {
      data_container->settings.collision.aabb_min = aabbmin;
      data_container->settings.collision.aabb_max = aabbmax;
      data_container->settings.collision.use_aabb_active = true;
   }

   bool GetAABB(real3 &aabbmin,
                real3 &aabbmax) {
      aabbmin = data_container->settings.collision.aabb_min;
      aabbmax = data_container->settings.collision.aabb_max;

      return data_container->settings.collision.use_aabb_active;
   }

 private:

   ChCBroadphase* broadphase;
   ChCNarrowphaseDispatch* narrowphase;

   ChCAABBGenerator aabb_generator;

   ChParallelDataManager *data_container;

   friend class chrono::ChSystemParallel;
};

}     // END_OF_NAMESPACE____
}     // END_OF_NAMESPACE____

#endif

