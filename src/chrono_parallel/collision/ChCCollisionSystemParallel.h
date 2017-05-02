#pragma once

#include "physics/ChProximityContainer.h"
#include "physics/ChBody.h"

#include "collision/ChCCollisionSystem.h"

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChContactContainerParallel.h"
#include "chrono_parallel/collision/ChCCollisionModelParallel.h"
#include "chrono_parallel/collision/ChCAABBGenerator.h"
#include "chrono_parallel/collision/ChCNarrowphaseDispatch.h"
#include "chrono_parallel/collision/ChCBroadphase.h"

namespace chrono {

class ChSystemParallel;  // forward declaration

namespace collision {

/// @addtogroup parallel_module
/// @{

///
/// Class for collision engine based on the spatial subdivision method.
/// Contains both the broadphase and the narrow phase methods.
///

class CH_PARALLEL_API ChCollisionSystemParallel : public ChCollisionSystem {
 public:
  ChCollisionSystemParallel(ChParallelDataManager* dc);
  virtual ~ChCollisionSystemParallel();

  /// Clears all data instanced by this algorithm
  /// if any (like persistent contact manifolds)
  virtual void Clear(void) {}

  /// Adds a collision model to the collision
  /// engine (custom data may be allocated).
  virtual void Add(ChCollisionModel* model);

  /// Removes a collision model from the collision
  /// engine (custom data may be deallocated).
  virtual void Remove(ChCollisionModel* model);

  /// Removes all collision models from the collision
  /// engine (custom data may be deallocated).
  // virtual void RemoveAll();

  /// Run the algorithm and finds all the contacts.
  /// (Contacts will be managed by the Bullet persistent contact cache).
  virtual void Run();

  /// After the Run() has completed, you can call this function to
  /// fill a 'contact container', that is an object inherited from class
  /// ChContactContainer. For instance ChSystem, after each Run()
  /// collision detection, calls this method multiple times for all contact containers in the system,
  /// The basic behavior of the implementation is the following: collision system
  /// will call in sequence the functions BeginAddContact(), AddContact() (x n times),
  /// EndAddContact() of the contact container. But if a special container (say, GPU enabled)
  /// is passed, a more rapid buffer copy might be performed)
  virtual void ReportContacts(ChContactContainer* mcontactcontainer) {}

  /// After the Run() has completed, you can call this function to
  /// fill a 'proximity container' (container of narrow phase pairs), that is
  /// an object inherited from class ChProximityContainer. For instance ChSystem, after each Run()
  /// collision detection, calls this method multiple times for all proximity containers in the system,
  /// The basic behavior of the implementation is  the following: collision system
  /// will call in sequence the functions BeginAddProximities(), AddProximity() (x n times),
  /// EndAddProximities() of the proximity container. But if a special container (say, GPU enabled)
  /// is passed, a more rapid buffer copy might be performed)
  virtual void ReportProximities(ChProximityContainer* mproximitycontainer) {}

  /// Perform a raycast (ray-hit test with the collision models).
  virtual bool RayHit(const ChVector<>& from, const ChVector<>& to, ChRayhitResult& mresult) { return false; }

  std::vector<int2> GetOverlappingPairs();
  void GetOverlappingAABB(custom_vector<bool>& active_id, real3 Amin, real3 Amax);

  void SetAABB(real3 aabbmin, real3 aabbmax) {
    data_manager->settings.collision.aabb_min = aabbmin;
    data_manager->settings.collision.aabb_max = aabbmax;
    data_manager->settings.collision.use_aabb_active = true;
  }

  bool GetAABB(real3& aabbmin, real3& aabbmax) {
    aabbmin = data_manager->settings.collision.aabb_min;
    aabbmax = data_manager->settings.collision.aabb_max;

    return data_manager->settings.collision.use_aabb_active;
  }

 private:
  ChCBroadphase* broadphase;
  ChCNarrowphaseDispatch* narrowphase;

  ChCAABBGenerator* aabb_generator;

  ChParallelDataManager* data_manager;

  friend class chrono::ChSystemParallel;
};

/// @} parallel_module
}  // end namespace collision
}  // end namespace chrono
