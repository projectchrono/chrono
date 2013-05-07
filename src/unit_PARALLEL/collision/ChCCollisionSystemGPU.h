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

#include "../ChCudaDefines.h"
#include "ChCCollisionModelGPU.h"
#include "physics/ChBody.h"

#include "../ChLcpSystemDescriptorGPU.h"
#include "ChContactContainerGPU.h"
#include "../../collision/ChCCollisionSystem.h"
#include "../../physics/ChProximityContainerBase.h"
#include "ChBodyGPU.h"
#include "ChCAABBGenerator.cuh"
#include "ChCNarrowphase.cuh"
#include "ChCBroadphase.cuh"
namespace chrono {
namespace collision {
///
/// Class for collision engine based on the spatial subdivision method.
/// Contains both the broadphase and the narrow phase methods.
///

class ChApiGPU ChCollisionSystemGPU: public ChCollisionSystem {
public:

	ChCollisionSystemGPU();

	virtual ~ChCollisionSystemGPU() {
	}

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
	virtual bool RayHit(const ChVector<> &from, const ChVector<> &to, ChRayhitResult &mresult) {
		return false;
	}

	void SetCollisionEnvelope(const real &envelope) {
		collision_envelope = envelope;
	}
	real GetCollisionEnvelope() {
		return collision_envelope;
	}

	vector<int2> GetOverlappingPairs();

	void setBinsPerAxis(real3 binsPerAxis) {

		bpa = binsPerAxis;

	}
	void setBodyPerBin(int max, int min) {
		min_body_per_bin = min;
		max_body_per_bin = max;
	}
	ChGPUDataManager *data_container;

private:
	real collision_envelope;
	/// Update data structures to pass into GPU for collision detection
	//unsigned int counter;
	real3 bpa;
	int min_body_per_bin;
	int max_body_per_bin;

};
} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif

