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
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "ChCuda.h"
#include "ChCCollisionModelGPU.h"
#include "physics/ChBody.h"
#include "ChCCollisionGPU.h"
#include "ChLcpSystemDescriptorGPU.h"
#include "ChContactContainerGPUsimple.h"
#include "../collision/ChCCollisionSystem.h"
#include "../physics/ChProximityContainerBase.h"
#include "ChBodyGPU.h"
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
		/// EndAddContact() of the contact container. But if a special container (say, GPU enabled)
		/// is passed, a more rapid buffer copy might be performed)
		virtual void ReportContacts(ChContactContainerBase* mcontactcontainer) {
		}

		/// After the Run() has completed, you can call this function to
		/// fill a 'proximity container' (container of narrow phase pairs), that is
		/// an object inherited from class ChProximityContainerBase. For instance ChSystem, after each Run()
		/// collision detection, calls this method multiple times for all proximity containers in the system,
		/// The basic behavior of the implementation is  the following: collision system
		/// will call in sequence the functions BeginAddProximities(), AddProximity() (x n times),
		/// EndAddProximities() of the proximity container. But if a special container (say, GPU enabled)
		/// is passed, a more rapid buffer copy might be performed)
		virtual void ReportProximities(
				ChProximityContainerBase* mproximitycontainer) {
		}

		/// Perform a raycast (ray-hit test with the collision models).
		virtual bool RayHit(
				const ChVector<>& from,
				const ChVector<>& to,
				ChRayhitResult& mresult) {
			return false;
		}

		virtual void ComputeAABBHOST(ChGPUDataManager * data_container) {
			mGPU->ComputeAABBHOST(data_container);
		}
		virtual void ComputeBoundsHOST(ChGPUDataManager * data_container) {
			mGPU->ComputeBoundsHOST(data_container);
		}
		virtual void ComputeUpdateAABBHOST(ChGPUDataManager * data_container) {
			mGPU->UpdateAABBHOST(data_container);
		}

		virtual void ComputeAABB(
				uint &number_of_models,
				device_vector<float3> & device_pos_data,
				device_vector<float4> & device_rot_data,
				device_vector<float3> & device_ObA_data,
				device_vector<float3> & device_ObB_data,
				device_vector<float3> & device_ObC_data,
				device_vector<float4> & device_ObR_data,
				device_vector<int3> & device_typ_data,
				device_vector<float3> & device_aabb_data) {
			mGPU->ComputeAABB(
					number_of_models,
					device_pos_data,
					device_rot_data,
					device_ObA_data,
					device_ObB_data,
					device_ObC_data,
					device_ObR_data,
					device_typ_data,
					device_aabb_data);
		}
		virtual void ComputeBounds(
				uint & number_of_models,
				device_vector<float3> & device_aabb_data,
				float3& min_bounding_point,
				float3 &max_bounding_point) {
			mGPU->ComputeBounds(
					number_of_models,
					device_aabb_data,
					min_bounding_point,
					max_bounding_point);
		}
		virtual void ComputeUpdateAABB(
				device_vector<float3> &device_aabb_data,
				float3 & min_bounding_point,
				float3 & max_bounding_point,
				float3 & bin_size_vec,
				float & max_dimension,
				float & collision_envelope,
				uint & number_of_models) {
			mGPU->UpdateAABB(
					device_aabb_data,
					min_bounding_point,
					max_bounding_point,
					bin_size_vec,
					max_dimension,
					collision_envelope,
					number_of_models);
		}
		virtual void ComputeBroadPhase(
				device_vector<float3> &device_aabb_data,
				device_vector<int2> &device_fam_data,
				device_vector<int3> &device_typ_data,
				device_vector<long long> &contact_pair,
				uint &number_of_models,
				float3 &bin_size_vec,
				uint &number_of_contacts_possible) {
			mGPU->Broadphase(
					device_aabb_data,
					device_fam_data,
					device_typ_data,
					contact_pair,
					number_of_models,
					bin_size_vec,
					number_of_contacts_possible);
		}
		virtual void ComputeNarrowPhase(
				device_vector<float3> &device_norm_data,
				device_vector<float3> &device_cpta_data,
				device_vector<float3> &device_cptb_data,
				device_vector<float> &device_dpth_data,
				device_vector<int2> &device_bids_data,
				device_vector<float3> &device_gam_data,
				device_vector<float3> &device_pos_data,
				device_vector<float4> &device_rot_data,
				device_vector<float3> &device_ObA_data,
				device_vector<float3> &device_ObB_data,
				device_vector<float3> &device_ObC_data,
				device_vector<float4> &device_ObR_data,
				device_vector<int3> &device_typ_data,
				device_vector<long long> &contact_pair,
				uint &number_of_contacts_possible,
				uint & number_of_contacts) {
			mGPU->Narrowphase(
					device_norm_data,
					device_cpta_data,
					device_cptb_data,
					device_dpth_data,
					device_bids_data,
					device_gam_data,
					device_pos_data,
					device_rot_data,
					device_ObA_data,
					device_ObB_data,
					device_ObC_data,
					device_ObR_data,
					device_typ_data,
					contact_pair,
					number_of_contacts_possible,
					number_of_contacts);
		}

		ChCCollisionGPU *mGPU;
		ChGPUDataManager * data_container;
	private:
		/// Update data structures to pass into GPU for collision detection
		unsigned int counter;
};
} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif
