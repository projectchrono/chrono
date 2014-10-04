#ifndef CHC_NARROWPHASE_H
#define CHC_NARROWPHASE_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"

namespace chrono {
namespace collision {
/*
 * Narrowphase dispatch will handle the outer loop for the collision detection code
 * For each contact pair it will decide what algorithm to use
 * The user can specify if they want to use only MPR, GJK etc or a hybrid approach with custom functions for certain pair types
 *
 *
 *
 */

//// TODO: what is a good value here?
////       Should this even be a constant?  Maybe set it based on object size?
const real ChCNarrowphaseR::edge_radius = 0.1;

class CH_PARALLEL_API ChCNarrowphaseDispatch {
 public:
   ChCNarrowphaseDispatch() {
   }
   ~ChCNarrowphaseDispatch() {
   }
   //Perform collision detection
   //This function makes it easier to call different versions of the narrowphase
   // CPU/GPU etc without changing the actual call to run the narrowphase
   void Process(ChParallelDataManager* data_container) = 0;

   void PreprocessCount(const shape_type* obj_data_T,
                        const long long* collision_pair,
                        const NARROWPHASETYPE &narrowphase_algorithm,
                        uint* max_contacts);

   //Transform the shape data to the global reference frame
   //Perform this as a preprocessing step to improve performance
   //Performance is improved because the amount of data loaded is still the same
   // but it does not have to be transformed per contact pair, now it is
   // transformed once per shape
   void PreprocessLocalToParent(const int numAABB,
                                const shape_type *obj_data_T,
                                const real3 *obj_data_A,
                                const real3 *obj_data_B,
                                const real3 *obj_data_C,
                                const real4 *obj_data_R,
                                const uint *obj_data_ID,
                                const bool * obj_active,
                                const real3 *body_pos,
                                const real4 *body_rot,
                                real3 *obj_data_A_mod,
                                real3 *obj_data_B_mod,
                                real3 *obj_data_C_mod);

   //Actual narrowphase code, runs preprocessing and then calls the dispatcher
   void PerformNarrowphase(const custom_vector<shape_type> &obj_data_T,
                           const custom_vector<real3> &obj_data_A,
                           const custom_vector<real3> &obj_data_B,
                           const custom_vector<real3> &obj_data_C,
                           const custom_vector<real4> &obj_data_R,
                           const custom_vector<uint> &obj_data_ID,
                           const custom_vector<bool> & obj_active,
                           const custom_vector<real3> &body_pos,
                           const custom_vector<real4> &body_rot,
                           const real & collision_envelope,
                           const NARROWPHASETYPE &narrowphase_algorithm,
                           custom_vector<long long> &potentialCollisions,
                           custom_vector<real3> &norm_data,
                           custom_vector<real3> &cpta_data,
                           custom_vector<real3> &cptb_data,
                           custom_vector<real> &dpth_data,
                           custom_vector<real> &erad_data,
                           custom_vector<int2> &bids_data,
                           uint & number_of_contacts);
   //For each contact pair decide what to do.
   void Dispatch(const shape_type *obj_data_T,
                 const real3 *obj_data_A,
                 const real3 *obj_data_B,
                 const real3 *obj_data_C,
                 const real4 *obj_data_R,
                 const uint *obj_data_ID,
                 const bool * obj_active,
                 const real3 *body_pos,
                 const real4 *body_rot,
                 const real & collision_envelope,
                 const NARROWPHASETYPE &narrowphase_algorithm,
                 long long *contact_pair,
                 uint *contact_active,
                 real3 *norm,
                 real3 *ptA,
                 real3 *ptB,
                 real *contactDepth,
                 int2 *ids,
                 uint* start_index);

 private:
   unsigned int num_potentialCollisions;custom_vector<real3> obj_data_A_global, obj_data_B_global, obj_data_C_global;custom_vector<uint> contact_active;
};
}  // end namespace collision
}  // end namespace chrono

#endif

