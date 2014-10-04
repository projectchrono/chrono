#ifndef CHC_NARROWPHASEDISPATCH_H
#define CHC_NARROWPHASEDISPATCH_H

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

class CH_PARALLEL_API ChCNarrowphaseDispatch {
 public:
   ChCNarrowphaseDispatch() {
   }
   ~ChCNarrowphaseDispatch() {
   }
   //Perform collision detection
   void Process(ChParallelDataManager* data_container);

   void PreprocessCount(const shape_type* obj_data_T,
                        const long long* collision_pair,
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
                 long long *contact_pair,
                 uint *contact_active,
                 real3 *norm,
                 real3 *ptA,
                 real3 *ptB,
                 real *contactDepth,
                 real* erad,
                 int2 *ids,
                 uint* start_index);

 private:

   custom_vector<real3> obj_data_A_global, obj_data_B_global, obj_data_C_global;   //
   custom_vector<uint> contact_active;   //
   custom_vector<uint> contact_index;
   unsigned int num_potentialCollisions;
   real collision_envelope;
   uint number_of_contacts;
   NARROWPHASETYPE narrowphase_algorithm;

};
}  // end namespace collision
}  // end namespace chrono

#endif

