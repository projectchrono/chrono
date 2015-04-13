#ifndef CHC_NARROWPHASEDISPATCH_H
#define CHC_NARROWPHASEDISPATCH_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChCDataStructures.h"
namespace chrono {
namespace collision {
/*
 * Narrowphase dispatch will handle the outer loop for the collision detection code
 * For each contact pair it will decide what algorithm to use
 * The user can specify if they want to use only MPR, GJK etc or a hybrid approach with custom functions for certain
 *pair types
 *
 *
 *
 */

class CH_PARALLEL_API ChCNarrowphaseDispatch {
 public:
  ChCNarrowphaseDispatch() {}
  ~ChCNarrowphaseDispatch() {}
  // Perform collision detection
  void Process();

  void PreprocessCount();

  // Transform the shape data to the global reference frame
  // Perform this as a preprocessing step to improve performance
  // Performance is improved because the amount of data loaded is still the same
  // but it does not have to be transformed per contact pair, now it is
  // transformed once per shape
  void PreprocessLocalToParent();

  // For each contact pair decide what to do.
  void Dispatch();
  void DispatchMPR();
  void DispatchGJK();
  void DispatchR();
  void DispatchHybridMPR();
  void DispatchHybridGJK();
  void Dispatch_Init(uint index, uint& icoll, uint& ID_A, uint& ID_B, ConvexShape& shapeA, ConvexShape& shapeB);
  void Dispatch_Finalize(uint icoll, uint ID_A, uint ID_B, int nC);
  ChParallelDataManager* data_manager;

 private:
  custom_vector<real3> obj_data_A_global, obj_data_B_global, obj_data_C_global;  //
  custom_vector<real4> obj_data_R_global;
  custom_vector<bool> contact_active;
  custom_vector<uint> contact_index;
  unsigned int num_potentialCollisions;
  real collision_envelope;
  NARROWPHASETYPE narrowphase_algorithm;
  SYSTEMTYPE system_type;
};
}  // end namespace collision
}  // end namespace chrono

#endif
