#ifndef CHC_BROADPHASE_H
#define CHC_BROADPHASE_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/collision/ChCAABBGenerator.h"

namespace chrono {
namespace collision {

class CH_PARALLEL_API ChCBroadphase {
 public:
  // variables

  // functions
  ChCBroadphase();
  int detectPossibleCollisions(ChParallelDataManager* data_container);
  void setBinsPerAxis(int3 binsPerAxis);
  int3 getBinsPerAxis();
  void setBodyPerBin(int max, int min) {
    min_body_per_bin = min;
    max_body_per_bin = max;
  }
  // These are measured variables for the broadphase
  real3 min_bounding_point;
  real3 max_bounding_point;
  real3 global_origin;
  real3 bin_size_vec;
  int3 grid_size;

  uint numAABB;
  int min_body_per_bin, max_body_per_bin;

 private:
  // variables

  uint last_active_bin, number_of_bin_intersections, number_of_contacts_possible;
  uint val;

  custom_vector<uint> Bins_Intersected;
  custom_vector<uint> bin_number;
  custom_vector<uint> shape_number;
  custom_vector<uint> bin_start_index;
  custom_vector<uint> Num_ContactD;

  // functions
  void host_Count_AABB_BIN_Intersection(const real3* aabb_data, uint* Bins_Intersected);
  void host_Store_AABB_BIN_Intersection(const real3* aabb_data,
                                        const uint* Bins_Intersected,
                                        uint* bin_number,
                                        uint* shape_number);
  void host_Count_AABB_AABB_Intersection(const real3* aabb_data,
                                         const uint* bin_number,
                                         const uint* shape_number,
                                         const uint* bin_start_index,
                                         const short2* fam_data,
                                         const bool* body_active,
                                         const uint* body_id,
                                         uint* Num_ContactD);
  void host_Store_AABB_AABB_Intersection(const real3* aabb_data,
                                         const uint* bin_number,
                                         const uint* shape_number,
                                         const uint* bin_start_index,
                                         const uint* Num_ContactD,
                                         const short2* fam_data,
                                         const bool* body_active,
                                         const uint* body_id,
                                         long long* potentialCollisions);
};
}
}

#endif
