#ifndef CHC_BROADPHASE_H
#define CHC_BROADPHASE_H

#include "../ChCudaMath.h"
#include "../ChCudaDefines.h"
#include "ChCAABBGenerator.cuh"
namespace chrono {
    namespace collision {




        class ChApiGPU ChCBroadphase {
            public:
                // variables
                

                // functions
                ChCBroadphase();
                int detectPossibleCollisions(custom_vector<real3> &aabb_data, custom_vector<long long> &potentialCollisions);
                int setBinsPerAxis(real3 binsPerAxis);
                real3 getBinsPerAxis();

            private:
                // variables
                real3 min_bounding_point;
                real3 max_bounding_point;
                real3 global_origin;
                real3 bin_size_vec;
                real3 bins_per_axis;
                uint numAABB;
                uint last_active_bin, number_of_bin_intersections, number_of_contacts_possible;
                uint val;

                // functions
                void host_Count_AABB_BIN_Intersection(
                    const real3 *aabb_data,
                    uint *Bins_Intersected);
                void host_Store_AABB_BIN_Intersection(
                    const real3 *aabb_data,
                    const uint *Bins_Intersected,
                    uint *bin_number,
                    uint *body_number);
                void host_Count_AABB_AABB_Intersection(
                    const real3 *aabb_data,
                    const uint *bin_number,
                    const uint *body_number,
                    const uint *bin_start_index,
                    uint *Num_ContactD);
                void host_Store_AABB_AABB_Intersection(
                    const real3 *aabb_data,
                    const uint *bin_number,
                    const uint *body_number,
                    const uint *bin_start_index,
                    const uint *Num_ContactD,
                    long long *potentialCollisions);




        };
        struct AABB {
            real3 min, max;
        };


    }
}



#endif



