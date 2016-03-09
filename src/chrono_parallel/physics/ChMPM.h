// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: Class for the Pure MPM solve, takes positions and velocities
// as input, outputs updated positions and velocities
// =============================================================================
#include "chrono_parallel/ChCudaHelper.cuh"
#include "chrono_parallel/math/matrix.h"
#include "chrono_parallel/math/other_types.h"

namespace chrono {
class ChMPM {
  public:
    ChMPM() {}
    ~ChMPM() {}
    void Initialize(const real marker_mass, const real radius, std::vector<real3>& positions);
    void Solve(const real kernel_radius, std::vector<real3>& positions, std::vector<real3>& velocities);
    void UpdateState();
    void Bounds(const real kernel_radius, std::vector<real3>& positions);
    void Multiply(gpu_vector<real>& input, gpu_vector<real>& output, gpu_vector<real>& r);
    void BBSolver(gpu_vector<real>& rhs, gpu_vector<real>& delta_v);
    uint num_mpm_markers;
    uint num_mpm_nodes;

    real3 min_bounding_point;
    real3 max_bounding_point;

    vec3 bins_per_axis;
    real bin_edge;
    real inv_bin_edge;
    real mass;
    real kernel_radius;

    std::vector<int> particle_node_mapping;
    std::vector<int> node_particle_mapping;
    std::vector<int> node_start_index;
    std::vector<int> particle_number;
    uint num_mpm_nodes_active;
    std::vector<Mat33> volume_Ap_Fe_transpose;

    // GPU Things
    real3* lower_bound;
    real3* upper_bound;

    gpu_vector<real3> pos, vel;
    gpu_vector<real> node_mass;
    gpu_vector<real> marker_volume;
    gpu_vector<real> grid_vel, delta_v;
    gpu_vector<real> rhs;
    gpu_vector<Mat33> marker_Fe, marker_Fe_hat, marker_Fp, marker_delta_F;
    gpu_vector<real> old_vel_node_mpm;
    gpu_vector<real> temp, ml, mg, mg_p, ml_candidate, ms, my, mdir, ml_p;
};
}
