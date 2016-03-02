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
#include "chrono_parallel/physics/ChCudaHelper.cuh"

class ChMPM {
  public:
    ChMPM() {}
    ~ChMPM() {}
    void Initialize();
    void Solve(const real kernel_radius, std::vector<real3>& positions, std::vector<real3>& velocities);
    void UpdateState();

    uint num_mpm_markers;
    uint num_mpm_nodes;

    real3 min_bounding_point;
    real3 max_bounding_point;

    int3 bins_per_axis;
    real bin_edge;
    real inv_bin_edge;
    real mass;
    real kernel_radius;
    custom_vector<real> old_vel_node_mpm;

    custom_vector<real> delta_v;

    custom_vector<int> particle_node_mapping;
    custom_vector<int> node_particle_mapping;
    custom_vector<int> node_start_index;
    custom_vector<int> particle_number;
    uint num_mpm_nodes_active;
    custom_vector<Mat33> volume_Ap_Fe_transpose;

    // GPU Things
    real3* lower_bound;
    real3* upper_bound;

    gpu_vector<real3> pos, vel;
    gpu_vector<real> node_mass;
    gpu_vector<real> marker_volume;
    gpu_vector<real> grid_vel;
    gpu_vector<real> rhs;
    gpu_vector<Mat33> marker_Fe, marker_Fe_hat, marker_Fp, marker_delta_F;

};
