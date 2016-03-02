#include "chrono_parallel/physics/ChMPM.h"
#include "chrono_parallel/physics/MPMUtils.h"

struct Bounds {
    real minimum[3];
    real maximum[3];
};

CUDA_CONSTANT Bounds system_bounds;

//========================================================================================================================================================================
static inline CUDA_DEVICE void AtomicAdd(real3* pointer, real3 val) {
    atomicAdd(&pointer->x, val.x);
    atomicAdd(&pointer->y, val.y);
    atomicAdd(&pointer->z, val.z);
}
//========================================================================================================================================================================

CUDA_GLOBAL void kComputeBounds(int num_particles,  // parameter
                                const real4* pos,   // input
                                real3* lower,       // output
                                real3* upper        // output
                                ) {
    typedef cub::BlockReduce<real3, num_threads_per_block> BlockReduce;

    __shared__ typename BlockReduce::TempStorage temp_storage;

    const int block_start = blockDim.x * blockIdx.x;
    const int num_valid = min(num_particles - block_start, blockDim.x);

    const int tid = block_start + threadIdx.x;

    if (tid < num_particles) {
        const int index = active_indices[tid];

        real3 data = real3(pos[index]);

        real3 blockUpper = BlockReduce(temp_storage).Reduce(data, real3Max(), num_valid);

        // sync threads because second reduce uses same temp storage as first
        __syncthreads();

        real3 blockLower = BlockReduce(temp_storage).Reduce(data, real3Min(), num_valid);

        if (threadIdx.x == 0) {
            // write out block results, expanded by the radius
            AtomicMax(upper, blockUpper);
            AtomicMin(lower, blockLower);
        }
    }
}
//========================================================================================================================================================================
CUDA_GLOBAL void kRasterize(int num_particles,        // parameters
                            int3 bins_per_axis,       // parameters
                            real mass,                // parameters
                            const real3* sorted_pos,  // input
                            const real3* sorted_vel,  // input
                            real* grid_mass,          // output
                            real* grid_vel) {         // output
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < num_particles) {
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];
        LOOPOVERNODES(                                                         //
            real weight = N(xi - current_node_location, inv_bin_edge) * mass;  //
            atomicAdd(&grid_mass[current_node], weight);                       //
            AtomicAdd(&((real3*)grid_vel)[current_node], weight * real3(vi));  //
            )
    }
}
CUDA_GLOBAL void kRasterize(int num_particles,        // parameters
                            int3 bins_per_axis,       // parameters
                            real mass,                // parameters
                            const real3* sorted_pos,  // input
                            real* grid_mass) {        // output
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < num_particles) {
        const real3 xi = sorted_pos[p];
        LOOPOVERNODES(                                                         //
            real weight = N(xi - current_node_location, inv_bin_edge) * mass;  //
            atomicAdd(&grid_mass[current_node], weight);                       //
            )
    }
}
//========================================================================================================================================================================

CUDA_GLOBAL void kComputeParticleVolumes(int num_particles,        // parameters
                                         int3 bins_per_axis,       // parameters
                                         const real4* sorted_pos,  // input
                                         real* grid_mass,          // output
                                         real* volume) {
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < num_particles) {
        const real4 xi = sorted_pos[p];
        real mass = 1.0 / xi.w;
        real particle_density = 0;
        LOOPOVERNODES(                                                         //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge);  //
            particle_density += grid_mass[current_node] * weight;              //
            atomicAdd(&grid_mass[current_node], weight);                       //
            )
        particle_density /= bin_edge * bin_edge * bin_edge;
        volume[p] = mass / particle_density;
    }
}
void ChMPM::Bounds(const real kernel_radius, std::vector<real3>& positions) {
    max_bounding_point = -FLT_MAX;
    min_bounding_point = FLT_MAX;

    cudaMemcpyAsync(lower_bound, &min_bounding_point, sizeof(real3), cudaMemcpyHostToDevice);
    cudaMemcpyAsync(upper_bound, &max_bounding_point, sizeof(real3), cudaMemcpyHostToDevice);

    kComputeBounds<<<CONFIG(n)>>>(num_mpm_markers,  //
                                  pos.data_d,       //
                                  lower_bound,      //
                                  upper_bound);     //

    cudaMemcpy(&min_bounding_point, lower_bound, sizeof(real3), cudaMemcpyDeviceToHost);
    cudaMemcpy(&max_bounding_point, upper_bound, sizeof(real3), cudaMemcpyDeviceToHost);

    max_bounding_point = max_bounding_point + kernel_radius * 8;
    min_bounding_point = min_bounding_point - kernel_radius * 6;

    cudaMemcpyToSymbolAsync(system_bounds, &min_bounding_point, sizeof(real3), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbolAsync(system_bounds, &max_bounding_point, sizeof(real3), sizeof(real3), cudaMemcpyHostToDevice);

    bin_edge = kernel_radius * 2;
    bins_per_axis = int3(real3(max_bounding_point - min_bounding_point) / bin_edge);
    inv_bin_edge = real(1.) / bin_edge;
    num_mpm_nodes = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Compute DOF [%d %d %d] [%f] %d %d\n", bins_per_axis.x, bins_per_axis.y, bins_per_axis.z, bin_edge,
           num_mpm_nodes, num_mpm_markers);
}

void ChMPM::Initialize(const real marker_mass, const real radius, std::vector<real3>& positions) {
    num_mpm_markers = positions.size();
    mass = marker_mass;
    kernel_radius = radius;

    cudaCheck(cudaMalloc(&lower_bound, sizeof(real3)));
    cudaCheck(cudaMalloc(&upper_bound, sizeof(real3)));

    pos.h_data = positions;
    pos.copyHostToDevice();
    Bounds(kernel_radius, positions);
    marker_volume.resize(num_mpm_markers);
    node_mass.resize(num_mpm_nodes);
    node_mass.set(0);

    kRasterize<<<CONFIG(num_mpm_markers)>>>(num_mpm_markers,    // parameter
                                            bins_per_axis,      // parameter
                                            mass,               // parameter
                                            pos.d_data,         // input
                                            grid_mass.d_data);  // output

    kComputeParticleVolumes<<<CONFIG(num_mpm_markers)>>>(num_mpm_markers,   // parameters
                                                         bins_per_axis,     // parameters
                                                         pos.d_data,        // input
                                                         grid_mass.d_data,  // output
                                                         marker_volume.d_data);
}

void ChMPM::Solve(const real kernel_radius, std::vector<real3>& positions, std::vector<real3>& velocities) {
    num_mpm_markers = positions.size();

    Bounds(kernel_radius, positions);

    // ========================================================================================
    kRasterize<<<CONFIG(num_mpm_markers)>>>(num_mpm_markers,  // parameter
                                            bins_per_axis,    // parameter
                                            mass,
                                            container.sorted_pos,  // input
                                            container.sorted_vel,  // input
                                            container.grid_mass,   // output
                                            container.grid_vel     // output
                                            );

#pragma omp parallel for
    for (int index = 0; index < num_mpm_nodes_active; index++) {
        uint start = node_start_index[index];
        uint end = node_start_index[index + 1];
        const int current_node = node_particle_mapping[index];
        int3 g = GridDecode(current_node, bins_per_axis);
        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
        real3 vel_mass = real3(0);
        real mass_w = 0;
        for (uint i = start; i < end; i++) {
            int p = particle_number[i];
            real weight = N(pos_marker[p] - current_node_location, inv_bin_edge) * mass;  //
            mass_w += weight;
            vel_mass += weight * vel_marker[p];
        }

        node_mass[current_node] = mass_w;
        grid_vel[current_node * 3 + 0] = vel_mass.x;  //
        grid_vel[current_node * 3 + 1] = vel_mass.y;  //
        grid_vel[current_node * 3 + 2] = vel_mass.z;  //
    }

#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        real n_mass = node_mass[i];
        if (n_mass > C_EPSILON) {
            grid_vel[i * 3 + 0] /= n_mass;
            grid_vel[i * 3 + 1] /= n_mass;
            grid_vel[i * 3 + 2] /= n_mass;
        }
        // printf("N: %d [%f %f %f]\n", i, grid_vel[i * 3 + 0], grid_vel[i * 3 + 1], grid_vel[i * 3 + 2]);
    }

    old_vel_node_mpm = grid_vel;
    SVD_Fe_hat_R.resize(num_mpm_markers);
    SVD_Fe_hat_S.resize(num_mpm_markers);

#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        marker_Fe_hat[p] = Mat33(1.0);
        Mat33 Fe_hat_t(1);
        LOOPOVERNODES(  //
            real3 vel(grid_vel[i * 3 + 0], grid_vel[i * 3 + 1], grid_vel[i * 3 + 2]);
            real3 kern = dN(xi - current_node_location, inv_bin_edge);  //
            Fe_hat_t += OuterProduct(dt * vel, kern);                   //
            )
        Mat33 Fe_hat = Fe_hat_t * marker_Fe[p];
        marker_Fe_hat[p] = Fe_hat;
        Mat33 U, V;
        real3 E;
        SVD(Fe_hat, U, E, V);
        SVD_Fe_hat_R[p] = MultTranspose(U, V);
        SVD_Fe_hat_S[p] = V * MultTranspose(Mat33(E), V);
    }

    custom_vector<Mat33> rhs_temp(num_mpm_markers);
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];

        Mat33 PED =
            Potential_Energy_Derivative_Deviatoric(marker_Fe_hat[p], marker_Fp[p], mu, lambda, hardening_coefficient);

        Mat33 vPEDFepT = marker_volume[p] * MultTranspose(PED, marker_Fe[p]);
        real JE = Determinant(marker_Fe[p]);  //
        real JP = Determinant(marker_Fp[p]);  //
        rhs_temp[p] = (vPEDFepT) * (1.0 / (JE * JP));
    }

#pragma omp parallel for
    for (int index = 0; index < num_mpm_nodes_active; index++) {
        uint start = node_start_index[index];
        uint end = node_start_index[index + 1];
        const int current_node = node_particle_mapping[index];
        int3 g = GridDecode(current_node, bins_per_axis);
        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
        real3 force = real3(0);
        for (uint i = start; i < end; i++) {
            int p = particle_number[i];
            force += dt * rhs_temp[p] * dN(pos_marker[p] - current_node_location, inv_bin_edge);  //;
        }

        rhs[current_node * 3 + 0] = -force.x;  //
        rhs[current_node * 3 + 1] = -force.y;  //
        rhs[current_node * 3 + 2] = -force.z;  //
    }
}
