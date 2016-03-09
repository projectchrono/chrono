#include "chrono_parallel/physics/ChMPM.cuh"
#include "chrono_parallel/physics/MPMUtils.h"
#include "chrono_parallel/ChCudaHelper.cuh"
namespace chrono {

real3 min_bounding_point;
real3 max_bounding_point;

vec3 bins_per_axis;

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

void MPM_UpdateState();
void MPM_ComputeBounds();
void MPM_Multiply(gpu_vector<real>& input, gpu_vector<real>& output, gpu_vector<real>& r);
void MPM_BBSolver(gpu_vector<real>& rhs, gpu_vector<real>& delta_v);

struct Bounds {
    real minimum[3];
    real maximum[3];
};

MPM_Settings host_settings;

CUDA_CONSTANT MPM_Settings device_settings;
CUDA_CONSTANT Bounds system_bounds;

/////// BB Constants
__device__ real alpha = 0.0001;
__device__ real dot_ms_ms = 0;
__device__ real dot_ms_my = 0;
__device__ real dot_my_my = 0;
__device__ real gdiff = 1;

CUDA_CONSTANT real a_min = 1e-13;
CUDA_CONSTANT real a_max = 1e13;
CUDA_CONSTANT real neg_BB1_fallback = 0.11;
CUDA_CONSTANT real neg_BB2_fallback = 0.12;

#define LOOP_TWO_RING_GPU(X)                                                                         \
    const real bin_edge = device_settings.bin_edge;                                                  \
    const real inv_bin_edge = 1.f / bin_edge;                                                        \
                                                                                                     \
    const int cx = GridCoord(xi.x, inv_bin_edge, system_bounds.minimum[0]);                          \
    const int cy = GridCoord(xi.y, inv_bin_edge, system_bounds.minimum[1]);                          \
    const int cz = GridCoord(xi.z, inv_bin_edge, system_bounds.minimum[2]);                          \
                                                                                                     \
    for (int k = cz - 2; k <= cz + 2; ++k) {                                                         \
        for (int j = cy - 2; j <= cy + 2; ++j) {                                                     \
            for (int i = cx - 2; i <= cx + 2; ++i) {                                                 \
                vec3 bins_per_axis(device_settings.bins_per_axis_x, device_settings.bins_per_axis_y, \
                                   device_settings.bins_per_axis_z);                                 \
                const int current_node = GridHash(i, j, k, bins_per_axis);                           \
                real3 current_node_location;                                                         \
                current_node_location.x = i * bin_edge + system_bounds.minimum[0];                   \
                current_node_location.y = j * bin_edge + system_bounds.minimum[1];                   \
                current_node_location.z = k * bin_edge + system_bounds.minimum[2];                   \
                X                                                                                    \
            }                                                                                        \
        }                                                                                            \
    }

//////========================================================================================================================================================================
////
CUDA_GLOBAL void kComputeBounds(const real3* pos,  // input
                                real3* lower,      // output
                                real3* upper       // output
                                ) {
    typedef cub::BlockReduce<real3, num_threads_per_block> BlockReduce;

    __shared__ typename BlockReduce::TempStorage temp_storage;

    const int block_start = blockDim.x * blockIdx.x;
    const int num_valid = min(device_settings.num_mpm_markers - block_start, blockDim.x);

    const int index = block_start + threadIdx.x;
    if (index < device_settings.num_mpm_markers) {
        real3 data = pos[index];

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
////========================================================================================================================================================================
CUDA_GLOBAL void kRasterize(const real3* sorted_pos,  // input
                            const real3* sorted_vel,  // input
                            real* grid_mass,          // output
                            real* grid_vel) {         // output
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];
        LOOP_TWO_RING_GPU(                                                                     //
            real weight = N(xi - current_node_location, inv_bin_edge) * device_settings.mass;  //
            atomicAdd(&grid_mass[current_node], weight);                                       //
            AtomicAdd(&((real3*)grid_vel)[current_node], weight * real3(vi));                  //
            )
    }
}
CUDA_GLOBAL void kRasterize(const real3* sorted_pos,  // input
                            real* grid_mass) {        // output
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];
        LOOP_TWO_RING_GPU(                                                                     //
            real weight = N(xi - current_node_location, inv_bin_edge) * device_settings.mass;  //
            atomicAdd(&grid_mass[current_node], weight);                                       //
            )
    }
}
//
////========================================================================================================================================================================
//
CUDA_GLOBAL void kNormalizeWeights(real* grid_mass,   // input
                                   real* grid_vel) {  // output
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < device_settings.num_mpm_nodes) {
        real n_mass = grid_mass[i];
        if (n_mass > C_EPSILON) {
            grid_vel[i * 3 + 0] /= n_mass;
            grid_vel[i * 3 + 1] /= n_mass;
            grid_vel[i * 3 + 2] /= n_mass;
        }
    }
}
//////========================================================================================================================================================================
////
CUDA_GLOBAL void kComputeParticleVolumes(const real3* sorted_pos,  // input
                                         real* grid_mass,          // output
                                         real* volume) {
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];
        real particle_density = 0;
        LOOP_TWO_RING_GPU(                                              //
            real weight = N(xi - current_node_location, inv_bin_edge);  //
            particle_density += grid_mass[current_node] * weight;       //
            )
        // Inverse density to remove division
        particle_density = (bin_edge * bin_edge * bin_edge) / particle_density;
        volume[p] = device_settings.mass * particle_density;
    }
}

CUDA_GLOBAL void kRhs(const real3* sorted_pos,     // input
                      const Mat33* marker_Fe_hat,  // input
                      const Mat33* marker_Fe,      // input
                      const Mat33* marker_Fp,      // input
                      const real* marker_volume,   // input
                      real* rhs,                   // output
                      real* volume) {
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];

        Mat33 PED =
            Potential_Energy_Derivative_Deviatoric(marker_Fe_hat[p], marker_Fp[p], device_settings.mu,
                                                   device_settings.lambda, device_settings.hardening_coefficient);
        Mat33 vPEDFepT = marker_volume[p] * MultTranspose(PED, marker_Fe[p]);
        real JE = Determinant(marker_Fe[p]);  //
        real JP = Determinant(marker_Fp[p]);

        LOOP_TWO_RING_GPU(                                                  //
            real3 d_weight = dN(xi - current_node_location, inv_bin_edge);  //
            real3 force = device_settings.dt * (vPEDFepT * d_weight) / (JE * JP);

            rhs[current_node * 3 + 0] -= force.x;  //
            rhs[current_node * 3 + 1] -= force.y;  //
            rhs[current_node * 3 + 2] -= force.z;  //
            )
    }
}

CUDA_GLOBAL void kMultiplyA(const real3* sorted_pos,  // input
                            const real* v_array,
                            const real* old_vel_node_mpm,
                            const Mat33* marker_Fe_hat,  // input
                            const Mat33* marker_Fe,      // input
                            const Mat33* marker_Fp,      // input
                            const real* marker_volume,   // input
                            real* result_array) {
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];
        Mat33 delta_F(0);
        LOOP_TWO_RING_GPU(  //
            real3 vnew(v_array[current_node * 3 + 0], v_array[current_node * 3 + 1], v_array[current_node * 3 + 2]);
            real3 vold(old_vel_node_mpm[current_node * 3 + 0], old_vel_node_mpm[current_node * 3 + 1],
                       old_vel_node_mpm[current_node * 3 + 2]);
            real3 v0 = vold + vnew;                                   //
            real3 v1 = dN(xi - current_node_location, inv_bin_edge);  //
            delta_F += OuterProduct(v0, v1);                          //
            )

        real plastic_determinant = Determinant(marker_Fp[p]);
        real J = Determinant(marker_Fe_hat[p]);
        real current_mu = device_settings.mu * Exp(device_settings.hardening_coefficient * (1.0 - plastic_determinant));
        real current_lambda =
            device_settings.lambda * Exp(device_settings.hardening_coefficient * (1.0 - plastic_determinant));
        Mat33 Fe_hat_inv_transpose = InverseTranspose(marker_Fe_hat[p]);

        real dJ = J * InnerProduct(Fe_hat_inv_transpose, delta_F);
        Mat33 dF_inverse_transposed = -Fe_hat_inv_transpose * Transpose(delta_F) * Fe_hat_inv_transpose;
        Mat33 dJF_inverse_transposed = dJ * Fe_hat_inv_transpose + J * dF_inverse_transposed;
        Mat33 RD = Rotational_Derivative(marker_Fe_hat[p], delta_F);

        Mat33 volume_Ap_Fe_transpose =
            marker_volume[p] * (2 * current_mu * (delta_F - RD) + (current_lambda * J * dJ) * Fe_hat_inv_transpose +
                                (current_lambda * (J - 1.0)) * dJF_inverse_transposed) *
            Transpose(marker_Fe[p]);
        {
            LOOP_TWO_RING_GPU(  //
                real3 res = volume_Ap_Fe_transpose * dN(xi - current_node_location, inv_bin_edge);
                atomicAdd(&result_array[current_node * 3 + 0], res.x);
                atomicAdd(&result_array[current_node * 3 + 1], res.y);
                atomicAdd(&result_array[current_node * 3 + 2], res.z););
        }
    }
}
CUDA_GLOBAL void kMultiplyB(const real* v_array,
                            const real* old_vel_node_mpm,
                            const real* node_mass,
                            const real* offset_array,
                            real* result_array) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < device_settings.num_mpm_nodes) {
        real mass = node_mass[i];
        if (mass > C_EPSILON) {
            result_array[i * 3 + 0] =
                (mass * (v_array[i * 3 + 0] + old_vel_node_mpm[i * 3 + 0]) + result_array[i * 3 + 0]) -
                offset_array[i * 3 + 0];
            result_array[i * 3 + 1] =
                (mass * (v_array[i * 3 + 1] + old_vel_node_mpm[i * 3 + 1]) + result_array[i * 3 + 1]) -
                offset_array[i * 3 + 1];
            result_array[i * 3 + 2] =
                (mass * (v_array[i * 3 + 2] + old_vel_node_mpm[i * 3 + 2]) + result_array[i * 3 + 2]) -
                offset_array[i * 3 + 2];
        }
    }
}

void MPM_ComputeBounds() {
    max_bounding_point = real3(-FLT_MAX);
    min_bounding_point = real3(FLT_MAX);

    cudaMemcpyAsync(lower_bound, &min_bounding_point, sizeof(real3), cudaMemcpyHostToDevice);
    cudaMemcpyAsync(upper_bound, &max_bounding_point, sizeof(real3), cudaMemcpyHostToDevice);

    kComputeBounds<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,    //
                                                              lower_bound,   //
                                                              upper_bound);  //

    cudaMemcpy(&min_bounding_point, lower_bound, sizeof(real3), cudaMemcpyDeviceToHost);
    cudaMemcpy(&max_bounding_point, upper_bound, sizeof(real3), cudaMemcpyDeviceToHost);

    min_bounding_point.x = host_settings.kernel_radius * Round(min_bounding_point.x / host_settings.kernel_radius);
    min_bounding_point.y = host_settings.kernel_radius * Round(min_bounding_point.y / host_settings.kernel_radius);
    min_bounding_point.z = host_settings.kernel_radius * Round(min_bounding_point.z / host_settings.kernel_radius);

    max_bounding_point.x = host_settings.kernel_radius * Round(max_bounding_point.x / host_settings.kernel_radius);
    max_bounding_point.y = host_settings.kernel_radius * Round(max_bounding_point.y / host_settings.kernel_radius);
    max_bounding_point.z = host_settings.kernel_radius * Round(max_bounding_point.z / host_settings.kernel_radius);

    max_bounding_point = max_bounding_point + host_settings.kernel_radius * 8;
    min_bounding_point = min_bounding_point - host_settings.kernel_radius * 6;

    cudaMemcpyToSymbolAsync(system_bounds, &min_bounding_point, sizeof(real3), 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbolAsync(system_bounds, &max_bounding_point, sizeof(real3), sizeof(real3), cudaMemcpyHostToDevice);

    host_settings.bin_edge = host_settings.kernel_radius * 2;
    real3 bpa = real3(max_bounding_point - min_bounding_point) / host_settings.bin_edge;

    bins_per_axis.x = bpa.x;
    bins_per_axis.y = bpa.y;
    bins_per_axis.z = bpa.z;

    host_settings.inv_bin_edge = real(1.) / host_settings.bin_edge;
    host_settings.num_mpm_nodes = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;
    host_settings.bins_per_axis_x = bins_per_axis.x;
    host_settings.bins_per_axis_y = bins_per_axis.y;
    host_settings.bins_per_axis_z = bins_per_axis.z;

    cudaCheck(cudaMemcpyToSymbolAsync(device_settings, &host_settings, sizeof(MPM_Settings)));

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Compute DOF [%d %d %d] [%f] %d %d\n", bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           host_settings.bin_edge, host_settings.num_mpm_nodes, host_settings.num_mpm_markers);
}
//

void Multiply(gpu_vector<real>& input, gpu_vector<real>& output, gpu_vector<real>& r) {
    //    int size = input.size();
    //    kMultiplyA<<<CONFIG(size)>>>(sorted_pos,  // input
    //                                 input.data_d, old_vel_node_mpm.data_d,
    //                                 marker_Fe_hat.data_d,  // input
    //                                 marker_Fe.data_d,      // input
    //                                 marker_Fp.data_d,      // input
    //                                 marker_volume.data_d,  // input
    //                                 output.data_d);
    //
    //    kMultiplyB<<<CONFIG(size)>>>(input, old_vel_node_mpm.data_d, node_mass.data_d, r.data_d, result_array.data_d);
}
template <bool inner>
CUDA_GLOBAL void kResetGlobals(int size) {
    if (inner) {
        dot_ms_ms = 0;
        dot_ms_my = 0;
        dot_my_my = 0;
    } else {
        alpha = 0.0001;
        gdiff = 1.0 / pow(size, 2.0);
    }
}

template <bool even>
CUDA_GLOBAL void kUpdateAlpha(int num_items, real* ml_p, real* ml, real* mg_p, real* mg) {
    typedef cub::BlockReduce<real, num_threads_per_block> BlockReduce;
    __shared__ typename BlockReduce::TempStorage temp_storage;
    const int block_start = blockDim.x * blockIdx.x;
    const int num_valid = min(num_items - block_start, blockDim.x);

    const int tid = block_start + threadIdx.x;
    if (tid < num_items) {
        real data, block_sum;
        real ms = ml_p[tid] - ml[tid];
        real my = mg_p[tid] - mg[tid];

        if (even) {
            data = ms * ms;
            block_sum = BlockReduce(temp_storage).Reduce(data, cub::Sum(), num_valid);
            if (threadIdx.x == 0) {
                atomicAdd(&dot_ms_ms, block_sum);
            }
        } else {
            data = my * my;
            block_sum = BlockReduce(temp_storage).Reduce(data, cub::Sum(), num_valid);
            if (threadIdx.x == 0) {
                atomicAdd(&dot_my_my, block_sum);
            }
        }
        __syncthreads();
        data = ms * my;
        block_sum = BlockReduce(temp_storage).Reduce(data, cub::Sum(), num_valid);

        if (threadIdx.x == 0) {
            atomicAdd(&dot_ms_my, block_sum);
        }
    }
}

template <bool even>
CUDA_GLOBAL void kAlpha() {
    if (even) {
        if (dot_ms_my <= 0) {
            alpha = neg_BB1_fallback;
        } else {
            alpha = Min(a_max, Max(a_min, dot_ms_ms / dot_ms_my));
        }
    } else {
        if (dot_ms_my <= 0) {
            alpha = neg_BB2_fallback;
        } else {
            alpha = Min(a_max, Max(a_min, dot_ms_my / dot_my_my));
        }
    }
}

CUDA_GLOBAL void kCompute_ml_p(int num_items, real* ml, real* mg, real* ml_p) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < num_items) {
        ml_p[i] = ml[i] + alpha * mg[i];
    }
}
CUDA_GLOBAL void kResidual(int num_items, real* mg, real* dot_g_proj_norm) {
    typedef cub::BlockReduce<real, num_threads_per_block> BlockReduce;
    __shared__ typename BlockReduce::TempStorage temp_storage;
    const int block_start = blockDim.x * blockIdx.x;
    const int num_valid = min(num_items - block_start, blockDim.x);
    real data, block_sum;
    const int tid = block_start + threadIdx.x;
    if (tid < num_items) {
        data = gdiff * mg[tid] / (-gdiff);
        block_sum = BlockReduce(temp_storage).Reduce(data, cub::Sum(), num_valid);

        if (threadIdx.x == 0) {
            atomicAdd(&dot_g_proj_norm[0], block_sum);
        }
    }
}
void MPM_BBSolver(gpu_vector<real>& r, gpu_vector<real>& delta_v) {
    const uint size = rhs.size();
    gpu_vector<real> dot_g_proj_norm(1);
    ml.resize(size);
    mg.resize(size);
    mg_p.resize(size);
    ml_candidate.resize(size);
    mdir.resize(size);
    ml_p.resize(size);

    mg = 0;
    mg_p = 0;

    real lastgoodres = 10e30;

    // Kernel 1
    Multiply(delta_v, mg, r);
    //
    ml = delta_v;
    ml_candidate = delta_v;

    kResetGlobals<false><<<CONFIG(1)>>>(size);

    for (int current_iteration = 0; current_iteration < 40; current_iteration++) {
        kResetGlobals<true><<<CONFIG(1)>>>(size);

        kCompute_ml_p<<<CONFIG(size)>>>(size, ml.data_d, mg.data_d, ml_p.data_d);
        Multiply(ml_p, mg_p, r);
        ml = ml_p;
        mg = mg_p;

        if (current_iteration % 2 == 0) {
            kUpdateAlpha<true><<<CONFIG(size)>>>(size, ml_p.data_d, ml.data_d, mg_p.data_d, mg.data_d);
            kAlpha<true><<<CONFIG(1)>>>();
        } else {
            kUpdateAlpha<false><<<CONFIG(size)>>>(size, ml_p.data_d, ml.data_d, mg_p.data_d, mg.data_d);
            kAlpha<false><<<CONFIG(1)>>>();
        }

        kResidual<<<CONFIG(size)>>>(size, mg.data_d, dot_g_proj_norm.data_d);
        dot_g_proj_norm.copyDeviceToHost();
        real g_proj_norm = Sqrt(dot_g_proj_norm.data_h[0]);
        if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            ml_candidate = ml;
        }
    }

    delta_v = ml_candidate;
}

void MPM_Solve(MPM_Settings& settings, std::vector<real3>& positions, std::vector<real3>& velocities) {
    host_settings = settings;

    pos.data_h = positions;
    pos.copyHostToDevice();

    vel.data_h = velocities;
    vel.copyHostToDevice();

    cudaCheck(cudaMemcpyToSymbolAsync(device_settings, &host_settings, sizeof(MPM_Settings)));

    MPM_ComputeBounds();

    node_mass.resize(host_settings.num_mpm_nodes);
    node_mass = 0;

    grid_vel.resize(host_settings.num_mpm_nodes * 3);
    grid_vel = 0;

    // ========================================================================================
    kRasterize<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,        // input
                                                          vel.data_d,        // input
                                                          node_mass.data_d,  // output
                                                          grid_vel.data_d    // output
                                                          );

    kNormalizeWeights<<<CONFIG(host_settings.num_mpm_nodes)>>>(node_mass.data_d,  // output
                                                               grid_vel.data_d);

    old_vel_node_mpm.resize(host_settings.num_mpm_nodes * 3);
    old_vel_node_mpm = grid_vel;

    cudaCheck(cudaPeekAtLastError());
    cudaCheck(cudaDeviceSynchronize());

    rhs.resize(host_settings.num_mpm_nodes * 3);
    rhs = 0;
    //    kRhs<<<CONFIG(num_mpm_markers)>>>(pos.data_d,            // input
    //                                      marker_Fe_hat.data_d,  // input
    //                                      marker_Fe.data_d,      // input
    //                                      marker_Fp.data_d,      // input
    //                                      marker_volume.data_d,  // input
    //                                      rhs.data_d,            // output
    //                                      marker_volume.data_d);
    //    delta_v.resize(num_mpm_nodes);
    //    delta_v.set(0);
    //
    //    BBSolver(rhs, delta_v);

    // Solver Kernels
    // Resize
}

CUDA_GLOBAL void kInitFeFp(Mat33* marker_Fe, Mat33* marker_Fp) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < device_settings.num_mpm_markers) {
        marker_Fe[i] = Mat33(1);
        marker_Fp[i] = Mat33(1);
    }
}

void MPM_Initialize(MPM_Settings& settings, std::vector<real3>& positions) {
    host_settings = settings;

    cudaCheck(cudaMalloc(&lower_bound, sizeof(real3)));
    cudaCheck(cudaMalloc(&upper_bound, sizeof(real3)));

    pos.data_h = positions;
    pos.copyHostToDevice();

    cudaCheck(cudaMemcpyToSymbolAsync(device_settings, &host_settings, sizeof(MPM_Settings)));

    MPM_ComputeBounds();
    marker_volume.resize(host_settings.num_mpm_markers);
    node_mass.resize(host_settings.num_mpm_nodes);
    node_mass = 0;

    kRasterize<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,         // input
                                                          node_mass.data_d);  // output

    kComputeParticleVolumes<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,        // input
                                                                       node_mass.data_d,  // input
                                                                       marker_volume.data_d);  // output

    marker_Fe.resize(host_settings.num_mpm_markers);
    marker_Fe_hat.resize(host_settings.num_mpm_markers);
    marker_Fp.resize(host_settings.num_mpm_markers);
    marker_delta_F.resize(host_settings.num_mpm_markers);

    kInitFeFp<<<CONFIG(host_settings.num_mpm_markers)>>>(marker_Fe.data_d,   // output
                                                         marker_Fp.data_d);  // output

    cudaCheck(cudaPeekAtLastError());
    cudaCheck(cudaDeviceSynchronize());
}
}
