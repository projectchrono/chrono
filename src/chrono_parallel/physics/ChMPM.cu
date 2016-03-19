#include "chrono_parallel/physics/ChMPM.cuh"
#include "chrono_parallel/physics/MPMUtils.h"
#include "chrono_parallel/ChCudaHelper.cuh"
#include "chrono_parallel/ChGPUVector.cuh"
#include "thirdparty/cub/cub.cuh"
namespace chrono {

struct Bounds {
    real minimum[3];
    real maximum[3];
};

real3 min_bounding_point;
real3 max_bounding_point;
vec3 bins_per_axis;

MPM_Settings host_settings;

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
gpu_vector<Mat33> marker_Fe, marker_Fe_hat, marker_Fp, marker_delta_F, PolarR, PolarS;
gpu_vector<real> old_vel_node_mpm;
gpu_vector<real> ml, mg, mg_p, ml_p;
gpu_vector<real> dot_g_proj_norm;

CUDA_CONSTANT MPM_Settings device_settings;
CUDA_CONSTANT Bounds system_bounds;

cudaEvent_t start;
cudaEvent_t stop;
float time_measured = 0;
/////// BB Constants
__device__ real alpha = 0.0001;
__device__ real dot_ms_ms = 0;
__device__ real dot_ms_my = 0;
__device__ real dot_my_my = 0;

#define a_min 1e-13
#define a_max 1e13
#define neg_BB1_fallback 0.11
#define neg_BB2_fallback 0.12

#define LOOP_TWO_RING_GPU(X)                                                                                         \
    const real bin_edge = device_settings.bin_edge;                                                                  \
    const real inv_bin_edge = real(1.0) / bin_edge;                                                                        \
                                                                                                                     \
    const int cx = GridCoord(xi.x, inv_bin_edge, system_bounds.minimum[0]);                                          \
    const int cy = GridCoord(xi.y, inv_bin_edge, system_bounds.minimum[1]);                                          \
    const int cz = GridCoord(xi.z, inv_bin_edge, system_bounds.minimum[2]);                                          \
    for (int i = cx - 2; i <= cx + 2; ++i) {                                                                         \
        for (int j = cy - 2; j <= cy + 2; ++j) {                                                                     \
            for (int k = cz - 2; k <= cz + 2; ++k) {                                                                 \
                const int current_node = GridHash(i, j, k, device_settings.bins_per_axis_x,                          \
                                                  device_settings.bins_per_axis_y, device_settings.bins_per_axis_z); \
                real3 current_node_location;                                                                         \
                current_node_location.x = i * bin_edge + system_bounds.minimum[0];                                   \
                current_node_location.y = j * bin_edge + system_bounds.minimum[1];                                   \
                current_node_location.z = k * bin_edge + system_bounds.minimum[2];                                   \
                X                                                                                                    \
            }                                                                                                        \
        }                                                                                                            \
    }

#define LOOP_TWO_RING_GPUSP(X)                                                                                         \
    cx = GridCoord(xi.x, inv_bin_edge, system_bounds.minimum[0]);                                                      \
    cy = GridCoord(xi.y, inv_bin_edge, system_bounds.minimum[1]);                                                      \
    cz = GridCoord(xi.z, inv_bin_edge, system_bounds.minimum[2]);                                                      \
    for (int i = cx - 2; i <= cx + 2; ++i) {                                                                           \
        for (int j = cy - 2; j <= cy + 2; ++j) {                                                                       \
            for (int k = cz - 2; k <= cz + 2; ++k) {                                                                   \
                int current_node = GridHash(i, j, k, device_settings.bins_per_axis_x, device_settings.bins_per_axis_y, \
                                            device_settings.bins_per_axis_z);                                          \
                real current_node_locationx = i * bin_edge + system_bounds.minimum[0];                                 \
                real current_node_locationy = j * bin_edge + system_bounds.minimum[1];                                 \
                real current_node_locationz = k * bin_edge + system_bounds.minimum[2];                                 \
                X                                                                                                      \
            }                                                                                                          \
        }                                                                                                              \
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
        __syncthreads();
        real3 blockLower = BlockReduce(temp_storage).Reduce(data, real3Min(), num_valid);
        if (threadIdx.x == 0) {
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

        int cx, cy, cz;
        const real bin_edge = device_settings.bin_edge;
        const real inv_bin_edge = real(1.0) / bin_edge;

        LOOP_TWO_RING_GPUSP(  //

            real weight = N((xi.x - current_node_locationx) * inv_bin_edge) *
                          N((xi.y - current_node_locationy) * inv_bin_edge) *
                          N((xi.z - current_node_locationz) * inv_bin_edge) * device_settings.mass;

            ATOMIC_ADD(&grid_mass[current_node], weight);  //
            ATOMIC_ADD(&grid_vel[current_node * 3 + 0], weight * vi.x);
            ATOMIC_ADD(&grid_vel[current_node * 3 + 1], weight * vi.y);
            ATOMIC_ADD(&grid_vel[current_node * 3 + 2], weight * vi.z);)
    }
}
CUDA_GLOBAL void kRasterize(const real3* sorted_pos,  // input
                            real* grid_mass) {        // output
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];
        LOOP_TWO_RING_GPU(                                                                     //
            real weight = N(xi - current_node_location, inv_bin_edge) * device_settings.mass;  //
            ATOMIC_ADD(&grid_mass[current_node], weight);                                      //
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
CUDA_GLOBAL void kFeHat(const real3* sorted_pos,  // input
                        const Mat33* marker_Fe,   // input
                        const real* grid_vel,     // input
                        Mat33* marker_Fe_hat) {   // output

    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];
        Mat33 Fe_hat_t(0.0);

        int cx, cy, cz;
        const real bin_edge = device_settings.bin_edge;
        const real inv_bin_edge = real(1.0) / bin_edge;

        LOOP_TWO_RING_GPUSP(

            real vnx = grid_vel[current_node * 3 + 0];  //
            real vny = grid_vel[current_node * 3 + 1];  //
            real vnz = grid_vel[current_node * 3 + 2];

            real Tx = (xi.x - current_node_locationx) * inv_bin_edge;  //
            real Ty = (xi.y - current_node_locationy) * inv_bin_edge;  //
            real Tz = (xi.z - current_node_locationz) * inv_bin_edge;  //

            real valx = dN(Tx) * inv_bin_edge * N(Ty) * N(Tz);  //
            real valy = N(Tx) * dN(Ty) * inv_bin_edge * N(Tz);  //
            real valz = N(Tx) * N(Ty) * dN(Tz) * inv_bin_edge;  //

            Fe_hat_t[0] += vnx * valx; Fe_hat_t[1] += vny * valx; Fe_hat_t[2] += vnz * valx;  //
            Fe_hat_t[4] += vnx * valy; Fe_hat_t[5] += vny * valy; Fe_hat_t[6] += vnz * valy;  //
            Fe_hat_t[8] += vnx * valz; Fe_hat_t[9] += vny * valz; Fe_hat_t[10] += vnz * valz;

            //            real3 vel(grid_vel[current_node * 3 + 0], grid_vel[current_node * 3 + 1],
            //                      grid_vel[current_node * 3 + 2]);                  //
            //            real3 kern = dN(xi - current_node_location, inv_bin_edge);  //
            //            Fe_hat_t += OuterProduct(device_settings.dt * vel, kern);

            )

        marker_Fe_hat[p] = (Mat33(1.0) + device_settings.dt * Fe_hat_t) * marker_Fe[p];
    }
}

// CUDA_GLOBAL void kSVD(Mat33* marker_Fe_hat, Mat33* PolarR, Mat33* PolarS) {
//    const int p = blockIdx.x * blockDim.x + threadIdx.x;
//    if (p < device_settings.num_mpm_markers) {
//        Mat33 U, V, R, S, W;
//        real3 E;
//        SVD(marker_Fe_hat[p], U, E, V);
//        // Perform polar decomposition F = R*S
//        R = MultTranspose(U, V);
//        S = V * MultTranspose(Mat33(E), V);
//
//        PolarR[p] = R;
//        PolarS[p] = S;
//    }
//}

CUDA_GLOBAL void kApplyForces(const real3* sorted_pos,     // input
                              const Mat33* marker_Fe_hat,  // input
                              const Mat33* marker_Fe,      // input
                              const Mat33* marker_Fp,      // input
                              const real* marker_volume,   // input
                              const real* node_mass,       // input
                              Mat33* PolarR,               // input
                              Mat33* PolarS,               // input
                              real* grid_vel) {
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];
        Mat33 FE = marker_Fe[p];
        Mat33 FE_hat = marker_Fe_hat[p];
        Mat33 FP = marker_Fp[p];

        real JE = Determinant(marker_Fe[p]);  //
        real JP = Determinant(FP);
        real JE_hat = Determinant(FE_hat);

        real current_mu = device_settings.mu * Exp(device_settings.hardening_coefficient * (real(1.) - JP));
        Mat33 UE, VE;
        real3 EE;
        SVD(FE_hat, UE, EE, VE);
        /* Perform a polar decomposition, FE=RE*SE, RE is the Unitary part*/
        Mat33 RE = MultTranspose(UE, VE);
        PolarS[p] = VE * MultTranspose(Mat33(EE), VE);
        PolarR[p] = RE;
        PolarR[p][3] = JP;

        Mat33 vPEDFepT = device_settings.dt * marker_volume[p] * real(2.) * current_mu *
                         MultTranspose((FE_hat - RE), FE) * (real(1.0) / (JE * JP));

        int cx, cy, cz;
        const real bin_edge = device_settings.bin_edge;
        const real inv_bin_edge = real(1.0) / bin_edge;

        LOOP_TWO_RING_GPUSP(  //

            real Tx = (xi.x - current_node_locationx) * inv_bin_edge;  //
            real Ty = (xi.y - current_node_locationy) * inv_bin_edge;  //
            real Tz = (xi.z - current_node_locationz) * inv_bin_edge;  //

            real valx = dN(Tx) * inv_bin_edge * N(Ty) * N(Tz);  //
            real valy = N(Tx) * dN(Ty) * inv_bin_edge * N(Tz);  //
            real valz = N(Tx) * N(Ty) * dN(Tz) * inv_bin_edge;  //

            real fx = vPEDFepT[0] * valx + vPEDFepT[4] * valy + vPEDFepT[8] * valz;
            real fy = vPEDFepT[1] * valx + vPEDFepT[5] * valy + vPEDFepT[9] * valz;
            real fz = vPEDFepT[2] * valx + vPEDFepT[6] * valy + vPEDFepT[10] * valz;

            real mass = node_mass[current_node];  //
            if (mass > 0) {
                ATOMIC_ADD(&grid_vel[current_node * 3 + 0], -fx / mass);  //
                ATOMIC_ADD(&grid_vel[current_node * 3 + 1], -fy / mass);  //
                ATOMIC_ADD(&grid_vel[current_node * 3 + 2], -fz / mass);  //
            })
    }
}
CUDA_GLOBAL void kRhs(const real* node_mass,  // input
                      const real* grid_vel,
                      real* rhs) {
    const int current_node = blockIdx.x * blockDim.x + threadIdx.x;
    if (current_node < device_settings.num_mpm_nodes) {
        real mass = node_mass[current_node];  //
        if (mass > 0) {
            rhs[current_node * 3 + 0] = mass * grid_vel[current_node * 3 + 0];  //
            rhs[current_node * 3 + 1] = mass * grid_vel[current_node * 3 + 1];  //
            rhs[current_node * 3 + 2] = mass * grid_vel[current_node * 3 + 2];  //
        } else {
            rhs[current_node * 3 + 0] = 0;
            rhs[current_node * 3 + 1] = 0;
            rhs[current_node * 3 + 2] = 0;
        }
    }
}

CUDA_GLOBAL void kMultiplyA(const real3* sorted_pos,  // input
                            const real* v_array,
                            const real* old_vel_node_mpm,
                            const Mat33* PolarR,        // input
                            const Mat33* PolarS,        // input
                            const Mat33* marker_Fe,     // input
                            const real* marker_volume,  // input
                            real* result_array) {
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = sorted_pos[p];
        real VAP[9];
        real delta_F[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

        int cx, cy, cz;
        const real bin_edge = device_settings.bin_edge;
        const real inv_bin_edge = real(1.0) / bin_edge;

        LOOP_TWO_RING_GPUSP(  //

            real vnx = v_array[current_node * 3 + 0];  //
            real vny = v_array[current_node * 3 + 1];  //
            real vnz = v_array[current_node * 3 + 2];

            real Tx = (xi.x - current_node_locationx) * inv_bin_edge;  //
            real Ty = (xi.y - current_node_locationy) * inv_bin_edge;  //
            real Tz = (xi.z - current_node_locationz) * inv_bin_edge;  //

            real valx = dN(Tx) * inv_bin_edge * N(Ty) * N(Tz);  //
            real valy = N(Tx) * dN(Ty) * inv_bin_edge * N(Tz);  //
            real valz = N(Tx) * N(Ty) * dN(Tz) * inv_bin_edge;  //

            delta_F[0] += vnx * valx; delta_F[1] += vny * valx; delta_F[2] += vnz * valx;  //
            delta_F[3] += vnx * valy; delta_F[4] += vny * valy; delta_F[5] += vnz * valy;  //
            delta_F[6] += vnx * valz; delta_F[7] += vny * valz; delta_F[8] += vnz * valz;)

        Mat33 m_FE = marker_Fe[p];
        Mat33 R = PolarR[p];
        Mat33 S = PolarS[p];
        real plastic_determinant = R[3];  // Determinant(marker_Fp[p]);

        // delta_F = delta_F * m_FE;

        real current_mu = marker_volume[p] * 2 * device_settings.mu *
                          Exp(device_settings.hardening_coefficient * (real(1.0) - plastic_determinant));

        Vol_APFunc(S, R, delta_F, m_FE, current_mu, VAP);

        LOOP_TWO_RING_GPUSP(                                           //
            real Tx = (xi.x - current_node_locationx) * inv_bin_edge;  //
            real Ty = (xi.y - current_node_locationy) * inv_bin_edge;  //
            real Tz = (xi.z - current_node_locationz) * inv_bin_edge;  //

            real valx = dN(Tx) * inv_bin_edge * N(Ty) * N(Tz);  //
            real valy = N(Tx) * dN(Ty) * inv_bin_edge * N(Tz);  //
            real valz = N(Tx) * N(Ty) * dN(Tz) * inv_bin_edge;  //

            real resx = VAP[0] * valx + VAP[3] * valy + VAP[6] * valz;
            real resy = VAP[1] * valx + VAP[4] * valy + VAP[7] * valz;
            real resz = VAP[2] * valx + VAP[5] * valy + VAP[8] * valz;

            ATOMIC_ADD(&result_array[current_node * 3 + 0], resx);
            ATOMIC_ADD(&result_array[current_node * 3 + 1], resy);
            ATOMIC_ADD(&result_array[current_node * 3 + 2], resz););
    }
}
CUDA_GLOBAL void kMultiplyB(const real* v_array,
                            const real* old_vel_node_mpm,
                            const real* node_mass,
                            real* result_array) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < device_settings.num_mpm_nodes) {
        real mass = node_mass[i];
        if (mass > 0) {
            result_array[i * 3 + 0] += mass * (v_array[i * 3 + 0]);
            result_array[i * 3 + 1] += mass * (v_array[i * 3 + 1]);
            result_array[i * 3 + 2] += mass * (v_array[i * 3 + 2]);
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

void Multiply(gpu_vector<real>& input, gpu_vector<real>& output) {
    int size = input.size();

    kMultiplyA<<<CONFIG(size)>>>(pos.data_d,    // input
                                 input.data_d,  //
                                 old_vel_node_mpm.data_d,
                                 PolarR.data_d,         // input
                                 PolarS.data_d,         // input
                                 marker_Fe.data_d,      // input
                                 marker_volume.data_d,  // input
                                 output.data_d);

    kMultiplyB<<<CONFIG(size)>>>(input.data_d, old_vel_node_mpm.data_d, node_mass.data_d, output.data_d);
}

CUDA_GLOBAL void kSubtract(int size, real* x, real* y) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < size) {
        y[i] = y[i] - x[i];
    }
}

template <bool inner>
CUDA_GLOBAL void kResetGlobals() {
    if (inner) {
        dot_ms_ms = 0;
        dot_ms_my = 0;
        dot_my_my = 0;
    } else {
        alpha = 0.0001;
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
                ATOMIC_ADD(&dot_ms_ms, block_sum);
            }
        } else {
            data = my * my;
            block_sum = BlockReduce(temp_storage).Reduce(data, cub::Sum(), num_valid);
            if (threadIdx.x == 0) {
                ATOMIC_ADD(&dot_my_my, block_sum);
            }
        }
        __syncthreads();
        data = ms * my;
        block_sum = BlockReduce(temp_storage).Reduce(data, cub::Sum(), num_valid);

        if (threadIdx.x == 0) {
            ATOMIC_ADD(&dot_ms_my, block_sum);
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
    // printf("alpha: %f %f %f %f \n", alpha, dot_ms_ms, dot_ms_my, dot_my_my);
}

CUDA_GLOBAL void kCompute_ml_p(int num_items, real* ml, real* mg, real* ml_p) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < num_items) {
        ml_p[i] = ml[i] - alpha * mg[i];
        // printf("mlps : [%f %f %f]\n", ml_p[i], ml[i], mg[i]);
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
        data = mg[tid] * mg[tid];

        block_sum = BlockReduce(temp_storage).Reduce(data, cub::Sum(), num_valid);

        if (threadIdx.x == 0) {
            ATOMIC_ADD(&dot_g_proj_norm[0], block_sum);
        }
        // printf("resid [%f %f]\n", mg[tid], dot_g_proj_norm[0]);
    }
}

float time_no_shur = 0;
float time_shur = 0;
void MPM_BBSolver(gpu_vector<real>& r, gpu_vector<real>& delta_v) {
    time_shur = 0;
    time_no_shur = 0;
    const uint size = r.size();
    real lastgoodres = 10e30;
    {
        CudaEventTimer timer(start, stop, true, time_no_shur);

        dot_g_proj_norm.resize(1);
        ml.resize(size);
        mg.resize(size);
        mg_p.resize(size);
        ml_p.resize(size);
        ml = delta_v;
        mg = 0;
    }
    {
        CudaEventTimer timer(start, stop, true, time_shur);
        Multiply(ml, mg);
    }
    {
        CudaEventTimer timer(start, stop, true, time_no_shur);
        kSubtract<<<CONFIG(size)>>>(size, r.data_d, mg.data_d);
        mg_p = mg;
    }

    kResetGlobals<false><<<1, 1>>>();

    for (int current_iteration = 0; current_iteration < host_settings.num_iterations; current_iteration++) {
        {
            CudaEventTimer timer(start, stop, true, time_no_shur);
            kResetGlobals<true><<<1, 1>>>();
            kCompute_ml_p<<<CONFIG(size)>>>(size, ml.data_d, mg.data_d, ml_p.data_d);
            mg_p = 0;
        }
        {
            CudaEventTimer timer(start, stop, true, time_shur);
            Multiply(ml_p, mg_p);
        }
        {
            CudaEventTimer timer(start, stop, true, time_no_shur);
            kSubtract<<<CONFIG(size)>>>(size, r.data_d, mg_p.data_d);

            if (current_iteration % 2 == 0) {
                kUpdateAlpha<true><<<CONFIG(size)>>>(size, ml_p.data_d, ml.data_d, mg_p.data_d, mg.data_d);
                kAlpha<true><<<1, 1>>>();
            } else {
                kUpdateAlpha<false><<<CONFIG(size)>>>(size, ml_p.data_d, ml.data_d, mg_p.data_d, mg.data_d);
                kAlpha<false><<<1, 1>>>();
            }

            ml = ml_p;
            mg = mg_p;

            dot_g_proj_norm = 0;
            kResidual<<<CONFIG(size)>>>(size, mg.data_d, dot_g_proj_norm.data_d);
            dot_g_proj_norm.copyDeviceToHost();
            real g_proj_norm = Sqrt(dot_g_proj_norm.data_h[0]);

            if (g_proj_norm < lastgoodres) {
                lastgoodres = g_proj_norm;
                delta_v = ml;
            }
            //            printf("[%f]\n", lastgoodres);
        }
    }
    cudaCheck(cudaPeekAtLastError());
    cudaCheck(cudaDeviceSynchronize());
    printf("MPM Solver: [%f, %f %f] \n", time_no_shur, time_shur, lastgoodres);
}
CUDA_GLOBAL void kIncrementVelocity(real* delta_v, real* old_vel_node_mpm, real* grid_vel) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < device_settings.num_mpm_nodes) {
        grid_vel[i * 3 + 0] += delta_v[i * 3 + 0] - old_vel_node_mpm[i * 3 + 0];
        grid_vel[i * 3 + 1] += delta_v[i * 3 + 1] - old_vel_node_mpm[i * 3 + 1];
        grid_vel[i * 3 + 2] += delta_v[i * 3 + 2] - old_vel_node_mpm[i * 3 + 2];
    }
}

CUDA_GLOBAL void kUpdateParticleVelocity(real* grid_vel,
                                         real* old_vel_node_mpm,
                                         real3* pos_marker,
                                         real3* vel_marker,
                                         Mat33* marker_Fe,
                                         Mat33* marker_Fp) {
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = pos_marker[p];
        real3 V_flip = vel_marker[p];
        real3 V_pic = real3(0.0);

        const real bin_edge = device_settings.bin_edge;
        const real inv_bin_edge = real(1.0) / bin_edge;
        int cx, cy, cz;

        LOOP_TWO_RING_GPUSP(

            real weight = N((xi.x - current_node_locationx) * inv_bin_edge) *
                          N((xi.y - current_node_locationy) * inv_bin_edge) *
                          N((xi.z - current_node_locationz) * inv_bin_edge);

            real vnx = grid_vel[current_node * 3 + 0];  //
            real vny = grid_vel[current_node * 3 + 1];  //
            real vnz = grid_vel[current_node * 3 + 2];

            V_pic.x += vnx * weight;                                              //
            V_pic.y += vny * weight;                                              //
            V_pic.z += vnz * weight;                                              //
            V_flip.x += (vnx - old_vel_node_mpm[current_node * 3 + 0]) * weight;  //
            V_flip.y += (vny - old_vel_node_mpm[current_node * 3 + 1]) * weight;  //
            V_flip.z += (vnz - old_vel_node_mpm[current_node * 3 + 2]) * weight;  //
            )
        real3 new_vel = (1.0 - alpha) * V_pic + alpha * V_flip;

        real speed = Length(new_vel);
        if (speed > device_settings.max_velocity) {
            new_vel = new_vel * device_settings.max_velocity / speed;
        }
        vel_marker[p] = new_vel;
    }
}
CUDA_GLOBAL void kUpdateDeformationGradient(real* grid_vel, real3* pos_marker, Mat33* marker_Fe, Mat33* marker_Fp) {
    const int p = blockIdx.x * blockDim.x + threadIdx.x;
    if (p < device_settings.num_mpm_markers) {
        const real3 xi = pos_marker[p];
        Mat33 velocity_gradient(0.0);

        int cx, cy, cz;
        const real bin_edge = device_settings.bin_edge;
        const real inv_bin_edge = real(1.0) / bin_edge;

        LOOP_TWO_RING_GPUSP(real vnx = grid_vel[current_node * 3 + 0];  //
                            real vny = grid_vel[current_node * 3 + 1];  //
                            real vnz = grid_vel[current_node * 3 + 2];

                            real Tx = (xi.x - current_node_locationx) * inv_bin_edge;  //
                            real Ty = (xi.y - current_node_locationy) * inv_bin_edge;  //
                            real Tz = (xi.z - current_node_locationz) * inv_bin_edge;  //

                            real valx = dN(Tx) * inv_bin_edge * N(Ty) * N(Tz);  //
                            real valy = N(Tx) * dN(Ty) * inv_bin_edge * N(Tz);  //
                            real valz = N(Tx) * N(Ty) * dN(Tz) * inv_bin_edge;  //

                            velocity_gradient[0] += vnx * valx; velocity_gradient[1] += vny * valx;
                            velocity_gradient[2] += vnz * valx;  //
                            velocity_gradient[4] += vnx * valy; velocity_gradient[5] += vny * valy;
                            velocity_gradient[6] += vnz * valy;  //
                            velocity_gradient[8] += vnx * valz; velocity_gradient[9] += vny * valz;
                            velocity_gradient[10] += vnz * valz;

                            )
        Mat33 Fe_tmp = (Mat33(1.0) + device_settings.dt * velocity_gradient) * marker_Fe[p];
        Mat33 F_tmp = Fe_tmp * marker_Fp[p];
        Mat33 U, V;
        real3 E;
        SVD(Fe_tmp, U, E, V);
        real3 E_clamped;

        E_clamped.x = Clamp(E.x, 1.0 - device_settings.theta_c, 1.0 + device_settings.theta_s);
        E_clamped.y = Clamp(E.y, 1.0 - device_settings.theta_c, 1.0 + device_settings.theta_s);
        E_clamped.z = Clamp(E.z, 1.0 - device_settings.theta_c, 1.0 + device_settings.theta_s);

        marker_Fe[p] = U * MultTranspose(Mat33(E_clamped), V);
        // Inverse of Diagonal E_clamped matrix is 1/E_clamped
        marker_Fp[p] = V * MultTranspose(Mat33(1.0 / E_clamped), U) * F_tmp;
    }
}

void MPM_Solve(MPM_Settings& settings, std::vector<real3>& positions, std::vector<real3>& velocities) {
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    host_settings = settings;
    printf("Solving MPM: %d\n", host_settings.num_iterations);
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
    {
        CudaEventTimer timer(start, stop, true, time_measured);
        // ========================================================================================
        kRasterize<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,        // input
                                                              vel.data_d,        // input
                                                              node_mass.data_d,  // output
                                                              grid_vel.data_d    // output
                                                              );
    }
    printf("kRasterize: %f\n", time_measured);
    time_measured = 0;
    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kNormalizeWeights<<<CONFIG(host_settings.num_mpm_nodes)>>>(node_mass.data_d,  // output
                                                                   grid_vel.data_d);
    }
    printf("kNormalizeWeights: %f\n", time_measured);
    time_measured = 0;
    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kUpdateDeformationGradient<<<CONFIG(host_settings.num_mpm_markers)>>>(grid_vel.data_d, pos.data_d,
                                                                              marker_Fe.data_d, marker_Fp.data_d);
    }
    printf("kUpdateDeformationGradient: %f\n", time_measured);
    time_measured = 0;

    old_vel_node_mpm.resize(host_settings.num_mpm_nodes * 3);
    rhs.resize(host_settings.num_mpm_nodes * 3);
    old_vel_node_mpm = grid_vel;

    //    cudaCheck(cudaPeekAtLastError());
    //    cudaCheck(cudaDeviceSynchronize());
    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kFeHat<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d, marker_Fe.data_d, grid_vel.data_d,
                                                          marker_Fe_hat.data_d);
    }
    printf("kFeHat: %f\n", time_measured);
    time_measured = 0;
    // kSVD<<<CONFIG(host_settings.num_mpm_markers)>>>(marker_Fe_hat.data_d, PolarR.data_d, PolarS.data_d);
    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kApplyForces<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,            // input
                                                                marker_Fe_hat.data_d,  // input
                                                                marker_Fe.data_d,      // input
                                                                marker_Fp.data_d,      // input
                                                                marker_volume.data_d,  // input
                                                                node_mass.data_d,      // input
                                                                PolarR.data_d,         // output
                                                                PolarS.data_d,         // output
                                                                grid_vel.data_d);      // output
    }
    printf("kApplyForces: %f\n", time_measured);
    time_measured = 0;

    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kRhs<<<CONFIG(host_settings.num_mpm_nodes)>>>(node_mass.data_d, grid_vel.data_d, rhs.data_d);
    }
    printf("kRhs: %f\n", time_measured);
    time_measured = 0;

    delta_v.resize(host_settings.num_mpm_nodes * 3);
    delta_v = old_vel_node_mpm;

    MPM_BBSolver(rhs, delta_v);
    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kIncrementVelocity<<<CONFIG(host_settings.num_mpm_nodes)>>>(delta_v.data_d, old_vel_node_mpm.data_d,
                                                                    grid_vel.data_d);
    }
    printf("kIncrementVelocity: %f\n", time_measured);
    time_measured = 0;
    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kUpdateParticleVelocity<<<CONFIG(host_settings.num_mpm_markers)>>>(
            grid_vel.data_d, old_vel_node_mpm.data_d, pos.data_d, vel.data_d, marker_Fe.data_d, marker_Fp.data_d);
    }
    printf("kUpdateParticleVelocity: %f\n", time_measured);
    time_measured = 0;

    vel.copyDeviceToHost();

    velocities = vel.data_h;

    cudaEventDestroy(start);
    cudaEventDestroy(stop);
}

CUDA_GLOBAL void kInitFeFp(Mat33* marker_Fe, Mat33* marker_Fp) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < device_settings.num_mpm_markers) {
        marker_Fe[i] = Mat33(1);
        marker_Fp[i] = Mat33(1);
    }
}
void MPM_Update_Deformation_Gradient(MPM_Settings& settings, std::vector<real3>& velocities) {
    vel.data_h = velocities;
    vel.copyHostToDevice();
    host_settings = settings;

    cudaCheck(cudaMemcpyToSymbolAsync(device_settings, &host_settings, sizeof(MPM_Settings)));

    // ========================================================================================
    kRasterize<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,        // input
                                                          vel.data_d,        // input
                                                          node_mass.data_d,  // output
                                                          grid_vel.data_d    // output
                                                          );

    kNormalizeWeights<<<CONFIG(host_settings.num_mpm_nodes)>>>(node_mass.data_d,  // output
                                                               grid_vel.data_d);

    kUpdateDeformationGradient<<<CONFIG(host_settings.num_mpm_markers)>>>(grid_vel.data_d, pos.data_d, marker_Fe.data_d,
                                                                          marker_Fp.data_d);
    vel.copyDeviceToHost();
    velocities = vel.data_h;
}

void MPM_Initialize(MPM_Settings& settings, std::vector<real3>& positions) {
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

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

    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kRasterize<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,         // input
                                                              node_mass.data_d);  // output
    }
    printf("kRasterize: %f\n", time_measured);
    time_measured = 0;

    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kComputeParticleVolumes<<<CONFIG(host_settings.num_mpm_markers)>>>(pos.data_d,             // input
                                                                           node_mass.data_d,       // input
                                                                           marker_volume.data_d);  // output
    }

    printf("kComputeParticleVolumes: %f\n", time_measured);
    time_measured = 0;
    {
        CudaEventTimer timer(start, stop, true, time_measured);

        marker_Fe.resize(host_settings.num_mpm_markers);
        marker_Fe_hat.resize(host_settings.num_mpm_markers);
        marker_Fp.resize(host_settings.num_mpm_markers);
        marker_delta_F.resize(host_settings.num_mpm_markers);
        PolarR.resize(host_settings.num_mpm_markers);
        PolarS.resize(host_settings.num_mpm_markers);
    }
    printf("Resize: %f\n", time_measured);
    time_measured = 0;
    {
        CudaEventTimer timer(start, stop, true, time_measured);
        kInitFeFp<<<CONFIG(host_settings.num_mpm_markers)>>>(marker_Fe.data_d,   // output
                                                             marker_Fp.data_d);  // output
    }
    printf("kInitFeFp: %f\n", time_measured);
    time_measured = 0;
    //    cudaCheck(cudaPeekAtLastError());
    //    cudaCheck(cudaDeviceSynchronize());

    cudaEventDestroy(start);
    cudaEventDestroy(stop);
}
}
