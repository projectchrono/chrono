// SCMGpuKernels.hip.cpp — HIP kernels for SCM contact forces.

#include <hip/hip_runtime.h>

#include <cmath>
#include <cstring>

namespace {

struct SoilParamsDev {
    double bekker_kphi;
    double bekker_kc;
    double bekker_n;
    double mohr_cohesion;
    double mohr_mu;
    double janosi_shear;
    double elastic_k;
    double damping_r;
    double area;
    double dt;
};

struct HitInputDev {
    double level_initial;
    double level;
    double sinkage_plastic;
    double kshear;
    double sigma_yield;
    double normal_z;
    double hit_level;
    double patch_oob;
    double vn;
    double vt;
    double nx, ny, nz;
    double tx, ty, tz;
    double px, py, pz;
    double bx, by, bz;
    double c_cohesion;
    double c_mu;
    double c_janosi;
    double area_ratio;
    int body_id;
    int grid_ix;
    int grid_iy;
    int active;
};

struct HitOutputDev {
    double level;
    double sinkage;
    double sinkage_plastic;
    double sinkage_elastic;
    double sigma;
    double sigma_yield;
    double kshear;
    double tau;
    double step_plastic_flow;
    double fn_x, fn_y, fn_z;
    double ft_x, ft_y, ft_z;
    int active;
};

__device__ void scm_compute_one(const SoilParamsDev& s, const HitInputDev& in, HitOutputDev& out) {
    out = {};
    if (!in.active) {
        out.active = 0;
        return;
    }

    const double ca = in.normal_z;
    const double p_hit_offset = ca * (in.level_initial - in.hit_level);

    double sigma = s.elastic_k * (p_hit_offset - in.sinkage_plastic);
    if (sigma < 0.0) {
        out.active = 0;
        out.sigma = 0.0;
        return;
    }

    const double level = in.hit_level;
    const double sinkage = p_hit_offset;
    double sinkage_plastic = in.sinkage_plastic;
    double kshear = in.kshear;
    double sigma_yield = in.sigma_yield;

    kshear += in.vt * s.dt;

    double step_plastic_flow = 0.0;
    if (sigma > sigma_yield) {
        sigma = (in.patch_oob * s.bekker_kc + s.bekker_kphi) * pow(sinkage, s.bekker_n);
        sigma_yield = sigma;
        const double old_plastic = sinkage_plastic;
        sinkage_plastic = sinkage - sigma / s.elastic_k;
        step_plastic_flow = (sinkage_plastic - old_plastic) / s.dt;
    }

    const double sinkage_elastic = sinkage - sinkage_plastic;
    sigma += -in.vn * s.damping_r;

    const double tau_max = s.mohr_cohesion + sigma * s.mohr_mu;
    const double tau_soil = tau_max * (1.0 - exp(-(kshear / s.janosi_shear)));

    double tau = tau_soil;
    if (in.area_ratio > 0.0 && in.c_janosi > 0.0) {
        const double c_tau_max = in.c_cohesion + sigma * in.c_mu;
        const double c_tau = c_tau_max * (1.0 - exp(-(kshear / in.c_janosi)));
        tau = (1.0 - in.area_ratio) * tau_soil + in.area_ratio * c_tau;
    }

    const double fn_scale = s.area * sigma;
    const double ft_scale = s.area * tau;

    out.level = level;
    out.sinkage = sinkage;
    out.sinkage_plastic = sinkage_plastic;
    out.sinkage_elastic = sinkage_elastic;
    out.sigma = sigma;
    out.sigma_yield = sigma_yield;
    out.kshear = kshear;
    out.tau = tau;
    out.step_plastic_flow = step_plastic_flow;
    out.active = 1;

    out.fn_x = in.nx * fn_scale;
    out.fn_y = in.ny * fn_scale;
    out.fn_z = in.nz * fn_scale;
    out.ft_x = in.tx * ft_scale;
    out.ft_y = in.ty * ft_scale;
    out.ft_z = in.tz * ft_scale;
}

__global__ void scm_compute_forces_kernel(const SoilParamsDev soil,
                                          const HitInputDev* in,
                                          HitOutputDev* out,
                                          int n) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n)
        return;
    scm_compute_one(soil, in[i], out[i]);
}

__global__ void scm_reduce_body_forces_kernel(const HitInputDev* in,
                                              const HitOutputDev* out,
                                              double* body_forces,
                                              int n,
                                              int n_bodies) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n)
        return;
    if (!out[i].active)
        return;

    const int bid = in[i].body_id;
    if (bid < 0 || bid >= n_bodies)
        return;

    const double fx = out[i].fn_x + out[i].ft_x;
    const double fy = out[i].fn_y + out[i].ft_y;
    const double fz = out[i].fn_z + out[i].ft_z;

    const double rx = in[i].px - in[i].bx;
    const double ry = in[i].py - in[i].by;
    const double rz = in[i].pz - in[i].bz;

    const double mx = ry * fz - rz * fy;
    const double my = rz * fx - rx * fz;
    const double mz = rx * fy - ry * fx;

    double* bf = body_forces + static_cast<std::size_t>(bid) * 6;
    atomicAdd(bf + 0, fx);
    atomicAdd(bf + 1, fy);
    atomicAdd(bf + 2, fz);
    atomicAdd(bf + 3, mx);
    atomicAdd(bf + 4, my);
    atomicAdd(bf + 5, mz);
}

}  // namespace

extern "C" int scm_launch_compute_forces(const void* soil_host,
                                         const void* in_dev,
                                         void* out_dev,
                                         int n,
                                         hipStream_t stream) {
    if (n <= 0)
        return 0;

    SoilParamsDev soil_dev;
    memcpy(&soil_dev, soil_host, sizeof(SoilParamsDev));

    const int block = 256;
    const int grid = (n + block - 1) / block;
    scm_compute_forces_kernel<<<grid, block, 0, stream>>>(soil_dev,
                                                          static_cast<const HitInputDev*>(in_dev),
                                                          static_cast<HitOutputDev*>(out_dev),
                                                          n);
    return hipGetLastError();
}

extern "C" int scm_launch_reduce_body_forces(const void* in_dev,
                                             const void* out_dev,
                                             void* body_forces_dev,
                                             int n,
                                             int n_bodies,
                                             hipStream_t stream) {
    if (n <= 0 || n_bodies <= 0)
        return 0;

    const int block = 256;
    const int grid = (n + block - 1) / block;
    scm_reduce_body_forces_kernel<<<grid, block, 0, stream>>>(static_cast<const HitInputDev*>(in_dev),
                                                              static_cast<const HitOutputDev*>(out_dev),
                                                              static_cast<double*>(body_forces_dev),
                                                              n,
                                                              n_bodies);
    return hipGetLastError();
}
