// SCMRaycastGpuHost.cpp — HIP host bridge for the SCM ray-cast backend: device buffer management,
// synchronous upload/run (v1 -- see SCMRaycastGpu.h for why this isn't pipelined yet).
//
// Supports two kernel precisions (ScmRaycastGpuPrecision): FP64, the validated default, and FP32, added for GPUs with weak double-precision throughput --
// notably consumer NVIDIA cards (e.g. RTX 4080/5090), unlike this project's AMD MI300X target, a proper
// datacenter part with strong FP64. The public API types (SCMRaycastGpuTypes.h) stay double-precision
// throughout -- Chrono itself is double internally -- this file downcasts to float on upload and
// upconverts results back to double when precision == kFP32, so callers (SCMTerrainRaycastGpu.cpp)
// don't need to know or care which precision is active.
//
// Mesh geometry and per-body transforms are uploaded through SEPARATE calls: mesh (vertices/faces/
// margins) only changes when the candidate body set changes (rare -- never, in the common
// single-vehicle case), while transforms change every step. The caller (SCMTerrainRaycastGpu.cpp)
// caches the candidate set and only calls scm_raycast_gpu_upload_mesh when it actually changes.

#include "chrono_vehicle/terrain/SCMRaycastGpu.h"

#include <hip/hip_runtime.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

using chrono::vehicle::scm::gpu::RaycastBodyMargin;
using chrono::vehicle::scm::gpu::RaycastBodyTransform;
using chrono::vehicle::scm::gpu::RaycastFace;
using chrono::vehicle::scm::gpu::RaycastQuery;
using chrono::vehicle::scm::gpu::RaycastResult;
using chrono::vehicle::scm::gpu::RaycastVertex;

// Float mirrors of the double-precision public types. Only the memory layout needs to match the
// .hip.cpp kernel's own VertexDevT<float>/TransformDevT<float>/MarginDevT<float>/QueryDevT<float>/
// ResultDevT<float> -- not the C++ type identity -- since the two are compiled by different compilers
// (g++ here, hipcc there) and only ever communicate via raw device pointers.
struct VertexF {
    float x, y, z;
};
struct TransformF {
    float px, py, pz;
    float r00, r01, r02;
    float r10, r11, r12;
    float r20, r21, r22;
};
struct MarginF {
    float margin;
};
struct QueryF {
    float from_x, from_y, from_z;
    float to_x, to_y, to_z;
};
struct ResultF {
    int32_t hit;
    int32_t body_slot;
    float hit_x, hit_y, hit_z;
};

extern "C" int scm_launch_raycast_fp64(const void* queries_dev,
                                       int n_queries,
                                       const void* verts_dev,
                                       const void* faces_dev,
                                       int n_faces,
                                       const void* xforms_dev,
                                       const void* margins_dev,
                                       void* results_dev,
                                       hipStream_t stream);
extern "C" int scm_launch_raycast_fp32(const void* queries_dev,
                                       int n_queries,
                                       const void* verts_dev,
                                       const void* faces_dev,
                                       int n_faces,
                                       const void* xforms_dev,
                                       const void* margins_dev,
                                       void* results_dev,
                                       hipStream_t stream);

void die_hip(const char* msg, hipError_t err) {
    fprintf(stderr, "SCM RAYCAST GPU FATAL: %s -- %s\n", msg, hipGetErrorString(err));
    std::abort();
}

void ensure_device(int device) {
    int current = -1;
    hipGetDevice(&current);
    if (current != device)
        hipSetDevice(device);
}

template <typename T>
void ensure_capacity(T** d_ptr, std::size_t& capacity, std::size_t n) {
    if (n <= capacity)
        return;
    if (*d_ptr)
        (void)hipFree(*d_ptr);
    hipError_t e = hipMalloc(d_ptr, n * sizeof(T));
    if (e != hipSuccess)
        die_hip("hipMalloc", e);
    capacity = n;
}

}  // namespace

struct ScmRaycastGpuContext {
    int device = 0;
    ScmRaycastGpuPrecision precision = ScmRaycastGpuPrecision::kFP32;

    // FP64 device buffers (used when precision == kFP64).
    RaycastVertex* d_verts = nullptr;
    RaycastBodyTransform* d_xforms = nullptr;
    RaycastBodyMargin* d_margins = nullptr;
    RaycastQuery* d_queries = nullptr;
    RaycastResult* d_results = nullptr;
    std::size_t cap_verts = 0;
    std::size_t cap_bodies = 0;
    std::size_t cap_margins = 0;
    std::size_t cap_queries = 0;
    std::size_t cap_results = 0;

    // FP32 device buffers (used when precision == kFP32).
    VertexF* d_verts_f = nullptr;
    TransformF* d_xforms_f = nullptr;
    MarginF* d_margins_f = nullptr;
    QueryF* d_queries_f = nullptr;
    ResultF* d_results_f = nullptr;
    std::size_t cap_verts_f = 0;
    std::size_t cap_bodies_f = 0;
    std::size_t cap_margins_f = 0;
    std::size_t cap_queries_f = 0;
    std::size_t cap_results_f = 0;

    // Faces (indices + body_slot) are precision-independent -- always int32_t, one shared buffer.
    RaycastFace* d_faces = nullptr;
    std::size_t cap_faces = 0;

    int n_faces_current = 0;
};

extern "C" ScmRaycastGpuContext* scm_raycast_gpu_create(int device_id, ScmRaycastGpuPrecision precision) {
    auto* ctx = new ScmRaycastGpuContext();
    ctx->device = device_id;
    ctx->precision = precision;
    ensure_device(device_id);
    return ctx;
}

extern "C" void scm_raycast_gpu_destroy(ScmRaycastGpuContext* ctx) {
    if (!ctx)
        return;
    ensure_device(ctx->device);
    if (ctx->d_verts)
        hipFree(ctx->d_verts);
    if (ctx->d_faces)
        hipFree(ctx->d_faces);
    if (ctx->d_xforms)
        hipFree(ctx->d_xforms);
    if (ctx->d_margins)
        hipFree(ctx->d_margins);
    if (ctx->d_queries)
        hipFree(ctx->d_queries);
    if (ctx->d_results)
        hipFree(ctx->d_results);
    if (ctx->d_verts_f)
        hipFree(ctx->d_verts_f);
    if (ctx->d_xforms_f)
        hipFree(ctx->d_xforms_f);
    if (ctx->d_margins_f)
        hipFree(ctx->d_margins_f);
    if (ctx->d_queries_f)
        hipFree(ctx->d_queries_f);
    if (ctx->d_results_f)
        hipFree(ctx->d_results_f);
    delete ctx;
}

extern "C" int scm_raycast_gpu_upload_mesh(ScmRaycastGpuContext* ctx,
                                           const RaycastVertex* verts,
                                           int n_verts,
                                           const RaycastFace* faces,
                                           int n_faces,
                                           const RaycastBodyMargin* margins,
                                           int n_bodies) {
    if (!ctx)
        return -1;
    ensure_device(ctx->device);

    if (n_verts > 0) {
        if (ctx->precision == ScmRaycastGpuPrecision::kFP32) {
            std::vector<VertexF> tmp(n_verts);
            for (int i = 0; i < n_verts; ++i)
                tmp[i] = {static_cast<float>(verts[i].x), static_cast<float>(verts[i].y),
                         static_cast<float>(verts[i].z)};
            ensure_capacity(&ctx->d_verts_f, ctx->cap_verts_f, static_cast<std::size_t>(n_verts));
            hipError_t e = hipMemcpy(ctx->d_verts_f, tmp.data(), n_verts * sizeof(VertexF), hipMemcpyHostToDevice);
            if (e != hipSuccess)
                return static_cast<int>(e);
        } else {
            ensure_capacity(&ctx->d_verts, ctx->cap_verts, static_cast<std::size_t>(n_verts));
            hipError_t e = hipMemcpy(ctx->d_verts, verts, n_verts * sizeof(RaycastVertex), hipMemcpyHostToDevice);
            if (e != hipSuccess)
                return static_cast<int>(e);
        }
    }
    if (n_faces > 0) {
        ensure_capacity(&ctx->d_faces, ctx->cap_faces, static_cast<std::size_t>(n_faces));
        hipError_t e = hipMemcpy(ctx->d_faces, faces, n_faces * sizeof(RaycastFace), hipMemcpyHostToDevice);
        if (e != hipSuccess)
            return static_cast<int>(e);
    }
    if (n_bodies > 0) {
        if (ctx->precision == ScmRaycastGpuPrecision::kFP32) {
            std::vector<MarginF> tmp(n_bodies);
            for (int i = 0; i < n_bodies; ++i)
                tmp[i] = {static_cast<float>(margins[i].margin)};
            ensure_capacity(&ctx->d_margins_f, ctx->cap_margins_f, static_cast<std::size_t>(n_bodies));
            hipError_t e =
                hipMemcpy(ctx->d_margins_f, tmp.data(), n_bodies * sizeof(MarginF), hipMemcpyHostToDevice);
            if (e != hipSuccess)
                return static_cast<int>(e);
        } else {
            ensure_capacity(&ctx->d_margins, ctx->cap_margins, static_cast<std::size_t>(n_bodies));
            hipError_t e =
                hipMemcpy(ctx->d_margins, margins, n_bodies * sizeof(RaycastBodyMargin), hipMemcpyHostToDevice);
            if (e != hipSuccess)
                return static_cast<int>(e);
        }
    }

    ctx->n_faces_current = n_faces;
    return static_cast<int>(hipSuccess);
}

extern "C" int scm_raycast_gpu_upload_transforms(ScmRaycastGpuContext* ctx,
                                                 const RaycastBodyTransform* xforms,
                                                 int n_bodies) {
    if (!ctx)
        return -1;
    if (n_bodies <= 0)
        return 0;
    ensure_device(ctx->device);

    if (ctx->precision == ScmRaycastGpuPrecision::kFP32) {
        std::vector<TransformF> tmp(n_bodies);
        for (int i = 0; i < n_bodies; ++i) {
            const RaycastBodyTransform& t = xforms[i];
            tmp[i] = {static_cast<float>(t.px),  static_cast<float>(t.py),  static_cast<float>(t.pz),
                     static_cast<float>(t.r00), static_cast<float>(t.r01), static_cast<float>(t.r02),
                     static_cast<float>(t.r10), static_cast<float>(t.r11), static_cast<float>(t.r12),
                     static_cast<float>(t.r20), static_cast<float>(t.r21), static_cast<float>(t.r22)};
        }
        ensure_capacity(&ctx->d_xforms_f, ctx->cap_bodies_f, static_cast<std::size_t>(n_bodies));
        hipError_t e = hipMemcpy(ctx->d_xforms_f, tmp.data(), n_bodies * sizeof(TransformF), hipMemcpyHostToDevice);
        if (e != hipSuccess)
            return static_cast<int>(e);
    } else {
        ensure_capacity(&ctx->d_xforms, ctx->cap_bodies, static_cast<std::size_t>(n_bodies));
        hipError_t e =
            hipMemcpy(ctx->d_xforms, xforms, n_bodies * sizeof(RaycastBodyTransform), hipMemcpyHostToDevice);
        if (e != hipSuccess)
            return static_cast<int>(e);
    }

    return static_cast<int>(hipSuccess);
}

extern "C" int scm_raycast_gpu_run(ScmRaycastGpuContext* ctx,
                                   const RaycastQuery* queries,
                                   RaycastResult* out_results,
                                   int n_queries) {
    if (!ctx || !queries || !out_results)
        return -1;
    if (n_queries <= 0)
        return 0;

    ensure_device(ctx->device);

    if (ctx->precision == ScmRaycastGpuPrecision::kFP32) {
        std::vector<QueryF> q_tmp(n_queries);
        for (int i = 0; i < n_queries; ++i)
            q_tmp[i] = {static_cast<float>(queries[i].from_x), static_cast<float>(queries[i].from_y),
                       static_cast<float>(queries[i].from_z), static_cast<float>(queries[i].to_x),
                       static_cast<float>(queries[i].to_y),   static_cast<float>(queries[i].to_z)};

        ensure_capacity(&ctx->d_queries_f, ctx->cap_queries_f, static_cast<std::size_t>(n_queries));
        ensure_capacity(&ctx->d_results_f, ctx->cap_results_f, static_cast<std::size_t>(n_queries));

        hipError_t e1 = hipMemcpy(ctx->d_queries_f, q_tmp.data(), n_queries * sizeof(QueryF), hipMemcpyHostToDevice);
        if (e1 != hipSuccess)
            return static_cast<int>(e1);

        int launch_err = scm_launch_raycast_fp32(ctx->d_queries_f, n_queries, ctx->d_verts_f, ctx->d_faces,
                                                 ctx->n_faces_current, ctx->d_xforms_f, ctx->d_margins_f,
                                                 ctx->d_results_f, nullptr);
        if (launch_err != hipSuccess)
            return launch_err;

        hipError_t e2 = hipDeviceSynchronize();
        if (e2 != hipSuccess)
            return static_cast<int>(e2);

        std::vector<ResultF> r_tmp(n_queries);
        hipError_t e3 =
            hipMemcpy(r_tmp.data(), ctx->d_results_f, n_queries * sizeof(ResultF), hipMemcpyDeviceToHost);
        if (e3 != hipSuccess)
            return static_cast<int>(e3);

        for (int i = 0; i < n_queries; ++i) {
            out_results[i].hit = r_tmp[i].hit;
            out_results[i].body_slot = r_tmp[i].body_slot;
            out_results[i].hit_x = r_tmp[i].hit_x;
            out_results[i].hit_y = r_tmp[i].hit_y;
            out_results[i].hit_z = r_tmp[i].hit_z;
        }
        return static_cast<int>(hipSuccess);
    }

    ensure_capacity(&ctx->d_queries, ctx->cap_queries, static_cast<std::size_t>(n_queries));
    ensure_capacity(&ctx->d_results, ctx->cap_results, static_cast<std::size_t>(n_queries));

    hipError_t e1 = hipMemcpy(ctx->d_queries, queries, n_queries * sizeof(RaycastQuery), hipMemcpyHostToDevice);
    if (e1 != hipSuccess)
        return static_cast<int>(e1);

    int launch_err = scm_launch_raycast_fp64(ctx->d_queries, n_queries, ctx->d_verts, ctx->d_faces,
                                             ctx->n_faces_current, ctx->d_xforms, ctx->d_margins, ctx->d_results,
                                             nullptr);
    if (launch_err != hipSuccess)
        return launch_err;

    hipError_t e2 = hipDeviceSynchronize();
    if (e2 != hipSuccess)
        return static_cast<int>(e2);

    hipError_t e3 = hipMemcpy(out_results, ctx->d_results, n_queries * sizeof(RaycastResult), hipMemcpyDeviceToHost);
    if (e3 != hipSuccess)
        return static_cast<int>(e3);

    return static_cast<int>(hipSuccess);
}
