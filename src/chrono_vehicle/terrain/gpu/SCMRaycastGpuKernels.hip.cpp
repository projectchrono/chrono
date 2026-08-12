// SCMRaycastGpuKernels.hip.cpp — HIP kernel for the SCM GPU ray-cast backend.
//
// One thread BLOCK per query (SCM grid node ray), not one thread. Threads within a block split the
// triangle scan (each thread handles a strided subset), transform each triangle from local to world
// space using its body's transform, and each keeps its own local nearest segment-triangle intersection
// -- the same "closest hit to from" semantics as Bullet's ClosestRayResultCallback (see
// SCMLoader::ComputeRayCastGpuReference's comment block in SCMTerrain.cpp for why that's the right
// semantics for SCM's ray window). A block-wide reduction over shared memory then picks the true
// nearest hit for the ray. This is the v2 kernel design: v1 (one thread per query, serial triangle scan
// inside each thread) was measured at only ~1.55-1.9x overall speedup vs CPU/Bullet -- ~2400-3400
// threads total is a small fraction of the concurrency a GPU like the MI300X wants, so most of the
// chip sat idle. Spreading triangles across kThreadsPerRay threads per ray turns that into
// ~2400-3400 x kThreadsPerRay threads, without changing the total amount of work
// (queries x triangles) or the algorithm itself.
//
// Templated on Real (double or float) so the same kernel logic runs at either precision. double is the
// validated default for this project's AMD MI300X target. float is offered for GPUs with weak double-precision throughput -- notably consumer NVIDIA
// cards (RTX 4080/5090-class), where FP64 is deliberately throttled relative to FP32 (unlike MI300X, a
// proper datacenter part) -- so a straight double-precision port would be correct there but far slower
// than it needs to be. Both precisions are exported (scm_launch_raycast_fp64 / _fp32); the host bridge
// (SCMRaycastGpuHost.cpp) selects one per-context based on ScmRaycastGpuPrecision, which
// SCMTerrainRaycastGpu.cpp defaults by which HIP platform (AMD vs NVIDIA) this build targets.
//
// Includes the empirically-determined margin-correction sign; an exact match to Bullet's hit set is
// not the goal.

#include <hip/hip_runtime.h>

#include <cstdint>

namespace {

template <typename Real>
struct Vec3 {
    Real x, y, z;
};

template <typename Real>
struct VertexDevT {
    Real x, y, z;
};

// Precision-independent: indices and body_slot are always int32_t.
struct FaceDev {
    int32_t i0, i1, i2;
    int32_t body_slot;
};

template <typename Real>
struct TransformDevT {
    Real px, py, pz;
    Real r00, r01, r02;
    Real r10, r11, r12;
    Real r20, r21, r22;
};

template <typename Real>
struct MarginDevT {
    Real margin;
};

template <typename Real>
struct QueryDevT {
    Real from_x, from_y, from_z;
    Real to_x, to_y, to_z;
};

template <typename Real>
struct ResultDevT {
    int32_t hit;
    int32_t body_slot;
    Real hit_x, hit_y, hit_z;
};

template <typename Real>
__device__ __forceinline__ Vec3<Real> make_v3(Real x, Real y, Real z) {
    Vec3<Real> v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

template <typename Real>
__device__ __forceinline__ Vec3<Real> operator-(Vec3<Real> a, Vec3<Real> b) {
    return make_v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

template <typename Real>
__device__ __forceinline__ Vec3<Real> operator+(Vec3<Real> a, Vec3<Real> b) {
    return make_v3(a.x + b.x, a.y + b.y, a.z + b.z);
}

template <typename Real>
__device__ __forceinline__ Vec3<Real> operator*(Vec3<Real> a, Real s) {
    return make_v3(a.x * s, a.y * s, a.z * s);
}

template <typename Real>
__device__ __forceinline__ Real dot3(Vec3<Real> a, Vec3<Real> b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename Real>
__device__ __forceinline__ Vec3<Real> cross3(Vec3<Real> a, Vec3<Real> b) {
    return make_v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

template <typename Real>
__device__ __forceinline__ Vec3<Real> transform_point(const TransformDevT<Real>& t, Vec3<Real> p) {
    return make_v3(t.r00 * p.x + t.r01 * p.y + t.r02 * p.z + t.px, t.r10 * p.x + t.r11 * p.y + t.r12 * p.z + t.py,
                   t.r20 * p.x + t.r21 * p.y + t.r22 * p.z + t.pz);
}

// Segment-vs-triangle intersection (Moller-Trumbore, clamped to t in [0,1] along the segment). Mirrors
// SCMTerrain.cpp's RaycastSegmentTriangleIntersect exactly (Real controls only the floating-point
// precision used to evaluate it, not the algorithm).
template <typename Real>
__device__ bool segment_triangle_intersect(Vec3<Real> from,
                                           Vec3<Real> dir,
                                           Vec3<Real> v0,
                                           Vec3<Real> v1,
                                           Vec3<Real> v2,
                                           Real& t_out,
                                           Vec3<Real>& point_out,
                                           Vec3<Real>& normal_out) {
    const Real eps = static_cast<Real>(1e-9);
    Vec3<Real> edge1 = v1 - v0;
    Vec3<Real> edge2 = v2 - v0;
    Vec3<Real> h = cross3(dir, edge2);
    Real a = dot3(edge1, h);
    if (fabs(a) < eps)
        return false;
    Real f = Real(1.0) / a;
    Vec3<Real> s = from - v0;
    Real u = f * dot3(s, h);
    if (u < Real(0.0) || u > Real(1.0))
        return false;
    Vec3<Real> q = cross3(s, edge1);
    Real v = f * dot3(dir, q);
    if (v < Real(0.0) || u + v > Real(1.0))
        return false;
    Real t = f * dot3(edge2, q);
    if (t < Real(0.0) || t > Real(1.0))
        return false;

    t_out = t;
    point_out = from + dir * t;

    Vec3<Real> n = cross3(edge1, edge2);
    Real n_len = sqrt(dot3(n, n));
    if (n_len > eps)
        n = n * (Real(1.0) / n_len);
    normal_out = (dot3(n, dir) > Real(0.0)) ? n * Real(-1.0) : n;
    return true;
}

constexpr int kThreadsPerRay = 256;  // 8 NVIDIA warps / 4 AMD wavefronts; shared-mem reduction below

template <typename Real>
__global__ void scm_raycast_kernel(const QueryDevT<Real>* queries,
                                   int n_queries,
                                   const VertexDevT<Real>* verts,
                                   const FaceDev* faces,
                                   int n_faces,
                                   const TransformDevT<Real>* xforms,
                                   const MarginDevT<Real>* margins,
                                   ResultDevT<Real>* results) {
    const int ray = blockIdx.x;  // one block per query
    if (ray >= n_queries)
        return;
    const int tid = threadIdx.x;

    __shared__ Real s_t[kThreadsPerRay];
    __shared__ Real s_px[kThreadsPerRay], s_py[kThreadsPerRay], s_pz[kThreadsPerRay];
    __shared__ Real s_nx[kThreadsPerRay], s_ny[kThreadsPerRay], s_nz[kThreadsPerRay];
    __shared__ int s_body[kThreadsPerRay];

    QueryDevT<Real> q = queries[ray];
    Vec3<Real> from = make_v3<Real>(q.from_x, q.from_y, q.from_z);
    Vec3<Real> to = make_v3<Real>(q.to_x, q.to_y, q.to_z);
    Vec3<Real> dir = to - from;

    Real best_t = static_cast<Real>(1e30);
    int best_body = -1;
    Vec3<Real> best_point = make_v3<Real>(0, 0, 0);
    Vec3<Real> best_normal = make_v3<Real>(0, 0, 0);

    // Each thread scans a strided subset of the triangles -- total work (n_faces) is unchanged, just
    // spread across kThreadsPerRay threads instead of done serially by one.
    for (int f = tid; f < n_faces; f += kThreadsPerRay) {
        FaceDev face = faces[f];
        const TransformDevT<Real>& xf = xforms[face.body_slot];
        Vec3<Real> v0 = transform_point(xf, make_v3<Real>(verts[face.i0].x, verts[face.i0].y, verts[face.i0].z));
        Vec3<Real> v1 = transform_point(xf, make_v3<Real>(verts[face.i1].x, verts[face.i1].y, verts[face.i1].z));
        Vec3<Real> v2 = transform_point(xf, make_v3<Real>(verts[face.i2].x, verts[face.i2].y, verts[face.i2].z));

        Real t;
        Vec3<Real> pt, n;
        if (segment_triangle_intersect(from, dir, v0, v1, v2, t, pt, n) && t < best_t) {
            best_t = t;
            best_body = face.body_slot;
            best_point = pt;
            best_normal = n;
        }
    }

    s_t[tid] = best_t;
    s_px[tid] = best_point.x;
    s_py[tid] = best_point.y;
    s_pz[tid] = best_point.z;
    s_nx[tid] = best_normal.x;
    s_ny[tid] = best_normal.y;
    s_nz[tid] = best_normal.z;
    s_body[tid] = best_body;
    __syncthreads();

    // Block-wide reduction to the true nearest hit for this ray. Simple pairwise halving (not the
    // full wavefront-shuffle-optimized form) -- kThreadsPerRay is small (256) and this runs once per
    // ray, not per triangle, so it's not the dominant cost; revisit only if profiling says otherwise.
    for (int stride = kThreadsPerRay / 2; stride > 0; stride >>= 1) {
        if (tid < stride && s_t[tid + stride] < s_t[tid]) {
            s_t[tid] = s_t[tid + stride];
            s_px[tid] = s_px[tid + stride];
            s_py[tid] = s_py[tid + stride];
            s_pz[tid] = s_pz[tid + stride];
            s_nx[tid] = s_nx[tid + stride];
            s_ny[tid] = s_ny[tid + stride];
            s_nz[tid] = s_nz[tid + stride];
            s_body[tid] = s_body[tid + stride];
        }
        __syncthreads();
    }

    if (tid != 0)
        return;

    ResultDevT<Real> out;
    if (s_body[0] >= 0) {
        // Empirically-determined margin-correction sign: moves the raw
        // intersection toward shallower sinkage, matching the CPU reference / Bullet comparison.
        Real margin = margins[s_body[0]].margin;
        Vec3<Real> point = make_v3<Real>(s_px[0], s_py[0], s_pz[0]);
        Vec3<Real> normal = make_v3<Real>(s_nx[0], s_ny[0], s_nz[0]);
        Vec3<Real> corrected = point + normal * margin;
        out.hit = 1;
        out.body_slot = s_body[0];
        out.hit_x = corrected.x;
        out.hit_y = corrected.y;
        out.hit_z = corrected.z;
    } else {
        out.hit = 0;
        out.body_slot = -1;
        out.hit_x = out.hit_y = out.hit_z = 0;
    }
    results[ray] = out;
}

template <typename Real>
int launch(const void* queries_dev,
          int n_queries,
          const void* verts_dev,
          const void* faces_dev,
          int n_faces,
          const void* xforms_dev,
          const void* margins_dev,
          void* results_dev,
          hipStream_t stream) {
    if (n_queries <= 0)
        return 0;

    const int grid = n_queries;        // one block per ray
    const int block = kThreadsPerRay;  // threads split the triangle scan for that ray
    scm_raycast_kernel<Real><<<grid, block, 0, stream>>>(static_cast<const QueryDevT<Real>*>(queries_dev), n_queries,
                                                         static_cast<const VertexDevT<Real>*>(verts_dev),
                                                         static_cast<const FaceDev*>(faces_dev), n_faces,
                                                         static_cast<const TransformDevT<Real>*>(xforms_dev),
                                                         static_cast<const MarginDevT<Real>*>(margins_dev),
                                                         static_cast<ResultDevT<Real>*>(results_dev));
    return hipGetLastError();
}

}  // namespace

extern "C" int scm_launch_raycast_fp64(const void* queries_dev,
                                       int n_queries,
                                       const void* verts_dev,
                                       const void* faces_dev,
                                       int n_faces,
                                       const void* xforms_dev,
                                       const void* margins_dev,
                                       void* results_dev,
                                       hipStream_t stream) {
    return launch<double>(queries_dev, n_queries, verts_dev, faces_dev, n_faces, xforms_dev, margins_dev, results_dev,
                          stream);
}

extern "C" int scm_launch_raycast_fp32(const void* queries_dev,
                                       int n_queries,
                                       const void* verts_dev,
                                       const void* faces_dev,
                                       int n_faces,
                                       const void* xforms_dev,
                                       const void* margins_dev,
                                       void* results_dev,
                                       hipStream_t stream) {
    return launch<float>(queries_dev, n_queries, verts_dev, faces_dev, n_faces, xforms_dev, margins_dev, results_dev,
                         stream);
}
