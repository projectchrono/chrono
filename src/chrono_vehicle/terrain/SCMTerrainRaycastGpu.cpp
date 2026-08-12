// SCMTerrainRaycastGpu.cpp — SCMLoader::ComputeRayCastGpuHip (HIP ray-cast backend host packing).
//
// Same I/O contract as ComputeRayCastGpuReference (SCMTerrain.cpp): produces a vector<RaycastHit>,
// consumed identically by ComputeInternalForces() regardless of which backend produced it. See
// SCM_RAYCAST_GPU_PLAN.md for the overall design and the convex-decomposition finding that explains
// why matching Bullet exactly isn't the goal.

#ifdef CHRONO_VEHICLE_SCM_GPU

    #include <cstdlib>
    #include <limits>
    #include <string>
    #include <unordered_map>
    #include <vector>

    #include <hip/hip_runtime.h>

    #include "chrono/physics/ChBody.h"
    #include "chrono/collision/ChCollisionShapeTriangleMesh.h"
    #include "chrono/geometry/ChTriangleMeshConnected.h"
    #include "chrono_vehicle/terrain/SCMTerrain.h"

    #include "chrono_vehicle/terrain/SCMRaycastGpu.h"

namespace chrono {
namespace vehicle {

namespace {

using scm::gpu::RaycastBodyMargin;
using scm::gpu::RaycastBodyTransform;
using scm::gpu::RaycastFace;
using scm::gpu::RaycastQuery;
using scm::gpu::RaycastResult;
using scm::gpu::RaycastVertex;

// Default precision is picked by which HIP platform this build targets: AMD (this project's validated
// MI300X target, strong FP64) gets kFP64; NVIDIA (consumer cards like RTX 4080/5090 deliberately
// throttle FP64 relative to FP32) gets kFP32. Override with env SCM_RAYCAST_GPU_PRECISION=fp32|fp64 --
// e.g. to validate the FP32 path on hardware that doesn't need it, or to force FP64 on NVIDIA if fidelity
// matters more than speed there. Read once (env doesn't change mid-run) and cached for the process.
ScmRaycastGpuPrecision DesiredRaycastGpuPrecision() {
    static ScmRaycastGpuPrecision precision = [] {
        ScmRaycastGpuPrecision p =
#if defined(__HIP_PLATFORM_NVIDIA__)
            ScmRaycastGpuPrecision::kFP32;
#else
            ScmRaycastGpuPrecision::kFP64;
#endif
        if (const char* e = std::getenv("SCM_RAYCAST_GPU_PRECISION")) {
            std::string mode = e;
            if (mode == "fp32")
                p = ScmRaycastGpuPrecision::kFP32;
            else if (mode == "fp64")
                p = ScmRaycastGpuPrecision::kFP64;
        }
        return p;
    }();
    return precision;
}

ScmRaycastGpuContext* RaycastGpuContext() {
    static ScmRaycastGpuContext* ctx = scm_raycast_gpu_create(0, DesiredRaycastGpuPrecision());
    return ctx;
}

// Mesh geometry (vertices/faces/margins) only changes when the candidate body SET changes -- which,
// in the common case (a fixed set of wheels tracked by explicit active domains), never happens after
// the first step. Cache the last-uploaded candidate set (by body pointer, order-sensitive since it
// determines body_slot) and skip the mesh re-upload -- and the CPU-side re-extraction that would
// otherwise precede it -- whenever it's unchanged. Only the small per-body transform is genuinely
// per-step data; see SCM_RAYCAST_GPU_PLAN.md.
std::vector<ChBody*>& LastUploadedCandidates() {
    static std::vector<ChBody*> last;
    return last;
}

// Local-space (body-relative) mesh extraction, appended into shared buffers with this body's vertex
// offset baked into its faces' indices. Unlike ComputeRayCastGpuReference's ExtractRaycastWorldTriangles,
// this does NOT transform to world space -- the kernel does that per-query, once per body per step,
// via the small per-body transform uploaded alongside.
void AppendLocalMesh(ChBody* body,
                     int body_slot,
                     std::vector<RaycastVertex>& verts,
                     std::vector<RaycastFace>& faces,
                     double& out_margin) {
    out_margin = 0;
    auto model = body->GetCollisionModel();
    if (!model)
        return;

    double envelope = model->GetEnvelope();

    for (const auto& inst : model->GetShapeInstances()) {
        if (inst.shape->GetType() != ChCollisionShape::TRIANGLEMESH)
            continue;

        auto mesh_shape = std::static_pointer_cast<ChCollisionShapeTriangleMesh>(inst.shape);
        auto trimesh = std::dynamic_pointer_cast<ChTriangleMeshConnected>(mesh_shape->GetMesh());
        if (!trimesh)
            continue;

        out_margin = envelope + mesh_shape->GetRadius();

        const auto& src_verts = trimesh->GetCoordsVertices();
        const auto& src_faces = trimesh->GetIndicesVertices();
        ChFrame<> shape_frame = inst.frame;

        int32_t local_vertex_offset = static_cast<int32_t>(verts.size());
        for (const auto& v : src_verts) {
            ChVector3d wv = shape_frame.TransformPointLocalToParent(v);  // shape-local -> model-local
            verts.push_back({wv.x(), wv.y(), wv.z()});
        }
        for (const auto& f : src_faces) {
            RaycastFace rf;
            rf.i0 = local_vertex_offset + f.x();
            rf.i1 = local_vertex_offset + f.y();
            rf.i2 = local_vertex_offset + f.z();
            rf.body_slot = body_slot;
            faces.push_back(rf);
        }
    }
}

RaycastBodyTransform BuildTransform(ChBody* body) {
    ChFrame<> f = body->GetFrameRefToAbs();
    ChVector3d p = f.GetPos();
    ChMatrix33<> R = f.GetRotMat();
    RaycastBodyTransform t;
    t.px = p.x();
    t.py = p.y();
    t.pz = p.z();
    t.r00 = R(0, 0);
    t.r01 = R(0, 1);
    t.r02 = R(0, 2);
    t.r10 = R(1, 0);
    t.r11 = R(1, 1);
    t.r12 = R(1, 2);
    t.r20 = R(2, 0);
    t.r21 = R(2, 1);
    t.r22 = R(2, 2);
    return t;
}

}  // namespace

void SCMLoader::ComputeRayCastGpuHip(std::vector<RaycastHit>& out_hits, int& num_ray_casts) {
    num_ray_casts = 0;

    std::vector<ChBody*> candidates;
    DiscoverRaycastCandidates(candidates);
    if (candidates.empty())
        return;

    ScmRaycastGpuContext* ctx = RaycastGpuContext();

    // Mesh (vertices/faces/margins) only re-extracted and re-uploaded when the candidate set actually
    // changed since last step -- see LastUploadedCandidates(). Common case: identical set every step,
    // so this CPU-side extraction and the associated H2D copy are skipped entirely after the first call.
    if (candidates != LastUploadedCandidates()) {
        std::vector<RaycastVertex> verts;
        std::vector<RaycastFace> faces;
        std::vector<RaycastBodyMargin> margins;
        margins.reserve(candidates.size());

        for (std::size_t i = 0; i < candidates.size(); ++i) {
            double margin = 0;
            AppendLocalMesh(candidates[i], static_cast<int>(i), verts, faces, margin);
            margins.push_back({margin});
        }

        if (faces.empty())
            return;

        int mesh_rc = scm_raycast_gpu_upload_mesh(ctx,
                                                  verts.data(),
                                                  static_cast<int>(verts.size()),
                                                  faces.data(),
                                                  static_cast<int>(faces.size()),
                                                  margins.data(),
                                                  static_cast<int>(candidates.size()));
        if (mesh_rc != 0)
            return;

        LastUploadedCandidates() = candidates;
    }

    // Per-body transform: genuinely per-step data, always rebuilt and uploaded.
    std::vector<RaycastBodyTransform> xforms;
    xforms.reserve(candidates.size());
    for (auto* body : candidates)
        xforms.push_back(BuildTransform(body));

    int xform_rc = scm_raycast_gpu_upload_transforms(ctx, xforms.data(), static_cast<int>(xforms.size()));
    if (xform_rc != 0)
        return;

    // Build the compacted query list (same RayOBBtest prefilter as the CPU paths).
    std::vector<RaycastQuery> queries;
    std::vector<ChVector2i> query_ij;

    for (auto& p : m_active_domains) {
        for (const auto& ij : p.m_range) {
            double x = ij.x() * m_delta;
            double y = ij.y() * m_delta;
            double z = GetHeight(ij);
            ChVector3d vertex_abs = m_frame.TransformPointLocalToParent(ChVector3d(x, y, z));
            ChVector3d to = vertex_abs + m_Z * m_test_offset_up;
            ChVector3d from = to - m_Z * m_test_offset_down;

            if (m_user_domains && !RayOBBtest(p, from, m_Z))
                continue;

            ++num_ray_casts;
            queries.push_back({from.x(), from.y(), from.z(), to.x(), to.y(), to.z()});
            query_ij.push_back(ij);
        }
    }

    if (queries.empty())
        return;

    std::vector<RaycastResult> results(queries.size());
    int run_rc = scm_raycast_gpu_run(ctx, queries.data(), results.data(), static_cast<int>(queries.size()));
    if (run_rc != 0)
        return;

    out_hits.reserve(out_hits.size() + results.size());
    for (std::size_t i = 0; i < results.size(); ++i) {
        if (!results[i].hit)
            continue;
        int slot = results[i].body_slot;
        if (slot < 0 || slot >= static_cast<int>(candidates.size()))
            continue;
        out_hits.push_back(
            {query_ij[i], candidates[slot], ChVector3d(results[i].hit_x, results[i].hit_y, results[i].hit_z)});
    }
}

}  // namespace vehicle
}  // namespace chrono

#endif  // CHRONO_VEHICLE_SCM_GPU
