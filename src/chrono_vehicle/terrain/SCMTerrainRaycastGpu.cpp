// SCMTerrainRaycastGpu.cpp — SCMLoader::ComputeRayCastGpuHip (HIP ray-cast backend host packing).
//
// Same I/O contract as ComputeRayCastGpuReference (SCMTerrain.cpp): produces a vector<RaycastHit>,
// consumed identically by ComputeInternalForces() regardless of which backend produced it. This does
// not reproduce Bullet's hits: on demo_ROBOT_Viper_SCM the wheel sinks ~5.5 cm here against ~4.3 cm
// through ChCollisionSystem::RayHit. The difference is in the formulation, not this implementation --
// ComputeRayCastGpuReference, which is the same algorithm on the CPU, lands within 0.09% of these
// kernels. Ruled out along the way: the contact-force backend (~2%), the collision margin (the
// envelope is 0 for these models, so the margin is just the 5 mm mesh thickness; zeroing it moves
// sinkage 4.7 mm, not 12), and kernel precision (FP32 vs FP64 is 3 um).

#ifdef CHRONO_VEHICLE_SCM_GPU

    #include <algorithm>
    #include <cstdlib>
    #include <iostream>
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

// Local AABB of a body's collision mesh, in the body's own frame. Constant for the life of the
// candidate set, so it is computed at upload and only transformed per step.
struct LocalBox {
    double x0, y0, z0, x1, y1, z1;
};

// FP32 on every platform, so a model produces the same trajectory on AMD and NVIDIA. The kernels can
// run in FP64 -- and MI300X-class parts have the throughput for it, unlike consumer NVIDIA cards where
// FP64 is deliberately throttled -- but making the default depend on the vendor would mean the same
// build of the same model diverging by hardware, which is worse than the precision itself. FP32 was
// validated against the CPU reference on AMD at zero hit difference and 0-0.12% per-wheel force error,
// and the ray cast feeds a soil model whose parameters are empirical to well under that. Override with
// env SCM_RAYCAST_GPU_PRECISION=fp32|fp64. Read once and cached for the process.
ScmRaycastGpuPrecision DesiredRaycastGpuPrecision() {
    static ScmRaycastGpuPrecision precision = [] {
        ScmRaycastGpuPrecision p = ScmRaycastGpuPrecision::kFP32;
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
// per-step data.
std::vector<ChBody*>& LastUploadedCandidates() {
    static std::vector<ChBody*> last;
    return last;
}

// Cached alongside the candidate set: each body's local AABB and its face range, both fixed until
// the candidate set changes.
std::vector<LocalBox>& CachedLocalBoxes() {
    static std::vector<LocalBox> boxes;
    return boxes;
}

// Local-space (body-relative) mesh extraction, appended into shared buffers with this body's vertex
// offset baked into its faces' indices. Unlike ComputeRayCastGpuReference's ExtractRaycastWorldTriangles,
// this does NOT transform to world space -- the kernel does that per-query, once per body per step,
// via the small per-body transform uploaded alongside.
void AppendLocalMesh(ChBody* body,
                     int body_slot,
                     std::vector<RaycastVertex>& verts,
                     std::vector<RaycastFace>& faces,
                     double& out_margin,
                     LocalBox& out_box) {
    out_margin = 0;
    out_box = {1e30, 1e30, 1e30, -1e30, -1e30, -1e30};
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
            out_box.x0 = std::min(out_box.x0, wv.x());
            out_box.y0 = std::min(out_box.y0, wv.y());
            out_box.z0 = std::min(out_box.z0, wv.z());
            out_box.x1 = std::max(out_box.x1, wv.x());
            out_box.y1 = std::max(out_box.y1, wv.y());
            out_box.z1 = std::max(out_box.z1, wv.z());
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

void WarnNoMeshGeometryOnce() {
    static bool warned = false;
    if (warned)
        return;
    warned = true;
    std::cerr << "SCM ray-cast: GPU backend requires triangle-mesh collision geometry; none found on the "
                 "active-domain bodies. Using the CPU ray-cast path." << std::endl;
}

RaycastBodyTransform BuildTransform(ChBody* body, const LocalBox& box) {
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

    // World AABB: transform the eight corners of the local box. Eight points per body per step is
    // nothing next to the per-triangle work it lets the kernel skip.
    double lo[3] = {1e30, 1e30, 1e30};
    double hi[3] = {-1e30, -1e30, -1e30};
    for (int c = 0; c < 8; ++c) {
        ChVector3d corner((c & 1) ? box.x1 : box.x0, (c & 2) ? box.y1 : box.y0, (c & 4) ? box.z1 : box.z0);
        ChVector3d w = f.TransformPointLocalToParent(corner);
        lo[0] = std::min(lo[0], w.x()); hi[0] = std::max(hi[0], w.x());
        lo[1] = std::min(lo[1], w.y()); hi[1] = std::max(hi[1], w.y());
        lo[2] = std::min(lo[2], w.z()); hi[2] = std::max(hi[2], w.z());
    }
    // Inflate slightly. The box is only a rejection test, so being generous costs a few extra
    // triangle tests, while being tight risks dropping a grazing hit once the bounds are rounded to
    // float for the FP32 kernel.
    const double eps = 1e-3;
    t.bx0 = lo[0] - eps; t.by0 = lo[1] - eps; t.bz0 = lo[2] - eps;
    t.bx1 = hi[0] + eps; t.by1 = hi[1] + eps; t.bz1 = hi[2] + eps;
    return t;
}

}  // namespace

bool SCMLoader::ComputeRayCastGpuHip(std::vector<RaycastHit>& out_hits, int& num_ray_casts) {
    num_ray_casts = 0;

    std::vector<ChBody*> candidates;
    DiscoverRaycastCandidates(candidates);
    if (candidates.empty())
        return false;

    ScmRaycastGpuContext* ctx = RaycastGpuContext();

    // Mesh (vertices/faces/margins) only re-extracted and re-uploaded when the candidate set actually
    // changed since last step -- see LastUploadedCandidates(). Common case: identical set every step,
    // so this CPU-side extraction and the associated H2D copy are skipped entirely after the first call.
    if (candidates != LastUploadedCandidates()) {
        std::vector<RaycastVertex> verts;
        std::vector<RaycastFace> faces;
        std::vector<RaycastBodyMargin> margins;
        margins.reserve(candidates.size());

        std::vector<LocalBox> boxes;
        boxes.reserve(candidates.size());
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            double margin = 0;
            LocalBox box{};
            const int face_begin = static_cast<int>(faces.size());
            AppendLocalMesh(candidates[i], static_cast<int>(i), verts, faces, margin, box);
            margins.push_back({margin, face_begin, static_cast<int>(faces.size())});
            boxes.push_back(box);
        }

        if (faces.empty()) {
            // No candidate body carries triangle-mesh collision geometry, so there is nothing for
            // these kernels to intersect. Models built from primitives or convex hulls (e.g. the
            // RoboSimian limbs and sled) land here. Report once and let the caller use the CPU path,
            // which tests against the full collision system and handles every shape type.
            WarnNoMeshGeometryOnce();
            return false;
        }

        int mesh_rc = scm_raycast_gpu_upload_mesh(ctx,
                                                  verts.data(),
                                                  static_cast<int>(verts.size()),
                                                  faces.data(),
                                                  static_cast<int>(faces.size()),
                                                  margins.data(),
                                                  static_cast<int>(candidates.size()));
        if (mesh_rc != 0)
            return false;

        LastUploadedCandidates() = candidates;
        CachedLocalBoxes() = boxes;
    }

    // Per-body transform: genuinely per-step data, always rebuilt and uploaded.
    std::vector<RaycastBodyTransform> xforms;
    xforms.reserve(candidates.size());
    const auto& boxes = CachedLocalBoxes();
    if (boxes.size() != candidates.size())
        return false;
    for (std::size_t i = 0; i < candidates.size(); ++i)
        xforms.push_back(BuildTransform(candidates[i], boxes[i]));

    int xform_rc = scm_raycast_gpu_upload_transforms(ctx, xforms.data(), static_cast<int>(xforms.size()));
    if (xform_rc != 0)
        return false;

    // Footprint of each candidate body in the SCM plane. A ray is cast along the SCM normal, so in
    // SCM-local coordinates it is exactly vertical and a node outside every body's (x,y) footprint
    // cannot produce a hit. Testing that first keeps the ray budget proportional to the geometry
    // instead of to the active domain the user happened to declare -- with obstacles the two differ
    // by an order of magnitude, and every wasted node costs a height lookup, a query slot, a device
    // copy and a block launch.
    struct Footprint {
        double x0, y0, x1, y1;
    };
    std::vector<Footprint> footprints;
    footprints.reserve(candidates.size());
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const LocalBox& lb = boxes[i];
        ChFrame<> bf = candidates[i]->GetFrameRefToAbs();
        Footprint fp{1e30, 1e30, -1e30, -1e30};
        for (int c = 0; c < 8; ++c) {
            ChVector3d corner((c & 1) ? lb.x1 : lb.x0, (c & 2) ? lb.y1 : lb.y0, (c & 4) ? lb.z1 : lb.z0);
            ChVector3d loc = m_frame.TransformPointParentToLocal(bf.TransformPointLocalToParent(corner));
            fp.x0 = std::min(fp.x0, loc.x());
            fp.y0 = std::min(fp.y0, loc.y());
            fp.x1 = std::max(fp.x1, loc.x());
            fp.y1 = std::max(fp.y1, loc.y());
        }
        footprints.push_back(fp);
    }

    // Build the compacted query list (same RayOBBtest prefilter as the CPU paths).
    std::vector<RaycastQuery> queries;
    std::vector<ChVector2i> query_ij;

    for (auto& p : m_active_domains) {
        for (const auto& ij : p.m_range) {
            double x = ij.x() * m_delta;
            double y = ij.y() * m_delta;

            bool in_footprint = false;
            for (const auto& fp : footprints) {
                if (x >= fp.x0 && x <= fp.x1 && y >= fp.y0 && y <= fp.y1) {
                    in_footprint = true;
                    break;
                }
            }
            if (!in_footprint)
                continue;

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
        return false;

    std::vector<RaycastResult> results(queries.size());
    int run_rc = scm_raycast_gpu_run(ctx, queries.data(), results.data(), static_cast<int>(queries.size()));
    if (run_rc != 0)
        return false;

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

    return true;
}

}  // namespace vehicle
}  // namespace chrono

#endif  // CHRONO_VEHICLE_SCM_GPU
