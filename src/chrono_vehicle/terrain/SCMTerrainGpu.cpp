// SCMTerrainGpu.cpp -- SCM GPU backend selection and SCMLoader::ComputeContactForcesGpu (GPU body reduce,
// grid-only scatter). Compiled in every build; without GPU support (CHRONO_VEHICLE_SCM_GPU undefined) the GPU
// entry points are stubs that decline, so SCMTerrain uses its CPU paths.

#include <cstdio>
#include <unordered_map>
#include <vector>

#include "chrono_vehicle/terrain/SCMTerrain.h"

#ifdef CHRONO_VEHICLE_SCM_GPU
    #include <chrono>
    #include <mutex>

    #include "chrono/physics/ChBody.h"
    #include "chrono_vehicle/terrain/SCMGpu.h"
#endif

namespace chrono {
namespace vehicle {

namespace {

// Report once per process that the GPU was requested explicitly (Backend::GPU) but cannot be used.
void WarnGpuUnavailableOnce(const char* reason) {
    static bool warned = false;
    if (!warned) {
        warned = true;
        std::fprintf(stderr, "SCMTerrain: GPU backend requested but %s; using the CPU implementation.\n", reason);
    }
}

}  // namespace

bool SCMLoader::GpuAllowed(SCMTerrain::Backend backend) const {
    if (backend == SCMTerrain::Backend::CPU)
        return false;
    if (SCMTerrain::IsGpuAvailable())
        return true;
    if (backend == SCMTerrain::Backend::GPU) {
#ifdef CHRONO_VEHICLE_SCM_GPU
        WarnGpuUnavailableOnce("no usable GPU device was found");
#else
        WarnGpuUnavailableOnce("Chrono was built without SCM GPU support");
#endif
    }
    return false;
}

#ifdef CHRONO_VEHICLE_SCM_GPU

namespace {

// The contact-force GPU context (device buffers and streams) is shared by all SCM terrains in the process.
// A single step uses its buffers from packing through scatter, so whole steps are serialized.
std::mutex& GpuMutex() {
    static std::mutex mtx;
    return mtx;
}

// Null when no usable device is present (see scm_gpu_device_available); callers then use the CPU path.
// The first caller's settings size the initial buffers; buffers grow on demand afterwards.
ScmGpuContext* GpuContext(const SCMTerrain::GpuSettings& settings) {
    static ScmGpuContext* ctx = [&settings] {
        ScmGpuContext* c = scm_gpu_create(0);
        if (c)
            scm_gpu_warmup(c, settings.reserve_hits, settings.async ? 1 : 0);
        return c;
    }();
    return ctx;
}

int32_t BodySlot(std::vector<ChBody*>& bodies, ChBody* body) {
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i] == body)
            return static_cast<int32_t>(i);
    }
    bodies.push_back(body);
    return static_cast<int32_t>(bodies.size() - 1);
}

}  // namespace

bool SCMTerrain::IsGpuAvailable() {
    return scm_gpu_device_available() != 0;
}

void SCMLoader::PrimeGpuBuffers() {
    if (m_contact_force_backend == SCMTerrain::Backend::CPU || !SCMTerrain::IsGpuAvailable())
        return;
    std::lock_guard<std::mutex> lock(GpuMutex());
    if (ScmGpuContext* ctx = GpuContext(m_gpu_settings))
        scm_gpu_reserve(ctx, m_gpu_settings.reserve_hits);
}

bool SCMLoader::ComputeContactForcesGpu(const std::unordered_map<ChVector2i, ScmHitRecord, CoordHash>& hits, const std::vector<double>& patch_oob) {
    using Clock = std::chrono::steady_clock;

    // No usable device: decline before anything else, so the caller runs the CPU path and does not
    // count this step as a GPU step.
    if (!scm_gpu_device_available())
        return false;

    const std::size_t n = hits.size();
    if (n == 0)
        return true;

    std::lock_guard<std::mutex> lock(GpuMutex());

    ScmGpuContext* ctx = GpuContext(m_gpu_settings);
    if (!ctx)
        return false;

    const bool profile = m_gpu_settings.profile;
    const auto t0 = Clock::now();

    scm::gpu::HitInput* batch_in = scm_gpu_prepare_input(ctx, n);
    scm::gpu::HitOutput* batch_out = scm_gpu_prepare_output(ctx, n);
    if (!batch_in || !batch_out)
        return false;

    std::vector<ChBody*> bodies;
    bodies.reserve(64);

    scm::gpu::SoilParams soil{};
    soil.bekker_kphi = m_Bekker_Kphi;
    soil.bekker_kc = m_Bekker_Kc;
    soil.bekker_n = m_Bekker_n;
    soil.mohr_cohesion = m_Mohr_cohesion;
    soil.mohr_mu = m_Mohr_mu;
    soil.janosi_shear = m_Janosi_shear;
    soil.elastic_k = m_elastic_K;
    soil.damping_r = m_damping_R;
    soil.area = m_area;
    soil.dt = GetSystem()->GetStep();

    std::size_t idx = 0;
    for (const auto& h : hits) {
        auto* body = dynamic_cast<ChBody*>(h.second.contactable);
        if (!body)
            return false;

        const ChVector2i& ij = h.first;
        const int32_t bid = BodySlot(bodies, body);

        auto& nr = m_grid_map.at(ij);
        const double ca = nr.normal.z();

        const auto hit_point_loc = m_frame.TransformPointParentToLocal(h.second.abs_point);

        const ChVector3d point_local(ij.x() * m_delta, ij.y() * m_delta, nr.level);
        const ChVector3d point_abs = m_frame.TransformPointLocalToParent(point_local);
        const ChVector3d speed_abs = h.second.contactable->GetContactPointSpeed(point_abs);

        ChVector3d N = m_frame.TransformDirectionLocalToParent(nr.normal);
        const double Vn = Vdot(speed_abs, N);
        ChVector3d T = -(speed_abs - Vn * N);
        T.Normalize();

        double patch_oob_val = 0.0;
        if (h.second.patch_id >= 0 && h.second.patch_id < static_cast<int>(patch_oob.size()))
            patch_oob_val = patch_oob[static_cast<std::size_t>(h.second.patch_id)];

        scm::gpu::HitInput& in = batch_in[idx];
        in = {};
        in.level_initial = nr.level_initial;
        in.level = nr.level;
        in.sinkage_plastic = nr.sinkage_plastic;
        in.kshear = nr.kshear;
        in.sigma_yield = nr.sigma_yield;
        in.normal_z = ca;
        in.hit_level = hit_point_loc.z();
        in.patch_oob = patch_oob_val;
        in.vn = Vn;
        in.vt = Vdot(speed_abs, -T);
        in.nx = N.x();
        in.ny = N.y();
        in.nz = N.z();
        in.tx = T.x();
        in.ty = T.y();
        in.tz = T.z();
        in.px = point_abs.x();
        in.py = point_abs.y();
        in.pz = point_abs.z();
        in.bx = body->GetPos().x();
        in.by = body->GetPos().y();
        in.bz = body->GetPos().z();
        in.body_id = bid;
        in.grid_ix = ij.x();
        in.grid_iy = ij.y();
        in.active = 1;

        if (auto cprops = body->GetUserData<SCMContactableData>()) {
            in.c_cohesion = cprops->Mohr_cohesion;
            in.c_mu = cprops->Mohr_mu;
            in.c_janosi = cprops->Janosi_shear;
            in.area_ratio = cprops->area_ratio;
        }

        ++idx;
    }

    const auto t1 = Clock::now();
    const std::size_t n_bodies = bodies.size();

    scm::gpu::BodyForceAccum* body_forces = scm_gpu_prepare_body_forces(ctx, n_bodies);
    if (!body_forces)
        return false;

    const int gpu_rc = scm_gpu_compute_forces_staged(ctx, soil, n, n_bodies, m_gpu_settings.async ? 1 : 0);
    if (gpu_rc != 0) {
        std::fprintf(stderr, "SCM GPU: scm_gpu_compute_forces_staged failed rc=%d n=%zu\n", gpu_rc, n);
        return false;
    }

    const auto t2 = Clock::now();

    m_body_forces.clear();
    m_modified_nodes.reserve(m_modified_nodes.size() + n);

    for (std::size_t i = 0; i < n; ++i) {
        const scm::gpu::HitOutput& o = batch_out[i];
        const ChVector2i ij(batch_in[i].grid_ix, batch_in[i].grid_iy);
        auto& nr = m_grid_map.at(ij);

        // Record the hit level for every hit node, as the CPU path does (bulldozing uses it to cap material raise)
        nr.hit_level = batch_in[i].hit_level;

        // Handle uni-laterality
        if (!o.active) {
            nr.sigma = 0;
            continue;
        }

        const double ca = nr.normal.z();

        nr.sinkage = o.sinkage;
        nr.sinkage_plastic = o.sinkage_plastic;
        nr.sinkage_elastic = o.sinkage_elastic;
        nr.sigma = o.sigma;
        nr.sigma_yield = o.sigma_yield;
        nr.kshear = o.kshear;
        nr.tau = o.tau;
        nr.step_plastic_flow = o.step_plastic_flow;
        nr.level = nr.level_initial - nr.sinkage / ca;

        m_modified_nodes.push_back(ij);
    }

    for (std::size_t b = 0; b < n_bodies; ++b) {
        const scm::gpu::BodyForceAccum& bf = body_forces[b];
        m_body_forces.emplace(bodies[b], std::make_pair(ChVector3d(bf.fx, bf.fy, bf.fz), ChVector3d(bf.mx, bf.my, bf.mz)));
    }

    const auto t3 = Clock::now();

    if (profile) {
        static int n_profile = 0;
        if (n_profile < 8) {
            const double pack_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            const double gpu_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
            const double scatter_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
            std::fprintf(stderr,
                         "SCM GPU profile n=%zu bodies=%zu pack_ms=%.3f gpu_ms=%.3f scatter_ms=%.3f\n",
                         n,
                         n_bodies,
                         pack_ms,
                         gpu_ms,
                         scatter_ms);
            ++n_profile;
        }
    }

    return true;
}

#else  // CHRONO_VEHICLE_SCM_GPU

bool SCMTerrain::IsGpuAvailable() {
    return false;
}

void SCMLoader::PrimeGpuBuffers() {}

bool SCMLoader::ComputeContactForcesGpu(const std::unordered_map<ChVector2i, ScmHitRecord, CoordHash>&, const std::vector<double>&) {
    return false;
}

#endif  // CHRONO_VEHICLE_SCM_GPU

}  // namespace vehicle
}  // namespace chrono
