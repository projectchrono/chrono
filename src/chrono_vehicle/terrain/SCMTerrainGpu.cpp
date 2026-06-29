// SCMTerrainGpu.cpp — SCMLoader::ComputeContactForcesGpu (v3: GPU body reduce, grid-only scatter).

#ifdef CHRONO_VEHICLE_SCM_GPU

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <unordered_map>
#include <vector>

#include "chrono/physics/ChBody.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "scm_gpu.h"

namespace chrono {
namespace vehicle {
namespace scm_gpu {

bool Enabled() {
    const char* e = std::getenv("CHRONO_SCM_GPU");
    return e && e[0] == '1' && e[1] == '\0';
}

static ScmGpuContext* GpuContext() {
    static ScmGpuContext* ctx = scm_gpu_create(0);
    static bool primed = false;
    if (!primed) {
        scm_gpu_reserve(ctx, scm_gpu_reserve_hits());
        scm_gpu_warmup(ctx);
        primed = true;
    }
    return ctx;
}

void PrimeBuffers() {
    if (!Enabled())
        return;
    (void)GpuContext();
}

static int32_t BodySlot(std::vector<ChBody*>& bodies, ChBody* body) {
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i] == body)
            return static_cast<int32_t>(i);
    }
    bodies.push_back(body);
    return static_cast<int32_t>(bodies.size() - 1);
}

}  // namespace scm_gpu

bool SCMLoader::ComputeContactForcesGpu(
    const std::unordered_map<ChVector2i, scm_gpu::ScmHitRecord, CoordHash>& hits,
    const std::vector<double>& patch_oob) {
    using Clock = std::chrono::steady_clock;

    const std::size_t n = hits.size();
    if (n == 0)
        return true;

    if (n < scm_gpu_min_hits())
        return false;

    const bool profile = (std::getenv("CHRONO_SCM_GPU_PROFILE") && std::getenv("CHRONO_SCM_GPU_PROFILE")[0] == '1');
    const auto t0 = Clock::now();

    ScmGpuContext* ctx = scm_gpu::GpuContext();
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
        const int32_t bid = scm_gpu::BodySlot(bodies, body);

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

    const int gpu_rc = scm_gpu_compute_forces_staged(ctx, soil, n, n_bodies);
    if (gpu_rc != 0) {
        std::fprintf(stderr, "SCM GPU: scm_gpu_compute_forces_staged failed rc=%d n=%zu\n", gpu_rc, n);
        return false;
    }

    const auto t2 = Clock::now();

    m_body_forces.clear();
    m_modified_nodes.reserve(m_modified_nodes.size() + n);

    for (std::size_t i = 0; i < n; ++i) {
        const scm::gpu::HitOutput& o = batch_out[i];
        if (!o.active)
            continue;

        const ChVector2i ij(batch_in[i].grid_ix, batch_in[i].grid_iy);
        auto& nr = m_grid_map.at(ij);
        const double ca = nr.normal.z();

        nr.hit_level = batch_in[i].hit_level;
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
        m_body_forces.emplace(bodies[b],
                              std::make_pair(ChVector3d(bf.fx, bf.fy, bf.fz), ChVector3d(bf.mx, bf.my, bf.mz)));
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

}  // namespace vehicle
}  // namespace chrono

#endif  // CHRONO_VEHICLE_SCM_GPU
