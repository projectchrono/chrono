// SCMTerrainGpu.cpp — SCMLoader::ComputeContactForcesGpu implementation.

#ifdef CHRONO_VEHICLE_SCM_GPU

#include <cstdlib>
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
    return ctx;
}

}  // namespace scm_gpu

bool SCMLoader::ComputeContactForcesGpu(const std::vector<ChVector2i>& keys,
                                        const std::vector<scm_gpu::ScmHitRecord>& hits,
                                        const std::vector<double>& patch_oob) {
    const std::size_t n = hits.size();
    if (n == 0 || keys.size() != n)
        return true;

    if (n < scm_gpu_min_hits())
        return false;

    for (const auto& rec : hits) {
        if (!dynamic_cast<ChBody*>(rec.contactable))
            return false;
    }

    ScmGpuContext* ctx = scm_gpu::GpuContext();
    scm::gpu::HitInput* batch_in = scm_gpu_prepare_input(ctx, n);
    scm::gpu::HitOutput* batch_out = scm_gpu_prepare_output(ctx, n);
    if (!batch_in || !batch_out)
        return false;

    std::vector<ChBody*> bodies(n);

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

    for (std::size_t idx = 0; idx < n; ++idx) {
        const auto& rec = hits[idx];
        const ChVector2i& ij = keys[idx];
        auto& nr = m_grid_map.at(ij);
        const double ca = nr.normal.z();

        ChBody* body = dynamic_cast<ChBody*>(rec.contactable);
        bodies[idx] = body;

        auto hit_point_loc = m_frame.TransformPointParentToLocal(rec.abs_point);

        ChVector3d point_local(ij.x() * m_delta, ij.y() * m_delta, nr.level);
        ChVector3d point_abs = m_frame.TransformPointLocalToParent(point_local);
        ChVector3d speed_abs = rec.contactable->GetContactPointSpeed(point_abs);

        ChVector3d N = m_frame.TransformDirectionLocalToParent(nr.normal);
        const double Vn = Vdot(speed_abs, N);
        ChVector3d T = -(speed_abs - Vn * N);
        T.Normalize();

        double patch_oob_val = 0.0;
        if (rec.patch_id >= 0 && rec.patch_id < static_cast<int>(patch_oob.size()))
            patch_oob_val = patch_oob[static_cast<std::size_t>(rec.patch_id)];

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
        in.body_id = static_cast<int32_t>(idx);
        in.active = 1;

        if (auto cprops = rec.contactable->GetUserData<SCMContactableData>()) {
            in.c_cohesion = cprops->Mohr_cohesion;
            in.c_mu = cprops->Mohr_mu;
            in.c_janosi = cprops->Janosi_shear;
            in.area_ratio = cprops->area_ratio;
        }
    }

    const int gpu_rc = scm_gpu_compute_forces_staged(ctx, soil, n);
    if (gpu_rc != 0) {
        std::fprintf(stderr, "SCM GPU: scm_gpu_compute_forces_staged failed rc=%d n=%zu\n", gpu_rc,
                     static_cast<std::size_t>(n));
        return false;
    }

    m_body_forces.clear();

    for (std::size_t i = 0; i < n; ++i) {
        const scm::gpu::HitOutput& o = batch_out[i];
        if (!o.active)
            continue;

        auto& nr = m_grid_map.at(keys[i]);
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

        m_modified_nodes.push_back(keys[i]);

        ChVector3d force(o.fn_x + o.ft_x, o.fn_y + o.ft_y, o.fn_z + o.ft_z);
        ChVector3d moment = Vcross(ChVector3d(batch_in[i].px, batch_in[i].py, batch_in[i].pz) - bodies[i]->GetPos(),
                                  force);

        auto itr = m_body_forces.find(bodies[i]);
        if (itr == m_body_forces.end())
            m_body_forces.insert(std::make_pair(bodies[i], std::make_pair(force, moment)));
        else {
            itr->second.first += force;
            itr->second.second += moment;
        }
    }

    return true;
}

}  // namespace vehicle
}  // namespace chrono

#endif  // CHRONO_VEHICLE_SCM_GPU
