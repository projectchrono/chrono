#include "chrono_parallel/constraints/ChConstraintRigidFluid.h"
#include "chrono_parallel/constraints/ChConstraintUtils.h"

using namespace chrono;

bool Cone_generalized_rf(real& gamma_n, real& gamma_u, real& gamma_v, const real& mu) {
    real f_tang = sqrt(gamma_u * gamma_u + gamma_v * gamma_v);

    // inside upper cone? keep untouched!
    if (f_tang < (mu * gamma_n)) {
        return false;
    }

    // inside lower cone? reset  normal,u,v to zero!
    if ((f_tang) < -(1.0 / mu) * gamma_n || (fabs(gamma_n) < 10e-15)) {
        gamma_n = 0;
        gamma_u = 0;
        gamma_v = 0;
        return false;
    }

    // remaining case: project orthogonally to generator segment of upper cone

    gamma_n = (f_tang * mu + gamma_n) / (mu * mu + 1);
    real tproj_div_t = (gamma_n * mu) / f_tang;
    gamma_u *= tproj_div_t;
    gamma_v *= tproj_div_t;

    return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintRigidFluid::Project(real* gamma) {
    // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    real mu = data_manager->settings.fluid.mu;
    real coh = data_manager->settings.fluid.cohesion;

    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
    int num_fluid_bodies = data_manager->num_fluid_bodies;

    //#pragma omp parallel for
    int index = 0;
    for (int p = 0; p < num_fluid_bodies; p++) {
        for (int i = 0; i < contact_counts[p]; ++i) {
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];  // rigid is stored in the first index
            int fluid = p;                                                  // fluid body is in second index
            real rigid_fric = data_manager->host_data.fric_data[rigid].x;
            real cohesion = Max((data_manager->host_data.cohesion_data[rigid] + coh) * .5, 0.0);
            real friction = (rigid_fric == 0 || mu == 0) ? 0 : (rigid_fric + mu) * .5;

            real3 gam;
            gam.x = gamma[num_unilaterals + num_bilaterals + index * 3 + 0];
            gam.y = gamma[num_unilaterals + num_bilaterals + index * 3 + 1];
            gam.z = gamma[num_unilaterals + num_bilaterals + index * 3 + 2];

            gam.x += cohesion;

            real mu = friction;
            if (mu == 0) {
                gam.x = gam.x < 0 ? 0 : gam.x - cohesion;
                gam.y = gam.z = 0;

                gamma[num_unilaterals + num_bilaterals + index * 3 + 0] = gam.x;
                gamma[num_unilaterals + num_bilaterals + index * 3 + 1] = gam.y;
                gamma[num_unilaterals + num_bilaterals + index * 3 + 2] = gam.z;

                continue;
            }

            if (Cone_generalized_rf(gam.x, gam.y, gam.z, mu)) {
            }

            gamma[num_unilaterals + num_bilaterals + index * 3 + 0] = gam.x - cohesion;
            gamma[num_unilaterals + num_bilaterals + index * 3 + 1] = gam.y;
            gamma[num_unilaterals + num_bilaterals + index * 3 + 2] = gam.z;
            index++;
        }
    }
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void ChConstraintRigidFluid::Build_D() {
    LOG(INFO) << "ChConstraintRigidFluid::Build_D";

    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;

    if (num_rigid_fluid_contacts <= 0) {
        return;
    }
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    custom_vector<real3>& pos = data_manager->host_data.sorted_pos_fluid;

    custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;
    custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;

    real h = data_manager->settings.fluid.kernel_radius;
    // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
    custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_fluid;
    custom_vector<real3>& norm = data_manager->host_data.norm_rigid_fluid;
    data_manager->system_timer.start("ChSolverParallel_solverC");
    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
    int num_fluid_bodies = data_manager->num_fluid_bodies;
    //#pragma omp parallel for
    int index = 0;
    for (int p = 0; p < num_fluid_bodies; p++) {
        for (int i = 0; i < contact_counts[p]; i++) {
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
            int fluid = p;  // fluid body is in second index
            real3 U = norm[p * max_rigid_neighbors + i], V, W;
            Orthogonalize(U, V, W);
            real3 T1, T2, T3;
            Compute_Jacobian(rot_rigid[rigid], U, V, W, cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1, T2,
                             T3);

            SetRow6(D_T, num_unilaterals + num_bilaterals + index * 3 + 0, rigid * 6, -U, T1);
            SetRow6(D_T, num_unilaterals + num_bilaterals + index * 3 + 1, rigid * 6, -V, T2);
            SetRow6(D_T, num_unilaterals + num_bilaterals + index * 3 + 2, rigid * 6, -W, T3);

            SetRow3(D_T, num_unilaterals + num_bilaterals + index * 3 + 0,
                    num_rigid_bodies * 6 + num_shafts + fluid * 3, U);
            SetRow3(D_T, num_unilaterals + num_bilaterals + index * 3 + 1,
                    num_rigid_bodies * 6 + num_shafts + fluid * 3, V);
            SetRow3(D_T, num_unilaterals + num_bilaterals + index * 3 + 2,
                    num_rigid_bodies * 6 + num_shafts + fluid * 3, W);
            index++;
        }
    }
    data_manager->system_timer.stop("ChSolverParallel_solverC");
}
void ChConstraintRigidFluid::Build_b() {
    LOG(INFO) << "ChConstraintRigidFluid::Build_b";
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    real step_size = data_manager->settings.step_size;
    if (num_rigid_fluid_contacts <= 0) {
        return;
    }

    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
    int num_fluid_bodies = data_manager->num_fluid_bodies;

    //#pragma omp parallel for
    int index = 0;
    for (int p = 0; p < num_fluid_bodies; p++) {
        for (int i = 0; i < contact_counts[p]; i++) {
            real bi = 0;
            real depth = data_manager->host_data.dpth_rigid_fluid[p * max_rigid_neighbors + i];

            bi = std::max(real(1.0) / step_size * depth, -data_manager->settings.fluid.contact_recovery_speed);
            //
            data_manager->host_data.b[num_unilaterals + num_bilaterals + index * 3 + 0] = bi;
            data_manager->host_data.b[num_unilaterals + num_bilaterals + index * 3 + 1] = 0;
            data_manager->host_data.b[num_unilaterals + num_bilaterals + index * 3 + 2] = 0;
            index++;
        }
    }
}
void ChConstraintRigidFluid::Build_E() {
    LOG(INFO) << "ChConstraintRigidFluid::Build_E";
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    real step_size = data_manager->settings.step_size;
    if (num_rigid_fluid_contacts <= 0) {
        return;
    }
    DynamicVector<real>& E = data_manager->host_data.E;

#pragma omp parallel for
    for (int index = 0; index < num_rigid_fluid_contacts; index++) {
        int ind = num_unilaterals + num_bilaterals;
        real epsilon = data_manager->settings.fluid.epsilon;
        real tau = data_manager->settings.fluid.tau;
        real h = data_manager->settings.fluid.kernel_radius;
        real compliance = 4.0 / (step_size * step_size) * (epsilon / (1.0 + 4.0 * tau / h));

        E[ind + index * 3 + 0] = 0;
        E[ind + index * 3 + 1] = 0;
        E[ind + index * 3 + 2] = 0;
    }
}

void ChConstraintRigidFluid::GenerateSparsity() {
    LOG(INFO) << "ChConstraintRigidFluid::GenerateSparsity";
    uint num_rigid_fluid_contacts = data_manager->num_rigid_fluid_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    real step_size = data_manager->settings.step_size;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
    int index = 0;

    int num_fluid_bodies = data_manager->num_fluid_bodies;
    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

    for (int p = 0; p < num_fluid_bodies; p++) {
        for (int i = 0; i < contact_counts[p]; i++) {
            // int2 body_id = bids[index];
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
            int fluid = p;
            int off = num_unilaterals + num_bilaterals;

            AppendRow6(D_T, off + index * 3 + 0, rigid * 6, 0);
            AppendRow3(D_T, off + index * 3 + 0, num_rigid_bodies * 6 + num_shafts + fluid * 3, 0);
            D_T.finalize(off + index * 3 + 0);

            AppendRow6(D_T, off + index * 3 + 1, rigid * 6, 0);
            AppendRow3(D_T, off + index * 3 + 1, num_rigid_bodies * 6 + num_shafts + fluid * 3, 0);
            D_T.finalize(off + index * 3 + 1);

            AppendRow6(D_T, off + index * 3 + 2, rigid * 6, 0);
            AppendRow3(D_T, off + index * 3 + 2, num_rigid_bodies * 6 + num_shafts + fluid * 3, 0);

            D_T.finalize(off + index * 3 + 2);
            index++;
        }
    }
}
