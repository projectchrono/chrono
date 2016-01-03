#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

#include "chrono_parallel/solver/ChSolverAPGD.h"
#include "chrono_parallel/solver/ChSolverAPGDREF.h"
#include "chrono_parallel/lcp/MPMUtils.h"
#include <algorithm>

using namespace chrono;
#define CLEAR_RESERVE_RESIZE(M, nnz, rows, cols) \
    clear(M);                                    \
    M.reserve(nnz);                              \
    M.resize(rows, cols, false);

// Make sure that differences between this and the physbam version are not indexing related.
// Here the particles are sorted for performance reasons

void ChLcpSolverParallelMPM::Initialize() {
    const real3 max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    const real3 min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    const int3 bins_per_axis = data_manager->measures.collision.ff_bins_per_axis;
    const real fluid_radius = data_manager->settings.fluid.kernel_radius;
    const real bin_edge = fluid_radius * 2 + data_manager->settings.fluid.collision_envelope;
    const real inv_bin_edge = real(1) / bin_edge;
    const real dt = data_manager->settings.step_size;
    const real mass = data_manager->settings.mpm.mass;
    const real3 gravity = data_manager->settings.gravity;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_fluid;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_fluid;
    const int num_particles = data_manager->num_fluid_bodies;
    real mu = data_manager->settings.mpm.mu;
    real hardening_coefficient = data_manager->settings.mpm.hardening_coefficient;
    real lambda = data_manager->settings.mpm.lambda;

    size_t num_nodes = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    grid_mass.resize(num_nodes);
    grid_vel.resize(num_nodes * 3);
    grid_vel_old.resize(num_nodes * 3);
    grid_forces.resize(num_nodes);

    volume.resize(num_particles);
    rhs.resize(num_nodes * 3);

    delta_F.resize(num_particles);
    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Rasterize [%d] [%d %d %d] [%f] %d\n", num_nodes, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_particles);
    // clear initial vectors
    grid_vel = 0;
    grid_mass = 0;

    for (int p = 0; p < num_particles; p++) {
        int original_index = data_manager->host_data.particle_indices_fluid[p];
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];

        LOOPOVERNODES(                                                                //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge) * mass;  //
            //            printf("node: %d %f [%f %f %f] [%f %f %f]\n", current_node, weight, xi.x, xi.y, xi.z,
            //            current_node_location.x,
            //            current_node_location.y, current_node_location.z);
            grid_mass[current_node] += weight;                //
            grid_vel[current_node * 3 + 0] += weight * vi.x;  //
            grid_vel[current_node * 3 + 1] += weight * vi.y;  //
            grid_vel[current_node * 3 + 2] += weight * vi.z;  //
            //            printf("node: %f [%f %f %f] [%f %f %f] [%f %f %f] %d [%d %d %d]\n", weight, weight * vi.x,
            //            weight * vi.y,
            //                   weight * vi.z, grid_vel[current_node * 3 + 0], grid_vel[current_node * 3 + 1],
            //                   grid_vel[current_node * 3 + 2], current_node_location.x, current_node_location.y,
            //                   current_node_location.z, original_index, i, j, k);
            )
    }

    printf("Compute_Particle_Volumes\n");
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];
        real particle_density = 0;

        LOOPOVERNODES(                                                  //
            real weight = N(xi - current_node_location, inv_bin_edge);  //
            particle_density += grid_mass[current_node] * weight;)
        particle_density /= (bin_edge * bin_edge * bin_edge);
        volume[p] = mass / particle_density;
    }
    printf("Initialize_Deformation_Gradients\n");
    //
    Fe.resize(num_particles);
    std::fill(Fe.begin(), Fe.end(), Mat33(1));

    Fe_hat.resize(num_particles);
    Fp.resize(num_particles);
    std::fill(Fp.begin(), Fp.end(), Mat33(1));

    // Initialize_Bodies
}

void ChLcpSolverParallelMPM::RunTimeStep() {
    const real3 max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    const real3 min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    const int3 bins_per_axis = data_manager->measures.collision.ff_bins_per_axis;
    const real fluid_radius = data_manager->settings.fluid.kernel_radius;
    const real bin_edge = fluid_radius * 2 + data_manager->settings.fluid.collision_envelope;
    const real inv_bin_edge = real(1) / bin_edge;
    const real dt = data_manager->settings.step_size;
    const real mass = data_manager->settings.mpm.mass;
    const real3 gravity = data_manager->settings.gravity;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_fluid;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_fluid;
    const int num_particles = data_manager->num_fluid_bodies;
    real mu = data_manager->settings.mpm.mu;
    real hardening_coefficient = data_manager->settings.mpm.hardening_coefficient;
    real lambda = data_manager->settings.mpm.lambda;

    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;

    size_t num_nodes = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    printf("START MPM STEP\n");
    // clear initial vectors
    grid_vel = 0;
    grid_mass = 0;

    DynamicVector<real3> grid_loc(num_nodes);
    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);

    printf("Rasterize [%d] [%d %d %d] [%f] %d\n", num_nodes, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_particles);
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];

        LOOPOVERNODES(                                                         //
            real weight = N(xi - current_node_location, inv_bin_edge) * mass;  //
            grid_mass[current_node] += weight;                                 //
            grid_vel[current_node * 3 + 0] += weight * vi.x;                   //
            grid_vel[current_node * 3 + 1] += weight * vi.y;                   //
            grid_vel[current_node * 3 + 2] += weight * vi.z;                   //
            grid_loc[current_node] = current_node_location;

            )
    }
// normalize weights for the velocity (to conserve momentum)
#pragma omp parallel for
    for (int i = 0; i < num_nodes; i++) {
        if (grid_mass[i] > C_EPSILON) {
            grid_vel[i * 3 + 0] /= grid_mass[i];
            grid_vel[i * 3 + 1] /= grid_mass[i];
            grid_vel[i * 3 + 2] /= grid_mass[i];
        }
    }
    // Save_Grid_Velocities
    grid_vel_old = grid_vel;

    printf("Compute_Elastic_Deformation_Gradient_Hat\n");
#pragma omp parallel for
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        Fe_hat[p] = Mat33(1.0);
        Mat33 Fe_hat_t(1);
        LOOPOVERNODES(  //
            real3 vel(grid_vel[current_node * 3 + 0], grid_vel[current_node * 3 + 1], grid_vel[current_node * 3 + 2]);
            real3 kern = dN(xi - current_node_location, inv_bin_edge); Fe_hat_t += OuterProduct(dt * vel, kern);  //
            )
        Fe_hat[p] = Fe_hat_t * Fe[p];
    }

    printf("Compute_Grid_Forces\n");

    std::fill(grid_forces.begin(), grid_forces.end(), real3(0));

    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];

        Mat33 PED = Potential_Energy_Derivative(Fe_hat[p], Fp[p], mu, lambda, hardening_coefficient);

        Mat33 vPEDFepT = volume[p] * MultTranspose(PED, Fe[p]);
        real JE = Determinant(Fe[p]);                                       //
        real JP = Determinant(Fp[p]);                                       //
        LOOPOVERNODES(                                                      //
            real3 d_weight = dN(xi - current_node_location, inv_bin_edge);  //
            grid_forces[current_node] -= (vPEDFepT * d_weight) / (JE * JP);

            )
    }

    printf("Add_Body_Forces [%f %f %f]\n", gravity.x, gravity.y, gravity.z);

#pragma omp parallel for
    for (int i = 0; i < num_nodes; i++) {
        grid_forces[i] += grid_mass[i] * gravity;
    }
    printf("Update_Grid_Velocities\n");
#pragma omp parallel for
    for (int i = 0; i < num_nodes; i++) {
        if (grid_mass[i] >= C_EPSILON) {
            real3 forces = grid_forces[i];

            grid_vel[i * 3 + 0] += dt * forces.x / grid_mass[i];
            grid_vel[i * 3 + 1] += dt * forces.y / grid_mass[i];
            grid_vel[i * 3 + 2] += dt * forces.z / grid_mass[i];
        }
    }

    printf("Semi_Implicit_Update\n");

    printf("Compute RHS\n");
#pragma omp parallel for
    for (int i = 0; i < num_nodes; i++) {
        rhs[i * 3 + 0] = grid_mass[i] * grid_vel[i * 3 + 0];
        rhs[i * 3 + 1] = grid_mass[i] * grid_vel[i * 3 + 1];
        rhs[i * 3 + 2] = grid_mass[i] * grid_vel[i * 3 + 2];
    }

    printf("Solve\n");
    Solve(rhs, grid_vel);

    const real theta_c = data_manager->settings.mpm.theta_c;
    const real theta_s = data_manager->settings.mpm.theta_s;
    const real alpha = data_manager->settings.mpm.alpha;

    printf("Update_Deformation_Gradient\n");
#pragma omp parallel for
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        Mat33 velocity_gradient(0);
        LOOPOVERNODES(  //
            real3 g_vel(grid_vel[current_node * 3 + 0], grid_vel[current_node * 3 + 1], grid_vel[current_node * 3 + 2]);
            velocity_gradient += OuterProduct(g_vel, dN(xi - current_node_location, inv_bin_edge));)

        Mat33 Fe_tmp = (Mat33(1.0) + dt * velocity_gradient) * Fe[p];
        Mat33 F_tmp = Fe_tmp * Fp[p];
        Mat33 U, V;
        real3 E;
        SVD(Fe_tmp, U, E, V);
        real3 E_clamped;

        E_clamped.x = Clamp(E.x, 1.0 - theta_c, 1.0 + theta_s);
        E_clamped.y = Clamp(E.y, 1.0 - theta_c, 1.0 + theta_s);
        E_clamped.z = Clamp(E.z, 1.0 - theta_c, 1.0 + theta_s);

        Fe[p] = U * MultTranspose(Mat33(E_clamped), V);
        // Inverse of Diagonal E_clamped matrix is 1/E_clamped
        Fp[p] = V * MultTranspose(Mat33(1.0 / E_clamped), U) * F_tmp;
    }
    printf("Update_Particle_Velocities\n");
#pragma omp parallel for
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        real3 V_flip;
        V_flip.x = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 0];
        V_flip.y = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 1];
        V_flip.z = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 2];

        real3 V_pic = real3(0.0);
        LOOPOVERNODES(                                                                                   //
            real weight = N(xi - current_node_location, inv_bin_edge);                                   //
            V_pic.x += grid_vel[current_node * 3 + 0] * weight;                                          //
            V_pic.y += grid_vel[current_node * 3 + 1] * weight;                                          //
            V_pic.z += grid_vel[current_node * 3 + 2] * weight;                                          //
            V_flip.x += (grid_vel[current_node * 3 + 0] - grid_vel_old[current_node * 3 + 0]) * weight;  //
            V_flip.y += (grid_vel[current_node * 3 + 1] - grid_vel_old[current_node * 3 + 1]) * weight;  //
            V_flip.z += (grid_vel[current_node * 3 + 2] - grid_vel_old[current_node * 3 + 2]) * weight;  //

            )

        real3 new_vel = (1.0 - alpha) * V_pic + alpha * V_flip;
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 0] = new_vel.x;
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 1] = new_vel.y;
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 2] = new_vel.z;
        // printf("v [%f %f %f] [%f %f %f]\n", V_pic.x, V_pic.y, V_pic.z, V_flip.x, V_flip.y, V_flip.z);
    }
}

void ChLcpSolverParallelMPM::Multiply(DynamicVector<real>& v_array, DynamicVector<real>& result_array) {
    real mu = data_manager->settings.mpm.mu;
    real hardening_coefficient = data_manager->settings.mpm.hardening_coefficient;
    real lambda = data_manager->settings.mpm.lambda;
    const int num_particles = data_manager->num_fluid_bodies;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_fluid;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_fluid;
    const real fluid_radius = data_manager->settings.fluid.kernel_radius;
    const real bin_edge = fluid_radius * 2 + data_manager->settings.fluid.collision_envelope;
    const real inv_bin_edge = real(1) / bin_edge;
    const real dt = data_manager->settings.step_size;
    const real3 max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    const real3 min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    const int3 bins_per_axis = data_manager->measures.collision.ff_bins_per_axis;
    size_t num_nodes = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;
    printf("Apply Hessian A\n");

#pragma omp parallel for
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        Mat33 delta_F_t(0);
        LOOPOVERNODES(                                                                                              //
            real3 v0(v_array[current_node * 3 + 0], v_array[current_node * 3 + 1], v_array[current_node * 3 + 2]);  //
            real3 v1 = dN(xi - current_node_location, inv_bin_edge);
            delta_F_t = delta_F_t + OuterProduct(v0, v1) * Fe[p];)
        delta_F[p] = delta_F_t;
    }

    printf("Apply Hessian B\n");

    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        real plastic_determinant = Determinant(Fp[p]);
        real J = Determinant(Fe_hat[p]);
        real current_mu = mu * Exp(hardening_coefficient * (1.0 - plastic_determinant));
        real current_lambda = lambda * Exp(hardening_coefficient * (1.0 - plastic_determinant));
        Mat33 Fe_hat_inv_transpose = InverseTranspose(Fe_hat[p]);

        real dJ = J * InnerProduct(Fe_hat_inv_transpose, delta_F[p]);
        Mat33 dF_inverse_transposed = -Fe_hat_inv_transpose * Transpose(delta_F[p]) * Fe_hat_inv_transpose;
        Mat33 dJF_inverse_transposed = dJ * Fe_hat_inv_transpose + J * dF_inverse_transposed;
        Mat33 RD = Rotational_Derivative(Fe_hat[p], delta_F[p]);

        Mat33 Ap = 2 * current_mu * (delta_F[p] - RD) + (current_lambda * J * dJ) * Fe_hat_inv_transpose +
                   (current_lambda * (J - 1.0)) * dJF_inverse_transposed;

        LOOPOVERNODES(                                                                                     //
            real3 res = volume[p] * Ap * Transpose(Fe[p]) * dN(xi - current_node_location, inv_bin_edge);  //
            result_array[current_node * 3 + 0] += res.x;                                                   //
            result_array[current_node * 3 + 1] += res.y;                                                   //
            result_array[current_node * 3 + 2] += res.z;                                                   //

            )
    }
#pragma omp parallel for
    for (int i = 0; i < num_nodes; i++) {
        result_array[i * 3 + 0] = grid_mass[i] * v_array[i * 3 + 0] + dt * dt * result_array[i * 3 + 0];
        result_array[i * 3 + 1] = grid_mass[i] * v_array[i * 3 + 1] + dt * dt * result_array[i * 3 + 1];
        result_array[i * 3 + 2] = grid_mass[i] * v_array[i * 3 + 2] + dt * dt * result_array[i * 3 + 2];
    }
}

real Convergence_Norm(const DynamicVector<real>& r) {
    real result = (real)0.;
    for (int i = 0; i < r.size(); i += 3) {
        real3 v(r[i + 0], r[i + 1], r[i + 2]);
        real mag = Length(v);
        result = Max(result, mag);
    }
    return result;
}

void ChLcpSolverParallelMPM::Solve(const DynamicVector<real>& b, DynamicVector<real>& x) {
    r.resize(b.size());
    q.resize(b.size());
    s.resize(b.size());

    // Ap.resize(mb.size();
    //    real rsold;
    //    real alpha;
    //    real rsnew = 0;
    //    real normb = Sqrt((mb, mb));
    //
    //    if (normb == 0.0) {
    //        normb = 1;
    //    }
    //
    //    Multiply(ml, r);
    //    p = r = mb - r;
    //    rsold = (r, r);
    //    normb = 1.0 / normb;
    //    //    if (Sqrt(rsold) * normb <= data_manager->settings.solver.tolerance) {
    //    //        return ;
    //    //    }
    //    for (int current_iteration = 0; current_iteration < 10; current_iteration++) {
    //        Multiply(p, Ap);  // Ap = data_manager->host_data.D_T *
    //                          // (data_manager->host_data.M_invD * p);
    //        real denom = (p, Ap);
    //        //        if (denom == 0) {
    //        //            break;
    //        //        }
    //        alpha = rsold / denom;
    //        rsnew = 0;
    //        ml = alpha * p + ml;
    //        r = -alpha * Ap + r;
    //        rsnew = (r, r);
    //
    //        residual = Sqrt(rsnew) * normb;
    //        if (residual < 1e-10) {
    //            break;
    //        }
    //        p = rsnew / rsold * p + r;
    //        rsold = rsnew;
    //    }
    int max_iterations = 10;
    real rho_old = FLT_MAX;
    real convergence_norm = 0;
    real tolerance = Max(1e-4 * Convergence_Norm(b), 1e-6);
    int min_iterations = 0;

    int iterations;
    int restart_iterations = 100;
    for (iterations = 0;; iterations++) {
        bool restart = !iterations || (restart_iterations && iterations % restart_iterations == 0);
        if (restart) {
            printf("restarting cg\n");
            r = b;
            Multiply(x, q);
            r -= q;
        }
        // system.Project(r);
        convergence_norm = Convergence_Norm(r);
        printf("%f\n", convergence_norm);

        if (convergence_norm <= tolerance && (iterations >= min_iterations || convergence_norm < C_EPSILON)) {
            printf("cg iterations %d\n", iterations);
            return;
        }
        if (iterations == max_iterations) {
            break;
        }

        real rho = (r, r);
        if (restart) {
            s = r;
        } else {
            s = rho / rho_old * s + r;
        }
        Multiply(s, q);
        real s_dot_q = (s, q);
        real alpha = s_dot_q ? rho / s_dot_q : (real)FLT_MAX;
        x = alpha * s + x;
        r = -alpha * q + r;
        rho_old = rho;
    }
}
void ChLcpSolverParallelMPM::ChangeSolverType(SOLVERTYPE type) {}
