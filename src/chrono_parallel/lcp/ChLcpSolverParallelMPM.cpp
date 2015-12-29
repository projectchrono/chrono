#include "chrono_parallel/lcp/ChLcpSolverParallel.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

#include "chrono_parallel/solver/ChSolverAPGD.h"
#include "chrono_parallel/solver/ChSolverAPGDREF.h"
#include "chrono_parallel/lcp/MPMUtils.h"

using namespace chrono;
#define CLEAR_RESERVE_RESIZE(M, nnz, rows, cols) \
    clear(M);                                    \
    M.reserve(nnz);                              \
    M.resize(rows, cols, false);

void ChLcpSolverParallelMPM::RunTimeStep() {
    const real3 max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    const real3 min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    const int3 bins_per_axis = data_manager->measures.collision.ff_bins_per_axis;
    const real fluid_radius = data_manager->settings.mpm.kernel_radius;
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

    // Rasterize
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];

        LOOPOVERNODES(                                                                //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge) * mass;  //
            //            printf("node: %d %f [%f %f %f] [%f %f %f]\n", current_node, weight, xi.x, xi.y, xi.z,
            //            current_node_location.x,
            //            current_node_location.y, current_node_location.z);
            grid_mass[current_node] += weight;
            ((real3*)grid_vel.data())[current_node] += weight * real3(vi);)
    }
    // Copy Grid velocities
    grid_vel_old = grid_vel;
    // Compute_Elastic_Deformation_Gradient_Hat
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        Fe_hat[p] = Mat33(1.0);
        LOOPOVERNODES(  //
            Fe_hat[p] += OuterProduct(dt * ((real3*)grid_vel.data())[current_node],
                                      dN(real3(xi) - current_node_location, inv_bin_edge));  //
            )
        Fe_hat[p] *= Fe[p];
    }
    // Compute_Grid_Forces
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        LOOPOVERNODES(  //
            Mat33 PED = Potential_Energy_Derivative(Fe_hat[i], Fp[i], mu, lambda, hardening_coefficient);
            grid_forces[current_node] -=
            (volume[i] * PED * Transpose(Fe[i]) * dN(xi - current_node_location, inv_bin_edge)) /
            (Determinant(Fe[i]) * Determinant(Fp[i]));)
    }
    // Add_Body_Forces
    for (int i = 0; i < num_nodes; i++) {
        grid_forces[i] += grid_mass[i] * gravity;
    }
    // Update_Grid_Velocities
    for (int i = 0; i < num_nodes; i++) {
        if (grid_mass[i] >= FLT_EPSILON) {
            ((real3*)grid_vel.data())[i] += dt * grid_forces[i] / grid_mass[i];
        }
    }

    // Semi_Implicit_Update
    // Compute RHS
    for (int i = 0; i < num_nodes; i++) {
        ((real3*)rhs.data())[i] += grid_mass[i] * ((real3*)grid_vel.data())[i];
    }

    Solve(rhs, grid_vel);

    const real theta_c = data_manager->settings.mpm.theta_c;
    const real theta_s = data_manager->settings.mpm.theta_s;
    const real alpha = data_manager->settings.mpm.alpha;

    //   Update_Deformation_Gradient
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        Mat33 velocity_gradient(0);
        LOOPOVERNODES(  //
            velocity_gradient += OuterProduct(((real3*)grid_vel.data())[current_node],
                                              dN(real3(xi) - current_node_location, inv_bin_edge));  //
            )
        Mat33 Fe_tmp = (Mat33(1.0) + dt * velocity_gradient) * Fe[p];
        Mat33 F_tmp = Fe_tmp * Fp[p];
        Mat33 U, V;
        real3 E;
        SVD(Fe_tmp, U, E, V);
        real3 E_clamped;

        E_clamped.x = Clamp(E.x, 1.0 - theta_c, 1.0 + theta_s);
        E_clamped.y = Clamp(E.y, 1.0 - theta_c, 1.0 + theta_s);
        E_clamped.z = Clamp(E.z, 1.0 - theta_c, 1.0 + theta_s);

        Fe[p] = U * MultTranspose(E_clamped, V);
        // Inverse of Diagonal E_clamped matrix is 1/E_clamped
        Fp[p] = V * MultTranspose(Mat33(1.0 / E_clamped), U) * F_tmp;
    }

    // Update_Particle_Velocities
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        real3 V_flip = sorted_vel[p];
        real3 V_pic = real3(0.0);
        LOOPOVERNODES(                                                         //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge);  //
            V_pic += ((real3*)grid_vel.data())[current_node] * weight;         //
            V_flip +=
            (((real3*)grid_vel.data())[current_node] - ((real3*)grid_vel_old.data())[current_node]) * weight;  //
            )

        real3 new_vel = (1.0 - alpha) * V_pic + alpha * V_flip;
        sorted_vel[p] = new_vel;
        sorted_pos[p] += new_vel * dt;
    }
}

void ChLcpSolverParallelMPM::Multiply(DynamicVector<real>& v_array, DynamicVector<real>& result_array) {
    real mu = data_manager->settings.mpm.mu;
    real hardening_coefficient = data_manager->settings.mpm.hardening_coefficient;
    real lambda = data_manager->settings.mpm.lambda;
    const int num_particles = data_manager->num_fluid_bodies;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_fluid;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_fluid;
    const real fluid_radius = data_manager->settings.mpm.kernel_radius;
    const real bin_edge = fluid_radius * 2 + data_manager->settings.fluid.collision_envelope;
    const real inv_bin_edge = real(1) / bin_edge;
    const real dt = data_manager->settings.step_size;
    const real3 max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    const real3 min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    const int3 bins_per_axis = data_manager->measures.collision.ff_bins_per_axis;
    size_t num_nodes = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    // Apply Hessian A

    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        delta_F[p] = Mat33(0);

        LOOPOVERNODES(                                                //
            real3 v0 = ((real3*)v_array.data())[current_node];        //
            real3 v1 = dN(xi - current_node_location, inv_bin_edge);  //
            delta_F[p] += OuterProduct(v0, v1) * Fe[p];               //
            )
    }

    // Apply Hessian B

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
        Mat33 Ap = 2 * current_mu * (delta_F[p] - Rotational_Derivative(Fe_hat[p], delta_F[p])) +
                   current_lambda * J * Fe_hat_inv_transpose * dJ +
                   current_lambda * (J - 1.0f) * dJF_inverse_transposed;

        LOOPOVERNODES(((real3*)result_array.data())[current_node] +=
                      volume[p] * Ap * Transpose(Fe[p]) * dN(xi - current_node_location, inv_bin_edge);  //
                      )                                                                                  //
    }

    for (int i = 0; i < num_nodes; i++) {
        ((real3*)result_array.data())[i] +=
            grid_mass[i] * ((real3*)v_array.data())[i] + dt * dt * ((real3*)result_array.data())[i];
    }
}

void ChLcpSolverParallelMPM::Solve(DynamicVector<real>& mb, DynamicVector<real>& ml) {
    r.resize(mb.size()), Ap.resize(mb.size());

    real rsold;
    real alpha;
    real rsnew = 0;
    real normb = Sqrt((mb, mb));

    if (normb == 0.0) {
        normb = 1;
    }

    Multiply(ml, r);
    p = r = mb - r;
    rsold = (r, r);
    normb = 1.0 / normb;
    //    if (Sqrt(rsold) * normb <= data_manager->settings.solver.tolerance) {
    //        return ;
    //    }
    for (int current_iteration = 0; current_iteration < 100; current_iteration++) {
        Multiply(p, Ap);  // Ap = data_manager->host_data.D_T *
                          // (data_manager->host_data.M_invD * p);
        alpha = rsold / (p, Ap);
        rsnew = 0;
        ml = alpha * p + ml;
        r = -alpha * Ap + r;
        rsnew = (r, r);

        residual = Sqrt(rsnew) * normb;
        if (residual < 1e-10) {
            break;
        }
        p = rsnew / rsold * p + r;
        rsold = rsnew;
    }
}
void ChLcpSolverParallelMPM::ChangeSolverType(SOLVERTYPE type) {}
