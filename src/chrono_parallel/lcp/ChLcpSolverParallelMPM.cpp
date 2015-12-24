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
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_fluid;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_fluid;
    const int num_particles = data_manager->num_fluid_bodies;
    real mu = data_manager->settings.mpm.mu;
    real hardening_coefficient = data_manager->settings.mpm.hardening_coefficient;
    real lambda = data_manager->settings.mpm.lambda;

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
        Fe_hat[p] = Mat33(1.0);
        LOOPOVERNODES(  //
            Mat33 PED = Potential_Energy_Derivative(Fe_hat[i], Fp[i], mu, lambda, hardening_coefficient);
            grid_forces[current_node] -=
            (volume[i] * PED * Transpose(Fe[i]) * dN(xi - current_node_location, inv_bin_edge)) /
            (Determinant(Fe[i]) * Determinant(Fp[i]));)
    }
}

void ChLcpSolverParallelMPM::ChangeSolverType(SOLVERTYPE type) {}
