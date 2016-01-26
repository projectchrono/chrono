
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/lcp/MPMUtils.h"
#include <chrono_parallel/collision/ChCBroadphaseUtils.h>
#include <thrust/transform_reduce.h>
namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE

ChMPMContainer::ChMPMContainer(ChSystemParallelDVI* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->Add3DOFContainer(this);

    max_iterations = 10;
    real mass = 1;
    real mu = 1;
    real hardening_coefficient = 10;
    real lambda = 1;
    real theta_s = 7.5e-3;
    real theta_c = 2.5e-2;
    real alpha = .95;
}
ChMPMContainer::~ChMPMContainer() {}

void ChMPMContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);
}

void ChMPMContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

    pos_fluid.insert(pos_fluid.end(), positions.begin(), positions.end());
    vel_fluid.insert(vel_fluid.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_fluid.resize(pos_fluid.size());
    data_manager->num_fluid_bodies = pos_fluid.size();
}
void ChMPMContainer::Update(double ChTime) {
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;

    bbox res(pos_fluid[0], pos_fluid[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(pos_fluid.begin(), pos_fluid.end(), unary_op, res, binary_op);

    max_bounding_point = real3((Ceil(res.second.x), (res.second.x + kernel_radius * 12)),
                               (Ceil(res.second.y), (res.second.y + kernel_radius * 12)),
                               (Ceil(res.second.z), (res.second.z + kernel_radius * 12)));

    min_bounding_point = real3((Floor(res.first.x), (res.first.x - kernel_radius * 12)),
                               (Floor(res.first.y), (res.first.y - kernel_radius * 12)),
                               (Floor(res.first.z), (res.first.z - kernel_radius * 12)));
    real3 diag = max_bounding_point - min_bounding_point;
    bins_per_axis = int3(diag / (kernel_radius * 2));
    bin_edge = kernel_radius * 2 + collision_envelope;
    inv_bin_edge = real(1.) / bin_edge;

    grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("UPDATE [%d] [%d %d %d] [%f] %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z, bin_edge,
           num_fluid_bodies);

    real3 h_gravity = data_manager->settings.step_size * mass * data_manager->settings.gravity;
    for (int i = 0; i < num_fluid_bodies; i++) {
        // This was moved to after fluid collision detection
        // real3 vel = vel_fluid[i];
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = vel.x;
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = vel.y;
        // data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = vel.z;

        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + i * 3 + 2] = h_gravity.z;
    }
}

void ChMPMContainer::UpdatePosition(double ChTime) {
    custom_vector<real3>& pos_fluid = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_fluid = data_manager->host_data.vel_3dof;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;

    const real dt = data_manager->settings.step_size;

    printf("Update_Particle_Velocities\n");
#pragma omp parallel for
    for (int p = 0; p < num_fluid_bodies; p++) {
        int original_index = data_manager->host_data.particle_indices_3dof[p];
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

        real speed = Length(new_vel);
        if (speed > max_velocity) {
            new_vel = new_vel * max_velocity / speed;
        }
        vel_fluid[original_index] = new_vel;
        pos_fluid[original_index] += new_vel * data_manager->settings.step_size;

        // printf("v [%f %f %f] [%f %f %f]\n", V_pic.x, V_pic.y, V_pic.z, V_flip.x, V_flip.y, V_flip.z);
    }

    for (int i = 0; i < num_fluid_bodies; i++) {
        real3 vel;
        int original_index = data_manager->host_data.particle_indices_3dof[i];
        // these are sorted so we have to unsort them
        vel.x = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 0];
        vel.y = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 1];
        vel.z = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + i * 3 + 2];
    }
}

int ChMPMContainer::GetNumConstraints() {
    return 0;
}
int ChMPMContainer::GetNumNonZeros() {
    return 0;
}

void ChMPMContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    real inv_mass = 1.0 / mass;
    for (int i = 0; i < num_fluid_bodies; i++) {
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
        M_inv.finalize(offset + i * 3 + 2);
    }
}
void ChMPMContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    for (int i = 0; i < num_fluid_bodies; i++) {
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, mass);
        M.finalize(offset + i * 3 + 2);
    }
}

void ChMPMContainer::Initialize() {
    const real dt = data_manager->settings.step_size;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
    const int num_particles = data_manager->num_fluid_bodies;

    bbox res(sorted_pos[0], sorted_pos[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(sorted_pos.begin(), sorted_pos.end(), unary_op, res, binary_op);

    max_bounding_point = real3(Max((res.second.x), (res.second.x + kernel_radius * 12)),
                               Max((res.second.y), (res.second.y + kernel_radius * 12)),
                               Max((res.second.z), (res.second.z + kernel_radius * 12)));

    min_bounding_point = real3(Min((res.first.x), (res.first.x - kernel_radius * 12)),
                               Min((res.first.y), (res.first.y - kernel_radius * 12)),
                               Min((res.first.z), (res.first.z - kernel_radius * 12)));
    real3 diag = max_bounding_point - min_bounding_point;
    bins_per_axis = int3(diag / (kernel_radius * 2));
    bin_edge = kernel_radius * 2 + collision_envelope;
    inv_bin_edge = real(1.) / bin_edge;

    grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;

    grid_mass.resize(grid_size);
    grid_vel.resize(grid_size * 3);
    grid_vel_old.resize(grid_size * 3);
    grid_forces.resize(grid_size);

    volume.resize(num_particles);
    rhs.resize(grid_size * 3);

    delta_F.resize(num_particles);
    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Rasterize [%d] [%d %d %d] [%f] %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_particles);
    // clear initial vectors
    grid_vel = 0;
    grid_mass = 0;

    for (int p = 0; p < num_particles; p++) {
        int original_index = data_manager->host_data.particle_indices_3dof[p];
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];

        LOOPOVERNODES(                                                                //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge) * mass;  //
            grid_mass[current_node] += weight;                                        //
            grid_vel[current_node * 3 + 0] += weight * vi.x;                          //
            grid_vel[current_node * 3 + 1] += weight * vi.y;                          //
            grid_vel[current_node * 3 + 2] += weight * vi.z;                          //
            )
    }

    printf("Compute_Particle_Volumes\n");
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];
        real particle_density = 0;

        LOOPOVERNODES(                                                  //
            real weight = N(xi - current_node_location, inv_bin_edge);  //
            particle_density += grid_mass[current_node] * weight;       //
            )
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
void ChMPMContainer::IsInside() {
    grid_inside.resize(grid_size);
    std::fill(grid_inside.begin(), grid_inside.end(), false);
    uint num_shapes = data_manager->num_rigid_shapes;
#if 0
    for (int i = 0; i < grid_size; i++) {
        if (grid_mass[i] <= 0) {
            continue;
        }

        real3 location = grid_loc[i];
        for (int shape_id_a = 0; shape_id_a < num_shapes; shape_id_a++) {
            ConvexShape shapeA(data_manager->host_data.typ_rigid[shape_id_a],          //
                               data_manager->host_data.obj_data_A_global[shape_id_a],  //
                               data_manager->host_data.obj_data_B_global[shape_id_a],  //
                               data_manager->host_data.obj_data_C_global[shape_id_a],  //
                               data_manager->host_data.obj_data_R_global[shape_id_a],  //
                               data_manager->host_data.convex_data.data());            //

            ConvexShape shapeB(SPHERE, location,                             //
                               real3(kernel_radius, 0, 0),                   //
                               real3(0),                                     //
                               quaternion(1, 0, 0, 0),                       //
                               data_manager->host_data.convex_data.data());  //

            real3 ptA, ptB, norm;
            real depth;

            if (MPRCollision(shapeA, shapeB, data_manager->settings.collision.collision_envelope, norm, ptA, ptB,
                             depth)) {
                grid_inside[i] = true;
            }
        }
    }
#endif
}
void ChMPMContainer::Multiply(DynamicVector<real>& v_array, DynamicVector<real>& result_array) {
    const int num_particles = data_manager->num_fluid_bodies;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
    const real dt = data_manager->settings.step_size;

    printf("Apply Hessian A\n");

#pragma omp parallel for
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        Mat33 delta_F_t(0);
        delta_F[p] = Mat33(0);
        LOOPOVERNODES(                                                                                              //
            real3 v0(v_array[current_node * 3 + 0], v_array[current_node * 3 + 1], v_array[current_node * 3 + 2]);  //
            real3 v1 = dN(xi - current_node_location, inv_bin_edge);                                                //
            if (grid_inside[current_node] == false) {                                                               //
                delta_F[p] += OuterProduct(v0, v1) * Fe[p];                                                         //
            })
    }

    printf("Apply Hessian B\n");
    //#pragma omp parallel for
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

        Mat33 volume_Ap_Fe_transpose =
            volume[p] * (2 * current_mu * (delta_F[p] - RD) + (current_lambda * J * dJ) * Fe_hat_inv_transpose +
                         (current_lambda * (J - 1.0)) * dJF_inverse_transposed) *
            Transpose(Fe[p]);

        const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);
        const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);
        const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);

        for (int i = cx - 2; i <= cx + 2; ++i) {
            for (int j = cy - 2; j <= cy + 2; ++j) {
                for (int k = cz - 2; k <= cz + 2; ++k) {
                    const int current_node = GridHash(i, j, k, bins_per_axis);
                    real3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point);
                    real3 res = volume_Ap_Fe_transpose * dN(xi - current_node_location, inv_bin_edge);  //
                    if (grid_inside[current_node] == false) {
                        //#pragma omp atomic
                        result_array[current_node * 3 + 0] += res.x;  //
                                                                      //#pragma omp atomic
                        result_array[current_node * 3 + 1] += res.y;  //
                                                                      //#pragma omp atomic
                        result_array[current_node * 3 + 2] += res.z;  //
                    }
                }
            }
        }
    }
#pragma omp parallel for
    for (int i = 0; i < grid_size; i++) {
        if (grid_inside[i] == false) {
            result_array[i * 3 + 0] = grid_mass[i] * v_array[i * 3 + 0] + dt * dt * result_array[i * 3 + 0];
            result_array[i * 3 + 1] = grid_mass[i] * v_array[i * 3 + 1] + dt * dt * result_array[i * 3 + 1];
            result_array[i * 3 + 2] = grid_mass[i] * v_array[i * 3 + 2] + dt * dt * result_array[i * 3 + 2];
        } else {
            result_array[i * 3 + 0] = 0;
            result_array[i * 3 + 1] = 0;
            result_array[i * 3 + 2] = 0;
        }
    }
}

real ChMPMContainer::Convergence_Norm(const DynamicVector<real>& r) {
    real result = (real)0.;
    for (int i = 0; i < r.size(); i += 3) {
        if (grid_inside[i] == false) {
            real3 v(r[i + 0], r[i + 1], r[i + 2]);
            real mag = Length(v);
            result = Max(result, mag);
        }
    }
    return result;
}
real ChMPMContainer::DotProduct(const DynamicVector<real>& a, const DynamicVector<real>& b) {
    real result = 0;

    for (int i = 0; i < grid_inside.size(); i++) {
        if (grid_inside[i] == false) {
            result += a[i * 3 + 0] * b[i * 3 + 0] + a[i * 3 + 1] * b[i * 3 + 1] + a[i * 3 + 2] * b[i * 3 + 2];
        } else {
        }
    }
    return result;
}
real ChMPMContainer::ComputeTotalEnergy(DynamicVector<real>& x) {
    //    // Kinetic Energy of grid nodes
    //    const int num_particles = data_manager->num_fluid_bodies;
    //    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    //    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
    //    const real fluid_radius = kernel_radius;
    //    const real bin_edge = fluid_radius * 2 + collision_envelope;
    //    const real inv_bin_edge = real(1) / bin_edge;
    //    const real dt = data_manager->settings.step_size;
    //    const real3 max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    //    const real3 min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    //    const int3 bins_per_axis = data_manager->measures.collision.ff_bins_per_axis;
    //    size_t grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;
    //
    //// Fe_hat depends on the velocity of the grid node, if that is changing we can also update the Fe_hat quantity
    //// Not in the original paper like this but using it for testing
    //#if 1
    //#pragma omp parallel for
    //    for (int p = 0; p < num_particles; p++) {
    //        const real3 xi = sorted_pos[p];
    //        Fe_hat[p] = Mat33(1.0);
    //        Mat33 Fe_hat_t(1);
    //        LOOPOVERNODES(  //
    //            real3 vel(x[current_node * 3 + 0], x[current_node * 3 + 1], x[current_node * 3 + 2]);
    //            real3 kern = dN(xi - current_node_location, inv_bin_edge);  //
    //            Fe_hat_t += OuterProduct(dt * vel, kern);                   //
    //            )
    //        Fe_hat[p] = Fe_hat_t * Fe[p];
    //    }
    //#endif
    //
    //    real KE = 0;
    //
    //    for (int i = 0; i < grid_size; i++) {
    //        real3 v;
    //        v.x = x[i * 3 + 0] - grid_vel_old[i * 3 + 0];
    //        v.y = x[i * 3 + 1] - grid_vel_old[i * 3 + 1];
    //        v.z = x[i * 3 + 2] - grid_vel_old[i * 3 + 2];
    //
    //        KE += 1.0 / 2.0 * grid_mass[i] * Length(v) * Length(v);
    //    }
    //
    //    real PE = 0;
    //
    //    for (int p = 0; p < num_particles; p++) {
    //        const real3 xi = sorted_pos[p];
    //        real plastic_determinant = Determinant(Fp[p]);
    //        real J = Determinant(Fe_hat[p]);
    //        real current_mu = mu * Exp(hardening_coefficient * (1.0 - plastic_determinant));
    //        real current_lambda = lambda * Exp(hardening_coefficient * (1.0 - plastic_determinant));
    //
    //        Mat33 U, V, RE;
    //        real3 E, b;
    //        SVD(Fe_hat[p], U, E, V);
    //        // Perform polar decomposition F = R*S
    //        RE = MultTranspose(U, V);
    //
    //        PE += volume[p] * current_mu * Norm(Fe_hat[p] - RE) + current_lambda / 2.0 * (J - 1.0) * (J - 1.0);
    //    }
    //    return PE + KE;
    //
    //    // Compute derivative of the potential energy wrt x
    //    // Compute force on each grid node
    //
    //    for (int p = 0; p < num_particles; p++) {
    //        const real3 xi = sorted_pos[p];
    //
    //        real plastic_determinant = Determinant(Fp[p]);
    //        real J = Determinant(Fe_hat[p]);
    //        real current_mu = mu * Exp(hardening_coefficient * (1.0 - plastic_determinant));
    //        real current_lambda = lambda * Exp(hardening_coefficient * (1.0 - plastic_determinant));
    //        Mat33 Fe_hat_inv_transpose = InverseTranspose(Fe_hat[p]);
    //
    //        Mat33 v_sigma_p;
    //
    //        Mat33 U, V, RE;
    //        real3 E, b;
    //        SVD(Fe_hat[p], U, E, V);
    //        // Perform polar decomposition F = R*S
    //        RE = MultTranspose(U, V);
    //
    //        partial_Psi_partial_FE =
    //            2 * current_mu * (Fe_hat[p] - RE) + current_lambda * (J - 1) * J * Fe_hat_inv_transpose;
    //        v_sigma_p = -volume[p] * 1. / plastic_determinant * partial_Psi_partial_FE * Transpose(Fe[p]);
    //
    //        LOOPOVERNODES(  //
    //            real3 res = v_sigma_p * dN(xi - current_node_location, inv_bin_edge);
    //
    //            force_internal[current_node * 3 + 0] += res.x;  //
    //            force_internal[current_node * 3 + 1] += res.y;  //
    //            force_internal[current_node * 3 + 2] += res.z;  //
    //            )
    //    }
}

void ChMPMContainer::Solve(const DynamicVector<real>& b, DynamicVector<real>& x) {
    r.resize(b.size());
    q.resize(b.size());
    s.resize(b.size());

    real rho_old = FLT_MAX;
    real convergence_norm = 0;
    real tolerance = 1e-4;  // Max(1e-4 * Convergence_Norm(b), 1e-6);
    int min_iterations = 0;

    int iterations;
    int restart_iterations = 100;
    printf("ENERGY S: %f\n", (x,x));
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
            break;
        }
        if (iterations == max_iterations) {
            break;
        }

        real rho = DotProduct(r, r);
        if (restart) {
            s = r;
        } else {
            s = rho / rho_old * s + r;
        }
        Multiply(s, q);
        real s_dot_q = DotProduct(s, q);
        real alpha = s_dot_q ? rho / s_dot_q : (real)FLT_MAX;
        x = alpha * s + x;
        r = -alpha * q + r;
        rho_old = rho;


    }
    printf("ENERGY E: %f\n", (x,x));
}
void ChMPMContainer::PreSolve() {
    const real dt = data_manager->settings.step_size;
    const real3 gravity = data_manager->settings.gravity;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
    const int num_particles = data_manager->num_fluid_bodies;

    printf("START MPM STEP\n");

    grid_mass.resize(grid_size);
    grid_vel.resize(grid_size * 3);
    grid_vel_old.resize(grid_size * 3);
    grid_forces.resize(grid_size);
    grid_loc.resize(grid_size);

    volume.resize(num_particles);
    rhs.resize(grid_size * 3);

    delta_F.resize(num_particles);

    Fe.resize(num_particles);
    std::fill(Fe.begin(), Fe.end(), Mat33(1));

    Fe_hat.resize(num_particles);
    Fp.resize(num_particles);
    std::fill(Fp.begin(), Fp.end(), Mat33(1));

    // clear initial vectors
    grid_vel = 0;
    grid_mass = 0;

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);

    printf("Rasterize [%d] [%d %d %d] [%f] %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_particles);
    for (int p = 0; p < num_particles; p++) {
        const real3 xi = sorted_pos[p];
        const real3 vi = sorted_vel[p];
        //        real3 vi;
        //        vi.x = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 0];
        //        vi.y = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 1];
        //        vi.z = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + p * 3 + 2];

        LOOPOVERNODES(                                                         //
            real weight = N(xi - current_node_location, inv_bin_edge) * mass;  //
            grid_mass[current_node] += weight;                                 //
            grid_vel[current_node * 3 + 0] += weight * vi.x;                   //
            grid_vel[current_node * 3 + 1] += weight * vi.y;                   //
            grid_vel[current_node * 3 + 2] += weight * vi.z;                   //
            grid_loc[current_node] = current_node_location;                    // This is not efficient
            )
    }

    IsInside();

// normalize weights for the velocity (to conserve momentum)
#pragma omp parallel for
    for (int i = 0; i < grid_size; i++) {
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
    for (int i = 0; i < grid_size; i++) {
        grid_forces[i] += grid_mass[i] * gravity;
    }

    printf("Update_Grid_Velocities\n");
#pragma omp parallel for
    for (int i = 0; i < grid_size; i++) {
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
    for (int i = 0; i < grid_size; i++) {
        if (grid_inside[i] == false) {
            rhs[i * 3 + 0] = grid_mass[i] * grid_vel[i * 3 + 0];
            rhs[i * 3 + 1] = grid_mass[i] * grid_vel[i * 3 + 1];
            rhs[i * 3 + 2] = grid_mass[i] * grid_vel[i * 3 + 2];
        } else {
            rhs[i * 3 + 0] = 0;
            rhs[i * 3 + 1] = 0;
            rhs[i * 3 + 2] = 0;
        }
    }

    printf("Solve\n");
    Solve(rhs, grid_vel);
}
void ChMPMContainer::PostSolve() {
    const real dt = data_manager->settings.step_size;
    const real3 gravity = data_manager->settings.gravity;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
    const int num_particles = data_manager->num_fluid_bodies;

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

        Fe[p] = U * Mat33(E_clamped) * Transpose(V);
        // Inverse of Diagonal E_clamped matrix is 1/E_clamped
        Fp[p] = V * Mat33(1.0 / E_clamped) * Transpose(U) * F_tmp;
    }
}

void ChMPMContainer::UpdateRhs() {}

void ChMPMContainer::ComputeInternalForces() {
    //    // Kinetic Energy of grid nodes
    //    const int num_particles = data_manager->num_fluid_bodies;
    //    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    //    custom_vector<real3>& sorted_vel = data_manager->host_data.sorted_vel_3dof;
    //    const real fluid_radius = kernel_radius;
    //    const real bin_edge = fluid_radius * 2 + collision_envelope;
    //    const real inv_bin_edge = real(1) / bin_edge;
    //    const real dt = data_manager->settings.step_size;
    //    const real3 max_bounding_point = data_manager->measures.collision.ff_max_bounding_point;
    //    const real3 min_bounding_point = data_manager->measures.collision.ff_min_bounding_point;
    //    const int3 bins_per_axis = data_manager->measures.collision.ff_bins_per_axis;
    //    size_t grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;
    //
    //// Fe_hat depends on the velocity of the grid node, if that is changing we can also update the Fe_hat quantity
    //// Not in the original paper like this but using it for testing
    //#if 1
    //#pragma omp parallel for
    //    for (int p = 0; p < num_particles; p++) {
    //        const real3 xi = sorted_pos[p];
    //        Fe_hat[p] = Mat33(1.0);
    //        Mat33 Fe_hat_t(1);
    //        LOOPOVERNODES(  //
    //            real3 vel(grid_vel[current_node * 3 + 0], grid_vel[current_node * 3 + 1], grid_vel[current_node * 3 +
    //            2]);
    //            real3 kern = dN(xi - current_node_location, inv_bin_edge);  //
    //            Fe_hat_t += OuterProduct(dt * vel, kern);                   //
    //            )
    //        Fe_hat[p] = Fe_hat_t * Fe[p];
    //    }
    //#endif
    //
    //    // Compute derivative of the potential energy wrt x
    //    // Compute force on each grid node
    //
    //    for (int p = 0; p < num_particles; p++) {
    //        const real3 xi = sorted_pos[p];
    //
    //        real plastic_determinant = Determinant(Fp[p]);
    //        real J = Determinant(Fe_hat[p]);
    //        real current_mu = mu * Exp(hardening_coefficient * (1.0 - plastic_determinant));
    //        real current_lambda = lambda * Exp(hardening_coefficient * (1.0 - plastic_determinant));
    //        Mat33 Fe_hat_inv_transpose = InverseTranspose(Fe_hat[p]);
    //
    //        Mat33 v_sigma_p;
    //
    //        Mat33 U, V, RE;
    //        real3 E, b;
    //        SVD(Fe_hat[p], U, E, V);
    //        // Perform polar decomposition F = R*S
    //        RE = MultTranspose(U, V);
    //
    //        partial_Psi_partial_FE =
    //            2 * current_mu * (Fe_hat[p] - RE) + current_lambda * (J - 1) * J * Fe_hat_inv_transpose;
    //        v_sigma_p = -volume[p] * 1. / plastic_determinant * partial_Psi_partial_FE * Transpose(Fe[p]);
    //
    //        LOOPOVERNODES(  //
    //            real3 res = v_sigma_p * dN(xi - current_node_location, inv_bin_edge);
    //
    //            force_internal[current_node * 3 + 0] += res.x;  //
    //            force_internal[current_node * 3 + 1] += res.y;  //
    //            force_internal[current_node * 3 + 2] += res.z;  //
    //            )
    //    }
}

}  // END_OF_NAMESPACE____

/////////////////////
