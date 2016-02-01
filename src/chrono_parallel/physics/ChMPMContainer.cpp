
#include <algorithm>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/lcp/MPMUtils.h"
#include <chrono_parallel/collision/ChCBroadphaseUtils.h>
#include <thrust/transform_reduce.h>
#include "chrono_parallel/constraints/ChConstraintUtils.h"

#include "chrono_parallel/math/other_types.h"  // for uint, int2, int3
#include "chrono_parallel/math/real.h"         // for real
#include "chrono_parallel/math/real3.h"        // for real3
#include "chrono_parallel/math/mat33.h"        // for quaternion, real4

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE

ChMPMContainer::ChMPMContainer(ChSystemParallelDVI* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->AddMPMContainer(this);

    max_iterations = 10;
    real mass = 1;
    real mu = 1;
    real hardening_coefficient = 1;
    real lambda = 1;
    real theta_s = 1;
    real theta_c = 1;
    real alpha = 1;
}
ChMPMContainer::~ChMPMContainer() {}

void ChMPMContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);
    start_node = start_constraint;
    body_offset = num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + num_fea_nodes * 3;
    min_bounding_point = data_manager->measures.collision.mpm_min_bounding_point;
    max_bounding_point = data_manager->measures.collision.mpm_max_bounding_point;
}

void ChMPMContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;

    pos_marker.insert(pos_marker.end(), positions.begin(), positions.end());
    vel_marker.insert(vel_marker.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_marker.resize(pos_marker.size());
    data_manager->num_mpm_markers = pos_marker.size();

    Fe.resize(data_manager->num_mpm_markers);
    Fe_hat.resize(data_manager->num_mpm_markers);
    Fp.resize(data_manager->num_mpm_markers);
    delta_F.resize(data_manager->num_mpm_markers);
    volume.resize(data_manager->num_mpm_markers);

    std::fill(Fp.begin(), Fp.end(), Mat33(1));
    std::fill(Fe.begin(), Fe.end(), Mat33(1));
}
void ChMPMContainer::ComputeDOF() {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    real3& min_bounding_point = data_manager->measures.collision.mpm_min_bounding_point;
    real3& max_bounding_point = data_manager->measures.collision.mpm_max_bounding_point;

    bbox res(pos_marker[0], pos_marker[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(pos_marker.begin(), pos_marker.end(), unary_op, res, binary_op);

    max_bounding_point = real3((Ceil(res.second.x), (res.second.x + kernel_radius * 12)),
                               (Ceil(res.second.y), (res.second.y + kernel_radius * 12)),
                               (Ceil(res.second.z), (res.second.z + kernel_radius * 12)));

    min_bounding_point = real3((Floor(res.first.x), (res.first.x - kernel_radius * 12)),
                               (Floor(res.first.y), (res.first.y - kernel_radius * 12)),
                               (Floor(res.first.z), (res.first.z - kernel_radius * 12)));

    real3 diag = max_bounding_point - min_bounding_point;
    bin_edge = kernel_radius * 2;
    bins_per_axis = int3(diag / bin_edge);
    inv_bin_edge = real(1.) / bin_edge;
    uint grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;
    data_manager->num_mpm_nodes = grid_size;
    num_mpm_nodes = grid_size;
    num_mpm_constraints = num_mpm_nodes * 6;

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Compute DOF [%d] [%d %d %d] [%f] %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);
}

void ChMPMContainer::Update(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    const real dt = data_manager->settings.step_size;
    Setup(0);

    grid_mass.resize(num_mpm_nodes);
    grid_vel_old.resize(num_mpm_nodes * 3);
    Fe_node.resize(num_mpm_nodes);
    Fp_node.resize(num_mpm_nodes);
    rhs.resize(num_mpm_nodes * 3);

    std::fill(Fe_node.begin(), Fe_node.end(), Mat33(1));
    std::fill(Fp_node.begin(), Fp_node.end(), Mat33(1));

    grid_mass = 0;

    for (int i = 0; i < num_mpm_nodes; i++) {
        //        grid_vel_old[i * 3 + 0] = data_manager->host_data.v[body_offset + i * 3 + 0];
        //        grid_vel_old[i * 3 + 1] = data_manager->host_data.v[body_offset + i * 3 + 1];
        //        grid_vel_old[i * 3 + 2] = data_manager->host_data.v[body_offset + i * 3 + 2];

        data_manager->host_data.v[body_offset + i * 3 + 0] = 0;
        data_manager->host_data.v[body_offset + i * 3 + 1] = 0;
        data_manager->host_data.v[body_offset + i * 3 + 2] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 0] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = 0;
    }
    std::cout << "SIZE " << (data_manager->host_data.v.size()) << std::endl;

    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        const real3 vi = vel_marker[p];
        // printf("marker_vel: [%.20f %.20f %.20f] \n", vi.x, vi.y, vi.z);
        LOOPOVERNODES(                                                                       //
            real weight = N(xi - current_node_location, inv_bin_edge) * mass;                //
            grid_mass[current_node] += weight;                                               //
            data_manager->host_data.v[body_offset + current_node * 3 + 0] += weight * vi.x;  //
            data_manager->host_data.v[body_offset + current_node * 3 + 1] += weight * vi.y;  //
            data_manager->host_data.v[body_offset + current_node * 3 + 2] += weight * vi.z;  //
            //            printf("Contribution vel: [%.20f %.20f %.20f] \n",
            //                   data_manager->host_data.v[body_offset + current_node * 3 + 0],
            //                   data_manager->host_data.v[body_offset + current_node * 3 + 1],
            //                   data_manager->host_data.v[body_offset + current_node * 3 + 2]);
            )
    }
    // normalize weights for the velocity (to conserve momentum)
    //#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        if (grid_mass[i] > C_EPSILON) {
            data_manager->host_data.v[body_offset + i * 3 + 0] /= grid_mass[i];
            data_manager->host_data.v[body_offset + i * 3 + 1] /= grid_mass[i];
            data_manager->host_data.v[body_offset + i * 3 + 2] /= grid_mass[i];
            grid_vel_old[i * 3 + 0] = data_manager->host_data.v[body_offset + i * 3 + 0];
            grid_vel_old[i * 3 + 1] = data_manager->host_data.v[body_offset + i * 3 + 1];
            grid_vel_old[i * 3 + 2] = data_manager->host_data.v[body_offset + i * 3 + 2];
            //            printf("Node_vel: [%.20f %.20f %.20f] [%.20f \n", data_manager->host_data.v[body_offset + i *
            //            3 + 0],
            //                   data_manager->host_data.v[body_offset + i * 3 + 1],
            //                   data_manager->host_data.v[body_offset + i * 3 + 2], grid_mass[i]);
        }
    }

    printf("Compute_Elastic_Deformation_Gradient_Hat\n");
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        Fe_hat[p] = Mat33(1.0);
        Mat33 Fe_hat_t(1);
        LOOPOVERNODES(  //
            real3 vel(data_manager->host_data.v[body_offset + i * 3 + 0],
                      data_manager->host_data.v[body_offset + i * 3 + 1],
                      data_manager->host_data.v[body_offset + i * 3 + 2]);
            real3 kern = dN(xi - current_node_location, inv_bin_edge); Fe_hat_t += OuterProduct(dt * vel, kern);  //
            )
        Fe_hat[p] = Fe_hat_t * Fe[p];
    }

    //    printf("Compute_Grid_Forces\n");

    //    for (int p = 0; p < num_mpm_markers; p++) {
    //        const real3 xi = pos_marker[p];
    //
    //        Mat33 PED = Potential_Energy_Derivative(Fe_hat[p], Fp[p], mu, lambda, hardening_coefficient);
    //
    //        Mat33 vPEDFepT = volume[p] * MultTranspose(PED, Fe[p]);
    //        real JE = Determinant(Fe[p]);                                       //
    //        real JP = Determinant(Fp[p]);                                       //
    //        LOOPOVERNODES(                                                      //
    //            real3 d_weight = dN(xi - current_node_location, inv_bin_edge);  //
    //            real3 force = (vPEDFepT * d_weight) / (JE * JP);
    //
    //            data_manager->host_data.hf[body_offset + i * 3 + 0] -= force.x;
    //            data_manager->host_data.hf[body_offset + i * 3 + 1] -= force.y;
    //            data_manager->host_data.hf[body_offset + i * 3 + 2] -= force.z;
    //
    //            )
    //    }

    // update the position of the markers based on the nodal velocities
}

void ChMPMContainer::UpdatePosition(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    DynamicVector<real>& v = data_manager->host_data.v;
    printf("Update_Particle_Velocities\n");
    //#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real3 V_flip = vel_marker[p];

        real3 V_pic = real3(0.0);
        LOOPOVERNODES(                                                                                          //
            real weight = N(xi - current_node_location, inv_bin_edge);                                          //
            V_pic.x += v[body_offset + current_node * 3 + 0] * weight;                                          //
            V_pic.y += v[body_offset + current_node * 3 + 1] * weight;                                          //
            V_pic.z += v[body_offset + current_node * 3 + 2] * weight;                                          //
            V_flip.x += (v[body_offset + current_node * 3 + 0] - grid_vel_old[current_node * 3 + 0]) * weight;  //
            V_flip.y += (v[body_offset + current_node * 3 + 1] - grid_vel_old[current_node * 3 + 1]) * weight;  //
            V_flip.z += (v[body_offset + current_node * 3 + 2] - grid_vel_old[current_node * 3 + 2]) * weight;  //
            )

        real3 new_vel = (1.0 - alpha) * V_pic + alpha * V_flip;

        real speed = Length(new_vel);
        if (speed > max_velocity) {
            new_vel = new_vel * max_velocity / speed;
        }
        vel_marker[p] = new_vel;
        pos_marker[p] += new_vel * data_manager->settings.step_size;
    }
}

int ChMPMContainer::GetNumConstraints() {
    return 0;  // num_mpm_nodes * 6;
}

int ChMPMContainer::GetNumNonZeros() {
    return 0;  // num_mpm_nodes * 6 * 3;
}

void ChMPMContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    for (int i = 0; i < num_mpm_nodes; i++) {
        if (grid_mass[i] > C_EPSILON) {
            real inv_mass = 1.0 / grid_mass[i];
            M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
            M_inv.finalize(offset + i * 3 + 0);
            M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
            M_inv.finalize(offset + i * 3 + 1);
            M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
            M_inv.finalize(offset + i * 3 + 2);
        }
    }
}
void ChMPMContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    for (int i = 0; i < num_mpm_nodes; i++) {
        if (grid_mass[i] > C_EPSILON) {
            real mass = grid_mass[i];
            M.append(offset + i * 3 + 0, offset + i * 3 + 0, mass);
            M.finalize(offset + i * 3 + 0);
            M.append(offset + i * 3 + 1, offset + i * 3 + 1, mass);
            M.finalize(offset + i * 3 + 1);
            M.append(offset + i * 3 + 2, offset + i * 3 + 2, mass);
            M.finalize(offset + i * 3 + 2);
        }
    }
}

void ChMPMContainer::Initialize() {
    const real dt = data_manager->settings.step_size;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Initialize [%d] [%d %d %d] [%f] %d\n", num_mpm_nodes, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);

    volume.resize(num_mpm_markers);
    grid_mass.resize(num_mpm_nodes);
    grid_mass = 0;
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        const real3 vi = vel_marker[p];

        LOOPOVERNODES(                                                                //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge) * mass;  //
            grid_mass[current_node] += weight;                                        //
            data_manager->host_data.v[body_offset + current_node * 3 + 0] = 0;
            data_manager->host_data.v[body_offset + current_node * 3 + 1] = 0;
            data_manager->host_data.v[body_offset + current_node * 3 + 2] = 0;)
    }

    printf("Compute_Particle_Volumes %f\n", mass);
    //#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real particle_density = 0;

        LOOPOVERNODES(                                                  //
            real weight = N(xi - current_node_location, inv_bin_edge);  //
            particle_density += grid_mass[current_node] * weight;       //
            )
        particle_density /= (bin_edge * bin_edge * bin_edge);
        volume[p] = mass / particle_density;
        // printf("Volumes: %.20f \n", volume[p], particle_density);
    }
}

void ChMPMContainer::Build_D() {
    LOG(INFO) << "ChMPMContainer::Build_D";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    const real dt = data_manager->settings.step_size;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
}
void ChMPMContainer::Build_b() {
    LOG(INFO) << "ChMPMContainer::Build_b";
    SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, start_node, num_mpm_constraints);
    b_sub = 0;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
}
void ChMPMContainer::Build_E() {
    LOG(INFO) << "ChMPMContainer::Build_E";
    SubVectorType E_sub = blaze::subvector(data_manager->host_data.E, start_node, num_mpm_constraints);
    E_sub = 0;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
}
void ChMPMContainer::GenerateSparsity() {
    //    LOG(INFO) << "ChMPMContainer::GenerateSparsity";
    //    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    //    for (int i = 0; i < num_mpm_nodes; i++) {
    //        AppendRow3(D_T, start_node + i * 6 + 0, body_offset + i * 3, 0);
    //        D_T.finalize(start_node + i * 6 + 0);
    //
    //        AppendRow3(D_T, start_node + i * 6 + 1, body_offset + i * 3, 0);
    //        D_T.finalize(start_node + i * 6 + 1);
    //
    //        AppendRow3(D_T, start_node + i * 6 + 2, body_offset + i * 3, 0);
    //        D_T.finalize(start_node + i * 6 + 2);
    //        ///==================================================================================================================================
    //
    //        AppendRow3(D_T, start_node + i * 6 + 3, body_offset + i * 3, 0);
    //        D_T.finalize(start_node + i * 6 + 3);
    //
    //        AppendRow3(D_T, start_node + i * 6 + 4, body_offset + i * 3, 0);
    //        D_T.finalize(start_node + i * 6 + 4);
    //
    //        AppendRow3(D_T, start_node + i * 6 + 5, body_offset + i * 3, 0);
    //        D_T.finalize(start_node + i * 6 + 5);
    //    }
}
void ChMPMContainer::PreSolve() {
    SubVectorType grid_vel = blaze::subvector(data_manager->host_data.v, body_offset, num_mpm_nodes * 3);

    UpdateRhs();

    DynamicVector<real> delta_v(num_mpm_nodes * 3);
    delta_v = 0;
    Solve(rhs, delta_v);

    grid_vel += delta_v;
}
void ChMPMContainer::PostSolve() {
    const real dt = data_manager->settings.step_size;
    const real3 gravity = data_manager->settings.gravity;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;

    printf("Update_Deformation_Gradient\n");
    //#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        Mat33 velocity_gradient(0);
        LOOPOVERNODES(  //
            real3 g_vel(data_manager->host_data.v[body_offset + current_node * 3 + 0],
                        data_manager->host_data.v[body_offset + current_node * 3 + 1],
                        data_manager->host_data.v[body_offset + current_node * 3 + 2]);
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
}

void ChMPMContainer::Multiply(DynamicVector<real>& v_array, DynamicVector<real>& result_array) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    const real dt = data_manager->settings.step_size;

    printf("Apply Hessian\n");
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];

        Mat33 delta_F(0);
        {
            LOOPOVERNODES(  //

                real3 vnew(v_array[current_node * 3 + 0], v_array[current_node * 3 + 1], v_array[current_node * 3 + 2]);
                real3 vold(grid_vel_old[current_node * 3 + 0], grid_vel_old[current_node * 3 + 1],
                           grid_vel_old[current_node * 3 + 2]);
                real3 v0 = vold + vnew;                                   //
                real3 v1 = dN(xi - current_node_location, inv_bin_edge);  //
                delta_F += OuterProduct(v0, v1);                          //
                )
        }
        delta_F = delta_F * Fe[p];

        real plastic_determinant = Determinant(Fp[p]);
        real J = Determinant(Fe_hat[p]);
        real current_mu = mu * Exp(hardening_coefficient * (1.0 - plastic_determinant));
        real current_lambda = lambda * Exp(hardening_coefficient * (1.0 - plastic_determinant));
        Mat33 Fe_hat_inv_transpose = InverseTranspose(Fe_hat[p]);

        real dJ = J * InnerProduct(Fe_hat_inv_transpose, delta_F);
        Mat33 dF_inverse_transposed = -Fe_hat_inv_transpose * Transpose(delta_F) * Fe_hat_inv_transpose;
        Mat33 dJF_inverse_transposed = dJ * Fe_hat_inv_transpose + J * dF_inverse_transposed;
        Mat33 RD = Rotational_Derivative(Fe_hat[p], delta_F);

        Mat33 volume_Ap_Fe_transpose =
            volume[p] * (2 * current_mu * (delta_F - RD) + (current_lambda * J * dJ) * Fe_hat_inv_transpose +
                         (current_lambda * (J - 1.0)) * dJF_inverse_transposed) *
            Transpose(Fe[p]);
        {
            const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);
            const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);
            const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);

            for (int i = cx - 2; i <= cx + 2; ++i) {
                for (int j = cy - 2; j <= cy + 2; ++j) {
                    for (int k = cz - 2; k <= cz + 2; ++k) {
                        const int current_node = GridHash(i, j, k, bins_per_axis);
                        real3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point);
                        real3 res = volume_Ap_Fe_transpose * dN(xi - current_node_location, inv_bin_edge);  //
#pragma omp atomic
                        result_array[current_node * 3 + 0] += res.x;  //
#pragma omp atomic
                        result_array[current_node * 3 + 1] += res.y;  //
#pragma omp atomic
                        result_array[current_node * 3 + 2] += res.z;  //
                    }
                }
            }
        }
    }
#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        if (grid_mass[i] > C_EPSILON) {
            result_array[i * 3 + 0] =
                grid_mass[i] * (v_array[i * 3 + 0] + grid_vel_old[i * 3 + 0]) + result_array[i * 3 + 0];
            result_array[i * 3 + 1] =
                grid_mass[i] * (v_array[i * 3 + 1] + grid_vel_old[i * 3 + 1]) + result_array[i * 3 + 1];
            result_array[i * 3 + 2] =
                grid_mass[i] * (v_array[i * 3 + 2] + grid_vel_old[i * 3 + 2]) + result_array[i * 3 + 2];
        }
    }
}

void ChMPMContainer::Solve(const DynamicVector<real>& s, DynamicVector<real>& gamma) {
    real lastgoodres;
    real objective_value;
    uint size = num_mpm_nodes * 3;

    temp.resize(size);
    ml.resize(size);
    mg.resize(size);
    mg_p.resize(size);
    ml_candidate.resize(size);
    ms.resize(size);
    my.resize(size);
    mdir.resize(size);
    ml_p.resize(size);

    temp = 0;
    ml = 0;
    mg = 0;
    mg_p = 0;
    ml_candidate = 0;
    ms = 0;
    my = 0;
    mdir = 0;
    ml_p = 0;

    // Tuning of the spectral gradient search
    real a_min = 1e-13;
    real a_max = 1e13;
    real sigma_min = 0.1;
    real sigma_max = 0.9;

    real alpha = 0.0001;

    real gmma = 1e-4;
    real gdiff = 1.0 / pow(size, 2.0);
    bool do_preconditioning = false;
    real neg_BB1_fallback = 0.11;
    real neg_BB2_fallback = 0.12;
    ml = gamma;
    lastgoodres = 10e30;
    real lastgoodfval = 10e30;
    ml_candidate = ml;
    Multiply(ml, temp);
    mg = temp - rhs;
    mg_p = mg;

    real mf_p = 0;
    real mf = 1e29;
    int n_armijo = 10;
    int max_armijo_backtrace = 3;
    std::vector<real> f_hist;

    for (int current_iteration = 0; current_iteration < max_iterations; current_iteration++) {
        temp = (ml - alpha * mg);
        mdir = temp - ml;

        real dTg = (mdir, mg);
        real lambda = 1.0;
        int n_backtracks = 0;
        bool armijo_repeat = true;
        // t2.stop();
        // t3.start();
        while (armijo_repeat) {
            ml_p = ml + lambda * mdir;

            Multiply(ml_p, temp);
            mg_p = temp - rhs;
            mf_p = (ml_p, 0.5 * temp - rhs);

            f_hist.push_back(mf_p);

            real max_compare = 10e29;
            for (int h = 1; h <= Min(current_iteration, n_armijo); h++) {
                real compare = f_hist[current_iteration - h] + gmma * lambda * dTg;
                if (compare > max_compare)
                    max_compare = compare;
            }
            if (mf_p > max_compare) {
                armijo_repeat = true;
                if (current_iteration > 0)
                    mf = f_hist[current_iteration - 1];
                real lambdanew = -lambda * lambda * dTg / (2 * (mf_p - mf - lambda * dTg));
                lambda = Max(sigma_min * lambda, Min(sigma_max * lambda, lambdanew));
                printf("Repeat Armijo, new lambda = %f \n", lambda);
            } else {
                armijo_repeat = false;
            }
            n_backtracks = n_backtracks + 1;
            if (n_backtracks > max_armijo_backtrace)
                armijo_repeat = false;
        }

        ms = ml_p - ml;
        my = mg_p - mg;
        ml = ml_p;
        mg = mg_p;

        if (current_iteration % 2 == 0) {
            real sDs = (ms, ms);
            real sy = (ms, my);
            if (sy <= 0) {
                alpha = neg_BB1_fallback;
            } else {
                alpha = Min(a_max, Max(a_min, sDs / sy));
            }
        } else {
            real sy = (ms, my);
            real yDy = (my, my);
            if (sy <= 0) {
                alpha = neg_BB2_fallback;
            } else {
                alpha = Min(a_max, Max(a_min, sy / yDy));
            }
        }
        temp = ml - gdiff * mg;
        temp = (ml - temp) / (-gdiff);

        real g_proj_norm = Sqrt((temp, temp));

        printf("g_proj_norm %f\n", g_proj_norm);
        if (g_proj_norm < lastgoodres) {
            lastgoodres = g_proj_norm;
            objective_value = mf_p;
            ml_candidate = ml;
        }

        if (lastgoodres < data_manager->settings.solver.tolerance) {
            break;
        }
    }

    gamma = ml_candidate;
}

void ChMPMContainer::UpdateRhs() {
    rhs = 0;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    const real dt = data_manager->settings.step_size;
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];

        Mat33 PED = Potential_Energy_Derivative(Fe_hat[p], Fp[p], mu, lambda, hardening_coefficient);

        Mat33 vPEDFepT = volume[p] * MultTranspose(PED, Fe[p]);
        real JE = Determinant(Fe[p]);                                       //
        real JP = Determinant(Fp[p]);                                       //
        LOOPOVERNODES(                                                      //
            real3 d_weight = dN(xi - current_node_location, inv_bin_edge);  //
            real3 force = (vPEDFepT * d_weight) / (JE * JP);

            rhs[i * 3 + 0] -= force.x;  //
            rhs[i * 3 + 1] -= force.y;  //
            rhs[i * 3 + 2] -= force.z;  //

            )
    }

    //    for (int i = 0; i < num_mpm_nodes; i++) {
    //        rhs[i * 3 + 0] = grid_mass[i] * data_manager->host_data.v[body_offset + i * 3 + 0];
    //        rhs[i * 3 + 1] = grid_mass[i] * data_manager->host_data.v[body_offset + i * 3 + 1];
    //        rhs[i * 3 + 2] = grid_mass[i] * data_manager->host_data.v[body_offset + i * 3 + 2];
    //    }
}

}  // END_OF_NAMESPACE____

/////////////////////
