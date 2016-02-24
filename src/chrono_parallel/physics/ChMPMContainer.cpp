
#include <algorithm>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/lcp/MPMUtils.h"
#include <chrono_parallel/collision/ChBroadphaseUtils.h>
#include <thrust/transform_reduce.h>
#include "chrono_parallel/constraints/ChConstraintUtils.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

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
//
class CH_PARALLEL_API ChShurProductMPM : public ChShurProduct {
  public:
    ChShurProductMPM() {}
    virtual ~ChShurProductMPM() {}
    void Setup(ChParallelDataManager data_container_) {}
    void Setup(ChMPMContainer* mpm_container, ChParallelDataManager* manager) {
        data_manager = manager;
        container = mpm_container;
    }

    // Perform the Multiplication
    void operator()(const DynamicVector<real>& v_array, DynamicVector<real>& result_array) {
        custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
        custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
        uint num_mpm_markers = container->num_mpm_markers;
        uint num_mpm_nodes = container->num_mpm_nodes;
        const real dt = data_manager->settings.step_size;
        real hardening_coefficient = container->hardening_coefficient;
        real mu = container->mu;
        real lambda = container->lambda;
        real inv_bin_edge = container->inv_bin_edge;
        real bin_edge = container->bin_edge;
        int3 bins_per_axis = container->bins_per_axis;
        real3 min_bounding_point = container->min_bounding_point;

#pragma omp parallel for
        for (int p = 0; p < num_mpm_markers; p++) {
            const real3 xi = pos_marker[p];

            Mat33 delta_F(0);
            {
                LOOPOVERNODES(  //

                    real3 vnew(v_array[current_node * 3 + 0], v_array[current_node * 3 + 1],
                               v_array[current_node * 3 + 2]);
                    real3 vold(container->old_vel_node_mpm[current_node * 3 + 0],
                               container->old_vel_node_mpm[current_node * 3 + 1],
                               container->old_vel_node_mpm[current_node * 3 + 2]);
                    real3 v0 = vold + vnew;                                   //
                    real3 v1 = dN(xi - current_node_location, inv_bin_edge);  //
                    delta_F += OuterProduct(v0, v1);                          //
                    )
            }
            delta_F = delta_F * container->marker_Fe[p];

            // real plastic_determinant = container->det_marker_Fp[p];
            Mat33 R = container->SVD_Fe_hat_R[p];
            Mat33 S = container->SVD_Fe_hat_S[p];

            real plastic_determinant = Determinant(container->marker_Fp[p]);                          // Constant
            real J = Determinant(container->marker_Fe_hat[p]);                                        // Constant
            real current_mu = mu * Exp(hardening_coefficient * (1.0 - plastic_determinant));          // Constant
            real current_lambda = lambda * Exp(hardening_coefficient * (1.0 - plastic_determinant));  // Constant

            // Mat33 Fe_hat_inv_transpose;
            // if (J > 0.0) {
            //    Fe_hat_inv_transpose = AdjointTranspose(data_manager->host_data.marker_Fe_hat[p]) * real(1.0 / J);
            //} else {
            //    Fe_hat_inv_transpose = Mat33(0);
            //}

            // Mat33 Fe_hat_inv_transpose = InverseTranspose(data_manager->host_data.marker_Fe_hat[p]);  // Constant
            // Mat33 U, V;
            // real3 E;
            // SVD(data_manager->host_data.marker_Fe_hat[p], U, E, V);  // Constant
            // Mat33 R = MultTranspose(U, V);                           // Constant
            // Mat33 S = V * MultTranspose(Mat33(E), V);                // Constant

            // real dJ = J * InnerProduct(Fe_hat_inv_transpose, delta_F);
            // Mat33 dF_inverse_transposed = -Fe_hat_inv_transpose * TransposeMult(delta_F, Fe_hat_inv_transpose);
            // Mat33 dJF_inverse_transposed = dJ * Fe_hat_inv_transpose + J * dF_inverse_transposed;

            Mat33 W = TransposeMult(R, delta_F);
            Mat33 RD = Solve_dR(R, S, W);

            Mat33 volume_Ap_Fe_transpose =
                container->marker_volume[p] * (2 * current_mu * (delta_F - RD)) * Transpose(container->marker_Fe[p]);
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
            if (container->node_mass[i] > C_EPSILON) {
                result_array[i * 3 + 0] =
                    container->node_mass[i] * (v_array[i * 3 + 0] + container->old_vel_node_mpm[i * 3 + 0]) +
                    result_array[i * 3 + 0];
                result_array[i * 3 + 1] =
                    container->node_mass[i] * (v_array[i * 3 + 1] + container->old_vel_node_mpm[i * 3 + 1]) +
                    result_array[i * 3 + 1];
                result_array[i * 3 + 2] =
                    container->node_mass[i] * (v_array[i * 3 + 2] + container->old_vel_node_mpm[i * 3 + 2]) +
                    result_array[i * 3 + 2];
            }
        }
    }

    // Pointer to the system's data manager
    ChParallelDataManager* data_manager;
    ChMPMContainer* container;
};

ChMPMContainer::ChMPMContainer(ChSystemParallelDVI* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->AddMPMContainer(this);
    num_mpm_markers = 0;
    num_mpm_nodes = 0;
    max_iterations = 10;
    real mass = 1;
    real mu = 1;
    real hardening_coefficient = 1;
    real lambda = 1;
    real theta_s = 1;
    real theta_c = 1;
    real alpha = 1;
    solver = new ChSolverBB();
    solver->Setup(data_manager);
}
ChMPMContainer::~ChMPMContainer() {}

void ChMPMContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);
    start_node = start_constraint;
    body_offset = num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + num_fea_nodes * 3;
    min_bounding_point = data_manager->measures.collision.mpm_min_bounding_point;
    max_bounding_point = data_manager->measures.collision.mpm_max_bounding_point;
    num_rigid_mpm_contacts = data_manager->num_rigid_fluid_contacts;
    num_mpm_constraints = num_rigid_mpm_contacts * 3;
}

void ChMPMContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;

    pos_marker.insert(pos_marker.end(), positions.begin(), positions.end());
    vel_marker.insert(vel_marker.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_marker.resize(pos_marker.size());
    data_manager->num_fluid_bodies = pos_marker.size();
    num_mpm_markers = pos_marker.size();

    marker_Fe.resize(data_manager->num_fluid_bodies);
    marker_Fe_hat.resize(data_manager->num_fluid_bodies);
    marker_Fp.resize(data_manager->num_fluid_bodies);
    marker_delta_F.resize(data_manager->num_fluid_bodies);
    marker_volume.resize(data_manager->num_fluid_bodies);

    marker_Fp[data_manager->num_fluid_bodies - 1] = Mat33(1);
}
void ChMPMContainer::ComputeDOF() {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    real3& min_bounding_point = data_manager->measures.collision.mpm_min_bounding_point;
    real3& max_bounding_point = data_manager->measures.collision.mpm_max_bounding_point;

    bbox res(pos_marker[0], pos_marker[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(pos_marker.begin(), pos_marker.end(), unary_op, res, binary_op);

    res.first.x = kernel_radius * Round(res.first.x / kernel_radius);
    res.first.y = kernel_radius * Round(res.first.y / kernel_radius);
    res.first.z = kernel_radius * Round(res.first.z / kernel_radius);

    res.second.x = kernel_radius * Round(res.second.x / kernel_radius);
    res.second.y = kernel_radius * Round(res.second.y / kernel_radius);
    res.second.z = kernel_radius * Round(res.second.z / kernel_radius);

    max_bounding_point = real3(res.second.x, res.second.y, res.second.z) + kernel_radius * 8;
    min_bounding_point = real3(res.first.x, res.first.y, res.first.z) - kernel_radius * 6;

    real3 diag = max_bounding_point - min_bounding_point;
    bin_edge = kernel_radius * 2;
    bins_per_axis = int3(diag / bin_edge);
    inv_bin_edge = real(1.) / bin_edge;
    uint grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;
    num_mpm_nodes = grid_size;

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Compute DOF [%d] [%d %d %d] [%f] %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);
}

void ChMPMContainer::Update(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    const real dt = data_manager->settings.step_size;
    Setup(0);

    node_mass.resize(num_mpm_nodes);
    old_vel_node_mpm.resize(num_mpm_nodes * 3);
    rhs.resize(num_mpm_nodes * 3);
    grid_vel.resize(num_mpm_nodes * 3);

    std::fill(node_mass.begin(), node_mass.end(), 0);
    std::fill(grid_vel.begin(), grid_vel.end(), 0);

#pragma omp paralle for
    for (int i = 0; i < num_mpm_markers; i++) {
        data_manager->host_data.v[body_offset + i * 3 + 0] = vel_marker[i].x;
        data_manager->host_data.v[body_offset + i * 3 + 1] = vel_marker[i].y;
        data_manager->host_data.v[body_offset + i * 3 + 2] = vel_marker[i].z;
        data_manager->host_data.hf[body_offset + i * 3 + 0] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = 0;
    }

    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        const real3 vi = vel_marker[p];
        // printf("marker_vel: [%.20f %.20f %.20f] \n", vi.x, vi.y, vi.z);
        LOOPOVERNODES(                                                         //
            real weight = N(xi - current_node_location, inv_bin_edge) * mass;  //
            node_mass[current_node] += weight;                                 //
            grid_vel[current_node * 3 + 0] += weight * vi.x;                   //
            grid_vel[current_node * 3 + 1] += weight * vi.y;                   //
            grid_vel[current_node * 3 + 2] += weight * vi.z;                   //
            )
    }
// normalize weights for the velocity (to conserve momentum)
#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        if (node_mass[i] > C_EPSILON) {
            grid_vel[i * 3 + 0] /= node_mass[i];
            grid_vel[i * 3 + 1] /= node_mass[i];
            grid_vel[i * 3 + 2] /= node_mass[i];
            old_vel_node_mpm[i * 3 + 0] = grid_vel[i * 3 + 0];
            old_vel_node_mpm[i * 3 + 1] = grid_vel[i * 3 + 1];
            old_vel_node_mpm[i * 3 + 2] = grid_vel[i * 3 + 2];
        }
    }
    SVD_Fe_hat_R.resize(num_mpm_markers);
    SVD_Fe_hat_S.resize(num_mpm_markers);
    printf("Compute_Elastic_Deformation_Gradient_Hat\n");
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        marker_Fe_hat[p] = Mat33(1.0);
        Mat33 Fe_hat_t(1);
        LOOPOVERNODES(  //
            real3 vel(grid_vel[i * 3 + 0], grid_vel[i * 3 + 1], grid_vel[i * 3 + 2]);
            real3 kern = dN(xi - current_node_location, inv_bin_edge); Fe_hat_t += OuterProduct(dt * vel, kern);  //
            )
        Mat33 Fe_hat = Fe_hat_t * marker_Fe[p];
        marker_Fe_hat[p] = Fe_hat;
        Mat33 U, V;
        real3 E;
        SVD(Fe_hat, U, E, V);
        SVD_Fe_hat_R[p] = MultTranspose(U, V);
        SVD_Fe_hat_S[p] = V * MultTranspose(Mat33(E), V);
    }

    //    printf("Compute_Grid_Forces\n");

    //    for (int p = 0; p < num_mpm_markers; p++) {
    //        const real3 xi = pos_marker[p];
    //
    //        Mat33 PED = Potential_Energy_Derivative(data_manager->host_data.marker_Fe_hat[p],
    //        data_manager->host_data.marker_Fp[p], mu, lambda, hardening_coefficient);
    //
    //        Mat33 vPEDFepT = data_manager->host_data.marker_volume[p] * MultTranspose(PED,
    //        data_manager->host_data.marker_Fe[p]);
    //        real JE = Determinant(data_manager->host_data.marker_Fe[p]);                                       //
    //        real JP = Determinant(data_manager->host_data.marker_Fp[p]);                                       //
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
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    custom_vector<real>& old_vel_node_mpm = old_vel_node_mpm;
    printf("Update_Particle_Velocities\n");
    //#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real3 new_vel;

        new_vel.x = data_manager->host_data.v[body_offset + p * 3 + 0];
        new_vel.y = data_manager->host_data.v[body_offset + p * 3 + 1];
        new_vel.z = data_manager->host_data.v[body_offset + p * 3 + 2];

        real speed = Length(new_vel);
        if (speed > max_velocity) {
            new_vel = new_vel * max_velocity / speed;
        }

        vel_marker[p] = new_vel;
        pos_marker[p] += new_vel * data_manager->settings.step_size;
    }
}

int ChMPMContainer::GetNumConstraints() {
    return data_manager->num_rigid_fluid_contacts * 3;
}

int ChMPMContainer::GetNumNonZeros() {
    return data_manager->num_rigid_fluid_contacts * 3 * 9;
}

void ChMPMContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    real inv_mass = 1.0 / mass;
    for (int i = 0; i < num_mpm_markers; i++) {
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
    for (int i = 0; i < num_mpm_markers; i++) {
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
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Initialize [%d] [%d %d %d] [%f] %d\n", num_mpm_nodes, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);

    marker_volume.resize(num_mpm_markers);
    node_mass.resize(num_mpm_nodes);
    std::fill(node_mass.begin(), node_mass.end(), 0);

    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        const real3 vi = vel_marker[p];

        LOOPOVERNODES(                                                                //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge) * mass;  //
            node_mass[current_node] += weight;                                        //
            )
    }

    printf("Compute_Particle_Volumes %f\n", mass);
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real particle_density = 0;

        LOOPOVERNODES(                                                  //
            real weight = N(xi - current_node_location, inv_bin_edge);  //
            particle_density += node_mass[current_node] * weight;       //
            )
        particle_density /= (bin_edge * bin_edge * bin_edge);
        marker_volume[p] = mass / particle_density;
        // printf("Volumes: %.20f \n", data_manager->host_data.marker_volume[p], particle_density);
    }
}

void ChMPMContainer::Build_D() {
    LOG(INFO) << "ChMPMContainer::Build_D";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    const real dt = data_manager->settings.step_size;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    //
    //    if (num_rigid_mpm_contacts > 0) {
    //        custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;
    //        custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;
    //
    //        // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
    //        custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_mpm;
    //        custom_vector<real3>& norm = data_manager->host_data.norm_rigid_mpm;
    //        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_mpm;
    //        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_mpm;
    //#pragma omp parallel for
    //        for (int p = 0; p < num_mpm_markers; p++) {
    //            int start = contact_counts[p];
    //            int end = contact_counts[p + 1];
    //            for (int index = start; index < end; index++) {
    //                int i = index - start;  // index that goes from 0
    //                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
    //                real3 U = norm[p * max_rigid_neighbors + i], V, W;
    //                Orthogonalize(U, V, W);
    //                real3 T1, T2, T3;
    //                Compute_Jacobian(rot_rigid[rigid], U, V, W, cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid],
    //                T1,
    //                                 T2, T3);
    //
    //                SetRow6Check(D_T, start_row + index + 0, rigid * 6, -U, T1);
    //                SetRow6Check(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 0, rigid * 6, -V, T2);
    //                SetRow6Check(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 1, rigid * 6, -W, T3);
    //
    //                SetRow3Check(D_T, start_row + index + 0, body_offset + p * 3, U);
    //                SetRow3Check(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 0, body_offset + p * 3, V);
    //                SetRow3Check(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 1, body_offset + p * 3, W);
    //            }
    //        }
    //    }
}
void ChMPMContainer::Build_b() {
    LOG(INFO) << "ChMPMContainer::Build_b";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    real dt = data_manager->settings.step_size;
    //    if (num_rigid_mpm_contacts > 0) {
    //        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_mpm;
    //        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_mpm;
    //
    //#pragma omp parallel for
    //        for (int p = 0; p < num_mpm_markers; p++) {
    //            int start = contact_counts[p];
    //            int end = contact_counts[p + 1];
    //            for (int index = start; index < end; index++) {
    //                int i = index - start;  // index that goes from 0
    //                real depth = data_manager->host_data.dpth_rigid_mpm[p * max_rigid_neighbors + i];
    //
    //                real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
    //                // printf("boundary: [%f]\n", bi);
    //                data_manager->host_data.b[start_row + index + 0] = bi;
    //                data_manager->host_data.b[start_row + num_rigid_mpm_contacts + index * 2 + 0] = 0;
    //                data_manager->host_data.b[start_row + num_rigid_mpm_contacts + index * 2 + 1] = 0;
    //            }
    //        }
    //    }
}
void ChMPMContainer::Build_E() {
    //    LOG(INFO) << "ChMPMContainer::Build_E";
    //    SubVectorType E_sub = blaze::subvector(data_manager->host_data.E, start_row, num_mpm_constraints);
    //    E_sub = 0;
    //    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
}
void ChMPMContainer::GenerateSparsity() {
    LOG(INFO) << "ChMPMContainer::GenerateSparsity";
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
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

    //    if (num_rigid_mpm_contacts > 0) {
    //        LOG(INFO) << "ChConstraintRigidMPM::GenerateSparsity " << num_rigid_mpm_contacts;
    //
    //        int index_t = 0;
    //
    //        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_mpm;
    //        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_mpm;
    //
    //        for (int p = 0; p < num_mpm_markers; p++) {
    //            int start = contact_counts[p];
    //            int end = contact_counts[p + 1];
    //            for (int index = start; index < end; index++) {
    //                int i = index - start;  // index that goes from 0
    //                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
    //
    //                AppendRow6(D_T, start_row + index + 0, rigid * 6, 0);
    //                AppendRow3(D_T, start_row + index + 0, body_offset + p * 3, 0);
    //                D_T.finalize(start_row + index + 0);
    //            }
    //        }
    //        for (int p = 0; p < num_mpm_markers; p++) {
    //            int start = contact_counts[p];
    //            int end = contact_counts[p + 1];
    //            for (int index = start; index < end; index++) {
    //                int i = index - start;  // index that goes from 0
    //                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
    //
    //                AppendRow6(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 0, rigid * 6, 0);
    //                AppendRow3(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 0, body_offset + p * 3, 0);
    //                D_T.finalize(start_row + num_rigid_mpm_contacts + index * 2 + 0);
    //
    //                AppendRow6(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 1, rigid * 6, 0);
    //                AppendRow3(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 1, body_offset + p * 3, 0);
    //
    //                D_T.finalize(start_row + num_rigid_mpm_contacts + index * 2 + 1);
    //            }
    //        }
    //    }
}
void ChMPMContainer::PreSolve() {
    UpdateRhs();
    // CollideGrid();

    DynamicVector<real> delta_v(num_mpm_nodes * 3);
    delta_v = 0;
    Solve(rhs, delta_v);

    grid_vel += delta_v;

    const real dt = data_manager->settings.step_size;
    const real3 gravity = data_manager->settings.gravity;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;

    custom_vector<real>& old_vel_node_mpm = old_vel_node_mpm;
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real3 V_flip = vel_marker[p];

        real3 V_pic = real3(0.0);
        LOOPOVERNODES(                                                                                       //
            real weight = N(xi - current_node_location, inv_bin_edge);                                       //
            V_pic.x += grid_vel[current_node * 3 + 0] * weight;                                              //
            V_pic.y += grid_vel[current_node * 3 + 1] * weight;                                              //
            V_pic.z += grid_vel[current_node * 3 + 2] * weight;                                              //
            V_flip.x += (grid_vel[current_node * 3 + 0] - old_vel_node_mpm[current_node * 3 + 0]) * weight;  //
            V_flip.y += (grid_vel[current_node * 3 + 1] - old_vel_node_mpm[current_node * 3 + 1]) * weight;  //
            V_flip.z += (grid_vel[current_node * 3 + 2] - old_vel_node_mpm[current_node * 3 + 2]) * weight;  //
            )

        real3 new_vel = (1.0 - alpha) * V_pic + alpha * V_flip;

        real speed = Length(new_vel);
        if (speed > max_velocity) {
            new_vel = new_vel * max_velocity / speed;
        }
        data_manager->host_data.v[body_offset + p * 3 + 0] = new_vel.x;
        data_manager->host_data.v[body_offset + p * 3 + 1] = new_vel.y;
        data_manager->host_data.v[body_offset + p * 3 + 2] = new_vel.z;

        vel_marker[p] = new_vel;
        // pos_marker[p] += new_vel * data_manager->settings.step_size;
    }
}
void ChMPMContainer::PostSolve() {
    //    UpdateRhs();
    //
    //    DynamicVector<real> delta_v(num_mpm_nodes * 3);
    //    delta_v = 0;
    //    Solve(rhs, delta_v);
    //
    //    grid_vel += delta_v;

    const real dt = data_manager->settings.step_size;
    const real3 gravity = data_manager->settings.gravity;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;

    printf("Update_Deformation_Gradient\n");
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        Mat33 velocity_gradient(0);
        LOOPOVERNODES(  //
            real3 g_vel(grid_vel[current_node * 3 + 0], grid_vel[current_node * 3 + 1], grid_vel[current_node * 3 + 2]);
            velocity_gradient += OuterProduct(g_vel, dN(xi - current_node_location, inv_bin_edge));)

        Mat33 Fe_tmp = (Mat33(1.0) + dt * velocity_gradient) * marker_Fe[p];
        Mat33 F_tmp = Fe_tmp * marker_Fp[p];
        Mat33 U, V;
        real3 E;
        SVD(Fe_tmp, U, E, V);
        real3 E_clamped;

        E_clamped.x = Clamp(E.x, 1.0 - theta_c, 1.0 + theta_s);
        E_clamped.y = Clamp(E.y, 1.0 - theta_c, 1.0 + theta_s);
        E_clamped.z = Clamp(E.z, 1.0 - theta_c, 1.0 + theta_s);

        marker_Fe[p] = U * MultTranspose(Mat33(E_clamped), V);
        // Inverse of Diagonal E_clamped matrix is 1/E_clamped
        marker_Fp[p] = V * MultTranspose(Mat33(1.0 / E_clamped), U) * F_tmp;
    }
}

void ChMPMContainer::Solve(const DynamicVector<real>& rhs, DynamicVector<real>& delta_v) {
    ChShurProductMPM Multiply;
    Multiply.Setup(this, data_manager);
    ChProjectNone ProjectNone;
    solver->Solve(Multiply, ProjectNone, max_iterations, num_mpm_nodes * 3, rhs, delta_v);
}

void ChMPMContainer::UpdateRhs() {
    rhs = 0;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    const real dt = data_manager->settings.step_size;
    // contact forces
    custom_vector<real3> node_force(num_mpm_nodes);

    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];

        Mat33 PED =
            Potential_Energy_Derivative_Deviatoric(marker_Fe_hat[p], marker_Fp[p], mu, lambda, hardening_coefficient);

        Mat33 vPEDFepT = marker_volume[p] * MultTranspose(PED, marker_Fe[p]);
        real JE = Determinant(marker_Fe[p]);  //
        real JP = Determinant(marker_Fp[p]);  //
        real3 contact_force = real3(0);
        if (contact_forces.size() == num_mpm_markers) {
            contact_force.x = contact_forces[p * 3 + 0];
            contact_force.y = contact_forces[p * 3 + 1];
            contact_force.z = contact_forces[p * 3 + 2];
        }
        LOOPOVERNODES(                                                      //
            real3 d_weight = dN(xi - current_node_location, inv_bin_edge);  //
            real3 force = (vPEDFepT * d_weight) / (JE * JP);

            node_force[current_node] -= force;

            rhs[current_node * 3 + 0] -= dt * force.x;  // - dt * contact_force.x;  //
            rhs[current_node * 3 + 1] -= dt * force.y;  // - dt * contact_force.y;  //
            rhs[current_node * 3 + 2] -= dt * force.z;  //- dt * contact_force.z;  //
            )
    }
    //    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_mpm;
    //    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_mpm;
    //    custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_mpm;
    //    for (int p = 0; p < num_mpm_markers; p++) {
    //        const real3 xi = pos_marker[p];
    //
    //        int start = contact_counts[p];
    //        int end = contact_counts[p + 1];
    //
    //        real3 f = real3(0);
    //        LOOPOVERNODES(                                                  //
    //            real weight = N(xi - current_node_location, inv_bin_edge);  //
    //            f += weight * node_force[current_node];                     //
    //            )
    //
    //        for (int index = start; index < end; index++) {
    //            int i = index - start;  // index that goes from 0
    //            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
    //            real3 point = cpta[p * max_rigid_neighbors + i];
    //
    //            real3 torque = Cross(point - data_manager->host_data.pos_rigid[rigid], f);
    //
    //            data_manager->host_data.hf[rigid * 6 + 0] += f.x;
    //            data_manager->host_data.hf[rigid * 6 + 1] += f.y;
    //            data_manager->host_data.hf[rigid * 6 + 2] += f.z;
    //
    //            data_manager->host_data.hf[rigid * 6 + 3] += torque.x;
    //            data_manager->host_data.hf[rigid * 6 + 4] += torque.y;
    //            data_manager->host_data.hf[rigid * 6 + 5] += torque.z;
    //        }
    //    }
}

}  // END_OF_NAMESPACE____

/////////////////////
