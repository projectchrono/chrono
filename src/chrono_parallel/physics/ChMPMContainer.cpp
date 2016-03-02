
#include <algorithm>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/physics/MPMUtils.h"
#include <chrono_parallel/collision/ChBroadphaseUtils.h>
#include <thrust/transform_reduce.h>
#include <thrust/sort.h>
#include <thrust/count.h>
#include <thrust/iterator/constant_iterator.h>
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
        container->volume_Ap_Fe_transpose.resize(num_mpm_markers);

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

            Mat33 R = container->SVD_Fe_hat_R[p];
            Mat33 S = container->SVD_Fe_hat_S[p];

            real plastic_determinant = Determinant(container->marker_Fp[p]);                  // Constant
            real current_mu = mu * Exp(hardening_coefficient * (1.0 - plastic_determinant));  // Constant

            Mat33 W = TransposeMult(R, delta_F);
            Mat33 RD = Solve_dR(R, S, W);
            container->volume_Ap_Fe_transpose[p] =
                container->marker_volume[p] * MultTranspose(2 * current_mu * (delta_F - RD), container->marker_Fe[p]);
        }

//        for (int p = 0; p < num_mpm_markers; p++) {
//            const real3 xi = pos_marker[p];
//            LOOPOVERNODES(                                                                             //
//                real3 res = volume_Ap_Fe_transpose[p] * dN(xi - current_node_location, inv_bin_edge);  //
//                result_array[current_node * 3 + 0] += res.x;                                           //
//                result_array[current_node * 3 + 1] += res.y;                                           //
//                result_array[current_node * 3 + 2] += res.z;                                           //
//                // printf("A: %d %d\n", current_node, p);
//                )
//        }

#pragma omp parallel for
        for (int index = 0; index < container->num_mpm_nodes_active; index++) {
            uint start = container->node_start_index[index];
            uint end = container->node_start_index[index + 1];
            const int current_node = container->node_particle_mapping[index];
            int3 g = GridDecode(current_node, bins_per_axis);
            real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
            real3 res = real3(0);
            for (uint i = start; i < end; i++) {
                int p = container->particle_number[i];
                res +=
                    container->volume_Ap_Fe_transpose[p] * dN(pos_marker[p] - current_node_location, inv_bin_edge);  //
            }
            real mass = container->node_mass[current_node];

            if (mass > 0) {
                result_array[current_node * 3 + 0] +=
                    res.x + mass * (v_array[current_node * 3 + 0] + container->old_vel_node_mpm[current_node * 3 + 0]);
                result_array[current_node * 3 + 1] +=
                    res.y + mass * (v_array[current_node * 3 + 1] + container->old_vel_node_mpm[current_node * 3 + 1]);
                result_array[current_node * 3 + 2] +=
                    res.z + mass * (v_array[current_node * 3 + 2] + container->old_vel_node_mpm[current_node * 3 + 2]);
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
    cohesion = 0;
}
ChMPMContainer::~ChMPMContainer() {}

void ChMPMContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);
    body_offset = num_rigid_bodies * 6 + num_shafts;
    min_bounding_point = data_manager->measures.collision.mpm_min_bounding_point;
    max_bounding_point = data_manager->measures.collision.mpm_max_bounding_point;
    num_mpm_contacts = (num_fluid_contacts - num_fluid_bodies) / 2;
    start_boundary = start_constraint;
    if (contact_mu == 0) {
        start_contact = start_constraint + num_rigid_fluid_contacts;
    } else {
        start_contact = start_constraint + num_rigid_fluid_contacts * 3;
    }
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

    std::fill(marker_Fe.begin(), marker_Fe.end(), Mat33(1));
    std::fill(marker_Fp.begin(), marker_Fp.end(), Mat33(1));
}
void ChMPMContainer::ComputeDOF() {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
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
    // Note that 8 and 6 are the optimal values here, 6 and 4 will cause memory errors in valgrind
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
    printf("Compute DOF [%d] [%d %d %d] [%f] %d %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_nodes, num_mpm_markers);
}

void ChMPMContainer::Update(double ChTime) {
    Setup(0);
    ComputeDOF();
    node_mass.resize(num_mpm_nodes);
    old_vel_node_mpm.resize(num_mpm_nodes * 3);
    rhs.resize(num_mpm_nodes * 3);
    grid_vel.resize(num_mpm_nodes * 3);

    std::fill(node_mass.begin(), node_mass.end(), 0);
    std::fill(grid_vel.begin(), grid_vel.end(), 0);

    real3 g_acc = data_manager->settings.gravity;
    real3 h_gravity = data_manager->settings.step_size * mass * g_acc;
#pragma omp parallel for
    for (int i = 0; i < num_mpm_markers; i++) {
        // Velocity already set in the fluid fluid contact function
        data_manager->host_data.hf[body_offset + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = h_gravity.z;
    }

    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;

    particle_node_mapping.resize(num_mpm_markers * 125);
    particle_number.resize(num_mpm_markers * 125);
// For each particle determine nodes
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        int counter = 0;

        LOOPOVERNODES(                                                //
            particle_node_mapping[p * 125 + counter] = current_node;  //
            particle_number[p * 125 + counter] = p;                   //
            counter++;)

        // printf("Counter: %d\n", counter);
    }

    // After sorting we have a list of nodes in order with the particle at each node
    thrust::sort_by_key(particle_node_mapping.begin(), particle_node_mapping.end(), particle_number.begin());

    //    for (int i = 0; i < particle_node_mapping.size(); i++) {
    //        printf("%d %d \n", particle_node_mapping[i], particle_number[i]);
    //    }

    // Next we need to count the number of particles per node
    node_particle_mapping.resize(num_mpm_markers * 125);
    node_start_index.resize(num_mpm_nodes);
    // Input sorted list of nodes and the output list, get the start_index which is the number of particles in the node
    num_mpm_nodes_active = Run_Length_Encode(particle_node_mapping, node_particle_mapping, node_start_index);
    node_particle_mapping.resize(num_mpm_nodes_active);

    node_start_index.resize(num_mpm_nodes_active + 1);
    node_start_index[num_mpm_nodes_active] = 0;
    Thrust_Exclusive_Scan(node_start_index);

    //    custom_vector<int> test(num_mpm_markers);
    //    std::fill(test.begin(), test.end(), 0);
    //
    //    // CHECK we should get 125 for each particle
    //    //#pragma omp parallel for
    //    for (int index = 0; index < num_mpm_nodes_active; index++) {
    //        uint start = node_start_index[index];
    //        uint end = node_start_index[index + 1];
    //        const int current_node = node_particle_mapping[index];
    //        int3 g = GridDecode(current_node, bins_per_axis);
    //        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
    //
    //        for (uint i = start; i < end; i++) {
    //            int p = particle_number[i];
    //            test[p]++;
    //        }
    //    }
    //
    //    for (int i = 0; i < num_mpm_markers; i++) {
    //        printf("%d \n", test[i]);
    //    }
}

void ChMPMContainer::UpdatePosition(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    printf("Update_Particle_Velocities\n");

#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        real3 new_vel;
        int original_index = data_manager->host_data.particle_indices_3dof[p];
        new_vel.x = data_manager->host_data.v[body_offset + p * 3 + 0];
        new_vel.y = data_manager->host_data.v[body_offset + p * 3 + 1];
        new_vel.z = data_manager->host_data.v[body_offset + p * 3 + 2];

        real speed = Length(new_vel);
        if (speed > max_velocity) {
            new_vel = new_vel * max_velocity / speed;
        }

        vel_marker[original_index] = new_vel;
        pos_marker[original_index] += new_vel * data_manager->settings.step_size;
    }
}

int ChMPMContainer::GetNumConstraints() {
    int num_fluid_fluid = (data_manager->num_fluid_contacts - data_manager->num_fluid_bodies) / 2;
    if (contact_mu == 0) {
        num_fluid_fluid += data_manager->num_rigid_fluid_contacts;
    } else {
        num_fluid_fluid += data_manager->num_rigid_fluid_contacts * 3;
    }
    return num_fluid_fluid;
}

int ChMPMContainer::GetNumNonZeros() {
    int nnz_fluid_fluid = 0;
    nnz_fluid_fluid = (data_manager->num_fluid_contacts - data_manager->num_fluid_bodies) / 2 * 6;
    if (contact_mu == 0) {
        nnz_fluid_fluid += 9 * data_manager->num_rigid_fluid_contacts;
    } else {
        nnz_fluid_fluid += 9 * 3 * data_manager->num_rigid_fluid_contacts;
    }
    return nnz_fluid_fluid;
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
    printf("ChMPMContainer::Initialize()\n");
    ComputeDOF();
    const real dt = data_manager->settings.step_size;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;

    marker_volume.resize(num_mpm_markers);
    node_mass.resize(num_mpm_nodes);
    std::fill(node_mass.begin(), node_mass.end(), 0);
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        LOOPOVERNODES(                                                                //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge) * mass;  //
            node_mass[current_node] += weight;                                        //
            )
    }

    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real particle_density = 0;

        LOOPOVERNODES(                                                  //
            real weight = N(xi - current_node_location, inv_bin_edge);  //
            particle_density += node_mass[current_node] * weight;       //
            )
        particle_density /= (bin_edge * bin_edge * bin_edge);
        marker_volume[p] = mass / particle_density;
        // printf("Volumes: %.20f \n", marker_volume[p], particle_density);
    }
}

void ChMPMContainer::Build_D() {
    LOG(INFO) << "ChMPMContainer::Build_D " << num_rigid_fluid_contacts << " " << num_mpm_contacts;
    custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;
    const real dt = data_manager->settings.step_size;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    if (num_rigid_fluid_contacts > 0) {
        custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;
        custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;

        // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
        custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_fluid;
        custom_vector<real3>& norm = data_manager->host_data.norm_rigid_fluid;
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        if (contact_mu == 0) {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                                      real3 U = norm[p * max_rigid_neighbors + i]; real3 V; real3 W;  //
                                      Orthogonalize(U, V, W); real3 T1; real3 T2; real3 T3;           //
                                      Compute_Jacobian(rot_rigid[rigid], U, V, W,
                                                       cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1, T2,
                                                       T3);

                                      SetRow6Check(D_T, start_boundary + index + 0, rigid * 6, -U, T1);
                                      SetRow3Check(D_T, start_boundary + index + 0, body_offset + p * 3, U););
        } else {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                real3 U = norm[p * max_rigid_neighbors + i]; real3 V; real3 W;  //
                Orthogonalize(U, V, W); real3 T1; real3 T2; real3 T3;           //
                Compute_Jacobian(rot_rigid[rigid], U, V, W, cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1,
                                 T2, T3);

                SetRow6Check(D_T, start_boundary + index + 0, rigid * 6, -U, T1);
                SetRow6Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, rigid * 6, -V, T2);
                SetRow6Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, rigid * 6, -W, T3);

                SetRow3Check(D_T, start_boundary + index + 0, body_offset + p * 3, U);
                SetRow3Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, body_offset + p * 3, V);
                SetRow3Check(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, body_offset + p * 3, W););
        }
    }

    if (num_mpm_contacts > 0) {
        int index = 0;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        Loop_Over_Fluid_Neighbors(real3 U = -Normalize(xij); real3 V; real3 W;  //
                                  Orthogonalize(U, V, W);
                                  SetRow3Check(D_T, start_contact + index + 0, body_offset + body_a * 3, -U);
                                  SetRow3Check(D_T, start_contact + index + 0, body_offset + body_b * 3, U););
    }
}
void ChMPMContainer::Build_b() {
    LOG(INFO) << "ChMPMContainer::Build_b";
    real dt = data_manager->settings.step_size;
    DynamicVector<real>& b = data_manager->host_data.b;
    if (num_rigid_fluid_contacts > 0) {
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        if (contact_mu == 0) {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(
                real depth = data_manager->host_data.dpth_rigid_fluid[p * max_rigid_neighbors + i];

                real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed); b[start_boundary + index + 0] = bi;

                // printf("bi: %f\n", bi);

                );
        } else {
#pragma omp parallel for
            Loop_Over_Rigid_Neighbors(
                real depth = data_manager->host_data.dpth_rigid_fluid[p * max_rigid_neighbors + i];
                real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed); b[start_boundary + index + 0] = bi;
                b[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
                b[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;);
        }
    }
    if (num_mpm_contacts > 0) {
        int index = 0;
        custom_vector<real3>& sorted_pos = data_manager->host_data.sorted_pos_3dof;

        Loop_Over_Fluid_Neighbors(real depth = Length(xij) - kernel_radius; depth = Min(depth, 0);
                                  real bi = std::max(real(1.0) / dt * depth, -contact_recovery_speed);
                                  b[start_contact + index + 0] = bi;);
    }
}
void ChMPMContainer::Build_E() {
    DynamicVector<real>& E = data_manager->host_data.E;

    if (num_rigid_fluid_contacts > 0) {
        if (contact_mu == 0) {
#pragma omp parallel for
            for (int index = 0; index < num_rigid_fluid_contacts; index++) {
                E[start_boundary + index + 0] = 0;
            }
        } else {
#pragma omp parallel for
            for (int index = 0; index < num_rigid_fluid_contacts; index++) {
                E[start_boundary + index + 0] = 0;
                E[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = 0;
                E[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = 0;
            }
        }
    }
    if (num_mpm_contacts > 0) {
// printf("Stiffness: %f\n", lambda + 2 * mu);

#pragma omp parallel for
        for (int index = 0; index < num_mpm_contacts; index++) {
            E[start_contact + index + 0] = 0;
        }
    }
}

void ChMPMContainer::Project(real* gamma) {
    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

    if (contact_mu == 0) {
#pragma omp parallel for
        Loop_Over_Rigid_Neighbors(
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];  // rigid is stored in the first index
            real cohesion = Max((data_manager->host_data.cohesion_data[rigid] + contact_cohesion) * .5, 0.0); real3 gam;
            gam.x = gamma[start_boundary + index];     //
            gam.x += cohesion;                         //
            gam.x = gam.x < 0 ? 0 : gam.x - cohesion;  //
            gamma[start_boundary + index] = gam.x;);
    } else {
#pragma omp parallel for
        Loop_Over_Rigid_Neighbors(
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];  // rigid is stored in the first index
            real rigid_fric = data_manager->host_data.fric_data[rigid].x;
            real cohesion = Max((data_manager->host_data.cohesion_data[rigid] + contact_cohesion) * .5, 0.0);
            real friction = (rigid_fric == 0 || contact_mu == 0) ? 0 : (rigid_fric + contact_mu) * .5;

            real3 gam;                              //
            gam.x = gamma[start_boundary + index];  //
            gam.y = gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0];
            gam.z = gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1];

            gam.x += cohesion;  //

            real mu = friction;  //
            if (mu == 0) {
                gam.x = gam.x < 0 ? 0 : gam.x - cohesion;  //
                gam.y = gam.z = 0;                         //

                gamma[start_boundary + index] = gam.x;
                gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = gam.y;
                gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = gam.z;
                continue;
            }

            if (Cone_generalized_rigid(gam.x, gam.y, gam.z, mu)) {}

            gamma[start_boundary + index] = gam.x - cohesion;  //
            gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 0] = gam.y;
            gamma[start_boundary + num_rigid_fluid_contacts + index * 2 + 1] = gam.z;);
    }

#pragma omp parallel for
    for (int index = 0; index < num_mpm_contacts; index++) {
        real3 gam;
        gam.x = gamma[start_contact + index];
        gam.x += cohesion;
        gam.x = gam.x < 0 ? 0 : gam.x - cohesion;
        gamma[start_contact + index] = gam.x;
    }
}

void ChMPMContainer::GenerateSparsity() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    LOG(INFO) << "ChMPMContainer::GenerateSparsity";
    if (num_rigid_fluid_contacts > 0) {
        LOG(INFO) << "ChConstraintRigidFluid::GenerateSparsity";

        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        Loop_Over_Rigid_Neighbors(int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                                  AppendRow6(D_T, start_boundary + index + 0, rigid * 6, 0);
                                  AppendRow3(D_T, start_boundary + index + 0, body_offset + p * 3, 0);
                                  D_T.finalize(start_boundary + index + 0););
        if (contact_mu != 0) {
            Loop_Over_Rigid_Neighbors(
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];

                AppendRow6(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, rigid * 6, 0);
                AppendRow3(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 0, body_offset + p * 3, 0);
                D_T.finalize(start_boundary + num_rigid_fluid_contacts + index * 2 + 0);

                AppendRow6(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, rigid * 6, 0);
                AppendRow3(D_T, start_boundary + num_rigid_fluid_contacts + index * 2 + 1, body_offset + p * 3, 0);

                D_T.finalize(start_boundary + num_rigid_fluid_contacts + index * 2 + 1););
        }
    }
    if (num_mpm_contacts > 0) {
        int index_n = 0;
        int index_t = 0;
        for (int body_a = 0; body_a < num_fluid_bodies; body_a++) {
            for (int i = 0; i < data_manager->host_data.c_counts_3dof_3dof[body_a]; i++) {
                int body_b = data_manager->host_data.neighbor_3dof_3dof[body_a * max_neighbors + i];
                if (body_a == body_b || body_a > body_b) {
                    continue;
                }

                AppendRow3(D_T, start_contact + index_n + 0, body_offset + body_a * 3, 0);
                AppendRow3(D_T, start_contact + index_n + 0, body_offset + body_b * 3, 0);

                D_T.finalize(start_contact + index_n + 0);
                index_n++;
            }
        }
    }
}
void ChMPMContainer::PreSolve() {
    LOG(INFO) << "ChMPMContainer::PreSolve()";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    const real dt = data_manager->settings.step_size;

#pragma omp parallel for
    for (int index = 0; index < num_mpm_nodes_active; index++) {
        uint start = node_start_index[index];
        uint end = node_start_index[index + 1];
        const int current_node = node_particle_mapping[index];
        int3 g = GridDecode(current_node, bins_per_axis);
        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
        real3 vel_mass = real3(0);
        real mass_w = 0;
        for (uint i = start; i < end; i++) {
            int p = particle_number[i];
            real weight = N(pos_marker[p] - current_node_location, inv_bin_edge) * mass;  //
            mass_w += weight;
            vel_mass += weight * vel_marker[p];
        }

        node_mass[current_node] = mass_w;
        grid_vel[current_node * 3 + 0] = vel_mass.x;  //
        grid_vel[current_node * 3 + 1] = vel_mass.y;  //
        grid_vel[current_node * 3 + 2] = vel_mass.z;  //
    }

// normalize weights for the velocity (to conserve momentum)
#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        real n_mass = node_mass[i];
        if (n_mass > C_EPSILON) {
            grid_vel[i * 3 + 0] /= n_mass;
            grid_vel[i * 3 + 1] /= n_mass;
            grid_vel[i * 3 + 2] /= n_mass;
        }
        // printf("N: %d [%f %f %f]\n", i, grid_vel[i * 3 + 0], grid_vel[i * 3 + 1], grid_vel[i * 3 + 2]);
    }
    old_vel_node_mpm = grid_vel;
    SVD_Fe_hat_R.resize(num_mpm_markers);
    SVD_Fe_hat_S.resize(num_mpm_markers);

// printf("Compute_Elastic_Deformation_Gradient_Hat\n");
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        marker_Fe_hat[p] = Mat33(1.0);
        Mat33 Fe_hat_t(1);
        LOOPOVERNODES(  //
            real3 vel(grid_vel[i * 3 + 0], grid_vel[i * 3 + 1], grid_vel[i * 3 + 2]);
            real3 kern = dN(xi - current_node_location, inv_bin_edge);  //
            Fe_hat_t += OuterProduct(dt * vel, kern);                   //
            )
        Mat33 Fe_hat = Fe_hat_t * marker_Fe[p];
        marker_Fe_hat[p] = Fe_hat;
        Mat33 U, V;
        real3 E;
        SVD(Fe_hat, U, E, V);
        SVD_Fe_hat_R[p] = MultTranspose(U, V);
        SVD_Fe_hat_S[p] = V * MultTranspose(Mat33(E), V);
    }

    UpdateRhs();

    if (num_rigid_fluid_contacts > 0) {
        LOG(INFO) << "ChMPMContainer::RigidContact";

        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;

        custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;
        custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;

        // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
        custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_fluid;
        custom_vector<real3>& norm = data_manager->host_data.norm_rigid_fluid;

        Loop_Over_Rigid_Neighbors(                                          //
            int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];  //
            real3 U = norm[p * max_rigid_neighbors + i]; real3 V; real3 W;  //
            Orthogonalize(U, V, W); real3 T1; real3 T2; real3 T3;           //
            real3 c_pt = cpta[p * max_rigid_neighbors + i];

            real3 v_rigid = real3(data_manager->host_data.v[rigid * 6 + 0], data_manager->host_data.v[rigid * 6 + 1],
                                  data_manager->host_data.v[rigid * 6 + 2]);

            real3 o_rigid = real3(data_manager->host_data.v[rigid * 6 + 3], data_manager->host_data.v[rigid * 6 + 4],
                                  data_manager->host_data.v[rigid * 6 + 5]);

            real3 pt1_loc = TransformParentToLocal(pos_rigid[rigid], rot_rigid[rigid], c_pt);
            // Velocity of the contact point on the body:
            real3 vel1 = v_rigid + Rotate(Cross(o_rigid, pt1_loc), rot_rigid[rigid]);

            // Find the grid cell

            real3 xi = c_pt;
            LOOPOVERNODES(                                                  //
                real weight = N(xi - current_node_location, inv_bin_edge);  //
                grid_vel[current_node * 3 + 0] = vel1.x;                    //
                grid_vel[current_node * 3 + 1] = vel1.y;                    //
                grid_vel[current_node * 3 + 2] = vel1.z;                    //
                )

            //
            //            const int cx = GridCoord(c_pt.x, inv_bin_edge, min_bounding_point.x);
            //            const int cy = GridCoord(c_pt.y, inv_bin_edge, min_bounding_point.y);
            //            const int cz = GridCoord(c_pt.z, inv_bin_edge, min_bounding_point.z);
            //            const int current_node = GridHash(cx, cy, cz, bins_per_axis);
            //
            //            real3 current_node_location = NodeLocation(cx, cy, cz, bin_edge, min_bounding_point);
            //            real weight = N(c_pt - current_node_location, inv_bin_edge);  //
            //            grid_vel[current_node * 3 + 0] = vel1.x;                      //
            //            grid_vel[current_node * 3 + 1] = vel1.y;                      //
            //            grid_vel[current_node * 3 + 2] = vel1.z;
            //
            //            printf("rigid_cvel: [%f %f %f]\n", vel1.x, vel1.y, vel1.z);  //
            );
    }

    delta_v.resize(num_mpm_nodes * 3);
    delta_v = 0;
    Solve(rhs, delta_v);
    grid_vel += delta_v;

    const real3 gravity = data_manager->settings.gravity;
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real3 V_flip = vel_marker[p];

        real3 V_pic = real3(0.0);

        LOOPOVERNODES(                                                  //
            real weight = N(xi - current_node_location, inv_bin_edge);  //

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

        int index = data_manager->host_data.reverse_mapping_3dof[p];

        data_manager->host_data.v[body_offset + index * 3 + 0] = new_vel.x;
        data_manager->host_data.v[body_offset + index * 3 + 1] = new_vel.y;
        data_manager->host_data.v[body_offset + index * 3 + 2] = new_vel.z;
    }

    printf("Done Pre solve\n");
}
void ChMPMContainer::PostSolve() {
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

        //        real JE = Determinant(marker_Fe[p]);  //
        //        real JP = Determinant(marker_Fp[p]);  //
        //
        //        printf("JU: %d [%f %f] [%f %f %f]\n", p, JE, JP, E_clamped.x, E_clamped.y, E_clamped.z);
    }
}

void ChMPMContainer::Solve(const DynamicVector<real>& rhs, DynamicVector<real>& delta_v) {
    LOG(INFO) << "ChMPMContainer::Solve";
    ChShurProductMPM Multiply;
    Multiply.Setup(this, data_manager);
    ChProjectNone ProjectNone;
    solver->Solve(Multiply, ProjectNone, max_iterations, num_mpm_nodes * 3, rhs, delta_v);

    int size = data_manager->measures.solver.maxd_hist.size();
    for (int i = 0; i < size; i++) {
        printf("[%f %f]\n", data_manager->measures.solver.maxd_hist[i],
               data_manager->measures.solver.maxdeltalambda_hist[i]);
    }
}

void ChMPMContainer::UpdateRhs() {
    LOG(INFO) << "ChMPMContainer::UpdateRhs()";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    const real dt = data_manager->settings.step_size;

    // Precompute force for particle

    custom_vector<Mat33> rhs_temp(num_mpm_markers);
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];

        Mat33 PED =
            Potential_Energy_Derivative_Deviatoric(marker_Fe_hat[p], marker_Fp[p], mu, lambda, hardening_coefficient);

        Mat33 vPEDFepT = marker_volume[p] * MultTranspose(PED, marker_Fe[p]);
        real JE = Determinant(marker_Fe[p]);  //
        real JP = Determinant(marker_Fp[p]);  //
        rhs_temp[p] = (vPEDFepT) * (1.0 / (JE * JP));
    }

#pragma omp parallel for
    for (int index = 0; index < num_mpm_nodes_active; index++) {
        uint start = node_start_index[index];
        uint end = node_start_index[index + 1];
        const int current_node = node_particle_mapping[index];
        int3 g = GridDecode(current_node, bins_per_axis);
        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
        real3 force = real3(0);
        for (uint i = start; i < end; i++) {
            int p = particle_number[i];
            force += dt * rhs_temp[p] * dN(pos_marker[p] - current_node_location, inv_bin_edge);  //;
        }

        rhs[current_node * 3 + 0] = -force.x;  //
        rhs[current_node * 3 + 1] = -force.y;  //
        rhs[current_node * 3 + 2] = -force.z;  //
    }

    //    for (int i = 0; i < num_mpm_nodes; i++) {
    //        printf("R: %d [%f %f %f]\n", i, rhs[i * 3 + 0], rhs[i * 3 + 1], rhs[i * 3 + 2]);
    //    }
}

}  // END_OF_NAMESPACE____

/////////////////////
