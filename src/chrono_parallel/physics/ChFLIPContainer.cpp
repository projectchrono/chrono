
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

ChFLIPContainer::ChFLIPContainer(ChSystemParallelDVI* physics_system) {
    data_manager = physics_system->data_manager;
    data_manager->AddFLIPContainer(this);

    max_iterations = 10;
    real mass = 1;
    real mu = 1;
    real hardening_coefficient = 1;
    real lambda = 1;
    real theta_s = 1;
    real theta_c = 1;
    real alpha = 1;
    real rho = 1000;
}
ChFLIPContainer::~ChFLIPContainer() {}

void ChFLIPContainer::Setup(int start_constraint) {
    Ch3DOFContainer::Setup(start_constraint);
    start_node = start_constraint;
    body_offset = num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + num_fea_nodes * 3;
    min_bounding_point = data_manager->measures.collision.mpm_min_bounding_point;
    max_bounding_point = data_manager->measures.collision.mpm_max_bounding_point;
    num_rigid_mpm_contacts = data_manager->num_rigid_mpm_contacts;
}

void ChFLIPContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;

    pos_marker.insert(pos_marker.end(), positions.begin(), positions.end());
    vel_marker.insert(vel_marker.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_marker.resize(pos_marker.size());
    data_manager->num_mpm_markers = pos_marker.size();
}
void ChFLIPContainer::ComputeDOF() {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
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
    data_manager->num_mpm_nodes = grid_size;
    num_mpm_nodes = grid_size;

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("diag [%f %f %f] edge:%f \n", diag.x, diag.y, diag.z, bin_edge);
    printf("Compute DOF [%d] [%d %d %d] [%f] %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);

    num_mpm_constraints = num_mpm_nodes;
}

void ChFLIPContainer::Update(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    custom_vector<real>& node_mass = data_manager->host_data.node_mass;
    const real dt = data_manager->settings.step_size;
    real3 g_acc = data_manager->settings.gravity;
    Setup(0);

    node_mass.resize(num_mpm_nodes);
    data_manager->host_data.old_vel_node_mpm.resize(num_mpm_nodes * 3);
    rhs.resize(num_mpm_nodes * 3);
    grid_vel.resize(num_mpm_nodes * 3);

    std::fill(node_mass.begin(), node_mass.end(), 0);
    std::fill(grid_vel.begin(), grid_vel.end(), 0);

#pragma omp paralle for
    for (int i = 0; i < num_mpm_nodes; i++) {
        data_manager->host_data.v[body_offset + i * 3 + 0] = 0;
        data_manager->host_data.v[body_offset + i * 3 + 1] = 0;
        data_manager->host_data.v[body_offset + i * 3 + 2] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 0] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = 0;
    }
    // Mass at cell centers
    // velocity at face centers
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        const real3 vi = vel_marker[p];
        LOOPONERING(                                                                                     //
            node_mass[current_node] += N(xi - current_node_location, inv_bin_edge) * mass;               //
            real weight_x = N(xi - (current_node_location - real3(.5 * bin_edge, 0, 0)), inv_bin_edge);  //
            real weight_y = N(xi - (current_node_location - real3(0, .5 * bin_edge, 0)), inv_bin_edge);  //
            real weight_z = N(xi - (current_node_location - real3(0, 0, .5 * bin_edge)), inv_bin_edge);  //

            data_manager->host_data.v[body_offset + current_node * 3 + 0] += weight_x * vi.x;  //
            data_manager->host_data.v[body_offset + current_node * 3 + 1] += weight_y * vi.y;  //
            data_manager->host_data.v[body_offset + current_node * 3 + 2] += weight_z * vi.z;  //
            )
    }
#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        // if (data_manager->host_data.node_mass[i] > C_EPSILON) {

        data_manager->host_data.old_vel_node_mpm[i * 3 + 0] = data_manager->host_data.v[body_offset + i * 3 + 0];
        data_manager->host_data.old_vel_node_mpm[i * 3 + 1] = data_manager->host_data.v[body_offset + i * 3 + 1];
        data_manager->host_data.old_vel_node_mpm[i * 3 + 2] = data_manager->host_data.v[body_offset + i * 3 + 2];

        real3 h_gravity = dt * node_mass[i] * g_acc;
        data_manager->host_data.hf[body_offset + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = h_gravity.z;
        //}
    }

    //#pragma omp paralle for
    //    for (int i = 0; i < num_mpm_nodes; i++) {
    //        int3 g = GridDecode(i, bins_per_axis);
    //
    //        if (g.x == bins_per_axis.x - 1 || g.y == bins_per_axis.y - 1 || g.z == bins_per_axis.z - 1) {
    //            data_manager->host_data.v[body_offset + i * 3 + 0] = 0;
    //            data_manager->host_data.v[body_offset + i * 3 + 1] = 0;
    //            data_manager->host_data.v[body_offset + i * 3 + 2] = 0;
    //        }
    //        if (g.x == 0 || g.y == 0 || g.z == 0) {
    //            data_manager->host_data.hf[body_offset + i * 3 + 0] = 0;
    //            data_manager->host_data.hf[body_offset + i * 3 + 1] = 0;
    //            data_manager->host_data.hf[body_offset + i * 3 + 2] = 0;
    //        }
    //    }

    // update the position of the markers based on the nodal velocities
}

void ChFLIPContainer::UpdatePosition(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    custom_vector<real>& old_vel_node_mpm = data_manager->host_data.old_vel_node_mpm;

    DynamicVector<real>& v = data_manager->host_data.v;
    printf("Update_Particle_Velocities\n");
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real3 V_flip = vel_marker[p];

        real3 V_pic = real3(0.0);
        LOOPONERING(                                                                                                  //
            real weight_x = N(xi - (current_node_location - real3(.5 * bin_edge, 0, 0)), inv_bin_edge);               //
            real weight_y = N(xi - (current_node_location - real3(0, .5 * bin_edge, 0)), inv_bin_edge);               //
            real weight_z = N(xi - (current_node_location - real3(0, 0, .5 * bin_edge)), inv_bin_edge);               //
            V_pic.x += v[body_offset + current_node * 3 + 0] * weight_x;                                              //
            V_pic.y += v[body_offset + current_node * 3 + 1] * weight_y;                                              //
            V_pic.z += v[body_offset + current_node * 3 + 2] * weight_z;                                              //
            V_flip.x += (v[body_offset + current_node * 3 + 0] - old_vel_node_mpm[current_node * 3 + 0]) * weight_x;  //
            V_flip.y += (v[body_offset + current_node * 3 + 1] - old_vel_node_mpm[current_node * 3 + 1]) * weight_y;  //
            V_flip.z += (v[body_offset + current_node * 3 + 2] - old_vel_node_mpm[current_node * 3 + 2]) * weight_z;  //
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

int ChFLIPContainer::GetNumConstraints() {
    return num_mpm_nodes;
}

int ChFLIPContainer::GetNumNonZeros() {
    return num_mpm_nodes * 3 * 6;
}

void ChFLIPContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    custom_vector<real>& node_mass = data_manager->host_data.node_mass;
    const real dt = data_manager->settings.step_size;
    real inv_mass = 0;
    for (int i = 0; i < num_mpm_nodes; i++) {
        if (node_mass[i] > C_EPSILON) {
            // inv_mass = 1.0 / node_mass[i];
            real density = node_mass[i] / (bin_edge * bin_edge * bin_edge);
            inv_mass = 1 / node_mass[i];
        }
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
        M_inv.finalize(offset + i * 3 + 2);
    }
}
void ChFLIPContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    const real dt = data_manager->settings.step_size;
    for (int i = 0; i < num_mpm_nodes; i++) {
        // real node_mass = data_manager->host_data.node_mass[i];
        real node_mass = rho / dt;
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, node_mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, node_mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, node_mass);
        M.finalize(offset + i * 3 + 2);
    }
}

void ChFLIPContainer::Initialize() {
    const real dt = data_manager->settings.step_size;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Initialize [%d] [%d %d %d] [%f] %d\n", num_mpm_nodes, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);

    data_manager->host_data.marker_volume.resize(num_mpm_markers);
    data_manager->host_data.node_mass.resize(num_mpm_nodes);
    std::fill(data_manager->host_data.node_mass.begin(), data_manager->host_data.node_mass.end(), 0);

    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        const real3 vi = vel_marker[p];

        LOOPONERING(                                                                  //
            real weight = N(real3(xi) - current_node_location, inv_bin_edge) * mass;  //
            data_manager->host_data.node_mass[current_node] += weight;                //
            )
    }

    printf("Compute_Particle_Volumes %f\n", mass);
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real particle_density = 0;

        LOOPONERING(                                                                       //
            real weight = N(xi - current_node_location, inv_bin_edge);                     //
            particle_density += data_manager->host_data.node_mass[current_node] * weight;  //
            )
        particle_density /= (bin_edge * bin_edge * bin_edge);
        data_manager->host_data.marker_volume[p] = mass / particle_density;
        // printf("Volumes: %.20f \n", data_manager->host_data.marker_volume[p], particle_density);
    }
}
// Minv = dt/rho

// r = b - dt * D *b*(vn+dt/rho*f)
// r = b + D*(v+dt*(dt/rho)*f)

// r = b+D*v + D*dt*Minv*f
void ChFLIPContainer::Project(real* gamma) {
    //    for (int i = 0; i < num_mpm_constraints; i++) {
    //        if (gamma[start_node + i] < 0) {
    //            gamma[start_node + i] = 0;
    //        }
    //    }
}

void ChFLIPContainer::Build_D() {
    LOG(INFO) << "ChFLIPContainer::Build_D";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    const real dt = data_manager->settings.step_size;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    real factor = inv_bin_edge;

    for (int n = 0; n < num_mpm_nodes; n++) {
        int3 g = GridDecode(n, bins_per_axis);

        if (g.x == 0 || g.y == 0 || g.z == 0) {
            continue;
        }
        if (g.x == bins_per_axis.x - 1 || g.y == bins_per_axis.y - 1 || g.z == bins_per_axis.z - 1) {
            continue;
        }
        real3 fvec = real3(-factor, -factor, -factor);
        real3 fvecod = real3(0, 0, 0);
        if (g.x == 0) {
            fvec.x = 0;
        }
        if (g.y == 0) {
            fvec.y = 0;
        }
        if (g.z == 0) {
            fvec.z = 0;
        }
        SetRow3(D_T, start_row + n, body_offset + n * 3, fvec);

        if (g.x != bins_per_axis.x - 1) {
            SetRow3(D_T, start_row + n, body_offset + GridHash(g.x + 1, g.y, g.z, bins_per_axis) * 3,
                    real3(factor, 0, 0));
            fvecod.x = factor;
        }
        if (g.y != bins_per_axis.y - 1) {
            SetRow3(D_T, start_row + n, body_offset + GridHash(g.x, g.y + 1, g.z, bins_per_axis) * 3,
                    real3(0, factor, 0));
            fvecod.y = factor;
        }
        if (g.z != bins_per_axis.z - 1) {
            SetRow3(D_T, start_row + n, body_offset + GridHash(g.x, g.y, g.z + 1, bins_per_axis) * 3,
                    real3(0, 0, factor));
            fvecod.z = factor;
        }

        //        printf("D: [%d %d %d] %f %f %f %f %f %f\n", g.x, g.y, g.z, fvec.x, fvec.y, fvec.z, fvecod.x, fvecod.y,
        //               fvecod.z);
    }
}
void ChFLIPContainer::Build_b() {
    SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, start_node, num_mpm_constraints);
    // b_sub = 0;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    const real dt = data_manager->settings.step_size;

    //        for (int index = 0; index < num_mpm_nodes; index++) {
    //            if (data_manager->host_data.node_mass[index] > C_EPSILON) {
    //                real density = data_manager->host_data.node_mass[index] / (bin_edge * bin_edge * bin_edge);
    //                // printf("density: %f\n", density);
    //                b_sub[index] = (density / rho - 1.0);
    //            } else {
    //                b_sub[index] = 0;
    //            }
    //        }
    // SubVectorType v_sub = blaze::subvector(data_manager->host_data.v, body_offset, num_mpm_nodes * 3);
    // b_sub =
    //   b_sub- dt * blaze::submatrix(D_T, start_row, body_offset, num_mpm_nodes * 3, num_mpm_nodes * 3) * v_sub;
}
void ChFLIPContainer::Build_E() {
    LOG(INFO) << "ChMPMContainer::Build_E";
    SubVectorType E_sub = blaze::subvector(data_manager->host_data.E, start_node, num_mpm_constraints);
    E_sub = 0;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
}
void ChFLIPContainer::GenerateSparsity() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    LOG(INFO) << "ChFLIPContainer::GenerateSparsity" << D_T.rows() << " " << D_T.columns();
    int index_t = 0;

    custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_mpm;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_mpm;

    for (int n = 0; n < num_mpm_nodes; n++) {
        int3 g = GridDecode(n, bins_per_axis);

        AppendRow3(D_T, start_row + n, body_offset + n * 3, 0);
        if (g.x != bins_per_axis.x - 1) {
            AppendRow3(D_T, start_row + n, body_offset + GridHash(g.x + 1, g.y, g.z, bins_per_axis) * 3, 0);
        }
        if (g.y != bins_per_axis.y - 1) {
            AppendRow3(D_T, start_row + n, body_offset + GridHash(g.x, g.y + 1, g.z, bins_per_axis) * 3, 0);
        }
        if (g.z != bins_per_axis.z - 1) {
            AppendRow3(D_T, start_row + n, body_offset + GridHash(g.x, g.y, g.z + 1, bins_per_axis) * 3, 0);
        }
        D_T.finalize(start_row + n);
    }
}
void ChFLIPContainer::PreSolve() {}
void ChFLIPContainer::PostSolve() {
    SubVectorType gamma_sub = blaze::subvector(data_manager->host_data.gamma, start_node, num_mpm_constraints);
    custom_vector<real>& node_mass = data_manager->host_data.node_mass;
    for (int i = 0; i < gamma_sub.size(); i++) {
        int3 g = GridDecode(i, bins_per_axis);
        int hash = GridHash(g.x, g.y, g.z, bins_per_axis);
        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
        if (node_mass[i] > C_EPSILON) {
            printf("gamma: %f [%d %d %d] %.20f [%d %d] [%f %f %f] \n", gamma_sub[i], g.x, g.y, g.z, node_mass[i], hash,
                   i, current_node_location.x, current_node_location.y, current_node_location.z);
        }
    }

    SubVectorType v_sub = blaze::subvector(data_manager->host_data.v, body_offset, num_mpm_nodes * 3);

    for (int i = 0; i < v_sub.size(); i += 3) {
        printf("v: [%f,%f,%f] [%d] \n", v_sub[i * 3 + 0], v_sub[i * 3 + 1], v_sub[i * 3 + 2], i);
    }
}

void ChFLIPContainer::Solve(const DynamicVector<real>& rhs, DynamicVector<real>& delta_v) {}

void ChFLIPContainer::UpdateRhs() {}

}  // END_OF_NAMESPACE____

/////////////////////
