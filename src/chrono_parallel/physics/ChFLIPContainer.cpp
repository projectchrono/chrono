
#include <algorithm>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include "chrono_parallel/physics/MPMUtils.h"
#include <chrono_parallel/collision/ChBroadphaseUtils.h>
#include <thrust/transform_reduce.h>
#include "chrono_parallel/constraints/ChConstraintUtils.h"
#include "chrono_parallel/solver/ChSolverParallel.h"

#include "chrono_parallel/math/other_types.h"  // for uint, int2, vec3
#include "chrono_parallel/math/real.h"         // for real
#include "chrono_parallel/math/real3.h"        // for real3

namespace chrono {

using namespace collision;
using namespace geometry;

#define RINGS 1

#if RINGS == 2
#define MAX_OFF 10
#define MIN_OFF 8
#elif RINGS == 1
#define MAX_OFF 8
#define MIN_OFF 6
#else
#define MAX_OFF 4
#define MIN_OFF 2
#endif

void Print(std::ostream& os, const CompressedMatrix<real>& M, vec3 bins_per_axis) {
    for (size_t i = 0UL; i < (~M).rows(); ++i) {
        // os << "";
        for (size_t j = 0UL; j < (~M).columns(); ++j) {
            // os << std::setw(1) << (~M)(i, j) << ", ";
            if ((~M)(i, j) != 0) {
                vec3 g = GridDecode(i, bins_per_axis);
                vec3 f = GridDecode(j / 3, bins_per_axis);
                printf("[%d %d] [%f] [%d %d %d] [%d %d %d]\n", i, j, (~M)(i, j), g.x, g.y, g.z, f.x, f.y, f.z);
            }
        }
        // os << "\n";
    }
}

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

    body_offset = num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + num_fea_nodes * 3;
    min_bounding_point = data_manager->measures.collision.mpm_min_bounding_point;
    max_bounding_point = data_manager->measures.collision.mpm_max_bounding_point;
    num_rigid_mpm_contacts = data_manager->num_rigid_fluid_contacts;
    start_boundary = start_constraint;
    start_node = start_boundary + num_rigid_mpm_contacts;
}

void ChFLIPContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;

    pos_marker.insert(pos_marker.end(), positions.begin(), positions.end());
    vel_marker.insert(vel_marker.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_marker.resize(pos_marker.size());
    data_manager->num_fluid_bodies = pos_marker.size();
}
void ChFLIPContainer::ComputeDOF() {
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

#if 1
    max_bounding_point = real3(res.second.x, res.second.y, res.second.z) + kernel_radius * MAX_OFF;
    min_bounding_point = real3(res.first.x, res.first.y, res.first.z) - kernel_radius * MIN_OFF;
#else
    max_bounding_point = real3(.2, .15, .15);
    min_bounding_point = real3(-.2, -.15, -.15);
#endif

    real3 diag = max_bounding_point - min_bounding_point;
    bin_edge = kernel_radius * 2;
    bins_per_axis = vec3(diag / bin_edge);
    inv_bin_edge = real(1.) / bin_edge;
    uint grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;
    num_mpm_nodes = grid_size;

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("diag [%f %f %f] edge:%f \n", diag.x, diag.y, diag.z, bin_edge);
    printf("Compute DOF [%d] [%d %d %d] [%f] %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);
}

void ChFLIPContainer::Update(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    ;
    DynamicVector<real>& v = data_manager->host_data.v;
    const real dt = data_manager->settings.step_size;
    real3 g_acc = data_manager->settings.gravity;
    Setup(0);

    node_mass.resize(num_mpm_nodes);
    face_density.resize(num_mpm_nodes);
    face_volume.resize(num_mpm_nodes);
    old_vel_node_mpm.resize(num_mpm_nodes * 3);

    //#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        v[body_offset + i * 3 + 0] = 0;
        v[body_offset + i * 3 + 1] = 0;
        v[body_offset + i * 3 + 2] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 0] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = 0;
        node_mass[i] = 0;
        // face_density[i] = real3(0, 0, 0);
        face_volume[i] = 0;
    }

    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        const real3 vi = vel_marker[p];

        LOOPOVERNODESY(real weight = N_Tri(xi - (current_node_location), inv_bin_edge);  //
                       node_mass[current_node] += weight * mass;                         //
                       face_volume[current_node] += weight * mass / rho;                 //
                       v[body_offset + current_node * 3 + 0] += weight * vi.x * mass;    //
                       v[body_offset + current_node * 3 + 1] += weight * vi.y * mass;    //
                       v[body_offset + current_node * 3 + 2] += weight * vi.z * mass;    //
                       , RINGS);
    }
    real3 h_gravity = mass * g_acc;
#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        vec3 g = GridDecode(i, bins_per_axis);
        int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
        int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);

        int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);
        int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);

        int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
        int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);
        if (node_mass[i] > 0) {
            v[body_offset + i * 3 + 0] /= node_mass[i];  //
            v[body_offset + i * 3 + 1] /= node_mass[i];  //
            v[body_offset + i * 3 + 2] /= node_mass[i];  //
        }

        data_manager->host_data.hf[body_offset + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = h_gravity.z;

        old_vel_node_mpm[i * 3 + 0] = v[body_offset + i * 3 + 0];
        old_vel_node_mpm[i * 3 + 1] = v[body_offset + i * 3 + 1];
        old_vel_node_mpm[i * 3 + 2] = v[body_offset + i * 3 + 2];

        /// printf("new_vel: [%f %f %f] \n", face_density[i].x, face_density[i].y, face_density[i].z);
    }
}

void ChFLIPContainer::UpdatePosition(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    DynamicVector<real>& v = data_manager->host_data.v;
    printf("Update_Particle_Velocities\n");
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real3 V_flip = vel_marker[p];
        real3 V_pic = real3(0.0);
        LOOPOVERNODESY(                                                                                             //
            real weight = N_Tri(xi - (current_node_location), inv_bin_edge);                                        //
            V_pic.x += v[body_offset + current_node * 3 + 0] * weight;                                              //
            V_pic.y += v[body_offset + current_node * 3 + 1] * weight;                                              //
            V_pic.z += v[body_offset + current_node * 3 + 2] * weight;                                              //
            V_flip.x += (v[body_offset + current_node * 3 + 0] - old_vel_node_mpm[current_node * 3 + 0]) * weight;  //
            V_flip.y += (v[body_offset + current_node * 3 + 1] - old_vel_node_mpm[current_node * 3 + 1]) * weight;  //
            V_flip.z += (v[body_offset + current_node * 3 + 2] - old_vel_node_mpm[current_node * 3 + 2]) * weight;  //
            , RINGS                                                                                                 //
            )
        real3 new_vel = (1.0 - alpha) * V_pic + alpha * V_flip;

        real speed = Length(new_vel);
        if (speed > max_velocity) {
            new_vel = new_vel * max_velocity / speed;
        }

        //        printf("new_vel: [%f %f %f] pic:[%f %f %f] flip:[%f %f %f]\n", new_vel.x, new_vel.y, new_vel.z,
        //        V_pic.x,
        //               V_pic.y, V_pic.z, V_flip.x, V_flip.y, V_flip.z);

        vel_marker[p] = new_vel;
        pos_marker[p] += new_vel * data_manager->settings.step_size;
    }
}

int ChFLIPContainer::GetNumConstraints() {
    return data_manager->num_rigid_fluid_contacts + num_mpm_nodes;
}

int ChFLIPContainer::GetNumNonZeros() {
    return 9 * data_manager->num_rigid_fluid_contacts + num_mpm_nodes * 9;
}

void ChFLIPContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    const real dt = data_manager->settings.step_size;

    for (int i = 0; i < num_mpm_nodes; i++) {
        real3 inv_mass = real3(0);

        vec3 g = GridDecode(i, bins_per_axis);

        real density = node_mass[i] / (bin_edge * bin_edge * bin_edge);
        if (node_mass[i] > 0) {
            real mass = rho * (bin_edge * bin_edge * bin_edge);
            inv_mass.x = dt / mass;  //(face_density[i].x / face_volume[i].x);
            inv_mass.y = dt / mass;  //(face_density[i].y / face_volume[i].y);
            inv_mass.z = dt / mass;  //(face_density[i].z / face_volume[i].z);
        }
        //        printf("MASS: [%f %f %f] [%f %f %f]  [%d %d %d]\n", face_density[i].x, face_density[i].y,
        //        face_density[i].z,
        //               inv_mass.x, inv_mass.y, inv_mass.z, g.x, g.y, g.z);
        //
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass.x);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass.y);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass.z);
        M_inv.finalize(offset + i * 3 + 2);
    }
}
void ChFLIPContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    const real dt = data_manager->settings.step_size;

    for (int i = 0; i < num_mpm_nodes; i++) {
        vec3 g = GridDecode(i, bins_per_axis);

        //        M.append(offset + i * 3 + 0, offset + i * 3 + 0, face_density[i].x / dt);
        //        M.finalize(offset + i * 3 + 0);
        //        M.append(offset + i * 3 + 1, offset + i * 3 + 1, face_density[i].y / dt);
        //        M.finalize(offset + i * 3 + 1);
        //        M.append(offset + i * 3 + 2, offset + i * 3 + 2, face_density[i].z / dt);
        //        M.finalize(offset + i * 3 + 2);
    }
}

void ChFLIPContainer::Initialize() {
    const real dt = data_manager->settings.step_size;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Initialize [%d] [%d %d %d] [%f] %d\n", num_mpm_nodes, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);

    node_mass.resize(num_mpm_nodes);
    std::fill(node_mass.begin(), node_mass.end(), 0);

    //    for (int p = 0; p < num_mpm_markers; p++) {
    //        const real3 xi = pos_marker[p];
    //        const real3 vi = vel_marker[p];
    //
    //        LOOPOVERNODESY(                                                               //
    //            real weight = N(real3(xi) - current_node_location, inv_bin_edge) * mass;  //
    //            data_manager->host_data.node_mass[current_node] += weight;, 2             //
    //            )
    //    }

    printf("Compute_Particle_Volumes %f\n", mass);
    //#pragma omp parallel for
    //    for (int p = 0; p < num_mpm_markers; p++) {
    //        const real3 xi = pos_marker[p];
    //        real particle_density = 0;
    //
    //        LOOPOVERNODESY(                                                 //
    //            real weight = N(xi - current_node_location, inv_bin_edge);  //
    //            particle_density += data_manager->host_data.node_mass[current_node] * weight;, 2)
    //        particle_density /= (bin_edge * bin_edge * bin_edge);
    //        data_manager->host_data.marker_volume[p] = mass / particle_density;
    //        // printf("Volumes: %.20f \n", data_manager->host_data.marker_volume[p], particle_density);
    //    }
}
void ChFLIPContainer::Project(real* gamma) {
    for (int i = 0; i < data_manager->num_rigid_fluid_contacts; i++) {
        if (gamma[start_boundary + i] < 0) {
            gamma[start_boundary + i] = 0;
        }
    }

    for (int i = 0; i < num_mpm_nodes; i++) {
        vec3 g = GridDecode(i, bins_per_axis);

        // Dirchlet pressure boundary condition for empty cells
        if (node_mass[i] <= 0) {
            gamma[start_node + i] = 0;
        }

        //        if (gamma[start_node + i] < 0) {
        //            gamma[start_node + i] = 0;
        //        }
    }
}

void ChFLIPContainer::Build_D() {
    LOG(INFO) << "ChFLIPContainer::Build_D";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_3dof;
    const real dt = data_manager->settings.step_size;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;

    real Aij = bin_edge * bin_edge;
    real lij = bin_edge;

    if (num_rigid_mpm_contacts > 0) {
        custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;
        custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;

        // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
        custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_fluid;
        custom_vector<real3>& norm = data_manager->host_data.norm_rigid_fluid;
        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
#pragma omp parallel for
        for (int p = 0; p < num_mpm_markers; p++) {
            int start = contact_counts[p];
            int end = contact_counts[p + 1];
            const real3 xi = pos_marker[p];
            for (int index = start; index < end; index++) {
                int i = index - start;  // index that goes from 0
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];
                real3 U = norm[p * max_rigid_neighbors + i], V, W;
                Orthogonalize(U, V, W);
                real3 T1, T2, T3;
                Compute_Jacobian(rot_rigid[rigid], U, V, W, cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1,
                                 T2, T3);

                vec3 g;

                g.x = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);
                g.y = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);
                g.z = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);

                const int current_node = GridHash(g.x, g.y, g.z, bins_per_axis);

                int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
                int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);
                int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);
                int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);
                int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
                int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);

                SetRow6Check(D_T, start_boundary + index + 0, rigid * 6, -U, T1);
                // SetRow6Check(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 0, rigid * 6, -V, T2);
                // SetRow6Check(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 1, rigid * 6, -W, T3);

                // SetRow3Check(D_T, start_boundary + index + 0, body_offset + g_front * 3, U * N(xi -
                // (NodeLocation(g.x, g.y, g.z-1, bin_edge, min_bounding_point)), inv_bin_edge));
                // SetRow3Check(D_T, start_boundary + index + 0, body_offset + g_down * 3, U * N(xi - (NodeLocation(g.x,
                // g.y-1, g.z, bin_edge, min_bounding_point)), inv_bin_edge));
                // SetRow3Check(D_T, start_boundary + index + 0, body_offset + g_left * 3, U * N(xi -
                // (NodeLocation(g.x-1, g.y, g.z, bin_edge, min_bounding_point)), inv_bin_edge));

                SetRow3Check(D_T, start_boundary + index + 0, body_offset + current_node * 3,
                             U * N(xi - (NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point)), inv_bin_edge));

                // SetRow3Check(D_T, start_boundary + index + 0, body_offset + g_right * 3, U * N(xi -
                // (NodeLocation(g.x+1, g.y, g.z, bin_edge, min_bounding_point)), inv_bin_edge));
                // SetRow3Check(D_T, start_boundary + index + 0, body_offset + g_up * 3, U *    N(xi -
                // (NodeLocation(g.x, g.y+1, g.z, bin_edge, min_bounding_point)), inv_bin_edge));
                // SetRow3Check(D_T, start_boundary + index + 0, body_offset + g_back * 3, U *  N(xi -
                // (NodeLocation(g.x, g.y, g.z+1, bin_edge, min_bounding_point)), inv_bin_edge));
                // SetRow3Check(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 0, body_offset + p * 3, V);
                // SetRow3Check(D_T, start_row + num_rigid_mpm_contacts + index * 2 + 1, body_offset + p * 3, W);
            }
        }
    }

    for (int n = 0; n < num_mpm_nodes; n++) {
        vec3 g = GridDecode(n, bins_per_axis);
        int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
        int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);
        int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);
        int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);
        int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
        int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);

        real factor = inv_bin_edge;
        real3 ff(0, 0, 0);

        if (node_mass[n] > 0) {
            real3 sum_d = real3(0, 0, 0);
            real3 cell_d, qj, bij;
            real3 qi = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);  // Node location

            if (node_mass[g_left] > 0) {
                qj = NodeLocation(g.x - 1, g.y, g.z, bin_edge, min_bounding_point);  // Node location
                bij = qi - real3(.5 * bin_edge, 0, 0);                               // face barycenter
                cell_d = Aij / lij * (qj - bij);
                sum_d -= cell_d;
                SetRow3Check(D_T, start_node + n, body_offset + g_left * 3, cell_d);
            }

            if (node_mass[g_right] > 0) {
                qj = NodeLocation(g.x + 1, g.y, g.z, bin_edge, min_bounding_point);  // Node location
                bij = qi + real3(.5 * bin_edge, 0, 0);                               // face barycenter
                cell_d = Aij / lij * (qj - bij);
                sum_d -= cell_d;
                SetRow3Check(D_T, start_node + n, body_offset + g_right * 3, cell_d);
            }
            if (node_mass[g_down] > 0) {
                qj = NodeLocation(g.x, g.y - 1, g.z, bin_edge, min_bounding_point);  // Node location
                bij = qi - real3(0, .5 * bin_edge, 0);                               // face barycenter
                cell_d = Aij / lij * (qj - bij);
                sum_d -= cell_d;
                SetRow3Check(D_T, start_node + n, body_offset + g_down * 3, cell_d);
            }

            if (node_mass[g_up] > 0) {
                qj = NodeLocation(g.x, g.y + 1, g.z, bin_edge, min_bounding_point);  // Node location
                bij = qi + real3(0, .5 * bin_edge, 0);                               // face barycenter
                cell_d = Aij / lij * (qj - bij);
                sum_d -= cell_d;
                SetRow3Check(D_T, start_node + n, body_offset + g_up * 3, cell_d);
            }
            if (node_mass[g_front] > 0) {
                qj = NodeLocation(g.x, g.y, g.z - 1, bin_edge, min_bounding_point);  // Node location
                bij = qi - real3(0, 0, .5 * bin_edge);                               // face barycenter
                cell_d = Aij / lij * (qj - bij);
                sum_d -= cell_d;
                SetRow3Check(D_T, start_node + n, body_offset + g_front * 3, cell_d);
            }
            if (node_mass[g_back] > 0) {
                qj = NodeLocation(g.x, g.y, g.z + 1, bin_edge, min_bounding_point);  // Node location
                bij = qi + real3(0, 0, .5 * bin_edge);                               // face barycenter
                cell_d = Aij / lij * (qj - bij);
                sum_d -= cell_d;
                SetRow3Check(D_T, start_node + n, body_offset + g_back * 3, cell_d);
            }

            SetRow3Check(D_T, start_node + n, body_offset + n * 3, sum_d);
        }

        //            printf("D: [%f %f %f] [%d %d %d] x [%d %d %d] y [%d %d %d] z[%d %d %d]\n", ff.x, ff.y, ff.z,
        //            g.x, g.y, g.z,
        //                   g.x + 1, g.y, g.z, g.x, g.y + 1, g.z, g.x, g.y, g.z + 1);
    }
}
void ChFLIPContainer::Build_b() {
    SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, start_node, num_mpm_nodes);
    // b_sub = 0;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    const real dt = data_manager->settings.step_size;
    for (int index = 0; index < num_mpm_nodes; index++) {
        if (node_mass[index] > 0) {
            real density = node_mass[index] / pow(bin_edge, 3.0);
            b_sub[index] = -Min((density / rho - 1), .001);
            // b_sub[index] = -(density / rho - 1.0);
            printf("b: %f %f \n", b_sub[index], density);
        }
    }
}
void ChFLIPContainer::Build_E() {
    LOG(INFO) << "ChMPMContainer::Build_E";
    // SubVectorType E_sub = blaze::subvector(data_manager->host_data.E, start_node, num_mpm_nodes);
    // E_sub = 0;
    // custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
}
void ChFLIPContainer::GenerateSparsity() {
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    LOG(INFO) << "ChFLIPContainer::GenerateSparsity" << D_T.rows() << " " << D_T.columns();
    int index_t = 0;

    if (num_rigid_mpm_contacts > 0) {
        LOG(INFO) << "ChConstraintRigidMPM::GenerateSparsity " << num_rigid_mpm_contacts;

        int index_t = 0;

        custom_vector<int>& neighbor_rigid_fluid = data_manager->host_data.neighbor_rigid_fluid;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_fluid;
        custom_vector<real3>& pos_marker = data_manager->host_data.pos_3dof;

        for (int p = 0; p < num_mpm_markers; p++) {
            int start = contact_counts[p];
            int end = contact_counts[p + 1];
            const real3 xi = pos_marker[p];
            for (int index = start; index < end; index++) {
                int i = index - start;  // index that goes from 0
                int rigid = neighbor_rigid_fluid[p * max_rigid_neighbors + i];

                AppendRow6(D_T, start_boundary + index + 0, rigid * 6, 0);
                vec3 g;

                g.x = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);
                g.y = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);
                g.z = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);

                const int current_node = GridHash(g.x, g.y, g.z, bins_per_axis);

                int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
                int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);
                int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);
                int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);
                int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
                int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);

                // AppendRow3(D_T, start_boundary + index + 0, body_offset + g_front * 3, 0);
                // AppendRow3(D_T, start_boundary + index + 0, body_offset + g_down * 3, 0);
                // AppendRow3(D_T, start_boundary + index + 0, body_offset + g_left * 3, 0);
                AppendRow3(D_T, start_boundary + index + 0, body_offset + current_node * 3, 0);
                // AppendRow3(D_T, start_boundary + index + 0, body_offset + g_right * 3, 0);
                // AppendRow3(D_T, start_boundary + index + 0, body_offset + g_up * 3, 0);
                // AppendRow3(D_T, start_boundary + index + 0, body_offset + g_back * 3, 0);

                D_T.finalize(start_boundary + index + 0);
            }
        }
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
    }

    for (int n = 0; n < num_mpm_nodes; n++) {
        vec3 g = GridDecode(n, bins_per_axis);

        int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
        int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);
        int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);
        int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);

        int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
        int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);

        if (node_mass[n] > 0) {
            if (node_mass[g_front] > 0) {
                AppendRow3(D_T, start_node + n, body_offset + g_front * 3, 0);
            }
            if (node_mass[g_down] > 0) {
                AppendRow3(D_T, start_node + n, body_offset + g_down * 3, 0);
            }
            if (node_mass[g_left] > 0) {
                AppendRow3(D_T, start_node + n, body_offset + g_left * 3, 0);
            }
            AppendRow3(D_T, start_node + n, body_offset + n * 3, 0);

            if (node_mass[g_right] > 0) {
                AppendRow3(D_T, start_node + n, body_offset + g_right * 3, 0);
            }
            if (node_mass[g_up] > 0) {
                AppendRow3(D_T, start_node + n, body_offset + g_up * 3, 0);
            }
            if (node_mass[g_back] > 0) {
                AppendRow3(D_T, start_node + n, body_offset + g_back * 3, 0);
            }
        }

        // printf("row_done\n");
        D_T.finalize(start_node + n);
    }
}
void ChFLIPContainer::PreSolve() {}
void ChFLIPContainer::PostSolve() {
    return;
    SubVectorType gamma_sub = blaze::subvector(data_manager->host_data.gamma, start_node, num_mpm_nodes);
    SubVectorType v_sub = blaze::subvector(data_manager->host_data.v, body_offset, num_mpm_nodes * 3);
    SubVectorType R_sub = blaze::subvector(data_manager->host_data.R_full, start_node, num_mpm_nodes);
    //    for (int i = 0; i < gamma_sub.size(); i++) {
    //        vec3 g = GridDecode(i, bins_per_axis);
    //        int hash = GridHash(g.x, g.y, g.z, bins_per_axis);
    //        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
    //        if (gamma_sub[i] != 0) {
    //            printf("gamma: %f [%d %d %d] %.20f [%d %d] \n", gamma_sub[i], g.x, g.y, g.z, node_mass[i], hash,
    //            i);
    //        }
    //    }
    //
    const real dt = data_manager->settings.step_size;
    //
    //    DynamicVector<real> force =
    //        submatrix(data_manager->host_data.D, body_offset, start_node, num_mpm_nodes * 3, num_mpm_nodes) *
    //        gamma_sub /
    //        dt;
    //
    //    for (int i = 0; i < gamma_sub.size(); i++) {
    //        vec3 g = GridDecode(i, bins_per_axis);
    //        printf("v: [%f,%f,%f] r: [%.10f] gam: [%.10f] i:[%d] g:[%d %d %d]  v:[%.10f]  force :[ % f, % f % f]\n ",
    //               v_sub[i * 3 + 0], v_sub[i * 3 + 1], v_sub[i * 3 + 2], R_sub[i], gamma_sub[i], i, g.x, g.y, g.z,
    //               node_mass[i] / (bin_edge * bin_edge * bin_edge), force[i * 3 + 0], force[i * 3 + 1], force[i * 3 +
    //               2]);
    //    }
    //    int size = data_manager->measures.solver.maxd_hist.size();
    //    for (int i = 0; i < size; i++) {
    //        printf("[%f %f]\n", data_manager->measures.solver.maxd_hist[i],
    //               data_manager->measures.solver.maxdeltalambda_hist[i]);
    //    }

    // Print(std::cout, data_manager->host_data.D_T, bins_per_axis);
}

void ChFLIPContainer::Solve(const DynamicVector<real>& rhs, DynamicVector<real>& delta_v) {}

void ChFLIPContainer::UpdateRhs() {}

}  // END_OF_NAMESPACE____

/////////////////////
