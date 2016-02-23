
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

namespace chrono {

using namespace collision;
using namespace geometry;

#define RINGS 2

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

void Print(std::ostream& os, const CompressedMatrix<real>& M, int3 bins_per_axis) {
    for (size_t i = 0UL; i < (~M).rows(); ++i) {
        // os << "";
        for (size_t j = 0UL; j < (~M).columns(); ++j) {
            // os << std::setw(1) << (~M)(i, j) << ", ";
            if ((~M)(i, j) != 0) {
                int3 g = GridDecode(i, bins_per_axis);
                int3 f = GridDecode(j / 3, bins_per_axis);
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

#if 1
    max_bounding_point = real3(res.second.x, res.second.y, res.second.z) + kernel_radius * MAX_OFF;
    min_bounding_point = real3(res.first.x, res.first.y, res.first.z) - kernel_radius * MIN_OFF;
#else
    max_bounding_point = real3(.2, .15, .15);
    min_bounding_point = real3(-.2, -.15, -.15);
#endif

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
    DynamicVector<real>& v = data_manager->host_data.v;
    custom_vector<real>& old_vel_node_mpm = data_manager->host_data.old_vel_node_mpm;
    const real dt = data_manager->settings.step_size;
    real3 g_acc = data_manager->settings.gravity;
    Setup(0);

    node_mass.resize(num_mpm_nodes);
    face_density.resize(num_mpm_nodes);
    face_volume.resize(num_mpm_nodes);
    data_manager->host_data.old_vel_node_mpm.resize(num_mpm_nodes * 3);

    //#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        v[body_offset + i * 3 + 0] = 0;
        v[body_offset + i * 3 + 1] = 0;
        v[body_offset + i * 3 + 2] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 0] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 1] = 0;
        data_manager->host_data.hf[body_offset + i * 3 + 2] = 0;
        node_mass[i] = 0;
        face_density[i] = real3(0, 0, 0);
        face_volume[i] = real3(0, 0, 0);
    }

    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        const real3 vi = vel_marker[p];
        //        {
        //            const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);
        //            const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);
        //            const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);
        //            printf("Particle: %d [%d %d %d] \n", p, cx, cy, cz);
        //        }

        LOOPOVERNODESY(
            //
            real weight_x = N(xi - (current_node_location - real3(.5 * bin_edge, 0, 0)), inv_bin_edge);  //
            real weight_y = N(xi - (current_node_location - real3(0, .5 * bin_edge, 0)), inv_bin_edge);  //
            real weight_z = N(xi - (current_node_location - real3(0, 0, .5 * bin_edge)), inv_bin_edge);  //
                                                                                                         //
            if (i >= cx - 1 && j < cy + 2 && k < cz + 2) {
                face_density[current_node].x += weight_x * mass;  //
                face_volume[current_node].x += mass / rho * weight_x;
                v[body_offset + current_node * 3 + 0] += mass * weight_x * vi.x;  //
            }                                                                     //
            if (j >= cy - 1 && i < cx + 2 && k < cz + 2) {                        //
                face_density[current_node].y += weight_y * mass;                  //
                face_volume[current_node].y += mass / rho * weight_y;             //
                v[body_offset + current_node * 3 + 1] += mass * weight_y * vi.y;  //
            }                                                                     //
            if (k >= cz - 1 && i < cx + 2 && j < cy + 2) {
                face_density[current_node].z += weight_z * mass;                  //
                face_volume[current_node].z += mass / rho * weight_z;             //
                v[body_offset + current_node * 3 + 2] += mass * weight_z * vi.z;  //
            },
            RINGS);
    }

#pragma omp parallel for
    for (int i = 0; i < num_mpm_nodes; i++) {
        int3 g = GridDecode(i, bins_per_axis);
        int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
        int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);

        int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);
        int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);

        int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
        int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);

        if (face_density[i].x > 0) {
            v[body_offset + i * 3 + 0] /= face_density[i].x;
        }
        if (face_density[i].y > 0) {
            v[body_offset + i * 3 + 1] /= face_density[i].y;
        }
        if (face_density[i].z > 0) {
            v[body_offset + i * 3 + 2] /= face_density[i].z;
        }
        node_mass[i] = face_density[i].x + face_density[i].y + face_density[i].z;

        data_manager->host_data.old_vel_node_mpm[i * 3 + 0] = v[body_offset + i * 3 + 0];
        data_manager->host_data.old_vel_node_mpm[i * 3 + 1] = v[body_offset + i * 3 + 1];
        data_manager->host_data.old_vel_node_mpm[i * 3 + 2] = v[body_offset + i * 3 + 2];

        /// printf("new_vel: [%f %f %f] \n", face_density[i].x, face_density[i].y, face_density[i].z);
    }
}

void ChFLIPContainer::UpdatePosition(double ChTime) {
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    custom_vector<real>& old_vel_node_mpm = data_manager->host_data.old_vel_node_mpm;
    custom_vector<real>& node_mass = data_manager->host_data.node_mass;
    DynamicVector<real>& v = data_manager->host_data.v;
    printf("Update_Particle_Velocities\n");
#pragma omp parallel for
    for (int p = 0; p < num_mpm_markers; p++) {
        const real3 xi = pos_marker[p];
        real3 V_flip = vel_marker[p];
        real3 V_pic = real3(0.0);
        LOOPOVERNODESY(                                                                                               //
            real weight_x = N(xi - (current_node_location - real3(.5 * bin_edge, 0, 0)), inv_bin_edge);               //
            real weight_y = N(xi - (current_node_location - real3(0, .5 * bin_edge, 0)), inv_bin_edge);               //
            real weight_z = N(xi - (current_node_location - real3(0, 0, .5 * bin_edge)), inv_bin_edge);               //
            V_pic.x += v[body_offset + current_node * 3 + 0] * weight_x;                                              //
            V_pic.y += v[body_offset + current_node * 3 + 1] * weight_y;                                              //
            V_pic.z += v[body_offset + current_node * 3 + 2] * weight_z;                                              //
            V_flip.x += (v[body_offset + current_node * 3 + 0] - old_vel_node_mpm[current_node * 3 + 0]) * weight_x;  //
            V_flip.y += (v[body_offset + current_node * 3 + 1] - old_vel_node_mpm[current_node * 3 + 1]) * weight_y;  //
            V_flip.z += (v[body_offset + current_node * 3 + 2] - old_vel_node_mpm[current_node * 3 + 2]) * weight_z;  //
            , RINGS                                                                                                   //
            )
        real3 new_vel = (1.0 - alpha) * V_pic + alpha * V_flip;

        //        real speed = Length(new_vel);
        //        if (speed > max_velocity) {
        //            new_vel = new_vel * max_velocity / speed;
        //        }

        //        printf("new_vel: [%f %f %f] pic:[%f %f %f] flip:[%f %f %f]\n", new_vel.x, new_vel.y, new_vel.z,
        //        V_pic.x,
        //               V_pic.y, V_pic.z, V_flip.x, V_flip.y, V_flip.z);

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

    real N_x = N(real3(.5 * bin_edge, 0, 0), inv_bin_edge);  //
    real N_y = N(real3(0, .5 * bin_edge, 0), inv_bin_edge);  //
    real N_z = N(real3(0, 0, .5 * bin_edge), inv_bin_edge);  //
    real volx = N_x * N_x * N_x;
    real voly = N_y * N_y * N_y;
    real volz = N_z * N_z * N_z;

    printf("Volume of cell: %f face_vol = %.20f %.20f %.20f [%.20f %.20f %.20f]\n", bin_edge * bin_edge * bin_edge,
           volx, voly, volz, N_x, N_y, N_z);

    for (int i = 0; i < num_mpm_nodes; i++) {
        real3 inv_mass = real3(0);

        int3 g = GridDecode(i, bins_per_axis);
#if 0
        if (node_mass[i] > 0) {
            real density = node_mass[i] / (bin_edge * bin_edge * bin_edge);

            inv_mass.x = dt / rho;  //(face_density[i].x / face_volume[i].x);
            inv_mass.y = dt / rho;  //(face_density[i].y / face_volume[i].y);
            inv_mass.z = dt / rho;  //(face_density[i].z / face_volume[i].z);
        }
#else
        if (face_volume[i].x > 0) {
            inv_mass.x = dt / (face_density[i].x / face_volume[i].x);
        }
        if (face_volume[i].y > 0) {
            inv_mass.y = dt / (face_density[i].y / face_volume[i].y);
        }
        if (face_volume[i].z > 0) {
            inv_mass.z = dt / (face_density[i].z / face_volume[i].z);
        }

#endif

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
    custom_vector<real>& node_mass = data_manager->host_data.node_mass;

    for (int i = 0; i < num_mpm_nodes; i++) {
        int3 g = GridDecode(i, bins_per_axis);

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
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);
    printf("Initialize [%d] [%d %d %d] [%f] %d\n", num_mpm_nodes, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z,
           bin_edge, num_mpm_markers);

    data_manager->host_data.marker_volume.resize(num_mpm_markers);
    data_manager->host_data.node_mass.resize(num_mpm_nodes);
    std::fill(data_manager->host_data.node_mass.begin(), data_manager->host_data.node_mass.end(), 0);

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
    custom_vector<real>& node_mass = data_manager->host_data.node_mass;
    for (int i = 0; i < num_mpm_constraints; i++) {
        int3 g = GridDecode(i, bins_per_axis);
        if ((g.x + 1) >= bins_per_axis.x) {
            gamma[start_node + i] = 0;
        }
        if ((g.y + 1) >= bins_per_axis.y) {
            gamma[start_node + i] = 0;
        }
        if ((g.z + 1) >= bins_per_axis.z) {
            gamma[start_node + i] = 0;
        }
        if (g.x == 0) {
            gamma[start_node + i] = 0;
        }
        if (g.y == 0) {
            gamma[start_node + i] = 0;
        }
        if (g.z == 0) {
            gamma[start_node + i] = 0;
        }

        // Dirchlet pressure boundary condition for empty cells
        //        if (node_mass[i] <= C_EPSILON) {
        //            gamma[start_node + i] = 0;
        //        }

        if (face_density[i].x <= 0 || face_density[i].y <= 0 || face_density[i].z <= 0) {
            gamma[start_node + i] = 0;
        }

        //        if (gamma[start_node + i] < 0) {
        //            gamma[start_node + i] = 0;
        //        }
    }
}

void ChFLIPContainer::Build_D() {
    LOG(INFO) << "ChFLIPContainer::Build_D";
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    custom_vector<real3>& vel_marker = data_manager->host_data.vel_marker_mpm;
    const real dt = data_manager->settings.step_size;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    custom_vector<real>& old_vel_node_mpm = data_manager->host_data.old_vel_node_mpm;

    custom_vector<real>& node_mass = data_manager->host_data.node_mass;

    for (int n = 0; n < num_mpm_nodes; n++) {
        int3 g = GridDecode(n, bins_per_axis);
        int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
        int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);
        int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);
        int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);
        int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
        int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);

        real factor = inv_bin_edge;
        real3 ff(0, 0, 0);
        if ((g.x + 1) >= bins_per_axis.x) {
        } else if ((g.y + 1) >= bins_per_axis.y) {
        } else if ((g.z + 1) >= bins_per_axis.z) {
        } else {
            //            if (face_density[n].x != 0) {
            //                ff.x = -factor;
            //            }
            //            if (face_density[n].y != 0) {
            //                ff.y = -factor;
            //            }
            //            if (face_density[n].z != 0) {
            //                ff.z = -factor;
            //            }

            if (node_mass[n] > 0) {
                ff.x = -factor;
                ff.y = -factor;
                ff.z = -factor;

                SetRow3Check(D_T, start_row + n, body_offset + n * 3, ff);
                SetRow3Check(D_T, start_row + n, body_offset + g_right * 3, real3(-ff.x, 0, 0));
                SetRow3Check(D_T, start_row + n, body_offset + g_up * 3, real3(0, -ff.y, 0));
                SetRow3Check(D_T, start_row + n, body_offset + g_back * 3, real3(0, 0, -ff.z));
            }

            //            printf("D: [%f %f %f] [%d %d %d] x [%d %d %d] y [%d %d %d] z[%d %d %d]\n", ff.x, ff.y, ff.z,
            //            g.x, g.y, g.z,
            //                   g.x + 1, g.y, g.z, g.x, g.y + 1, g.z, g.x, g.y, g.z + 1);
        }
    }
}
void ChFLIPContainer::Build_b() {
    SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, start_node, num_mpm_constraints);
    b_sub = 0;
    custom_vector<real3>& pos_marker = data_manager->host_data.pos_marker_mpm;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    const real dt = data_manager->settings.step_size;
    custom_vector<real>& node_mass = data_manager->host_data.node_mass;
    //    for (int index = 0; index < num_mpm_nodes; index++) {
    //        if (node_mass[index] > 0) {
    //            real density = data_manager->host_data.node_mass[index] / (bin_edge * bin_edge * bin_edge);
    //            // b_sub[index] = -(Max(density / rho - 1.0, 0));
    //            b_sub[index] = -(density / rho - 1.0);
    //        }
    //    }
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

        int g_left = GridHash(g.x - 1, g.y, g.z, bins_per_axis);
        int g_right = GridHash(g.x + 1, g.y, g.z, bins_per_axis);
        int g_up = GridHash(g.x, g.y + 1, g.z, bins_per_axis);
        int g_down = GridHash(g.x, g.y - 1, g.z, bins_per_axis);

        int g_front = GridHash(g.x, g.y, g.z - 1, bins_per_axis);
        int g_back = GridHash(g.x, g.y, g.z + 1, bins_per_axis);

        if ((g.x + 1) >= bins_per_axis.x) {
        } else if ((g.y + 1) >= bins_per_axis.y) {
        } else if ((g.z + 1) >= bins_per_axis.z) {
        } else {
            AppendRow3(D_T, start_row + n, body_offset + n * 3, 0);
            AppendRow3(D_T, start_row + n, body_offset + g_right * 3, 0);
            AppendRow3(D_T, start_row + n, body_offset + g_up * 3, 0);
            AppendRow3(D_T, start_row + n, body_offset + g_back * 3, 0);
        }

        // printf("row_done\n");
        D_T.finalize(start_row + n);
    }
}
void ChFLIPContainer::PreSolve() {}
void ChFLIPContainer::PostSolve() {
    SubVectorType gamma_sub = blaze::subvector(data_manager->host_data.gamma, start_node, num_mpm_constraints);
    custom_vector<real>& node_mass = data_manager->host_data.node_mass;
    SubVectorType v_sub = blaze::subvector(data_manager->host_data.v, body_offset, num_mpm_nodes * 3);
    SubVectorType R_sub = blaze::subvector(data_manager->host_data.R_full, start_node, num_mpm_constraints);
    //    for (int i = 0; i < gamma_sub.size(); i++) {
    //        int3 g = GridDecode(i, bins_per_axis);
    //        int hash = GridHash(g.x, g.y, g.z, bins_per_axis);
    //        real3 current_node_location = NodeLocation(g.x, g.y, g.z, bin_edge, min_bounding_point);
    //        if (gamma_sub[i] != 0) {
    //            printf("gamma: %f [%d %d %d] %.20f [%d %d] \n", gamma_sub[i], g.x, g.y, g.z, node_mass[i], hash,
    //            i);
    //        }
    //    }
    //
    const real dt = data_manager->settings.step_size;

    DynamicVector<real> force =
        submatrix(data_manager->host_data.D, body_offset, start_row, num_mpm_nodes * 3, num_mpm_nodes) * gamma_sub / dt;

    //    for (int i = 0; i < gamma_sub.size(); i++) {
    //        int3 g = GridDecode(i, bins_per_axis);
    //        printf(
    //            "v: [%f,%f,%f] r: [%.10f] gam: [%.10f] i:[%d] g:[%d %d %d]  v:[%.10f] fm:[%f,%f,%f] force :[ % f, % f,
    //            % f "
    //            "]\n ",
    //            v_sub[i * 3 + 0], v_sub[i * 3 + 1], v_sub[i * 3 + 2], R_sub[i], gamma_sub[i], i, g.x, g.y, g.z,
    //            node_mass[i] / (bin_edge * bin_edge * bin_edge), face_density[i].x, face_density[i].y,
    //            face_density[i].z,
    //            force[i * 3 + 0], force[i * 3 + 1], force[i * 3 + 2]);
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
