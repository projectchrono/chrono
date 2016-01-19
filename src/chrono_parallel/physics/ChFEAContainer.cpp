
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include <thrust/fill.h>
#include "chrono_parallel/constraints/ChConstraintUtils.h"

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR TETRAHEDRAL FEA ELEMENTS

ChFEAContainer::ChFEAContainer(ChSystemParallelDVI* system) {
    data_manager = system->data_manager;
    data_manager->AddFEAContainer(this);
}

ChFEAContainer::~ChFEAContainer() {}

void ChFEAContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;
    custom_vector<real3>& vel_node = data_manager->host_data.vel_node;

    pos_node.insert(pos_node.end(), positions.begin(), positions.end());
    vel_node.insert(vel_node.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fea nodes
    vel_node.resize(pos_node.size());
    data_manager->num_nodes = pos_node.size();
}
void ChFEAContainer::AddElements(const std::vector<uint4>& indices) {
    custom_vector<uint4>& tet_indices = data_manager->host_data.tet_indices;
    tet_indices.insert(tet_indices.end(), indices.begin(), indices.end());
    data_manager->num_tets = tet_indices.size();
}
int ChFEAContainer::GetNumConstraints() {
    int num_constraints = data_manager->num_tets * 6;  // 6 rows in the tetrahedral jacobian
    num_constraints += data_manager->num_rigid_node_contacts * 3;
    return num_constraints;
}
int ChFEAContainer::GetNumNonZeros() {
    // 12*3 entries in the elastic, 12*3 entries in the shear
    int nnz = data_manager->num_tets * 12 * 3 + data_manager->num_tets * 12 * 3;
    nnz += 9 * 3 * data_manager->num_rigid_node_contacts;
    return nnz;
}
void ChFEAContainer::ComputeInvMass(int offset) {
    CompressedMatrix<real>& M_inv = data_manager->host_data.M_inv;
    uint num_nodes = data_manager->num_nodes;
    custom_vector<real>& mass_node = data_manager->host_data.mass_node;

    for (int i = 0; i < num_nodes; i++) {
        real inv_mass = 1.0 / mass_node[i];
        M_inv.append(offset + i * 3 + 0, offset + i * 3 + 0, inv_mass);
        M_inv.finalize(offset + i * 3 + 0);
        M_inv.append(offset + i * 3 + 1, offset + i * 3 + 1, inv_mass);
        M_inv.finalize(offset + i * 3 + 1);
        M_inv.append(offset + i * 3 + 2, offset + i * 3 + 2, inv_mass);
        M_inv.finalize(offset + i * 3 + 2);
    }
}
void ChFEAContainer::ComputeMass(int offset) {
    CompressedMatrix<real>& M = data_manager->host_data.M;
    uint num_nodes = data_manager->num_nodes;
    custom_vector<real>& mass_node = data_manager->host_data.mass_node;

    for (int i = 0; i < num_nodes; i++) {
        real mass = mass_node[i];
        M.append(offset + i * 3 + 0, offset + i * 3 + 0, mass);
        M.finalize(offset + i * 3 + 0);
        M.append(offset + i * 3 + 1, offset + i * 3 + 1, mass);
        M.finalize(offset + i * 3 + 1);
        M.append(offset + i * 3 + 2, offset + i * 3 + 2, mass);
        M.finalize(offset + i * 3 + 2);
    }
}

void ChFEAContainer::Update(double ChTime) {
    uint num_nodes = data_manager->num_nodes;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;
    custom_vector<real3>& vel_node = data_manager->host_data.vel_node;
    real3 g_acc = data_manager->settings.gravity;
    custom_vector<real>& mass_node = data_manager->host_data.mass_node;
    for (int i = 0; i < num_nodes; i++) {
        real3 vel = vel_node[i];
        real3 h_gravity = data_manager->settings.step_size * mass_node[i] * g_acc;
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 0] = vel.x;
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 1] = vel.y;
        data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 2] = vel.z;

        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 0] = h_gravity.x;
        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 1] = h_gravity.y;
        data_manager->host_data.hf[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 2] = h_gravity.z;
    }
}
void ChFEAContainer::UpdatePosition(double ChTime) {
    uint num_nodes = data_manager->num_nodes;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    //
    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;
    custom_vector<real3>& vel_node = data_manager->host_data.vel_node;
    //
    for (int i = 0; i < num_nodes; i++) {
        real3 vel;
        vel.x = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 0];
        vel.y = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 1];
        vel.z = data_manager->host_data.v[num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3 + i * 3 + 2];

        real speed = Length(vel);
        if (speed > max_velocity) {
            vel = vel * max_velocity / speed;
        }
        vel_node[i] = vel;
        pos_node[i] += vel * data_manager->settings.step_size;
    }
}
void ChFEAContainer::Initialize() {
    uint num_tets = data_manager->num_tets;
    uint num_nodes = data_manager->num_nodes;

    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;
    custom_vector<uint4>& tet_indices = data_manager->host_data.tet_indices;
    custom_vector<real>& mass_node = data_manager->host_data.mass_node;

    X0.resize(num_tets);
    V.resize(num_tets);
    mass_node.resize(num_nodes);

    // initialize node masses to zero;
    Thrust_Fill(mass_node, 0);

    for (int i = 0; i < num_tets; i++) {
        uint4 tet_ind = tet_indices[i];

        real3 x0 = pos_node[tet_ind.x];
        real3 x1 = pos_node[tet_ind.y];
        real3 x2 = pos_node[tet_ind.z];
        real3 x3 = pos_node[tet_ind.w];

        real3 c1 = x1 - x0;
        real3 c2 = x2 - x0;
        real3 c3 = x3 - x0;
        Mat33 Ds = Mat33(c1, c2, c3);

        X0[i] = Inverse(Ds);
        real det = Determinant(Ds);
        real vol = (det) / 6.0;
        real volSqrt = Sqrt(vol);

        V[i] = vol;

        real tet_mass = material_density * vol;
        real node_mass = tet_mass / 4.0;

        mass_node[tet_ind.x] += node_mass;
        mass_node[tet_ind.y] += node_mass;
        mass_node[tet_ind.z] += node_mass;
        mass_node[tet_ind.w] += node_mass;

        // printf("Vol: %f Mass: %f \n", vol, tet_mass);

        //        real3 y[4];
        //        y[1] = X0[i].row(0);
        //        y[2] = X0[i].row(1);
        //        y[3] = X0[i].row(2);
        //        y[0] = -y[1] - y[2] - y[3];
    }

    //    ed = yDamping;
    //    nud = pDamping;
    //    real omnd = 1.0 - nud;
    //    real om2nd = 1.0 - 2 * nud;
    //    real fd = ed / (1.0 + nud) / om2nd;
    //    Mat33 Ed = fd * Mat33(omnd, nud, nud,   //
    //                          nud, omnd, nud,   //
    //                          nud, nud, omnd);  //
}

bool Cone_generalized_rnode(real& gamma_n, real& gamma_u, real& gamma_v, const real& mu) {
    real f_tang = sqrt(gamma_u * gamma_u + gamma_v * gamma_v);

    // inside upper cone? keep untouched!
    if (f_tang < (mu * gamma_n)) {
        return false;
    }

    // inside lower cone? reset  normal,u,v to zero!
    if ((f_tang) < -(1.0 / mu) * gamma_n || (fabs(gamma_n) < 10e-15)) {
        gamma_n = 0;
        gamma_u = 0;
        gamma_v = 0;
        return false;
    }

    // remaining case: project orthogonally to generator segment of upper cone

    gamma_n = (f_tang * mu + gamma_n) / (mu * mu + 1);
    real tproj_div_t = (gamma_n * mu) / f_tang;
    gamma_u *= tproj_div_t;
    gamma_v *= tproj_div_t;

    return true;
}

void ChFEAContainer::Project(real* gamma) {
    // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
    uint num_rigid_node_contacts = data_manager->num_rigid_node_contacts;
    real mu = data_manager->fea_container->contact_mu;
    real coh = data_manager->fea_container->contact_cohesion;

    custom_vector<int>& neighbor_rigid_node = data_manager->host_data.neighbor_rigid_node;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_node;
    uint num_nodes = data_manager->num_nodes;
    uint num_tets = data_manager->num_tets;
    //#pragma omp parallel for
    int index = 0;
    for (int p = 0; p < num_nodes; p++) {
        for (int i = 0; i < contact_counts[p]; ++i) {
            int rigid = neighbor_rigid_node[p * max_rigid_neighbors + i];  // rigid is stored in the first index
            int node = p;                                                  // fluid body is in second index
            real rigid_fric = data_manager->host_data.fric_data[rigid].x;
            real cohesion = Max((data_manager->host_data.cohesion_data[rigid] + coh) * .5, 0.0);
            real friction = (rigid_fric == 0 || mu == 0) ? 0 : (rigid_fric + mu) * .5;

            real3 gam;
            gam.x = gamma[start_row + num_tets * 6 + index];
            gam.y = gamma[start_row + num_tets * 6 + num_rigid_node_contacts + index * 2 + 0];
            gam.z = gamma[start_row + num_tets * 6 + num_rigid_node_contacts + index * 2 + 1];

            gam.x += cohesion;

            real mu = friction;
            if (mu == 0) {
                gam.x = gam.x < 0 ? 0 : gam.x - cohesion;
                gam.y = gam.z = 0;

                gamma[start_row + num_tets * 6 + index] = gam.x;
                gamma[start_row + num_tets * 6 + num_rigid_node_contacts + index * 2 + 0] = gam.y;
                gamma[start_row + num_tets * 6 + num_rigid_node_contacts + index * 2 + 1] = gam.z;
                index++;
                continue;
            }

            if (Cone_generalized_rnode(gam.x, gam.y, gam.z, mu)) {
            }

            gamma[start_row + num_tets * 6 + index] = gam.x - cohesion;
            gamma[start_row + num_tets * 6 + num_rigid_node_contacts + index * 2 + 0] = gam.y;
            gamma[start_row + num_tets * 6 + num_rigid_node_contacts + index * 2 + 1] = gam.z;
            index++;
        }
    }
}
void ChFEAContainer::Build_D() {
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_tets = data_manager->num_tets;

    real e = youngs_modulus;
    real nu = poisson_ratio;

    const real mu = 0.5 * e / (1.0 + nu);  // 0.5?
    const real muInv = 1.0 / mu;

    real omn = 1.0 - nu;
    real om2n = 1.0 - 2 * nu;
    real s = e / (1.0 + nu);
    real f = s / om2n;
    Mat33 E = f * Mat33(omn, nu, nu,   //
                        nu, omn, nu,   //
                        nu, nu, omn);  //

    Mat33 Einv = Inverse(E);
    Mat33 C_upper = Einv;
    Mat33 C_lower(real3(muInv, muInv, muInv));

    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;
    custom_vector<uint4>& tet_indices = data_manager->host_data.tet_indices;
    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    uint b_off = num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3;

    for (int i = 0; i < num_tets; i++) {
        uint4 tet_ind = tet_indices[i];

        real3 p0 = pos_node[tet_ind.x];
        real3 p1 = pos_node[tet_ind.y];
        real3 p2 = pos_node[tet_ind.z];
        real3 p3 = pos_node[tet_ind.w];

        real3 c1 = p1 - p0;
        real3 c2 = p2 - p0;
        real3 c3 = p3 - p0;
        Mat33 Ds = Mat33(c1, c2, c3);

        real det = Determinant(Ds);
        real vol = (det) / 6.0;
        real volSqrt = Sqrt(vol);
        Mat33 X = X0[i];
        Mat33 F = Ds * X;  // 4.27
        Mat33 Ftr = Transpose(F);

        Mat33 strain = 0.5 * (Ftr * F - Mat33(1));  // Green strain

        // Print(Ds, "Ds");
        // Print(X, "X");
        // Print(strain, "strain");
        // std::cin.get();

        real3 y[4];
        y[1] = X.row(0);
        y[2] = X.row(1);
        y[3] = X.row(2);
        y[0] = -y[1] - y[2] - y[3];

        Mat33 Hs[4];
        Mat33 Hn[4];

        // Green strain Jacobian
        for (int j = 0; j < 4; j++) {
            Hn[j] = Mat33(y[j].x * Ftr[0], y[j].x * Ftr[4], y[j].x * Ftr[8],  //
                          y[j].y * Ftr[1], y[j].y * Ftr[5], y[j].y * Ftr[9],  //
                          y[j].z * Ftr[2], y[j].z * Ftr[6], y[j].z * Ftr[10]);

            Hs[j] = 0.5 * Mat33(0, y[j].z, y[j].y, y[j].z, 0, y[j].x, y[j].y, y[j].x, 0) * Ftr;
        }

        real3 delV[4];

        delV[1] = Cross(c2, c3);
        delV[2] = Cross(c3, c1);
        delV[3] = Cross(c1, c2);
        delV[0] = -delV[1] - delV[2] - delV[3];

        real eps[6];
        // diagonal elements of strain matrix
        eps[0] = strain[0];
        eps[1] = strain[5];
        eps[2] = strain[10];
        // Off diagonal elements
        eps[3] = strain[9];
        eps[4] = strain[8];
        eps[5] = strain[4];

        // printf("eps [%f,%f,%f,%f,%f,%f] \n", eps[0], eps[1], eps[2], eps[3], eps[4], eps[5]);

        real gradV[12];

        gradV[0] = delV[0][0];
        gradV[1] = delV[0][1];
        gradV[2] = delV[0][2];

        gradV[3] = delV[1][0];
        gradV[4] = delV[1][1];
        gradV[5] = delV[1][2];

        gradV[6] = delV[2][0];
        gradV[7] = delV[2][1];
        gradV[8] = delV[2][2];

        gradV[9] = delV[3][0];
        gradV[10] = delV[3][1];
        gradV[11] = delV[3][2];

        real vf = (0.5 / (6.0 * volSqrt));

        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.x * 3 + 0, volSqrt * y[0].x * Ftr[0] + vf * eps[0] * gradV[0]);
        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.x * 3 + 1, volSqrt * y[0].x * Ftr[1] + vf * eps[0] * gradV[1]);
        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.x * 3 + 2, volSqrt * y[0].x * Ftr[2] + vf * eps[0] * gradV[2]);

        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.y * 3 + 0, volSqrt * y[1].x * Ftr[0] + vf * eps[0] * gradV[3]);
        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.y * 3 + 1, volSqrt * y[1].x * Ftr[1] + vf * eps[0] * gradV[4]);
        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.y * 3 + 2, volSqrt * y[1].x * Ftr[2] + vf * eps[0] * gradV[5]);

        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.z * 3 + 0, volSqrt * y[2].x * Ftr[0] + vf * eps[0] * gradV[6]);
        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.z * 3 + 1, volSqrt * y[2].x * Ftr[1] + vf * eps[0] * gradV[7]);
        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.z * 3 + 2, volSqrt * y[2].x * Ftr[2] + vf * eps[0] * gradV[8]);

        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.w * 3 + 0, volSqrt * y[3].x * Ftr[0] + vf * eps[0] * gradV[9]);
        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.w * 3 + 1, volSqrt * y[3].x * Ftr[1] + vf * eps[0] * gradV[10]);
        D_T.set(start_row + i * 6 + 0, b_off + tet_ind.w * 3 + 2, volSqrt * y[3].x * Ftr[2] + vf * eps[0] * gradV[11]);
        /////==================================================================================================================================
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.x * 3 + 0, volSqrt * y[0].y * Ftr[4] + vf * eps[1] * gradV[0]);
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.x * 3 + 1, volSqrt * y[0].y * Ftr[5] + vf * eps[1] * gradV[1]);
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.x * 3 + 2, volSqrt * y[0].y * Ftr[6] + vf * eps[1] * gradV[2]);

        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.y * 3 + 0, volSqrt * y[1].y * Ftr[4] + vf * eps[1] * gradV[3]);
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.y * 3 + 1, volSqrt * y[1].y * Ftr[5] + vf * eps[1] * gradV[4]);
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.y * 3 + 2, volSqrt * y[1].y * Ftr[6] + vf * eps[1] * gradV[5]);

        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.z * 3 + 0, volSqrt * y[2].y * Ftr[4] + vf * eps[1] * gradV[6]);
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.z * 3 + 1, volSqrt * y[2].y * Ftr[5] + vf * eps[1] * gradV[7]);
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.z * 3 + 2, volSqrt * y[2].y * Ftr[6] + vf * eps[1] * gradV[8]);

        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.w * 3 + 0, volSqrt * y[3].y * Ftr[4] + vf * eps[1] * gradV[9]);
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.w * 3 + 1, volSqrt * y[3].y * Ftr[5] + vf * eps[1] * gradV[10]);
        D_T.set(start_row + i * 6 + 1, b_off + tet_ind.w * 3 + 2, volSqrt * y[3].y * Ftr[6] + vf * eps[1] * gradV[11]);
        /////==================================================================================================================================
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.x * 3 + 0, volSqrt * y[0].z * Ftr[8] + vf * eps[2] * gradV[0]);
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.x * 3 + 1, volSqrt * y[0].z * Ftr[9] + vf * eps[2] * gradV[1]);
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.x * 3 + 2, volSqrt * y[0].z * Ftr[10] + vf * eps[2] * gradV[2]);

        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.y * 3 + 0, volSqrt * y[1].z * Ftr[8] + vf * eps[2] * gradV[3]);
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.y * 3 + 1, volSqrt * y[1].z * Ftr[9] + vf * eps[2] * gradV[4]);
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.y * 3 + 2, volSqrt * y[1].z * Ftr[10] + vf * eps[2] * gradV[5]);

        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.z * 3 + 0, volSqrt * y[2].z * Ftr[8] + vf * eps[2] * gradV[6]);
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.z * 3 + 1, volSqrt * y[2].z * Ftr[9] + vf * eps[2] * gradV[7]);
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.z * 3 + 2, volSqrt * y[2].z * Ftr[10] + vf * eps[2] * gradV[8]);

        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.w * 3 + 0, volSqrt * y[3].z * Ftr[8] + vf * eps[2] * gradV[9]);
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.w * 3 + 1, volSqrt * y[3].z * Ftr[9] + vf * eps[2] * gradV[10]);
        D_T.set(start_row + i * 6 + 2, b_off + tet_ind.w * 3 + 2, volSqrt * y[3].z * Ftr[10] + vf * eps[2] * gradV[11]);
        /////==================================================================================================================================

        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.x * 3 + 0,
                volSqrt * (Ftr[4] * y[0].z + Ftr[8] * y[0].y) + vf * eps[3] * gradV[0]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.x * 3 + 1,
                volSqrt * (Ftr[5] * y[0].z + Ftr[9] * y[0].y) + vf * eps[3] * gradV[1]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.x * 3 + 2,
                volSqrt * (Ftr[6] * y[0].z + Ftr[10] * y[0].y) + vf * eps[3] * gradV[2]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.y * 3 + 0,
                volSqrt * (Ftr[4] * y[1].z + Ftr[8] * y[1].y) + vf * eps[3] * gradV[3]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.y * 3 + 1,
                volSqrt * (Ftr[5] * y[1].z + Ftr[9] * y[1].y) + vf * eps[3] * gradV[4]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.y * 3 + 2,
                volSqrt * (Ftr[6] * y[1].z + Ftr[10] * y[1].y) + vf * eps[3] * gradV[5]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.z * 3 + 0,
                volSqrt * (Ftr[4] * y[2].z + Ftr[8] * y[2].y) + vf * eps[3] * gradV[6]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.z * 3 + 1,
                volSqrt * (Ftr[5] * y[2].z + Ftr[9] * y[2].y) + vf * eps[3] * gradV[7]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.z * 3 + 2,
                volSqrt * (Ftr[6] * y[2].z + Ftr[10] * y[2].y) + vf * eps[3] * gradV[8]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.w * 3 + 0,
                volSqrt * (Ftr[4] * y[3].z + Ftr[8] * y[3].y) + vf * eps[3] * gradV[9]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.w * 3 + 1,
                volSqrt * (Ftr[5] * y[3].z + Ftr[9] * y[3].y) + vf * eps[3] * gradV[10]);
        D_T.set(start_row + i * 6 + 3, b_off + tet_ind.w * 3 + 2,
                volSqrt * (Ftr[6] * y[3].z + Ftr[10] * y[3].y) + vf * eps[3] * gradV[11]);
        /////==================================================================================================================================

        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.x * 3 + 0,
                volSqrt * (Ftr[0] * y[0].z + Ftr[8] * y[0].x) + vf * eps[4] * gradV[0]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.x * 3 + 1,
                volSqrt * (Ftr[1] * y[0].z + Ftr[9] * y[0].x) + vf * eps[4] * gradV[1]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.x * 3 + 2,
                volSqrt * (Ftr[2] * y[0].z + Ftr[10] * y[0].x) + vf * eps[4] * gradV[2]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.y * 3 + 0,
                volSqrt * (Ftr[0] * y[1].z + Ftr[8] * y[1].x) + vf * eps[4] * gradV[3]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.y * 3 + 1,
                volSqrt * (Ftr[1] * y[1].z + Ftr[9] * y[1].x) + vf * eps[4] * gradV[4]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.y * 3 + 2,
                volSqrt * (Ftr[2] * y[1].z + Ftr[10] * y[1].x) + vf * eps[4] * gradV[5]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.z * 3 + 0,
                volSqrt * (Ftr[0] * y[2].z + Ftr[8] * y[2].x) + vf * eps[4] * gradV[6]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.z * 3 + 1,
                volSqrt * (Ftr[1] * y[2].z + Ftr[9] * y[2].x) + vf * eps[4] * gradV[7]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.z * 3 + 2,
                volSqrt * (Ftr[2] * y[2].z + Ftr[10] * y[2].x) + vf * eps[4] * gradV[8]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.w * 3 + 0,
                volSqrt * (Ftr[0] * y[3].z + Ftr[8] * y[3].x) + vf * eps[4] * gradV[9]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.w * 3 + 1,
                volSqrt * (Ftr[1] * y[3].z + Ftr[9] * y[3].x) + vf * eps[4] * gradV[10]);
        D_T.set(start_row + i * 6 + 4, b_off + tet_ind.w * 3 + 2,
                volSqrt * (Ftr[2] * y[3].z + Ftr[10] * y[3].x) + vf * eps[4] * gradV[11]);
        /////==================================================================================================================================

        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.x * 3 + 0,
                volSqrt * (Ftr[0] * y[0].y + Ftr[4] * y[0].x) + vf * eps[5] * gradV[0]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.x * 3 + 1,
                volSqrt * (Ftr[1] * y[0].y + Ftr[5] * y[0].x) + vf * eps[5] * gradV[1]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.x * 3 + 2,
                volSqrt * (Ftr[2] * y[0].y + Ftr[6] * y[0].x) + vf * eps[5] * gradV[2]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.y * 3 + 0,
                volSqrt * (Ftr[0] * y[1].y + Ftr[4] * y[1].x) + vf * eps[5] * gradV[3]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.y * 3 + 1,
                volSqrt * (Ftr[1] * y[1].y + Ftr[5] * y[1].x) + vf * eps[5] * gradV[4]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.y * 3 + 2,
                volSqrt * (Ftr[2] * y[1].y + Ftr[6] * y[1].x) + vf * eps[5] * gradV[5]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.z * 3 + 0,
                volSqrt * (Ftr[0] * y[2].y + Ftr[4] * y[2].x) + vf * eps[5] * gradV[6]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.z * 3 + 1,
                volSqrt * (Ftr[1] * y[2].y + Ftr[5] * y[2].x) + vf * eps[5] * gradV[7]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.z * 3 + 2,
                volSqrt * (Ftr[2] * y[2].y + Ftr[6] * y[2].x) + vf * eps[5] * gradV[8]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.w * 3 + 0,
                volSqrt * (Ftr[0] * y[3].y + Ftr[4] * y[3].x) + vf * eps[5] * gradV[9]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.w * 3 + 1,
                volSqrt * (Ftr[1] * y[3].y + Ftr[5] * y[3].x) + vf * eps[5] * gradV[10]);
        D_T.set(start_row + i * 6 + 5, b_off + tet_ind.w * 3 + 2,
                volSqrt * (Ftr[2] * y[3].y + Ftr[6] * y[3].x) + vf * eps[5] * gradV[11]);
    }

    custom_vector<real3>& pos_rigid = data_manager->host_data.pos_rigid;
    custom_vector<quaternion>& rot_rigid = data_manager->host_data.rot_rigid;

    real h = data_manager->fea_container->kernel_radius;
    // custom_vector<int2>& bids = data_manager->host_data.bids_rigid_fluid;
    custom_vector<real3>& cpta = data_manager->host_data.cpta_rigid_node;
    custom_vector<real3>& norm = data_manager->host_data.norm_rigid_node;
    custom_vector<int>& neighbor_rigid_node = data_manager->host_data.neighbor_rigid_node;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_node;
    int num_nodes = data_manager->num_nodes;
    //#pragma omp parallel for
    int index = 0;
    for (int p = 0; p < num_nodes; p++) {
        for (int i = 0; i < contact_counts[p]; i++) {
            int rigid = neighbor_rigid_node[p * max_rigid_neighbors + i];
            int node = p;  // fluid body is in second index
            real3 U = norm[p * max_rigid_neighbors + i], V, W;
            Orthogonalize(U, V, W);
            real3 T1, T2, T3;
            Compute_Jacobian(rot_rigid[rigid], U, V, W, cpta[p * max_rigid_neighbors + i] - pos_rigid[rigid], T1, T2,
                             T3);

            SetRow6Check(D_T, start_row + num_tets * 6 + index + 0, rigid * 6, -U, T1);
            SetRow6Check(D_T, start_row + num_tets * 6 + index * 2 + 0, rigid * 6, -V, T2);
            SetRow6Check(D_T, start_row + num_tets * 6 + index * 2 + 1, rigid * 6, -W, T3);

            SetRow3Check(D_T, start_row + num_tets * 6 + index + 0, b_off + node * 3, U);
            SetRow3Check(D_T, start_row + num_tets * 6 + index * 2 + 0, b_off + node * 3, V);
            SetRow3Check(D_T, start_row + num_tets * 6 + index * 2 + 1, b_off + node * 3, W);
            index++;
        }
    }
}
void ChFEAContainer::Build_b() {
    LOG(INFO) << "ChConstraintTet::Build_b";
    uint num_tets = data_manager->num_tets;
    SubVectorType b_sub = blaze::subvector(data_manager->host_data.b, start_row, num_tets * 6);
    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;
    custom_vector<uint4>& tet_indices = data_manager->host_data.tet_indices;

#pragma omp parallel for
    for (int i = 0; i < num_tets; i++) {
        uint4 tet_ind = tet_indices[i];

        real3 x0 = pos_node[tet_ind.x];
        real3 x1 = pos_node[tet_ind.y];
        real3 x2 = pos_node[tet_ind.z];
        real3 x3 = pos_node[tet_ind.w];

        real3 c1 = x1 - x0;
        real3 c2 = x2 - x0;
        real3 c3 = x3 - x0;
        Mat33 Ds = Mat33(c1, c2, c3);

        real det = Determinant(Ds);
        real vol = (det) / 6.0;
        real volSqrt = Sqrt(vol);
        Mat33 X = X0[i];
        Mat33 F = Ds * X;
        Mat33 Ftr = Transpose(F);

        real3 y[4];
        y[1] = X.row(0);
        y[2] = X.row(1);
        y[3] = X.row(2);
        y[0] = -y[1] - y[2] - y[3];

        Mat33 strain = 0.5 * (Ftr * F - Mat33(1));  // Green strain

        b_sub[i * 6 + 0] = volSqrt * strain[0];
        b_sub[i * 6 + 1] = volSqrt * strain[5];
        b_sub[i * 6 + 2] = volSqrt * strain[10];

        b_sub[i * 6 + 3] = volSqrt * strain[9];
        b_sub[i * 6 + 4] = volSqrt * strain[8];
        b_sub[i * 6 + 5] = volSqrt * strain[4];

        //        printf("b [%f,%f,%f,%f,%f,%f] \n", b_sub[i * 6 + 0], b_sub[i * 6 + 1], b_sub[i * 6 + 2], b_sub[i * 6 +
        //        3],
        //               b_sub[i * 6 + 4], b_sub[i * 6 + 5]);
    }

    LOG(INFO) << "ChConstraintRigidNode::Build_b";
    uint num_rigid_node_contacts = data_manager->num_rigid_node_contacts;
    uint num_unilaterals = data_manager->num_unilaterals;
    uint num_bilaterals = data_manager->num_bilaterals;
    real step_size = data_manager->settings.step_size;
    if (num_rigid_node_contacts > 0) {
        custom_vector<int>& neighbor_rigid_node = data_manager->host_data.neighbor_rigid_node;
        custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_node;
        int num_nodes = data_manager->num_nodes;

        //#pragma omp parallel for
        int index = 0;
        for (int p = 0; p < num_nodes; p++) {
            for (int i = 0; i < contact_counts[p]; i++) {
                real bi = 0;
                real depth = data_manager->host_data.dpth_rigid_node[p * max_rigid_neighbors + i];

                bi = std::max(real(1.0) / step_size * depth, -data_manager->fea_container->contact_recovery_speed);
                //
                data_manager->host_data.b[start_row + num_tets * 6 + index + 0] = bi;
                data_manager->host_data.b[start_row + num_tets * 6 + num_rigid_node_contacts + index * 2 + 0] = 0;
                data_manager->host_data.b[start_row + num_tets * 6 + num_rigid_node_contacts + index * 2 + 1] = 0;
                index++;
            }
        }
    }
}
void ChFEAContainer::Build_E() {}

template <typename T>
static void inline AppendRow12(T& D, const int row, const int offset, const uint4 col, const real init) {
    D.append(row, offset + col.x * 3 + 0, init);
    D.append(row, offset + col.x * 3 + 1, init);
    D.append(row, offset + col.x * 3 + 2, init);

    D.append(row, offset + col.y * 3 + 0, init);
    D.append(row, offset + col.y * 3 + 1, init);
    D.append(row, offset + col.y * 3 + 2, init);

    D.append(row, offset + col.z * 3 + 0, init);
    D.append(row, offset + col.z * 3 + 1, init);
    D.append(row, offset + col.z * 3 + 2, init);

    D.append(row, offset + col.w * 3 + 0, init);
    D.append(row, offset + col.w * 3 + 1, init);
    D.append(row, offset + col.w * 3 + 2, init);
}
void ChFEAContainer::GenerateSparsity() {
    uint num_tets = data_manager->num_tets;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;
    uint num_rigid_node_contacts = data_manager->num_rigid_node_contacts;

    uint body_offset = num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3;

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    custom_vector<uint4>& tet_indices = data_manager->host_data.tet_indices;
    // std::cout << "start_row " << start_row << std::endl;
    for (int i = 0; i < num_tets; i++) {
        uint4 tet_ind = tet_indices[i];

        tet_ind = Sort(tet_ind);

        AppendRow12(D_T, start_row + i * 6 + 0, body_offset, tet_ind, 0);
        D_T.finalize(start_row + i * 6 + 0);

        AppendRow12(D_T, start_row + i * 6 + 1, body_offset, tet_ind, 0);
        D_T.finalize(start_row + i * 6 + 1);

        AppendRow12(D_T, start_row + i * 6 + 2, body_offset, tet_ind, 0);
        D_T.finalize(start_row + i * 6 + 2);
        ///==================================================================================================================================

        AppendRow12(D_T, start_row + i * 6 + 3, body_offset, tet_ind, 0);
        D_T.finalize(start_row + i * 6 + 3);

        AppendRow12(D_T, start_row + i * 6 + 4, body_offset, tet_ind, 0);
        D_T.finalize(start_row + i * 6 + 4);

        AppendRow12(D_T, start_row + i * 6 + 5, body_offset, tet_ind, 0);
        D_T.finalize(start_row + i * 6 + 5);
    }

    int index_n = 0;
    int index_t = 0;
    int num_nodes = data_manager->num_fluid_bodies;
    custom_vector<int>& neighbor_rigid_node = data_manager->host_data.neighbor_rigid_node;
    custom_vector<int>& contact_counts = data_manager->host_data.c_counts_rigid_node;
    int off = start_row + num_tets * 6;
    for (int p = 0; p < num_nodes; p++) {
        for (int i = 0; i < contact_counts[p]; i++) {
            int rigid = neighbor_rigid_node[p * max_rigid_neighbors + i];
            int node = p;
            AppendRow6(D_T, off + index_n + 0, rigid * 6, 0);
            AppendRow3(D_T, off + index_n + 0, body_offset + node * 3, 0);
            D_T.finalize(off + index_n + 0);
            index_n++;
        }
    }
    for (int p = 0; p < num_nodes; p++) {
        for (int i = 0; i < contact_counts[p]; i++) {
            int rigid = neighbor_rigid_node[p * max_rigid_neighbors + i];
            int node = p;

            AppendRow6(D_T, off + num_rigid_node_contacts + index_t * 2 + 0, rigid * 6, 0);
            AppendRow3(D_T, off + num_rigid_node_contacts + index_t * 2 + 0, body_offset + node * 3, 0);
            D_T.finalize(off + num_rigid_node_contacts + index_t * 2 + 0);

            AppendRow6(D_T, off + num_rigid_node_contacts + index_t * 2 + 1, rigid * 6, 0);
            AppendRow3(D_T, off + num_rigid_node_contacts + index_t * 2 + 1, body_offset + node * 3, 0);

            D_T.finalize(off + num_rigid_node_contacts + index_t * 2 + 1);
            index_t++;
        }
    }
}
}  // END_OF_NAMESPACE____

/////////////////////
