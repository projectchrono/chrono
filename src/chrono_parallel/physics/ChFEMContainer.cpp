
#include <stdlib.h>
#include <algorithm>
#include <math.h>
#include "chrono_parallel/physics/ChSystemParallel.h"
#include <chrono_parallel/physics/Ch3DOFContainer.h>
#include <thrust/fill.h>

namespace chrono {

using namespace collision;
using namespace geometry;

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A 3DOF FLUID NODE

ChFEMContainer::ChFEMContainer(ChSystemParallelDVI* system) {}

ChFEMContainer::~ChFEMContainer() {}

void ChFEMContainer::AddNodes(const std::vector<real3>& positions, const std::vector<real3>& velocities) {
    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;
    custom_vector<real3>& vel_node = data_manager->host_data.vel_node;

    pos_node.insert(pos_node.end(), positions.begin(), positions.end());
    vel_node.insert(vel_node.end(), velocities.begin(), velocities.end());
    // In case the number of velocities provided were not enough, resize to the number of fluid bodies
    vel_node.resize(pos_node.size());
    data_manager->num_nodes = pos_node.size();
}
void ChFEMContainer::AddElements(const std::vector<uint4>& indices) {
    custom_vector<uint4>& tet_indices = data_manager->host_data.tet_indices;
    tet_indices.insert(tet_indices.end(), indices.begin(), indices.end());
    data_manager->num_tets = tet_indices.size();
}
int ChFEMContainer::GetNumConstraints() {
    int num_constraints = data_manager->num_tets * 6;  // 6 rows in the tetrahedral jacobian
    return num_constraints;
}
int ChFEMContainer::GetNumNonZeros() {
    // 12*3 entries in the elastic, 12*3 entries in the shear
    int nnz = data_manager->num_tets * 12 * 3 + data_manager->num_tets * 12 * 3;

    return nnz;
}
void ChFEMContainer::ComputeInvMass(int offset) {
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
void ChFEMContainer::ComputeMass(int offset) {
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
void ChFEMContainer::Initialize() {
    uint num_tets = data_manager->num_tets;

    X0.resize(num_tets);

    custom_vector<real3>& pos_node = data_manager->host_data.pos_node;
    custom_vector<uint4>& tet_indices = data_manager->host_data.tet_indices;
    custom_vector<real>& mass_node = data_manager->host_data.mass_node;
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
        Mat33 D = Mat33(c1, c2, c3);

        X0[i] = Inverse(D);
        V[i] = Determinant(D) / 6.0;

        real tet_mass = material_density * V[i];
        real node_mass = tet_mass / 4.0;

        mass_node[tet_ind.x] += node_mass;
        mass_node[tet_ind.y] += node_mass;
        mass_node[tet_ind.z] += node_mass;
        mass_node[tet_ind.w] += node_mass;

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

void ChFEMContainer::Build_D() {
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

        real3 x0 = pos_node[tet_ind.x];
        real3 x1 = pos_node[tet_ind.y];
        real3 x2 = pos_node[tet_ind.z];
        real3 x3 = pos_node[tet_ind.w];

        real3 c1 = x1 - x0;
        real3 c2 = x2 - x0;
        real3 c3 = x3 - x0;
        Mat33 Ds = Mat33(c1, c2, c3);

        real det = Determinant(Ds);
        real vol = Abs(det) / 6.0;
        real volSqrt = Sqrt(vol);
        Mat33 X = X0[i];
        Mat33 F = Ds * X;
        Mat33 Ftr = Transpose(F);

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

        Mat33 strain = 0.5 * (Ftr * F - Mat33(1));  // Green strain
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
}
void ChFEMContainer::Build_b() {
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
        real vol = Abs(det) / 6.0;
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
    }
}
void ChFEMContainer::Build_E() {}

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
void ChFEMContainer::GenerateSparsity() {
    uint num_tets = data_manager->num_tets;
    uint num_rigid_bodies = data_manager->num_rigid_bodies;
    uint num_shafts = data_manager->num_shafts;
    uint num_fluid_bodies = data_manager->num_fluid_bodies;

    uint body_offset = num_rigid_bodies * 6 + num_shafts + num_fluid_bodies * 3;

    CompressedMatrix<real>& D_T = data_manager->host_data.D_T;
    custom_vector<uint4>& tet_indices = data_manager->host_data.tet_indices;

    for (int i = 0; i < num_tets; i++) {
        uint4 tet_ind = tet_indices[i];
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
}
}  // END_OF_NAMESPACE____

/////////////////////
