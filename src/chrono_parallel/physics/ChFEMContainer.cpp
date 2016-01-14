
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
void ChFEMContainer::AddTets(const std::vector<int4>& indices) {
    custom_vector<int4>& tet_indices = data_manager->host_data.tet_indices;
    tet_indices.insert(tet_indices.end(), indices.begin(), indices.end());
    data_manager->num_tets = tet_indices.size();
}
int ChFEMContainer::GetNumConstraints() {
    int num_constraints = data_manager->num_tets * 6;  // 6 rows in the tetrahedral jacobian
    return num_constraints;
}
int ChFEMContainer::GetNumNonZeros() {
    // 12 entries in the elastic, 24 entries in the shear
    int nnz = data_manager->num_tets * 12 + data_manager->num_tets * 24;

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
    custom_vector<int4>& tet_indices = data_manager->host_data.tet_indices;
    custom_vector<real>& mass_node = data_manager->host_data.mass_node;
    // initialize node masses to zero;
    Thrust_Fill(mass_node, 0);

    for (int i = 0; i < num_tets; i++) {
        int4 tet_index = tet_indices[i];

        real3 x0 = pos_node[tet_index.x];
        real3 x1 = pos_node[tet_index.y];
        real3 x2 = pos_node[tet_index.z];
        real3 x3 = pos_node[tet_index.w];

        real3 c1 = x1 - x0;
        real3 c2 = x2 - x0;
        real3 c3 = x3 - x0;
        Mat33 D = Mat33(c1, c2, c3);

        X0[i] = Inverse(D);
        V[i] = Determinant(D) / 6.0;

        real tet_mass = material_density * V[i];
        real node_mass = tet_mass / 4.0;

        mass_node[tet_index.x] += node_mass;
        mass_node[tet_index.y] += node_mass;
        mass_node[tet_index.z] += node_mass;
        mass_node[tet_index.w] += node_mass;

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
}
}  // END_OF_NAMESPACE____

/////////////////////
