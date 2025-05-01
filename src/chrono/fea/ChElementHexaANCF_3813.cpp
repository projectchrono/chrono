// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bryan Peterson, Antonio Recuero, Radu Serban
// =============================================================================
// Hexahedronal element with 8 nodes (with EAS)
// =============================================================================

//// RADU
//// A lot more to do here...
//// - reconsider the use of large static matrices
//// - more use of Eigen expressions

# include <cmath>

#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChElementHexaANCF_3813.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------

ChElementHexaANCF_3813::ChElementHexaANCF_3813() : m_flag_HE(ANALYTICAL) {
    m_nodes.resize(8);
}

// -----------------------------------------------------------------------------
void ChElementHexaANCF_3813::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
                                      std::shared_ptr<ChNodeFEAxyz> nodeB,
                                      std::shared_ptr<ChNodeFEAxyz> nodeC,
                                      std::shared_ptr<ChNodeFEAxyz> nodeD,
                                      std::shared_ptr<ChNodeFEAxyz> nodeE,
                                      std::shared_ptr<ChNodeFEAxyz> nodeF,
                                      std::shared_ptr<ChNodeFEAxyz> nodeG,
                                      std::shared_ptr<ChNodeFEAxyz> nodeH) {
    assert(nodeA);
    assert(nodeB);
    assert(nodeC);
    assert(nodeD);
    assert(nodeE);
    assert(nodeF);
    assert(nodeG);
    assert(nodeH);

    m_nodes[0] = nodeA;
    m_nodes[1] = nodeB;
    m_nodes[2] = nodeC;
    m_nodes[3] = nodeD;
    m_nodes[4] = nodeE;
    m_nodes[5] = nodeF;
    m_nodes[6] = nodeG;
    m_nodes[7] = nodeH;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[3]->Variables());
    mvars.push_back(&m_nodes[4]->Variables());
    mvars.push_back(&m_nodes[5]->Variables());
    mvars.push_back(&m_nodes[6]->Variables());
    mvars.push_back(&m_nodes[7]->Variables());
    Kmatr.SetVariables(mvars);
    // EAS
    // Initial position
    ChVector3d pA = m_nodes[0]->GetPos();
    ChVector3d pB = m_nodes[1]->GetPos();
    ChVector3d pC = m_nodes[2]->GetPos();
    ChVector3d pD = m_nodes[3]->GetPos();
    ChVector3d pE = m_nodes[4]->GetPos();
    ChVector3d pF = m_nodes[5]->GetPos();
    ChVector3d pG = m_nodes[6]->GetPos();
    ChVector3d pH = m_nodes[7]->GetPos();
    m_d0(0, 0) = pA[0];
    m_d0(0, 1) = pA[1];
    m_d0(0, 2) = pA[2];
    m_d0(1, 0) = pB[0];
    m_d0(1, 1) = pB[1];
    m_d0(1, 2) = pB[2];
    m_d0(2, 0) = pC[0];
    m_d0(2, 1) = pC[1];
    m_d0(2, 2) = pC[2];
    m_d0(3, 0) = pD[0];
    m_d0(3, 1) = pD[1];
    m_d0(3, 2) = pD[2];
    m_d0(4, 0) = pE[0];
    m_d0(4, 1) = pE[1];
    m_d0(4, 2) = pE[2];
    m_d0(5, 0) = pF[0];
    m_d0(5, 1) = pF[1];
    m_d0(5, 2) = pF[2];
    m_d0(6, 0) = pG[0];
    m_d0(6, 1) = pG[1];
    m_d0(6, 2) = pG[2];
    m_d0(7, 0) = pH[0];
    m_d0(7, 1) = pH[1];
    m_d0(7, 2) = pH[2];
    // EAS
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::SetStockAlpha(double a1,
                                           double a2,
                                           double a3,
                                           double a4,
                                           double a5,
                                           double a6,
                                           double a7,
                                           double a8,
                                           double a9) {
    m_stock_alpha_EAS(0) = a1;
    m_stock_alpha_EAS(1) = a2;
    m_stock_alpha_EAS(2) = a3;
    m_stock_alpha_EAS(3) = a4;
    m_stock_alpha_EAS(4) = a5;
    m_stock_alpha_EAS(5) = a6;
    m_stock_alpha_EAS(6) = a7;
    m_stock_alpha_EAS(7) = a8;
    m_stock_alpha_EAS(8) = a9;
}

// -----------------------------------------------------------------------------

// Internal force, EAS stiffness, and analytical jacobian are calculated
class Brick_ForceAnalytical : public ChIntegrand3D<ChVectorN<double, 906>> {
  public:
    Brick_ForceAnalytical(ChMatrixNM<double, 8, 3>* d_,
                          ChMatrixNM<double, 8, 3>* d0_,
                          ChElementHexaANCF_3813* element_,
                          ChMatrixNM<double, 6, 6>* T0_,
                          double* detJ0C_,
                          ChVectorN<double, 9>* alpha_eas_);

    Brick_ForceAnalytical(ChMatrixNM<double, 8, 3>* d_,
                          ChMatrixNM<double, 8, 3>* d0_,
                          ChElementHexaANCF_3813* element_,
                          ChMatrixNM<double, 6, 6>* T0_,
                          double* detJ0C_,
                          ChVectorN<double, 9>* alpha_eas_,
                          double* E_,
                          double* v_);
    ~Brick_ForceAnalytical() {}

  private:
    ChElementHexaANCF_3813* element;
    ChMatrixNM<double, 8, 3>* d;      // Pointer to a matrix containing the element coordinates
    ChMatrixNM<double, 8, 3>* d0;     // Pointer to a matrix containing the element initial coordinates
    ChMatrixNM<double, 6, 6>* T0;     // Pointer to transformation matrix for Enhanced Assumed Strain (EAS)
    ChVectorN<double, 9>* alpha_eas;  // Pointer to the 9 internal parameters for EAS
    double* detJ0C;                   // Pointer to determinant of the initial Jacobian at the element center
    double* E;                        // Pointer to Young modulus
    double* v;                        // Pointer to Poisson ratio

    ChVectorN<double, 24> Fint;              // Generalized internal (elastic) force vector
    ChMatrixNM<double, 24, 24> JAC11;        // Jacobian of internal forces for implicit numerical integration
    ChMatrixNM<double, 9, 24> Gd;            // Jacobian (w.r.t. coordinates) of the initial pos. vector gradient matrix
    ChVectorN<double, 6> stress;             // stress tensor in vector form
    ChMatrixNM<double, 9, 9> Sigm;           // stress tensor in sparse form
    ChMatrixNM<double, 6, 6> E_eps;          // Matrix of elastic coefficients (features orthotropy)
    ChMatrixNM<double, 3, 24> Sx;            // Sparse shape function matrix, X derivative
    ChMatrixNM<double, 3, 24> Sy;            // Sparse shape function matrix, Y derivative
    ChMatrixNM<double, 3, 24> Sz;            // Sparse shape function matrix, Z derivative
    ChElementHexaANCF_3813::ShapeVector Nx;  // Dense shape function vector, X derivative
    ChElementHexaANCF_3813::ShapeVector Ny;  // Dense shape function vector, Y derivative
    ChElementHexaANCF_3813::ShapeVector Nz;  // Dense shape function vector, Z derivative
    ChMatrixNM<double, 6, 24> strainD;       // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
    ChVectorN<double, 6> strain;             // Vector of strains
    double detJ0;                            // Determinant of the initial position vector gradient matrix
    // EAS
    ChMatrixNM<double, 6, 9> M;       // Shape function matrix for Enhanced Assumed Strain
    ChMatrixNM<double, 6, 9> G;       // Matrix G interpolates the internal parameters of EAS
    ChVectorN<double, 6> strain_EAS;  // Enhanced assumed strain vector

    // Evaluate (strainD'*strain)  at a point
    virtual void Evaluate(ChVectorN<double, 906>& result, const double x, const double y, const double z) override;
};

Brick_ForceAnalytical::Brick_ForceAnalytical(ChMatrixNM<double, 8, 3>* d_,
                                             ChMatrixNM<double, 8, 3>* d0_,
                                             ChElementHexaANCF_3813* element_,
                                             ChMatrixNM<double, 6, 6>* T0_,
                                             double* detJ0C_,
                                             ChVectorN<double, 9>* alpha_eas_)
    : element(element_), d(d_), d0(d0_), T0(T0_), alpha_eas(alpha_eas_), detJ0C(detJ0C_) {
    E_eps.setZero();
    Gd.setZero();
    Sigm.setZero();

    Sx.setZero();
    Sy.setZero();
    Sz.setZero();
}

Brick_ForceAnalytical::Brick_ForceAnalytical(ChMatrixNM<double, 8, 3>* d_,
                                             ChMatrixNM<double, 8, 3>* d0_,
                                             ChElementHexaANCF_3813* element_,
                                             ChMatrixNM<double, 6, 6>* T0_,
                                             double* detJ0C_,
                                             ChVectorN<double, 9>* alpha_eas_,
                                             double* E_,
                                             double* v_)
    : element(element_), d(d_), d0(d0_), T0(T0_), alpha_eas(alpha_eas_), detJ0C(detJ0C_), E(E_), v(v_) {
    E_eps.setZero();
    Gd.setZero();
    Sigm.setZero();

    Sx.setZero();
    Sy.setZero();
    Sz.setZero();
}

void Brick_ForceAnalytical::Evaluate(ChVectorN<double, 906>& result, const double x, const double y, const double z) {
    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

    element->Basis_M(M, x, y, z);  // EAS

    if (!element->m_isMooney) {  // m_isMooney == false means use linear material
        double DD = (*E) * (1.0 - (*v)) / ((1.0 + (*v)) * (1.0 - 2.0 * (*v)));
        E_eps.fillDiagonal(1.0);
        E_eps(0, 1) = (*v) / (1.0 - (*v));
        E_eps(0, 3) = (*v) / (1.0 - (*v));
        E_eps(1, 0) = (*v) / (1.0 - (*v));
        E_eps(1, 3) = (*v) / (1.0 - (*v));
        E_eps(2, 2) = (1.0 - 2.0 * (*v)) / (2.0 * (1.0 - (*v)));
        E_eps(3, 0) = (*v) / (1.0 - (*v));
        E_eps(3, 1) = (*v) / (1.0 - (*v));
        E_eps(4, 4) = (1.0 - 2.0 * (*v)) / (2.0 * (1.0 - (*v)));
        E_eps(5, 5) = (1.0 - 2.0 * (*v)) / (2.0 * (1.0 - (*v)));
        E_eps *= DD;
    }

    // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]

    for (int i = 0; i < 8; i++) {
        Sx(0, 3 * i + 0) = Nx(i);
        Sx(1, 3 * i + 1) = Nx(i);
        Sx(2, 3 * i + 2) = Nx(i);
    }

    for (int i = 0; i < 8; i++) {
        Sy(0, 3 * i + 0) = Ny(i);
        Sy(1, 3 * i + 1) = Ny(i);
        Sy(2, 3 * i + 2) = Ny(i);
    }

    for (int i = 0; i < 8; i++) {
        Sz(0, 3 * i + 0) = Nz(i);
        Sz(1, 3 * i + 1) = Nz(i);
        Sz(2, 3 * i + 2) = Nz(i);
    }

    // EAS and Initial Shape
    ChMatrixNM<double, 3, 3> rd0;
    rd0.col(0) = (*d0).transpose() * Nx.transpose();
    rd0.col(1) = (*d0).transpose() * Ny.transpose();
    rd0.col(2) = (*d0).transpose() * Nz.transpose();
    detJ0 = rd0.determinant();

    // Transformation : Orthogonal transformation (A and J)

    ChVector3d G1;
    ChVector3d G2;
    ChVector3d G3;
    ChVector3d G1xG2;
    G1[0] = rd0(0, 0);
    G2[0] = rd0(0, 1);
    G3[0] = rd0(0, 2);
    G1[1] = rd0(1, 0);
    G2[1] = rd0(1, 1);
    G3[1] = rd0(1, 2);
    G1[2] = rd0(2, 0);
    G2[2] = rd0(2, 1);
    G3[2] = rd0(2, 2);
    G1xG2.Cross(G1, G2);

    // Tangent Frame
    ChVector3d A1 = G1 / std::sqrt(G1[0] * G1[0] + G1[1] * G1[1] + G1[2] * G1[2]);
    ChVector3d A3 = G1xG2 / std::sqrt(G1xG2[0] * G1xG2[0] + G1xG2[1] * G1xG2[1] + G1xG2[2] * G1xG2[2]);
    ChVector3d A2 = A3.Cross(A1);

    // Direction for orthotropic material
    double theta = 0.0;
    ChVector3d AA1 = A1 * std::cos(theta) + A2 * std::sin(theta);
    ChVector3d AA2 = -A1 * std::sin(theta) + A2 * std::cos(theta);
    ChVector3d AA3 = A3;

    // Beta
    ChMatrixNM<double, 3, 3> j0 = rd0.inverse();
    ChVector3d j01;
    ChVector3d j02;
    ChVector3d j03;
    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);
    ChVectorN<double, 9> beta;
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    // Enhanced Assumed Strain
    G = (*T0) * M * ((*detJ0C) / (detJ0));
    strain_EAS = G * (*alpha_eas);

    ChMatrixNM<double, 8, 8> d_d = (*d) * (*d).transpose();
    ChVectorN<double, 8> ddNx = d_d * Nx.transpose();
    ChVectorN<double, 8> ddNy = d_d * Ny.transpose();
    ChVectorN<double, 8> ddNz = d_d * Nz.transpose();

    ChMatrixNM<double, 8, 8> d0_d0 = (*d0) * (*d0).transpose();
    ChVectorN<double, 8> d0d0Nx = d0_d0 * Nx.transpose();
    ChVectorN<double, 8> d0d0Ny = d0_d0 * Ny.transpose();
    ChVectorN<double, 8> d0d0Nz = d0_d0 * Nz.transpose();

    // Strain component

    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * Nx.dot(ddNx - d0d0Nx);
    strain_til(1) = 0.5 * Ny.dot(ddNy - d0d0Ny);
    strain_til(2) = Nx.dot(ddNy - d0d0Ny);
    //== Compatible strain (No ANS) ==//
    strain_til(3) = 0.5 * Nz.dot(ddNz - d0d0Nz);
    strain_til(4) = Nx.dot(ddNz - d0d0Nz);
    strain_til(5) = Ny.dot(ddNz - d0d0Nz);

    //		//// For orthotropic material ///
    strain(0) = strain_til(0) * beta(0) * beta(0) + strain_til(1) * beta(3) * beta(3) +
                strain_til(2) * beta(0) * beta(3) + strain_til(3) * beta(6) * beta(6) +
                strain_til(4) * beta(0) * beta(6) + strain_til(5) * beta(3) * beta(6);
    strain(1) = strain_til(0) * beta(1) * beta(1) + strain_til(1) * beta(4) * beta(4) +
                strain_til(2) * beta(1) * beta(4) + strain_til(3) * beta(7) * beta(7) +
                strain_til(4) * beta(1) * beta(7) + strain_til(5) * beta(4) * beta(7);
    strain(2) = strain_til(0) * 2.0 * beta(0) * beta(1) + strain_til(1) * 2.0 * beta(3) * beta(4) +
                strain_til(2) * (beta(1) * beta(3) + beta(0) * beta(4)) + strain_til(3) * 2.0 * beta(6) * beta(7) +
                strain_til(4) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                strain_til(5) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3) = strain_til(0) * beta(2) * beta(2) + strain_til(1) * beta(5) * beta(5) +
                strain_til(2) * beta(2) * beta(5) + strain_til(3) * beta(8) * beta(8) +
                strain_til(4) * beta(2) * beta(8) + strain_til(5) * beta(5) * beta(8);
    strain(4) = strain_til(0) * 2.0 * beta(0) * beta(2) + strain_til(1) * 2.0 * beta(3) * beta(5) +
                strain_til(2) * (beta(2) * beta(3) + beta(0) * beta(5)) + strain_til(3) * 2.0 * beta(6) * beta(8) +
                strain_til(4) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                strain_til(5) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5) = strain_til(0) * 2.0 * beta(1) * beta(2) + strain_til(1) * 2.0 * beta(4) * beta(5) +
                strain_til(2) * (beta(2) * beta(4) + beta(1) * beta(5)) + strain_til(3) * 2.0 * beta(7) * beta(8) +
                strain_til(4) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                strain_til(5) * (beta(5) * beta(7) + beta(4) * beta(8));

    // Straint derivative component

    ChMatrixNM<double, 6, 24> strainD_til;
    strainD_til.row(0) = Nx * (*d) * Sx;
    strainD_til.row(1) = Ny * (*d) * Sy;
    strainD_til.row(2) = Nx * (*d) * Sy + Ny * (*d) * Sx;
    //== Compatible strain (No ANS)==//
    strainD_til.row(3) = Nz * (*d) * Sz;
    strainD_til.row(4) = Nx * (*d) * Sz + Nz * (*d) * Sx;
    strainD_til.row(5) = Ny * (*d) * Sz + Nz * (*d) * Sy;

    // For orthotropic material
    for (int ii = 0; ii < 24; ii++) {
        strainD(0, ii) = strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                         strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                         strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
        strainD(1, ii) = strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                         strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                         strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
        strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) + strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                         strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                         strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                         strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
        strainD(3, ii) = strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
                         strainD_til(2, ii) * beta(2) * beta(5) + strainD_til(3, ii) * beta(8) * beta(8) +
                         strainD_til(4, ii) * beta(2) * beta(8) + strainD_til(5) * beta(5) * beta(8);
        strainD(4, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(2) + strainD_til(1, ii) * 2.0 * beta(3) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(6) + beta(3) * beta(8));
        strainD(5, ii) = strainD_til(0, ii) * 2.0 * beta(1) * beta(2) + strainD_til(1, ii) * 2.0 * beta(4) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(7) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(7) + beta(4) * beta(8));
    }

    // Gd (8x24) calculation
    for (int ii = 0; ii < 8; ii++) {
        Gd(0, 3 * (ii) + 0) = j0(0, 0) * Nx(ii) + j0(1, 0) * Ny(ii) + j0(2, 0) * Nz(ii);
        Gd(1, 3 * (ii) + 1) = j0(0, 0) * Nx(ii) + j0(1, 0) * Ny(ii) + j0(2, 0) * Nz(ii);
        Gd(2, 3 * (ii) + 2) = j0(0, 0) * Nx(ii) + j0(1, 0) * Ny(ii) + j0(2, 0) * Nz(ii);

        Gd(3, 3 * (ii) + 0) = j0(0, 1) * Nx(ii) + j0(1, 1) * Ny(ii) + j0(2, 1) * Nz(ii);
        Gd(4, 3 * (ii) + 1) = j0(0, 1) * Nx(ii) + j0(1, 1) * Ny(ii) + j0(2, 1) * Nz(ii);
        Gd(5, 3 * (ii) + 2) = j0(0, 1) * Nx(ii) + j0(1, 1) * Ny(ii) + j0(2, 1) * Nz(ii);

        Gd(6, 3 * (ii) + 0) = j0(0, 2) * Nx(ii) + j0(1, 2) * Ny(ii) + j0(2, 2) * Nz(ii);
        Gd(7, 3 * (ii) + 1) = j0(0, 2) * Nx(ii) + j0(1, 2) * Ny(ii) + j0(2, 2) * Nz(ii);
        Gd(8, 3 * (ii) + 2) = j0(0, 2) * Nx(ii) + j0(1, 2) * Ny(ii) + j0(2, 2) * Nz(ii);
    }

    // Enhanced Assumed Strain 2nd
    strain += strain_EAS;

    ChMatrixNM<double, 9, 6> temp56;
    ChVectorN<double, 9> HE1;
    ChMatrixNM<double, 9, 24> GDEPSP;
    ChMatrixNM<double, 9, 9> KALPHA;

    // If Mooney-Rivlin Material is selected -> Calculates internal forces and their Jacobian accordingly (new E_eps)
    if (element->m_isMooney) {
        ChMatrix33<> CG;     // CG: Right Cauchy-Green deformation tensor  C=trans(F)*F
        ChMatrix33<> INVCG;  // INVCG: Inverse of Right Cauchy-Green deformation tensor  C=trans(F)*F
        ChMatrix33<> I1PC;   // Stress tensor from first term of Mooney-Rivlin strain energy
        ChMatrix33<> I2PC;   // Stress tensor from second term of Mooney-Rivlin strain energy
        ChMatrix33<> JPC;    // Stress tensor from penalty term to ensure incompressibility
        ChMatrix33<> STR;    // Stress tensor from strain energy (with penalty for incompressibility CCOM3)

        // Same quantities for the numerical calculation of the Jacobian of Mooney-Rivlin forces
        ChMatrix33<> CGN;     // CG: Right Cauchy-Green deformation tensor  C=trans(F)*F
        ChMatrix33<> INVCGN;  // INVCG: Inverse of Right Cauchy-Green deformation tensor  C=trans(F)*F
        ChMatrix33<> I1PCN;   // Stress tensor from first term of Mooney-Rivlin strain energy
        ChMatrix33<> I2PCN;   // Stress tensor from second term of Mooney-Rivlin strain energy
        ChMatrix33<> JPCN;    // Stress tensor from penalty term to ensure incompressibility
        ChMatrix33<> STRN;    // Stress tensor from strain energy (with penalty for incompressibility CCOM3)

        ChVectorN<double, 6> strain_1;

        // Right Cauchy - Green deformation tensor
        CG(0, 0) = 2.0 * strain(0) + 1.0;
        CG(1, 1) = 2.0 * strain(1) + 1.0;
        CG(2, 2) = 2.0 * strain(3) + 1.0;
        CG(1, 0) = strain(2);
        CG(0, 1) = CG(1, 0);
        CG(2, 0) = strain(4);
        CG(0, 2) = CG(2, 0);
        CG(2, 1) = strain(5);
        CG(1, 2) = CG(2, 1);

        INVCG = CG.inverse();

        // Calculation of invariants I1, I2, and I3 and its deviatoric counterparts I1bar, I2bar, and I3bar
        double Deld = 0.000001;
        // First invariant of Right Cauchy-Green deformation tensor
        double I1 = CG(0, 0) + CG(1, 1) + CG(2, 2);
        // Second invariant of Right Cauchy-Green deformation tensor
        double I2 = 0.5 * (std::pow(I1, 2) - (std::pow(CG(0, 0), 2) + std::pow(CG(1, 0), 2) + std::pow(CG(2, 0), 2) +
                                              std::pow(CG(0, 1), 2) + std::pow(CG(1, 1), 2) + std::pow(CG(2, 1), 2) +
                                              std::pow(CG(0, 2), 2) + std::pow(CG(1, 2), 2) + std::pow(CG(2, 2), 2)));
        // Third invariant of Right Cauchy-Green deformation tensor (must be very close to 1 for incompressible
        // material)
        double I3 = CG(0, 0) * CG(1, 1) * CG(2, 2) - CG(0, 0) * CG(1, 2) * CG(2, 1) + CG(0, 1) * CG(1, 2) * CG(2, 0) -
                    CG(0, 1) * CG(1, 0) * CG(2, 2) + CG(0, 2) * CG(1, 0) * CG(2, 1) - CG(2, 0) * CG(1, 1) * CG(0, 2);
        double cbrtI3inv = 1.0 / std::cbrt(I3);
        double J = std::sqrt(I3);
        // double CCOM1 = 551584.0;                                    // C10   not 0.551584
        // double CCOM2 = 137896.0;                                    // C01   not 0.137896
        double CCOM3 = 2.0 * (element->CCOM1 + element->CCOM2) / (1.0 - 2.0 * 0.49);  // K:bulk modulus
        double StockEPS;

        /// Calculation of stress tensor STR term to term: I1PC, I2PC, and JPC.

        // Stress tensor from first term of Mooney-Rivlin strain energy
        I1PC = (ChMatrix33<>::Identity() - INVCG * (CH_1_3 * I1)) * cbrtI3inv;
        // Stress tensor from second term of Mooney-Rivlin strain energy
        I2PC = (((ChMatrix33<>::Identity() * I1) - CG) - (INVCG * CH_2_3 * I2)) * cbrtI3inv * cbrtI3inv;
        // Stress tensor from penalty for incompressibility
        JPC = INVCG * (J / 2.0);
        // Definition of stress tensor from strain energy (including penalty for incompressibility CCOM3)
        STR = I1PC * (element->CCOM1 * 2.0) + I2PC * (element->CCOM2 * 2.0) + JPC * (CCOM3 * (J - 1.0) * 2.0);

        // Put the stress in vector form
        ChVectorN<double, 6> TEMP5;
        TEMP5(0) = STR(0, 0);
        TEMP5(1) = STR(1, 1);
        TEMP5(2) = STR(0, 1);
        TEMP5(3) = STR(2, 2);
        TEMP5(4) = STR(0, 2);
        TEMP5(5) = STR(1, 2);

        E_eps.setZero();

        // Compatible plus enhanced assumed strain
        strain_1 = strain;

        // Loop to obtain our Mooney-Rivling E_eps (tangential matrix of elastic coefficients)
        // E_eps is necessary for obtaining the Jacobian of MR internal forces
        ChVectorN<double, 6> TEMP5N;
        for (int JJJ = 0; JJJ < 6; JJJ++) {
            StockEPS = strain_1(JJJ);
            strain_1(JJJ) = StockEPS + Deld;
            CGN(0, 0) = 2.0 * strain_1(0) + 1.0;
            CGN(1, 1) = 2.0 * strain_1(1) + 1.0;
            CGN(2, 2) = 2.0 * strain_1(3) + 1.0;
            CGN(1, 0) = strain_1(2);
            CGN(0, 1) = CGN(1, 0);
            CGN(2, 0) = strain_1(4);
            CGN(0, 2) = CGN(2, 0);
            CGN(2, 1) = strain_1(5);
            CGN(1, 2) = CGN(2, 1);
            INVCGN = CGN.inverse();

            I1 = CGN(0, 0) + CGN(1, 1) + CGN(2, 2);
            I2 = 0.5 * (std::pow(I1, 2) - (std::pow(CGN(0, 0), 2) + std::pow(CGN(1, 0), 2) + std::pow(CGN(2, 0), 2) +
                                           std::pow(CGN(0, 1), 2) + std::pow(CGN(1, 1), 2) + std::pow(CGN(2, 1), 2) +
                                           std::pow(CGN(0, 2), 2) + std::pow(CGN(1, 2), 2) + std::pow(CGN(2, 2), 2)));
            I3 = CGN(0, 0) * CGN(1, 1) * CGN(2, 2) - CGN(0, 0) * CGN(1, 2) * CGN(2, 1) +
                 CGN(0, 1) * CGN(1, 2) * CGN(2, 0) - CGN(0, 1) * CGN(1, 0) * CGN(2, 2) +
                 CGN(0, 2) * CGN(1, 0) * CGN(2, 1) - CGN(2, 0) * CGN(1, 1) * CGN(0, 2);
            cbrtI3inv = 1.0 / std::cbrt(I3);
            J = std::sqrt(I3);
            I1PCN = (ChMatrix33<>::Identity() - INVCGN * (CH_1_3 * I1)) * cbrtI3inv;
            I2PCN = (((ChMatrix33<>::Identity() * I1) - CGN) - (INVCGN * CH_2_3 * I2)) * cbrtI3inv * cbrtI3inv;
            JPCN = INVCGN * (J / 2.0);
            STRN = I1PCN * (element->CCOM1 * 2.0) + I2PCN * (element->CCOM2 * 2.0) + JPCN * (CCOM3 * (J - 1.0) * 2.0);
            TEMP5N(0) = STRN(0, 0);
            TEMP5N(1) = STRN(1, 1);
            TEMP5N(2) = STRN(0, 1);
            TEMP5N(3) = STRN(2, 2);
            TEMP5N(4) = STRN(0, 2);
            TEMP5N(5) = STRN(1, 2);
            strain_1(JJJ) = StockEPS;
            E_eps(JJJ, 0) = (TEMP5N(0) - TEMP5(0)) / Deld;
            E_eps(JJJ, 1) = (TEMP5N(1) - TEMP5(1)) / Deld;
            E_eps(JJJ, 2) = (TEMP5N(2) - TEMP5(2)) / Deld;
            E_eps(JJJ, 3) = (TEMP5N(3) - TEMP5(3)) / Deld;
            E_eps(JJJ, 4) = (TEMP5N(4) - TEMP5(4)) / Deld;
            E_eps(JJJ, 5) = (TEMP5N(5) - TEMP5(5)) / Deld;
        }
        // Add internal forces to Fint and HE1 for Mooney-Rivlin
        temp56 = G.transpose() * E_eps;
        Fint = strainD.transpose() * TEMP5;
        Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        HE1 = G.transpose() * TEMP5;
        HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        Sigm(0, 0) = TEMP5(0);
        Sigm(1, 1) = TEMP5(0);
        Sigm(2, 2) = TEMP5(0);
        Sigm(0, 3) = TEMP5(2);
        Sigm(1, 4) = TEMP5(2);
        Sigm(2, 5) = TEMP5(2);
        Sigm(0, 6) = TEMP5(4);
        Sigm(1, 7) = TEMP5(4);
        Sigm(2, 8) = TEMP5(4);
        Sigm(3, 0) = TEMP5(2);
        Sigm(4, 1) = TEMP5(2);
        Sigm(5, 2) = TEMP5(2);
        Sigm(3, 3) = TEMP5(1);
        Sigm(4, 4) = TEMP5(1);
        Sigm(5, 5) = TEMP5(1);
        Sigm(3, 6) = TEMP5(5);
        Sigm(4, 7) = TEMP5(5);
        Sigm(5, 8) = TEMP5(5);
        Sigm(6, 0) = TEMP5(4);
        Sigm(7, 1) = TEMP5(4);
        Sigm(8, 2) = TEMP5(4);
        Sigm(6, 3) = TEMP5(5);
        Sigm(7, 4) = TEMP5(5);
        Sigm(8, 5) = TEMP5(5);
        Sigm(6, 6) = TEMP5(3);
        Sigm(7, 7) = TEMP5(3);
        Sigm(8, 8) = TEMP5(3);
    } else {
        /// Stress tensor calculation
        stress = E_eps * strain;
        Sigm(0, 0) = stress(0);
        Sigm(0, 3) = stress(2);
        Sigm(0, 6) = stress(4);
        Sigm(1, 1) = stress(0);
        Sigm(1, 4) = stress(2);
        Sigm(1, 7) = stress(4);
        Sigm(2, 2) = stress(0);
        Sigm(2, 5) = stress(2);
        Sigm(2, 8) = stress(4);
        // XX                   //XY                     //XZ
        Sigm(3, 0) = stress(2);
        Sigm(3, 3) = stress(1);
        Sigm(3, 6) = stress(5);
        Sigm(4, 1) = stress(2);
        Sigm(4, 4) = stress(1);
        Sigm(4, 7) = stress(5);
        Sigm(5, 2) = stress(2);
        Sigm(5, 5) = stress(1);
        Sigm(5, 8) = stress(5);
        // XY                  //YY                     //YZ
        Sigm(6, 0) = stress(4);
        Sigm(6, 3) = stress(5);
        Sigm(6, 6) = stress(3);
        Sigm(7, 1) = stress(4);
        Sigm(7, 4) = stress(5);
        Sigm(7, 7) = stress(3);
        Sigm(8, 2) = stress(4);
        Sigm(8, 5) = stress(5);
        Sigm(8, 8) = stress(3);
        // XZ                     //YZ                     //ZZ
        // Add internal forces to Fint and HE1 for linear elastic material
        // temp56 is actually 9x6 for the brick element (9 EAS internal parameters)
        temp56 = G.transpose() * E_eps;
        // Add generalized internal force
        Fint = strainD.transpose() * E_eps * strain;
        Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        // Add EAS internal force (vector of 9 components for each element)
        HE1 = temp56 * strain;
        HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    }  // end of   if(isMooney==1)

    // Internal force (linear isotropic or Mooney-Rivlin) Jacobian calculation
    // First term for Jacobian matrix
    // Second term for Jacobian matrix
    JAC11 = strainD.transpose() * E_eps * strainD + Gd.transpose() * Sigm * Gd;
    // Final expression for the Jacobian
    JAC11 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    // Jacobian of EAS forces w.r.t. element coordinates
    double factor_g =
        detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    GDEPSP = factor_g * temp56 * strainD;
    // Jacobian of EAS forces (w.r.t. EAS internal parameters)
    double factor_k =
        detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    KALPHA = factor_k * temp56 * G;

    ChVectorN<double, 216> GDEPSPVec;
    ChVectorN<double, 81> KALPHAVec;
    ChVectorN<double, 576> JACVec;

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 24; j++) {
            GDEPSPVec(i * 24 + j) = GDEPSP(i, j);
        }
    }

    // GDEPSP = GDEPSPvec;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            KALPHAVec(i * 9 + j) = KALPHA(i, j);
        }
    }

    // KALPHAVec = KALPHA;
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 24; j++) {
            JACVec(i * 24 + j) = JAC11(i, j);
        }
    }

    // JACVec = JAC11;
    result.segment(0, 24) = Fint;
    result.segment(24, 9) = HE1;
    result.segment(33, 216) = GDEPSPVec;
    result.segment(249, 81) = KALPHAVec;
    result.segment(330, 576) = JACVec;
}

// -----------------------------------------------------------------------------

class Brick_ForceNumerical : public ChIntegrand3D<ChVectorN<double, 330>> {
  public:
    Brick_ForceNumerical(ChMatrixNM<double, 8, 3>* d_,
                         ChMatrixNM<double, 8, 3>* d0_,
                         ChElementHexaANCF_3813* element_,
                         ChMatrixNM<double, 6, 6>* T0_,
                         double* detJ0C_,
                         ChVectorN<double, 9>* alpha_eas_);

    Brick_ForceNumerical(ChMatrixNM<double, 8, 3>* d_,
                         ChMatrixNM<double, 8, 3>* d0_,
                         ChElementHexaANCF_3813* element_,
                         ChMatrixNM<double, 6, 6>* T0_,
                         double* detJ0C_,
                         ChVectorN<double, 9>* alpha_eas_,
                         double* E_,
                         double* v_);
    ~Brick_ForceNumerical() {}

  private:
    ChElementHexaANCF_3813* element;
    // Pointers used for external values
    ChMatrixNM<double, 8, 3>* d;             // Pointer to a matrix containing the element coordinates
    ChMatrixNM<double, 8, 3>* d0;            // Pointer to a matrix containing the element initial coordinates
    ChMatrixNM<double, 6, 6>* T0;            // Pointer to transformation matrix for Enhanced Assumed Strain (EAS)
    ChVectorN<double, 9>* alpha_eas;         // Pointer to the 9 internal parameters for EAS
    double* detJ0C;                          // Pointer to determinant of the initial Jacobian at the element center
    double* E;                               // Pointer to Young modulus
    double* v;                               // Pointer to Poisson ratio
    ChVectorN<double, 24> Fint;              // Generalized internal (elastic) force vector
    ChMatrixNM<double, 6, 6> E_eps;          // Matrix of elastic coefficients (features orthotropy)
    ChMatrixNM<double, 3, 24> Sx;            // Sparse shape function matrix, X derivative
    ChMatrixNM<double, 3, 24> Sy;            // Sparse shape function matrix, Y derivative
    ChMatrixNM<double, 3, 24> Sz;            // Sparse shape function matrix, Z derivative
    ChElementHexaANCF_3813::ShapeVector Nx;  // Dense shape function vector, X derivative
    ChElementHexaANCF_3813::ShapeVector Ny;  // Dense shape function vector, Y derivative
    ChElementHexaANCF_3813::ShapeVector Nz;  // Dense shape function vector, Z derivative
    ChMatrixNM<double, 6, 24> strainD;       // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
    ChVectorN<double, 6> strain;             // Vector of strains
    double detJ0;                            // Determinant of the initial position vector gradient matrix
    // EAS
    ChMatrixNM<double, 6, 9> M;       // Shape function matrix for Enhanced Assumed Strain
    ChMatrixNM<double, 6, 9> G;       // Matrix G interpolates the internal parameters of EAS
    ChVectorN<double, 6> strain_EAS;  // Enhanced assumed strain vector

    // Gaussian integration to calculate internal forces and EAS matrices
    virtual void Evaluate(ChVectorN<double, 330>& result, const double x, const double y, const double z) override;
};

Brick_ForceNumerical::Brick_ForceNumerical(ChMatrixNM<double, 8, 3>* d_,
                                           ChMatrixNM<double, 8, 3>* d0_,
                                           ChElementHexaANCF_3813* element_,
                                           ChMatrixNM<double, 6, 6>* T0_,
                                           double* detJ0C_,
                                           ChVectorN<double, 9>* alpha_eas_)
    : element(element_), d(d_), d0(d0_), T0(T0_), alpha_eas(alpha_eas_), detJ0C(detJ0C_) {
    E_eps.setZero();

    Sx.setZero();
    Sy.setZero();
    Sz.setZero();
}

Brick_ForceNumerical::Brick_ForceNumerical(ChMatrixNM<double, 8, 3>* d_,
                                           ChMatrixNM<double, 8, 3>* d0_,
                                           ChElementHexaANCF_3813* element_,
                                           ChMatrixNM<double, 6, 6>* T0_,
                                           double* detJ0C_,
                                           ChVectorN<double, 9>* alpha_eas_,
                                           double* E_,
                                           double* v_)
    : element(element_), d(d_), d0(d0_), T0(T0_), alpha_eas(alpha_eas_), detJ0C(detJ0C_), E(E_), v(v_) {
    E_eps.setZero();

    Sx.setZero();
    Sy.setZero();
    Sz.setZero();
}

void Brick_ForceNumerical::Evaluate(ChVectorN<double, 330>& result, const double x, const double y, const double z) {
    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);
    element->Basis_M(M, x, y, z);  // EAS

    if (!element->m_isMooney) {  // m_isMooney == false means linear elastic material
        double DD = (*E) * (1.0 - (*v)) / ((1.0 + (*v)) * (1.0 - 2.0 * (*v)));
        E_eps.fillDiagonal(1.0);
        E_eps(0, 1) = (*v) / (1.0 - (*v));
        E_eps(0, 3) = (*v) / (1.0 - (*v));
        E_eps(1, 0) = (*v) / (1.0 - (*v));
        E_eps(1, 3) = (*v) / (1.0 - (*v));
        E_eps(2, 2) = (1.0 - 2.0 * (*v)) / (2.0 * (1.0 - (*v)));
        E_eps(3, 0) = (*v) / (1.0 - (*v));
        E_eps(3, 1) = (*v) / (1.0 - (*v));
        E_eps(4, 4) = (1.0 - 2.0 * (*v)) / (2.0 * (1.0 - (*v)));
        E_eps(5, 5) = (1.0 - 2.0 * (*v)) / (2.0 * (1.0 - (*v)));
        E_eps *= DD;
    }
    // Expand shape functions Sx, Sy, Sz
    // Sx=[Nx1*eye(3) Nx2*eye(3) Nx3*eye(3) Nx4*eye(3) Nx5*eye(3) Nx6*eye(3) Nx7*eye(3) Nx8*eye(3)]

    for (int i = 0; i < 8; i++) {
        Sx(0, 3 * i + 0) = Nx(i);
        Sx(1, 3 * i + 1) = Nx(i);
        Sx(2, 3 * i + 2) = Nx(i);
    }

    for (int i = 0; i < 8; i++) {
        Sy(0, 3 * i + 0) = Ny(i);
        Sy(1, 3 * i + 1) = Ny(i);
        Sy(2, 3 * i + 2) = Ny(i);
    }

    for (int i = 0; i < 8; i++) {
        Sz(0, 3 * i + 0) = Nz(i);
        Sz(1, 3 * i + 1) = Nz(i);
        Sz(2, 3 * i + 2) = Nz(i);
    }

    //==EAS and Initial Shape==//
    ChMatrixNM<double, 3, 3> rd0;
    rd0.col(0) = (*d0).transpose() * Nx.transpose();
    rd0.col(1) = (*d0).transpose() * Ny.transpose();
    rd0.col(2) = (*d0).transpose() * Nz.transpose();
    detJ0 = rd0.determinant();

    //////////////////////////////////////////////////////////////
    //// Transformation : Orthogonal transformation (A and J) ////
    //////////////////////////////////////////////////////////////
    ChVector3d G1;
    ChVector3d G2;
    ChVector3d G3;
    ChVector3d G1xG2;
    G1[0] = rd0(0, 0);
    G2[0] = rd0(0, 1);
    G3[0] = rd0(0, 2);
    G1[1] = rd0(1, 0);
    G2[1] = rd0(1, 1);
    G3[1] = rd0(1, 2);
    G1[2] = rd0(2, 0);
    G2[2] = rd0(2, 1);
    G3[2] = rd0(2, 2);
    G1xG2.Cross(G1, G2);

    ////Tangent Frame
    ChVector3d A1 = G1 / std::sqrt(G1[0] * G1[0] + G1[1] * G1[1] + G1[2] * G1[2]);
    ChVector3d A3 = G1xG2 / std::sqrt(G1xG2[0] * G1xG2[0] + G1xG2[1] * G1xG2[1] + G1xG2[2] * G1xG2[2]);
    ChVector3d A2 = A3.Cross(A1);

    ////Direction for orthotropic material//
    double theta = 0.0;
    ChVector3d AA1 = A1 * std::cos(theta) + A2 * std::sin(theta);
    ChVector3d AA2 = -A1 * std::sin(theta) + A2 * std::cos(theta);
    ChVector3d AA3 = A3;

    ////Beta
    ChMatrixNM<double, 3, 3> j0 = rd0.inverse();
    ChVector3d j01;
    ChVector3d j02;
    ChVector3d j03;
    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);
    ChVectorN<double, 9> beta;
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    //////////////////////////////////////////////////
    //// Enhanced Assumed Strain /////////////////////
    //////////////////////////////////////////////////
    G = (*T0) * M * ((*detJ0C) / detJ0);
    strain_EAS = G * (*alpha_eas);

    ChMatrixNM<double, 8, 8> d_d = (*d) * (*d).transpose();
    ChVectorN<double, 8> ddNx = d_d * Nx.transpose();
    ChVectorN<double, 8> ddNy = d_d * Ny.transpose();
    ChVectorN<double, 8> ddNz = d_d * Nz.transpose();

    ChMatrixNM<double, 8, 8> d0_d0 = (*d0) * (*d0).transpose();
    ChVectorN<double, 8> d0d0Nx = d0_d0 * Nx.transpose();
    ChVectorN<double, 8> d0d0Ny = d0_d0 * Ny.transpose();
    ChVectorN<double, 8> d0d0Nz = d0_d0 * Nz.transpose();

    ///////////////////////////
    /// Strain component //////
    ///////////////////////////
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * Nx.dot(ddNx - d0d0Nx);
    strain_til(1) = 0.5 * Ny.dot(ddNy - d0d0Ny);
    strain_til(2) = Nx.dot(ddNy - d0d0Ny);
    //== Compatible strain (No ANS) ==//
    strain_til(3) = 0.5 * Nz.dot(ddNz - d0d0Nz);
    strain_til(4) = Nx.dot(ddNz - d0d0Nz);
    strain_til(5) = Ny.dot(ddNz - d0d0Nz);

    //// For orthotropic material ///
    strain(0) = strain_til(0) * beta(0) * beta(0) + strain_til(1) * beta(3) * beta(3) +
                strain_til(2) * beta(0) * beta(3) + strain_til(3) * beta(6) * beta(6) +
                strain_til(4) * beta(0) * beta(6) + strain_til(5) * beta(3) * beta(6);
    strain(1) = strain_til(0) * beta(1) * beta(1) + strain_til(1) * beta(4) * beta(4) +
                strain_til(2) * beta(1) * beta(4) + strain_til(3) * beta(7) * beta(7) +
                strain_til(4) * beta(1) * beta(7) + strain_til(5) * beta(4) * beta(7);
    strain(2) = strain_til(0) * 2.0 * beta(0) * beta(1) + strain_til(1) * 2.0 * beta(3) * beta(4) +
                strain_til(2) * (beta(1) * beta(3) + beta(0) * beta(4)) + strain_til(3) * 2.0 * beta(6) * beta(7) +
                strain_til(4) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                strain_til(5) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3) = strain_til(0) * beta(2) * beta(2) + strain_til(1) * beta(5) * beta(5) +
                strain_til(2) * beta(2) * beta(5) + strain_til(3) * beta(8) * beta(8) +
                strain_til(4) * beta(2) * beta(8) + strain_til(5) * beta(5) * beta(8);
    strain(4) = strain_til(0) * 2.0 * beta(0) * beta(2) + strain_til(1) * 2.0 * beta(3) * beta(5) +
                strain_til(2) * (beta(2) * beta(3) + beta(0) * beta(5)) + strain_til(3) * 2.0 * beta(6) * beta(8) +
                strain_til(4) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                strain_til(5) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5) = strain_til(0) * 2.0 * beta(1) * beta(2) + strain_til(1) * 2.0 * beta(4) * beta(5) +
                strain_til(2) * (beta(2) * beta(4) + beta(1) * beta(5)) + strain_til(3) * 2.0 * beta(7) * beta(8) +
                strain_til(4) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                strain_til(5) * (beta(5) * beta(7) + beta(4) * beta(8));

    ////////////////////////////////////
    /// Straint derivative component ///
    ////////////////////////////////////
    ChMatrixNM<double, 6, 24> strainD_til;
    strainD_til.row(0) = Nx * (*d) * Sx;
    strainD_til.row(1) = Ny * (*d) * Sy;
    strainD_til.row(2) = Nx * (*d) * Sy + Ny * (*d) * Sx;
    //== Compatible strain (No ANS)==//
    strainD_til.row(3) = Nz * (*d) * Sz;
    strainD_til.row(4) = Nx * (*d) * Sz + Nz * (*d) * Sx;
    strainD_til.row(5) = Ny * (*d) * Sz + Nz * (*d) * Sy;

    //// For orthotropic material ///
    for (int ii = 0; ii < 24; ii++) {
        strainD(0, ii) = strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                         strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                         strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
        strainD(1, ii) = strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                         strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                         strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
        strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) + strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                         strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                         strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                         strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
        strainD(3, ii) = strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
                         strainD_til(2, ii) * beta(2) * beta(5) + strainD_til(3, ii) * beta(8) * beta(8) +
                         strainD_til(4, ii) * beta(2) * beta(8) + strainD_til(5) * beta(5) * beta(8);
        strainD(4, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(2) + strainD_til(1, ii) * 2.0 * beta(3) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(6) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(6) + beta(3) * beta(8));
        strainD(5, ii) = strainD_til(0, ii) * 2.0 * beta(1) * beta(2) + strainD_til(1, ii) * 2.0 * beta(4) * beta(5) +
                         strainD_til(2, ii) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                         strainD_til(3, ii) * 2.0 * beta(7) * beta(8) +
                         strainD_til(4, ii) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                         strainD_til(5, ii) * (beta(5) * beta(7) + beta(4) * beta(8));
    }

    ///////////////////////////////////
    /// Enhanced Assumed Strain 2nd ///
    ///////////////////////////////////
    strain += strain_EAS;  // same as EPS in FORTRAN

    ChMatrixNM<double, 9, 6> temp56;  // same as TEMP1 in FORTRAN
    ChVectorN<double, 9> HE1;
    ChMatrixNM<double, 9, 24> GDEPSP;
    ChMatrixNM<double, 9, 9> KALPHA;

    // m_isMooney == 1 use Iso_Nonlinear_Mooney-Rivlin Material (2-parameters=> 3 inputs)
    if (element->m_isMooney == 1) {
        ChMatrix33<> CG;  // CG: Right Cauchy-Green tensor  C=trans(F)*F
        ChMatrix33<> INVCG;
        ChMatrix33<> I1PC;
        ChMatrix33<> I2PC;
        ChMatrix33<> JPC;
        ChMatrix33<> STR;

        ChMatrix33<> CGN;
        ChMatrix33<> INVCGN;
        ChMatrix33<> I1PCN;
        ChMatrix33<> I2PCN;
        ChMatrix33<> JPCN;
        ChMatrix33<> STRN;

        ChVectorN<double, 6> strain_1;

        CG(0, 0) = 2.0 * strain(0) + 1.0;
        CG(1, 1) = 2.0 * strain(1) + 1.0;
        CG(2, 2) = 2.0 * strain(3) + 1.0;
        CG(1, 0) = strain(2);
        CG(0, 1) = CG(1, 0);
        CG(2, 0) = strain(4);
        CG(0, 2) = CG(2, 0);
        CG(2, 1) = strain(5);
        CG(1, 2) = CG(2, 1);

        INVCG = CG.inverse();

        double Deld = 0.000001;
        double I1 = CG(0, 0) + CG(1, 1) + CG(2, 2);
        double I2 = 0.5 * (std::pow(I1, 2) - (std::pow(CG(0, 0), 2) + std::pow(CG(1, 0), 2) + std::pow(CG(2, 0), 2) +
                                              std::pow(CG(0, 1), 2) + std::pow(CG(1, 1), 2) + std::pow(CG(2, 1), 2) +
                                              std::pow(CG(0, 2), 2) + std::pow(CG(1, 2), 2) + std::pow(CG(2, 2), 2)));
        double I3 = CG(0, 0) * CG(1, 1) * CG(2, 2) - CG(0, 0) * CG(1, 2) * CG(2, 1) + CG(0, 1) * CG(1, 2) * CG(2, 0) -
                    CG(0, 1) * CG(1, 0) * CG(2, 2) + CG(0, 2) * CG(1, 0) * CG(2, 1) - CG(2, 0) * CG(1, 1) * CG(0, 2);
        double cbrtI3inv = 1.0 / std::cbrt(I3);
        double J = std::sqrt(I3);
        // double CCOM1 = 551584.0;                                    // C10   not 0.551584
        // double CCOM2 = 137896.0;                                    // C01   not 0.137896
        double CCOM3 = 2.0 * (element->CCOM1 + element->CCOM2) / (1.0 - 2.0 * 0.49);  // K:bulk modulus
        double StockEPS;

        I1PC = (ChMatrix33<>::Identity() - INVCG * (CH_1_3 * I1)) * cbrtI3inv;
        I2PC = (((ChMatrix33<>::Identity() * I1) - CG) - (INVCG * CH_2_3 * I2)) * cbrtI3inv * cbrtI3inv;
        JPC = INVCG * (J / 2.0);

        STR = I1PC * (element->CCOM1 * 2.0) + I2PC * (element->CCOM2 * 2.0) + JPC * (CCOM3 * (J - 1.0) * 2.0);

        ChVectorN<double, 6> TEMP5;
        TEMP5(0) = STR(0, 0);
        TEMP5(1) = STR(1, 1);
        TEMP5(2) = STR(0, 1);
        TEMP5(3) = STR(2, 2);
        TEMP5(4) = STR(0, 2);
        TEMP5(5) = STR(1, 2);

        E_eps.setZero();

        ChVectorN<double, 6> TEMP5N;
        strain_1 = strain;
        for (int JJJ = 0; JJJ < 6; JJJ++) {
            StockEPS = strain_1(JJJ);
            strain_1(JJJ) = StockEPS + Deld;
            CGN(0, 0) = 2.0 * strain_1(0) + 1.0;
            CGN(1, 1) = 2.0 * strain_1(1) + 1.0;
            CGN(2, 2) = 2.0 * strain_1(3) + 1.0;
            CGN(1, 0) = strain_1(2);
            CGN(0, 1) = CGN(1, 0);
            CGN(2, 0) = strain_1(4);
            CGN(0, 2) = CGN(2, 0);
            CGN(2, 1) = strain_1(5);
            CGN(1, 2) = CGN(2, 1);
            INVCGN = CGN.inverse();
            I1 = CGN(0, 0) + CGN(1, 1) + CGN(2, 2);
            I2 = 0.5 * (std::pow(I1, 2) - (std::pow(CGN(0, 0), 2) + std::pow(CGN(1, 0), 2) + std::pow(CGN(2, 0), 2) +
                                           std::pow(CGN(0, 1), 2) + std::pow(CGN(1, 1), 2) + std::pow(CGN(2, 1), 2) +
                                           std::pow(CGN(0, 2), 2) + std::pow(CGN(1, 2), 2) + std::pow(CGN(2, 2), 2)));
            I3 = CGN(0, 0) * CGN(1, 1) * CGN(2, 2) - CGN(0, 0) * CGN(1, 2) * CGN(2, 1) +
                 CGN(0, 1) * CGN(1, 2) * CGN(2, 0) - CGN(0, 1) * CGN(1, 0) * CGN(2, 2) +
                 CGN(0, 2) * CGN(1, 0) * CGN(2, 1) - CGN(2, 0) * CGN(1, 1) * CGN(0, 2);
            cbrtI3inv = 1.0 / std::cbrt(I3);
            J = std::sqrt(I3);
            I1PCN = (ChMatrix33<>::Identity() - INVCGN * (CH_1_3 * I1)) * cbrtI3inv;
            I2PCN = (((ChMatrix33<>::Identity() * I1) - CGN) - (INVCGN * CH_2_3 * I2)) * cbrtI3inv * cbrtI3inv;
            JPCN = INVCGN * (J / 2.0);
            STRN = I1PCN * (element->CCOM1 * 2.0) + I2PCN * (element->CCOM2 * 2.0) + JPCN * (CCOM3 * (J - 1.0) * 2.0);
            TEMP5N(0) = STRN(0, 0);
            TEMP5N(1) = STRN(1, 1);
            TEMP5N(2) = STRN(0, 1);
            TEMP5N(3) = STRN(2, 2);
            TEMP5N(4) = STRN(0, 2);
            TEMP5N(5) = STRN(1, 2);
            strain_1(JJJ) = StockEPS;
            E_eps(JJJ, 0) = (TEMP5N(0) - TEMP5(0)) / Deld;
            E_eps(JJJ, 1) = (TEMP5N(1) - TEMP5(1)) / Deld;
            E_eps(JJJ, 2) = (TEMP5N(2) - TEMP5(2)) / Deld;
            E_eps(JJJ, 3) = (TEMP5N(3) - TEMP5(3)) / Deld;
            E_eps(JJJ, 4) = (TEMP5N(4) - TEMP5(4)) / Deld;
            E_eps(JJJ, 5) = (TEMP5N(5) - TEMP5(5)) / Deld;
        }
        temp56 = G.transpose() * E_eps;
        Fint = strainD.transpose() * TEMP5;
        Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        HE1 = G.transpose() * TEMP5;
        HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    } else {
        temp56 = G.transpose() * E_eps;
        Fint = strainD.transpose() * E_eps * strain;
        Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        HE1 = temp56 * strain;
        HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    }  // end of   if(*flag_Mooney==1)

    double factor_k =
        detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    KALPHA = factor_k * temp56 * G;
    double factor_g =
        detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    GDEPSP = factor_g * temp56 * strainD;

    ChVectorN<double, 216> GDEPSPVec = Eigen::Map<ChVectorN<double, 216>>(GDEPSP.data(), 216);
    ChVectorN<double, 81> KALPHAVec = Eigen::Map<ChVectorN<double, 81>>(KALPHA.data(), 81);

    result.segment(0, 24) = Fint;
    result.segment(24, 9) = HE1;
    result.segment(33, 216) = GDEPSPVec;
    result.segment(249, 81) = KALPHAVec;
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    int ie = GetElemNum();

    ChVector3d pA = m_nodes[0]->GetPos();
    ChVector3d pB = m_nodes[1]->GetPos();
    ChVector3d pC = m_nodes[2]->GetPos();
    ChVector3d pD = m_nodes[3]->GetPos();
    ChVector3d pE = m_nodes[4]->GetPos();
    ChVector3d pF = m_nodes[5]->GetPos();
    ChVector3d pG = m_nodes[6]->GetPos();
    ChVector3d pH = m_nodes[7]->GetPos();

    ChMatrixNM<double, 8, 3> d;
    d(0, 0) = pA.x();
    d(0, 1) = pA.y();
    d(0, 2) = pA.z();
    d(1, 0) = pB.x();
    d(1, 1) = pB.y();
    d(1, 2) = pB.z();
    d(2, 0) = pC.x();
    d(2, 1) = pC.y();
    d(2, 2) = pC.z();
    d(3, 0) = pD.x();
    d(3, 1) = pD.y();
    d(3, 2) = pD.z();
    d(4, 0) = pE.x();
    d(4, 1) = pE.y();
    d(4, 2) = pE.z();
    d(5, 0) = pF.x();
    d(5, 1) = pF.y();
    d(5, 2) = pF.z();
    d(6, 0) = pG.x();
    d(6, 1) = pG.y();
    d(6, 2) = pG.z();
    d(7, 0) = pH.x();
    d(7, 1) = pH.y();
    d(7, 2) = pH.z();

    double v = m_Material->GetPoissonRatio();
    double E = m_Material->GetYoungModulus();

    Fi.setZero();

    /// If numerical differentiation is used, only the internal force and EAS stiffness
    /// will be calculated. If the numerical differentiation is not used, the jacobian
    /// will also be calculated.
    bool use_numerical_differentiation = false;

    /// Internal force and EAS parameters are calculated for numerical differentiation.
    if (use_numerical_differentiation) {
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        ChVectorN<double, 330> TempIntegratedResult;
        ChVectorN<double, 24> Finternal;

        ChMatrixNM<double, 6, 6> T0;
        ChVectorN<double, 9> HE;
        ChMatrixNM<double, 9, 24> GDEPSP;
        ChMatrixNM<double, 9, 9> KALPHA;
        ChMatrixNM<double, 9, 9> KALPHA1;
        ChVectorN<double, 9> ResidHE;
        double detJ0C;
        ChVectorN<double, 9> alpha_eas;
        ChVectorN<double, 9> renewed_alpha_eas;
        ChVectorN<double, 9> previous_alpha;

        previous_alpha = m_stock_alpha_EAS;
        alpha_eas = previous_alpha;
        ResidHE.setZero();
        int count = 0;
        int fail = 1;
        /// Begin EAS loop
        while (fail == 1) {
            /// Update alpha EAS
            alpha_eas = alpha_eas - ResidHE;
            renewed_alpha_eas = alpha_eas;

            Finternal.setZero();
            HE.setZero();
            GDEPSP.setZero();
            KALPHA.setZero();

            // Enhanced Assumed Strain (EAS)
            T0.setZero();
            detJ0C = 0.0;
            T0DetJElementCenterForEAS(m_d0, T0, detJ0C);
            //== F_internal ==//
            // Choose constructors depending on m_isMooney
            Brick_ForceNumerical myformula =
                !m_isMooney ? Brick_ForceNumerical(&d, &m_d0, this, &T0, &detJ0C, &alpha_eas, &E, &v)
                            : Brick_ForceNumerical(&d, &m_d0, this, &T0, &detJ0C, &alpha_eas);
            TempIntegratedResult.setZero();
            ChQuadrature::Integrate3D<ChVectorN<double, 330>>(
                TempIntegratedResult,  // result of integration will go there
                myformula,             // formula to integrate
                -1,                    // start of x
                1,                     // end of x
                -1,                    // start of y
                1,                     // end of y
                -1,                    // start of z
                1,                     // end of z
                2                      // order of integration
            );

            ///===============================================================//
            ///===TempIntegratedResult(0:23,1) -> InternalForce(24x1)=========//
            ///===TempIntegratedResult(24:32,1) -> HE(9x1)   brick   =========//
            ///===TempIntegratedResult(33:248,1) -> GDEPSP(9x24)     =========//
            ///===TempIntegratedResult(249:329,1) -> KALPHA(9x9)     =========//
            ///===============================================================//
            ChVectorN<double, 216> GDEPSPvec;
            ChVectorN<double, 81> KALPHAvec;
            Finternal = TempIntegratedResult.segment(0, 24);
            HE = TempIntegratedResult.segment(24, 9);
            GDEPSPvec = TempIntegratedResult.segment(33, 216);
            KALPHAvec = TempIntegratedResult.segment(249, 81);
            GDEPSP = Eigen::Map<ChMatrixNM<double, 9, 24>>(GDEPSPvec.data(), 9, 24);
            KALPHA = Eigen::Map<ChMatrixNM<double, 9, 9>>(KALPHAvec.data(), 9, 9);
            KALPHA1 = KALPHA;

            if (m_flag_HE == NUMERICAL)
                break;  // When numerical jacobian loop, no need to calculate HE
            count = count + 1;
            double norm_HE = HE.norm();

            if (norm_HE < 0.00001) {
                fail = 0;
            } else {
                // Solve for ResidHE
                ResidHE = KALPHA1.colPivHouseholderQr().solve(HE);
            }
            if (m_flag_HE == ANALYTICAL && count > 2) {
                std::cerr << ie << "  count " << count << "  NormHE " << norm_HE << std::endl;
            }
        }
        Fi = -Finternal;
        //== Stock_Alpha=================//
        if (m_flag_HE == ANALYTICAL) {
            SetStockAlpha(renewed_alpha_eas(0), renewed_alpha_eas(1), renewed_alpha_eas(2), renewed_alpha_eas(3),
                          renewed_alpha_eas(4), renewed_alpha_eas(5), renewed_alpha_eas(6), renewed_alpha_eas(7),
                          renewed_alpha_eas(8));  // this->
        }
        //== Jacobian Matrix for alpha ==//
        if (m_flag_HE == ANALYTICAL) {
            ChMatrixNM<double, 9, 9> INV_KALPHA;
            ChMatrixNM<double, 24, 24> stock_jac_EAS_elem;

            for (int ii = 0; ii < 9; ii++) {
                ChVectorN<double, 9> DAMMY_vec;
                DAMMY_vec.setZero();
                DAMMY_vec(ii) = 1.0;
                INV_KALPHA.col(ii) = KALPHA.colPivHouseholderQr().solve(DAMMY_vec);
            }
            stock_jac_EAS_elem = GDEPSP.transpose() * INV_KALPHA * GDEPSP;
            m_stock_jac_EAS = stock_jac_EAS_elem;
        }
    } else {
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        ChVectorN<double, 906> TempIntegratedResult;
        ChVectorN<double, 24> Finternal;
        // Enhanced Assumed Strain (EAS)
        ChMatrixNM<double, 6, 6> T0;
        ChVectorN<double, 9> HE;
        ChMatrixNM<double, 9, 24> GDEPSP;
        ChMatrixNM<double, 9, 9> KALPHA;
        ChMatrixNM<double, 24, 24> KTE;
        ChMatrixNM<double, 9, 9> KALPHA1;
        ChVectorN<double, 9> ResidHE;
        double detJ0C;
        ChVectorN<double, 9> alpha_eas;
        ChVectorN<double, 9> renewed_alpha_eas;
        ChVectorN<double, 9> previous_alpha;

        previous_alpha = m_stock_alpha_EAS;
        alpha_eas = previous_alpha;
        ResidHE.setZero();
        int count = 0;
        int fail = 1;
        // Loop to obtain convergence in EAS internal parameters alpha
        // This loops call ChQuadrature::Integrate3D on MyAnalyticalForce,
        // which calculates the Jacobian at every iteration of each time step
        int iteralpha = 0;  //  Counts number of iterations
        while (fail == 1) {
            iteralpha++;
            alpha_eas = alpha_eas - ResidHE;
            renewed_alpha_eas = alpha_eas;

            Finternal.setZero();  // Internal force vector
            HE.setZero();         // Internal force vector from EAS
            GDEPSP.setZero();     // Jacobian of EAS forces w.r.t. coordinates
            KALPHA.setZero();     // Jacobian of EAS forces w.r.t. EAS internal parameters

            // Enhanced Assumed Strain (EAS)
            T0.setZero();
            detJ0C = 0.0;
            T0DetJElementCenterForEAS(m_d0, T0, detJ0C);

            //== F_internal ==//
            Brick_ForceAnalytical myformula =
                !m_isMooney ? Brick_ForceAnalytical(&d, &m_d0, this, &T0, &detJ0C, &alpha_eas, &E, &v)
                            : Brick_ForceAnalytical(&d, &m_d0, this, &T0, &detJ0C, &alpha_eas);
            TempIntegratedResult.setZero();
            ChQuadrature::Integrate3D<ChVectorN<double, 906>>(
                TempIntegratedResult,  // result of integration will go there
                myformula,             // formula to integrate
                -1,                    // start of x
                1,                     // end of x
                -1,                    // start of y
                1,                     // end of y
                -1,                    // start of z
                1,                     // end of z
                2                      // order of integration
            );

            //	///===============================================================//
            //	///===TempIntegratedResult(0:23,1) -> InternalForce(24x1)=========//
            //	///===TempIntegratedResult(24:28,1) -> HE(5x1)           =========//
            //	///===TempIntegratedResult(29:148,1) -> GDEPSP(5x24)     =========//
            //	///===TempIntegratedResult(149:173,1) -> KALPHA(5x5)     =========//
            //	///===TempIntegratedResult(174:749,1) -> Stiffness Matrix(24x24) =//
            //	///===============================================================//
            ChVectorN<double, 216> GDEPSPvec;
            ChVectorN<double, 81> KALPHAvec;
            ChVectorN<double, 576> JACvec;
            Finternal = TempIntegratedResult.segment(0, 24);
            HE = TempIntegratedResult.segment(24, 9);
            GDEPSPvec = TempIntegratedResult.segment(33, 216);
            KALPHAvec = TempIntegratedResult.segment(249, 81);
            JACvec = TempIntegratedResult.segment(330, 576);
            for (int i = 0; i < 9; i++) {
                for (int j = 0; j < 24; j++) {
                    GDEPSP(i, j) = GDEPSPvec(i * 24 + j);
                }
            }
            // GDEPSP = GDEPSPvec;
            for (int i = 0; i < 9; i++) {
                for (int j = 0; j < 9; j++) {
                    KALPHA(i, j) = KALPHAvec(i * 9 + j);
                }
            }
            // KALPHA = KALPHAvec;
            for (int i = 0; i < 24; i++) {
                for (int j = 0; j < 24; j++) {
                    KTE(i, j) = JACvec(i * 24 + j);
                }
            }
            // KTE = JACvec;

            // Calculation of the element Jacobian for implicit integrator
            // KTE and stock_jac_EAS_elem.
            KALPHA1 = KALPHA;
            if (m_flag_HE == NUMERICAL)
                break;  // When numerical jacobian loop, no need to calculate HE
            count = count + 1;
            double norm_HE = HE.norm();
            if (norm_HE < 0.00001) {
                fail = 0;
            } else {
                // Solve for ResidHE
                ResidHE = KALPHA1.colPivHouseholderQr().solve(HE);
            }
        }  // end of while
        Fi = -Finternal;
        ////== Stock_Alpha=================//
        if (m_flag_HE == ANALYTICAL) {
            SetStockAlpha(renewed_alpha_eas(0), renewed_alpha_eas(1), renewed_alpha_eas(2), renewed_alpha_eas(3),
                          renewed_alpha_eas(4), renewed_alpha_eas(5), renewed_alpha_eas(6), renewed_alpha_eas(7),
                          renewed_alpha_eas(8));  // this->
        }
        ////== Jacobian Matrix for alpha ==//
        if (m_flag_HE == ANALYTICAL) {
            ChMatrixNM<double, 9, 9> INV_KALPHA;
            ChMatrixNM<double, 24, 24> stock_jac_EAS_elem;

            for (int ii = 0; ii < 9; ii++) {
                ChVectorN<double, 9> DAMMY_vec;
                DAMMY_vec.setZero();
                DAMMY_vec(ii) = 1.0;
                INV_KALPHA.col(ii) = KALPHA.colPivHouseholderQr().solve(DAMMY_vec);
            }
            stock_jac_EAS_elem = GDEPSP.transpose() * INV_KALPHA * GDEPSP;
            m_stock_KTE = KTE;
            m_stock_jac_EAS = stock_jac_EAS_elem;
        }
    }  // end of else for numerical or analytical
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::ShapeFunctions(ShapeVector& N, double x, double y, double z) {
    N(0) = 0.125 * (1.0 - x) * (1.0 - y) * (1.0 - z);
    N(1) = 0.125 * (1.0 + x) * (1.0 - y) * (1.0 - z);
    N(2) = 0.125 * (1.0 + x) * (1.0 + y) * (1.0 - z);
    N(3) = 0.125 * (1.0 - x) * (1.0 + y) * (1.0 - z);
    N(4) = 0.125 * (1.0 - x) * (1.0 - y) * (1.0 + z);
    N(5) = 0.125 * (1.0 + x) * (1.0 - y) * (1.0 + z);
    N(6) = 0.125 * (1.0 + x) * (1.0 + y) * (1.0 + z);
    N(7) = 0.125 * (1.0 - x) * (1.0 + y) * (1.0 + z);
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::ShapeFunctionsDerivativeX(ShapeVector& Nx, double x, double y, double z) {
    double a = GetLengthX();

    Nx(0) = 2.0 / a * 0.125 * (-1.0) * (1.0 - y) * (1.0 - z);
    Nx(1) = 2.0 / a * 0.125 * (1.0) * (1.0 - y) * (1.0 - z);
    Nx(2) = 2.0 / a * 0.125 * (1.0) * (1.0 + y) * (1.0 - z);
    Nx(3) = 2.0 / a * 0.125 * (-1.0) * (1.0 + y) * (1.0 - z);
    Nx(4) = 2.0 / a * 0.125 * (-1.0) * (1.0 - y) * (1.0 + z);
    Nx(5) = 2.0 / a * 0.125 * (1.0) * (1.0 - y) * (1.0 + z);
    Nx(6) = 2.0 / a * 0.125 * (1.0) * (1.0 + y) * (1.0 + z);
    Nx(7) = 2.0 / a * 0.125 * (-1.0) * (1.0 + y) * (1.0 + z);
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::ShapeFunctionsDerivativeY(ShapeVector& Ny, double x, double y, double z) {
    double b = GetLengthY();

    Ny(0) = 2.0 / b * 0.125 * (1.0 - x) * (-1.0) * (1.0 - z);
    Ny(1) = 2.0 / b * 0.125 * (1.0 + x) * (-1.0) * (1.0 - z);
    Ny(2) = 2.0 / b * 0.125 * (1.0 + x) * (1.0) * (1.0 - z);
    Ny(3) = 2.0 / b * 0.125 * (1.0 - x) * (1.0) * (1.0 - z);
    Ny(4) = 2.0 / b * 0.125 * (1.0 - x) * (-1.0) * (1.0 + z);
    Ny(5) = 2.0 / b * 0.125 * (1.0 + x) * (-1.0) * (1.0 + z);
    Ny(6) = 2.0 / b * 0.125 * (1.0 + x) * (1.0) * (1.0 + z);
    Ny(7) = 2.0 / b * 0.125 * (1.0 - x) * (1.0) * (1.0 + z);
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::ShapeFunctionsDerivativeZ(ShapeVector& Nz, double x, double y, double z) {
    double c = GetLengthZ();

    Nz(0) = 2.0 / c * 0.125 * (1.0 - x) * (1.0 - y) * (-1.0);
    Nz(1) = 2.0 / c * 0.125 * (1.0 + x) * (1.0 - y) * (-1.0);
    Nz(2) = 2.0 / c * 0.125 * (1.0 + x) * (1.0 + y) * (-1.0);
    Nz(3) = 2.0 / c * 0.125 * (1.0 - x) * (1.0 + y) * (-1.0);
    Nz(4) = 2.0 / c * 0.125 * (1.0 - x) * (1.0 - y) * (1.0);
    Nz(5) = 2.0 / c * 0.125 * (1.0 + x) * (1.0 - y) * (1.0);
    Nz(6) = 2.0 / c * 0.125 * (1.0 + x) * (1.0 + y) * (1.0);
    Nz(7) = 2.0 / c * 0.125 * (1.0 - x) * (1.0 + y) * (1.0);
}

// ----------------------------------------------------------------------------

void ChElementHexaANCF_3813::Update() {
    // parent class update:
    ChElementGeneric::Update();
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(6, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(9, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(12, 3) = m_nodes[4]->GetPos().eigen();
    mD.segment(15, 3) = m_nodes[5]->GetPos().eigen();
    mD.segment(18, 3) = m_nodes[6]->GetPos().eigen();
    mD.segment(21, 3) = m_nodes[7]->GetPos().eigen();
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::ComputeStiffnessMatrix() {
    bool use_numerical_differentiation = false;

    if (use_numerical_differentiation) {
        double diff = 1e-8;
        ChVectorDynamic<> F0(24);
        ChVectorDynamic<> F1(24);
        ComputeInternalForces(F0);
        for (int inode = 0; inode < 8; ++inode) {
            m_nodes[inode]->pos.x() += diff;
            ComputeInternalForces(F1);  // Flag=1 > Jacobian of internal force calculation
            m_StiffnessMatrix.col(0 + inode * 3) = (F0 - F1) * (1.0 / diff);
            m_nodes[inode]->pos.x() -= diff;

            m_nodes[inode]->pos.y() += diff;
            ComputeInternalForces(F1);
            m_StiffnessMatrix.col(1 + inode * 3) = (F0 - F1) * (1.0 / diff);
            m_nodes[inode]->pos.y() -= diff;

            m_nodes[inode]->pos.z() += diff;
            ComputeInternalForces(F1);
            m_StiffnessMatrix.col(2 + inode * 3) = (F0 - F1) * (1.0 / diff);
            m_nodes[inode]->pos.z() -= diff;
        }
        // flag_HE=0 is default
        m_StiffnessMatrix -= m_stock_jac_EAS;  // For Enhanced Assumed Strain
    } else {
        // Put in m_StiffnessMatrix the values for the Jacobian already calculated in the computation of internal forces
        // Note that m_stock_KTE and m_stock_jac_EAS are updated at each iteration of the time step
        m_StiffnessMatrix = m_stock_KTE;
        m_StiffnessMatrix -= m_stock_jac_EAS;
    }
}

// -----------------------------------------------------------------------------

class Brick_Mass : public ChIntegrand3D<ChMatrixNM<double, 24, 24>> {
  public:
    Brick_Mass(ChMatrixNM<double, 8, 3>* d0_, ChElementHexaANCF_3813* element_);
    ~Brick_Mass() {}

  private:
    ChElementHexaANCF_3813* element;
    ChMatrixNM<double, 8, 3>* d0;            ///< Pointer to a matrix containing the element initial coordinates
    ChMatrixNM<double, 3, 24> S;             ///< Sparse shape function matrix
    ChElementHexaANCF_3813::ShapeVector N;   ///< Dense shape function vector
    ChElementHexaANCF_3813::ShapeVector Nx;  ///< Dense shape function vector, X derivative
    ChElementHexaANCF_3813::ShapeVector Ny;  ///< Dense shape function vector, Y derivative
    ChElementHexaANCF_3813::ShapeVector Nz;  ///< Dense shape function vector, Z derivative

    /// Evaluate the S'*S  at point x
    virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z) override;
};

Brick_Mass::Brick_Mass(ChMatrixNM<double, 8, 3>* d0_, ChElementHexaANCF_3813* element_) : element(element_), d0(d0_) {
    S.setZero();
}

void Brick_Mass::Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z) {
    element->ShapeFunctions(N, x, y, z);
    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

    // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)...]
    for (int i = 0; i < 8; i++) {
        S(0, 3 * i + 0) = N(i);
        S(1, 3 * i + 1) = N(i);
        S(2, 3 * i + 2) = N(i);
    }

    ChMatrixNM<double, 3, 3> rd0;
    rd0.col(0) = (*d0).transpose() * Nx.transpose();
    rd0.col(1) = (*d0).transpose() * Ny.transpose();
    rd0.col(2) = (*d0).transpose() * Nz.transpose();
    double detJ0 = rd0.determinant();

    // Perform  r = S'*S
    double factor = detJ0 * (element->GetLengthX() / 2) * (element->GetLengthY() / 2) * (element->GetLengthZ() / 2);
    result = factor * S.transpose() * S;
}

void ChElementHexaANCF_3813::ComputeMassMatrix() {
    double rho = m_Material->GetDensity();
    Brick_Mass myformula(&m_d0, this);
    m_MassMatrix.setZero();
    ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 24>>(m_MassMatrix,  // result of integration will go there
                                                          myformula,     // formula to integrate
                                                          -1,            // start of x
                                                          1,             // end of x
                                                          -1,            // start of y
                                                          1,             // end of y
                                                          -1,            // start of z
                                                          1,             // end of z
                                                          2              // order of integration
    );

    m_MassMatrix *= rho;
}

// -----------------------------------------------------------------------------

// Class to calculate the gravity forces of a brick element
class BrickGravity : public ChIntegrand3D<ChVectorN<double, 8>> {
  public:
    BrickGravity(ChMatrixNM<double, 8, 3>* d0_, ChElementHexaANCF_3813* element_);
    ~BrickGravity() {}

  private:
    ChElementHexaANCF_3813* element;
    ChMatrixNM<double, 8, 3>* d0;            // Pointer to a matrix containing the element initial coordinates
    ChMatrixNM<double, 3, 24> S;             // Sparse shape function matrix
    ChElementHexaANCF_3813::ShapeVector N;   // Dense shape function vector
    ChElementHexaANCF_3813::ShapeVector Nx;  // Dense shape function vector, X derivative
    ChElementHexaANCF_3813::ShapeVector Ny;  // Dense shape function vector, Y derivative
    ChElementHexaANCF_3813::ShapeVector Nz;  // Dense shape function vector, Z derivative

    virtual void Evaluate(ChVectorN<double, 8>& result, const double x, const double y, const double z) override;
};

BrickGravity::BrickGravity(ChMatrixNM<double, 8, 3>* d0_, ChElementHexaANCF_3813* element_)
    : element(element_), d0(d0_) {}

void BrickGravity::Evaluate(ChVectorN<double, 8>& result, const double x, const double y, const double z) {
    element->ShapeFunctions(N, x, y, z);
    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

    // Weights for Gaussian integration
    double wx2 = (element->GetLengthX()) / 2;
    double wy2 = (element->GetLengthY()) / 2;
    double wz2 = (element->GetLengthZ()) / 2;

    ChMatrixNM<double, 3, 3> rd0;
    rd0.col(0) = (*d0).transpose() * Nx.transpose();
    rd0.col(1) = (*d0).transpose() * Ny.transpose();
    rd0.col(2) = (*d0).transpose() * Nz.transpose();
    double detJ0 = rd0.determinant();

    result = detJ0 * wx2 * wy2 * wz2 * N.transpose();
}

void ChElementHexaANCF_3813::ComputeGravityForceScale() {
    BrickGravity myformula1(&m_d0, this);
    m_GravForceScale.setZero();
    ChQuadrature::Integrate3D<ChVectorN<double, 8>>(m_GravForceScale,  // result of integration will go there
                                                    myformula1,        // formula to integrate
                                                    -1, 1,             // limits in x direction
                                                    -1, 1,             // limits in y direction
                                                    -1, 1,             // limits in z direction
                                                    2                  // order of integration
    );

    m_GravForceScale *= m_Material->GetDensity();
}

// Compute the generalized force vector due to gravity
void ChElementHexaANCF_3813::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector3d& G_acc) {
    assert(Fg.size() == 24);

    // Calculate and add the generalized force due to gravity to the generalized internal force vector for the element.
    // The generalized force due to gravity could be computed once prior to the start of the simulation if gravity was
    // assumed constant throughout the entire simulation.  However, this implementation assumes that the acceleration
    // due to gravity, while a constant for the entire system, can change from step to step which could be useful for
    // gravity loaded units tests as an example.  The generalized force due to gravity is calculated in compact matrix
    // form and is pre-mapped to the desired vector format
    Eigen::Map<ChMatrixNM<double, 8, 3>> GravForceCompact(Fg.data(), 8, 3);
    GravForceCompact = m_GravForceScale * G_acc.eigen().transpose();
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::SetupInitial(ChSystem* system) {
    // Compute gravitational forces
    ComputeGravityForceScale();
    // Compute mass matrix
    ComputeMassMatrix();
    // initial EAS parameters
    m_stock_jac_EAS.setZero();
    // Compute stiffness matrix
    // (this is not constant in ANCF and will be called automatically many times by ComputeKRMmatricesGlobal()
    // when the solver will run, yet maybe nice to privide an initial nonzero value)
    ComputeStiffnessMatrix();
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 24) && (H.cols() == 24));

    // Compute global stiffness matrix:
    ComputeStiffnessMatrix();

    // For K stiffness matrix and R matrix: scale by factors
    // because [R] = r*[K] , so kf*[K]+rf*[R] = (kf+rf*r)*[K]
    double kr_factor = Kfactor + Rfactor * m_Material->GetRayleighDampingBeta();

    // Paste scaled K stiffness matrix and R matrix in resulting H and add scaled mass matrix.
    H.block(0, 0, 24, 24) = kr_factor * m_StiffnessMatrix + Mfactor * m_MassMatrix;
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::T0DetJElementCenterForEAS(ChMatrixNM<double, 8, 3>& d0,
                                                       ChMatrixNM<double, 6, 6>& T0,
                                                       double& detJ0C) {
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ShapeFunctionsDerivativeX(Nx, 0, 0, 0);
    ShapeFunctionsDerivativeY(Ny, 0, 0, 0);
    ShapeFunctionsDerivativeZ(Nz, 0, 0, 0);

    ChMatrixNM<double, 3, 3> rd0;
    rd0.col(0) = d0.transpose() * Nx.transpose();
    rd0.col(1) = d0.transpose() * Ny.transpose();
    rd0.col(2) = d0.transpose() * Nz.transpose();
    detJ0C = rd0.determinant();

    // Transformation : Orthogonal transformation (A and J) ////
    ChVector3d G1;
    ChVector3d G2;
    ChVector3d G3;
    ChVector3d G1xG2;
    G1[0] = rd0(0, 0);
    G2[0] = rd0(0, 1);
    G3[0] = rd0(0, 2);
    G1[1] = rd0(1, 0);
    G2[1] = rd0(1, 1);
    G3[1] = rd0(1, 2);
    G1[2] = rd0(2, 0);
    G2[2] = rd0(2, 1);
    G3[2] = rd0(2, 2);
    G1xG2.Cross(G1, G2);

    // Tangent Frame
    ChVector3d A1 = G1 / std::sqrt(G1[0] * G1[0] + G1[1] * G1[1] + G1[2] * G1[2]);
    ChVector3d A3 = G1xG2 / std::sqrt(G1xG2[0] * G1xG2[0] + G1xG2[1] * G1xG2[1] + G1xG2[2] * G1xG2[2]);
    ChVector3d A2 = A3.Cross(A1);
    double theta = 0.0;
    ChVector3d AA1 = A1 * std::cos(theta) + A2 * std::sin(theta);
    ChVector3d AA2 = -A1 * std::sin(theta) + A2 * std::cos(theta);
    ChVector3d AA3 = A3;

    // Beta
    ChMatrixNM<double, 3, 3> j0 = rd0.inverse();
    ChVector3d j01;
    ChVector3d j02;
    ChVector3d j03;
    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);
    ChVectorN<double, 9> beta;
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    T0(0, 0) = std::pow(beta(0), 2);
    T0(1, 0) = std::pow(beta(1), 2);
    T0(2, 0) = 2.0 * beta(0) * beta(1);
    T0(3, 0) = std::pow(beta(2), 2);
    T0(4, 0) = 2.0 * beta(0) * beta(2);
    T0(5, 0) = 2.0 * beta(1) * beta(2);

    T0(0, 1) = std::pow(beta(3), 2);
    T0(1, 1) = std::pow(beta(4), 2);
    T0(2, 1) = 2.0 * beta(3) * beta(4);
    T0(3, 1) = std::pow(beta(5), 2);
    T0(4, 1) = 2.0 * beta(3) * beta(5);
    T0(5, 1) = 2.0 * beta(4) * beta(5);

    T0(0, 2) = beta(0) * beta(3);
    T0(1, 2) = beta(1) * beta(4);
    T0(2, 2) = beta(0) * beta(4) + beta(1) * beta(3);
    T0(3, 2) = beta(2) * beta(5);
    T0(4, 2) = beta(0) * beta(5) + beta(2) * beta(3);
    T0(5, 2) = beta(2) * beta(4) + beta(1) * beta(5);

    T0(0, 3) = std::pow(beta(6), 2);
    T0(1, 3) = std::pow(beta(7), 2);
    T0(2, 3) = 2.0 * beta(6) * beta(7);
    T0(3, 3) = std::pow(beta(8), 2);
    T0(4, 3) = 2.0 * beta(6) * beta(8);
    T0(5, 3) = 2.0 * beta(7) * beta(8);

    T0(0, 4) = beta(0) * beta(6);
    T0(1, 4) = beta(1) * beta(7);
    T0(2, 4) = beta(0) * beta(7) + beta(6) * beta(1);
    T0(3, 4) = beta(2) * beta(8);
    T0(4, 4) = beta(0) * beta(8) + beta(2) * beta(6);
    T0(5, 4) = beta(1) * beta(8) + beta(2) * beta(7);

    T0(0, 5) = beta(3) * beta(6);
    T0(1, 5) = beta(4) * beta(7);
    T0(2, 5) = beta(3) * beta(7) + beta(4) * beta(6);
    T0(3, 5) = beta(5) * beta(8);
    T0(4, 5) = beta(3) * beta(8) + beta(6) * beta(5);
    T0(5, 5) = beta(4) * beta(8) + beta(5) * beta(7);
}

void ChElementHexaANCF_3813::Basis_M(ChMatrixNM<double, 6, 9>& M, double x, double y, double z) {
    M.setZero();
    M(0, 0) = x;
    M(1, 1) = y;
    M(2, 2) = x;
    M(2, 3) = y;
    M(3, 4) = z;
    M(4, 5) = x;
    M(4, 6) = z;
    M(5, 7) = y;
    M(5, 8) = z;
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::LoadableGetStateBlockPosLevel(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(block_offset + 12, 3) = m_nodes[4]->GetPos().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[5]->GetPos().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[6]->GetPos().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[7]->GetPos().eigen();
}

void ChElementHexaANCF_3813::LoadableGetStateBlockVelLevel(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPosDt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[1]->GetPosDt().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[2]->GetPosDt().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[3]->GetPosDt().eigen();
    mD.segment(block_offset + 12, 3) = m_nodes[4]->GetPosDt().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[5]->GetPosDt().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[6]->GetPosDt().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[7]->GetPosDt().eigen();
}

void ChElementHexaANCF_3813::LoadableStateIncrement(const unsigned int off_x,
                                                    ChState& x_new,
                                                    const ChState& x,
                                                    const unsigned int off_v,
                                                    const ChStateDelta& Dv) {
    for (int i = 0; i < 8; ++i) {
        this->m_nodes[i]->NodeIntStateIncrement(off_x + 3 * 1, x_new, x, off_v + 3 * i, Dv);
    }
}

void ChElementHexaANCF_3813::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i)
        mvars.push_back(&this->m_nodes[i]->Variables());
}

// -----------------------------------------------------------------------------

void ChElementHexaANCF_3813::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    // this->ComputeNF(U, V, Qi, detJ, F, state_x, state_w);
    ShapeVector N;
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ShapeFunctions(N, U, V, W);  // evaluate shape functions (in compressed vector)
    ShapeFunctionsDerivativeX(Nx, U, V, W);
    ShapeFunctionsDerivativeY(Ny, U, V, W);
    ShapeFunctionsDerivativeZ(Nz, U, V, W);

    ChMatrixNM<double, 3, 3> rd0;
    rd0.col(0) = m_d0.transpose() * Nx.transpose();
    rd0.col(1) = m_d0.transpose() * Ny.transpose();
    rd0.col(2) = m_d0.transpose() * Nz.transpose();
    detJ = rd0.determinant();
    detJ *= this->GetLengthX() * this->GetLengthY() * this->GetLengthZ() / 8.0;

    for (int i = 0; i < 8; i++) {
        Qi.segment(3 * i, 3) = N(i) * F.segment(0, 3);
    }
}

}  // end namespace fea
}  // end namespace chrono
