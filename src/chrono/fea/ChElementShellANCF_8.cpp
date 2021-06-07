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
// Authors: Antonio Recuero, Radu Serban
// =============================================================================
// ANCF laminated shell element with eight nodes: High-Order.
// Element 3833 of paper: 'Analysis of higher-order quadrilateral plate elements
// based on the absolute nodal coordinate formulation for three-dimensional
// elasticity'
// H.C.J. Ebel, M.K.Matikainen, V.V.T. Hurskainen, A.M.Mikkola, Multibody System
// Dynamics, To be published, 2017
// =============================================================================

//// RADU
//// A lot more to do here...
//// - reconsider the use of large static matrices
//// - more use of Eigen expressions
//// - remove unecessary initializations to zero

#include <cmath>

#include "chrono/fea/ChElementShellANCF_8.h"
#include "chrono/core/ChException.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace fea {

// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementShellANCF_8::ChElementShellANCF_8()
    : m_numLayers(0), m_lenX(0), m_lenY(0), m_thickness(0), m_Alpha(0), m_gravity_on(false) {
    m_nodes.resize(8);
}

// ------------------------------------------------------------------------------
// Set element nodes
// ------------------------------------------------------------------------------

void ChElementShellANCF_8::SetNodes(std::shared_ptr<ChNodeFEAxyzDD> nodeA,
                                    std::shared_ptr<ChNodeFEAxyzDD> nodeB,
                                    std::shared_ptr<ChNodeFEAxyzDD> nodeC,
                                    std::shared_ptr<ChNodeFEAxyzDD> nodeD,
                                    std::shared_ptr<ChNodeFEAxyzDD> nodeE,
                                    std::shared_ptr<ChNodeFEAxyzDD> nodeF,
                                    std::shared_ptr<ChNodeFEAxyzDD> nodeG,
                                    std::shared_ptr<ChNodeFEAxyzDD> nodeH) {
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
    mvars.push_back(&m_nodes[0]->Variables_D());
    mvars.push_back(&m_nodes[0]->Variables_DD());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[1]->Variables_D());
    mvars.push_back(&m_nodes[1]->Variables_DD());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[2]->Variables_D());
    mvars.push_back(&m_nodes[2]->Variables_DD());
    mvars.push_back(&m_nodes[3]->Variables());
    mvars.push_back(&m_nodes[3]->Variables_D());
    mvars.push_back(&m_nodes[3]->Variables_DD());

    mvars.push_back(&m_nodes[4]->Variables());
    mvars.push_back(&m_nodes[4]->Variables_D());
    mvars.push_back(&m_nodes[4]->Variables_DD());
    mvars.push_back(&m_nodes[5]->Variables());
    mvars.push_back(&m_nodes[5]->Variables_D());
    mvars.push_back(&m_nodes[5]->Variables_DD());
    mvars.push_back(&m_nodes[6]->Variables());
    mvars.push_back(&m_nodes[6]->Variables_D());
    mvars.push_back(&m_nodes[6]->Variables_DD());
    mvars.push_back(&m_nodes[7]->Variables());
    mvars.push_back(&m_nodes[7]->Variables_D());
    mvars.push_back(&m_nodes[7]->Variables_DD());
    Kmatr.SetVariables(mvars);

    // Initial positions and slopes of the element nodes
    CalcCoordMatrix(m_d0);
    m_d0d0T = m_d0 * m_d0.transpose();
}

// -----------------------------------------------------------------------------
// Add a layer.
// -----------------------------------------------------------------------------

void ChElementShellANCF_8::AddLayer(double thickness, double theta, std::shared_ptr<ChMaterialShellANCF> material) {
    m_layers.push_back(Layer(this, thickness, theta, material));
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellANCF_8::SetupInitial(ChSystem* system) {
    // Perform layer initialization and accumulate element thickness.
    m_numLayers = m_layers.size();
    m_thickness = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        m_layers[kl].SetupInitial();
        m_thickness += m_layers[kl].Get_thickness();
    }

    // Loop again over the layers and calculate the range for Gauss integration in the
    // z direction (values in [-1,1]).
    m_GaussZ.push_back(-1);
    double z = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        z += m_layers[kl].Get_thickness();
        m_GaussZ.push_back(2 * z / m_thickness - 1);
    }

    // Cache the scaling factor (due to change of integration intervals)
    m_GaussScaling = (m_lenX * m_lenY * m_thickness) / 8;

    // Compute mass matrix and gravitational forces (constant)
    ComputeMassMatrix();
    ComputeGravityForce(system->Get_G_acc());
}

// State update.
void ChElementShellANCF_8::Update() {
    ChElementGeneric::Update();
}

// Fill the DD vector with the current field values at the element nodes.
void ChElementShellANCF_8::GetStateBlock(ChVectorDynamic<>& mDD) {
    mDD.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    mDD.segment(3, 3) = m_nodes[0]->GetD().eigen();
    mDD.segment(6, 3) = m_nodes[0]->GetDD().eigen();
    mDD.segment(9, 3) = m_nodes[1]->GetPos().eigen();
    mDD.segment(12, 3) = m_nodes[1]->GetD().eigen();
    mDD.segment(15, 3) = m_nodes[1]->GetDD().eigen();
    mDD.segment(18, 3) = m_nodes[2]->GetPos().eigen();
    mDD.segment(21, 3) = m_nodes[2]->GetD().eigen();
    mDD.segment(24, 3) = m_nodes[2]->GetDD().eigen();
    mDD.segment(27, 3) = m_nodes[3]->GetPos().eigen();
    mDD.segment(30, 3) = m_nodes[3]->GetD().eigen();
    mDD.segment(33, 3) = m_nodes[3]->GetDD().eigen();
    mDD.segment(36, 3) = m_nodes[4]->GetPos().eigen();
    mDD.segment(39, 3) = m_nodes[4]->GetD().eigen();
    mDD.segment(42, 3) = m_nodes[4]->GetDD().eigen();
    mDD.segment(45, 3) = m_nodes[5]->GetPos().eigen();
    mDD.segment(48, 3) = m_nodes[5]->GetD().eigen();
    mDD.segment(51, 3) = m_nodes[5]->GetDD().eigen();
    mDD.segment(54, 3) = m_nodes[6]->GetPos().eigen();
    mDD.segment(57, 3) = m_nodes[6]->GetD().eigen();
    mDD.segment(60, 3) = m_nodes[6]->GetDD().eigen();
    mDD.segment(63, 3) = m_nodes[7]->GetPos().eigen();
    mDD.segment(66, 3) = m_nodes[7]->GetD().eigen();
    mDD.segment(69, 3) = m_nodes[7]->GetDD().eigen();
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
void ChElementShellANCF_8::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 72) && (H.cols() == 72));

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 72; i++)
        for (int j = 0; j < 72; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// Return the mass matrix.
void ChElementShellANCF_8::ComputeMmatrixGlobal(ChMatrixRef M) {
    M = m_MassMatrix;
}

// -----------------------------------------------------------------------------
// Mass matrix calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the inertia matrix.
class ShellANCF8_Mass : public ChIntegrable3D<ChMatrixNM<double, 72, 72>> {
  public:
    ShellANCF8_Mass(ChElementShellANCF_8* element) : m_element(element) {}
    ~ShellANCF8_Mass() {}

  private:
    ChElementShellANCF_8* m_element;

    virtual void Evaluate(ChMatrixNM<double, 72, 72>& result, const double x, const double y, const double z) override;
};

void ShellANCF8_Mass::Evaluate(ChMatrixNM<double, 72, 72>& result, const double x, const double y, const double z) {
    ChElementShellANCF_8::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)...
    // N9*eye(3) N10*eye(3) N11*eye(3) N12*eye(3) N13*eye(3) N14*eye(3) N15*eye(3) N16*eye(3)...
    // N17*eye(3) N18*eye(3) N19*eye(3) N20*eye(3) N21*eye(3) N22*eye(3) N23*eye(3) N24*eye(3)]

    ChMatrixNM<double, 3, 72> S;
    S.setZero();
    for (int i = 0; i < 24; i++) {
        S(0, 3 * i + 0) = N(i);
        S(1, 3 * i + 1) = N(i);
        S(2, 3 * i + 2) = N(i);
    }

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    // perform  r = S'*S, scaled by integration weights
    result = detJ0 * m_element->m_GaussScaling * S.transpose() * S;
};

void ChElementShellANCF_8::ComputeMassMatrix() {
    m_MassMatrix.setZero();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        ShellANCF8_Mass myformula(this);
        ChMatrixNM<double, 72, 72> TempMassMatrix;
        TempMassMatrix.setZero();
        ChQuadrature::Integrate3D<ChMatrixNM<double, 72, 72>>(TempMassMatrix,  // result of integration will go there
                                                              myformula,       // formula to integrate
                                                              -1, 1,           // x limits
                                                              -1, 1,           // y limits
                                                              m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                              3                                // order of integration
        );
        TempMassMatrix *= rho;
        m_MassMatrix += TempMassMatrix;
    }
}
/// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass
void ChElementShellANCF_8::ComputeNodalMass() {
    m_nodes[0]->m_TotalMass += m_MassMatrix(0, 0) + m_MassMatrix(0, 9) + m_MassMatrix(0, 18) + m_MassMatrix(0, 27) +
                               m_MassMatrix(0, 36) + m_MassMatrix(0, 45) + m_MassMatrix(0, 54) + m_MassMatrix(0, 63);
    m_nodes[1]->m_TotalMass += m_MassMatrix(9, 0) + m_MassMatrix(9, 9) + m_MassMatrix(9, 18) + m_MassMatrix(9, 27) +
                               m_MassMatrix(9, 36) + m_MassMatrix(9, 45) + m_MassMatrix(9, 54) + m_MassMatrix(9, 63);
    m_nodes[2]->m_TotalMass += m_MassMatrix(18, 0) + m_MassMatrix(18, 9) + m_MassMatrix(18, 18) + m_MassMatrix(18, 27) +
                               m_MassMatrix(18, 36) + m_MassMatrix(18, 45) + m_MassMatrix(18, 54) +
                               m_MassMatrix(18, 63);
    m_nodes[3]->m_TotalMass += m_MassMatrix(27, 0) + m_MassMatrix(27, 9) + m_MassMatrix(27, 18) + m_MassMatrix(27, 27) +
                               m_MassMatrix(27, 36) + m_MassMatrix(27, 45) + m_MassMatrix(27, 54) +
                               m_MassMatrix(27, 63);
    m_nodes[4]->m_TotalMass += m_MassMatrix(36, 0) + m_MassMatrix(36, 9) + m_MassMatrix(36, 18) + m_MassMatrix(36, 27) +
                               m_MassMatrix(36, 36) + m_MassMatrix(36, 45) + m_MassMatrix(36, 54) +
                               m_MassMatrix(36, 63);
    m_nodes[5]->m_TotalMass += m_MassMatrix(45, 0) + m_MassMatrix(45, 9) + m_MassMatrix(45, 18) + m_MassMatrix(45, 27) +
                               m_MassMatrix(45, 36) + m_MassMatrix(45, 45) + m_MassMatrix(45, 54) +
                               m_MassMatrix(45, 63);
    m_nodes[6]->m_TotalMass += m_MassMatrix(54, 0) + m_MassMatrix(54, 9) + m_MassMatrix(54, 18) + m_MassMatrix(54, 27) +
                               m_MassMatrix(56, 36) + m_MassMatrix(54, 45) + m_MassMatrix(54, 54) +
                               m_MassMatrix(54, 63);
    m_nodes[7]->m_TotalMass += m_MassMatrix(63, 0) + m_MassMatrix(63, 9) + m_MassMatrix(63, 18) + m_MassMatrix(63, 27) +
                               m_MassMatrix(0, 36) + m_MassMatrix(63, 45) + m_MassMatrix(63, 54) + m_MassMatrix(63, 63);
}
// -----------------------------------------------------------------------------
// Gravitational force calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the element gravity forces
class ShellANCF8_Gravity : public ChIntegrable3D<ChVectorN<double, 72>> {
  public:
    ShellANCF8_Gravity(ChElementShellANCF_8* element, const ChVector<> gacc) : m_element(element), m_gacc(gacc) {}
    ~ShellANCF8_Gravity() {}

  private:
    ChElementShellANCF_8* m_element;
    ChVector<> m_gacc;

    virtual void Evaluate(ChVectorN<double, 72>& result, const double x, const double y, const double z) override;
};

void ShellANCF8_Gravity::Evaluate(ChVectorN<double, 72>& result, const double x, const double y, const double z) {
    ChElementShellANCF_8::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);
    double detJ0 = m_element->Calc_detJ0(x, y, z);

    for (int i = 0; i < 24; i++) {
        result(i * 3 + 0) = N(0, i) * m_gacc.x();
        result(i * 3 + 1) = N(0, i) * m_gacc.y();
        result(i * 3 + 2) = N(0, i) * m_gacc.z();
    }
    result *= detJ0 * m_element->m_GaussScaling;
};

void ChElementShellANCF_8::ComputeGravityForce(const ChVector<>& g_acc) {
    m_GravForce.setZero();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        ShellANCF8_Gravity myformula(this, g_acc);
        ChVectorN<double, 72> Fgravity;
        Fgravity.setZero();
        ChQuadrature::Integrate3D<ChVectorN<double, 72>>(Fgravity,   // result of integration will go there
                                                         myformula,  // formula to integrate
                                                         -1, 1,      // x limits
                                                         -1, 1,      // y limits
                                                         m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                         3                                // order of integration
        );

        Fgravity *= rho;
        m_GravForce += Fgravity;
    }
}

// -----------------------------------------------------------------------------
// Elastic force calculation
// -----------------------------------------------------------------------------

// The class ShellANCF8_Force provides the integrand for the calculation of the internal forces
// for one layer of an ANCF shell element.
// The first 72 entries in the integrand represent the internal force.
// This implementation also features a composite material implementation
// that allows for selecting a number of layers over the element thickness; each of which
// has an independent, user-selected fiber angle (direction for orthotropic constitutive behavior)
class ShellANCF8_Force : public ChIntegrable3D<ChVectorN<double, 72>> {
  public:
    ShellANCF8_Force(ChElementShellANCF_8* element,  // Containing element
                     size_t kl                       // Current layer index
                     )
        : m_element(element), m_kl(kl) {}
    ~ShellANCF8_Force() {}

  private:
    ChElementShellANCF_8* m_element;
    size_t m_kl;

    /// Evaluate (strainD'*strain)  at point x
    virtual void Evaluate(ChVectorN<double, 72>& result, const double x, const double y, const double z) override;
};

void ShellANCF8_Force::Evaluate(ChVectorN<double, 72>& result, const double x, const double y, const double z) {
    // Element shape function
    ChElementShellANCF_8::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 24> Nx;
    ChMatrixNM<double, 1, 24> Ny;
    ChMatrixNM<double, 1, 24> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(1) * Ny_d0(2) - Nx_d0(2) * Ny_d0(1);
    G1xG2.y() = Nx_d0(2) * Ny_d0(0) - Nx_d0(0) * Ny_d0(2);
    G1xG2.z() = Nx_d0(0) * Ny_d0(1) - Nx_d0(1) * Ny_d0(0);
    G1dotG1 = Nx_d0(0) * Nx_d0(0) + Nx_d0(1) * Nx_d0(1) + Nx_d0(2) * Nx_d0(2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0);
    A1.y() = Nx_d0(1);
    A1.z() = Nx_d0(2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    // Direction for orthotropic material
    double theta = m_element->GetLayer(m_kl).Get_theta();  // Fiber angle
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(theta) + A2 * sin(theta);
    AA2 = -A1 * sin(theta) + A2 * cos(theta);
    AA3 = A3;

    /// Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    ChVectorN<double, 9> beta;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0(1) * Nz_d0(2) - Nz_d0(1) * Ny_d0(2);
    j0(0, 1) = Ny_d0(2) * Nz_d0(0) - Ny_d0(0) * Nz_d0(2);
    j0(0, 2) = Ny_d0(0) * Nz_d0(1) - Nz_d0(0) * Ny_d0(1);
    j0(1, 0) = Nz_d0(1) * Nx_d0(2) - Nx_d0(1) * Nz_d0(2);
    j0(1, 1) = Nz_d0(2) * Nx_d0(0) - Nx_d0(2) * Nz_d0(0);
    j0(1, 2) = Nz_d0(0) * Nx_d0(1) - Nz_d0(1) * Nx_d0(0);
    j0(2, 0) = Nx_d0(1) * Ny_d0(2) - Ny_d0(1) * Nx_d0(2);
    j0(2, 1) = Ny_d0(0) * Nx_d0(2) - Nx_d0(0) * Ny_d0(2);
    j0(2, 2) = Nx_d0(0) * Ny_d0(1) - Ny_d0(0) * Nx_d0(1);
    j0 /= detJ0;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    // Coefficients of contravariant transformation
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    // Transformation matrix, function of fiber angle

    ChVectorN<double, 24> ddNx = m_element->m_ddT * Nx.transpose();
    ChVectorN<double, 24> ddNy = m_element->m_ddT * Ny.transpose();
    ChVectorN<double, 24> ddNz = m_element->m_ddT * Nz.transpose();

    ChVectorN<double, 24> d0d0Nx = m_element->m_d0d0T * Nx.transpose();
    ChVectorN<double, 24> d0d0Ny = m_element->m_d0d0T * Ny.transpose();
    ChVectorN<double, 24> d0d0Nz = m_element->m_d0d0T * Nz.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);          // xy
    strain_til(3) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));  // zz
    strain_til(4) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);          // yz
    strain_til(5) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);          // xz

    // For orthotropic material
    ChVectorN<double, 6> strain;

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

    // Strain derivative component

    ChMatrixNM<double, 6, 72> strainD_til;

    ChMatrixNM<double, 1, 72> tempB;
    ChMatrixNM<double, 1, 3> tempB3;
    ChMatrixNM<double, 1, 3> tempB31;
    ChMatrixNM<double, 1, 3> tempB32;

    tempB3 = Nx * m_element->m_d;  // rx
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(0) = tempB;  // xx

    tempB31 = Ny * m_element->m_d;  // ry
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(1) = tempB;  // yy

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i) + tempB31(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(2) = tempB;  // xy

    tempB32 = Nz * m_element->m_d;  // rz
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Nz(0, i);
        }
    }
    strainD_til.row(3) = tempB;  // zz

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Ny(0, i) + tempB31(0, j) * Nz(0, i);
        }
    }
    strainD_til.row(4) = tempB;  // yz

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i) + tempB32(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(5) = tempB;  // xz

    // For orthotropic material
    ChMatrixNM<double, 6, 72> strainD;  // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
    for (int ii = 0; ii < 72; ii++) {
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

    // Strain time derivative for structural damping
    ChVectorN<double, 6> DEPS;
    DEPS.setZero();
    for (int ii = 0; ii < 72; ii++) {
        DEPS(0) += strainD(0, ii) * m_element->m_d_dt(ii);
        DEPS(1) += strainD(1, ii) * m_element->m_d_dt(ii);
        DEPS(2) += strainD(2, ii) * m_element->m_d_dt(ii);
        DEPS(3) += strainD(3, ii) * m_element->m_d_dt(ii);
        DEPS(4) += strainD(4, ii) * m_element->m_d_dt(ii);
        DEPS(5) += strainD(5, ii) * m_element->m_d_dt(ii);
    }

    // Add structural damping
    strain += DEPS * m_element->m_Alpha;

    // Matrix of elastic coefficients: the input assumes the material *could* be orthotropic
    const ChMatrixNM<double, 6, 6>& E_eps = m_element->GetLayer(m_kl).GetMaterial()->Get_E_eps();

    // Internal force calculation
    ChMatrixNM<double, 72, 6> tempC = strainD.transpose() * E_eps;

    result = (tempC * strain) * (detJ0 * m_element->m_GaussScaling);
}

void ChElementShellANCF_8::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    // Current nodal coordinates and velocities
    CalcCoordMatrix(m_d);
    CalcCoordDerivMatrix(m_d_dt);
    m_ddT = m_d * m_d.transpose();
    Fi.setZero();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        ShellANCF8_Force formula(this, kl);
        ChVectorN<double, 72> Finternal;
        Finternal.setZero();
        ChQuadrature::Integrate3D<ChVectorN<double, 72>>(Finternal,                       // result of integration
                                                         formula,                         // integrand formula
                                                         -1, 1,                           // x limits
                                                         -1, 1,                           // y limits
                                                         m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                         5                                // order of integration
        );

        // Accumulate internal force
        Fi -= Finternal;

    }  // Layer Loop

    if (m_gravity_on) {
        Fi += m_GravForce;
    }
}

// -----------------------------------------------------------------------------
// Jacobians of internal forces
// -----------------------------------------------------------------------------

// The class ShellANCF8_Jacobian provides the integrand for the calculation of the Jacobians
// (stiffness and damping matrices) of the internal forces for one layer of an ANCF
// shell element.
// 72x72 Jacobian
//      Kfactor * [K] + Rfactor * [R]

class ShellANCF8_Jacobian : public ChIntegrable3D<ChVectorN<double, 5184>> {
  public:
    ShellANCF8_Jacobian(ChElementShellANCF_8* element,  // Containing element
                        double Kfactor,                 // Scaling coefficient for stiffness component
                        double Rfactor,                 // Scaling coefficient for damping component
                        size_t kl                       // Current layer index
                        )
        : m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor), m_kl(kl) {}

  private:
    ChElementShellANCF_8* m_element;
    double m_Kfactor;
    double m_Rfactor;
    size_t m_kl;

    // Evaluate integrand at the specified point.
    virtual void Evaluate(ChVectorN<double, 5184>& result, const double x, const double y, const double z) override;
};

void ShellANCF8_Jacobian::Evaluate(ChVectorN<double, 5184>& result, const double x, const double y, const double z) {
    // Element shape function
    ChElementShellANCF_8::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 24> Nx;
    ChMatrixNM<double, 1, 24> Ny;
    ChMatrixNM<double, 1, 24> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(1) * Ny_d0(2) - Nx_d0(2) * Ny_d0(1);
    G1xG2.y() = Nx_d0(2) * Ny_d0(0) - Nx_d0(0) * Ny_d0(2);
    G1xG2.z() = Nx_d0(0) * Ny_d0(1) - Nx_d0(1) * Ny_d0(0);
    G1dotG1 = Nx_d0(0) * Nx_d0(0) + Nx_d0(1) * Nx_d0(1) + Nx_d0(2) * Nx_d0(2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0);
    A1.y() = Nx_d0(1);
    A1.z() = Nx_d0(2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    // Direction for orthotropic material
    double theta = m_element->GetLayer(m_kl).Get_theta();  // Fiber angle
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(theta) + A2 * sin(theta);
    AA2 = -A1 * sin(theta) + A2 * cos(theta);
    AA3 = A3;

    /// Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    ChVectorN<double, 9> beta;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0(1) * Nz_d0(2) - Nz_d0(1) * Ny_d0(2);
    j0(0, 1) = Ny_d0(2) * Nz_d0(0) - Ny_d0(0) * Nz_d0(2);
    j0(0, 2) = Ny_d0(0) * Nz_d0(1) - Nz_d0(0) * Ny_d0(1);
    j0(1, 0) = Nz_d0(1) * Nx_d0(2) - Nx_d0(1) * Nz_d0(2);
    j0(1, 1) = Nz_d0(2) * Nx_d0(0) - Nx_d0(2) * Nz_d0(0);
    j0(1, 2) = Nz_d0(0) * Nx_d0(1) - Nz_d0(1) * Nx_d0(0);
    j0(2, 0) = Nx_d0(1) * Ny_d0(2) - Ny_d0(1) * Nx_d0(2);
    j0(2, 1) = Ny_d0(0) * Nx_d0(2) - Nx_d0(0) * Ny_d0(2);
    j0(2, 2) = Nx_d0(0) * Ny_d0(1) - Ny_d0(0) * Nx_d0(1);
    j0 /= detJ0;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    // Coefficients of contravariant transformation
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    // Transformation matrix, function of fiber angle

    ChVectorN<double, 24> ddNx = m_element->m_ddT * Nx.transpose();
    ChVectorN<double, 24> ddNy = m_element->m_ddT * Ny.transpose();
    ChVectorN<double, 24> ddNz = m_element->m_ddT * Nz.transpose();

    ChVectorN<double, 24> d0d0Nx = m_element->m_d0d0T * Nx.transpose();
    ChVectorN<double, 24> d0d0Ny = m_element->m_d0d0T * Ny.transpose();
    ChVectorN<double, 24> d0d0Nz = m_element->m_d0d0T * Nz.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);          // xy
    strain_til(3) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));  // zz
    strain_til(4) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);          // yz
    strain_til(5) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);          // xz

    // For orthotropic material
    ChVectorN<double, 6> strain;

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

    // Strain derivative component

    ChMatrixNM<double, 6, 72> strainD_til;
    strainD_til.setZero();

    ChMatrixNM<double, 1, 72> tempB;
    ChMatrixNM<double, 1, 3> tempB3;
    ChMatrixNM<double, 1, 3> tempB31;
    ChMatrixNM<double, 1, 3> tempB32;

    tempB3 = Nx * m_element->m_d;  // rx
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(0) = tempB;  // xx

    tempB31 = Ny * m_element->m_d;  // ry
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(1) = tempB;  // yy

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i) + tempB31(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(2) = tempB;  // xy

    tempB32 = Nz * m_element->m_d;  // rz
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Nz(0, i);
        }
    }
    strainD_til.row(3) = tempB;  // zz

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Ny(0, i) + tempB31(0, j) * Nz(0, i);
        }
    }
    strainD_til.row(4) = tempB;  // yz

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i) + tempB32(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(5) = tempB;  // xz

    //// For orthotropic material
    ChMatrixNM<double, 6, 72> strainD;  // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
    for (int ii = 0; ii < 72; ii++) {
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

    /// Gd : Jacobian (w.r.t. coordinates) of the initial position vector gradient matrix
    ChMatrixNM<double, 9, 72> Gd;
    Gd.setZero();

    for (int ii = 0; ii < 24; ii++) {
        Gd(0, 3 * (ii)) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
        Gd(1, 3 * (ii) + 1) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
        Gd(2, 3 * (ii) + 2) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);

        Gd(3, 3 * (ii)) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
        Gd(4, 3 * (ii) + 1) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
        Gd(5, 3 * (ii) + 2) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);

        Gd(6, 3 * (ii)) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
        Gd(7, 3 * (ii) + 1) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
        Gd(8, 3 * (ii) + 2) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
    }

    // Structural damping
    // Strain time derivative for structural damping
    ChVectorN<double, 6> DEPS;
    DEPS.setZero();
    for (int ii = 0; ii < 72; ii++) {
        DEPS(0) += strainD(0, ii) * m_element->m_d_dt(ii);
        DEPS(1) += strainD(1, ii) * m_element->m_d_dt(ii);
        DEPS(2) += strainD(2, ii) * m_element->m_d_dt(ii);
        DEPS(3) += strainD(3, ii) * m_element->m_d_dt(ii);
        DEPS(4) += strainD(4, ii) * m_element->m_d_dt(ii);
        DEPS(5) += strainD(5, ii) * m_element->m_d_dt(ii);
    }

    // Add structural damping
    strain += DEPS * m_element->m_Alpha;

    // Matrix of elastic coefficients: The input assumes the material *could* be orthotropic
    const ChMatrixNM<double, 6, 6>& E_eps = m_element->GetLayer(m_kl).GetMaterial()->Get_E_eps();

    // Stress tensor calculation
    ChVectorN<double, 6> stress = E_eps * strain;

    // Declaration and computation of Sigm, to be removed
    ChMatrixNM<double, 9, 9> Sigm;  ///< Rearrangement of stress vector (not always needed)
    Sigm.setZero();

    Sigm(0, 0) = stress(0);  // XX
    Sigm(1, 1) = stress(0);
    Sigm(2, 2) = stress(0);

    Sigm(0, 3) = stress(2);  // XY
    Sigm(1, 4) = stress(2);
    Sigm(2, 5) = stress(2);

    Sigm(0, 6) = stress(4);  // XZ
    Sigm(1, 7) = stress(4);
    Sigm(2, 8) = stress(4);

    Sigm(3, 0) = stress(2);  // XY
    Sigm(4, 1) = stress(2);
    Sigm(5, 2) = stress(2);

    Sigm(3, 3) = stress(1);  // YY
    Sigm(4, 4) = stress(1);
    Sigm(5, 5) = stress(1);

    Sigm(3, 6) = stress(5);  // YZ
    Sigm(4, 7) = stress(5);
    Sigm(5, 8) = stress(5);

    Sigm(6, 0) = stress(4);  // XZ
    Sigm(7, 1) = stress(4);
    Sigm(8, 2) = stress(4);

    Sigm(6, 3) = stress(5);  // YZ
    Sigm(7, 4) = stress(5);
    Sigm(8, 5) = stress(5);

    Sigm(6, 6) = stress(3);  // ZZ
    Sigm(7, 7) = stress(3);
    Sigm(8, 8) = stress(3);

    // Jacobian of internal forces (excluding the EAS contribution).
    ChMatrixNM<double, 72, 72> KTE;
    KTE = (strainD.transpose() * E_eps * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) +
          (Gd.transpose() * Sigm * Gd) * m_Kfactor;
    KTE *= detJ0 * m_element->m_GaussScaling;

    // Load result vector (integrand)
    result = Eigen::Map<ChVectorN<double, 72 * 72>>(KTE.data(), 72 * 72);
}

void ChElementShellANCF_8::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    // Note that the matrices with current nodal coordinates and velocities are
    // already available in m_d and m_d_dt (as set in ComputeInternalForces).

    m_JacobianMatrix.setZero();

    // Loop over all layers.
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        ShellANCF8_Jacobian formula(this, Kfactor, Rfactor, kl);
        ChVectorN<double, 5184> result;
        result.setZero();
        ChQuadrature::Integrate3D<ChVectorN<double, 5184>>(result,                          // result of integration
                                                           formula,                         // integrand formula
                                                           -1, 1,                           // x limits
                                                           -1, 1,                           // y limits
                                                           m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                           3                                // order of integration
        );

        // Extract matrices from result of integration
        ChMatrixNM<double, 72, 72> KTE = Eigen::Map<ChMatrixNM<double, 72, 72>>(result.data(), 72, 72);

        // Accumulate Jacobian
        m_JacobianMatrix += KTE;
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementShellANCF_8::ShapeFunctions(ShapeVector& N, double x, double y, double z) {
    double c = m_thickness;

    N(0) = -(-1 + y) * (-1 + x) * (x + y + 1) / 4;
    N(1) = -c * z * (-1 + y) * (-1 + x) * (x + y + 1) / 8;
    N(2) = -z * z * c * c * (-1 + y) * (-1 + x) * (x + y + 1) / 32;
    N(3) = -(-1 + y) * (1 + x) * (x - y - 1) / 4;
    N(4) = -c * z * (-1 + y) * (1 + x) * (x - 1 + y) / 8;
    N(5) = -z * z * c * c * (-1 + y) * (1 + x) * (x - y - 1) / 32;
    N(6) = (1 + y) * (1 + x) * (x + y - 1) / 4;
    N(7) = c * z * (1 + y) * (1 + x) * (x - 1 + y) / 8;
    N(8) = z * z * c * c * (1 + y) * (1 + x) * (x + y - 1) / 32;
    N(9) = (1 + y) * (-1 + x) * (x - y + 1) / 4;
    N(10) = c * z * (1 + y) * (-1 + x) * (x + 1 - y) / 8;
    N(11) = z * z * c * c * (1 + y) * (-1 + x) * (x - y + 1) / 32;
    N(12) = (-1 + x) * (1 + x) * (-1 + y) / 2;
    N(13) = c * z * (-1 + x) * (1 + x) * (-1 + y) / 4;
    N(14) = z * z * c * c * (-1 + x) * (1 + x) * (-1 + y) / 16;
    N(15) = -(-1 + y) * (1 + y) * (1 + x) / 2;
    N(16) = -c * z * (-1 + y) * (1 + y) * (1 + x) / 4;
    N(17) = -z * z * c * c * (-1 + y) * (1 + y) * (1 + x) / 16;
    N(18) = -(-1 + x) * (1 + x) * (1 + y) / 2;
    N(19) = -c * z * (-1 + x) * (1 + x) * (1 + y) / 4;
    N(20) = -z * z * c * c * (-1 + x) * (1 + x) * (1 + y) / 16;
    N(21) = (-1 + y) * (1 + y) * (-1 + x) / 2;
    N(22) = c * z * (-1 + y) * (1 + y) * (-1 + x) / 4;
    N(23) = z * z * c * c * (-1 + y) * (1 + y) * (-1 + x) / 16;
}

void ChElementShellANCF_8::ShapeFunctionsDerivativeX(ShapeVector& Nx, double x, double y, double z) {
    double a = GetLengthX();
    double c = m_thickness;

    Nx(0) = -((2 * x + y) * (y - 1)) / (2 * a);
    Nx(1) = -(c * z * (2 * x + y) * (y - 1)) / (4 * a);
    Nx(2) = -(c * c * z * c * (2 * x + y) * (y - 1)) / (16 * a);
    Nx(3) = -((2 * x - y) * (y - 1)) / (2 * a);
    Nx(4) = -(c * z * (2 * x - y) * (y - 1)) / (4 * a);
    Nx(5) = -(c * c * z * z * (2 * x - y) * (y - 1)) / (16 * a);
    Nx(6) = ((2 * x + y) * (y + 1)) / (2 * a);
    Nx(7) = (c * z * (2 * x + y) * (y + 1)) / (4 * a);
    Nx(8) = (c * c * z * z * (2 * x + y) * (y + 1)) / (16 * a);
    Nx(9) = ((2 * x - y) * (y + 1)) / (2 * a);
    Nx(10) = (c * z * (2 * x - y) * (y + 1)) / (4 * a);
    Nx(11) = (c * c * z * z * (2 * x - y) * (y + 1)) / (16 * a);
    Nx(12) = (2 * x * (y - 1)) / a;
    Nx(13) = (c * x * z * (y - 1)) / a;
    Nx(14) = (c * c * x * z * z * (y - 1)) / (4 * a);
    Nx(15) = -(y * y - 1) / a;
    Nx(16) = -(c * z * (y - 1) * (y + 1)) / (2 * a);
    Nx(17) = -(c * c * z * z * (y - 1) * (y + 1)) / (8 * a);
    Nx(18) = -(2 * x * (y + 1)) / a;
    Nx(19) = -(c * x * z * (y + 1)) / a;
    Nx(20) = -(c * c * x * z * z * (y + 1)) / (4 * a);
    Nx(21) = (y * y - 1) / a;
    Nx(22) = (c * z * (y - 1) * (y + 1)) / (2 * a);
    Nx(23) = (c * c * z * z * (y - 1) * (y + 1)) / (8 * a);
}

void ChElementShellANCF_8::ShapeFunctionsDerivativeY(ShapeVector& Ny, double x, double y, double z) {
    double b = GetLengthY();
    double c = m_thickness;

    Ny(0) = -((x + 2 * y) * (x - 1)) / (2 * b);
    Ny(1) = -(c * z * (x + 2 * y) * (x - 1)) / (4 * b);
    Ny(2) = -(c * c * z * c * (x + 2 * y) * (x - 1)) / (16 * b);
    Ny(3) = -((x - 2 * y) * (x + 1)) / (2 * b);
    Ny(4) = -(c * z * (x - 2 * y) * (x + 1)) / (4 * b);
    Ny(5) = -(c * c * z * z * (x - 2 * y) * (x + 1)) / (16 * b);
    Ny(6) = ((x + 2 * y) * (x + 1)) / (2 * b);
    Ny(7) = (c * z * (x + 2 * y) * (x + 1)) / (4 * b);
    Ny(8) = (c * c * z * z * (x + 2 * y) * (x + 1)) / (16 * b);
    Ny(9) = ((x - 2 * y) * (x - 1)) / (2 * b);
    Ny(10) = (c * z * (x - 2 * y) * (x - 1)) / (4 * b);
    Ny(11) = (c * c * z * z * (x - 2 * y) * (x - 1)) / (16 * b);
    Ny(12) = (x * x - 1) / b;
    Ny(13) = (c * z * (x - 1) * (x + 1)) / (2 * b);
    Ny(14) = (c * c * z * z * (x - 1) * (x + 1)) / (8 * b);
    Ny(15) = -(2 * y * (x + 1)) / b;
    Ny(16) = -(c * y * z * (x + 1)) / b;
    Ny(17) = -(c * c * y * z * z * (x + 1)) / (4 * b);
    Ny(18) = -(x * x - 1) / b;
    Ny(19) = -(c * z * (x - 1) * (x + 1)) / (2 * b);
    Ny(20) = -(c * c * z * z * (x - 1) * (x + 1)) / (8 * b);
    Ny(21) = (2 * y * (x - 1)) / b;
    Ny(22) = (c * y * z * (x - 1)) / b;
    Ny(23) = (c * c * y * z * z * (x - 1)) / (4 * b);
}

void ChElementShellANCF_8::ShapeFunctionsDerivativeZ(ShapeVector& Nz, double x, double y, double z) {
    double c = m_thickness;
    Nz(0) = 0.0;
    Nz(1) = -((x - 1) * (y - 1) * (x + y + 1)) / 4;
    Nz(2) = -(c * z * (x - 1) * (y - 1) * (x + y + 1)) / 8;
    Nz(3) = 0.0;
    Nz(4) = ((x + 1) * (y - 1) * (y - x + 1)) / 4;
    Nz(5) = (c * z * (x + 1) * (y - 1) * (y - x + 1)) / 8;
    Nz(6) = 0.0;
    Nz(7) = ((x + 1) * (y + 1) * (x + y - 1)) / 4;
    Nz(8) = (c * z * (x + 1) * (y + 1) * (x + y - 1)) / 8;
    Nz(9) = 0.0;
    Nz(10) = ((x - 1) * (y + 1) * (x - y + 1)) / 4;
    Nz(11) = (c * z * (x - 1) * (y + 1) * (x - y + 1)) / 8;
    Nz(12) = 0.0;
    Nz(13) = ((x - 1) * (x + 1) * (y - 1)) / 2;
    Nz(14) = (c * z * (x - 1) * (x + 1) * (y - 1)) / 4;
    Nz(15) = 0.0;
    Nz(16) = -((x + 1) * (y - 1) * (y + 1)) / 2;
    Nz(17) = -(c * z * (x + 1) * (y - 1) * (y + 1)) / 4;
    Nz(18) = 0.0;
    Nz(19) = -((x - 1) * (x + 1) * (y + 1)) / 2;
    Nz(20) = -(c * z * (x - 1) * (x + 1) * (y + 1)) / 4;
    Nz(21) = 0.0;
    Nz(22) = ((x - 1) * (y - 1) * (y + 1)) / 2;
    Nz(23) = (c * z * (x - 1) * (y - 1) * (y + 1)) / 4;
}

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------
double ChElementShellANCF_8::Calc_detJ0(double x,
                                        double y,
                                        double z,
                                        ShapeVector& Nx,
                                        ShapeVector& Ny,
                                        ShapeVector& Nz,
                                        ChMatrixNM<double, 1, 3>& Nx_d0,
                                        ChMatrixNM<double, 1, 3>& Ny_d0,
                                        ChMatrixNM<double, 1, 3>& Nz_d0) {
    ShapeFunctionsDerivativeX(Nx, x, y, z);
    ShapeFunctionsDerivativeY(Ny, x, y, z);
    ShapeFunctionsDerivativeZ(Nz, x, y, z);

    Nx_d0 = Nx * m_d0;
    Ny_d0 = Ny * m_d0;
    Nz_d0 = Nz * m_d0;

    double detJ0 = Nx_d0(0, 0) * Ny_d0(0, 1) * Nz_d0(0, 2) + Ny_d0(0, 0) * Nz_d0(0, 1) * Nx_d0(0, 2) +
                   Nz_d0(0, 0) * Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1) * Nz_d0(0, 0) -
                   Ny_d0(0, 2) * Nz_d0(0, 1) * Nx_d0(0, 0) - Nz_d0(0, 2) * Nx_d0(0, 1) * Ny_d0(0, 0);

    return detJ0;
}

double ChElementShellANCF_8::Calc_detJ0(double x, double y, double z) {
    ChMatrixNM<double, 1, 24> Nx;
    ChMatrixNM<double, 1, 24> Ny;
    ChMatrixNM<double, 1, 24> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;

    return Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);
}

void ChElementShellANCF_8::CalcCoordMatrix(ChMatrixNM<double, 24, 3>& d) {
    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& dA = m_nodes[0]->GetD();
    const ChVector<>& ddA = m_nodes[0]->GetDD();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& dB = m_nodes[1]->GetD();
    const ChVector<>& ddB = m_nodes[1]->GetDD();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& dC = m_nodes[2]->GetD();
    const ChVector<>& ddC = m_nodes[2]->GetDD();
    const ChVector<>& pD = m_nodes[3]->GetPos();
    const ChVector<>& dD = m_nodes[3]->GetD();
    const ChVector<>& ddD = m_nodes[3]->GetDD();

    const ChVector<>& pE = m_nodes[4]->GetPos();
    const ChVector<>& dE = m_nodes[4]->GetD();
    const ChVector<>& ddE = m_nodes[4]->GetDD();
    const ChVector<>& pF = m_nodes[5]->GetPos();
    const ChVector<>& dF = m_nodes[5]->GetD();
    const ChVector<>& ddF = m_nodes[5]->GetDD();
    const ChVector<>& pG = m_nodes[6]->GetPos();
    const ChVector<>& dG = m_nodes[6]->GetD();
    const ChVector<>& ddG = m_nodes[6]->GetDD();
    const ChVector<>& pH = m_nodes[7]->GetPos();
    const ChVector<>& dH = m_nodes[7]->GetD();
    const ChVector<>& ddH = m_nodes[7]->GetDD();

    d(0, 0) = pA.x();
    d(0, 1) = pA.y();
    d(0, 2) = pA.z();
    d(1, 0) = dA.x();
    d(1, 1) = dA.y();
    d(1, 2) = dA.z();
    d(2, 0) = ddA.x();
    d(2, 1) = ddA.y();
    d(2, 2) = ddA.z();

    d(3, 0) = pB.x();
    d(3, 1) = pB.y();
    d(3, 2) = pB.z();
    d(4, 0) = dB.x();
    d(4, 1) = dB.y();
    d(4, 2) = dB.z();
    d(5, 0) = ddB.x();
    d(5, 1) = ddB.y();
    d(5, 2) = ddB.z();

    d(6, 0) = pC.x();
    d(6, 1) = pC.y();
    d(6, 2) = pC.z();
    d(7, 0) = dC.x();
    d(7, 1) = dC.y();
    d(7, 2) = dC.z();
    d(8, 0) = ddC.x();
    d(8, 1) = ddC.y();
    d(8, 2) = ddC.z();

    d(9, 0) = pD.x();
    d(9, 1) = pD.y();
    d(9, 2) = pD.z();
    d(10, 0) = dD.x();
    d(10, 1) = dD.y();
    d(10, 2) = dD.z();
    d(11, 0) = ddD.x();
    d(11, 1) = ddD.y();
    d(11, 2) = ddD.z();

    d(12, 0) = pE.x();
    d(12, 1) = pE.y();
    d(12, 2) = pE.z();
    d(13, 0) = dE.x();
    d(13, 1) = dE.y();
    d(13, 2) = dE.z();
    d(14, 0) = ddE.x();
    d(14, 1) = ddE.y();
    d(14, 2) = ddE.z();

    d(15, 0) = pF.x();
    d(15, 1) = pF.y();
    d(15, 2) = pF.z();
    d(16, 0) = dF.x();
    d(16, 1) = dF.y();
    d(16, 2) = dF.z();
    d(17, 0) = ddF.x();
    d(17, 1) = ddF.y();
    d(17, 2) = ddF.z();

    d(18, 0) = pG.x();
    d(18, 1) = pG.y();
    d(18, 2) = pG.z();
    d(19, 0) = dG.x();
    d(19, 1) = dG.y();
    d(19, 2) = dG.z();
    d(20, 0) = ddG.x();
    d(20, 1) = ddG.y();
    d(20, 2) = ddG.z();

    d(21, 0) = pH.x();
    d(21, 1) = pH.y();
    d(21, 2) = pH.z();
    d(22, 0) = dH.x();
    d(22, 1) = dH.y();
    d(22, 2) = dH.z();
    d(23, 0) = ddH.x();
    d(23, 1) = ddH.y();
    d(23, 2) = ddH.z();
}

void ChElementShellANCF_8::CalcCoordDerivMatrix(ChVectorN<double, 72>& dt) {
    const ChVector<>& pA_dt = m_nodes[0]->GetPos_dt();
    const ChVector<>& dA_dt = m_nodes[0]->GetD_dt();
    const ChVector<>& ddA_dt = m_nodes[0]->GetDD_dt();
    const ChVector<>& pB_dt = m_nodes[1]->GetPos_dt();
    const ChVector<>& dB_dt = m_nodes[1]->GetD_dt();
    const ChVector<>& ddB_dt = m_nodes[1]->GetDD_dt();
    const ChVector<>& pC_dt = m_nodes[2]->GetPos_dt();
    const ChVector<>& dC_dt = m_nodes[2]->GetD_dt();
    const ChVector<>& ddC_dt = m_nodes[2]->GetDD_dt();
    const ChVector<>& pD_dt = m_nodes[3]->GetPos_dt();
    const ChVector<>& dD_dt = m_nodes[3]->GetD_dt();
    const ChVector<>& ddD_dt = m_nodes[3]->GetDD_dt();

    const ChVector<>& pE_dt = m_nodes[4]->GetPos_dt();
    const ChVector<>& dE_dt = m_nodes[4]->GetD_dt();
    const ChVector<>& ddE_dt = m_nodes[4]->GetDD_dt();
    const ChVector<>& pF_dt = m_nodes[5]->GetPos_dt();
    const ChVector<>& dF_dt = m_nodes[5]->GetD_dt();
    const ChVector<>& ddF_dt = m_nodes[5]->GetDD_dt();
    const ChVector<>& pG_dt = m_nodes[6]->GetPos_dt();
    const ChVector<>& dG_dt = m_nodes[6]->GetD_dt();
    const ChVector<>& ddG_dt = m_nodes[6]->GetDD_dt();
    const ChVector<>& pH_dt = m_nodes[7]->GetPos_dt();
    const ChVector<>& dH_dt = m_nodes[7]->GetD_dt();
    const ChVector<>& ddH_dt = m_nodes[7]->GetDD_dt();

    dt(0) = pA_dt.x();
    dt(1) = pA_dt.y();
    dt(2) = pA_dt.z();
    dt(3) = dA_dt.x();
    dt(4) = dA_dt.y();
    dt(5) = dA_dt.z();
    dt(6) = ddA_dt.x();
    dt(7) = ddA_dt.y();
    dt(8) = ddA_dt.z();

    dt(9) = pB_dt.x();
    dt(10) = pB_dt.y();
    dt(11) = pB_dt.z();
    dt(12) = dB_dt.x();
    dt(13) = dB_dt.y();
    dt(14) = dB_dt.z();
    dt(15) = ddB_dt.x();
    dt(16) = ddB_dt.y();
    dt(17) = ddB_dt.z();

    dt(18) = pC_dt.x();
    dt(19) = pC_dt.y();
    dt(20) = pC_dt.z();
    dt(21) = dC_dt.x();
    dt(22) = dC_dt.y();
    dt(23) = dC_dt.z();
    dt(24) = ddC_dt.x();
    dt(25) = ddC_dt.y();
    dt(26) = ddC_dt.z();

    dt(27) = pD_dt.x();
    dt(28) = pD_dt.y();
    dt(29) = pD_dt.z();
    dt(30) = dD_dt.x();
    dt(31) = dD_dt.y();
    dt(32) = dD_dt.z();
    dt(33) = ddD_dt.x();
    dt(34) = ddD_dt.y();
    dt(35) = ddD_dt.z();

    dt(36) = pE_dt.x();
    dt(37) = pE_dt.y();
    dt(38) = pE_dt.z();
    dt(39) = dE_dt.x();
    dt(40) = dE_dt.y();
    dt(41) = dE_dt.z();
    dt(42) = ddE_dt.x();
    dt(43) = ddE_dt.y();
    dt(44) = ddE_dt.z();

    dt(45) = pF_dt.x();
    dt(46) = pF_dt.y();
    dt(47) = pF_dt.z();
    dt(48) = dF_dt.x();
    dt(49) = dF_dt.y();
    dt(50) = dF_dt.z();
    dt(51) = ddF_dt.x();
    dt(52) = ddF_dt.y();
    dt(53) = ddF_dt.z();

    dt(54) = pG_dt.x();
    dt(55) = pG_dt.y();
    dt(56) = pG_dt.z();
    dt(57) = dG_dt.x();
    dt(58) = dG_dt.y();
    dt(59) = dG_dt.z();
    dt(60) = ddG_dt.x();
    dt(61) = ddG_dt.y();
    dt(62) = ddG_dt.z();

    dt(63) = pH_dt.x();
    dt(64) = pH_dt.y();
    dt(65) = pH_dt.z();
    dt(66) = dH_dt.x();
    dt(67) = dH_dt.y();
    dt(68) = dH_dt.z();
    dt(69) = ddH_dt.x();
    dt(70) = ddH_dt.y();
    dt(71) = ddH_dt.z();
}

ChStrainStress3D ChElementShellANCF_8::EvaluateSectionStrainStress(const ChVector<>& loc, int layer_id) {
    // Element shape function
    ShapeVector N;
    this->ShapeFunctions(N, loc.x(), loc.y(), loc.z());

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 24> Nx;
    ChMatrixNM<double, 1, 24> Ny;
    ChMatrixNM<double, 1, 24> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = this->Calc_detJ0(loc.x(), loc.y(), loc.z(), Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(1) * Ny_d0(2) - Nx_d0(2) * Ny_d0(1);
    G1xG2.y() = Nx_d0(2) * Ny_d0(0) - Nx_d0(0) * Ny_d0(2);
    G1xG2.z() = Nx_d0(0) * Ny_d0(1) - Nx_d0(1) * Ny_d0(0);
    G1dotG1 = Nx_d0(0) * Nx_d0(0) + Nx_d0(1) * Nx_d0(1) + Nx_d0(2) * Nx_d0(2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0);
    A1.y() = Nx_d0(1);
    A1.z() = Nx_d0(2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    // Direction for orthotropic material
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1;
    AA2 = A2;
    AA3 = A3;

    /// Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    ChVectorN<double, 9> beta;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0(1) * Nz_d0(2) - Nz_d0(1) * Ny_d0(2);
    j0(0, 1) = Ny_d0(2) * Nz_d0(0) - Ny_d0(0) * Nz_d0(2);
    j0(0, 2) = Ny_d0(0) * Nz_d0(1) - Nz_d0(0) * Ny_d0(1);
    j0(1, 0) = Nz_d0(1) * Nx_d0(2) - Nx_d0(1) * Nz_d0(2);
    j0(1, 1) = Nz_d0(2) * Nx_d0(0) - Nx_d0(2) * Nz_d0(0);
    j0(1, 2) = Nz_d0(0) * Nx_d0(1) - Nz_d0(1) * Nx_d0(0);
    j0(2, 0) = Nx_d0(1) * Ny_d0(2) - Ny_d0(1) * Nx_d0(2);
    j0(2, 1) = Ny_d0(0) * Nx_d0(2) - Nx_d0(0) * Ny_d0(2);
    j0(2, 2) = Nx_d0(0) * Ny_d0(1) - Ny_d0(0) * Nx_d0(1);
    j0 /= detJ0;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    // Coefficients of contravariant transformation
    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    // Transformation matrix, function of fiber angle

    ChVectorN<double, 24> ddNx = m_ddT * Nx.transpose();
    ChVectorN<double, 24> ddNy = m_ddT * Ny.transpose();
    ChVectorN<double, 24> ddNz = m_ddT * Nz.transpose();

    ChVectorN<double, 24> d0d0Nx = m_d0d0T * Nx.transpose();
    ChVectorN<double, 24> d0d0Ny = m_d0d0T * Ny.transpose();
    ChVectorN<double, 24> d0d0Nz = m_d0d0T * Nz.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);          // xy
    strain_til(3) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));  // zz
    strain_til(4) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);          // yz
    strain_til(5) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);          // xz

    // For orthotropic material
    ChVectorN<double, 6> strain;

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

    const ChMatrixNM<double, 6, 6>& E_eps = GetLayer(layer_id).GetMaterial()->Get_E_eps();
    const ChVectorN<double, 6>& stress = E_eps * strain;
    const ChStrainStress3D strainStressOut {strain, stress};

    return strainStressOut;
}

void ChElementShellANCF_8::EvaluateSectionDisplacement(const double u,
                                                       const double v,
                                                       ChVector<>& u_displ,
                                                       ChVector<>& u_rotaz) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, u_displ);
    u_rotaz = VNULL;  // no angles.. this is ANCF (or maybe return here the slope derivatives?)
}

void ChElementShellANCF_8::EvaluateSectionFrame(const double u,
                                                const double v,
                                                ChVector<>& point,
                                                ChQuaternion<>& rot) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, point);
    rot = QUNIT;  // or maybe use gram-schmidt to get csys of section from slopes?
}

void ChElementShellANCF_8::EvaluateSectionPoint(const double u, const double v, ChVector<>& point) {
    double x = u;  // because ShapeFunctions() works in -1..1 range
    double y = v;  // because ShapeFunctions() works in -1..1 range
    double z = 0;
    ShapeVector N;
    ShapeFunctions(N, x, y, z);

    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& pD = m_nodes[3]->GetPos();
    const ChVector<>& pE = m_nodes[4]->GetPos();
    const ChVector<>& pF = m_nodes[5]->GetPos();
    const ChVector<>& pG = m_nodes[6]->GetPos();
    const ChVector<>& pH = m_nodes[7]->GetPos();

    point.x() = N(0) * pA.x() + N(3) * pB.x() + N(6) * pC.x() + N(9) * pD.x() + N(12) * pE.x() + N(15) * pF.x() +
                N(18) * pG.x() + N(21) * pH.x();
    point.y() = N(0) * pA.y() + N(3) * pB.y() + N(6) * pC.y() + N(9) * pD.y() + N(12) * pE.y() + N(15) * pF.y() +
                N(18) * pG.y() + N(21) * pH.y();
    point.z() = N(0) * pA.z() + N(3) * pB.z() + N(6) * pC.z() + N(9) * pD.z() + N(12) * pE.z() + N(15) * pF.z() +
                N(18) * pG.z() + N(21) * pH.z();
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------

// Gets all the DOFs packed in a single vector (position part).
void ChElementShellANCF_8::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[0]->GetDD().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 12, 3) = m_nodes[1]->GetD().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[1]->GetDD().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[2]->GetD().eigen();
    mD.segment(block_offset + 24, 3) = m_nodes[2]->GetDD().eigen();
    mD.segment(block_offset + 27, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(block_offset + 30, 3) = m_nodes[3]->GetD().eigen();
    mD.segment(block_offset + 33, 3) = m_nodes[3]->GetDD().eigen();
    mD.segment(block_offset + 36, 3) = m_nodes[4]->GetPos().eigen();
    mD.segment(block_offset + 39, 3) = m_nodes[4]->GetD().eigen();
    mD.segment(block_offset + 42, 3) = m_nodes[4]->GetDD().eigen();
    mD.segment(block_offset + 45, 3) = m_nodes[5]->GetPos().eigen();
    mD.segment(block_offset + 48, 3) = m_nodes[5]->GetD().eigen();
    mD.segment(block_offset + 51, 3) = m_nodes[5]->GetDD().eigen();
    mD.segment(block_offset + 54, 3) = m_nodes[6]->GetPos().eigen();
    mD.segment(block_offset + 57, 3) = m_nodes[6]->GetD().eigen();
    mD.segment(block_offset + 60, 3) = m_nodes[6]->GetDD().eigen();
    mD.segment(block_offset + 63, 3) = m_nodes[7]->GetPos().eigen();
    mD.segment(block_offset + 66, 3) = m_nodes[7]->GetD().eigen();
    mD.segment(block_offset + 69, 3) = m_nodes[7]->GetDD().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChElementShellANCF_8::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD_dt().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[0]->GetDD_dt().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 12, 3) = m_nodes[1]->GetD_dt().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[1]->GetDD_dt().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[2]->GetPos_dt().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[2]->GetD_dt().eigen();
    mD.segment(block_offset + 24, 3) = m_nodes[2]->GetDD_dt().eigen();
    mD.segment(block_offset + 27, 3) = m_nodes[3]->GetPos_dt().eigen();
    mD.segment(block_offset + 30, 3) = m_nodes[3]->GetD_dt().eigen();
    mD.segment(block_offset + 33, 3) = m_nodes[3]->GetDD_dt().eigen();
    mD.segment(block_offset + 36, 3) = m_nodes[4]->GetPos_dt().eigen();
    mD.segment(block_offset + 39, 3) = m_nodes[4]->GetD_dt().eigen();
    mD.segment(block_offset + 42, 3) = m_nodes[4]->GetDD_dt().eigen();
    mD.segment(block_offset + 45, 3) = m_nodes[5]->GetPos_dt().eigen();
    mD.segment(block_offset + 48, 3) = m_nodes[5]->GetD_dt().eigen();
    mD.segment(block_offset + 51, 3) = m_nodes[5]->GetDD_dt().eigen();
    mD.segment(block_offset + 54, 3) = m_nodes[6]->GetPos_dt().eigen();
    mD.segment(block_offset + 57, 3) = m_nodes[6]->GetD_dt().eigen();
    mD.segment(block_offset + 60, 3) = m_nodes[6]->GetDD_dt().eigen();
    mD.segment(block_offset + 63, 3) = m_nodes[7]->GetPos_dt().eigen();
    mD.segment(block_offset + 66, 3) = m_nodes[7]->GetD_dt().eigen();
    mD.segment(block_offset + 69, 3) = m_nodes[7]->GetDD_dt().eigen();
}

void ChElementShellANCF_8::LoadableStateIncrement(const unsigned int off_x,
                                                  ChState& x_new,
                                                  const ChState& x,
                                                  const unsigned int off_v,
                                                  const ChStateDelta& Dv) {
    for (int i = 0; i < 8; i++) {
        this->m_nodes[i]->NodeIntStateIncrement(off_x + 9 * i, x_new, x, off_v + 9 * i, Dv);
    }
}

void ChElementShellANCF_8::EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) {
    ShapeVector N;
    ShapeFunctions(N, U, V, 0);
    for (unsigned int ii = 0; ii < m_nodes.size(); ii++) {
        Result += N(ii * 3) * m_nodes[ii]->GetPos_dt();
    }
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChElementShellANCF_8::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
        mvars.push_back(&m_nodes[i]->Variables_D());
        mvars.push_back(&m_nodes[i]->Variables_DD());
    }
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChElementShellANCF_8::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ChMatrixNM<double, 1, 24> N;
    ShapeFunctions(N, U, V, 0);

    detJ = Calc_detJ0(U, V, 0);
    detJ *= GetLengthX() * GetLengthY() / 4.0;

    for (int i = 0; i < 24; i++) {
        Qi.segment(3 * i, 3) = N(i) * F.segment(0, 3);
    }

    //// RADU: Under development; requires additional testing
    /*
    // Compute the generalized force vector for the applied moment
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ChMatrixNM<double, 24, 3> e_bar;
    ChMatrixNM<double, 3, 24> Sxi_D_transpose;
    ChMatrix33<double> J_Cxi;
    ChMatrix33<double> J_Cxi_Inv;
    ChVectorN<double, 24> G_A;
    ChVectorN<double, 24> G_B;
    ChVectorN<double, 24> G_C;
    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    ShapeFunctionsDerivativeX(Nx, U, V, 0);
    ShapeFunctionsDerivativeY(Ny, U, V, 0);
    ShapeFunctionsDerivativeZ(Nz, U, V, 0);
    Sxi_D_transpose.row(0) = Nx;
    Sxi_D_transpose.row(1) = Ny;
    Sxi_D_transpose.row(2) = Nz;

    CalcCoordMatrix(e_bar);

    // Calculate the element Jacobian between the current configuration and the normalized configuration
    J_Cxi.noalias() = e_bar.transpose() * Sxi_D_transpose.transpose();
    J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.
    G_A = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 0) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 0) +
        Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 0);
    G_B = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 1) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 1) +
        Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 1);
    G_C = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 2) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 2) +
        Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 2);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    //// MIKE Clean-up when slicing becomes available in Eigen 3.4
    for (unsigned int i = 0; i < 24; i++) {
        Qi(3 * i + 0) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi(3 * i + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi(3 * i + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }
    */
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V,W) coordinates of the surface.
void ChElementShellANCF_8::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ChMatrixNM<double, 1, 24> N;
    ShapeFunctions(N, U, V, W);

    detJ = Calc_detJ0(U, V, W);
    detJ *= m_GaussScaling;

    for (int i = 0; i < 24; i++) {
        Qi.segment(3 * i, 3) = N(i) * F.segment(0, 3);
    }

    //// RADU: Under development; requires additional testing
    /*
    // Compute the generalized force vector for the applied moment
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ChMatrixNM<double, 24, 3> e_bar;
    ChMatrixNM<double, 3, 24> Sxi_D_transpose;
    ChMatrix33<double> J_Cxi;
    ChMatrix33<double> J_Cxi_Inv;
    ChVectorN<double, 24> G_A;
    ChVectorN<double, 24> G_B;
    ChVectorN<double, 24> G_C;
    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    ShapeFunctionsDerivativeX(Nx, U, V, W);
    ShapeFunctionsDerivativeY(Ny, U, V, W);
    ShapeFunctionsDerivativeZ(Nz, U, V, W);
    Sxi_D_transpose.row(0) = Nx;
    Sxi_D_transpose.row(1) = Ny;
    Sxi_D_transpose.row(2) = Nz;

    CalcCoordMatrix(e_bar);

    // Calculate the element Jacobian between the current configuration and the normalized configuration
    J_Cxi.noalias() = e_bar.transpose() * Sxi_D_transpose.transpose();
    J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.
    G_A = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 0) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 0) +
        Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 0);
    G_B = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 1) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 1) +
        Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 1);
    G_C = Sxi_D_transpose.row(0) * J_Cxi_Inv(0, 2) + Sxi_D_transpose.row(1) * J_Cxi_Inv(1, 2) +
        Sxi_D_transpose.row(2) * J_Cxi_Inv(2, 2);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    //// MIKE Clean-up when slicing becomes available in Eigen 3.4
    for (unsigned int i = 0; i < 24; i++) {
        Qi(3 * i + 0) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi(3 * i + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi(3 * i + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }
    */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

// Calculate avergae element density (needed for ChLoaderVolumeGravity).
double ChElementShellANCF_8::GetDensity() {
    double tot_density = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        double layerthick = m_layers[kl].Get_thickness();
        tot_density += rho * layerthick;
    }
    return tot_density / m_thickness;
}

// Calculate normal to the surface at (U,V) coordinates.
ChVector<> ChElementShellANCF_8::ComputeNormal(const double U, const double V) {
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ShapeFunctionsDerivativeX(Nx, U, V, 0);
    ShapeFunctionsDerivativeY(Ny, U, V, 0);
    ShapeFunctionsDerivativeZ(Nz, U, V, 0);

    ChMatrixNM<double, 24, 3> mDD;
    CalcCoordMatrix(mDD);

    ChMatrixNM<double, 1, 3> Nx_d = Nx * mDD;
    ChMatrixNM<double, 1, 3> Ny_d = Ny * mDD;
    ChMatrixNM<double, 1, 3> Nz_d = Nz * mDD;

    ChMatrixNM<double, 3, 3> rd;
    rd(0, 0) = Nx_d(0, 0);
    rd(1, 0) = Nx_d(0, 1);
    rd(2, 0) = Nx_d(0, 2);
    rd(0, 1) = Ny_d(0, 0);
    rd(1, 1) = Ny_d(0, 1);
    rd(2, 1) = Ny_d(0, 2);
    rd(0, 2) = Nz_d(0, 0);
    rd(1, 2) = Nz_d(0, 1);
    rd(2, 2) = Nz_d(0, 2);

    ChVector<> G1xG2;
    G1xG2[0] = rd(1, 0) * rd(2, 1) - rd(2, 0) * rd(1, 1);
    G1xG2[1] = rd(2, 0) * rd(0, 1) - rd(0, 0) * rd(2, 1);
    G1xG2[2] = rd(0, 0) * rd(1, 1) - rd(1, 0) * rd(0, 1);

    double G1xG2nrm = sqrt(G1xG2[0] * G1xG2[0] + G1xG2[1] * G1xG2[1] + G1xG2[2] * G1xG2[2]);
    return G1xG2 / G1xG2nrm;
}

// ============================================================================
// Implementation of ChElementShellANCF_8::Layer methods
// ============================================================================

// Private constructor (a layer can be created only by adding it to an element)
ChElementShellANCF_8::Layer::Layer(ChElementShellANCF_8* element,
                                   double thickness,
                                   double theta,
                                   std::shared_ptr<ChMaterialShellANCF> material)
    : m_element(element), m_thickness(thickness), m_theta(theta), m_material(material) {}

// Initial setup for this layer: calculate T0 and detJ0 at the element center.
void ChElementShellANCF_8::Layer::SetupInitial() {
    // Evaluate shape functions at element center
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    m_element->ShapeFunctionsDerivativeX(Nx, 0, 0, 0);
    m_element->ShapeFunctionsDerivativeY(Ny, 0, 0, 0);
    m_element->ShapeFunctionsDerivativeZ(Nz, 0, 0, 0);

    ChMatrixNM<double, 1, 3> Nx_d0 = Nx * m_element->m_d0;
    ChMatrixNM<double, 1, 3> Ny_d0 = Ny * m_element->m_d0;
    ChMatrixNM<double, 1, 3> Nz_d0 = Nz * m_element->m_d0;

    // Determinant of position vector gradient matrix: Initial configuration
    m_detJ0C = Nx_d0(0, 0) * Ny_d0(0, 1) * Nz_d0(0, 2) + Ny_d0(0, 0) * Nz_d0(0, 1) * Nx_d0(0, 2) +
               Nz_d0(0, 0) * Nx_d0(0, 1) * Ny_d0(0, 2) - Nx_d0(0, 2) * Ny_d0(0, 1) * Nz_d0(0, 0) -
               Ny_d0(0, 2) * Nz_d0(0, 1) * Nx_d0(0, 0) - Nz_d0(0, 2) * Nx_d0(0, 1) * Ny_d0(0, 0);

    //// Transformation : Orthogonal transformation (A and J) ////
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0(1) * Ny_d0(2) - Nx_d0(2) * Ny_d0(1);
    G1xG2.y() = Nx_d0(2) * Ny_d0(0) - Nx_d0(0) * Ny_d0(2);
    G1xG2.z() = Nx_d0(0) * Ny_d0(1) - Nx_d0(1) * Ny_d0(0);
    G1dotG1 = Nx_d0(0) * Nx_d0(0) + Nx_d0(1) * Nx_d0(1) + Nx_d0(2) * Nx_d0(2);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0(0);
    A1.y() = Nx_d0(1);
    A1.z() = Nx_d0(2);
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(m_theta) + A2 * sin(m_theta);
    AA2 = -A1 * sin(m_theta) + A2 * cos(m_theta);
    AA3 = A3;

    ////Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    ChVectorN<double, 9> beta;

    j0(0, 0) = Ny_d0(1) * Nz_d0(2) - Nz_d0(1) * Ny_d0(2);
    j0(0, 1) = Ny_d0(2) * Nz_d0(0) - Ny_d0(0) * Nz_d0(2);
    j0(0, 2) = Ny_d0(0) * Nz_d0(1) - Nz_d0(0) * Ny_d0(1);
    j0(1, 0) = Nz_d0(1) * Nx_d0(2) - Nx_d0(1) * Nz_d0(2);
    j0(1, 1) = Nz_d0(2) * Nx_d0(0) - Nx_d0(2) * Nz_d0(0);
    j0(1, 2) = Nz_d0(0) * Nx_d0(1) - Nz_d0(1) * Nx_d0(0);
    j0(2, 0) = Nx_d0(1) * Ny_d0(2) - Ny_d0(1) * Nx_d0(2);
    j0(2, 1) = Ny_d0(0) * Nx_d0(2) - Nx_d0(0) * Ny_d0(2);
    j0(2, 2) = Nx_d0(0) * Ny_d0(1) - Ny_d0(0) * Nx_d0(1);
    j0 /= m_detJ0C;

    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);

    beta(0) = Vdot(AA1, j01);
    beta(1) = Vdot(AA2, j01);
    beta(2) = Vdot(AA3, j01);
    beta(3) = Vdot(AA1, j02);
    beta(4) = Vdot(AA2, j02);
    beta(5) = Vdot(AA3, j02);
    beta(6) = Vdot(AA1, j03);
    beta(7) = Vdot(AA2, j03);
    beta(8) = Vdot(AA3, j03);

    // Calculate T0: transformation matrix, function of fiber angle (see Yamashita et al, 2015, JCND)
    m_T0(0, 0) = pow(beta(0), 2);
    m_T0(1, 0) = pow(beta(1), 2);
    m_T0(2, 0) = 2.0 * beta(0) * beta(1);
    m_T0(3, 0) = pow(beta(2), 2);
    m_T0(4, 0) = 2.0 * beta(0) * beta(2);
    m_T0(5, 0) = 2.0 * beta(1) * beta(2);

    m_T0(0, 1) = pow(beta(3), 2);
    m_T0(1, 1) = pow(beta(4), 2);
    m_T0(2, 1) = 2.0 * beta(3) * beta(4);
    m_T0(3, 1) = pow(beta(5), 2);
    m_T0(4, 1) = 2.0 * beta(3) * beta(5);
    m_T0(5, 1) = 2.0 * beta(4) * beta(5);

    m_T0(0, 2) = beta(0) * beta(3);
    m_T0(1, 2) = beta(1) * beta(4);
    m_T0(2, 2) = beta(0) * beta(4) + beta(1) * beta(3);
    m_T0(3, 2) = beta(2) * beta(5);
    m_T0(4, 2) = beta(0) * beta(5) + beta(2) * beta(3);
    m_T0(5, 2) = beta(2) * beta(4) + beta(1) * beta(5);

    m_T0(0, 3) = pow(beta(6), 2);
    m_T0(1, 3) = pow(beta(7), 2);
    m_T0(2, 3) = 2.0 * beta(6) * beta(7);
    m_T0(3, 3) = pow(beta(8), 2);
    m_T0(4, 3) = 2.0 * beta(6) * beta(8);
    m_T0(5, 3) = 2.0 * beta(7) * beta(8);

    m_T0(0, 4) = beta(0) * beta(6);
    m_T0(1, 4) = beta(1) * beta(7);
    m_T0(2, 4) = beta(0) * beta(7) + beta(6) * beta(1);
    m_T0(3, 4) = beta(2) * beta(8);
    m_T0(4, 4) = beta(0) * beta(8) + beta(2) * beta(6);
    m_T0(5, 4) = beta(1) * beta(8) + beta(2) * beta(7);

    m_T0(0, 5) = beta(3) * beta(6);
    m_T0(1, 5) = beta(4) * beta(7);
    m_T0(2, 5) = beta(3) * beta(7) + beta(4) * beta(6);
    m_T0(3, 5) = beta(5) * beta(8);
    m_T0(4, 5) = beta(3) * beta(8) + beta(6) * beta(5);
    m_T0(5, 5) = beta(4) * beta(8) + beta(5) * beta(7);
}

}  // end of namespace fea
}  // end of namespace chrono
