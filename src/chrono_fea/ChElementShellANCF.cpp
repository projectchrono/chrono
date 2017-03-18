// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bryan Peterson, Milad Rakhsha, Antonio Recuero, Radu Serban
// =============================================================================
// ANCF laminated shell element with four nodes.
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChUtilsFEA.h"
#include <cmath>

namespace chrono {
namespace fea {

// ------------------------------------------------------------------------------
// Static variables
// ------------------------------------------------------------------------------
const double ChElementShellANCF::m_toleranceEAS = 1e-5;
const int ChElementShellANCF::m_maxIterationsEAS = 100;

// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementShellANCF::ChElementShellANCF()
    : m_gravity_on(false), m_numLayers(0), m_thickness(0), m_lenX(0), m_lenY(0), m_Alpha(0) {
    m_nodes.resize(4);
}

// ------------------------------------------------------------------------------
// Set element nodes
// ------------------------------------------------------------------------------

void ChElementShellANCF::SetNodes(std::shared_ptr<ChNodeFEAxyzD> nodeA,
                                  std::shared_ptr<ChNodeFEAxyzD> nodeB,
                                  std::shared_ptr<ChNodeFEAxyzD> nodeC,
                                  std::shared_ptr<ChNodeFEAxyzD> nodeD) {
    assert(nodeA);
    assert(nodeB);
    assert(nodeC);
    assert(nodeD);

    m_nodes[0] = nodeA;
    m_nodes[1] = nodeB;
    m_nodes[2] = nodeC;
    m_nodes[3] = nodeD;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&m_nodes[0]->Variables());
    mvars.push_back(&m_nodes[0]->Variables_D());
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[1]->Variables_D());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[2]->Variables_D());
    mvars.push_back(&m_nodes[3]->Variables());
    mvars.push_back(&m_nodes[3]->Variables_D());
    Kmatr.SetVariables(mvars);

    // Initial positions and slopes of the element nodes
    CalcCoordMatrix(m_d0);
    m_d0d0T.MatrMultiplyT(m_d0, m_d0);
}

// -----------------------------------------------------------------------------
// Add a layer.
// -----------------------------------------------------------------------------

void ChElementShellANCF::AddLayer(double thickness, double theta, std::shared_ptr<ChMaterialShellANCF> material) {
    m_layers.push_back(Layer(this, thickness, theta, material));
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellANCF::SetupInitial(ChSystem* system) {
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

    // Reserve space for the EAS parameters and Jacobians.
    m_alphaEAS.resize(m_numLayers);
    m_KalphaEAS.resize(m_numLayers);

    // Cache the scaling factor (due to change of integration intervals)
    m_GaussScaling = (m_lenX * m_lenY * m_thickness) / 8;

    // Compute mass matrix and gravitational forces (constant)
    ComputeMassMatrix();
    ComputeGravityForce(system->Get_G_acc());
}

// State update.
void ChElementShellANCF::Update() {
    ChElementGeneric::Update();
}

// Fill the D vector with the current field values at the element nodes.
void ChElementShellANCF::GetStateBlock(ChMatrixDynamic<>& mD) {
    mD.PasteVector(m_nodes[0]->GetPos(), 0, 0);
    mD.PasteVector(m_nodes[0]->GetD(), 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos(), 6, 0);
    mD.PasteVector(m_nodes[1]->GetD(), 9, 0);
    mD.PasteVector(m_nodes[2]->GetPos(), 12, 0);
    mD.PasteVector(m_nodes[2]->GetD(), 15, 0);
    mD.PasteVector(m_nodes[3]->GetPos(), 18, 0);
    mD.PasteVector(m_nodes[3]->GetD(), 21, 0);
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
void ChElementShellANCF::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 24) && (H.GetColumns() == 24));

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 24; i++)
        for (int j = 0; j < 24; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// Return the mass matrix.
void ChElementShellANCF::ComputeMmatrixGlobal(ChMatrix<>& M) {
    M = m_MassMatrix;
}

// -----------------------------------------------------------------------------
// Mass matrix calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the inertia matrix.
class MyMass : public ChIntegrable3D<ChMatrixNM<double, 24, 24> > {
  public:
    MyMass(ChElementShellANCF* element) : m_element(element) {}
    ~MyMass() {}

  private:
    ChElementShellANCF* m_element;

    virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z) override;
};

void MyMass::Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 8> N;
    m_element->ShapeFunctions(N, x, y, z);

    // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)]
    ChMatrixNM<double, 3, 24> S;
    ChMatrix33<> Si;
    Si.FillDiag(N(0));
    S.PasteMatrix(Si, 0, 0);
    Si.FillDiag(N(1));
    S.PasteMatrix(Si, 0, 3);
    Si.FillDiag(N(2));
    S.PasteMatrix(Si, 0, 6);
    Si.FillDiag(N(3));
    S.PasteMatrix(Si, 0, 9);
    Si.FillDiag(N(4));
    S.PasteMatrix(Si, 0, 12);
    Si.FillDiag(N(5));
    S.PasteMatrix(Si, 0, 15);
    Si.FillDiag(N(6));
    S.PasteMatrix(Si, 0, 18);
    Si.FillDiag(N(7));
    S.PasteMatrix(Si, 0, 21);

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    // perform  r = S'*S
    result.MatrTMultiply(S, S);

    // multiply integration weights
    result *= detJ0 * (m_element->m_GaussScaling);
};

void ChElementShellANCF::ComputeMassMatrix() {
    m_MassMatrix.Reset();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        MyMass myformula(this);
        ChMatrixNM<double, 24, 24> TempMassMatrix;

        ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 24> >(TempMassMatrix,  // result of integration will go there
                                                               myformula,       // formula to integrate
                                                               -1, 1,           // x limits
                                                               -1, 1,           // y limits
                                                               m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                               2                                // order of integration
                                                               );
        TempMassMatrix *= rho;
        m_MassMatrix += TempMassMatrix;
    }
}
/// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass
void ChElementShellANCF::ComputeNodalMass() {
    m_nodes[0]->m_TotalMass += m_MassMatrix(0, 0) + m_MassMatrix(0, 6) + m_MassMatrix(0, 12) + m_MassMatrix(0, 18);
    m_nodes[1]->m_TotalMass += m_MassMatrix(6, 6) + m_MassMatrix(6, 0) + m_MassMatrix(6, 12) + m_MassMatrix(6, 18);
    m_nodes[2]->m_TotalMass += m_MassMatrix(12, 12) + m_MassMatrix(12, 0) + m_MassMatrix(12, 6) + m_MassMatrix(12, 18);
    m_nodes[3]->m_TotalMass += m_MassMatrix(18, 18) + m_MassMatrix(18, 0) + m_MassMatrix(18, 6) + m_MassMatrix(18, 12);
}
// -----------------------------------------------------------------------------
// Gravitational force calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the element gravity forces
class MyGravity : public ChIntegrable3D<ChMatrixNM<double, 24, 1> > {
  public:
    MyGravity(ChElementShellANCF* element, const ChVector<> gacc) : m_element(element), m_gacc(gacc) {}
    ~MyGravity() {}

  private:
    ChElementShellANCF* m_element;
    ChVector<> m_gacc;

    virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y, const double z) override;
};

void MyGravity::Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 8> N;
    m_element->ShapeFunctions(N, x, y, z);

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    for (int i = 0; i < 8; i++) {
        result(i * 3 + 0, 0) = N(0, i) * m_gacc.x();
        result(i * 3 + 1, 0) = N(0, i) * m_gacc.y();
        result(i * 3 + 2, 0) = N(0, i) * m_gacc.z();
    }

    result *= detJ0 * m_element->m_GaussScaling;
};

void ChElementShellANCF::ComputeGravityForce(const ChVector<>& g_acc) {
    m_GravForce.Reset();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        MyGravity myformula(this, g_acc);
        ChMatrixNM<double, 24, 1> Fgravity;

        ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 1> >(Fgravity,   // result of integration will go there
                                                              myformula,  // formula to integrate
                                                              -1, 1,      // x limits
                                                              -1, 1,      // y limits
                                                              m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                              2                                // order of integration
                                                              );

        Fgravity *= rho;
        m_GravForce += Fgravity;
    }
}

// -----------------------------------------------------------------------------
// Elastic force calculation
// -----------------------------------------------------------------------------

// The class MyForce provides the integrand for the calculation of the internal forces
// for one layer of an ANCF shell element.
// The first 24 entries in the integrand represent the internal force.
// The next 5 entries represent the residual of the EAS nonlinear system.
// The last 25 entries represent the 5x5 Jacobian of the EAS nonlinear system.
// Capabilities of this class include: application of enhanced assumed strain (EAS) and
// assumed natural strain (ANS) formulations to avoid thickness and (tranvese and in-plane)
// shear locking. This implementation also features a composite material implementation
// that allows for selecting a number of layers over the element thickness; each of which
// has an independent, user-selected fiber angle (direction for orthotropic constitutive behavior)
class MyForce : public ChIntegrable3D<ChMatrixNM<double, 54, 1> > {
  public:
    MyForce(ChElementShellANCF* element,         // Containing element
            size_t kl,                           // Current layer index
            ChMatrixNM<double, 5, 1>* alpha_eas  // Vector of internal parameters for EAS formulation
            )
        : m_element(element), m_kl(kl), m_alpha_eas(alpha_eas) {}
    ~MyForce() {}

  private:
    ChElementShellANCF* m_element;
    size_t m_kl;
    ChMatrixNM<double, 5, 1>* m_alpha_eas;

    /// Evaluate (strainD'*strain)  at point x, include ANS and EAS.
    virtual void Evaluate(ChMatrixNM<double, 54, 1>& result, const double x, const double y, const double z) override;
};

void MyForce::Evaluate(ChMatrixNM<double, 54, 1>& result, const double x, const double y, const double z) {
    // Element shape function
    ChMatrixNM<double, 1, 8> N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // ANS shape function
    ChMatrixNM<double, 1, 4> S_ANS;  // Shape function vector for Assumed Natural Strain
    ChMatrixNM<double, 6, 5> M;      // Shape function vector for Enhanced Assumed Strain
    m_element->ShapeFunctionANSbilinearShell(S_ANS, x, y);
    m_element->Basis_M(M, x, y, z);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0[0][1] * Ny_d0[0][2] - Nx_d0[0][2] * Ny_d0[0][1];
    G1xG2.y() = Nx_d0[0][2] * Ny_d0[0][0] - Nx_d0[0][0] * Ny_d0[0][2];
    G1xG2.z() = Nx_d0[0][0] * Ny_d0[0][1] - Nx_d0[0][1] * Ny_d0[0][0];
    G1dotG1 = Nx_d0[0][0] * Nx_d0[0][0] + Nx_d0[0][1] * Nx_d0[0][1] + Nx_d0[0][2] * Nx_d0[0][2];

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0[0][0];
    A1.y() = Nx_d0[0][1];
    A1.z() = Nx_d0[0][2];
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
    ChMatrixNM<double, 9, 1> beta;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0[0][1] * Nz_d0[0][2] - Nz_d0[0][1] * Ny_d0[0][2];
    j0(0, 1) = Ny_d0[0][2] * Nz_d0[0][0] - Ny_d0[0][0] * Nz_d0[0][2];
    j0(0, 2) = Ny_d0[0][0] * Nz_d0[0][1] - Nz_d0[0][0] * Ny_d0[0][1];
    j0(1, 0) = Nz_d0[0][1] * Nx_d0[0][2] - Nx_d0[0][1] * Nz_d0[0][2];
    j0(1, 1) = Nz_d0[0][2] * Nx_d0[0][0] - Nx_d0[0][2] * Nz_d0[0][0];
    j0(1, 2) = Nz_d0[0][0] * Nx_d0[0][1] - Nz_d0[0][1] * Nx_d0[0][0];
    j0(2, 0) = Nx_d0[0][1] * Ny_d0[0][2] - Ny_d0[0][1] * Nx_d0[0][2];
    j0(2, 1) = Ny_d0[0][0] * Nx_d0[0][2] - Nx_d0[0][0] * Ny_d0[0][2];
    j0(2, 2) = Nx_d0[0][0] * Ny_d0[0][1] - Ny_d0[0][0] * Nx_d0[0][1];
    j0.MatrDivScale(detJ0);

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
    beta(0, 0) = Vdot(AA1, j01);
    beta(1, 0) = Vdot(AA2, j01);
    beta(2, 0) = Vdot(AA3, j01);
    beta(3, 0) = Vdot(AA1, j02);
    beta(4, 0) = Vdot(AA2, j02);
    beta(5, 0) = Vdot(AA3, j02);
    beta(6, 0) = Vdot(AA1, j03);
    beta(7, 0) = Vdot(AA2, j03);
    beta(8, 0) = Vdot(AA3, j03);

    // Transformation matrix, function of fiber angle
    const ChMatrixNM<double, 6, 6>& T0 = m_element->GetLayer(m_kl).Get_T0();
    // Determinant of the initial position vector gradient at the element center
    double detJ0C = m_element->GetLayer(m_kl).Get_detJ0C();

    // Enhanced Assumed Strain
    ChMatrixNM<double, 6, 5> G = T0 * M * (detJ0C / detJ0);
    ChMatrixNM<double, 6, 1> strain_EAS = G * (*m_alpha_eas);

    ChMatrixNM<double, 8, 1> ddNx;
    ChMatrixNM<double, 8, 1> ddNy;
    ChMatrixNM<double, 8, 1> ddNz;
    ddNx.MatrMultiplyT(m_element->m_ddT, Nx);
    ddNy.MatrMultiplyT(m_element->m_ddT, Ny);
    ddNz.MatrMultiplyT(m_element->m_ddT, Nz);

    ChMatrixNM<double, 8, 1> d0d0Nx;
    ChMatrixNM<double, 8, 1> d0d0Ny;
    ChMatrixNM<double, 8, 1> d0d0Nz;
    d0d0Nx.MatrMultiplyT(m_element->m_d0d0T, Nx);
    d0d0Ny.MatrMultiplyT(m_element->m_d0d0T, Ny);
    d0d0Nz.MatrMultiplyT(m_element->m_d0d0T, Nz);

    // Strain component
    ChMatrixNM<double, 6, 1> strain_til;
    strain_til(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
    strain_til(3, 0) = N(0, 0) * m_element->m_strainANS(0, 0) + N(0, 2) * m_element->m_strainANS(1, 0) +
                       N(0, 4) * m_element->m_strainANS(2, 0) + N(0, 6) * m_element->m_strainANS(3, 0);
    strain_til(4, 0) = S_ANS(0, 2) * m_element->m_strainANS(6, 0) + S_ANS(0, 3) * m_element->m_strainANS(7, 0);
    strain_til(5, 0) = S_ANS(0, 0) * m_element->m_strainANS(4, 0) + S_ANS(0, 1) * m_element->m_strainANS(5, 0);

    // For orthotropic material
    ChMatrixNM<double, 6, 1> strain;

    strain(0, 0) = strain_til(0, 0) * beta(0) * beta(0) + strain_til(1, 0) * beta(3) * beta(3) +
                   strain_til(2, 0) * beta(0) * beta(3) + strain_til(3, 0) * beta(6) * beta(6) +
                   strain_til(4, 0) * beta(0) * beta(6) + strain_til(5, 0) * beta(3) * beta(6);
    strain(1, 0) = strain_til(0, 0) * beta(1) * beta(1) + strain_til(1, 0) * beta(4) * beta(4) +
                   strain_til(2, 0) * beta(1) * beta(4) + strain_til(3, 0) * beta(7) * beta(7) +
                   strain_til(4, 0) * beta(1) * beta(7) + strain_til(5, 0) * beta(4) * beta(7);
    strain(2, 0) = strain_til(0, 0) * 2.0 * beta(0) * beta(1) + strain_til(1, 0) * 2.0 * beta(3) * beta(4) +
                   strain_til(2, 0) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                   strain_til(3, 0) * 2.0 * beta(6) * beta(7) +
                   strain_til(4, 0) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                   strain_til(5, 0) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3, 0) = strain_til(0, 0) * beta(2) * beta(2) + strain_til(1, 0) * beta(5) * beta(5) +
                   strain_til(2, 0) * beta(2) * beta(5) + strain_til(3, 0) * beta(8) * beta(8) +
                   strain_til(4, 0) * beta(2) * beta(8) + strain_til(5, 0) * beta(5) * beta(8);
    strain(4, 0) = strain_til(0, 0) * 2.0 * beta(0) * beta(2) + strain_til(1, 0) * 2.0 * beta(3) * beta(5) +
                   strain_til(2, 0) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                   strain_til(3, 0) * 2.0 * beta(6) * beta(8) +
                   strain_til(4, 0) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                   strain_til(5, 0) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5, 0) = strain_til(0, 0) * 2.0 * beta(1) * beta(2) + strain_til(1, 0) * 2.0 * beta(4) * beta(5) +
                   strain_til(2, 0) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                   strain_til(3, 0) * 2.0 * beta(7) * beta(8) +
                   strain_til(4, 0) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                   strain_til(5, 0) * (beta(5) * beta(7) + beta(4) * beta(8));

    // Strain derivative component

    ChMatrixNM<double, 6, 24> strainD_til;
    ChMatrixNM<double, 1, 24> tempB;
    ChMatrixNM<double, 1, 24> tempBB;
    ChMatrixNM<double, 1, 3> tempB3;
    ChMatrixNM<double, 1, 3> tempB31;
    strainD_til.Reset();
    tempB3.MatrMultiply(Nx, m_element->m_d);
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 0, 0);
    tempB3.MatrMultiply(Ny, m_element->m_d);
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 1, 0);
    tempB31.MatrMultiply(Nx, m_element->m_d);
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i) + tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 2, 0);

    tempBB.Reset();
    for (int i = 0; i < 4; i++) {
        int ij = i * 2;
        tempB.PasteClippedMatrix(m_element->m_strainANS_D, i, 0, 1, 24, 0, 0);
        tempB *= N(0, ij);
        tempBB += tempB;
    }
    strainD_til.PasteClippedMatrix(tempBB, 0, 0, 1, 24, 3, 0);  // strainD for zz
    //
    tempBB.Reset();
    for (int i = 0; i < 2; i++) {
        int ij = i + 6;
        int ij1 = i + 2;
        tempB.PasteClippedMatrix(m_element->m_strainANS_D, ij, 0, 1, 24, 0, 0);
        tempB *= S_ANS(0, ij1);
        tempBB += tempB;
    }
    strainD_til.PasteClippedMatrix(tempBB, 0, 0, 1, 24, 4, 0);  // strainD for xz
    //
    tempBB.Reset();
    for (int i = 0; i < 2; i++) {
        int ij = i + 4;
        int ij1 = i;
        tempB.PasteClippedMatrix(m_element->m_strainANS_D, ij, 0, 1, 24, 0, 0);
        tempB *= S_ANS(0, ij1);
        tempBB += tempB;
    }
    strainD_til.PasteClippedMatrix(tempBB, 0, 0, 1, 24, 5, 0);  // strainD for yz

    // For orthotropic material
    ChMatrixNM<double, 6, 24> strainD;  // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
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

    // Enhanced Assumed Strain 2nd
    strain += strain_EAS;

    // Strain time derivative for structural damping
    ChMatrixNM<double, 6, 1> DEPS;
    DEPS.Reset();
    for (int ii = 0; ii < 24; ii++) {
        DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
        DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
        DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
        DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
        DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
        DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
    }

    // Add structural damping
    strain += DEPS * m_element->m_Alpha;

    // Matrix of elastic coefficients: the input assumes the material *could* be orthotropic
    const ChMatrixNM<double, 6, 6>& E_eps = m_element->GetLayer(m_kl).GetMaterial()->Get_E_eps();

    // Internal force calculation
    ChMatrixNM<double, 24, 6> tempC;
    tempC.MatrTMultiply(strainD, E_eps);
    ChMatrixNM<double, 24, 1> Fint = (tempC * strain) * (detJ0 * m_element->m_GaussScaling);

    // EAS terms
    ChMatrixNM<double, 5, 6> temp56;
    temp56.MatrTMultiply(G, E_eps);
    ChMatrixNM<double, 5, 1> HE = (temp56 * strain) * (detJ0 * m_element->m_GaussScaling);  // EAS residual
    ChMatrixNM<double, 5, 5> KALPHA = (temp56 * G) * (detJ0 * m_element->m_GaussScaling);   // EAS Jacobian

    /// Total result vector
    result.PasteClippedMatrix(Fint, 0, 0, 24, 1, 0, 0);
    result.PasteClippedMatrix(HE, 0, 0, 5, 1, 24, 0);
    result.PasteClippedMatrixToVector(KALPHA, 0, 0, 5, 5, 29);
}

void ChElementShellANCF::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    // Current nodal coordinates and velocities
    CalcCoordMatrix(m_d);
    CalcCoordDerivMatrix(m_d_dt);
    m_ddT.MatrMultiplyT(m_d, m_d);
    // Assumed Natural Strain (ANS):  Calculate m_strainANS and m_strainANS_D
    CalcStrainANSbilinearShell();

    Fi.Reset();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        ChMatrixNM<double, 24, 1> Finternal;
        ChMatrixNM<double, 5, 1> HE;
        ChMatrixNM<double, 5, 5> KALPHA;

        // Initial guess for EAS parameters
        ChMatrixNM<double, 5, 1> alphaEAS = m_alphaEAS[kl];

        // Newton loop for EAS
        for (int count = 0; count < m_maxIterationsEAS; count++) {
            ChMatrixNM<double, 54, 1> result;
            MyForce formula(this, kl, &alphaEAS);
            ChQuadrature::Integrate3D<ChMatrixNM<double, 54, 1> >(result,   // result of integration
                                                                  formula,  // integrand formula
                                                                  -1, 1,    // x limits
                                                                  -1, 1,    // y limits
                                                                  m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                                  2  // order of integration
                                                                  );

            // Extract vectors and matrices from result of integration
            Finternal.PasteClippedMatrix(result, 0, 0, 24, 1, 0, 0);
            HE.PasteClippedMatrix(result, 24, 0, 5, 1, 0, 0);
            KALPHA.PasteClippedVectorToMatrix(result, 0, 0, 5, 5, 29);

            // Check convergence (residual check)
            double norm_HE = HE.NormTwo();
            if (norm_HE < m_toleranceEAS)
                break;

            // Calculate increment (in place) and update EAS parameters
            ChMatrixNM<int, 5, 1> INDX;
            bool pivoting;
            ChMatrixNM<double, 5, 5> KALPHA1 = KALPHA;
            if (!LU_factor(KALPHA1, INDX, pivoting))
                throw ChException("Singular matrix in LU factorization");
            LU_solve(KALPHA1, INDX, HE);
            alphaEAS = alphaEAS - HE;

            if (count >= 2)
                GetLog() << "  count " << count << "  NormHE " << norm_HE << "\n";
        }

        // Accumulate internal force
        Fi -= Finternal;

        // Cache alphaEAS and KALPHA for use in Jacobian calculation
        m_alphaEAS[kl] = alphaEAS;
        m_KalphaEAS[kl] = KALPHA;

    }  // Layer Loop

    if (m_gravity_on) {
        Fi += m_GravForce;
    }
}

// -----------------------------------------------------------------------------
// Jacobians of internal forces
// -----------------------------------------------------------------------------

// The class MyJacobian provides the integrand for the calculation of the Jacobians
// (stiffness and damping matrices) of the internal forces for one layer of an ANCF
// shell element.
// The first 576 entries in the integrated vector represent the 24x24 Jacobian
//      Kfactor * [K] + Rfactor * [R]
// where K does not include the EAS contribution.
// The last 120 entries represent the 5x24 cross-dependency matrix.
class MyJacobian : public ChIntegrable3D<ChMatrixNM<double, 696, 1> > {
  public:
    MyJacobian(ChElementShellANCF* element,  // Containing element
               double Kfactor,               // Scaling coefficient for stiffness component
               double Rfactor,               // Scaling coefficient for damping component
               size_t kl                     // Current layer index
               )
        : m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor), m_kl(kl) {}

  private:
    ChElementShellANCF* m_element;
    double m_Kfactor;
    double m_Rfactor;
    size_t m_kl;

    // Evaluate integrand at the specified point.
    virtual void Evaluate(ChMatrixNM<double, 696, 1>& result, const double x, const double y, const double z) override;
};

void MyJacobian::Evaluate(ChMatrixNM<double, 696, 1>& result, const double x, const double y, const double z) {
    // Element shape function
    ChMatrixNM<double, 1, 8> N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = m_element->Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // ANS shape function
    ChMatrixNM<double, 1, 4> S_ANS;  // Shape function vector for Assumed Natural Strain
    ChMatrixNM<double, 6, 5> M;      // Shape function vector for Enhanced Assumed Strain
    m_element->ShapeFunctionANSbilinearShell(S_ANS, x, y);
    m_element->Basis_M(M, x, y, z);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0[0][1] * Ny_d0[0][2] - Nx_d0[0][2] * Ny_d0[0][1];
    G1xG2.y() = Nx_d0[0][2] * Ny_d0[0][0] - Nx_d0[0][0] * Ny_d0[0][2];
    G1xG2.z() = Nx_d0[0][0] * Ny_d0[0][1] - Nx_d0[0][1] * Ny_d0[0][0];
    G1dotG1 = Nx_d0[0][0] * Nx_d0[0][0] + Nx_d0[0][1] * Nx_d0[0][1] + Nx_d0[0][2] * Nx_d0[0][2];

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0[0][0];
    A1.y() = Nx_d0[0][1];
    A1.z() = Nx_d0[0][2];
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
    ChMatrixNM<double, 9, 1> beta;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0[0][1] * Nz_d0[0][2] - Nz_d0[0][1] * Ny_d0[0][2];
    j0(0, 1) = Ny_d0[0][2] * Nz_d0[0][0] - Ny_d0[0][0] * Nz_d0[0][2];
    j0(0, 2) = Ny_d0[0][0] * Nz_d0[0][1] - Nz_d0[0][0] * Ny_d0[0][1];
    j0(1, 0) = Nz_d0[0][1] * Nx_d0[0][2] - Nx_d0[0][1] * Nz_d0[0][2];
    j0(1, 1) = Nz_d0[0][2] * Nx_d0[0][0] - Nx_d0[0][2] * Nz_d0[0][0];
    j0(1, 2) = Nz_d0[0][0] * Nx_d0[0][1] - Nz_d0[0][1] * Nx_d0[0][0];
    j0(2, 0) = Nx_d0[0][1] * Ny_d0[0][2] - Ny_d0[0][1] * Nx_d0[0][2];
    j0(2, 1) = Ny_d0[0][0] * Nx_d0[0][2] - Nx_d0[0][0] * Ny_d0[0][2];
    j0(2, 2) = Nx_d0[0][0] * Ny_d0[0][1] - Ny_d0[0][0] * Nx_d0[0][1];
    j0.MatrDivScale(detJ0);

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
    beta(0, 0) = Vdot(AA1, j01);
    beta(1, 0) = Vdot(AA2, j01);
    beta(2, 0) = Vdot(AA3, j01);
    beta(3, 0) = Vdot(AA1, j02);
    beta(4, 0) = Vdot(AA2, j02);
    beta(5, 0) = Vdot(AA3, j02);
    beta(6, 0) = Vdot(AA1, j03);
    beta(7, 0) = Vdot(AA2, j03);
    beta(8, 0) = Vdot(AA3, j03);

    // Transformation matrix, function of fiber angle
    const ChMatrixNM<double, 6, 6>& T0 = m_element->GetLayer(m_kl).Get_T0();
    // Determinant of the initial position vector gradient at the element center
    double detJ0C = m_element->GetLayer(m_kl).Get_detJ0C();

// Enhanced Assumed Strain
#ifdef CHRONO_HAS_AVX
    // Referring to test_AVX, expected speed up Vs. non-AVX operation: 1.7x
    ChMatrixNM<double, 6, 5> G;
    G.MatrMultiplyAVX(T0, M);
    G = G * (detJ0C / detJ0);
#else
    ChMatrixNM<double, 6, 5> G = T0 * M * (detJ0C / detJ0);
#endif

    ChMatrixNM<double, 6, 1> strain_EAS = G * m_element->m_alphaEAS[m_kl];
    ChMatrixNM<double, 8, 1> ddNx;
    ChMatrixNM<double, 8, 1> ddNy;
    ChMatrixNM<double, 8, 1> ddNz;
    ddNx.MatrMultiplyT(m_element->m_ddT, Nx);
    ddNy.MatrMultiplyT(m_element->m_ddT, Ny);
    ddNz.MatrMultiplyT(m_element->m_ddT, Nz);

    ChMatrixNM<double, 8, 1> d0d0Nx;
    ChMatrixNM<double, 8, 1> d0d0Ny;
    ChMatrixNM<double, 8, 1> d0d0Nz;
    d0d0Nx.MatrMultiplyT(m_element->m_d0d0T, Nx);
    d0d0Ny.MatrMultiplyT(m_element->m_d0d0T, Ny);
    d0d0Nz.MatrMultiplyT(m_element->m_d0d0T, Nz);

    // Strain component
    ChMatrixNM<double, 6, 1> strain_til;
    strain_til(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
    strain_til(3, 0) = N(0, 0) * m_element->m_strainANS(0, 0) + N(0, 2) * m_element->m_strainANS(1, 0) +
                       N(0, 4) * m_element->m_strainANS(2, 0) + N(0, 6) * m_element->m_strainANS(3, 0);
    strain_til(4, 0) = S_ANS(0, 2) * m_element->m_strainANS(6, 0) + S_ANS(0, 3) * m_element->m_strainANS(7, 0);
    strain_til(5, 0) = S_ANS(0, 0) * m_element->m_strainANS(4, 0) + S_ANS(0, 1) * m_element->m_strainANS(5, 0);

    // For orthotropic material
    ChMatrixNM<double, 6, 1> strain;

    strain(0, 0) = strain_til(0, 0) * beta(0) * beta(0) + strain_til(1, 0) * beta(3) * beta(3) +
                   strain_til(2, 0) * beta(0) * beta(3) + strain_til(3, 0) * beta(6) * beta(6) +
                   strain_til(4, 0) * beta(0) * beta(6) + strain_til(5, 0) * beta(3) * beta(6);
    strain(1, 0) = strain_til(0, 0) * beta(1) * beta(1) + strain_til(1, 0) * beta(4) * beta(4) +
                   strain_til(2, 0) * beta(1) * beta(4) + strain_til(3, 0) * beta(7) * beta(7) +
                   strain_til(4, 0) * beta(1) * beta(7) + strain_til(5, 0) * beta(4) * beta(7);
    strain(2, 0) = strain_til(0, 0) * 2.0 * beta(0) * beta(1) + strain_til(1, 0) * 2.0 * beta(3) * beta(4) +
                   strain_til(2, 0) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                   strain_til(3, 0) * 2.0 * beta(6) * beta(7) +
                   strain_til(4, 0) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                   strain_til(5, 0) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3, 0) = strain_til(0, 0) * beta(2) * beta(2) + strain_til(1, 0) * beta(5) * beta(5) +
                   strain_til(2, 0) * beta(2) * beta(5) + strain_til(3, 0) * beta(8) * beta(8) +
                   strain_til(4, 0) * beta(2) * beta(8) + strain_til(5, 0) * beta(5) * beta(8);
    strain(4, 0) = strain_til(0, 0) * 2.0 * beta(0) * beta(2) + strain_til(1, 0) * 2.0 * beta(3) * beta(5) +
                   strain_til(2, 0) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                   strain_til(3, 0) * 2.0 * beta(6) * beta(8) +
                   strain_til(4, 0) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                   strain_til(5, 0) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5, 0) = strain_til(0, 0) * 2.0 * beta(1) * beta(2) + strain_til(1, 0) * 2.0 * beta(4) * beta(5) +
                   strain_til(2, 0) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                   strain_til(3, 0) * 2.0 * beta(7) * beta(8) +
                   strain_til(4, 0) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                   strain_til(5, 0) * (beta(5) * beta(7) + beta(4) * beta(8));

    // Strain derivative component

    ChMatrixNM<double, 6, 24> strainD_til;
    ChMatrixNM<double, 1, 24> tempB;
    ChMatrixNM<double, 1, 24> tempBB;
    ChMatrixNM<double, 1, 3> tempB3;
    ChMatrixNM<double, 1, 3> tempB31;
    strainD_til.Reset();
    // Expected speed up for AVX operation = 1.1x
    tempB3.MatrMultiply(Nx, m_element->m_d);
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 0, 0);
    tempB3.MatrMultiply(Ny, m_element->m_d);
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 1, 0);
    tempB31.MatrMultiply(Nx, m_element->m_d);
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i) + tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 2, 0);

    tempBB.Reset();
    for (int i = 0; i < 4; i++) {
        int ij = i * 2;
        tempB.PasteClippedMatrix(m_element->m_strainANS_D, i, 0, 1, 24, 0, 0);
        tempB *= N(0, ij);
        tempBB += tempB;
    }
    strainD_til.PasteClippedMatrix(tempBB, 0, 0, 1, 24, 3, 0);  // strainD for zz
    //
    tempBB.Reset();
    for (int i = 0; i < 2; i++) {
        int ij = i + 6;
        int ij1 = i + 2;
        tempB.PasteClippedMatrix(m_element->m_strainANS_D, ij, 0, 1, 24, 0, 0);
        tempB *= S_ANS(0, ij1);
        tempBB += tempB;
    }
    strainD_til.PasteClippedMatrix(tempBB, 0, 0, 1, 24, 4, 0);  // strainD for xz
    //
    tempBB.Reset();
    for (int i = 0; i < 2; i++) {
        int ij = i + 4;
        int ij1 = i;
        tempB.PasteClippedMatrix(m_element->m_strainANS_D, ij, 0, 1, 24, 0, 0);
        tempB *= S_ANS(0, ij1);
        tempBB += tempB;
    }
    strainD_til.PasteClippedMatrix(tempBB, 0, 0, 1, 24, 5, 0);  // strainD for yz

    //// For orthotropic material
    ChMatrixNM<double, 6, 24> strainD;  // Derivative of the strains w.r.t. the coordinates. Includes orthotropy
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

    /// Gd : Jacobian (w.r.t. coordinates) of the initial position vector gradient matrix
    ChMatrixNM<double, 9, 24> Gd;

    for (int ii = 0; ii < 8; ii++) {
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

    // Enhanced Assumed Strain 2nd
    strain += strain_EAS;

    // Structural damping
    // Strain time derivative for structural damping
    ChMatrixNM<double, 6, 1> DEPS;
    DEPS.Reset();
    for (int ii = 0; ii < 24; ii++) {
        DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * m_element->m_d_dt(ii, 0);
        DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * m_element->m_d_dt(ii, 0);
        DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * m_element->m_d_dt(ii, 0);
        DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * m_element->m_d_dt(ii, 0);
        DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * m_element->m_d_dt(ii, 0);
        DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * m_element->m_d_dt(ii, 0);
    }

    // Add structural damping
    strain += DEPS * m_element->m_Alpha;

    // Matrix of elastic coefficients: The input assumes the material *could* be orthotropic
    const ChMatrixNM<double, 6, 6>& E_eps = m_element->GetLayer(m_kl).GetMaterial()->Get_E_eps();

    // Stress tensor calculation
    ChMatrixNM<double, 6, 1> stress;
    stress.MatrMultiply(E_eps, strain);

    // Declaration and computation of Sigm, to be removed
    ChMatrixNM<double, 9, 9> Sigm;  ///< Rearrangement of stress vector (not always needed)

    Sigm(0, 0) = stress(0, 0);  // XX
    Sigm(1, 1) = stress(0, 0);
    Sigm(2, 2) = stress(0, 0);

    Sigm(0, 3) = stress(2, 0);  // XY
    Sigm(1, 4) = stress(2, 0);
    Sigm(2, 5) = stress(2, 0);

    Sigm(0, 6) = stress(4, 0);  // XZ
    Sigm(1, 7) = stress(4, 0);
    Sigm(2, 8) = stress(4, 0);

    Sigm(3, 0) = stress(2, 0);  // XY
    Sigm(4, 1) = stress(2, 0);
    Sigm(5, 2) = stress(2, 0);

    Sigm(3, 3) = stress(1, 0);  // YY
    Sigm(4, 4) = stress(1, 0);
    Sigm(5, 5) = stress(1, 0);

    Sigm(3, 6) = stress(5, 0);  // YZ
    Sigm(4, 7) = stress(5, 0);
    Sigm(5, 8) = stress(5, 0);

    Sigm(6, 0) = stress(4, 0);  // XZ
    Sigm(7, 1) = stress(4, 0);
    Sigm(8, 2) = stress(4, 0);

    Sigm(6, 3) = stress(5, 0);  // YZ
    Sigm(7, 4) = stress(5, 0);
    Sigm(8, 5) = stress(5, 0);

    Sigm(6, 6) = stress(3, 0);  // ZZ
    Sigm(7, 7) = stress(3, 0);
    Sigm(8, 8) = stress(3, 0);

    // Jacobian of internal forces (excluding the EAS contribution).
    ChMatrixNM<double, 24, 6> temp246;
    ChMatrixNM<double, 24, 9> temp249;
    temp246.MatrTMultiply(strainD, E_eps);
    temp249.MatrTMultiply(Gd, Sigm);
    ChMatrixNM<double, 24, 24> KTE;

#ifdef CHRONO_HAS_AVX
    ChMatrixNM<double, 24, 24> KTE_temp1;
    ChMatrixNM<double, 24, 24> KTE_temp2;
    KTE_temp1.MatrMultiplyAVX(temp246, strainD);
    KTE_temp2.MatrMultiplyAVX(temp249, Gd);
    KTE = KTE_temp1 * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + KTE_temp2 * m_Kfactor;
#else
    KTE = (temp246 * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + (temp249 * Gd) * m_Kfactor;
#endif

    KTE *= detJ0 * (m_element->m_GaussScaling);

    // EAS cross-dependency matrix.
    ChMatrixNM<double, 5, 6> temp56;
    temp56.MatrTMultiply(G, E_eps);

#ifdef CHRONO_HAS_AVX
    ChMatrixNM<double, 5, 24> GDEPSP;
    GDEPSP.MatrMultiplyAVX(temp56, strainD);
    GDEPSP = GDEPSP * (detJ0 * m_element->m_GaussScaling);
#else
    ChMatrixNM<double, 5, 24> GDEPSP = (temp56 * strainD) * (detJ0 * m_element->m_GaussScaling);
#endif

    // Load result vector (integrand)
    result.PasteClippedMatrixToVector(KTE, 0, 0, 24, 24, 0);
    result.PasteClippedMatrixToVector(GDEPSP, 0, 0, 5, 24, 576);
}

void ChElementShellANCF::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    // Note that the matrices with current nodal coordinates and velocities are
    // already available in m_d and m_d_dt (as set in ComputeInternalForces).
    // Similarly, the ANS strain and strain derivatives are already available in
    // m_strainANS and m_strainANS_D (as calculated in ComputeInternalForces).

    m_JacobianMatrix.Reset();

    // Loop over all layers.
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        ChMatrixNM<double, 696, 1> result;
        MyJacobian formula(this, Kfactor, Rfactor, kl);
        ChQuadrature::Integrate3D<ChMatrixNM<double, 696, 1> >(result,                          // result of integration
                                                               formula,                         // integrand formula
                                                               -1, 1,                           // x limits
                                                               -1, 1,                           // y limits
                                                               m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                               2                                // order of integration
                                                               );

        // Extract matrices from result of integration
        ChMatrixNM<double, 24, 24> KTE;
        ChMatrixNM<double, 5, 24> GDEPSP;
        KTE.PasteClippedVectorToMatrix(result, 0, 0, 24, 24, 0);
        GDEPSP.PasteClippedVectorToMatrix(result, 0, 0, 5, 24, 576);

        // Include EAS contribution to the stiffness component (hence scaled by Kfactor)
        ChMatrixNM<double, 5, 5> KalphaEAS_inv;
        Inverse55_Analytical(KalphaEAS_inv, m_KalphaEAS[kl]);
        ChMatrixNM<double, 24, 24> EAS;
        EAS.MatrTMultiply(GDEPSP, KalphaEAS_inv * GDEPSP);

        // Accumulate Jacobian
        m_JacobianMatrix += KTE - EAS * Kfactor;
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementShellANCF::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
    double c = m_thickness;

    N(0) = 0.25 * (1.0 - x) * (1.0 - y);
    N(1) = z * c / 2.0 * 0.25 * (1.0 - x) * (1.0 - y);
    N(2) = 0.25 * (1.0 + x) * (1.0 - y);
    N(3) = z * c / 2.0 * 0.25 * (1.0 + x) * (1.0 - y);
    N(4) = 0.25 * (1.0 + x) * (1.0 + y);
    N(5) = z * c / 2.0 * 0.25 * (1.0 + x) * (1.0 + y);
    N(6) = 0.25 * (1.0 - x) * (1.0 + y);
    N(7) = z * c / 2.0 * 0.25 * (1.0 - x) * (1.0 + y);
}

void ChElementShellANCF::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
    double c = m_thickness;

    Nx(0) = 0.25 * (-2.0 / a) * (1.0 - y);
    Nx(1) = z * c / 2.0 * 0.25 * (-2.0 / a) * (1.0 - y);
    Nx(2) = 0.25 * (2.0 / a) * (1.0 - y);
    Nx(3) = z * c / 2.0 * 0.25 * (2.0 / a) * (1.0 - y);
    Nx(4) = 0.25 * (2.0 / a) * (1.0 + y);
    Nx(5) = z * c / 2.0 * 0.25 * (2.0 / a) * (1.0 + y);
    Nx(6) = 0.25 * (-2.0 / a) * (1.0 + y);
    Nx(7) = z * c / 2.0 * 0.25 * (-2.0 / a) * (1.0 + y);
}

void ChElementShellANCF::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
    double c = m_thickness;

    Ny(0) = 0.25 * (1.0 - x) * (-2.0 / b);
    Ny(1) = z * c / 2.0 * 0.25 * (1.0 - x) * (-2.0 / b);
    Ny(2) = 0.25 * (1.0 + x) * (-2.0 / b);
    Ny(3) = z * c / 2.0 * 0.25 * (1.0 + x) * (-2.0 / b);
    Ny(4) = 0.25 * (1.0 + x) * (2.0 / b);
    Ny(5) = z * c / 2.0 * 0.25 * (1.0 + x) * (2.0 / b);
    Ny(6) = 0.25 * (1.0 - x) * (2.0 / b);
    Ny(7) = z * c / 2.0 * 0.25 * (1.0 - x) * (2.0 / b);
}

void ChElementShellANCF::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
    double c = m_thickness;

    Nz(0) = 0.0;
    Nz(1) = 0.250 * (1.0 - x) * (1.0 - y);
    Nz(2) = 0.0;
    Nz(3) = 0.250 * (1.0 + x) * (1.0 - y);
    Nz(4) = 0.0;
    Nz(5) = 0.250 * (1.0 + x) * (1.0 + y);
    Nz(6) = 0.0;
    Nz(7) = 0.250 * (1.0 - x) * (1.0 + y);
}

void ChElementShellANCF::Basis_M(ChMatrixNM<double, 6, 5>& M, double x, double y, double z) {
    M.Reset();
    M(0, 0) = x;
    M(1, 1) = y;
    M(2, 2) = x;
    M(2, 3) = y;
    M(3, 4) = z;
}

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------
double ChElementShellANCF::Calc_detJ0(double x,
                                      double y,
                                      double z,
                                      ChMatrixNM<double, 1, 8>& Nx,
                                      ChMatrixNM<double, 1, 8>& Ny,
                                      ChMatrixNM<double, 1, 8>& Nz,
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

double ChElementShellANCF::Calc_detJ0(double x, double y, double z) {
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;

    return Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);
}

void ChElementShellANCF::CalcCoordMatrix(ChMatrixNM<double, 8, 3>& d) {
    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& dA = m_nodes[0]->GetD();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& dB = m_nodes[1]->GetD();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& dC = m_nodes[2]->GetD();
    const ChVector<>& pD = m_nodes[3]->GetPos();
    const ChVector<>& dD = m_nodes[3]->GetD();

    d(0, 0) = pA.x();
    d(0, 1) = pA.y();
    d(0, 2) = pA.z();
    d(1, 0) = dA.x();
    d(1, 1) = dA.y();
    d(1, 2) = dA.z();

    d(2, 0) = pB.x();
    d(2, 1) = pB.y();
    d(2, 2) = pB.z();
    d(3, 0) = dB.x();
    d(3, 1) = dB.y();
    d(3, 2) = dB.z();

    d(4, 0) = pC.x();
    d(4, 1) = pC.y();
    d(4, 2) = pC.z();
    d(5, 0) = dC.x();
    d(5, 1) = dC.y();
    d(5, 2) = dC.z();

    d(6, 0) = pD.x();
    d(6, 1) = pD.y();
    d(6, 2) = pD.z();
    d(7, 0) = dD.x();
    d(7, 1) = dD.y();
    d(7, 2) = dD.z();
}

void ChElementShellANCF::CalcCoordDerivMatrix(ChMatrixNM<double, 24, 1>& dt) {
    const ChVector<>& pA_dt = m_nodes[0]->GetPos_dt();
    const ChVector<>& dA_dt = m_nodes[0]->GetD_dt();
    const ChVector<>& pB_dt = m_nodes[1]->GetPos_dt();
    const ChVector<>& dB_dt = m_nodes[1]->GetD_dt();
    const ChVector<>& pC_dt = m_nodes[2]->GetPos_dt();
    const ChVector<>& dC_dt = m_nodes[2]->GetD_dt();
    const ChVector<>& pD_dt = m_nodes[3]->GetPos_dt();
    const ChVector<>& dD_dt = m_nodes[3]->GetD_dt();

    dt(0, 0) = pA_dt.x();
    dt(1, 0) = pA_dt.y();
    dt(2, 0) = pA_dt.z();
    dt(3, 0) = dA_dt.x();
    dt(4, 0) = dA_dt.y();
    dt(5, 0) = dA_dt.z();

    dt(6, 0) = pB_dt.x();
    dt(7, 0) = pB_dt.y();
    dt(8, 0) = pB_dt.z();
    dt(9, 0) = dB_dt.x();
    dt(10, 0) = dB_dt.y();
    dt(11, 0) = dB_dt.z();

    dt(12, 0) = pC_dt.x();
    dt(13, 0) = pC_dt.y();
    dt(14, 0) = pC_dt.z();
    dt(15, 0) = dC_dt.x();
    dt(16, 0) = dC_dt.y();
    dt(17, 0) = dC_dt.z();

    dt(18, 0) = pD_dt.x();
    dt(19, 0) = pD_dt.y();
    dt(20, 0) = pD_dt.z();
    dt(21, 0) = dD_dt.x();
    dt(22, 0) = dD_dt.y();
    dt(23, 0) = dD_dt.z();
}

// -----------------------------------------------------------------------------
// Assumed Natural Strain
// -----------------------------------------------------------------------------

// ANS shape function (interpolation of strain and strainD in thickness direction)
void ChElementShellANCF::ShapeFunctionANSbilinearShell(ChMatrixNM<double, 1, 4>& S_ANS, double x, double y) {
    S_ANS(0, 0) = -0.5 * x + 0.5;
    S_ANS(0, 1) = 0.5 * x + 0.5;
    S_ANS(0, 2) = -0.5 * y + 0.5;
    S_ANS(0, 3) = 0.5 * y + 0.5;
}

// Calculate ANS strain and its Jacobian
void ChElementShellANCF::CalcStrainANSbilinearShell() {
    std::vector<ChVector<> > knots(8);

    knots[0] = ChVector<>(-1, -1, 0);
    knots[1] = ChVector<>(1, -1, 0);
    knots[2] = ChVector<>(-1, 1, 0);
    knots[3] = ChVector<>(1, 1, 0);
    knots[4] = ChVector<>(-1, 0, 0);  // A
    knots[5] = ChVector<>(1, 0, 0);   // B
    knots[6] = ChVector<>(0, -1, 0);  // C
    knots[7] = ChVector<>(0, 1, 0);   // D

    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 8, 1> ddNz;
    ChMatrixNM<double, 8, 1> d0d0Nz;

    for (int kk = 0; kk < 8; kk++) {
        ShapeFunctionsDerivativeX(Nx, knots[kk].x(), knots[kk].y(), knots[kk].z());
        ShapeFunctionsDerivativeY(Ny, knots[kk].x(), knots[kk].y(), knots[kk].z());
        ShapeFunctionsDerivativeZ(Nz, knots[kk].x(), knots[kk].y(), knots[kk].z());

        ddNz.MatrMultiplyT(m_ddT, Nz);
        d0d0Nz.MatrMultiplyT(m_d0d0T, Nz);

        switch (kk) {
            case 0:
            case 1:
            case 2:
            case 3: {
                m_strainANS(kk, 0) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));
                ChMatrixNM<double, 1, 3> tmpZ = Nz * m_d;
                for (int i = 0; i < 8; i++)
                    for (int j = 0; j < 3; j++)
                        m_strainANS_D(kk, i * 3 + j) = tmpZ(0, j) * Nz(0, i);
                break;
            }
            case 4:
            case 5: {  // => yz
                m_strainANS(kk, 0) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);
                ChMatrixNM<double, 1, 3> tmpY = Ny * m_d;
                ChMatrixNM<double, 1, 3> tmpZ = Nz * m_d;
                for (int i = 0; i < 8; i++)
                    for (int j = 0; j < 3; j++)
                        m_strainANS_D(kk, i * 3 + j) = tmpY(0, j) * Nz(0, i) + tmpZ(0, j) * Ny(0, i);
                break;
            }
            case 6:
            case 7: {  // => xz
                m_strainANS(kk, 0) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);
                ChMatrixNM<double, 1, 3> tmpX = Nx * m_d;
                ChMatrixNM<double, 1, 3> tmpZ = Nz * m_d;
                for (int i = 0; i < 8; i++)
                    for (int j = 0; j < 3; j++)
                        m_strainANS_D(kk, i * 3 + j) = tmpX(0, j) * Nz(0, i) + tmpZ(0, j) * Nx(0, i);
                break;
            }
        }
    }
}

// -----------------------------------------------------------------------------
// Interface to ChElementShell base class
// -----------------------------------------------------------------------------
ChVector<> ChElementShellANCF::EvaluateSectionStrains() {
    // Element shape function
    ChMatrixNM<double, 1, 8> N;
    this->ShapeFunctions(N, 0, 0, 0);

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = this->Calc_detJ0(0, 0, 0, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // ANS shape function
    ChMatrixNM<double, 1, 4> S_ANS;  // Shape function vector for Assumed Natural Strain
    ChMatrixNM<double, 6, 5> M;      // Shape function vector for Enhanced Assumed Strain
    this->ShapeFunctionANSbilinearShell(S_ANS, 0, 0);
    this->Basis_M(M, 0, 0, 0);

    // Transformation : Orthogonal transformation (A and J)
    ChVector<double> G1xG2;  // Cross product of first and second column of
    double G1dotG1;          // Dot product of first column of position vector gradient

    G1xG2.x() = Nx_d0[0][1] * Ny_d0[0][2] - Nx_d0[0][2] * Ny_d0[0][1];
    G1xG2.y() = Nx_d0[0][2] * Ny_d0[0][0] - Nx_d0[0][0] * Ny_d0[0][2];
    G1xG2.z() = Nx_d0[0][0] * Ny_d0[0][1] - Nx_d0[0][1] * Ny_d0[0][0];
    G1dotG1 = Nx_d0[0][0] * Nx_d0[0][0] + Nx_d0[0][1] * Nx_d0[0][1] + Nx_d0[0][2] * Nx_d0[0][2];

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0[0][0];
    A1.y() = Nx_d0[0][1];
    A1.z() = Nx_d0[0][2];
    A1 = A1 / sqrt(G1dotG1);
    A3 = G1xG2.GetNormalized();
    A2.Cross(A3, A1);

    // Direction for orthotropic material
    double theta = 0.0;  // Fiber angle
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
    ChMatrixNM<double, 9, 1> beta;
    // Calculates inverse of rd0 (j0) (position vector gradient: Initial Configuration)
    j0(0, 0) = Ny_d0[0][1] * Nz_d0[0][2] - Nz_d0[0][1] * Ny_d0[0][2];
    j0(0, 1) = Ny_d0[0][2] * Nz_d0[0][0] - Ny_d0[0][0] * Nz_d0[0][2];
    j0(0, 2) = Ny_d0[0][0] * Nz_d0[0][1] - Nz_d0[0][0] * Ny_d0[0][1];
    j0(1, 0) = Nz_d0[0][1] * Nx_d0[0][2] - Nx_d0[0][1] * Nz_d0[0][2];
    j0(1, 1) = Nz_d0[0][2] * Nx_d0[0][0] - Nx_d0[0][2] * Nz_d0[0][0];
    j0(1, 2) = Nz_d0[0][0] * Nx_d0[0][1] - Nz_d0[0][1] * Nx_d0[0][0];
    j0(2, 0) = Nx_d0[0][1] * Ny_d0[0][2] - Ny_d0[0][1] * Nx_d0[0][2];
    j0(2, 1) = Ny_d0[0][0] * Nx_d0[0][2] - Nx_d0[0][0] * Ny_d0[0][2];
    j0(2, 2) = Nx_d0[0][0] * Ny_d0[0][1] - Ny_d0[0][0] * Nx_d0[0][1];
    j0.MatrDivScale(detJ0);

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
    beta(0, 0) = Vdot(AA1, j01);
    beta(1, 0) = Vdot(AA2, j01);
    beta(2, 0) = Vdot(AA3, j01);
    beta(3, 0) = Vdot(AA1, j02);
    beta(4, 0) = Vdot(AA2, j02);
    beta(5, 0) = Vdot(AA3, j02);
    beta(6, 0) = Vdot(AA1, j03);
    beta(7, 0) = Vdot(AA2, j03);
    beta(8, 0) = Vdot(AA3, j03);

    // Transformation matrix, function of fiber angle
    const ChMatrixNM<double, 6, 6>& T0 = this->GetLayer(0).Get_T0();
    // Determinant of the initial position vector gradient at the element center
    double detJ0C = this->GetLayer(0).Get_detJ0C();

// Enhanced Assumed Strain

// Enhanced Assumed Strain
#ifdef CHRONO_HAS_AVX
    // Referring to test_AVX, expected speed up Vs. non-AVX operation: 1.7x
    ChMatrixNM<double, 6, 5> G;
    G.MatrMultiplyAVX(T0, M);
    G = G * (detJ0C / detJ0);
#else
    ChMatrixNM<double, 6, 5> G = T0 * M * (detJ0C / detJ0);
#endif
    ChMatrixNM<double, 6, 1> strain_EAS = G * this->m_alphaEAS[0];

    ChMatrixNM<double, 8, 1> ddNx;
    ChMatrixNM<double, 8, 1> ddNy;
    ChMatrixNM<double, 8, 1> ddNz;
    ddNx.MatrMultiplyT(this->m_ddT, Nx);
    ddNy.MatrMultiplyT(this->m_ddT, Ny);
    ddNz.MatrMultiplyT(this->m_ddT, Nz);

    ChMatrixNM<double, 8, 1> d0d0Nx;
    ChMatrixNM<double, 8, 1> d0d0Ny;
    ChMatrixNM<double, 8, 1> d0d0Nz;
    d0d0Nx.MatrMultiplyT(this->m_d0d0T, Nx);
    d0d0Ny.MatrMultiplyT(this->m_d0d0T, Ny);
    d0d0Nz.MatrMultiplyT(this->m_d0d0T, Nz);

    // Strain component
    ChMatrixNM<double, 6, 1> strain_til;
    strain_til(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
    strain_til(3, 0) = N(0, 0) * this->m_strainANS(0, 0) + N(0, 2) * this->m_strainANS(1, 0) +
                       N(0, 4) * this->m_strainANS(2, 0) + N(0, 6) * this->m_strainANS(3, 0);
    strain_til(4, 0) = S_ANS(0, 2) * this->m_strainANS(6, 0) + S_ANS(0, 3) * this->m_strainANS(7, 0);
    strain_til(5, 0) = S_ANS(0, 0) * this->m_strainANS(4, 0) + S_ANS(0, 1) * this->m_strainANS(5, 0);

    // For orthotropic material
    ChMatrixNM<double, 6, 1> strain;

    strain(0, 0) = strain_til(0, 0) * beta(0) * beta(0) + strain_til(1, 0) * beta(3) * beta(3) +
                   strain_til(2, 0) * beta(0) * beta(3) + strain_til(3, 0) * beta(6) * beta(6) +
                   strain_til(4, 0) * beta(0) * beta(6) + strain_til(5, 0) * beta(3) * beta(6);
    strain(1, 0) = strain_til(0, 0) * beta(1) * beta(1) + strain_til(1, 0) * beta(4) * beta(4) +
                   strain_til(2, 0) * beta(1) * beta(4) + strain_til(3, 0) * beta(7) * beta(7) +
                   strain_til(4, 0) * beta(1) * beta(7) + strain_til(5, 0) * beta(4) * beta(7);
    strain(2, 0) = strain_til(0, 0) * 2.0 * beta(0) * beta(1) + strain_til(1, 0) * 2.0 * beta(3) * beta(4) +
                   strain_til(2, 0) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                   strain_til(3, 0) * 2.0 * beta(6) * beta(7) +
                   strain_til(4, 0) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                   strain_til(5, 0) * (beta(4) * beta(6) + beta(3) * beta(7));
    strain(3, 0) = strain_til(0, 0) * beta(2) * beta(2) + strain_til(1, 0) * beta(5) * beta(5) +
                   strain_til(2, 0) * beta(2) * beta(5) + strain_til(3, 0) * beta(8) * beta(8) +
                   strain_til(4, 0) * beta(2) * beta(8) + strain_til(5, 0) * beta(5) * beta(8);
    strain(4, 0) = strain_til(0, 0) * 2.0 * beta(0) * beta(2) + strain_til(1, 0) * 2.0 * beta(3) * beta(5) +
                   strain_til(2, 0) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                   strain_til(3, 0) * 2.0 * beta(6) * beta(8) +
                   strain_til(4, 0) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                   strain_til(5, 0) * (beta(5) * beta(6) + beta(3) * beta(8));
    strain(5, 0) = strain_til(0, 0) * 2.0 * beta(1) * beta(2) + strain_til(1, 0) * 2.0 * beta(4) * beta(5) +
                   strain_til(2, 0) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                   strain_til(3, 0) * 2.0 * beta(7) * beta(8) +
                   strain_til(4, 0) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                   strain_til(5, 0) * (beta(5) * beta(7) + beta(4) * beta(8));
    return ChVector<>(strain(0, 0), strain(1, 0), strain(2, 0));
}
void ChElementShellANCF::EvaluateSectionDisplacement(const double u,
                                                     const double v,
                                                     const ChMatrix<>& displ,
                                                     ChVector<>& u_displ,
                                                     ChVector<>& u_rotaz) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, displ, u_displ);
    u_rotaz = VNULL;  // no angles.. this is ANCF (or maybe return here the slope derivatives?)
}

void ChElementShellANCF::EvaluateSectionFrame(const double u,
                                              const double v,
                                              const ChMatrix<>& displ,
                                              ChVector<>& point,
                                              ChQuaternion<>& rot) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, displ, point);
    rot = QUNIT;  // or maybe use gram-schmidt to get csys of section from slopes?
}

void ChElementShellANCF::EvaluateSectionPoint(const double u,
                                              const double v,
                                              const ChMatrix<>& displ,
                                              ChVector<>& point) {
    ChVector<> u_displ;

    ChMatrixNM<double, 1, 8> N;

    double x = u;  // because ShapeFunctions() works in -1..1 range
    double y = v;  // because ShapeFunctions() works in -1..1 range
    double z = 0;

    this->ShapeFunctions(N, x, y, z);

    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& pD = m_nodes[3]->GetPos();

    point.x() = N(0) * pA.x() + N(2) * pB.x() + N(4) * pC.x() + N(6) * pD.x();
    point.y() = N(0) * pA.y() + N(2) * pB.y() + N(4) * pC.y() + N(6) * pD.y();
    point.z() = N(0) * pA.z() + N(2) * pB.z() + N(4) * pC.z() + N(6) * pD.z();
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------

// Gets all the DOFs packed in a single vector (position part).
void ChElementShellANCF::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.PasteVector(m_nodes[0]->GetPos(), block_offset, 0);
    mD.PasteVector(m_nodes[0]->GetD(), block_offset + 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos(), block_offset + 6, 0);
    mD.PasteVector(m_nodes[1]->GetD(), block_offset + 9, 0);
    mD.PasteVector(m_nodes[2]->GetPos(), block_offset + 12, 0);
    mD.PasteVector(m_nodes[2]->GetD(), block_offset + 15, 0);
    mD.PasteVector(m_nodes[3]->GetPos(), block_offset + 18, 0);
    mD.PasteVector(m_nodes[3]->GetD(), block_offset + 21, 0);
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChElementShellANCF::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.PasteVector(m_nodes[0]->GetPos_dt(), block_offset, 0);
    mD.PasteVector(m_nodes[0]->GetD_dt(), block_offset + 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos_dt(), block_offset + 6, 0);
    mD.PasteVector(m_nodes[1]->GetD_dt(), block_offset + 9, 0);
    mD.PasteVector(m_nodes[2]->GetPos_dt(), block_offset + 12, 0);
    mD.PasteVector(m_nodes[2]->GetD_dt(), block_offset + 15, 0);
    mD.PasteVector(m_nodes[3]->GetPos_dt(), block_offset + 18, 0);
    mD.PasteVector(m_nodes[3]->GetD_dt(), block_offset + 21, 0);
}

void ChElementShellANCF::LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv)  {
    for (int i = 0; i < 4; i++) {
        this->m_nodes[i]->NodeIntStateIncrement(off_x  + 6 * i  , x_new, x, off_v  + 6 * i  , Dv);
    }
}

void ChElementShellANCF::EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) {
    ChMatrixNM<double, 8, 1> N;
    ShapeFunctions(N, U, V, 0);
    for (unsigned int ii = 0; ii < 4; ii++) {
        Result += N(ii * 2) * m_nodes[ii]->GetPos_dt();
        Result += N(ii * 2 + 1) * m_nodes[ii]->GetPos_dt();
    }
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChElementShellANCF::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
        mvars.push_back(&m_nodes[i]->Variables_D());
    }
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChElementShellANCF::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 8> N;
    ShapeFunctions(N, U, V, 0);

    detJ = Calc_detJ0(U, V, 0);
    detJ *= GetLengthX() * GetLengthY() / 4.0;

    ChVector<> tmp;
    ChVector<> Fv = F.ClipVector(0, 0);
    tmp = N(0) * Fv;
    Qi.PasteVector(tmp, 0, 0);
    tmp = N(1) * Fv;
    Qi.PasteVector(tmp, 3, 0);
    tmp = N(2) * Fv;
    Qi.PasteVector(tmp, 6, 0);
    tmp = N(3) * Fv;
    Qi.PasteVector(tmp, 9, 0);
    tmp = N(4) * Fv;
    Qi.PasteVector(tmp, 12, 0);
    tmp = N(5) * Fv;
    Qi.PasteVector(tmp, 15, 0);
    tmp = N(6) * Fv;
    Qi.PasteVector(tmp, 18, 0);
    tmp = N(7) * Fv;
    Qi.PasteVector(tmp, 21, 0);
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V,W) coordinates of the surface.
void ChElementShellANCF::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 8> N;
    ShapeFunctions(N, U, V, W);

    detJ = Calc_detJ0(U, V, W);
    detJ *= m_GaussScaling;

    ChVector<> tmp;
    ChVector<> Fv = F.ClipVector(0, 0);
    tmp = N(0) * Fv;
    Qi.PasteVector(tmp, 0, 0);
    tmp = N(1) * Fv;
    Qi.PasteVector(tmp, 3, 0);
    tmp = N(2) * Fv;
    Qi.PasteVector(tmp, 6, 0);
    tmp = N(3) * Fv;
    Qi.PasteVector(tmp, 9, 0);
    tmp = N(4) * Fv;
    Qi.PasteVector(tmp, 12, 0);
    tmp = N(5) * Fv;
    Qi.PasteVector(tmp, 15, 0);
    tmp = N(6) * Fv;
    Qi.PasteVector(tmp, 18, 0);
    tmp = N(7) * Fv;
    Qi.PasteVector(tmp, 21, 0);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

// Calculate avergae element density (needed for ChLoaderVolumeGravity).
double ChElementShellANCF::GetDensity() {
    double tot_density = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        double layerthick = m_layers[kl].Get_thickness();
        tot_density += rho * layerthick;
    }
    return tot_density / m_thickness;
}

// Calculate normal to the surface at (U,V) coordinates.
ChVector<> ChElementShellANCF::ComputeNormal(const double U, const double V) {
    ChMatrixNM<double, 8, 3> mD;
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;

    ShapeFunctionsDerivativeX(Nx, U, V, 0);
    ShapeFunctionsDerivativeY(Ny, U, V, 0);
    ShapeFunctionsDerivativeZ(Nz, U, V, 0);

    CalcCoordMatrix(mD);

    ChMatrixNM<double, 1, 3> Nx_d = Nx * mD;
    ChMatrixNM<double, 1, 3> Ny_d = Ny * mD;
    ChMatrixNM<double, 1, 3> Nz_d = Nz * mD;

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

// -----------------------------------------------------------------------------
// Utility functions for inverting a 5x5 matrix
// -----------------------------------------------------------------------------

// Invert matrix by Gauss method
void ChElementShellANCF::Inverse55_Numerical(ChMatrixNM<double, 5, 5>& a, int n) {
    // - - - Local Variables - - -
    ChMatrixNM<double, 5, 5> b;
    double c;
    double d;
    double preValue = 0.0;
    ChMatrixNM<double, 5, 1> temp;
    int m;
    int count;
    ChMatrixNM<int, 1, 1> imax;
    ChMatrixNM<int, 5, 1> ipvt;
    // - - - - - - - - - - - - - -

    b = a;
    for (int i = 0; i < n; i++) {
        ipvt(i) = i;
    }

    for (int k = 0; k < n; k++) {
        // imax = MAXLOC(ABS(b(k:n,k)))
        count = 0;
        preValue = 0.0;
        for (int ii = k; ii < n; ii++) {
            if (preValue < std::abs(b(ii, k))) {
                preValue = std::abs(b(ii, k));
                count = count + 1;
            }
            imax(1) = count + k;
        }
        m = k - 1 + imax(1);

        if (m != k) {
            int temp_ipvt = ipvt(m);
            ipvt(m) = ipvt(k);  // ipvt( (/m,k/) ) = ipvt( (/k,m/) )
            ipvt(k) = temp_ipvt;
            for (int ii = 0; ii < n; ii++) {
                double temp_b = b((m), ii);  // b((/m,k/),:) = b((/k,m/),:)
                b(m, ii) = b(k, ii);
                b(k, ii) = temp_b;
            }
        }
        d = 1 / b(k, k);
        for (int ii = 0; ii < n; ii++) {
            temp(ii) = b(ii, k);
        }
        for (int j = 0; j < n; j++) {
            c = b(k, j) * d;
            for (int ii = 0; ii < n; ii++) {
                b(ii, j) = b(ii, j) - temp(ii) * c;
            }
            b(k, j) = c;
        }
        for (int ii = 0; ii < n; ii++) {
            b(ii, k) = temp(ii) * (-d);
        }
        b(k, k) = d;
    }
    for (int ii = 0; ii < n; ii++) {
        for (int jj = 0; jj < n; jj++) {
            a(ii, ipvt(jj)) = b(ii, jj);
        }
    }
}

// Analytical inverse for a 5x5 matrix
void ChElementShellANCF::Inverse55_Analytical(ChMatrixNM<double, 5, 5>& A, ChMatrixNM<double, 5, 5>& B) {
    const double& a1 = B(0, 0);
    const double& a2 = B(0, 1);
    const double& a3 = B(0, 2);
    const double& a4 = B(0, 3);
    const double& a5 = B(0, 4);
    const double& b1 = B(1, 0);
    const double& b2 = B(1, 1);
    const double& b3 = B(1, 2);
    const double& b4 = B(1, 3);
    const double& b5 = B(1, 4);
    const double& c1 = B(2, 0);
    const double& c2 = B(2, 1);
    const double& c3 = B(2, 2);
    const double& c4 = B(2, 3);
    const double& c5 = B(2, 4);
    const double& d1 = B(3, 0);
    const double& d2 = B(3, 1);
    const double& d3 = B(3, 2);
    const double& d4 = B(3, 3);
    const double& d5 = B(3, 4);
    const double& e1 = B(4, 0);
    const double& e2 = B(4, 1);
    const double& e3 = B(4, 2);
    const double& e4 = B(4, 3);
    const double& e5 = B(4, 4);

    double denom = a1 * b2 * c3 * d4 * e5 - a1 * b2 * c3 * d5 * e4 - a1 * b2 * c4 * d3 * e5 + a1 * b2 * c4 * d5 * e3 +
                   a1 * b2 * c5 * d3 * e4 - a1 * b2 * c5 * d4 * e3 - a1 * b3 * c2 * d4 * e5 + a1 * b3 * c2 * d5 * e4 +
                   a1 * b3 * c4 * d2 * e5 - a1 * b3 * c4 * d5 * e2 - a1 * b3 * c5 * d2 * e4 + a1 * b3 * c5 * d4 * e2 +
                   a1 * b4 * c2 * d3 * e5 - a1 * b4 * c2 * d5 * e3 - a1 * b4 * c3 * d2 * e5 + a1 * b4 * c3 * d5 * e2 +
                   a1 * b4 * c5 * d2 * e3 - a1 * b4 * c5 * d3 * e2 - a1 * b5 * c2 * d3 * e4 + a1 * b5 * c2 * d4 * e3 +
                   a1 * b5 * c3 * d2 * e4 - a1 * b5 * c3 * d4 * e2 - a1 * b5 * c4 * d2 * e3 + a1 * b5 * c4 * d3 * e2 -
                   a2 * b1 * c3 * d4 * e5 + a2 * b1 * c3 * d5 * e4 + a2 * b1 * c4 * d3 * e5 - a2 * b1 * c4 * d5 * e3 -
                   a2 * b1 * c5 * d3 * e4 + a2 * b1 * c5 * d4 * e3 + a2 * b3 * c1 * d4 * e5 - a2 * b3 * c1 * d5 * e4 -
                   a2 * b3 * c4 * d1 * e5 + a2 * b3 * c4 * d5 * e1 + a2 * b3 * c5 * d1 * e4 - a2 * b3 * c5 * d4 * e1 -
                   a2 * b4 * c1 * d3 * e5 + a2 * b4 * c1 * d5 * e3 + a2 * b4 * c3 * d1 * e5 - a2 * b4 * c3 * d5 * e1 -
                   a2 * b4 * c5 * d1 * e3 + a2 * b4 * c5 * d3 * e1 + a2 * b5 * c1 * d3 * e4 - a2 * b5 * c1 * d4 * e3 -
                   a2 * b5 * c3 * d1 * e4 + a2 * b5 * c3 * d4 * e1 + a2 * b5 * c4 * d1 * e3 - a2 * b5 * c4 * d3 * e1 +
                   a3 * b1 * c2 * d4 * e5 - a3 * b1 * c2 * d5 * e4 - a3 * b1 * c4 * d2 * e5 + a3 * b1 * c4 * d5 * e2 +
                   a3 * b1 * c5 * d2 * e4 - a3 * b1 * c5 * d4 * e2 - a3 * b2 * c1 * d4 * e5 + a3 * b2 * c1 * d5 * e4 +
                   a3 * b2 * c4 * d1 * e5 - a3 * b2 * c4 * d5 * e1 - a3 * b2 * c5 * d1 * e4 + a3 * b2 * c5 * d4 * e1 +
                   a3 * b4 * c1 * d2 * e5 - a3 * b4 * c1 * d5 * e2 - a3 * b4 * c2 * d1 * e5 + a3 * b4 * c2 * d5 * e1 +
                   a3 * b4 * c5 * d1 * e2 - a3 * b4 * c5 * d2 * e1 - a3 * b5 * c1 * d2 * e4 + a3 * b5 * c1 * d4 * e2 +
                   a3 * b5 * c2 * d1 * e4 - a3 * b5 * c2 * d4 * e1 - a3 * b5 * c4 * d1 * e2 + a3 * b5 * c4 * d2 * e1 -
                   a4 * b1 * c2 * d3 * e5 + a4 * b1 * c2 * d5 * e3 + a4 * b1 * c3 * d2 * e5 - a4 * b1 * c3 * d5 * e2 -
                   a4 * b1 * c5 * d2 * e3 + a4 * b1 * c5 * d3 * e2 + a4 * b2 * c1 * d3 * e5 - a4 * b2 * c1 * d5 * e3 -
                   a4 * b2 * c3 * d1 * e5 + a4 * b2 * c3 * d5 * e1 + a4 * b2 * c5 * d1 * e3 - a4 * b2 * c5 * d3 * e1 -
                   a4 * b3 * c1 * d2 * e5 + a4 * b3 * c1 * d5 * e2 + a4 * b3 * c2 * d1 * e5 - a4 * b3 * c2 * d5 * e1 -
                   a4 * b3 * c5 * d1 * e2 + a4 * b3 * c5 * d2 * e1 + a4 * b5 * c1 * d2 * e3 - a4 * b5 * c1 * d3 * e2 -
                   a4 * b5 * c2 * d1 * e3 + a4 * b5 * c2 * d3 * e1 + a4 * b5 * c3 * d1 * e2 - a4 * b5 * c3 * d2 * e1 +
                   a5 * b1 * c2 * d3 * e4 - a5 * b1 * c2 * d4 * e3 - a5 * b1 * c3 * d2 * e4 + a5 * b1 * c3 * d4 * e2 +
                   a5 * b1 * c4 * d2 * e3 - a5 * b1 * c4 * d3 * e2 - a5 * b2 * c1 * d3 * e4 + a5 * b2 * c1 * d4 * e3 +
                   a5 * b2 * c3 * d1 * e4 - a5 * b2 * c3 * d4 * e1 - a5 * b2 * c4 * d1 * e3 + a5 * b2 * c4 * d3 * e1 +
                   a5 * b3 * c1 * d2 * e4 - a5 * b3 * c1 * d4 * e2 - a5 * b3 * c2 * d1 * e4 + a5 * b3 * c2 * d4 * e1 +
                   a5 * b3 * c4 * d1 * e2 - a5 * b3 * c4 * d2 * e1 - a5 * b4 * c1 * d2 * e3 + a5 * b4 * c1 * d3 * e2 +
                   a5 * b4 * c2 * d1 * e3 - a5 * b4 * c2 * d3 * e1 - a5 * b4 * c3 * d1 * e2 + a5 * b4 * c3 * d2 * e1;

    A[0][0] = (b2 * c3 * d4 * e5 - b2 * c3 * d5 * e4 - b2 * c4 * d3 * e5 + b2 * c4 * d5 * e3 + b2 * c5 * d3 * e4 -
               b2 * c5 * d4 * e3 - b3 * c2 * d4 * e5 + b3 * c2 * d5 * e4 + b3 * c4 * d2 * e5 - b3 * c4 * d5 * e2 -
               b3 * c5 * d2 * e4 + b3 * c5 * d4 * e2 + b4 * c2 * d3 * e5 - b4 * c2 * d5 * e3 - b4 * c3 * d2 * e5 +
               b4 * c3 * d5 * e2 + b4 * c5 * d2 * e3 - b4 * c5 * d3 * e2 - b5 * c2 * d3 * e4 + b5 * c2 * d4 * e3 +
               b5 * c3 * d2 * e4 - b5 * c3 * d4 * e2 - b5 * c4 * d2 * e3 + b5 * c4 * d3 * e2) /
              denom;

    A[0][1] = -(a2 * c3 * d4 * e5 - a2 * c3 * d5 * e4 - a2 * c4 * d3 * e5 + a2 * c4 * d5 * e3 + a2 * c5 * d3 * e4 -
                a2 * c5 * d4 * e3 - a3 * c2 * d4 * e5 + a3 * c2 * d5 * e4 + a3 * c4 * d2 * e5 - a3 * c4 * d5 * e2 -
                a3 * c5 * d2 * e4 + a3 * c5 * d4 * e2 + a4 * c2 * d3 * e5 - a4 * c2 * d5 * e3 - a4 * c3 * d2 * e5 +
                a4 * c3 * d5 * e2 + a4 * c5 * d2 * e3 - a4 * c5 * d3 * e2 - a5 * c2 * d3 * e4 + a5 * c2 * d4 * e3 +
                a5 * c3 * d2 * e4 - a5 * c3 * d4 * e2 - a5 * c4 * d2 * e3 + a5 * c4 * d3 * e2) /
              denom;

    A[0][2] = (a2 * b3 * d4 * e5 - a2 * b3 * d5 * e4 - a2 * b4 * d3 * e5 + a2 * b4 * d5 * e3 + a2 * b5 * d3 * e4 -
               a2 * b5 * d4 * e3 - a3 * b2 * d4 * e5 + a3 * b2 * d5 * e4 + a3 * b4 * d2 * e5 - a3 * b4 * d5 * e2 -
               a3 * b5 * d2 * e4 + a3 * b5 * d4 * e2 + a4 * b2 * d3 * e5 - a4 * b2 * d5 * e3 - a4 * b3 * d2 * e5 +
               a4 * b3 * d5 * e2 + a4 * b5 * d2 * e3 - a4 * b5 * d3 * e2 - a5 * b2 * d3 * e4 + a5 * b2 * d4 * e3 +
               a5 * b3 * d2 * e4 - a5 * b3 * d4 * e2 - a5 * b4 * d2 * e3 + a5 * b4 * d3 * e2) /
              denom;

    A[0][3] = -(a2 * b3 * c4 * e5 - a2 * b3 * c5 * e4 - a2 * b4 * c3 * e5 + a2 * b4 * c5 * e3 + a2 * b5 * c3 * e4 -
                a2 * b5 * c4 * e3 - a3 * b2 * c4 * e5 + a3 * b2 * c5 * e4 + a3 * b4 * c2 * e5 - a3 * b4 * c5 * e2 -
                a3 * b5 * c2 * e4 + a3 * b5 * c4 * e2 + a4 * b2 * c3 * e5 - a4 * b2 * c5 * e3 - a4 * b3 * c2 * e5 +
                a4 * b3 * c5 * e2 + a4 * b5 * c2 * e3 - a4 * b5 * c3 * e2 - a5 * b2 * c3 * e4 + a5 * b2 * c4 * e3 +
                a5 * b3 * c2 * e4 - a5 * b3 * c4 * e2 - a5 * b4 * c2 * e3 + a5 * b4 * c3 * e2) /
              denom;

    A[0][4] = (a2 * b3 * c4 * d5 - a2 * b3 * c5 * d4 - a2 * b4 * c3 * d5 + a2 * b4 * c5 * d3 + a2 * b5 * c3 * d4 -
               a2 * b5 * c4 * d3 - a3 * b2 * c4 * d5 + a3 * b2 * c5 * d4 + a3 * b4 * c2 * d5 - a3 * b4 * c5 * d2 -
               a3 * b5 * c2 * d4 + a3 * b5 * c4 * d2 + a4 * b2 * c3 * d5 - a4 * b2 * c5 * d3 - a4 * b3 * c2 * d5 +
               a4 * b3 * c5 * d2 + a4 * b5 * c2 * d3 - a4 * b5 * c3 * d2 - a5 * b2 * c3 * d4 + a5 * b2 * c4 * d3 +
               a5 * b3 * c2 * d4 - a5 * b3 * c4 * d2 - a5 * b4 * c2 * d3 + a5 * b4 * c3 * d2) /
              denom;

    A[1][0] = -(b1 * c3 * d4 * e5 - b1 * c3 * d5 * e4 - b1 * c4 * d3 * e5 + b1 * c4 * d5 * e3 + b1 * c5 * d3 * e4 -
                b1 * c5 * d4 * e3 - b3 * c1 * d4 * e5 + b3 * c1 * d5 * e4 + b3 * c4 * d1 * e5 - b3 * c4 * d5 * e1 -
                b3 * c5 * d1 * e4 + b3 * c5 * d4 * e1 + b4 * c1 * d3 * e5 - b4 * c1 * d5 * e3 - b4 * c3 * d1 * e5 +
                b4 * c3 * d5 * e1 + b4 * c5 * d1 * e3 - b4 * c5 * d3 * e1 - b5 * c1 * d3 * e4 + b5 * c1 * d4 * e3 +
                b5 * c3 * d1 * e4 - b5 * c3 * d4 * e1 - b5 * c4 * d1 * e3 + b5 * c4 * d3 * e1) /
              denom;

    A[1][1] = (a1 * c3 * d4 * e5 - a1 * c3 * d5 * e4 - a1 * c4 * d3 * e5 + a1 * c4 * d5 * e3 + a1 * c5 * d3 * e4 -
               a1 * c5 * d4 * e3 - a3 * c1 * d4 * e5 + a3 * c1 * d5 * e4 + a3 * c4 * d1 * e5 - a3 * c4 * d5 * e1 -
               a3 * c5 * d1 * e4 + a3 * c5 * d4 * e1 + a4 * c1 * d3 * e5 - a4 * c1 * d5 * e3 - a4 * c3 * d1 * e5 +
               a4 * c3 * d5 * e1 + a4 * c5 * d1 * e3 - a4 * c5 * d3 * e1 - a5 * c1 * d3 * e4 + a5 * c1 * d4 * e3 +
               a5 * c3 * d1 * e4 - a5 * c3 * d4 * e1 - a5 * c4 * d1 * e3 + a5 * c4 * d3 * e1) /
              denom;

    A[1][2] = -(a1 * b3 * d4 * e5 - a1 * b3 * d5 * e4 - a1 * b4 * d3 * e5 + a1 * b4 * d5 * e3 + a1 * b5 * d3 * e4 -
                a1 * b5 * d4 * e3 - a3 * b1 * d4 * e5 + a3 * b1 * d5 * e4 + a3 * b4 * d1 * e5 - a3 * b4 * d5 * e1 -
                a3 * b5 * d1 * e4 + a3 * b5 * d4 * e1 + a4 * b1 * d3 * e5 - a4 * b1 * d5 * e3 - a4 * b3 * d1 * e5 +
                a4 * b3 * d5 * e1 + a4 * b5 * d1 * e3 - a4 * b5 * d3 * e1 - a5 * b1 * d3 * e4 + a5 * b1 * d4 * e3 +
                a5 * b3 * d1 * e4 - a5 * b3 * d4 * e1 - a5 * b4 * d1 * e3 + a5 * b4 * d3 * e1) /
              denom;

    A[1][3] = (a1 * b3 * c4 * e5 - a1 * b3 * c5 * e4 - a1 * b4 * c3 * e5 + a1 * b4 * c5 * e3 + a1 * b5 * c3 * e4 -
               a1 * b5 * c4 * e3 - a3 * b1 * c4 * e5 + a3 * b1 * c5 * e4 + a3 * b4 * c1 * e5 - a3 * b4 * c5 * e1 -
               a3 * b5 * c1 * e4 + a3 * b5 * c4 * e1 + a4 * b1 * c3 * e5 - a4 * b1 * c5 * e3 - a4 * b3 * c1 * e5 +
               a4 * b3 * c5 * e1 + a4 * b5 * c1 * e3 - a4 * b5 * c3 * e1 - a5 * b1 * c3 * e4 + a5 * b1 * c4 * e3 +
               a5 * b3 * c1 * e4 - a5 * b3 * c4 * e1 - a5 * b4 * c1 * e3 + a5 * b4 * c3 * e1) /
              denom;

    A[1][4] = -(a1 * b3 * c4 * d5 - a1 * b3 * c5 * d4 - a1 * b4 * c3 * d5 + a1 * b4 * c5 * d3 + a1 * b5 * c3 * d4 -
                a1 * b5 * c4 * d3 - a3 * b1 * c4 * d5 + a3 * b1 * c5 * d4 + a3 * b4 * c1 * d5 - a3 * b4 * c5 * d1 -
                a3 * b5 * c1 * d4 + a3 * b5 * c4 * d1 + a4 * b1 * c3 * d5 - a4 * b1 * c5 * d3 - a4 * b3 * c1 * d5 +
                a4 * b3 * c5 * d1 + a4 * b5 * c1 * d3 - a4 * b5 * c3 * d1 - a5 * b1 * c3 * d4 + a5 * b1 * c4 * d3 +
                a5 * b3 * c1 * d4 - a5 * b3 * c4 * d1 - a5 * b4 * c1 * d3 + a5 * b4 * c3 * d1) /
              denom;

    A[2][0] = (b1 * c2 * d4 * e5 - b1 * c2 * d5 * e4 - b1 * c4 * d2 * e5 + b1 * c4 * d5 * e2 + b1 * c5 * d2 * e4 -
               b1 * c5 * d4 * e2 - b2 * c1 * d4 * e5 + b2 * c1 * d5 * e4 + b2 * c4 * d1 * e5 - b2 * c4 * d5 * e1 -
               b2 * c5 * d1 * e4 + b2 * c5 * d4 * e1 + b4 * c1 * d2 * e5 - b4 * c1 * d5 * e2 - b4 * c2 * d1 * e5 +
               b4 * c2 * d5 * e1 + b4 * c5 * d1 * e2 - b4 * c5 * d2 * e1 - b5 * c1 * d2 * e4 + b5 * c1 * d4 * e2 +
               b5 * c2 * d1 * e4 - b5 * c2 * d4 * e1 - b5 * c4 * d1 * e2 + b5 * c4 * d2 * e1) /
              denom;

    A[2][1] = -(a1 * c2 * d4 * e5 - a1 * c2 * d5 * e4 - a1 * c4 * d2 * e5 + a1 * c4 * d5 * e2 + a1 * c5 * d2 * e4 -
                a1 * c5 * d4 * e2 - a2 * c1 * d4 * e5 + a2 * c1 * d5 * e4 + a2 * c4 * d1 * e5 - a2 * c4 * d5 * e1 -
                a2 * c5 * d1 * e4 + a2 * c5 * d4 * e1 + a4 * c1 * d2 * e5 - a4 * c1 * d5 * e2 - a4 * c2 * d1 * e5 +
                a4 * c2 * d5 * e1 + a4 * c5 * d1 * e2 - a4 * c5 * d2 * e1 - a5 * c1 * d2 * e4 + a5 * c1 * d4 * e2 +
                a5 * c2 * d1 * e4 - a5 * c2 * d4 * e1 - a5 * c4 * d1 * e2 + a5 * c4 * d2 * e1) /
              denom;

    A[2][2] = (a1 * b2 * d4 * e5 - a1 * b2 * d5 * e4 - a1 * b4 * d2 * e5 + a1 * b4 * d5 * e2 + a1 * b5 * d2 * e4 -
               a1 * b5 * d4 * e2 - a2 * b1 * d4 * e5 + a2 * b1 * d5 * e4 + a2 * b4 * d1 * e5 - a2 * b4 * d5 * e1 -
               a2 * b5 * d1 * e4 + a2 * b5 * d4 * e1 + a4 * b1 * d2 * e5 - a4 * b1 * d5 * e2 - a4 * b2 * d1 * e5 +
               a4 * b2 * d5 * e1 + a4 * b5 * d1 * e2 - a4 * b5 * d2 * e1 - a5 * b1 * d2 * e4 + a5 * b1 * d4 * e2 +
               a5 * b2 * d1 * e4 - a5 * b2 * d4 * e1 - a5 * b4 * d1 * e2 + a5 * b4 * d2 * e1) /
              denom;

    A[2][3] = -(a1 * b2 * c4 * e5 - a1 * b2 * c5 * e4 - a1 * b4 * c2 * e5 + a1 * b4 * c5 * e2 + a1 * b5 * c2 * e4 -
                a1 * b5 * c4 * e2 - a2 * b1 * c4 * e5 + a2 * b1 * c5 * e4 + a2 * b4 * c1 * e5 - a2 * b4 * c5 * e1 -
                a2 * b5 * c1 * e4 + a2 * b5 * c4 * e1 + a4 * b1 * c2 * e5 - a4 * b1 * c5 * e2 - a4 * b2 * c1 * e5 +
                a4 * b2 * c5 * e1 + a4 * b5 * c1 * e2 - a4 * b5 * c2 * e1 - a5 * b1 * c2 * e4 + a5 * b1 * c4 * e2 +
                a5 * b2 * c1 * e4 - a5 * b2 * c4 * e1 - a5 * b4 * c1 * e2 + a5 * b4 * c2 * e1) /
              denom;

    A[2][4] = (a1 * b2 * c4 * d5 - a1 * b2 * c5 * d4 - a1 * b4 * c2 * d5 + a1 * b4 * c5 * d2 + a1 * b5 * c2 * d4 -
               a1 * b5 * c4 * d2 - a2 * b1 * c4 * d5 + a2 * b1 * c5 * d4 + a2 * b4 * c1 * d5 - a2 * b4 * c5 * d1 -
               a2 * b5 * c1 * d4 + a2 * b5 * c4 * d1 + a4 * b1 * c2 * d5 - a4 * b1 * c5 * d2 - a4 * b2 * c1 * d5 +
               a4 * b2 * c5 * d1 + a4 * b5 * c1 * d2 - a4 * b5 * c2 * d1 - a5 * b1 * c2 * d4 + a5 * b1 * c4 * d2 +
               a5 * b2 * c1 * d4 - a5 * b2 * c4 * d1 - a5 * b4 * c1 * d2 + a5 * b4 * c2 * d1) /
              denom;

    A[3][0] = -(b1 * c2 * d3 * e5 - b1 * c2 * d5 * e3 - b1 * c3 * d2 * e5 + b1 * c3 * d5 * e2 + b1 * c5 * d2 * e3 -
                b1 * c5 * d3 * e2 - b2 * c1 * d3 * e5 + b2 * c1 * d5 * e3 + b2 * c3 * d1 * e5 - b2 * c3 * d5 * e1 -
                b2 * c5 * d1 * e3 + b2 * c5 * d3 * e1 + b3 * c1 * d2 * e5 - b3 * c1 * d5 * e2 - b3 * c2 * d1 * e5 +
                b3 * c2 * d5 * e1 + b3 * c5 * d1 * e2 - b3 * c5 * d2 * e1 - b5 * c1 * d2 * e3 + b5 * c1 * d3 * e2 +
                b5 * c2 * d1 * e3 - b5 * c2 * d3 * e1 - b5 * c3 * d1 * e2 + b5 * c3 * d2 * e1) /
              denom;

    A[3][1] = (a1 * c2 * d3 * e5 - a1 * c2 * d5 * e3 - a1 * c3 * d2 * e5 + a1 * c3 * d5 * e2 + a1 * c5 * d2 * e3 -
               a1 * c5 * d3 * e2 - a2 * c1 * d3 * e5 + a2 * c1 * d5 * e3 + a2 * c3 * d1 * e5 - a2 * c3 * d5 * e1 -
               a2 * c5 * d1 * e3 + a2 * c5 * d3 * e1 + a3 * c1 * d2 * e5 - a3 * c1 * d5 * e2 - a3 * c2 * d1 * e5 +
               a3 * c2 * d5 * e1 + a3 * c5 * d1 * e2 - a3 * c5 * d2 * e1 - a5 * c1 * d2 * e3 + a5 * c1 * d3 * e2 +
               a5 * c2 * d1 * e3 - a5 * c2 * d3 * e1 - a5 * c3 * d1 * e2 + a5 * c3 * d2 * e1) /
              denom;

    A[3][2] = -(a1 * b2 * d3 * e5 - a1 * b2 * d5 * e3 - a1 * b3 * d2 * e5 + a1 * b3 * d5 * e2 + a1 * b5 * d2 * e3 -
                a1 * b5 * d3 * e2 - a2 * b1 * d3 * e5 + a2 * b1 * d5 * e3 + a2 * b3 * d1 * e5 - a2 * b3 * d5 * e1 -
                a2 * b5 * d1 * e3 + a2 * b5 * d3 * e1 + a3 * b1 * d2 * e5 - a3 * b1 * d5 * e2 - a3 * b2 * d1 * e5 +
                a3 * b2 * d5 * e1 + a3 * b5 * d1 * e2 - a3 * b5 * d2 * e1 - a5 * b1 * d2 * e3 + a5 * b1 * d3 * e2 +
                a5 * b2 * d1 * e3 - a5 * b2 * d3 * e1 - a5 * b3 * d1 * e2 + a5 * b3 * d2 * e1) /
              denom;

    A[3][3] = (a1 * b2 * c3 * e5 - a1 * b2 * c5 * e3 - a1 * b3 * c2 * e5 + a1 * b3 * c5 * e2 + a1 * b5 * c2 * e3 -
               a1 * b5 * c3 * e2 - a2 * b1 * c3 * e5 + a2 * b1 * c5 * e3 + a2 * b3 * c1 * e5 - a2 * b3 * c5 * e1 -
               a2 * b5 * c1 * e3 + a2 * b5 * c3 * e1 + a3 * b1 * c2 * e5 - a3 * b1 * c5 * e2 - a3 * b2 * c1 * e5 +
               a3 * b2 * c5 * e1 + a3 * b5 * c1 * e2 - a3 * b5 * c2 * e1 - a5 * b1 * c2 * e3 + a5 * b1 * c3 * e2 +
               a5 * b2 * c1 * e3 - a5 * b2 * c3 * e1 - a5 * b3 * c1 * e2 + a5 * b3 * c2 * e1) /
              denom;

    A[3][4] = -(a1 * b2 * c3 * d5 - a1 * b2 * c5 * d3 - a1 * b3 * c2 * d5 + a1 * b3 * c5 * d2 + a1 * b5 * c2 * d3 -
                a1 * b5 * c3 * d2 - a2 * b1 * c3 * d5 + a2 * b1 * c5 * d3 + a2 * b3 * c1 * d5 - a2 * b3 * c5 * d1 -
                a2 * b5 * c1 * d3 + a2 * b5 * c3 * d1 + a3 * b1 * c2 * d5 - a3 * b1 * c5 * d2 - a3 * b2 * c1 * d5 +
                a3 * b2 * c5 * d1 + a3 * b5 * c1 * d2 - a3 * b5 * c2 * d1 - a5 * b1 * c2 * d3 + a5 * b1 * c3 * d2 +
                a5 * b2 * c1 * d3 - a5 * b2 * c3 * d1 - a5 * b3 * c1 * d2 + a5 * b3 * c2 * d1) /
              denom;

    A[4][0] = (b1 * c2 * d3 * e4 - b1 * c2 * d4 * e3 - b1 * c3 * d2 * e4 + b1 * c3 * d4 * e2 + b1 * c4 * d2 * e3 -
               b1 * c4 * d3 * e2 - b2 * c1 * d3 * e4 + b2 * c1 * d4 * e3 + b2 * c3 * d1 * e4 - b2 * c3 * d4 * e1 -
               b2 * c4 * d1 * e3 + b2 * c4 * d3 * e1 + b3 * c1 * d2 * e4 - b3 * c1 * d4 * e2 - b3 * c2 * d1 * e4 +
               b3 * c2 * d4 * e1 + b3 * c4 * d1 * e2 - b3 * c4 * d2 * e1 - b4 * c1 * d2 * e3 + b4 * c1 * d3 * e2 +
               b4 * c2 * d1 * e3 - b4 * c2 * d3 * e1 - b4 * c3 * d1 * e2 + b4 * c3 * d2 * e1) /
              denom;

    A[4][1] = -(a1 * c2 * d3 * e4 - a1 * c2 * d4 * e3 - a1 * c3 * d2 * e4 + a1 * c3 * d4 * e2 + a1 * c4 * d2 * e3 -
                a1 * c4 * d3 * e2 - a2 * c1 * d3 * e4 + a2 * c1 * d4 * e3 + a2 * c3 * d1 * e4 - a2 * c3 * d4 * e1 -
                a2 * c4 * d1 * e3 + a2 * c4 * d3 * e1 + a3 * c1 * d2 * e4 - a3 * c1 * d4 * e2 - a3 * c2 * d1 * e4 +
                a3 * c2 * d4 * e1 + a3 * c4 * d1 * e2 - a3 * c4 * d2 * e1 - a4 * c1 * d2 * e3 + a4 * c1 * d3 * e2 +
                a4 * c2 * d1 * e3 - a4 * c2 * d3 * e1 - a4 * c3 * d1 * e2 + a4 * c3 * d2 * e1) /
              denom;

    A[4][2] = (a1 * b2 * d3 * e4 - a1 * b2 * d4 * e3 - a1 * b3 * d2 * e4 + a1 * b3 * d4 * e2 + a1 * b4 * d2 * e3 -
               a1 * b4 * d3 * e2 - a2 * b1 * d3 * e4 + a2 * b1 * d4 * e3 + a2 * b3 * d1 * e4 - a2 * b3 * d4 * e1 -
               a2 * b4 * d1 * e3 + a2 * b4 * d3 * e1 + a3 * b1 * d2 * e4 - a3 * b1 * d4 * e2 - a3 * b2 * d1 * e4 +
               a3 * b2 * d4 * e1 + a3 * b4 * d1 * e2 - a3 * b4 * d2 * e1 - a4 * b1 * d2 * e3 + a4 * b1 * d3 * e2 +
               a4 * b2 * d1 * e3 - a4 * b2 * d3 * e1 - a4 * b3 * d1 * e2 + a4 * b3 * d2 * e1) /
              denom;

    A[4][3] = -(a1 * b2 * c3 * e4 - a1 * b2 * c4 * e3 - a1 * b3 * c2 * e4 + a1 * b3 * c4 * e2 + a1 * b4 * c2 * e3 -
                a1 * b4 * c3 * e2 - a2 * b1 * c3 * e4 + a2 * b1 * c4 * e3 + a2 * b3 * c1 * e4 - a2 * b3 * c4 * e1 -
                a2 * b4 * c1 * e3 + a2 * b4 * c3 * e1 + a3 * b1 * c2 * e4 - a3 * b1 * c4 * e2 - a3 * b2 * c1 * e4 +
                a3 * b2 * c4 * e1 + a3 * b4 * c1 * e2 - a3 * b4 * c2 * e1 - a4 * b1 * c2 * e3 + a4 * b1 * c3 * e2 +
                a4 * b2 * c1 * e3 - a4 * b2 * c3 * e1 - a4 * b3 * c1 * e2 + a4 * b3 * c2 * e1) /
              denom;

    A[4][4] = (a1 * b2 * c3 * d4 - a1 * b2 * c4 * d3 - a1 * b3 * c2 * d4 + a1 * b3 * c4 * d2 + a1 * b4 * c2 * d3 -
               a1 * b4 * c3 * d2 - a2 * b1 * c3 * d4 + a2 * b1 * c4 * d3 + a2 * b3 * c1 * d4 - a2 * b3 * c4 * d1 -
               a2 * b4 * c1 * d3 + a2 * b4 * c3 * d1 + a3 * b1 * c2 * d4 - a3 * b1 * c4 * d2 - a3 * b2 * c1 * d4 +
               a3 * b2 * c4 * d1 + a3 * b4 * c1 * d2 - a3 * b4 * c2 * d1 - a4 * b1 * c2 * d3 + a4 * b1 * c3 * d2 +
               a4 * b2 * c1 * d3 - a4 * b2 * c3 * d1 - a4 * b3 * c1 * d2 + a4 * b3 * c2 * d1) /
              denom;
}

// ============================================================================
// Implementation of ChMaterialShellANCF methods
// ============================================================================

// Construct an isotropic material.
ChMaterialShellANCF::ChMaterialShellANCF(double rho,  // material density
                                         double E,    // Young's modulus
                                         double nu    // Poisson ratio
                                         )
    : m_rho(rho) {
    double G = 0.5 * E / (1 + nu);
    Calc_E_eps(ChVector<>(E), ChVector<>(nu), ChVector<>(G));
}

// Construct a (possibly) orthotropic material.
ChMaterialShellANCF::ChMaterialShellANCF(double rho,            // material density
                                         const ChVector<>& E,   // elasticity moduli (E_x, E_y, E_z)
                                         const ChVector<>& nu,  // Poisson ratios (nu_xy, nu_xz, nu_yz)
                                         const ChVector<>& G    // shear moduli (G_xy, G_xz, G_yz)
                                         )
    : m_rho(rho) {
    Calc_E_eps(E, nu, G);
}

// Calculate the matrix of elastic coefficients.
// Always assume that the material could be orthotropic
void ChMaterialShellANCF::Calc_E_eps(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G) {
    double delta = 1.0 - (nu.x() * nu.x()) * E.y() / E.x() - (nu.y() * nu.y()) * E.z() / E.x() - (nu.z() * nu.z()) * E.z() / E.y() -
                   2.0 * nu.x() * nu.y() * nu.z() * E.z() / E.x();
    double nu_yx = nu.x() * E.y() / E.x();
    double nu_zx = nu.y() * E.z() / E.x();
    double nu_zy = nu.z() * E.z() / E.y();
    m_E_eps(0, 0) = E.x() * (1.0 - (nu.z() * nu.z()) * E.z() / E.y()) / delta;
    m_E_eps(1, 1) = E.y() * (1.0 - (nu.y() * nu.y()) * E.z() / E.x()) / delta;
    m_E_eps(3, 3) = E.z() * (1.0 - (nu.x() * nu.x()) * E.y() / E.x()) / delta;
    m_E_eps(0, 1) = E.y() * (nu.x() + nu.y() * nu.z() * E.z() / E.y()) / delta;
    m_E_eps(0, 3) = E.z() * (nu.y() + nu.z() * nu.x()) / delta;
    m_E_eps(1, 0) = E.y() * (nu.x() + nu.y() * nu.z() * E.z() / E.y()) / delta;
    m_E_eps(1, 3) = E.z() * (nu.z() + nu.y() * nu.x() * E.y() / E.x()) / delta;
    m_E_eps(3, 0) = E.z() * (nu.y() + nu.z() * nu.x()) / delta;
    m_E_eps(3, 1) = E.z() * (nu.z() + nu.y() * nu.x() * E.y() / E.x()) / delta;
    m_E_eps(2, 2) = G.x();
    m_E_eps(4, 4) = G.y();
    m_E_eps(5, 5) = G.z();
}

// ============================================================================
// Implementation of ChElementShellANCF::Layer methods
// ============================================================================

// Private constructor (a layer can be created only by adding it to an element)
ChElementShellANCF::Layer::Layer(ChElementShellANCF* element,
                                 double thickness,
                                 double theta,
                                 std::shared_ptr<ChMaterialShellANCF> material)
    : m_element(element), m_thickness(thickness), m_theta(theta), m_material(material) {}

// Initial setup for this layer: calculate T0 and detJ0 at the element center.
void ChElementShellANCF::Layer::SetupInitial() {
    // Evaluate shape functions at element center
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
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

    G1xG2.x() = Nx_d0[0][1] * Ny_d0[0][2] - Nx_d0[0][2] * Ny_d0[0][1];
    G1xG2.y() = Nx_d0[0][2] * Ny_d0[0][0] - Nx_d0[0][0] * Ny_d0[0][2];
    G1xG2.z() = Nx_d0[0][0] * Ny_d0[0][1] - Nx_d0[0][1] * Ny_d0[0][0];
    G1dotG1 = Nx_d0[0][0] * Nx_d0[0][0] + Nx_d0[0][1] * Nx_d0[0][1] + Nx_d0[0][2] * Nx_d0[0][2];

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1.x() = Nx_d0[0][0];
    A1.y() = Nx_d0[0][1];
    A1.z() = Nx_d0[0][2];
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
    ChMatrixNM<double, 9, 1> beta;

    j0(0, 0) = Ny_d0[0][1] * Nz_d0[0][2] - Nz_d0[0][1] * Ny_d0[0][2];
    j0(0, 1) = Ny_d0[0][2] * Nz_d0[0][0] - Ny_d0[0][0] * Nz_d0[0][2];
    j0(0, 2) = Ny_d0[0][0] * Nz_d0[0][1] - Nz_d0[0][0] * Ny_d0[0][1];
    j0(1, 0) = Nz_d0[0][1] * Nx_d0[0][2] - Nx_d0[0][1] * Nz_d0[0][2];
    j0(1, 1) = Nz_d0[0][2] * Nx_d0[0][0] - Nx_d0[0][2] * Nz_d0[0][0];
    j0(1, 2) = Nz_d0[0][0] * Nx_d0[0][1] - Nz_d0[0][1] * Nx_d0[0][0];
    j0(2, 0) = Nx_d0[0][1] * Ny_d0[0][2] - Ny_d0[0][1] * Nx_d0[0][2];
    j0(2, 1) = Ny_d0[0][0] * Nx_d0[0][2] - Nx_d0[0][0] * Ny_d0[0][2];
    j0(2, 2) = Nx_d0[0][0] * Ny_d0[0][1] - Ny_d0[0][0] * Nx_d0[0][1];
    j0.MatrDivScale(m_detJ0C);

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
