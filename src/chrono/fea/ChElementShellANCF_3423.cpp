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
// Authors: Bryan Peterson, Milad Rakhsha, Antonio Recuero, Radu Serban
// =============================================================================
// ANCF laminated shell element with four nodes.
// =============================================================================

//// RADU
//// A lot more to do here...
//// - reconsider the use of large static matrices
//// - more use of Eigen expressions
//// - remove unecessary initializations to zero

#include <cmath>

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/core/ChException.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
namespace fea {

// ------------------------------------------------------------------------------
// Static variables
// ------------------------------------------------------------------------------
const double ChElementShellANCF_3423::m_toleranceEAS = 1e-5;
const int ChElementShellANCF_3423::m_maxIterationsEAS = 100;

// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementShellANCF_3423::ChElementShellANCF_3423()
    : m_numLayers(0), m_lenX(0), m_lenY(0), m_thickness(0), m_Alpha(0) {
    m_nodes.resize(4);
}

// ------------------------------------------------------------------------------
// Set element nodes
// ------------------------------------------------------------------------------

void ChElementShellANCF_3423::SetNodes(std::shared_ptr<ChNodeFEAxyzD> nodeA,
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
    m_d0d0T = m_d0 * m_d0.transpose();
}

// -----------------------------------------------------------------------------
// Add a layer.
// -----------------------------------------------------------------------------

void ChElementShellANCF_3423::AddLayer(double thickness, double theta, std::shared_ptr<ChMaterialShellANCF> material) {
    m_layers.push_back(Layer(this, thickness, theta, material));
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellANCF_3423::SetupInitial(ChSystem* system) {
    m_element_dof = 0;
    for (int i = 0; i < 4; i++) {
        m_element_dof += m_nodes[i]->GetNdofX();
    }

    m_full_dof = (m_element_dof == 4 * 6);

    if (!m_full_dof) {
        m_mapping_dof.resize(m_element_dof);
        int dof = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < m_nodes[i]->GetNdofX(); j++)
                m_mapping_dof(dof++) = i * 6 + j;
        }
    }

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
    for (size_t i = 0; i < m_numLayers; i++) {
        m_alphaEAS[i].setZero();
        m_KalphaEAS[i].setZero();
    }

    // Cache the scaling factor (due to change of integration intervals)
    m_GaussScaling = (m_lenX * m_lenY * m_thickness) / 8;

    // Compute mass matrix and gravitational force scaling term (constant)
    ComputeMassMatrix();
    ComputeGravityForceScale();
}

// State update.
void ChElementShellANCF_3423::Update() {
    ChElementGeneric::Update();
}

// Fill the D vector with the current field values at the element nodes.
void ChElementShellANCF_3423::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.segment(0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(6, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(9, 3) = m_nodes[1]->GetD().eigen();
    mD.segment(12, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(15, 3) = m_nodes[2]->GetD().eigen();
    mD.segment(18, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(21, 3) = m_nodes[3]->GetD().eigen();
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
void ChElementShellANCF_3423::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == GetNdofs()) && (H.cols() == GetNdofs()));

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 24; i++)
        for (int j = 0; j < 24; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// Return the mass matrix.
void ChElementShellANCF_3423::ComputeMmatrixGlobal(ChMatrixRef M) {
    M = m_MassMatrix;
}

// -----------------------------------------------------------------------------
// Mass matrix calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the inertia matrix.
class ShellANCF_Mass : public ChIntegrable3D<ChMatrixNM<double, 24, 24>> {
  public:
    ShellANCF_Mass(ChElementShellANCF_3423* element) : m_element(element) {}
    ~ShellANCF_Mass() {}

  private:
    ChElementShellANCF_3423* m_element;

    virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z) override;
};

void ShellANCF_Mass::Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z) {
    ChElementShellANCF_3423::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)]
    ChMatrixNM<double, 3, 24> S;
    ChMatrix33<> Si;
    S.setZero();
    for (int i = 0; i < 8; i++) {
        S(0, 3 * i + 0) = N(i);
        S(1, 3 * i + 1) = N(i);
        S(2, 3 * i + 2) = N(i);
    }

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    // perform  r = S'*S, scaled by integration weights
    result = detJ0 * m_element->m_GaussScaling * S.transpose() * S;
};

void ChElementShellANCF_3423::ComputeMassMatrix() {
    m_MassMatrix.setZero();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        ShellANCF_Mass myformula(this);
        ChMatrixNM<double, 24, 24> TempMassMatrix;
        TempMassMatrix.setZero();
        ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 24>>(TempMassMatrix,  // result of integration will go there
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
void ChElementShellANCF_3423::ComputeNodalMass() {
    m_nodes[0]->m_TotalMass += m_MassMatrix(0, 0) + m_MassMatrix(0, 6) + m_MassMatrix(0, 12) + m_MassMatrix(0, 18);
    m_nodes[1]->m_TotalMass += m_MassMatrix(6, 6) + m_MassMatrix(6, 0) + m_MassMatrix(6, 12) + m_MassMatrix(6, 18);
    m_nodes[2]->m_TotalMass += m_MassMatrix(12, 12) + m_MassMatrix(12, 0) + m_MassMatrix(12, 6) + m_MassMatrix(12, 18);
    m_nodes[3]->m_TotalMass += m_MassMatrix(18, 18) + m_MassMatrix(18, 0) + m_MassMatrix(18, 6) + m_MassMatrix(18, 12);
}
// -----------------------------------------------------------------------------
// Gravitational force calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the element gravity forces
class ShellANCF_Gravity : public ChIntegrable3D<ChElementShellANCF_3423::VectorN> {
  public:
    ShellANCF_Gravity(ChElementShellANCF_3423* element) : m_element(element) {}
    ~ShellANCF_Gravity() {}

  private:
    ChElementShellANCF_3423* m_element;

    virtual void Evaluate(ChElementShellANCF_3423::VectorN& result,
                          const double x,
                          const double y,
                          const double z) override;
};

void ShellANCF_Gravity::Evaluate(ChElementShellANCF_3423::VectorN& result,
                                 const double x,
                                 const double y,
                                 const double z) {
    ChElementShellANCF_3423::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    result = m_element->Calc_detJ0(x, y, z) * m_element->m_GaussScaling * N.transpose();
};

void ChElementShellANCF_3423::ComputeGravityForceScale() {
    m_GravForceScale.setZero();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        ShellANCF_Gravity myformula(this);
        VectorN Fgravity;
        Fgravity.setZero();
        ChQuadrature::Integrate3D<VectorN>(Fgravity,                        // result of integration will go there
                                           myformula,                       // formula to integrate
                                           -1, 1,                           // x limits
                                           -1, 1,                           // y limits
                                           m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                           2                                // order of integration
        );

        m_GravForceScale += rho * Fgravity;
    }
}

// Compute the generalized force vector due to gravity using the efficient ANCF specific method
void ChElementShellANCF_3423::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {
    assert(Fg.size() == GetNdofs());

    // Calculate and add the generalized force due to gravity to the generalized internal force vector for the element.
    // The generalized force due to gravity could be computed once prior to the start of the simulation if gravity was
    // assumed constant throughout the entire simulation.  However, this implementation assumes that the acceleration
    // due to gravity, while a constant for the entire system, can change from step to step which could be useful for
    // gravity loaded units tests as an example.  The generalized force due to gravity is calculated in compact matrix
    // form and is pre-mapped to the desired vector format
    Eigen::Map<MatrixNx3> GravForceCompact(Fg.data(), NSF, 3);
    GravForceCompact = m_GravForceScale * G_acc.eigen().transpose();
}

// -----------------------------------------------------------------------------
// Elastic force calculation
// -----------------------------------------------------------------------------

// The class ShellANCF_Force provides the integrand for the calculation of the internal forces
// for one layer of an ANCF shell element.
// The first 24 entries in the integrand represent the internal force.
// The next 5 entries represent the residual of the EAS nonlinear system.
// The last 25 entries represent the 5x5 Jacobian of the EAS nonlinear system.
// Capabilities of this class include: application of enhanced assumed strain (EAS) and
// assumed natural strain (ANS) formulations to avoid thickness and (transverse and in-plane)
// shear locking. This implementation also features a composite material implementation
// that allows for selecting a number of layers over the element thickness; each of which
// has an independent, user-selected fiber angle (direction for orthotropic constitutive behavior)
class ShellANCF_Force : public ChIntegrable3D<ChVectorN<double, 54>> {
  public:
    ShellANCF_Force(ChElementShellANCF_3423* element,  // Containing element
                    size_t kl,                         // Current layer index
                    ChVectorN<double, 5>* alpha_eas    // Vector of internal parameters for EAS formulation
                    )
        : m_element(element), m_kl(kl), m_alpha_eas(alpha_eas) {}
    ~ShellANCF_Force() {}

  private:
    ChElementShellANCF_3423* m_element;
    size_t m_kl;
    ChVectorN<double, 5>* m_alpha_eas;

    /// Evaluate (strainD'*strain)  at point x, include ANS and EAS.
    virtual void Evaluate(ChVectorN<double, 54>& result, const double x, const double y, const double z) override;
};

void ShellANCF_Force::Evaluate(ChVectorN<double, 54>& result, const double x, const double y, const double z) {
    // Element shape function
    ChElementShellANCF_3423::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChElementShellANCF_3423::ShapeVector Nx;
    ChElementShellANCF_3423::ShapeVector Ny;
    ChElementShellANCF_3423::ShapeVector Nz;
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
    const ChMatrixNM<double, 6, 6>& T0 = m_element->GetLayer(m_kl).Get_T0();
    // Determinant of the initial position vector gradient at the element center
    double detJ0C = m_element->GetLayer(m_kl).Get_detJ0C();

    // Enhanced Assumed Strain
    ChMatrixNM<double, 6, 5> G = T0 * M * (detJ0C / detJ0);
    ChVectorN<double, 6> strain_EAS = G * (*m_alpha_eas);

    ChVectorN<double, 8> ddNx = m_element->m_ddT * Nx.transpose();
    ChVectorN<double, 8> ddNy = m_element->m_ddT * Ny.transpose();

    ChVectorN<double, 8> d0d0Nx = m_element->m_d0d0T * Nx.transpose();
    ChVectorN<double, 8> d0d0Ny = m_element->m_d0d0T * Ny.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
    strain_til(3) = N(0) * m_element->m_strainANS(0) + N(2) * m_element->m_strainANS(1) +
                    N(4) * m_element->m_strainANS(2) + N(6) * m_element->m_strainANS(3);
    strain_til(4) = S_ANS(0, 2) * m_element->m_strainANS(6) + S_ANS(0, 3) * m_element->m_strainANS(7);
    strain_til(5) = S_ANS(0, 0) * m_element->m_strainANS(4) + S_ANS(0, 1) * m_element->m_strainANS(5);

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

    ChMatrixNM<double, 6, 24> strainD_til;

    ChMatrixNM<double, 1, 24> tempB;
    ChMatrixNM<double, 1, 3> tempB3;
    ChMatrixNM<double, 1, 3> tempB31;

    tempB3 = Nx * m_element->m_d;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(0) = tempB;

    tempB3 = Ny * m_element->m_d;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(1) = tempB;

    tempB31 = Nx * m_element->m_d;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i) + tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(2) = tempB;

    tempB.setZero();
    for (int i = 0; i < 4; i++) {
        tempB += N(i * 2) * m_element->m_strainANS_D.row(i);
    }
    strainD_til.row(3) = tempB;  // strainD for zz

    tempB.setZero();
    for (int i = 0; i < 2; i++) {
        tempB += S_ANS(0, i + 2) * m_element->m_strainANS_D.row(i + 6);
    }
    strainD_til.row(4) = tempB;  // strainD for xz

    tempB.setZero();
    for (int i = 0; i < 2; i++) {
        tempB += S_ANS(0, i) * m_element->m_strainANS_D.row(i + 4);
    }
    strainD_til.row(5) = tempB;  // strainD for yz

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
    ChVectorN<double, 6> DEPS;
    DEPS.setZero();
    for (int ii = 0; ii < 24; ii++) {
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
    ChVectorN<double, 24> Fint = (strainD.transpose() * E_eps * strain) * (detJ0 * m_element->m_GaussScaling);

    // EAS terms
    ChMatrixNM<double, 5, 6> temp56 = G.transpose() * E_eps;
    ChVectorN<double, 5> HE = (temp56 * strain) * (detJ0 * m_element->m_GaussScaling);     // EAS residual
    ChMatrixNM<double, 5, 5> KALPHA = (temp56 * G) * (detJ0 * m_element->m_GaussScaling);  // EAS Jacobian

    /// Total result vector
    result.segment(0, 24) = Fint;
    result.segment(24, 5) = HE;
    result.segment(29, 5 * 5) = Eigen::Map<ChVectorN<double, 5 * 5>>(KALPHA.data(), 5 * 5);
}

void ChElementShellANCF_3423::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    // Current nodal coordinates and velocities
    CalcCoordMatrix(m_d);
    CalcCoordDerivMatrix(m_d_dt);
    m_ddT = m_d * m_d.transpose();
    // Assumed Natural Strain (ANS):  Calculate m_strainANS and m_strainANS_D
    CalcStrainANSbilinearShell();

    Fi.setZero();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        ChVectorN<double, 24> Finternal;
        ChVectorN<double, 5> HE;
        ChMatrixNM<double, 5, 5> KALPHA;

        // Initial guess for EAS parameters
        ChVectorN<double, 5> alphaEAS = m_alphaEAS[kl];

        // Newton loop for EAS
        for (int count = 0; count < m_maxIterationsEAS; count++) {
            ShellANCF_Force formula(this, kl, &alphaEAS);
            ChVectorN<double, 54> result;
            result.setZero();
            ChQuadrature::Integrate3D<ChVectorN<double, 54>>(result,                          // result of integration
                                                             formula,                         // integrand formula
                                                             -1, 1,                           // x limits
                                                             -1, 1,                           // y limits
                                                             m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                             2                                // order of integration
            );

            // Extract vectors and matrices from result of integration
            Finternal = result.segment(0, 24);
            HE = result.segment(24, 5);
            KALPHA = Eigen::Map<ChMatrixNM<double, 5, 5>>(result.segment(29, 5 * 5).data(), 5, 5);

            // Check convergence (residual check)
            double norm_HE = HE.norm();
            if (norm_HE < m_toleranceEAS)
                break;

            // Calculate increment and update EAS parameters
            ChVectorN<double, 5> sol = KALPHA.colPivHouseholderQr().solve(HE);
            alphaEAS -= sol;

            if (count >= 2)
                GetLog() << "  count " << count << "  NormHE " << norm_HE << "\n";
        }

        // Accumulate internal force
        Fi -= Finternal;

        // Cache alphaEAS and KALPHA for use in Jacobian calculation
        m_alphaEAS[kl] = alphaEAS;
        m_KalphaEAS[kl] = KALPHA;

    }  // Layer Loop
}

// -----------------------------------------------------------------------------
// Jacobians of internal forces
// -----------------------------------------------------------------------------

// The class ShellANCF_Jacobian provides the integrand for the calculation of the Jacobians
// (stiffness and damping matrices) of the internal forces for one layer of an ANCF
// shell element.
// The first 576 entries in the integrated vector represent the 24x24 Jacobian
//      Kfactor * [K] + Rfactor * [R]
// where K does not include the EAS contribution.
// The last 120 entries represent the 5x24 cross-dependency matrix.
class ShellANCF_Jacobian : public ChIntegrable3D<ChVectorN<double, 696>> {
  public:
    ShellANCF_Jacobian(ChElementShellANCF_3423* element,  // Containing element
                       double Kfactor,                    // Scaling coefficient for stiffness component
                       double Rfactor,                    // Scaling coefficient for damping component
                       size_t kl                          // Current layer index
                       )
        : m_element(element), m_Kfactor(Kfactor), m_Rfactor(Rfactor), m_kl(kl) {}

  private:
    ChElementShellANCF_3423* m_element;
    double m_Kfactor;
    double m_Rfactor;
    size_t m_kl;

    // Evaluate integrand at the specified point.
    virtual void Evaluate(ChVectorN<double, 696>& result, const double x, const double y, const double z) override;
};

void ShellANCF_Jacobian::Evaluate(ChVectorN<double, 696>& result, const double x, const double y, const double z) {
    // Element shape function
    ChElementShellANCF_3423::ShapeVector N;
    m_element->ShapeFunctions(N, x, y, z);

    // Determinant of position vector gradient matrix: Initial configuration
    ChElementShellANCF_3423::ShapeVector Nx;
    ChElementShellANCF_3423::ShapeVector Ny;
    ChElementShellANCF_3423::ShapeVector Nz;
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

    // Transformation matrix, function of fiber angle
    const ChMatrixNM<double, 6, 6>& T0 = m_element->GetLayer(m_kl).Get_T0();
    // Determinant of the initial position vector gradient at the element center
    double detJ0C = m_element->GetLayer(m_kl).Get_detJ0C();

    // Enhanced Assumed Strain
    ChMatrixNM<double, 6, 5> G = T0 * M * (detJ0C / detJ0);
    ChVectorN<double, 6> strain_EAS = G * m_element->m_alphaEAS[m_kl];

    ChVectorN<double, 8> ddNx = m_element->m_ddT * Nx.transpose();
    ChVectorN<double, 8> ddNy = m_element->m_ddT * Ny.transpose();

    ChVectorN<double, 8> d0d0Nx = m_element->m_d0d0T * Nx.transpose();
    ChVectorN<double, 8> d0d0Ny = m_element->m_d0d0T * Ny.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
    strain_til(3) = N(0) * m_element->m_strainANS(0) + N(2) * m_element->m_strainANS(1) +
                    N(4) * m_element->m_strainANS(2) + N(6) * m_element->m_strainANS(3);
    strain_til(4) = S_ANS(0, 2) * m_element->m_strainANS(6) + S_ANS(0, 3) * m_element->m_strainANS(7);
    strain_til(5) = S_ANS(0, 0) * m_element->m_strainANS(4) + S_ANS(0, 1) * m_element->m_strainANS(5);

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

    ChMatrixNM<double, 6, 24> strainD_til;
    strainD_til.setZero();

    ChMatrixNM<double, 1, 24> tempB;
    ChMatrixNM<double, 1, 3> tempB3;
    ChMatrixNM<double, 1, 3> tempB31;

    tempB3 = Nx * m_element->m_d;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.row(0) = tempB;

    tempB3 = Ny * m_element->m_d;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(1) = tempB;

    tempB31 = Nx * m_element->m_d;
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i) + tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.row(2) = tempB;

    tempB.setZero();
    for (int i = 0; i < 4; i++) {
        tempB += N(i * 2) * m_element->m_strainANS_D.row(i);
    }
    strainD_til.row(3) = tempB;  // strainD for zz

    tempB.setZero();
    for (int i = 0; i < 2; i++) {
        tempB += S_ANS(0, i + 2) * m_element->m_strainANS_D.row(i + 6);
    }
    strainD_til.row(4) = tempB;  // strainD for xz

    tempB.setZero();
    for (int i = 0; i < 2; i++) {
        tempB += S_ANS(0, i) * m_element->m_strainANS_D.row(i + 4);
    }
    strainD_til.row(5) = tempB;  // strainD for yz

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
    Gd.setZero();

    for (int ii = 0; ii < 8; ii++) {
        Gd(0, 3 * ii) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
        Gd(1, 3 * ii + 1) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);
        Gd(2, 3 * ii + 2) = j0(0, 0) * Nx(0, ii) + j0(1, 0) * Ny(0, ii) + j0(2, 0) * Nz(0, ii);

        Gd(3, 3 * ii) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
        Gd(4, 3 * ii + 1) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);
        Gd(5, 3 * ii + 2) = j0(0, 1) * Nx(0, ii) + j0(1, 1) * Ny(0, ii) + j0(2, 1) * Nz(0, ii);

        Gd(6, 3 * ii) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
        Gd(7, 3 * ii + 1) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
        Gd(8, 3 * ii + 2) = j0(0, 2) * Nx(0, ii) + j0(1, 2) * Ny(0, ii) + j0(2, 2) * Nz(0, ii);
    }

    // Enhanced Assumed Strain 2nd
    strain += strain_EAS;

    // Structural damping
    // Strain time derivative for structural damping
    ChVectorN<double, 6> DEPS;
    DEPS.setZero();
    for (int ii = 0; ii < 24; ii++) {
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
    ChMatrixNM<double, 24, 24> KTE;
    KTE = (strainD.transpose() * E_eps * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) +
          (Gd.transpose() * Sigm * Gd) * m_Kfactor;
    KTE *= detJ0 * m_element->m_GaussScaling;

    // EAS cross-dependency matrix.
    ChMatrixNM<double, 5, 24> GDEPSP = (G.transpose() * E_eps * strainD) * (detJ0 * m_element->m_GaussScaling);

    // Load result vector (integrand)
    result.segment(0, 24 * 24) = Eigen::Map<ChVectorN<double, 24 * 24>>(KTE.data(), 24 * 24);
    result.segment(576, 5 * 24) = Eigen::Map<ChVectorN<double, 5 * 24>>(GDEPSP.data(), 5 * 24);
}

void ChElementShellANCF_3423::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    // Note that the matrices with current nodal coordinates and velocities are
    // already available in m_d and m_d_dt (as set in ComputeInternalForces).
    // Similarly, the ANS strain and strain derivatives are already available in
    // m_strainANS and m_strainANS_D (as calculated in ComputeInternalForces).

    m_JacobianMatrix.setZero();

    // Loop over all layers.
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        ShellANCF_Jacobian formula(this, Kfactor, Rfactor, kl);
        ChVectorN<double, 696> result;
        result.setZero();
        ChQuadrature::Integrate3D<ChVectorN<double, 696>>(result,                          // result of integration
                                                          formula,                         // integrand formula
                                                          -1, 1,                           // x limits
                                                          -1, 1,                           // y limits
                                                          m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                          2                                // order of integration
        );

        // Extract matrices from result of integration
        ChMatrixNM<double, 24, 24> KTE;
        ChMatrixNM<double, 5, 24> GDEPSP;
        KTE = Eigen::Map<ChMatrixNM<double, 24, 24>>(result.segment(0, 24 * 24).data(), 24, 24);
        GDEPSP = Eigen::Map<ChMatrixNM<double, 5, 24>>(result.segment(576, 5 * 24).data(), 5, 24);

        // Include EAS contribution to the stiffness component (hence scaled by Kfactor)
        // EAS = GDEPSP' * KalphaEAS_inv * GDEPSP
        ChMatrixNM<double, 5, 5> KalphaEAS_inv = m_KalphaEAS[kl].inverse();
        m_JacobianMatrix += KTE - Kfactor * GDEPSP.transpose() * KalphaEAS_inv * GDEPSP;
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementShellANCF_3423::ShapeFunctions(ShapeVector& N, double x, double y, double z) {
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

void ChElementShellANCF_3423::ShapeFunctionsDerivativeX(ShapeVector& Nx, double x, double y, double z) {
    double a = GetLengthX();
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

void ChElementShellANCF_3423::ShapeFunctionsDerivativeY(ShapeVector& Ny, double x, double y, double z) {
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

void ChElementShellANCF_3423::ShapeFunctionsDerivativeZ(ShapeVector& Nz, double x, double y, double z) {
    Nz(0) = 0.0;
    Nz(1) = 0.250 * (1.0 - x) * (1.0 - y);
    Nz(2) = 0.0;
    Nz(3) = 0.250 * (1.0 + x) * (1.0 - y);
    Nz(4) = 0.0;
    Nz(5) = 0.250 * (1.0 + x) * (1.0 + y);
    Nz(6) = 0.0;
    Nz(7) = 0.250 * (1.0 - x) * (1.0 + y);
}

void ChElementShellANCF_3423::Basis_M(ChMatrixNM<double, 6, 5>& M, double x, double y, double z) {
    M.setZero();
    M(0, 0) = x;
    M(1, 1) = y;
    M(2, 2) = x;
    M(2, 3) = y;
    M(3, 4) = z;
}

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------
double ChElementShellANCF_3423::Calc_detJ0(double x,
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

double ChElementShellANCF_3423::Calc_detJ0(double x, double y, double z) {
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;

    return Calc_detJ0(x, y, z, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);
}

void ChElementShellANCF_3423::CalcCoordMatrix(ChMatrixNM<double, 8, 3>& d) {
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

void ChElementShellANCF_3423::CalcCoordDerivMatrix(ChVectorN<double, 24>& dt) {
    const ChVector<>& pA_dt = m_nodes[0]->GetPos_dt();
    const ChVector<>& dA_dt = m_nodes[0]->GetD_dt();
    const ChVector<>& pB_dt = m_nodes[1]->GetPos_dt();
    const ChVector<>& dB_dt = m_nodes[1]->GetD_dt();
    const ChVector<>& pC_dt = m_nodes[2]->GetPos_dt();
    const ChVector<>& dC_dt = m_nodes[2]->GetD_dt();
    const ChVector<>& pD_dt = m_nodes[3]->GetPos_dt();
    const ChVector<>& dD_dt = m_nodes[3]->GetD_dt();

    dt(0) = pA_dt.x();
    dt(1) = pA_dt.y();
    dt(2) = pA_dt.z();
    dt(3) = dA_dt.x();
    dt(4) = dA_dt.y();
    dt(5) = dA_dt.z();

    dt(6) = pB_dt.x();
    dt(7) = pB_dt.y();
    dt(8) = pB_dt.z();
    dt(9) = dB_dt.x();
    dt(10) = dB_dt.y();
    dt(11) = dB_dt.z();

    dt(12) = pC_dt.x();
    dt(13) = pC_dt.y();
    dt(14) = pC_dt.z();
    dt(15) = dC_dt.x();
    dt(16) = dC_dt.y();
    dt(17) = dC_dt.z();

    dt(18) = pD_dt.x();
    dt(19) = pD_dt.y();
    dt(20) = pD_dt.z();
    dt(21) = dD_dt.x();
    dt(22) = dD_dt.y();
    dt(23) = dD_dt.z();
}

// -----------------------------------------------------------------------------
// Assumed Natural Strain
// -----------------------------------------------------------------------------

// ANS shape function (interpolation of strain and strainD in thickness direction)
void ChElementShellANCF_3423::ShapeFunctionANSbilinearShell(ChMatrixNM<double, 1, 4>& S_ANS, double x, double y) {
    S_ANS(0, 0) = -0.5 * x + 0.5;
    S_ANS(0, 1) = 0.5 * x + 0.5;
    S_ANS(0, 2) = -0.5 * y + 0.5;
    S_ANS(0, 3) = 0.5 * y + 0.5;
}

// Calculate ANS strain and its Jacobian
void ChElementShellANCF_3423::CalcStrainANSbilinearShell() {
    std::vector<ChVector<>> knots(8);

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
    ChVectorN<double, 8> ddNz;
    ChVectorN<double, 8> d0d0Nz;

    for (int kk = 0; kk < 8; kk++) {
        ShapeFunctionsDerivativeX(Nx, knots[kk].x(), knots[kk].y(), knots[kk].z());
        ShapeFunctionsDerivativeY(Ny, knots[kk].x(), knots[kk].y(), knots[kk].z());
        ShapeFunctionsDerivativeZ(Nz, knots[kk].x(), knots[kk].y(), knots[kk].z());

        ddNz = m_ddT * Nz.transpose();
        d0d0Nz = m_d0d0T * Nz.transpose();

        switch (kk) {
            case 0:
            case 1:
            case 2:
            case 3: {
                m_strainANS(kk) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));
                ChMatrixNM<double, 1, 3> tmpZ = Nz * m_d;
                for (int i = 0; i < 8; i++)
                    for (int j = 0; j < 3; j++)
                        m_strainANS_D(kk, i * 3 + j) = tmpZ(0, j) * Nz(0, i);
                break;
            }
            case 4:
            case 5: {  // => yz
                m_strainANS(kk) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);
                ChMatrixNM<double, 1, 3> tmpY = Ny * m_d;
                ChMatrixNM<double, 1, 3> tmpZ = Nz * m_d;
                for (int i = 0; i < 8; i++)
                    for (int j = 0; j < 3; j++)
                        m_strainANS_D(kk, i * 3 + j) = tmpY(0, j) * Nz(0, i) + tmpZ(0, j) * Ny(0, i);
                break;
            }
            case 6:
            case 7: {  // => xz
                m_strainANS(kk) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);
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
void ChElementShellANCF_3423::EvaluateDeflection(double& def) {
    ChVector<> oldPos;
    ChVector<> newPos;
    ChVector<> defVec;

    for (int i = 0; i < 4; i++) {
        oldPos.x() += this->m_d0(2 * i, 0);
        oldPos.y() += this->m_d0(2 * i, 1);
        oldPos.z() += this->m_d0(2 * i, 2);
    }

    for (int i = 0; i < 4; i++) {
        newPos.x() += this->m_d(2 * i, 0);
        newPos.y() += this->m_d(2 * i, 1);
        newPos.z() += this->m_d(2 * i, 2);
    }

    defVec = (newPos - oldPos) / 4;
    def = defVec.Length();
}

ChStrainStress3D ChElementShellANCF_3423::EvaluateSectionStrainStress(const ChVector<>& loc, int layer_id) {
    // Element shape function
    ShapeVector N;
    ShapeFunctions(N, loc.x(), loc.y(), loc.z());

    // Determinant of position vector gradient matrix: Initial configuration
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = this->Calc_detJ0(loc.x(), loc.y(), loc.z(), Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

    // ANS shape function
    ChMatrixNM<double, 1, 4> S_ANS;  // Shape function vector for Assumed Natural Strain
    ChMatrixNM<double, 6, 5> M;      // Shape function vector for Enhanced Assumed Strain
    this->ShapeFunctionANSbilinearShell(S_ANS, loc.x(), loc.y());
    this->Basis_M(M, loc.x(), loc.y(), loc.z());

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

    ChVectorN<double, 8> ddNx = m_ddT * Nx.transpose();
    ChVectorN<double, 8> ddNy = m_ddT * Ny.transpose();

    ChVectorN<double, 8> d0d0Nx = m_d0d0T * Nx.transpose();
    ChVectorN<double, 8> d0d0Ny = m_d0d0T * Ny.transpose();

    // Strain component
    ChVectorN<double, 6> strain_til;
    strain_til(0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);
    strain_til(3) = N(0) * this->m_strainANS(0) + N(2) * this->m_strainANS(1) + N(4) * this->m_strainANS(2) +
                    N(6) * this->m_strainANS(3);
    strain_til(4) = S_ANS(0, 2) * this->m_strainANS(6) + S_ANS(0, 3) * this->m_strainANS(7);
    strain_til(5) = S_ANS(0, 0) * this->m_strainANS(4) + S_ANS(0, 1) * this->m_strainANS(5);

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
    ChStrainStress3D strainStressOut = {strain, stress};

    return strainStressOut;
}
void ChElementShellANCF_3423::EvaluateSectionDisplacement(const double u,
                                                          const double v,
                                                          ChVector<>& u_displ,
                                                          ChVector<>& u_rotaz) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, u_displ);
    u_rotaz = VNULL;  // no angles.. this is ANCF (or maybe return here the slope derivatives?)
}

void ChElementShellANCF_3423::EvaluateSectionFrame(const double u,
                                                   const double v,
                                                   ChVector<>& point,
                                                   ChQuaternion<>& rot) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, point);

    MatrixNx3 e_bar;
    CalcCoordMatrix(e_bar);

    ShapeVector Nx;
    ShapeVector Ny;
    ShapeFunctionsDerivativeX(Nx, u, v, 0);
    ShapeFunctionsDerivativeY(Ny, u, v, 0);

    // Since ANCF does not use rotations, calculate an approximate
    // rotation based off the position vector gradients
    ChVector<double> MidsurfaceX = e_bar.transpose() * Nx.transpose();
    ChVector<double> MidsurfaceY = e_bar.transpose() * Ny.transpose();

    // Since the position vector gradients are not in general orthogonal,
    // set the Dx direction tangent to the shell xi axis and
    // compute the Dy and Dz directions by using a
    // Gram-Schmidt orthonormalization, guided by the shell eta axis
    ChMatrix33<> msect;
    msect.Set_A_Xdir(MidsurfaceX, MidsurfaceY);

    rot = msect.Get_A_quaternion();
}

void ChElementShellANCF_3423::EvaluateSectionPoint(const double u, const double v, ChVector<>& point) {
    ChVector<> u_displ;

    double x = u;  // because ShapeFunctions() works in -1..1 range
    double y = v;  // because ShapeFunctions() works in -1..1 range
    double z = 0;
    ShapeVector N;
    ShapeFunctions(N, x, y, z);

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
void ChElementShellANCF_3423::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetD().eigen();
    mD.segment(block_offset + 12, 3) = m_nodes[2]->GetPos().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[2]->GetD().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[3]->GetPos().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[3]->GetD().eigen();
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChElementShellANCF_3423::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = m_nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = m_nodes[0]->GetD_dt().eigen();
    mD.segment(block_offset + 6, 3) = m_nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = m_nodes[1]->GetD_dt().eigen();
    mD.segment(block_offset + 12, 3) = m_nodes[2]->GetPos_dt().eigen();
    mD.segment(block_offset + 15, 3) = m_nodes[2]->GetD_dt().eigen();
    mD.segment(block_offset + 18, 3) = m_nodes[3]->GetPos_dt().eigen();
    mD.segment(block_offset + 21, 3) = m_nodes[3]->GetD_dt().eigen();
}

void ChElementShellANCF_3423::LoadableStateIncrement(const unsigned int off_x,
                                                     ChState& x_new,
                                                     const ChState& x,
                                                     const unsigned int off_v,
                                                     const ChStateDelta& Dv) {
    for (int i = 0; i < 4; i++) {
        this->m_nodes[i]->NodeIntStateIncrement(off_x + 6 * i, x_new, x, off_v + 6 * i, Dv);
    }
}

void ChElementShellANCF_3423::EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) {
    ShapeVector N;
    ShapeFunctions(N, U, V, 0);
    for (unsigned int ii = 0; ii < 4; ii++) {
        Result += N(ii * 2) * m_nodes[ii]->GetPos_dt();
        Result += N(ii * 2 + 1) * m_nodes[ii]->GetPos_dt();
    }
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChElementShellANCF_3423::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
        mvars.push_back(&m_nodes[i]->Variables_D());
    }
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
// This calculation takes a slightly different form for ANCF elements
// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
// the applied force in global coordinates and the second 3 entries is the applied moment in global space.
void ChElementShellANCF_3423::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ShapeVector N;
    ShapeFunctions(N, U, V, 0);

    Eigen::Map<MatrixNx3> QiCompact(Qi.data(), NSF, 3);
    QiCompact = N.transpose() * F.segment(0, 3).transpose();

    // Compute the generalized force vector for the applied moment component
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ShapeFunctionsDerivativeX(Nx, U, V, 0);
    ShapeFunctionsDerivativeY(Ny, U, V, 0);
    ShapeFunctionsDerivativeZ(Nz, U, V, 0);

    // Change the shape function derivatives with respect to the element local "x", "y", or "z" coordinates to be with
    // respect to the normalized coordinates "xi", "eta", and "zeta" when loading the results into the Sxi_D matrix
    MatrixNx3 Sxi_D;
    Sxi_D.col(0) = Nx * m_lenX / 2;
    Sxi_D.col(1) = Ny * m_lenY / 2;
    Sxi_D.col(2) = Nz * m_thickness / 2;

    MatrixNx3 e_bar;
    CalcCoordMatrix(e_bar);

    // Calculate the element Jacobian between the current configuration and the normalized configuration
    ChMatrix33<double> J_Cxi = e_bar.transpose() * Sxi_D;
    ChMatrix33<double> J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    VectorN G_A = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 0) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 0) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 0);
    VectorN G_B = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 1) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 1) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 1);
    VectorN G_C = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 2) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 2) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 2);

    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    for (unsigned int i = 0; i < NSF; i++) {
        Qi(3 * i + 0) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi(3 * i + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi(3 * i + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }

    // Compute the element Jacobian between the current configuration and the normalized configuration
    // This is different than the element Jacobian between the reference configuration and the normalized
    //  configuration used in the internal force calculations.  For this calculation, this is the ratio between the
    //  actual differential area and the normalized differential area.  The cross product is
    //  used to calculate this area ratio for potential use in Gauss-Quadrature or similar numeric integration.
    detJ = J_Cxi.col(0).cross(J_Cxi.col(1)).norm();
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V,W) coordinates of the surface.
// This calculation takes a slightly different form for ANCF elements
// For this ANCF element, only the first 6 entries in F are used in the calculation.  The first three entries is
// the applied force in global coordinates and the second 3 entries is the applied moment in global space.
void ChElementShellANCF_3423::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ShapeVector N;
    ShapeFunctions(N, U, V, W);

    Eigen::Map<MatrixNx3> QiCompact(Qi.data(), NSF, 3);
    QiCompact = N.transpose() * F.segment(0, 3).transpose();

    // Compute the generalized force vector for the applied moment component
    // See: Antonio M Recuero, Javier F Aceituno, Jose L Escalona, and Ahmed A Shabana.
    // A nonlinear approach for modeling rail flexibility using the absolute nodal coordinate
    // formulation. Nonlinear Dynamics, 83(1-2):463-481, 2016.
    ShapeVector Nx;
    ShapeVector Ny;
    ShapeVector Nz;
    ShapeFunctionsDerivativeX(Nx, U, V, W);
    ShapeFunctionsDerivativeY(Ny, U, V, W);
    ShapeFunctionsDerivativeZ(Nz, U, V, W);

    // Change the shape function derivatives with respect to the element local "x", "y", or "z" coordinates to be with
    // respect to the normalized coordinates "xi", "eta", and "zeta" when loading the results into the Sxi_D matrix
    MatrixNx3 Sxi_D;
    Sxi_D.col(0) = Nx * m_lenX / 2;
    Sxi_D.col(1) = Ny * m_lenY / 2;
    Sxi_D.col(2) = Nz * m_thickness / 2;

    MatrixNx3 e_bar;
    CalcCoordMatrix(e_bar);

    // Calculate the element Jacobian between the current configuration and the normalized configuration
    ChMatrix33<double> J_Cxi = e_bar.transpose() * Sxi_D;
    ChMatrix33<double> J_Cxi_Inv = J_Cxi.inverse();

    // Compute the unique pieces that make up the moment projection matrix "G"
    VectorN G_A = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 0) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 0) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 0);
    VectorN G_B = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 1) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 1) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 1);
    VectorN G_C = Sxi_D.col(0).transpose() * J_Cxi_Inv(0, 2) + Sxi_D.col(1).transpose() * J_Cxi_Inv(1, 2) +
                  Sxi_D.col(2).transpose() * J_Cxi_Inv(2, 2);

    ChVectorN<double, 3> M_scaled = 0.5 * F.segment(3, 3);

    // Compute G'M without actually forming the complete matrix "G" (since it has a sparsity pattern to it)
    for (unsigned int i = 0; i < NSF; i++) {
        Qi(3 * i + 0) += M_scaled(1) * G_C(i) - M_scaled(2) * G_B(i);
        Qi(3 * i + 1) += M_scaled(2) * G_A(i) - M_scaled(0) * G_C(i);
        Qi(3 * i + 2) += M_scaled(0) * G_B(i) - M_scaled(1) * G_A(i);
    }

    // Compute the element Jacobian between the current configuration and the normalized configuration
    // This is different than the element Jacobian between the reference configuration and the normalized
    //  configuration used in the internal force calculations.  For this calculation, this is the ratio between the
    //  actual differential volume and the normalized differential volume.  The determinate of the element Jacobian is
    //  used to calculate this volume ratio for potential use in Gauss-Quadrature or similar numeric integration.
    detJ = J_Cxi.determinant();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

// Calculate average element density (needed for ChLoaderVolumeGravity).
double ChElementShellANCF_3423::GetDensity() {
    double tot_density = 0;
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        double layerthick = m_layers[kl].Get_thickness();
        tot_density += rho * layerthick;
    }
    return tot_density / m_thickness;
}

// Calculate normal to the surface at (U,V) coordinates.
ChVector<> ChElementShellANCF_3423::ComputeNormal(const double U, const double V) {
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

// ============================================================================
// Implementation of ChElementShellANCF_3423::Layer methods
// ============================================================================

// Private constructor (a layer can be created only by adding it to an element)
ChElementShellANCF_3423::Layer::Layer(ChElementShellANCF_3423* element,
                                      double thickness,
                                      double theta,
                                      std::shared_ptr<ChMaterialShellANCF> material)
    : m_element(element), m_thickness(thickness), m_theta(theta), m_material(material) {}

// Initial setup for this layer: calculate T0 and detJ0 at the element center.
void ChElementShellANCF_3423::Layer::SetupInitial() {
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
