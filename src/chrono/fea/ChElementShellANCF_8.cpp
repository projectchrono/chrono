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
// Authors: Antonio Recuero
// =============================================================================
// ANCF laminated shell element with eight nodes: High-Order.
// Element 3833 of paper: 'Analysis of higher-order quadrilateral plate elements
// based on the absolute nodal coordinate formulation for three-dimensional
// elasticity'
// H.C.J. Ebel, M.K.Matikainen, V.V.T. Hurskainen, A.M.Mikkola, Multibody System
// Dynamics, To be published, 2017
// =============================================================================

#include <cmath>

#include "chrono/fea/ChElementShellANCF_8.h"
#include "chrono/core/ChException.h"
#include "chrono/core/ChQuadrature.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChUtilsFEA.h"

namespace chrono {
namespace fea {

// ------------------------------------------------------------------------------
// Constructor
// ------------------------------------------------------------------------------

ChElementShellANCF_8::ChElementShellANCF_8()
    : m_gravity_on(false), m_numLayers(0), m_thickness(0), m_lenX(0), m_lenY(0), m_Alpha(0) {
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
    m_d0d0T.MatrMultiplyT(m_d0, m_d0);
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
void ChElementShellANCF_8::GetStateBlock(ChMatrixDynamic<>& mDD) {
    mDD.PasteVector(m_nodes[0]->GetPos(), 0, 0);
    mDD.PasteVector(m_nodes[0]->GetD(), 3, 0);
    mDD.PasteVector(m_nodes[0]->GetDD(), 6, 0);
    mDD.PasteVector(m_nodes[1]->GetPos(), 9, 0);
    mDD.PasteVector(m_nodes[1]->GetD(), 12, 0);
    mDD.PasteVector(m_nodes[1]->GetDD(), 15, 0);
    mDD.PasteVector(m_nodes[2]->GetPos(), 18, 0);
    mDD.PasteVector(m_nodes[2]->GetD(), 21, 0);
    mDD.PasteVector(m_nodes[2]->GetDD(), 24, 0);
    mDD.PasteVector(m_nodes[3]->GetPos(), 27, 0);
    mDD.PasteVector(m_nodes[3]->GetD(), 30, 0);
    mDD.PasteVector(m_nodes[3]->GetDD(), 33, 0);
    mDD.PasteVector(m_nodes[4]->GetPos(), 36, 0);
    mDD.PasteVector(m_nodes[4]->GetD(), 39, 0);
    mDD.PasteVector(m_nodes[4]->GetDD(), 42, 0);
    mDD.PasteVector(m_nodes[5]->GetPos(), 45, 0);
    mDD.PasteVector(m_nodes[5]->GetD(), 48, 0);
    mDD.PasteVector(m_nodes[5]->GetDD(), 51, 0);
    mDD.PasteVector(m_nodes[6]->GetPos(), 54, 0);
    mDD.PasteVector(m_nodes[6]->GetD(), 57, 0);
    mDD.PasteVector(m_nodes[6]->GetDD(), 60, 0);
    mDD.PasteVector(m_nodes[7]->GetPos(), 63, 0);
    mDD.PasteVector(m_nodes[7]->GetD(), 66, 0);
    mDD.PasteVector(m_nodes[7]->GetDD(), 69, 0);
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
void ChElementShellANCF_8::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 72) && (H.GetColumns() == 72));

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 72; i++)
        for (int j = 0; j < 72; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// Return the mass matrix.
void ChElementShellANCF_8::ComputeMmatrixGlobal(ChMatrix<>& M) {
    M = m_MassMatrix;
}

// -----------------------------------------------------------------------------
// Mass matrix calculation
// -----------------------------------------------------------------------------

/// This class defines the calculations for the integrand of the inertia matrix.
class MyMass_8 : public ChIntegrable3D<ChMatrixNM<double, 72, 72> > {
  public:
    MyMass_8(ChElementShellANCF_8* element) : m_element(element) {}
    ~MyMass_8() {}

  private:
    ChElementShellANCF_8* m_element;

    virtual void Evaluate(ChMatrixNM<double, 72, 72>& result, const double x, const double y, const double z) override;
};

void MyMass_8::Evaluate(ChMatrixNM<double, 72, 72>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 24> N;
    m_element->ShapeFunctions(N, x, y, z);

    // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)...
    // N9*eye(3) N10*eye(3) N11*eye(3) N12*eye(3) N13*eye(3) N14*eye(3) N15*eye(3) N16*eye(3)...
    // N17*eye(3) N18*eye(3) N19*eye(3) N20*eye(3) N21*eye(3) N22*eye(3) N23*eye(3) N24*eye(3)]

    ChMatrixNM<double, 3, 72> S;
    ChMatrix33<> Si;
    Si.Reset();

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

    Si.FillDiag(N(8));
    S.PasteMatrix(Si, 0, 24);
    Si.FillDiag(N(9));
    S.PasteMatrix(Si, 0, 27);
    Si.FillDiag(N(10));
    S.PasteMatrix(Si, 0, 30);
    Si.FillDiag(N(11));
    S.PasteMatrix(Si, 0, 33);
    Si.FillDiag(N(12));
    S.PasteMatrix(Si, 0, 36);
    Si.FillDiag(N(13));
    S.PasteMatrix(Si, 0, 39);
    Si.FillDiag(N(14));
    S.PasteMatrix(Si, 0, 42);
    Si.FillDiag(N(15));
    S.PasteMatrix(Si, 0, 45);

    Si.FillDiag(N(16));
    S.PasteMatrix(Si, 0, 48);
    Si.FillDiag(N(17));
    S.PasteMatrix(Si, 0, 51);
    Si.FillDiag(N(18));
    S.PasteMatrix(Si, 0, 54);
    Si.FillDiag(N(19));
    S.PasteMatrix(Si, 0, 57);
    Si.FillDiag(N(20));
    S.PasteMatrix(Si, 0, 60);
    Si.FillDiag(N(21));
    S.PasteMatrix(Si, 0, 63);
    Si.FillDiag(N(22));
    S.PasteMatrix(Si, 0, 66);
    Si.FillDiag(N(23));
    S.PasteMatrix(Si, 0, 69);

    double detJ0 = m_element->Calc_detJ0(x, y, z);

    // perform  r = S'*S
    result.MatrTMultiply(S, S);

    // multiply integration weights
    result *= detJ0 * (m_element->m_GaussScaling);
};

void ChElementShellANCF_8::ComputeMassMatrix() {
    m_MassMatrix.Reset();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        MyMass_8 myformula(this);
        ChMatrixNM<double, 72, 72> TempMassMatrix;
        TempMassMatrix.Reset();
        ChQuadrature::Integrate3D<ChMatrixNM<double, 72, 72> >(TempMassMatrix,  // result of integration will go there
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
class MyGravity_8 : public ChIntegrable3D<ChMatrixNM<double, 72, 1> > {
  public:
    MyGravity_8(ChElementShellANCF_8* element, const ChVector<> gacc) : m_element(element), m_gacc(gacc) {}
    ~MyGravity_8() {}

  private:
    ChElementShellANCF_8* m_element;
    ChVector<> m_gacc;

    virtual void Evaluate(ChMatrixNM<double, 72, 1>& result, const double x, const double y, const double z) override;
};

void MyGravity_8::Evaluate(ChMatrixNM<double, 72, 1>& result, const double x, const double y, const double z) {
    ChMatrixNM<double, 1, 24> N;
    m_element->ShapeFunctions(N, x, y, z);
    double detJ0 = m_element->Calc_detJ0(x, y, z);

    for (int i = 0; i < 24; i++) {
        result(i * 3 + 0, 0) = N(0, i) * m_gacc.x();
        result(i * 3 + 1, 0) = N(0, i) * m_gacc.y();
        result(i * 3 + 2, 0) = N(0, i) * m_gacc.z();
    }
    result *= detJ0 * m_element->m_GaussScaling;
};

void ChElementShellANCF_8::ComputeGravityForce(const ChVector<>& g_acc) {
    m_GravForce.Reset();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        MyGravity_8 myformula(this, g_acc);
        ChMatrixNM<double, 72, 1> Fgravity;
        Fgravity.Reset();
        ChQuadrature::Integrate3D<ChMatrixNM<double, 72, 1> >(Fgravity,   // result of integration will go there
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

// The class MyForce provides the integrand for the calculation of the internal forces
// for one layer of an ANCF shell element.
// The first 72 entries in the integrand represent the internal force.
// This implementation also features a composite material implementation
// that allows for selecting a number of layers over the element thickness; each of which
// has an independent, user-selected fiber angle (direction for orthotropic constitutive behavior)
class MyForce_8 : public ChIntegrable3D<ChMatrixNM<double, 72, 1> > {
  public:
    MyForce_8(ChElementShellANCF_8* element,  // Containing element
              size_t kl                       // Current layer index
              )
        : m_element(element), m_kl(kl) {}
    ~MyForce_8() {}

  private:
    ChElementShellANCF_8* m_element;
    size_t m_kl;

    /// Evaluate (strainD'*strain)  at point x, include ANS and EAS.
    virtual void Evaluate(ChMatrixNM<double, 72, 1>& result, const double x, const double y, const double z) override;
};

void MyForce_8::Evaluate(ChMatrixNM<double, 72, 1>& result, const double x, const double y, const double z) {
    // Element shape function
    ChMatrixNM<double, 1, 24> N;
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

    ChMatrixNM<double, 24, 1> ddNx;
    ChMatrixNM<double, 24, 1> ddNy;
    ChMatrixNM<double, 24, 1> ddNz;
    ddNx.MatrMultiplyT(m_element->m_ddT, Nx);
    ddNy.MatrMultiplyT(m_element->m_ddT, Ny);
    ddNz.MatrMultiplyT(m_element->m_ddT, Nz);

    ChMatrixNM<double, 24, 1> d0d0Nx;
    ChMatrixNM<double, 24, 1> d0d0Ny;
    ChMatrixNM<double, 24, 1> d0d0Nz;
    d0d0Nx.MatrMultiplyT(m_element->m_d0d0T, Nx);
    d0d0Ny.MatrMultiplyT(m_element->m_d0d0T, Ny);
    d0d0Nz.MatrMultiplyT(m_element->m_d0d0T, Nz);
    // Strain component
    ChMatrixNM<double, 6, 1> strain_til;
    strain_til(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);          // xy
    strain_til(3, 0) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));  // zz
    strain_til(4, 0) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);          // yz
    strain_til(5, 0) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);          // xz

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

    ChMatrixNM<double, 6, 72> strainD_til;
    ChMatrixNM<double, 1, 72> tempB;
    ChMatrixNM<double, 1, 72> tempBB;
    ChMatrixNM<double, 1, 3> tempB3;
    ChMatrixNM<double, 1, 3> tempB31;
    ChMatrixNM<double, 1, 3> tempB32;

    strainD_til.Reset();
    tempB3.MatrMultiply(Nx, m_element->m_d);  // rx
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 0, 0);  // xx

    tempB31.MatrMultiply(Ny, m_element->m_d);  // ry
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 1, 0);  // yy

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i) + tempB31(0, j) * Nx(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 2, 0);  // xy

    // Recuero
    tempB32.MatrMultiply(Nz, m_element->m_d);  // rz
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Nz(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 3, 0);  // zz

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Ny(0, i) + tempB31(0, j) * Nz(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 4, 0);  // yz

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i) + tempB32(0, j) * Nx(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 5, 0);  // xz

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
    ChMatrixNM<double, 6, 1> DEPS;
    DEPS.Reset();
    for (int ii = 0; ii < 72; ii++) {
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
    ChMatrixNM<double, 72, 6> tempC;
    tempC.MatrTMultiply(strainD, E_eps);
    ChMatrixNM<double, 72, 1> Fint = (tempC * strain) * (detJ0 * m_element->m_GaussScaling);

    /// Total result vector
    result.PasteClippedMatrix(Fint, 0, 0, 72, 1, 0, 0);
}

void ChElementShellANCF_8::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    // Current nodal coordinates and velocities
    CalcCoordMatrix(m_d);
    CalcCoordDerivMatrix(m_d_dt);
    m_ddT.MatrMultiplyT(m_d, m_d);
    Fi.Reset();

    for (size_t kl = 0; kl < m_numLayers; kl++) {
        MyForce_8 formula(this, kl);
        ChMatrixNM<double, 72, 1> Finternal;
        Finternal.Reset();
        ChQuadrature::Integrate3D<ChMatrixNM<double, 72, 1> >(Finternal,                       // result of integration
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

// The class MyJacobian provides the integrand for the calculation of the Jacobians
// (stiffness and damping matrices) of the internal forces for one layer of an ANCF
// shell element.
// 72x72 Jacobian
//      Kfactor * [K] + Rfactor * [R]

class MyJacobian_8 : public ChIntegrable3D<ChMatrixNM<double, 5184, 1> > {
  public:
    MyJacobian_8(ChElementShellANCF_8* element,  // Containing element
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
    virtual void Evaluate(ChMatrixNM<double, 5184, 1>& result, const double x, const double y, const double z) override;
};

void MyJacobian_8::Evaluate(ChMatrixNM<double, 5184, 1>& result, const double x, const double y, const double z) {
    // Element shape function
    ChMatrixNM<double, 1, 24> N;
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

    ChMatrixNM<double, 24, 1> ddNx;
    ChMatrixNM<double, 24, 1> ddNy;
    ChMatrixNM<double, 24, 1> ddNz;
    ddNx.MatrMultiplyT(m_element->m_ddT, Nx);
    ddNy.MatrMultiplyT(m_element->m_ddT, Ny);
    ddNz.MatrMultiplyT(m_element->m_ddT, Nz);

    ChMatrixNM<double, 24, 1> d0d0Nx;
    ChMatrixNM<double, 24, 1> d0d0Ny;
    ChMatrixNM<double, 24, 1> d0d0Nz;
    d0d0Nx.MatrMultiplyT(m_element->m_d0d0T, Nx);
    d0d0Ny.MatrMultiplyT(m_element->m_d0d0T, Ny);
    d0d0Nz.MatrMultiplyT(m_element->m_d0d0T, Nz);

    // Strain component
    ChMatrixNM<double, 6, 1> strain_til;
    strain_til(0, 0) = 0.5 * ((Nx * ddNx)(0, 0) - (Nx * d0d0Nx)(0, 0));
    strain_til(1, 0) = 0.5 * ((Ny * ddNy)(0, 0) - (Ny * d0d0Ny)(0, 0));
    strain_til(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);          // xy
    strain_til(3, 0) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));  // zz
    strain_til(4, 0) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);          // yz
    strain_til(5, 0) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);          // xz

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

    ChMatrixNM<double, 6, 72> strainD_til;
    ChMatrixNM<double, 1, 72> tempB;
    ChMatrixNM<double, 1, 72> tempBB;
    ChMatrixNM<double, 1, 3> tempB3;
    ChMatrixNM<double, 1, 3> tempB31;
    ChMatrixNM<double, 1, 3> tempB32;

    strainD_til.Reset();
    tempB3.MatrMultiply(Nx, m_element->m_d);  // rx
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nx(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 0, 0);  // xx

    tempB31.MatrMultiply(Ny, m_element->m_d);  // ry
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB31(0, j) * Ny(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 1, 0);  // yy

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Ny(0, i) + tempB31(0, j) * Nx(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 2, 0);  // xy

    tempB32.MatrMultiply(Nz, m_element->m_d);  // rz
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Nz(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 3, 0);  // zz

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB32(0, j) * Ny(0, i) + tempB31(0, j) * Nz(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 4, 0);  // yz

    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 3; j++) {
            tempB(0, i * 3 + j) = tempB3(0, j) * Nz(0, i) + tempB32(0, j) * Nx(0, i);
        }
    }
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 72, 5, 0);  // xz

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
    Gd.Reset();

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
    ChMatrixNM<double, 6, 1> DEPS;
    DEPS.Reset();
    for (int ii = 0; ii < 72; ii++) {
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
    Sigm.Reset();

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
    ChMatrixNM<double, 72, 6> temp726;
    ChMatrixNM<double, 72, 9> temp729;
    temp726.MatrTMultiply(strainD, E_eps);
    temp729.MatrTMultiply(Gd, Sigm);
    ChMatrixNM<double, 72, 72> KTE;

#ifdef CHRONO_HAS_AVX
    ChMatrixNM<double, 72, 72> KTE_temp1;
    ChMatrixNM<double, 72, 72> KTE_temp2;
    KTE_temp1.MatrMultiplyAVX(temp726, strainD);
    KTE_temp2.MatrMultiplyAVX(temp729, Gd);
    KTE = KTE_temp1 * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + KTE_temp2 * m_Kfactor;
#else
    KTE = (temp726 * strainD) * (m_Kfactor + m_Rfactor * m_element->m_Alpha) + (temp729 * Gd) * m_Kfactor;
#endif
    KTE *= detJ0 * (m_element->m_GaussScaling);

    // Load result vector (integrand)
    result.PasteClippedMatrixToVector(KTE, 0, 0, 72, 72, 0);
}

void ChElementShellANCF_8::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    // Note that the matrices with current nodal coordinates and velocities are
    // already available in m_d and m_d_dt (as set in ComputeInternalForces).

    m_JacobianMatrix.Reset();
    // Loop over all layers.
    for (size_t kl = 0; kl < m_numLayers; kl++) {
        MyJacobian_8 formula(this, Kfactor, Rfactor, kl);
        ChMatrixNM<double, 5184, 1> result;
        result.Reset();
        ChQuadrature::Integrate3D<ChMatrixNM<double, 5184, 1> >(result,   // result of integration
                                                                formula,  // integrand formula
                                                                -1, 1,    // x limits
                                                                -1, 1,    // y limits
                                                                m_GaussZ[kl], m_GaussZ[kl + 1],  // z limits
                                                                3                                // order of integration
        );

        // Extract matrices from result of integration
        ChMatrixNM<double, 72, 72> KTE;
        KTE.PasteClippedVectorToMatrix(result, 0, 0, 72, 72, 0);

        // Accumulate Jacobian
        m_JacobianMatrix += KTE;
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementShellANCF_8::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
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

void ChElementShellANCF_8::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
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

void ChElementShellANCF_8::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
    double a = GetLengthX();
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

void ChElementShellANCF_8::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
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
                                        ChMatrixNM<double, 1, 24>& Nx,
                                        ChMatrixNM<double, 1, 24>& Ny,
                                        ChMatrixNM<double, 1, 24>& Nz,
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

void ChElementShellANCF_8::CalcCoordDerivMatrix(ChMatrixNM<double, 72, 1>& dt) {
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

    dt(0, 0) = pA_dt.x();
    dt(1, 0) = pA_dt.y();
    dt(2, 0) = pA_dt.z();
    dt(3, 0) = dA_dt.x();
    dt(4, 0) = dA_dt.y();
    dt(5, 0) = dA_dt.z();
    dt(6, 0) = ddA_dt.x();
    dt(7, 0) = ddA_dt.y();
    dt(8, 0) = ddA_dt.z();

    dt(9, 0) = pB_dt.x();
    dt(10, 0) = pB_dt.y();
    dt(11, 0) = pB_dt.z();
    dt(12, 0) = dB_dt.x();
    dt(13, 0) = dB_dt.y();
    dt(14, 0) = dB_dt.z();
    dt(15, 0) = ddB_dt.x();
    dt(16, 0) = ddB_dt.y();
    dt(17, 0) = ddB_dt.z();

    dt(18, 0) = pC_dt.x();
    dt(19, 0) = pC_dt.y();
    dt(20, 0) = pC_dt.z();
    dt(21, 0) = dC_dt.x();
    dt(22, 0) = dC_dt.y();
    dt(23, 0) = dC_dt.z();
    dt(24, 0) = ddC_dt.x();
    dt(25, 0) = ddC_dt.y();
    dt(26, 0) = ddC_dt.z();

    dt(27, 0) = pD_dt.x();
    dt(28, 0) = pD_dt.y();
    dt(29, 0) = pD_dt.z();
    dt(30, 0) = dD_dt.x();
    dt(31, 0) = dD_dt.y();
    dt(32, 0) = dD_dt.z();
    dt(33, 0) = ddD_dt.x();
    dt(34, 0) = ddD_dt.y();
    dt(35, 0) = ddD_dt.z();

    dt(36, 0) = pE_dt.x();
    dt(37, 0) = pE_dt.y();
    dt(38, 0) = pE_dt.z();
    dt(39, 0) = dE_dt.x();
    dt(40, 0) = dE_dt.y();
    dt(41, 0) = dE_dt.z();
    dt(42, 0) = ddE_dt.x();
    dt(43, 0) = ddE_dt.y();
    dt(44, 0) = ddE_dt.z();

    dt(45, 0) = pF_dt.x();
    dt(46, 0) = pF_dt.y();
    dt(47, 0) = pF_dt.z();
    dt(48, 0) = dF_dt.x();
    dt(49, 0) = dF_dt.y();
    dt(50, 0) = dF_dt.z();
    dt(51, 0) = ddF_dt.x();
    dt(52, 0) = ddF_dt.y();
    dt(53, 0) = ddF_dt.z();

    dt(54, 0) = pG_dt.x();
    dt(55, 0) = pG_dt.y();
    dt(56, 0) = pG_dt.z();
    dt(57, 0) = dG_dt.x();
    dt(58, 0) = dG_dt.y();
    dt(59, 0) = dG_dt.z();
    dt(60, 0) = ddG_dt.x();
    dt(61, 0) = ddG_dt.y();
    dt(62, 0) = ddG_dt.z();

    dt(63, 0) = pH_dt.x();
    dt(64, 0) = pH_dt.y();
    dt(65, 0) = pH_dt.z();
    dt(66, 0) = dH_dt.x();
    dt(67, 0) = dH_dt.y();
    dt(68, 0) = dH_dt.z();
    dt(69, 0) = ddH_dt.x();
    dt(70, 0) = ddH_dt.y();
    dt(71, 0) = ddH_dt.z();
}

// -----------------------------------------------------------------------------
// Interface to ChElementShell base class
// -----------------------------------------------------------------------------
ChVector<> ChElementShellANCF_8::EvaluateSectionStrains() {
    // Element shape function
    ChMatrixNM<double, 1, 24> N;
    this->ShapeFunctions(N, 0, 0, 0);

    // Determinant of position vector gradient matrix: Initial configuration
    ChMatrixNM<double, 1, 24> Nx;
    ChMatrixNM<double, 1, 24> Ny;
    ChMatrixNM<double, 1, 24> Nz;
    ChMatrixNM<double, 1, 3> Nx_d0;
    ChMatrixNM<double, 1, 3> Ny_d0;
    ChMatrixNM<double, 1, 3> Nz_d0;
    double detJ0 = this->Calc_detJ0(0, 0, 0, Nx, Ny, Nz, Nx_d0, Ny_d0, Nz_d0);

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
    strain_til(2, 0) = (Nx * ddNy)(0, 0) - (Nx * d0d0Ny)(0, 0);          // xy
    strain_til(3, 0) = 0.5 * ((Nz * ddNz)(0, 0) - (Nz * d0d0Nz)(0, 0));  // zz
    strain_til(4, 0) = (Ny * ddNz)(0, 0) - (Ny * d0d0Nz)(0, 0);          // yz
    strain_til(5, 0) = (Nx * ddNz)(0, 0) - (Nx * d0d0Nz)(0, 0);          // xz

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

void ChElementShellANCF_8::EvaluateSectionPoint(const double u,
                                                const double v,
                                                ChVector<>& point) {
    ChVector<> u_displ;

    ChMatrixNM<double, 1, 24> N;

    double x = u;  // because ShapeFunctions() works in -1..1 range
    double y = v;  // because ShapeFunctions() works in -1..1 range
    double z = 0;

    this->ShapeFunctions(N, x, y, z);

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
    mD.PasteVector(m_nodes[0]->GetPos(), block_offset, 0);
    mD.PasteVector(m_nodes[0]->GetD(), block_offset + 3, 0);
    mD.PasteVector(m_nodes[0]->GetDD(), block_offset + 6, 0);
    mD.PasteVector(m_nodes[1]->GetPos(), block_offset + 9, 0);
    mD.PasteVector(m_nodes[1]->GetD(), block_offset + 12, 0);
    mD.PasteVector(m_nodes[1]->GetDD(), block_offset + 15, 0);
    mD.PasteVector(m_nodes[2]->GetPos(), block_offset + 18, 0);
    mD.PasteVector(m_nodes[2]->GetD(), block_offset + 21, 0);
    mD.PasteVector(m_nodes[2]->GetDD(), block_offset + 24, 0);
    mD.PasteVector(m_nodes[3]->GetPos(), block_offset + 27, 0);
    mD.PasteVector(m_nodes[3]->GetD(), block_offset + 30, 0);
    mD.PasteVector(m_nodes[3]->GetDD(), block_offset + 33, 0);
    mD.PasteVector(m_nodes[4]->GetPos(), block_offset + 36, 0);
    mD.PasteVector(m_nodes[4]->GetD(), block_offset + 39, 0);
    mD.PasteVector(m_nodes[4]->GetDD(), block_offset + 42, 0);
    mD.PasteVector(m_nodes[5]->GetPos(), block_offset + 45, 0);
    mD.PasteVector(m_nodes[5]->GetD(), block_offset + 48, 0);
    mD.PasteVector(m_nodes[5]->GetDD(), block_offset + 51, 0);
    mD.PasteVector(m_nodes[6]->GetPos(), block_offset + 54, 0);
    mD.PasteVector(m_nodes[6]->GetD(), block_offset + 57, 0);
    mD.PasteVector(m_nodes[6]->GetDD(), block_offset + 60, 0);
    mD.PasteVector(m_nodes[7]->GetPos(), block_offset + 63, 0);
    mD.PasteVector(m_nodes[7]->GetD(), block_offset + 66, 0);
    mD.PasteVector(m_nodes[7]->GetDD(), block_offset + 69, 0);
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChElementShellANCF_8::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.PasteVector(m_nodes[0]->GetPos_dt(), block_offset, 0);
    mD.PasteVector(m_nodes[0]->GetD_dt(), block_offset + 3, 0);
    mD.PasteVector(m_nodes[0]->GetDD_dt(), block_offset + 6, 0);
    mD.PasteVector(m_nodes[1]->GetPos_dt(), block_offset + 9, 0);
    mD.PasteVector(m_nodes[1]->GetD_dt(), block_offset + 12, 0);
    mD.PasteVector(m_nodes[1]->GetDD_dt(), block_offset + 15, 0);
    mD.PasteVector(m_nodes[2]->GetPos_dt(), block_offset + 18, 0);
    mD.PasteVector(m_nodes[2]->GetD_dt(), block_offset + 21, 0);
    mD.PasteVector(m_nodes[2]->GetDD_dt(), block_offset + 24, 0);
    mD.PasteVector(m_nodes[3]->GetPos_dt(), block_offset + 27, 0);
    mD.PasteVector(m_nodes[3]->GetD_dt(), block_offset + 30, 0);
    mD.PasteVector(m_nodes[3]->GetDD_dt(), block_offset + 33, 0);
    mD.PasteVector(m_nodes[4]->GetPos_dt(), block_offset + 36, 0);
    mD.PasteVector(m_nodes[4]->GetD_dt(), block_offset + 39, 0);
    mD.PasteVector(m_nodes[4]->GetDD_dt(), block_offset + 42, 0);
    mD.PasteVector(m_nodes[5]->GetPos_dt(), block_offset + 45, 0);
    mD.PasteVector(m_nodes[5]->GetD_dt(), block_offset + 48, 0);
    mD.PasteVector(m_nodes[5]->GetDD_dt(), block_offset + 51, 0);
    mD.PasteVector(m_nodes[6]->GetPos_dt(), block_offset + 54, 0);
    mD.PasteVector(m_nodes[6]->GetD_dt(), block_offset + 57, 0);
    mD.PasteVector(m_nodes[6]->GetDD_dt(), block_offset + 60, 0);
    mD.PasteVector(m_nodes[7]->GetPos_dt(), block_offset + 63, 0);
    mD.PasteVector(m_nodes[7]->GetD_dt(), block_offset + 66, 0);
    mD.PasteVector(m_nodes[7]->GetDD_dt(), block_offset + 69, 0);
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
    ChMatrixNM<double, 24, 1> N;
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
    tmp = N(8) * Fv;
    Qi.PasteVector(tmp, 24, 0);
    tmp = N(9) * Fv;
    Qi.PasteVector(tmp, 27, 0);
    tmp = N(10) * Fv;
    Qi.PasteVector(tmp, 30, 0);
    tmp = N(11) * Fv;
    Qi.PasteVector(tmp, 33, 0);
    tmp = N(12) * Fv;
    Qi.PasteVector(tmp, 36, 0);
    tmp = N(13) * Fv;
    Qi.PasteVector(tmp, 39, 0);
    tmp = N(14) * Fv;
    Qi.PasteVector(tmp, 42, 0);
    tmp = N(15) * Fv;
    Qi.PasteVector(tmp, 45, 0);
    tmp = N(16) * Fv;
    Qi.PasteVector(tmp, 48, 0);
    tmp = N(17) * Fv;
    Qi.PasteVector(tmp, 51, 0);
    tmp = N(18) * Fv;
    Qi.PasteVector(tmp, 54, 0);
    tmp = N(19) * Fv;
    Qi.PasteVector(tmp, 57, 0);
    tmp = N(20) * Fv;
    Qi.PasteVector(tmp, 60, 0);
    tmp = N(21) * Fv;
    Qi.PasteVector(tmp, 63, 0);
    tmp = N(22) * Fv;
    Qi.PasteVector(tmp, 66, 0);
    tmp = N(23) * Fv;
    Qi.PasteVector(tmp, 69, 0);
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
    tmp = N(8) * Fv;
    Qi.PasteVector(tmp, 24, 0);
    tmp = N(9) * Fv;
    Qi.PasteVector(tmp, 27, 0);
    tmp = N(10) * Fv;
    Qi.PasteVector(tmp, 30, 0);
    tmp = N(11) * Fv;
    Qi.PasteVector(tmp, 33, 0);
    tmp = N(12) * Fv;
    Qi.PasteVector(tmp, 36, 0);
    tmp = N(13) * Fv;
    Qi.PasteVector(tmp, 39, 0);
    tmp = N(14) * Fv;
    Qi.PasteVector(tmp, 42, 0);
    tmp = N(15) * Fv;
    Qi.PasteVector(tmp, 45, 0);
    tmp = N(16) * Fv;
    Qi.PasteVector(tmp, 48, 0);
    tmp = N(17) * Fv;
    Qi.PasteVector(tmp, 51, 0);
    tmp = N(18) * Fv;
    Qi.PasteVector(tmp, 54, 0);
    tmp = N(19) * Fv;
    Qi.PasteVector(tmp, 57, 0);
    tmp = N(20) * Fv;
    Qi.PasteVector(tmp, 60, 0);
    tmp = N(21) * Fv;
    Qi.PasteVector(tmp, 63, 0);
    tmp = N(22) * Fv;
    Qi.PasteVector(tmp, 66, 0);
    tmp = N(23) * Fv;
    Qi.PasteVector(tmp, 69, 0);
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
    ChMatrixNM<double, 24, 3> mDD;
    ChMatrixNM<double, 1, 24> Nx;
    ChMatrixNM<double, 1, 24> Ny;
    ChMatrixNM<double, 1, 24> Nz;

    ShapeFunctionsDerivativeX(Nx, U, V, 0);
    ShapeFunctionsDerivativeY(Ny, U, V, 0);
    ShapeFunctionsDerivativeZ(Nz, U, V, 0);

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
    ChMatrixNM<double, 1, 24> Nx;
    ChMatrixNM<double, 1, 24> Ny;
    ChMatrixNM<double, 1, 24> Nz;
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
