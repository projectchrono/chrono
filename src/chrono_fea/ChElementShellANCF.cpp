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
//
// ANCF laminated shell element with four nodes.
//
// =============================================================================

#include "chrono/core/ChException.h"
#include "core/ChQuadrature.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChUtilsFEA.h"

namespace chrono {
namespace fea {

ChElementShellANCF::ChElementShellANCF() : m_flag_HE(ANALYTICAL) {
    m_nodes.resize(4);

    m_StiffnessMatrix.Resize(GetNdofs(), GetNdofs());
    m_MassMatrix.Resize(GetNdofs(), GetNdofs());
}

void ChElementShellANCF::SetNodes(ChSharedPtr<ChNodeFEAxyzD> nodeA,
                                  ChSharedPtr<ChNodeFEAxyzD> nodeB,
                                  ChSharedPtr<ChNodeFEAxyzD> nodeC,
                                  ChSharedPtr<ChNodeFEAxyzD> nodeD) {
    assert(!nodeA.IsNull());
    assert(!nodeB.IsNull());
    assert(!nodeC.IsNull());
    assert(!nodeD.IsNull());

    m_nodes[0] = nodeA;
    m_nodes[1] = nodeB;
    m_nodes[2] = nodeC;
    m_nodes[3] = nodeD;
    std::vector<ChLcpVariables*> mvars;
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
    ChVector<> pA = m_nodes[0]->GetPos();
    ChVector<> dA = m_nodes[0]->GetD();
    ChVector<> pB = m_nodes[1]->GetPos();
    ChVector<> dB = m_nodes[1]->GetD();
    ChVector<> pC = m_nodes[2]->GetPos();
    ChVector<> dC = m_nodes[2]->GetD();
    ChVector<> pD = m_nodes[3]->GetPos();
    ChVector<> dD = m_nodes[3]->GetD();

    m_d0(0, 0) = pA(0);
    m_d0(0, 1) = pA(1);
    m_d0(0, 2) = pA(2);

    m_d0(1, 0) = dA(0);
    m_d0(1, 1) = dA(1);
    m_d0(1, 2) = dA(2);

    m_d0(2, 0) = pB(0);
    m_d0(2, 1) = pB(1);
    m_d0(2, 2) = pB(2);

    m_d0(3, 0) = dB(0);
    m_d0(3, 1) = dB(1);
    m_d0(3, 2) = dB(2);

    m_d0(4, 0) = pC(0);
    m_d0(4, 1) = pC(1);
    m_d0(4, 2) = pC(2);

    m_d0(5, 0) = dC(0);
    m_d0(5, 1) = dC(1);
    m_d0(5, 2) = dC(2);

    m_d0(6, 0) = pD(0);
    m_d0(6, 1) = pD(1);
    m_d0(6, 2) = pD(2);

    m_d0(7, 0) = dD(0);
    m_d0(7, 1) = dD(1);
    m_d0(7, 2) = dD(2);
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
    N(4) = 0.25 * (1.0 - x) * (1.0 + y);
    N(5) = z * c / 2.0 * 0.25 * (1.0 - x) * (1.0 + y);
    N(6) = 0.25 * (1.0 + x) * (1.0 + y);
    N(7) = z * c / 2.0 * 0.25 * (1.0 + x) * (1.0 + y);
}

void ChElementShellANCF::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
    double c = m_thickness;

    Nx(0) = 0.25 * (-2.0 / a) * (1.0 - y);
    Nx(1) = z * c / 2.0 * 0.25 * (-2.0 / a) * (1.0 - y);
    Nx(2) = 0.25 * (2.0 / a) * (1.0 - y);
    Nx(3) = z * c / 2.0 * 0.25 * (2.0 / a) * (1.0 - y);
    Nx(4) = 0.25 * (-2.0 / a) * (1.0 + y);
    Nx(5) = z * c / 2.0 * 0.25 * (-2.0 / a) * (1.0 + y);
    Nx(6) = 0.25 * (2.0 / a) * (1.0 + y);
    Nx(7) = z * c / 2.0 * 0.25 * (2.0 / a) * (1.0 + y);
}

void ChElementShellANCF::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
    double a = GetLengthX();
    double b = GetLengthY();
    double c = m_thickness;

    Ny(0) = 0.25 * (1.0 - x) * (-2.0 / b);
    Ny(1) = z * c / 2.0 * 0.25 * (1.0 - x) * (-2.0 / b);
    Ny(2) = 0.25 * (1.0 + x) * (-2.0 / b);
    Ny(3) = z * c / 2.0 * 0.25 * (1.0 + x) * (-2.0 / b);
    Ny(4) = 0.25 * (1.0 - x) * (2.0 / b);
    Ny(5) = z * c / 2.0 * 0.25 * (1.0 - x) * (2.0 / b);
    Ny(6) = 0.25 * (1.0 + x) * (2.0 / b);
    Ny(7) = z * c / 2.0 * 0.25 * (1.0 + x) * (2.0 / b);
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
    Nz(5) = 0.250 * (1.0 - x) * (1.0 + y);
    Nz(6) = 0.0;
    Nz(7) = 0.250 * (1.0 + x) * (1.0 + y);
}

// -----------------------------------------------------------------------------

void ChElementShellANCF::Update() {
    // parent class update:
    ChElementGeneric::Update();
}

// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------

void ChElementShellANCF::ComputeStiffnessMatrix() {
    bool use_numerical_differentiation = false;
    // bool use_numerical_differentiation = true;

    if (use_numerical_differentiation) {
    } else {
        m_StiffnessMatrix = m_stock_KTE;
        m_StiffnessMatrix -= m_stock_jac_EAS;
    }
}

void ChElementShellANCF::ComputeMassMatrix() {
    ChMatrixNM<double, 24, 24> TempMassMatrix;
    m_MassMatrix.Reset();

    for (int kl = 0; kl < m_numLayers; kl++) {
        int ij = 14 * kl;
        double rho = m_InertFlexVec(ij);

        /// Integrate  rho*(S'*S)
        /// where S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)]
        class MyMass : public ChIntegrable3D<ChMatrixNM<double, 24, 24> > {
          public:
            ChElementShellANCF* element;
            ChMatrixNM<double, 8, 3>* d0;  //// pointer to initial coordinates
            ChMatrixNM<double, 3, 24> S;
            ChMatrixNM<double, 3, 24> Sx;
            ChMatrixNM<double, 3, 24> Sy;
            ChMatrixNM<double, 3, 24> Sz;
            ChMatrixNM<double, 1, 8> N;
            ChMatrixNM<double, 1, 8> Nx;
            ChMatrixNM<double, 1, 8> Ny;
            ChMatrixNM<double, 1, 8> Nz;

            /// Evaluate the S'*S  at point x
            virtual void Evaluate(ChMatrixNM<double, 24, 24>& result, const double x, const double y, const double z) {
                element->ShapeFunctions(N, x, y, z);
                element->ShapeFunctionsDerivativeX(Nx, x, y, z);
                element->ShapeFunctionsDerivativeY(Ny, x, y, z);
                element->ShapeFunctionsDerivativeZ(Nz, x, y, z);
                // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3) N5*eye(3) N6*eye(3) N7*eye(3) N8*eye(3)]
                ChMatrix33<> Si;
                Si.FillDiag(N(0));
                S.PasteMatrix(&Si, 0, 0);
                Si.FillDiag(N(1));
                S.PasteMatrix(&Si, 0, 3);
                Si.FillDiag(N(2));
                S.PasteMatrix(&Si, 0, 6);
                Si.FillDiag(N(3));
                S.PasteMatrix(&Si, 0, 9);
                Si.FillDiag(N(4));
                S.PasteMatrix(&Si, 0, 12);
                Si.FillDiag(N(5));
                S.PasteMatrix(&Si, 0, 15);
                Si.FillDiag(N(6));
                S.PasteMatrix(&Si, 0, 18);
                Si.FillDiag(N(7));
                S.PasteMatrix(&Si, 0, 21);

                ////Matrix Multiplication
                ChMatrixNM<double, 1, 3> Nx_d0;
                Nx_d0.MatrMultiply(Nx, *d0);

                ChMatrixNM<double, 1, 3> Ny_d0;
                Ny_d0.MatrMultiply(Ny, *d0);

                ChMatrixNM<double, 1, 3> Nz_d0;
                Nz_d0.MatrMultiply(Nz, *d0);

                ChMatrixNM<double, 3, 3> rd0;
                rd0(0, 0) = Nx_d0(0, 0);
                rd0(1, 0) = Nx_d0(0, 1);
                rd0(2, 0) = Nx_d0(0, 2);
                rd0(0, 1) = Ny_d0(0, 0);
                rd0(1, 1) = Ny_d0(0, 1);
                rd0(2, 1) = Ny_d0(0, 2);
                rd0(0, 2) = Nz_d0(0, 0);
                rd0(1, 2) = Nz_d0(0, 1);
                rd0(2, 2) = Nz_d0(0, 2);

                double detJ0 = rd0.Det();

                // perform  r = S'*S
                result.MatrTMultiply(S, S);

                // multiply integration weighs
                result *=
                    detJ0 * (element->GetLengthX() / 2) * (element->GetLengthY() / 2) * (element->m_thickness / 2);
            }
        };

        MyMass myformula;
        myformula.d0 = &m_d0;
        myformula.element = this;

        TempMassMatrix.Reset();

        ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 24> >(TempMassMatrix,  // result of integration will go there
                                                               myformula,       // formula to integrate
                                                               -1,              // start of x
                                                               1,               // end of x
                                                               -1,              // start of y
                                                               1,               // end of y
                                                               m_GaussZRange(kl, 0),  // start of z
                                                               m_GaussZRange(kl, 1),  // end of z
                                                               2                      // order of integration
                                                               );
        TempMassMatrix *= rho;
        m_MassMatrix += TempMassMatrix;
    }  // Layer Loop
}

void ChElementShellANCF::ComputeGravityForce() {
    m_GravForce.Reset();

    for (int kl = 0; kl < m_numLayers; kl++) {
        int ij = 14 * kl;

        //// Material properties
        double rho = m_InertFlexVec(ij);
        // Add gravity force
        class MyGravity : public ChIntegrable3D<ChMatrixNM<double, 24, 1> > {
          public:
            ChElementShellANCF* element;
            ChMatrixNM<double, 8, 3>* d0;
            ChMatrixNM<double, 3, 24> S;
            ChMatrixNM<double, 1, 8> N;
            ChMatrixNM<double, 1, 8> Nx;
            ChMatrixNM<double, 1, 8> Ny;
            ChMatrixNM<double, 1, 8> Nz;
            ChMatrixNM<double, 3, 1> LocalGravityForce;

            virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y, const double z) {
                element->ShapeFunctions(N, x, y, z);
                element->ShapeFunctionsDerivativeX(Nx, x, y, z);
                element->ShapeFunctionsDerivativeY(Ny, x, y, z);
                element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

                // Weights for Gaussian integration
                double wx2 = (element->GetLengthX()) / 2.0;
                double wy2 = (element->GetLengthY()) / 2.0;
                double wz2 = (element->m_thickness) / 2.0;

                // Set gravity acceleration
                if (element->m_gravity_on) {
                    LocalGravityForce(0, 0) = 0.0;
                    LocalGravityForce(1, 0) = 0.0;
                    LocalGravityForce(2, 0) = -9.81;
                } else {
                    LocalGravityForce(0, 0) = 0.0;
                    LocalGravityForce(1, 0) = 0.0;
                    LocalGravityForce(2, 0) = 0.0;
                }

                // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)]
                ChMatrix33<> Si;
                Si.FillDiag(N(0));
                S.PasteMatrix(&Si, 0, 0);
                Si.FillDiag(N(1));
                S.PasteMatrix(&Si, 0, 3);
                Si.FillDiag(N(2));
                S.PasteMatrix(&Si, 0, 6);
                Si.FillDiag(N(3));
                S.PasteMatrix(&Si, 0, 9);
                Si.FillDiag(N(4));
                S.PasteMatrix(&Si, 0, 12);
                Si.FillDiag(N(5));
                S.PasteMatrix(&Si, 0, 15);
                Si.FillDiag(N(6));
                S.PasteMatrix(&Si, 0, 18);
                Si.FillDiag(N(7));
                S.PasteMatrix(&Si, 0, 21);

                ChMatrixNM<double, 1, 3> Nx_d0;
                Nx_d0.MatrMultiply(Nx, *d0);

                ChMatrixNM<double, 1, 3> Ny_d0;
                Ny_d0.MatrMultiply(Ny, *d0);

                ChMatrixNM<double, 1, 3> Nz_d0;
                Nz_d0.MatrMultiply(Nz, *d0);

                ChMatrixNM<double, 3, 3> rd0;
                rd0(0, 0) = Nx_d0(0, 0);
                rd0(1, 0) = Nx_d0(0, 1);
                rd0(2, 0) = Nx_d0(0, 2);
                rd0(0, 1) = Ny_d0(0, 0);
                rd0(1, 1) = Ny_d0(0, 1);
                rd0(2, 1) = Ny_d0(0, 2);
                rd0(0, 2) = Nz_d0(0, 0);
                rd0(1, 2) = Nz_d0(0, 1);
                rd0(2, 2) = Nz_d0(0, 2);

                double detJ0 = rd0.Det();

                result.MatrTMultiply(S, LocalGravityForce);

                result *= detJ0 * wx2 * wy2 * wz2;
            }
        };

        MyGravity myformula1;
        myformula1.d0 = &m_d0;
        myformula1.element = this;

        ChMatrixNM<double, 24, 1> Fgravity;
        ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 1> >(Fgravity,    // result of integration will go there
                                                              myformula1,  // formula to integrate
                                                              -1,          // start of x
                                                              1,           // end of x
                                                              -1,          // start of y
                                                              1,           // end of y
                                                              m_GaussZRange(kl, 0),  // start of z
                                                              m_GaussZRange(kl, 1),  // end of z
                                                              2                      // order of integration
                                                              );

        Fgravity *= rho;
        m_GravForce += Fgravity;
    }
}

// -----------------------------------------------------------------------------

void ChElementShellANCF::SetupInitial() {
    ComputeGravityForce();
    // Compute initial Jacobian
    ChMatrixDynamic<double> Temp(GetNdofs(), 1);
    ComputeInternalForces(Temp);
    // Compute mass matrix
    ComputeMassMatrix();

    // Compute stiffness matrix
    // (this is not constant in ANCF and will be called automatically many times by ComputeKRMmatricesGlobal()
    // when the solver will run, yet maybe nice to provide an initial nonzero value)
    ComputeStiffnessMatrix();
}

// -----------------------------------------------------------------------------

void ChElementShellANCF::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 24) && (H.GetColumns() == 24));

    // Compute global stiffness matrix:
    ComputeStiffnessMatrix();

    //
    // 1) Store  +kf*[K] +rf*[R]
    //

    // For K stiffness matrix and R matrix: scale by factors
    // because [R] = r*[K] , so kf*[K]+rf*[R] = (kf+rf*r)*[K]
    double kr_factor = Kfactor + Rfactor * m_Material->Get_RayleighDampingK();

    ChMatrixDynamic<> temp(m_StiffnessMatrix);
    temp.MatrScale(kr_factor);

    // Paste scaled K stiffness matrix and R matrix in resulting H:
    H.PasteMatrix(&temp, 0, 0);

    //
    // 2) Store  +mf*[M]
    //

    temp = m_MassMatrix;
    temp.MatrScale(Mfactor);

    // Paste scaled M mass matrix in resulting H:
    H.PasteSumMatrix(&temp, 0, 0);
}

// -----------------------------------------------------------------------------

void ChElementShellANCF::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    /// Current nodal coordiantes
    ChVector<> pA = m_nodes[0]->GetPos();
    ChVector<> dA = m_nodes[0]->GetD();
    ChVector<> pB = m_nodes[1]->GetPos();
    ChVector<> dB = m_nodes[1]->GetD();
    ChVector<> pC = m_nodes[2]->GetPos();
    ChVector<> dC = m_nodes[2]->GetD();
    ChVector<> pD = m_nodes[3]->GetPos();
    ChVector<> dD = m_nodes[3]->GetD();

    /// Current nodal velocity for structural damping
    ChVector<> pA_dt = m_nodes[0]->GetPos_dt();
    ChVector<> dA_dt = m_nodes[0]->GetD_dt();
    ChVector<> pB_dt = m_nodes[1]->GetPos_dt();
    ChVector<> dB_dt = m_nodes[1]->GetD_dt();
    ChVector<> pC_dt = m_nodes[2]->GetPos_dt();
    ChVector<> dC_dt = m_nodes[2]->GetD_dt();
    ChVector<> pD_dt = m_nodes[3]->GetPos_dt();
    ChVector<> dD_dt = m_nodes[3]->GetD_dt();

    ChMatrixNM<double, 24, 1> d_dt;  // for structural damping
    d_dt(0, 0) = pA_dt.x;
    d_dt(1, 0) = pA_dt.y;
    d_dt(2, 0) = pA_dt.z;
    d_dt(3, 0) = dA_dt.x;
    d_dt(4, 0) = dA_dt.y;
    d_dt(5, 0) = dA_dt.z;
    d_dt(6, 0) = pB_dt.x;
    d_dt(7, 0) = pB_dt.y;
    d_dt(8, 0) = pB_dt.z;
    d_dt(9, 0) = dB_dt.x;
    d_dt(10, 0) = dB_dt.y;
    d_dt(11, 0) = dB_dt.z;
    d_dt(12, 0) = pC_dt.x;
    d_dt(13, 0) = pC_dt.y;
    d_dt(14, 0) = pC_dt.z;
    d_dt(15, 0) = dC_dt.x;
    d_dt(16, 0) = dC_dt.y;
    d_dt(17, 0) = dC_dt.z;
    d_dt(18, 0) = pD_dt.x;
    d_dt(19, 0) = pD_dt.y;
    d_dt(20, 0) = pD_dt.z;
    d_dt(21, 0) = dD_dt.x;
    d_dt(22, 0) = dD_dt.y;
    d_dt(23, 0) = dD_dt.z;

    ChMatrixNM<double, 8, 3> d;
    d(0, 0) = pA.x;
    d(0, 1) = pA.y;
    d(0, 2) = pA.z;
    d(1, 0) = dA.x;
    d(1, 1) = dA.y;
    d(1, 2) = dA.z;
    d(2, 0) = pB.x;
    d(2, 1) = pB.y;
    d(2, 2) = pB.z;
    d(3, 0) = dB.x;
    d(3, 1) = dB.y;
    d(3, 2) = dB.z;
    d(4, 0) = pC.x;
    d(4, 1) = pC.y;
    d(4, 2) = pC.z;
    d(5, 0) = dC.x;
    d(5, 1) = dC.y;
    d(5, 2) = dC.z;
    d(6, 0) = pD.x;
    d(6, 1) = pD.y;
    d(6, 2) = pD.z;
    d(7, 0) = dD.x;
    d(7, 1) = dD.y;
    d(7, 2) = dD.z;

    /// Material properties
    ChMatrixNM<double, 35, 1> StockAlpha1;
    StockAlpha1 = GetStockAlpha();

    ChMatrixNM<double, 24, 24> TempJacobian;
    ChMatrixNM<double, 24, 24> TempJacobian_EAS;
    ChMatrixNM<double, 24, 24> stock_jac_EAS_elem1;  // laminate structure
    ChMatrixNM<double, 24, 24> KTE1;                 // Laminate structure

    Fi.Reset();
    stock_jac_EAS_elem1.Reset();
    KTE1.Reset();

    for (int kl = 0; kl < m_numLayers; kl++) {
        TempJacobian_EAS.Reset();
        TempJacobian.Reset();

        // int j=0;
        int ij = 14 * kl;

        //// Material properties
        double rho = m_InertFlexVec(ij);                          // Material->Get_density();
        double theta = m_InertFlexVec(ij + 4) * CH_C_DEG_TO_RAD;  // Fiber angle (rad)
        double Ex = m_InertFlexVec(ij + 5);                       // Material->Get_Ex();
        double Ey = m_InertFlexVec(ij + 6);                       // Material->Get_Ey();
        double Ez = m_InertFlexVec(ij + 7);                       // Material->Get_Ez();
        double vx = m_InertFlexVec(ij + 8);                       // Material->Get_vx();
        double vy = m_InertFlexVec(ij + 9);                       // Material->Get_vy();
        double vz = m_InertFlexVec(ij + 10);                      // Material->Get_vz();
        double Gx = m_InertFlexVec(ij + 11);                      // Material->Get_Gx();
        double Gy = m_InertFlexVec(ij + 12);                      // Material->Get_Gy();
        double Gz = m_InertFlexVec(ij + 13);                      // Material->Get_Gz();

        //// Cauchy-Green Tensor Calculation
        ChMatrixNM<double, 6, 6> E_eps;
        double CCOM[12];

        CCOM[0] = Ex;                           // Ex
        CCOM[1] = Ey;                           // Ey
        CCOM[2] = Ez;                           // Ez
        CCOM[3] = vx;                           // Nuxy
        CCOM[4] = vy;                           // Nuxz
        CCOM[5] = vz;                           // Nuyz
        CCOM[6] = CCOM[3] * CCOM[1] / CCOM[0];  //!Nuyx
        CCOM[7] = CCOM[4] * CCOM[2] / CCOM[0];  //!Nuzx
        CCOM[8] = CCOM[5] * CCOM[2] / CCOM[1];  //!Nuzy
        CCOM[9] = Gx;                           // Gxy
        CCOM[10] = Gy;                          // Gxz
        CCOM[11] = Gz;                          // Gyz
        double DELTA = 1.0 - (CCOM[3] * CCOM[3]) * CCOM[1] / CCOM[0] - (CCOM[4] * CCOM[4]) * CCOM[2] / CCOM[0] -
                       (CCOM[5] * CCOM[5]) * CCOM[2] / CCOM[1] - 2.0 * CCOM[3] * CCOM[4] * CCOM[5] * CCOM[2] / CCOM[0];
        E_eps(0, 0) = CCOM[0] * (1.0 - (CCOM[5] * CCOM[5]) * CCOM[2] / CCOM[1]) / DELTA;
        E_eps(1, 1) = CCOM[1] * (1.0 - (CCOM[4] * CCOM[4]) * CCOM[2] / CCOM[0]) / DELTA;
        E_eps(3, 3) = CCOM[2] * (1.0 - (CCOM[3] * CCOM[3]) * CCOM[1] / CCOM[0]) / DELTA;
        E_eps(0, 1) = CCOM[1] * (CCOM[3] + CCOM[4] * CCOM[5] * CCOM[2] / CCOM[1]) / DELTA;
        E_eps(0, 3) = CCOM[2] * (CCOM[4] + CCOM[5] * CCOM[3]) / DELTA;
        E_eps(1, 0) = CCOM[1] * (CCOM[3] + CCOM[4] * CCOM[5] * CCOM[2] / CCOM[1]) / DELTA;
        E_eps(1, 3) = CCOM[2] * (CCOM[5] + CCOM[4] * CCOM[3] * CCOM[1] / CCOM[0]) / DELTA;
        E_eps(3, 0) = CCOM[2] * (CCOM[4] + CCOM[5] * CCOM[3]) / DELTA;
        E_eps(3, 1) = CCOM[2] * (CCOM[5] + CCOM[4] * CCOM[3] * CCOM[1] / CCOM[0]) / DELTA;
        E_eps(2, 2) = CCOM[9];
        E_eps(4, 4) = CCOM[10];
        E_eps(5, 5) = CCOM[11];

        /// If numerical differentiation is used, only the internal force and EAS stiffness
        /// will be calculated. If the numerical differentiation is not used, the jacobian
        /// will also be calculated.
        bool use_numerical_differentiation = false;

        /// Internal force and EAS parameters are caulculated for numerical differentiation.
        if (use_numerical_differentiation) {
        } else {
            ///==========================================================================================================
            ///============ Internal force, EAS stiffness, and analytical jacobian are calculated
            ///=======================
            ///==========================================================================================================

            class MyForce : public ChIntegrable3D<ChMatrixNM<double, 750, 1> > {
              public:
                ChElementShellANCF* element;
                //// External values
                ChMatrixNM<double, 8, 3>* d;
                ChMatrixNM<double, 8, 1>* strain_ans;
                ChMatrixNM<double, 8, 24>* strainD_ans;
                ChMatrixNM<double, 8, 3>* d0;
                ChMatrixNM<double, 24, 1>* d_dt;  // for structural damping
                ChMatrixNM<double, 6, 6>* T0;
                ChMatrixNM<double, 5, 1>* alpha_eas;
                ChMatrixNM<double, 6, 6>* E_eps;
                double* detJ0C;
                double* theta;

                ChMatrixNM<double, 24, 1> Fint;
                ChMatrixNM<double, 24, 24> JAC11;
                ChMatrixNM<double, 9, 24> Gd;
                ChMatrixNM<double, 6, 1> stress;
                ChMatrixNM<double, 9, 9> Sigm;
                ChMatrixNM<double, 24, 6> temp246;
                ChMatrixNM<double, 24, 9> temp249;
                ChMatrixNM<double, 3, 24> Sx;
                ChMatrixNM<double, 3, 24> Sy;
                ChMatrixNM<double, 3, 24> Sz;
                ChMatrixNM<double, 1, 8> Nx;
                ChMatrixNM<double, 1, 8> Ny;
                ChMatrixNM<double, 1, 8> Nz;
                ChMatrixNM<double, 6, 24> strainD;
                ChMatrixNM<double, 6, 1> strain;
                ChMatrixNM<double, 8, 8> d_d;
                ChMatrixNM<double, 8, 1> ddNx;
                ChMatrixNM<double, 8, 1> ddNy;
                ChMatrixNM<double, 8, 1> ddNz;
                ChMatrixNM<double, 8, 8> d0_d0;
                ChMatrixNM<double, 8, 1> d0d0Nx;
                ChMatrixNM<double, 8, 1> d0d0Ny;
                ChMatrixNM<double, 8, 1> d0d0Nz;
                ChMatrixNM<double, 1, 3> Nxd;
                ChMatrixNM<double, 1, 3> Nyd;
                ChMatrixNM<double, 1, 3> Nzd;
                ChMatrixNM<double, 1, 1> tempA;
                ChMatrixNM<double, 1, 1> tempA1;
                ChMatrixNM<double, 1, 24> tempB;
                ChMatrixNM<double, 24, 6> tempC;
                double detJ0;
                double alphaHHT;
                double betaHHT;
                double gammaHHT;
                // ANS
                ChMatrixNM<double, 1, 8> N;
                ChMatrixNM<double, 1, 4> S_ANS;
                ChMatrixNM<double, 1, 24> tempBB;
                // EAS
                ChMatrixNM<double, 6, 5> M;
                ChMatrixNM<double, 6, 5> G;
                ChMatrixNM<double, 5, 6> GT;
                ChMatrixNM<double, 6, 1> strain_EAS;

                /// Evaluate (strainD'*strain)  at point x
                virtual void Evaluate(ChMatrixNM<double, 750, 1>& result,
                                      const double x,
                                      const double y,
                                      const double z) {
                    element->ShapeFunctions(N, x, y, z);  // ANS used for ZZ strain and strainD
                    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
                    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
                    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

                    element->shapefunction_ANS_BilinearShell(S_ANS, x, y);
                    element->Basis_M(M, x, y, z);  // EAS

                    alphaHHT = -0.2;
                    betaHHT = 0.25 * (1.0 - alphaHHT) * (1.0 - alphaHHT);
                    gammaHHT = 0.5 - alphaHHT;

                    // Expand shape function derivatives in 3x24 matrices
                    ChMatrix33<> Sxi;
                    Sxi.FillDiag(Nx(0));
                    Sx.PasteMatrix(&Sxi, 0, 0);
                    Sxi.FillDiag(Nx(1));
                    Sx.PasteMatrix(&Sxi, 0, 3);
                    Sxi.FillDiag(Nx(2));
                    Sx.PasteMatrix(&Sxi, 0, 6);
                    Sxi.FillDiag(Nx(3));
                    Sx.PasteMatrix(&Sxi, 0, 9);
                    Sxi.FillDiag(Nx(4));
                    Sx.PasteMatrix(&Sxi, 0, 12);
                    Sxi.FillDiag(Nx(5));
                    Sx.PasteMatrix(&Sxi, 0, 15);
                    Sxi.FillDiag(Nx(6));
                    Sx.PasteMatrix(&Sxi, 0, 18);
                    Sxi.FillDiag(Nx(7));
                    Sx.PasteMatrix(&Sxi, 0, 21);

                    ChMatrix33<> Syi;
                    Syi.FillDiag(Ny(0));
                    Sy.PasteMatrix(&Syi, 0, 0);
                    Syi.FillDiag(Ny(1));
                    Sy.PasteMatrix(&Syi, 0, 3);
                    Syi.FillDiag(Ny(2));
                    Sy.PasteMatrix(&Syi, 0, 6);
                    Syi.FillDiag(Ny(3));
                    Sy.PasteMatrix(&Syi, 0, 9);
                    Syi.FillDiag(Ny(4));
                    Sy.PasteMatrix(&Syi, 0, 12);
                    Syi.FillDiag(Ny(5));
                    Sy.PasteMatrix(&Syi, 0, 15);
                    Syi.FillDiag(Ny(6));
                    Sy.PasteMatrix(&Syi, 0, 18);
                    Syi.FillDiag(Ny(7));
                    Sy.PasteMatrix(&Syi, 0, 21);

                    ChMatrix33<> Szi;
                    Szi.FillDiag(Nz(0));
                    Sz.PasteMatrix(&Szi, 0, 0);
                    Szi.FillDiag(Nz(1));
                    Sz.PasteMatrix(&Szi, 0, 3);
                    Szi.FillDiag(Nz(2));
                    Sz.PasteMatrix(&Szi, 0, 6);
                    Szi.FillDiag(Nz(3));
                    Sz.PasteMatrix(&Szi, 0, 9);
                    Szi.FillDiag(Nz(4));
                    Sz.PasteMatrix(&Szi, 0, 12);
                    Szi.FillDiag(Nz(5));
                    Sz.PasteMatrix(&Szi, 0, 15);
                    Szi.FillDiag(Nz(6));
                    Sz.PasteMatrix(&Szi, 0, 18);
                    Szi.FillDiag(Nz(7));
                    Sz.PasteMatrix(&Szi, 0, 21);

                    //==EAS and Initial Shape==//
                    ChMatrixNM<double, 3, 3> rd0;
                    ChMatrixNM<double, 1, 3> temp33;
                    temp33.Reset();
                    temp33 = (Nx * (*d0));
                    temp33.MatrTranspose();
                    rd0.PasteClippedMatrix(&temp33, 0, 0, 3, 1, 0, 0);
                    temp33.MatrTranspose();
                    temp33 = (Ny * (*d0));
                    temp33.MatrTranspose();
                    rd0.PasteClippedMatrix(&temp33, 0, 0, 3, 1, 0, 1);
                    temp33.MatrTranspose();
                    temp33 = (Nz * (*d0));
                    temp33.MatrTranspose();
                    rd0.PasteClippedMatrix(&temp33, 0, 0, 3, 1, 0, 2);
                    detJ0 = rd0.Det();

                    //////////////////////////////////////////////////////////////
                    //// Transformation : Orthogonal transformation (A and J) ////
                    //////////////////////////////////////////////////////////////
                    ChVector<double> G1;
                    ChVector<double> G2;
                    ChVector<double> G3;
                    ChVector<double> G1xG2;
                    double G1dotG1;
                    G1(0) = rd0(0, 0);
                    G2(0) = rd0(0, 1);
                    G3(0) = rd0(0, 2);
                    G1(1) = rd0(1, 0);
                    G2(1) = rd0(1, 1);
                    G3(1) = rd0(1, 2);
                    G1(2) = rd0(2, 0);
                    G2(2) = rd0(2, 1);
                    G3(2) = rd0(2, 2);
                    G1xG2.Cross(G1, G2);
                    G1dotG1 = Vdot(G1, G1);

                    // Tangent Frame
                    ChVector<double> A1;
                    ChVector<double> A2;
                    ChVector<double> A3;
                    A1 = G1 / sqrt(G1(0) * G1(0) + G1(1) * G1(1) + G1(2) * G1(2));
                    A3 = G1xG2 / sqrt(G1xG2(0) * G1xG2(0) + G1xG2(1) * G1xG2(1) + G1xG2(2) * G1xG2(2));
                    A2.Cross(A3, A1);

                    // Direction for orthotropic material
                    ChVector<double> AA1;
                    ChVector<double> AA2;
                    ChVector<double> AA3;

                    AA1 = A1 * cos(*theta) + A2 * sin(*theta);
                    AA2 = -A1 * sin(*theta) + A2 * cos(*theta);
                    AA3 = A3;

                    /// Beta
                    ChMatrixNM<double, 3, 3> j0;
                    ChVector<double> j01;
                    ChVector<double> j02;
                    ChVector<double> j03;
                    ChMatrixNM<double, 9, 1> beta;
                    double temp;
                    j0 = rd0;
                    j0.MatrInverse();
                    j01(0) = j0(0, 0);
                    j02(0) = j0(1, 0);
                    j03(0) = j0(2, 0);
                    j01(1) = j0(0, 1);
                    j02(1) = j0(1, 1);
                    j03(1) = j0(2, 1);
                    j01(2) = j0(0, 2);
                    j02(2) = j0(1, 2);
                    j03(2) = j0(2, 2);
                    temp = Vdot(AA1, j01);
                    beta(0, 0) = temp;
                    temp = Vdot(AA2, j01);
                    beta(1, 0) = temp;
                    temp = Vdot(AA3, j01);
                    beta(2, 0) = temp;
                    temp = Vdot(AA1, j02);
                    beta(3, 0) = temp;
                    temp = Vdot(AA2, j02);
                    beta(4, 0) = temp;
                    temp = Vdot(AA3, j02);
                    beta(5, 0) = temp;
                    temp = Vdot(AA1, j03);
                    beta(6, 0) = temp;
                    temp = Vdot(AA2, j03);
                    beta(7, 0) = temp;
                    temp = Vdot(AA3, j03);
                    beta(8, 0) = temp;

                    //////////////////////////////////////////////////
                    //// Enhanced Assumed Strain /////////////////////
                    //////////////////////////////////////////////////
                    G = (*T0) * M * ((*detJ0C) / (detJ0));
                    strain_EAS = G * (*alpha_eas);

                    //////////////////////////////////////////////////

                    d_d.MatrMultiplyT(*d, *d);
                    ddNx.MatrMultiplyT(d_d, Nx);
                    ddNy.MatrMultiplyT(d_d, Ny);
                    ddNz.MatrMultiplyT(d_d, Nz);

                    d0_d0.MatrMultiplyT(*d0, *d0);
                    d0d0Nx.MatrMultiplyT(d0_d0, Nx);
                    d0d0Ny.MatrMultiplyT(d0_d0, Ny);
                    d0d0Nz.MatrMultiplyT(d0_d0, Nz);

                    ///////////////////////////
                    /// Strain component //////
                    ///////////////////////////
                    ChMatrixNM<double, 6, 1> strain_til;
                    tempA = Nx * ddNx;
                    tempA1 = Nx * d0d0Nx;
                    strain_til(0, 0) = 0.5 * (tempA(0, 0) - tempA1(0, 0));
                    tempA = Ny * ddNy;
                    tempA1 = Ny * d0d0Ny;
                    strain_til(1, 0) = 0.5 * (tempA(0, 0) - tempA1(0, 0));
                    tempA = Nx * ddNy;
                    tempA1 = Nx * d0d0Ny;
                    strain_til(2, 0) = tempA(0, 0) - tempA1(0, 0);
                    //== Compatible strain (No ANS) ==//
                    // tempA=Nz*ddNz;
                    // strain_til(3,0) = 0.5*(tempA(0,0)-1.0);
                    // tempA=Nx*ddNz;
                    // strain_til(4,0) = tempA(0,0);
                    // tempA=Ny*ddNz;
                    // strain_til(5,0) = tempA(0,0);
                    //== Incompatible strain (ANS) ==//
                    strain_til(3, 0) = N(0, 0) * (*strain_ans)(0, 0) + N(0, 2) * (*strain_ans)(1, 0) +
                                       N(0, 4) * (*strain_ans)(2, 0) + N(0, 6) * (*strain_ans)(3, 0);
                    strain_til(4, 0) = S_ANS(0, 2) * (*strain_ans)(6, 0) + S_ANS(0, 3) * (*strain_ans)(7, 0);
                    strain_til(5, 0) = S_ANS(0, 0) * (*strain_ans)(4, 0) + S_ANS(0, 1) * (*strain_ans)(5, 0);
                    //// For orthotropic material ///
                    strain(0, 0) = strain_til(0, 0) * beta(0) * beta(0) + strain_til(1, 0) * beta(3) * beta(3) +
                                   strain_til(2, 0) * beta(0) * beta(3) + strain_til(3, 0) * beta(6) * beta(6) +
                                   strain_til(4, 0) * beta(0) * beta(6) + strain_til(5, 0) * beta(3) * beta(6);
                    strain(1, 0) = strain_til(0, 0) * beta(1) * beta(1) + strain_til(1, 0) * beta(4) * beta(4) +
                                   strain_til(2, 0) * beta(1) * beta(4) + strain_til(3, 0) * beta(7) * beta(7) +
                                   strain_til(4, 0) * beta(1) * beta(7) + strain_til(5, 0) * beta(4) * beta(7);
                    strain(2, 0) = strain_til(0, 0) * 2.0 * beta(0) * beta(1) +
                                   strain_til(1, 0) * 2.0 * beta(3) * beta(4) +
                                   strain_til(2, 0) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                                   strain_til(3, 0) * 2.0 * beta(6) * beta(7) +
                                   strain_til(4, 0) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                                   strain_til(5, 0) * (beta(4) * beta(6) + beta(3) * beta(7));
                    strain(3, 0) = strain_til(0, 0) * beta(2) * beta(2) + strain_til(1, 0) * beta(5) * beta(5) +
                                   strain_til(2, 0) * beta(2) * beta(5) + strain_til(3, 0) * beta(8) * beta(8) +
                                   strain_til(4, 0) * beta(2) * beta(8) + strain_til(5, 0) * beta(5) * beta(8);
                    strain(4, 0) = strain_til(0, 0) * 2.0 * beta(0) * beta(2) +
                                   strain_til(1, 0) * 2.0 * beta(3) * beta(5) +
                                   strain_til(2, 0) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                                   strain_til(3, 0) * 2.0 * beta(6) * beta(8) +
                                   strain_til(4, 0) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                                   strain_til(5, 0) * (beta(5) * beta(6) + beta(3) * beta(8));
                    strain(5, 0) = strain_til(0, 0) * 2.0 * beta(1) * beta(2) +
                                   strain_til(1, 0) * 2.0 * beta(4) * beta(5) +
                                   strain_til(2, 0) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                                   strain_til(3, 0) * 2.0 * beta(7) * beta(8) +
                                   strain_til(4, 0) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                                   strain_til(5, 0) * (beta(5) * beta(7) + beta(4) * beta(8));

                    ////////////////////////////////////
                    /// Straint derivative component ///
                    ////////////////////////////////////
                    ChMatrixNM<double, 6, 24> strainD_til;
                    strainD_til.Reset();
                    tempB = Nx * (*d) * Sx;
                    strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 0, 0);
                    tempB = Ny * (*d) * Sy;
                    strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 1, 0);
                    tempB = Nx * (*d) * Sy + Ny * (*d) * Sx;
                    strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 2, 0);
                    //== Compatible strain (No ANS)==//
                    // tempB = Nz*(*d)*Sz;
                    // strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,3,0);
                    // tempB = Nx*(*d)*Sz + Nz*(*d)*Sx;
                    // strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,4,0);
                    // tempB = Ny*(*d)*Sz + Nz*(*d)*Sy;
                    // strainD_til.PasteClippedMatrix(&tempB,0,0,1,24,5,0);
                    //== Incompatible strain (ANS)==//
                    tempBB.Reset();
                    for (int i = 0; i < 4; i++) {
                        int ij = i * 2;
                        tempB.PasteClippedMatrix(strainD_ans, i, 0, 1, 24, 0, 0);
                        tempB *= N(0, ij);
                        tempBB += tempB;
                    }
                    strainD_til.PasteClippedMatrix(&tempBB, 0, 0, 1, 24, 3, 0);  // strainD for zz
                    //
                    tempBB.Reset();
                    for (int i = 0; i < 2; i++) {
                        int ij = i + 6;
                        int ij1 = i + 2;
                        tempB.PasteClippedMatrix(strainD_ans, ij, 0, 1, 24, 0, 0);
                        tempB *= S_ANS(0, ij1);
                        tempBB += tempB;
                    }
                    strainD_til.PasteClippedMatrix(&tempBB, 0, 0, 1, 24, 4, 0);  // strainD for xz
                    //
                    tempBB.Reset();
                    for (int i = 0; i < 2; i++) {
                        int ij = i + 4;
                        int ij1 = i;
                        tempB.PasteClippedMatrix(strainD_ans, ij, 0, 1, 24, 0, 0);
                        tempB *= S_ANS(0, ij1);
                        tempBB += tempB;
                    }
                    strainD_til.PasteClippedMatrix(&tempBB, 0, 0, 1, 24, 5, 0);  // strainD for yz
                    //// For orthotropic material ///
                    for (int ii = 0; ii < 24; ii++) {
                        strainD(0, ii) =
                            strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                            strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                            strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
                        strainD(1, ii) =
                            strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                            strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                            strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
                        strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) +
                                         strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                                         strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                                         strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                                         strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                                         strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
                        strainD(3, ii) =
                            strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
                            strainD_til(2, ii) * beta(2) * beta(5) + strainD_til(3, ii) * beta(8) * beta(8) +
                            strainD_til(4, ii) * beta(2) * beta(8) + strainD_til(5) * beta(5) * beta(8);
                        strainD(4, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(2) +
                                         strainD_til(1, ii) * 2.0 * beta(3) * beta(5) +
                                         strainD_til(2, ii) * (beta(2) * beta(3) + beta(0) * beta(5)) +
                                         strainD_til(3, ii) * 2.0 * beta(6) * beta(8) +
                                         strainD_til(4, ii) * (beta(2) * beta(6) + beta(0) * beta(8)) +
                                         strainD_til(5, ii) * (beta(5) * beta(6) + beta(3) * beta(8));
                        strainD(5, ii) = strainD_til(0, ii) * 2.0 * beta(1) * beta(2) +
                                         strainD_til(1, ii) * 2.0 * beta(4) * beta(5) +
                                         strainD_til(2, ii) * (beta(2) * beta(4) + beta(1) * beta(5)) +
                                         strainD_til(3, ii) * 2.0 * beta(7) * beta(8) +
                                         strainD_til(4, ii) * (beta(2) * beta(7) + beta(1) * beta(8)) +
                                         strainD_til(5, ii) * (beta(5) * beta(7) + beta(4) * beta(8));
                    }
                    /// Gd (9x24) calculation
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

                    ///////////////////////////////////
                    /// Enhanced Assumed Strain 2nd ///
                    ///////////////////////////////////
                    strain += strain_EAS;

                    //////////////////////////////////////
                    /// Structural damping (6/23/2015) ///
                    //////////////////////////////////////
                    /*ChMatrixNM<double, 6,1> dstrain;
                    dstrain.Reset();
                    int kk=0;
                    for(int ii=0;ii<8;ii++)
                    {
                    dstrain(0,0)+=(strainD(0,kk)*(*v)(ii,0))+(strainD(0,kk+1)*(*v)(ii,1))+(strainD(0,kk+2)*(*v)(ii,2));
                    dstrain(1,0)+=(strainD(1,kk)*(*v)(ii,0))+(strainD(1,kk+1)*(*v)(ii,1))+(strainD(1,kk+2)*(*v)(ii,2));
                    dstrain(2,0)+=(strainD(2,kk)*(*v)(ii,0))+(strainD(2,kk+1)*(*v)(ii,1))+(strainD(2,kk+2)*(*v)(ii,2));
                    dstrain(3,0)+=(strainD(3,kk)*(*v)(ii,0))+(strainD(3,kk+1)*(*v)(ii,1))+(strainD(3,kk+2)*(*v)(ii,2));
                    dstrain(4,0)+=(strainD(4,kk)*(*v)(ii,0))+(strainD(4,kk+1)*(*v)(ii,1))+(strainD(4,kk+2)*(*v)(ii,2));
                    dstrain(5,0)+=(strainD(5,kk)*(*v)(ii,0))+(strainD(5,kk+1)*(*v)(ii,1))+(strainD(5,kk+2)*(*v)(ii,2));
                    kk=kk+3;
                    }
                    kk=0;*/
                    /// Strain time derivative for structural damping
                    ChMatrixNM<double, 6, 1> DEPS;
                    DEPS.Reset();
                    for (int ii = 0; ii < 24; ii++) {
                        DEPS(0, 0) = DEPS(0, 0) + strainD(0, ii) * ((*d_dt)(ii, 0));
                        DEPS(1, 0) = DEPS(1, 0) + strainD(1, ii) * ((*d_dt)(ii, 0));
                        DEPS(2, 0) = DEPS(2, 0) + strainD(2, ii) * ((*d_dt)(ii, 0));
                        DEPS(3, 0) = DEPS(3, 0) + strainD(3, ii) * ((*d_dt)(ii, 0));
                        DEPS(4, 0) = DEPS(4, 0) + strainD(4, ii) * ((*d_dt)(ii, 0));
                        DEPS(5, 0) = DEPS(5, 0) + strainD(5, ii) * ((*d_dt)(ii, 0));
                    }

                    double DampCoefficient = gammaHHT / (betaHHT * element->m_dt);  // dt*gammaHHT;

                    ///////////////////////////////////
                    /// Add structural damping      ///
                    ///////////////////////////////////
                    double stdamp = element->m_Alpha;
                    DEPS *= stdamp;
                    strain += DEPS;

                    /// Stress tensor calculation
                    stress.MatrMultiply(*E_eps, strain);
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

                    /// Jacobian calculation ///
                    temp246.MatrTMultiply(strainD, *E_eps);
                    temp249.MatrTMultiply(Gd, Sigm);
                    JAC11 = (temp246 * strainD * (1.0 + DampCoefficient * (element->m_Alpha))) + temp249 * Gd;
                    JAC11 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                             (element->m_thickness / 2.0);
                    /// Internal force calculation ///
                    tempC.MatrTMultiply(strainD, *E_eps);
                    Fint.MatrMultiply(tempC, strain);
                    Fint *=
                        detJ0 * (element->GetLengthX() / 2) * (element->GetLengthY() / 2) * (element->m_thickness / 2);

                    for (int ii = 0; ii < 5; ii++) {
                        for (int jj = 0; jj < 6; jj++) {
                            GT(ii, jj) = G(jj, ii);
                        }
                    }
                    // for EAS
                    ChMatrixNM<double, 5, 6> temp56;
                    temp56.MatrMultiply(GT, *E_eps);
                    ChMatrixNM<double, 5, 1> HE1;
                    ChMatrixNM<double, 5, 24> GDEPSP;
                    ChMatrixNM<double, 5, 5> KALPHA;
                    ChMatrixNM<double, 120, 1> GDEPSPVec;
                    ChMatrixNM<double, 25, 1> KALPHAVec;
                    ChMatrixNM<double, 576, 1> JACVec;
                    HE1.MatrMultiply(temp56, strain);
                    HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                           (element->m_thickness / 2.0);
                    GDEPSP.MatrMultiply(temp56, strainD);
                    GDEPSP *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                              (element->m_thickness / 2.0);
                    KALPHA.MatrMultiply(temp56, G);
                    KALPHA *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                              (element->m_thickness / 2.0);
                    result.Reset();

                    for (int i = 0; i < 5; i++) {
                        for (int j = 0; j < 24; j++) {
                            GDEPSPVec(i * 24 + j) = GDEPSP(i, j);
                        }
                    }
                    for (int i = 0; i < 5; i++) {
                        for (int j = 0; j < 5; j++) {
                            KALPHAVec(i * 5 + j) = KALPHA(i, j);
                        }
                    }
                    for (int i = 0; i < 24; i++) {
                        for (int j = 0; j < 24; j++) {
                            JACVec(i * 24 + j) = JAC11(i, j);
                        }
                    }

                    /// Total result vector
                    result.PasteClippedMatrix(&Fint, 0, 0, 24, 1, 0, 0);
                    result.PasteClippedMatrix(&HE1, 0, 0, 5, 1, 24, 0);
                    result.PasteClippedMatrix(&GDEPSPVec, 0, 0, 120, 1, 29, 0);
                    result.PasteClippedMatrix(&KALPHAVec, 0, 0, 25, 1, 149, 0);
                    result.PasteClippedMatrix(&JACVec, 0, 0, 576, 1, 174, 0);
                }
            };
            //////////////////////////////////////////////////////////////////////////////////////////////////////////
            ChMatrixNM<double, 750, 1> TempIntegratedResult;
            ChMatrixNM<double, 24, 1> Finternal;
            // Assumed Natural Strain (ANS)
            ChMatrixNM<double, 8, 1> strain_ans;
            ChMatrixNM<double, 8, 24> strainD_ans;
            // Enhanced Assumed Strain (EAS)
            ChMatrixNM<double, 6, 6> T0;
            ChMatrixNM<double, 5, 1> HE;
            ChMatrixNM<double, 5, 24> GDEPSP;
            ChMatrixNM<double, 5, 5> KALPHA;
            ChMatrixNM<double, 24, 24> KTE;
            ChMatrixNM<double, 5, 5> KALPHA1;
            ChMatrixNM<double, 5, 1> ResidHE;
            double detJ0C;
            ChMatrixNM<double, 5, 1> alpha_eas;
            ChMatrixNM<double, 5, 1> renewed_alpha_eas;
            ChMatrixNM<double, 5, 1> previous_alpha;

            int ijkl = kl * 5;
            previous_alpha(0, 0) = StockAlpha1(ijkl);
            previous_alpha(1, 0) = StockAlpha1(ijkl + 1);
            previous_alpha(2, 0) = StockAlpha1(ijkl + 2);
            previous_alpha(3, 0) = StockAlpha1(ijkl + 3);
            previous_alpha(4, 0) = StockAlpha1(ijkl + 4);
            alpha_eas = previous_alpha;
            ResidHE.Reset();
            int count = 0;
            int fail = 1;
            /// Begin EAS loop ///
            while (fail == 1) {
                alpha_eas = alpha_eas - ResidHE;
                renewed_alpha_eas = alpha_eas;

                Finternal.Reset();
                HE.Reset();
                GDEPSP.Reset();
                KALPHA.Reset();
                strain_ans.Reset();
                strainD_ans.Reset();

                // Assumed Natural Strain (ANS)
                AssumedNaturalStrain_BilinearShell(d, m_d0, strain_ans, strainD_ans);

                // Enhanced Assumed Strain (EAS)
                T0.Reset();
                detJ0C = 0.0;
                T0DetJElementCenterForEAS(m_d0, T0, detJ0C, theta);

                MyForce myformula;
                myformula.d = &d;
                // myformula.v = &v;
                myformula.d_dt = &d_dt;  // For Structural Damping
                myformula.strain_ans = &strain_ans;
                myformula.strainD_ans = &strainD_ans;
                myformula.d0 = &m_d0;
                myformula.E_eps = &E_eps;
                myformula.element = this;
                // EAS
                myformula.T0 = &T0;
                myformula.detJ0C = &detJ0C;
                myformula.theta = &theta;
                myformula.alpha_eas = &alpha_eas;
                ChQuadrature::Integrate3D<ChMatrixNM<double, 750, 1> >(
                    TempIntegratedResult,  // result of integration will go there
                    myformula,             // formula to integrate
                    -1,                    // start of x
                    1,                     // end of x
                    -1,                    // start of y
                    1,                     // end of y
                    m_GaussZRange(kl, 0),  // start of z
                    m_GaussZRange(kl, 1),  // end of z
                    2                      // order of integration
                    );

                ///===============================================================//
                ///===TempIntegratedResult(0:23,1) -> InternalForce(24x1)=========//
                ///===TempIntegratedResult(24:28,1) -> HE(5x1)           =========//
                ///===TempIntegratedResult(29:148,1) -> GDEPSP(5x24)     =========//
                ///===TempIntegratedResult(149:173,1) -> KALPHA(5x5)     =========//
                ///===TempIntegratedResult(174:749,1) -> Stiffness Matrix(24x24) =//
                ///===============================================================//
                ChMatrixNM<double, 120, 1> GDEPSPvec;
                ChMatrixNM<double, 25, 1> KALPHAvec;
                ChMatrixNM<double, 576, 1> JACvec;
                /// Storing result vector ///
                Finternal.PasteClippedMatrix(&TempIntegratedResult, 0, 0, 24, 1, 0, 0);    //
                HE.PasteClippedMatrix(&TempIntegratedResult, 24, 0, 5, 1, 0, 0);           //
                GDEPSPvec.PasteClippedMatrix(&TempIntegratedResult, 29, 0, 120, 1, 0, 0);  //
                KALPHAvec.PasteClippedMatrix(&TempIntegratedResult, 149, 0, 25, 1, 0, 0);  //
                JACvec.PasteClippedMatrix(&TempIntegratedResult, 174, 0, 576, 1, 0, 0);    //
                {
                    for (int i = 0; i < 5; i++) {
                        for (int j = 0; j < 24; j++) {
                            GDEPSP(i, j) = GDEPSPvec(i * 24 + j);
                        }
                    }
                    for (int i = 0; i < 5; i++) {
                        for (int j = 0; j < 5; j++) {
                            KALPHA(i, j) = KALPHAvec(i * 5 + j);
                        }
                    }
                    for (int i = 0; i < 24; i++) {
                        for (int j = 0; j < 24; j++) {
                            TempJacobian(i, j) = JACvec(i * 24 + j);
                        }
                    }
                }

                // GDEPSP=GDEPSPvec;
                // KALPHA=KALPHAvec;
                // TempJacobian = JACvec; // For Laminate shell

                KALPHA1 = KALPHA;
                if (m_flag_HE == NUMERICAL)
                    break;  // When numerical jacobian loop, no need to calculate HE
                count = count + 1;
                double norm_HE = HE.NormTwo();

                if (norm_HE < 0.00001) {
                    fail = 0;
                } else {
                    ChMatrixNM<int, 5, 1> INDX;
                    bool pivoting;
                    ResidHE = HE;
                    if (!LU_factor(KALPHA1, INDX, pivoting))
                        throw ChException("Singular matrix in LU factorization");
                    LU_solve(KALPHA1, INDX, ResidHE);
                }

                if (m_flag_HE == ANALYTICAL && count > 2) {
                    GetLog() << m_element_number << "  count " << count << "  NormHE " << norm_HE << "\n";
                }
            }
            Fi -= Finternal;

            //===============================//
            //== Stock_Alpha=================//
            //===============================//
            if (m_flag_HE == ANALYTICAL) {
                int ijkl = kl * 5;
                StockAlpha1(ijkl) = renewed_alpha_eas(0, 0);
                StockAlpha1(ijkl + 1) = renewed_alpha_eas(1, 0);
                StockAlpha1(ijkl + 2) = renewed_alpha_eas(2, 0);
                StockAlpha1(ijkl + 3) = renewed_alpha_eas(3, 0);
                StockAlpha1(ijkl + 4) = renewed_alpha_eas(4, 0);
                if (kl == m_numLayers - 1) {
                    SetStockAlpha(StockAlpha1);
                }
            }

            //===============================//
            //== Jacobian Matrix for alpha ==//
            //===============================//
            if (m_flag_HE == ANALYTICAL) {
                ChMatrixNM<double, 5, 5> INV_KALPHA;
                ChMatrixNM<double, 5, 24> TEMP_GDEPSP;
                ChMatrixNM<double, 5, 5> INV_KALPHA_Temp;
                Inverse55_Analytical(INV_KALPHA, KALPHA);

                TEMP_GDEPSP.MatrMultiply(INV_KALPHA, GDEPSP);
                TempJacobian_EAS.MatrTMultiply(GDEPSP, TEMP_GDEPSP);

                stock_jac_EAS_elem1 += TempJacobian_EAS;
                KTE1 += TempJacobian;

                if (kl == m_numLayers - 1) {
                    this->SetStockJac(stock_jac_EAS_elem1);
                    this->SetStockKTE(KTE1);
                }
            }
        }

        if (kl == 0) {
            // Add Tire Air Pressure force
            class MyAirPressure : public ChIntegrable2D<ChMatrixNM<double, 24, 1> > {
              public:
                ChElementShellANCF* element;
                ChMatrixNM<double, 8, 3>* d0;
                ChMatrixNM<double, 8, 3>* d;
                ChMatrixNM<double, 3, 24> S;
                ChMatrixNM<double, 1, 4> S_ANS;
                ChMatrixNM<double, 1, 8> N;
                ChMatrixNM<double, 1, 8> Nx;
                ChMatrixNM<double, 1, 8> Ny;
                ChMatrixNM<double, 1, 8> Nz;
                ChMatrixNM<double, 3, 1> LocalAirPressure;

                virtual void Evaluate(ChMatrixNM<double, 24, 1>& result, const double x, const double y) {
                    double z = 0.0;
                    element->ShapeFunctions(N, x, y, z);
                    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
                    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
                    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

                    // Weights for Gaussian integration
                    double wx2 = (element->GetLengthX()) / 2.0;
                    double wy2 = (element->GetLengthY()) / 2.0;

                    // Set Air Pressure
                    double Pressure0 = 0;
                    if (element->m_air_pressure_on)
                        Pressure0 = 220.0 * 1000.0;  // 220 KPa

                    // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)]
                    ChMatrix33<> Si;
                    Si.FillDiag(N(0));
                    S.PasteMatrix(&Si, 0, 0);
                    Si.FillDiag(N(1));
                    S.PasteMatrix(&Si, 0, 3);
                    Si.FillDiag(N(2));
                    S.PasteMatrix(&Si, 0, 6);
                    Si.FillDiag(N(3));
                    S.PasteMatrix(&Si, 0, 9);
                    Si.FillDiag(N(4));
                    S.PasteMatrix(&Si, 0, 12);
                    Si.FillDiag(N(5));
                    S.PasteMatrix(&Si, 0, 15);
                    Si.FillDiag(N(6));
                    S.PasteMatrix(&Si, 0, 18);
                    Si.FillDiag(N(7));
                    S.PasteMatrix(&Si, 0, 21);

                    ChMatrixNM<double, 1, 3> Nx_d;
                    Nx_d.MatrMultiply(Nx, *d);

                    ChMatrixNM<double, 1, 3> Ny_d;
                    Ny_d.MatrMultiply(Ny, *d);

                    ChMatrixNM<double, 1, 3> Nz_d;
                    Nz_d.MatrMultiply(Nz, *d);

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

                    ChMatrixNM<double, 1, 3> Nx_d0;
                    Nx_d0.MatrMultiply(Nx, *d0);

                    ChMatrixNM<double, 1, 3> Ny_d0;
                    Ny_d0.MatrMultiply(Ny, *d0);

                    ChMatrixNM<double, 1, 3> Nz_d0;
                    Nz_d0.MatrMultiply(Nz, *d0);

                    ChMatrixNM<double, 3, 3> rd0;
                    rd0(0, 0) = Nx_d0(0, 0);
                    rd0(1, 0) = Nx_d0(0, 1);
                    rd0(2, 0) = Nx_d0(0, 2);
                    rd0(0, 1) = Ny_d0(0, 0);
                    rd0(1, 1) = Ny_d0(0, 1);
                    rd0(2, 1) = Ny_d0(0, 2);
                    rd0(0, 2) = Nz_d0(0, 0);
                    rd0(1, 2) = Nz_d0(0, 1);
                    rd0(2, 2) = Nz_d0(0, 2);

                    double detJ0 = rd0.Det();

                    ChMatrixNM<double, 3, 1> G1xG2;
                    G1xG2(0) = rd(1, 0) * rd(2, 1) - rd(2, 0) * rd(1, 1);
                    G1xG2(1) = rd(2, 0) * rd(0, 1) - rd(0, 0) * rd(2, 1);
                    G1xG2(2) = rd(0, 0) * rd(1, 1) - rd(1, 0) * rd(0, 1);
                    double G1xG2norm = sqrt(G1xG2(0) * G1xG2(0) + G1xG2(1) * G1xG2(1) + G1xG2(2) * G1xG2(2));

                    LocalAirPressure = -G1xG2 * (Pressure0 / G1xG2norm);
                    result.MatrTMultiply(S, LocalAirPressure);

                    result *= detJ0 * wx2 * wy2;  // 6/12/2015
                }
            };

            MyAirPressure myformula2;
            myformula2.d0 = &m_d0;
            myformula2.d = &d;
            myformula2.element = this;

            ChMatrixNM<double, 24, 1> Fpressure;
            ChQuadrature::Integrate2D<ChMatrixNM<double, 24, 1> >(Fpressure,   // result of integration will go there
                                                                  myformula2,  // formula to integrate
                                                                  -1,          // start of x
                                                                  1,           // end of x
                                                                  -1,          // start of y
                                                                  1,           // end of y
                                                                  2            // order of integration
                                                                  );

            Fi += Fpressure;
        }
    }  // Layer Loop

    Fi += m_GravForce;
}

// -----------------------------------------------------------------------------

void ChElementShellANCF::shapefunction_ANS_BilinearShell(ChMatrixNM<double, 1, 4>& S_ANS, double x, double y) {
    S_ANS(0, 0) = -0.5 * x + 0.5;
    S_ANS(0, 1) = 0.5 * x + 0.5;
    S_ANS(0, 2) = -0.5 * y + 0.5;
    S_ANS(0, 3) = 0.5 * y + 0.5;
}

void ChElementShellANCF::AssumedNaturalStrain_BilinearShell(ChMatrixNM<double, 8, 3>& d,
                                                            ChMatrixNM<double, 8, 3>& d0,
                                                            ChMatrixNM<double, 8, 1>& strain_ans,
                                                            ChMatrixNM<double, 8, 24>& strainD_ans) {
    ChMatrixNM<double, 8, 3> temp_knot;
    temp_knot.Reset();
    temp_knot(0, 0) = -1.0;
    temp_knot(0, 1) = -1.0;
    temp_knot(1, 0) = 1.0;
    temp_knot(1, 1) = -1.0;
    temp_knot(2, 0) = -1.0;
    temp_knot(2, 1) = 1.0;
    temp_knot(3, 0) = 1.0;
    temp_knot(3, 1) = 1.0;

    temp_knot(4, 0) = -1.0;  // A
    temp_knot(4, 1) = 0.0;   // A
    temp_knot(5, 0) = 1.0;   // B
    temp_knot(5, 1) = 0.0;   // B

    temp_knot(6, 0) = 0.0;   // C
    temp_knot(6, 1) = -1.0;  // C
    temp_knot(7, 0) = 0.0;   // D
    temp_knot(7, 1) = 1.0;   // D

    ChMatrixNM<double, 3, 24> Sx;
    ChMatrixNM<double, 3, 24> Sy;
    ChMatrixNM<double, 3, 24> Sz;
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 8, 8> d_d;
    ChMatrixNM<double, 8, 1> ddNx;
    ChMatrixNM<double, 8, 1> ddNy;
    ChMatrixNM<double, 8, 1> ddNz;
    ChMatrixNM<double, 8, 8> d0_d0;
    ChMatrixNM<double, 8, 1> d0d0Nx;
    ChMatrixNM<double, 8, 1> d0d0Ny;
    ChMatrixNM<double, 8, 1> d0d0Nz;
    ChMatrixNM<double, 1, 3> Nxd;
    ChMatrixNM<double, 1, 3> Nyd;
    ChMatrixNM<double, 1, 3> Nzd;
    ChMatrixNM<double, 1, 1> tempA;
    ChMatrixNM<double, 1, 1> tempA1;
    ChMatrixNM<double, 1, 24> tempB;

    for (int kk = 0; kk < 8; kk++) {
        ShapeFunctionsDerivativeX(Nx, temp_knot(kk, 0), temp_knot(kk, 1), temp_knot(kk, 2));
        ShapeFunctionsDerivativeY(Ny, temp_knot(kk, 0), temp_knot(kk, 1), temp_knot(kk, 2));
        ShapeFunctionsDerivativeZ(Nz, temp_knot(kk, 0), temp_knot(kk, 1), temp_knot(kk, 2));

        // Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
        ChMatrix33<> Sxi;
        Sxi.FillDiag(Nx(0));
        Sx.PasteMatrix(&Sxi, 0, 0);
        Sxi.FillDiag(Nx(1));
        Sx.PasteMatrix(&Sxi, 0, 3);
        Sxi.FillDiag(Nx(2));
        Sx.PasteMatrix(&Sxi, 0, 6);
        Sxi.FillDiag(Nx(3));
        Sx.PasteMatrix(&Sxi, 0, 9);
        Sxi.FillDiag(Nx(4));
        Sx.PasteMatrix(&Sxi, 0, 12);
        Sxi.FillDiag(Nx(5));
        Sx.PasteMatrix(&Sxi, 0, 15);
        Sxi.FillDiag(Nx(6));
        Sx.PasteMatrix(&Sxi, 0, 18);
        Sxi.FillDiag(Nx(7));
        Sx.PasteMatrix(&Sxi, 0, 21);

        ChMatrix33<> Syi;
        Syi.FillDiag(Ny(0));
        Sy.PasteMatrix(&Syi, 0, 0);
        Syi.FillDiag(Ny(1));
        Sy.PasteMatrix(&Syi, 0, 3);
        Syi.FillDiag(Ny(2));
        Sy.PasteMatrix(&Syi, 0, 6);
        Syi.FillDiag(Ny(3));
        Sy.PasteMatrix(&Syi, 0, 9);
        Syi.FillDiag(Ny(4));
        Sy.PasteMatrix(&Syi, 0, 12);
        Syi.FillDiag(Ny(5));
        Sy.PasteMatrix(&Syi, 0, 15);
        Syi.FillDiag(Ny(6));
        Sy.PasteMatrix(&Syi, 0, 18);
        Syi.FillDiag(Ny(7));
        Sy.PasteMatrix(&Syi, 0, 21);

        ChMatrix33<> Szi;
        Szi.FillDiag(Nz(0));
        Sz.PasteMatrix(&Szi, 0, 0);
        Szi.FillDiag(Nz(1));
        Sz.PasteMatrix(&Szi, 0, 3);
        Szi.FillDiag(Nz(2));
        Sz.PasteMatrix(&Szi, 0, 6);
        Szi.FillDiag(Nz(3));
        Sz.PasteMatrix(&Szi, 0, 9);
        Szi.FillDiag(Nz(4));
        Sz.PasteMatrix(&Szi, 0, 12);
        Szi.FillDiag(Nz(5));
        Sz.PasteMatrix(&Szi, 0, 15);
        Szi.FillDiag(Nz(6));
        Sz.PasteMatrix(&Szi, 0, 18);
        Szi.FillDiag(Nz(7));
        Sz.PasteMatrix(&Szi, 0, 21);

        d_d.MatrMultiplyT(d, d);
        ddNx.MatrMultiplyT(d_d, Nx);
        ddNy.MatrMultiplyT(d_d, Ny);
        ddNz.MatrMultiplyT(d_d, Nz);

        d0_d0.MatrMultiplyT(d0, d0);
        d0d0Nx.MatrMultiplyT(d0_d0, Nx);
        d0d0Ny.MatrMultiplyT(d0_d0, Ny);
        d0d0Nz.MatrMultiplyT(d0_d0, Nz);

        if (kk == 0 || kk == 1 || kk == 2 || kk == 3) {
            tempA = Nz * ddNz;
            tempA1 = Nz * d0d0Nz;
            strain_ans(kk, 0) = 0.5 * (tempA(0, 0) - tempA1(0, 0));
            tempB = Nz * (d)*Sz;
            strainD_ans.PasteClippedMatrix(&tempB, 0, 0, 1, 24, kk, 0);
        }
        if (kk == 4 || kk == 5) {  // kk=4,5 =>yz
            tempA = Ny * ddNz;
            tempA1 = Ny * d0d0Nz;
            strain_ans(kk, 0) = tempA(0, 0) - tempA1(0, 0);
            tempB = Ny * (d)*Sz + Nz * (d)*Sy;
            strainD_ans.PasteClippedMatrix(&tempB, 0, 0, 1, 24, kk, 0);
        }
        if (kk == 6 || kk == 7) {  // kk=6,7 =>xz
            tempA = Nx * ddNz;
            tempA1 = Nx * d0d0Nz;
            strain_ans(kk, 0) = tempA(0, 0) - tempA1(0, 0);
            tempB = Nx * (d)*Sz + Nz * (d)*Sx;
            strainD_ans.PasteClippedMatrix(&tempB, 0, 0, 1, 24, kk, 0);
        }
    }
}

// -----------------------------------------------------------------------------

void ChElementShellANCF::Basis_M(ChMatrixNM<double, 6, 5>& M, double x, double y, double z) {
    M.Reset();
    M(0, 0) = x;
    M(1, 1) = y;
    M(2, 2) = x;
    M(2, 3) = y;
    M(3, 4) = z;
}

// -----------------------------------------------------------------------------

void ChElementShellANCF::T0DetJElementCenterForEAS(ChMatrixNM<double, 8, 3>& d0,
                                                   ChMatrixNM<double, 6, 6>& T0,
                                                   double& detJ0C,
                                                   double& theta) {
    double x = 0;
    double y = 0;
    double z = 0;
    // ChMatrixNM<double,6,6> T0;
    // double detJ0C;
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 3, 3> rd0;
    ChMatrixNM<double, 1, 3> tempA;
    tempA.Reset();
    ShapeFunctionsDerivativeX(Nx, x, y, z);
    ShapeFunctionsDerivativeY(Ny, x, y, z);
    ShapeFunctionsDerivativeZ(Nz, x, y, z);
    tempA = (Nx * d0);
    tempA.MatrTranspose();
    rd0.PasteClippedMatrix(&tempA, 0, 0, 3, 1, 0, 0);
    tempA.MatrTranspose();
    tempA = (Ny * d0);
    tempA.MatrTranspose();
    rd0.PasteClippedMatrix(&tempA, 0, 0, 3, 1, 0, 1);
    tempA.MatrTranspose();
    tempA = (Nz * d0);
    tempA.MatrTranspose();
    rd0.PasteClippedMatrix(&tempA, 0, 0, 3, 1, 0, 2);
    detJ0C = rd0.Det();

    //////////////////////////////////////////////////////////////
    //// Transformation : Orthogonal transformation (A and J) ////
    //////////////////////////////////////////////////////////////
    ChVector<double> G1;
    ChVector<double> G2;
    ChVector<double> G3;
    ChVector<double> G1xG2;
    double G1dotG1;
    // G1.PasteClippedMatrix(&rd0,0,0,3,1,0,0);
    // G2.PasteClippedMatrix(&rd0,0,1,3,1,0,0);
    G1(0) = rd0(0, 0);
    G2(0) = rd0(0, 1);
    G3(0) = rd0(0, 2);
    G1(1) = rd0(1, 0);
    G2(1) = rd0(1, 1);
    G3(1) = rd0(1, 2);
    G1(2) = rd0(2, 0);
    G2(2) = rd0(2, 1);
    G3(2) = rd0(2, 2);
    G1xG2.Cross(G1, G2);
    G1dotG1 = Vdot(G1, G1);

    ////Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    // A1=G1.Normalize();
    A1 = G1 / sqrt(G1(0) * G1(0) + G1(1) * G1(1) + G1(2) * G1(2));
    A3 = G1xG2 / sqrt(G1xG2(0) * G1xG2(0) + G1xG2(1) * G1xG2(1) + G1xG2(2) * G1xG2(2));
    A2.Cross(A3, A1);
    // double theta = 0.0;
    // if(NonlinearMaterialFlag==0){theta = 0.0;}
    // if(NonlinearMaterialFlag==2){theta = INRTAFlex[i][kl][4] * PI/180.0;}
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(theta) + A2 * sin(theta);
    AA2 = -A1 * sin(theta) + A2 * cos(theta);
    AA3 = A3;

    ////Beta
    ChMatrixNM<double, 3, 3> j0;
    ChVector<double> j01;
    ChVector<double> j02;
    ChVector<double> j03;
    ChMatrixNM<double, 9, 1> beta;
    double temp;
    j0 = rd0;
    j0.MatrInverse();
    // j01(0)=j0(0,0); j02(0)=j0(0,1); j03(0)=j0(0,2);
    // j01(1)=j0(1,0); j02(1)=j0(1,1); j03(1)=j0(1,2);
    // j01(2)=j0(2,0); j02(2)=j0(2,1); j03(2)=j0(2,2);
    j01(0) = j0(0, 0);
    j02(0) = j0(1, 0);
    j03(0) = j0(2, 0);
    j01(1) = j0(0, 1);
    j02(1) = j0(1, 1);
    j03(1) = j0(2, 1);
    j01(2) = j0(0, 2);
    j02(2) = j0(1, 2);
    j03(2) = j0(2, 2);
    temp = Vdot(AA1, j01);
    beta(0, 0) = temp;
    temp = Vdot(AA2, j01);
    beta(1, 0) = temp;
    temp = Vdot(AA3, j01);
    beta(2, 0) = temp;
    temp = Vdot(AA1, j02);
    beta(3, 0) = temp;
    temp = Vdot(AA2, j02);
    beta(4, 0) = temp;
    temp = Vdot(AA3, j02);
    beta(5, 0) = temp;
    temp = Vdot(AA1, j03);
    beta(6, 0) = temp;
    temp = Vdot(AA2, j03);
    beta(7, 0) = temp;
    temp = Vdot(AA3, j03);
    beta(8, 0) = temp;

    ////T0
    T0(0, 0) = pow(beta(0), 2);
    T0(1, 0) = pow(beta(1), 2);
    T0(2, 0) = 2.0 * beta(0) * beta(1);
    T0(3, 0) = pow(beta(2), 2);
    T0(4, 0) = 2.0 * beta(0) * beta(2);
    T0(5, 0) = 2.0 * beta(1) * beta(2);

    T0(0, 1) = pow(beta(3), 2);
    T0(1, 1) = pow(beta(4), 2);
    T0(2, 1) = 2.0 * beta(3) * beta(4);
    T0(3, 1) = pow(beta(5), 2);
    T0(4, 1) = 2.0 * beta(3) * beta(5);
    T0(5, 1) = 2.0 * beta(4) * beta(5);

    T0(0, 2) = beta(0) * beta(3);
    T0(1, 2) = beta(1) * beta(4);
    T0(2, 2) = beta(0) * beta(4) + beta(1) * beta(3);
    T0(3, 2) = beta(2) * beta(5);
    T0(4, 2) = beta(0) * beta(5) + beta(2) * beta(3);
    T0(5, 2) = beta(2) * beta(4) + beta(1) * beta(5);

    T0(0, 3) = pow(beta(6), 2);
    T0(1, 3) = pow(beta(7), 2);
    T0(2, 3) = 2.0 * beta(6) * beta(7);
    T0(3, 3) = pow(beta(8), 2);
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

// -----------------------------------------------------------------------------

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

    ChVector<> pA = m_nodes[0]->GetPos();
    ChVector<> pB = m_nodes[1]->GetPos();
    ChVector<> pC = m_nodes[2]->GetPos();
    ChVector<> pD = m_nodes[3]->GetPos();

    point.x = N(0) * pA.x + N(2) * pB.x + N(4) * pC.x + N(6) * pD.x;
    point.y = N(0) * pA.y + N(2) * pB.y + N(4) * pC.y + N(6) * pD.y;
    point.z = N(0) * pA.z + N(2) * pB.z + N(4) * pC.z + N(6) * pD.z;
}

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
            if (preValue < abs(b(ii, k))) {
                preValue = abs(b(ii, k));
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

// -----------------------------------------------------------------------------

void ChElementShellANCF::Inverse55_Analytical(ChMatrixNM<double, 5, 5>& A, ChMatrixNM<double, 5, 5>& B) {
    double a1 = B(0, 0);
    double a2 = B(0, 1);
    double a3 = B(0, 2);
    double a4 = B(0, 3);
    double a5 = B(0, 4);
    double b1 = B(1, 0);
    double b2 = B(1, 1);
    double b3 = B(1, 2);
    double b4 = B(1, 3);
    double b5 = B(1, 4);
    double c1 = B(2, 0);
    double c2 = B(2, 1);
    double c3 = B(2, 2);
    double c4 = B(2, 3);
    double c5 = B(2, 4);
    double d1 = B(3, 0);
    double d2 = B(3, 1);
    double d3 = B(3, 2);
    double d4 = B(3, 3);
    double d5 = B(3, 4);
    double e1 = B(4, 0);
    double e2 = B(4, 1);
    double e3 = B(4, 2);
    double e4 = B(4, 3);
    double e5 = B(4, 4);

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

}  // end of namespace fea
}  // end of namespace chrono
