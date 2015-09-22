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
// Authors: Bryan Peterson, Antonio Recuero
// =============================================================================
//
// Brick element with 8 nodes (with EAS)
//
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono_fea/ChElementGeneric.h"
#include "chrono_fea/ChElementBrick.h"
#include "chrono_fea/ChUtilsFEA.h"
#include "chrono/core/ChQuadrature.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
ChElementBrick::ChElementBrick() {
    m_nodes.resize(8);
}

// -----------------------------------------------------------------------------

void ChElementBrick::SetNodes(ChSharedPtr<ChNodeFEAxyz> nodeA,
                              ChSharedPtr<ChNodeFEAxyz> nodeB,
                              ChSharedPtr<ChNodeFEAxyz> nodeC,
                              ChSharedPtr<ChNodeFEAxyz> nodeD,
                              ChSharedPtr<ChNodeFEAxyz> nodeE,
                              ChSharedPtr<ChNodeFEAxyz> nodeF,
                              ChSharedPtr<ChNodeFEAxyz> nodeG,
                              ChSharedPtr<ChNodeFEAxyz> nodeH) {
    assert(!nodeA.IsNull());
    assert(!nodeB.IsNull());
    assert(!nodeC.IsNull());
    assert(!nodeD.IsNull());
    assert(!nodeE.IsNull());
    assert(!nodeF.IsNull());
    assert(!nodeG.IsNull());
    assert(!nodeH.IsNull());

    m_nodes[0] = nodeA;
	m_nodes[1] = nodeB;
	m_nodes[2] = nodeC;
	m_nodes[3] = nodeD;
	m_nodes[4] = nodeE;
	m_nodes[5] = nodeF;
	m_nodes[6] = nodeG;
	m_nodes[7] = nodeH;
    std::vector<ChLcpVariables*> mvars;
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
	ChVector<> pA = m_nodes[0]->GetPos();
	ChVector<> pB = m_nodes[1]->GetPos();
	ChVector<> pC = m_nodes[2]->GetPos();
	ChVector<> pD = m_nodes[3]->GetPos();
	ChVector<> pE = m_nodes[4]->GetPos();
	ChVector<> pF = m_nodes[5]->GetPos();
	ChVector<> pG = m_nodes[6]->GetPos();
	ChVector<> pH = m_nodes[7]->GetPos();
	m_initialpos(0, 0) = pA(0);
	m_initialpos(1, 0) = pA(1);
	m_initialpos(2, 0) = pA(2);
	m_initialpos(3, 0) = pB(0);
	m_initialpos(4, 0) = pB(1);
	m_initialpos(5, 0) = pB(2);
	m_initialpos(6, 0) = pC(0);
	m_initialpos(7, 0) = pC(1);
	m_initialpos(8, 0) = pC(2);
	m_initialpos(9, 0) = pD(0);
	m_initialpos(10, 0) = pD(1);
	m_initialpos(11, 0) = pD(2);
	m_initialpos(12, 0) = pE(0);
	m_initialpos(13, 0) = pE(1);
	m_initialpos(14, 0) = pE(2);
	m_initialpos(15, 0) = pF(0);
	m_initialpos(16, 0) = pF(1);
	m_initialpos(17, 0) = pF(2);
	m_initialpos(18, 0) = pG(0);
	m_initialpos(19, 0) = pG(1);
	m_initialpos(20, 0) = pG(2);
	m_initialpos(21, 0) = pH(0);
	m_initialpos(22, 0) = pH(1);
	m_initialpos(23, 0) = pH(2);
    // EAS
}

// -----------------------------------------------------------------------------

void ChElementBrick::SetStockAlpha(double a1,
                                   double a2,
                                   double a3,
                                   double a4,
                                   double a5,
                                   double a6,
                                   double a7,
                                   double a8,
                                   double a9) {
	m_stock_alpha_EAS(0, 0) = a1;  //// 2015/5/23  only for 10by1 bench mark
	m_stock_alpha_EAS(1, 0) = a2;
	m_stock_alpha_EAS(2, 0) = a3;
	m_stock_alpha_EAS(3, 0) = a4;
	m_stock_alpha_EAS(4, 0) = a5;
	m_stock_alpha_EAS(5, 0) = a6;
	m_stock_alpha_EAS(6, 0) = a7;
	m_stock_alpha_EAS(7, 0) = a8;
	m_stock_alpha_EAS(8, 0) = a9;
}

// -----------------------------------------------------------------------------

void ChElementBrick::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    int i = GetElemNum();

	ChVector<> pA = m_nodes[0]->GetPos();
	ChVector<> pB = m_nodes[1]->GetPos();
	ChVector<> pC = m_nodes[2]->GetPos();
	ChVector<> pD = m_nodes[3]->GetPos();
	ChVector<> pE = m_nodes[4]->GetPos();
	ChVector<> pF = m_nodes[5]->GetPos();
	ChVector<> pG = m_nodes[6]->GetPos();
	ChVector<> pH = m_nodes[7]->GetPos();

    ChMatrixNM<double, 8, 3> d;
    d(0, 0) = pA.x;
    d(0, 1) = pA.y;
    d(0, 2) = pA.z;
    d(1, 0) = pB.x;
    d(1, 1) = pB.y;
    d(1, 2) = pB.z;
    d(2, 0) = pC.x;
    d(2, 1) = pC.y;
    d(2, 2) = pC.z;
    d(3, 0) = pD.x;
    d(3, 1) = pD.y;
    d(3, 2) = pD.z;
    d(4, 0) = pE.x;
    d(4, 1) = pE.y;
    d(4, 2) = pE.z;
    d(5, 0) = pF.x;
    d(5, 1) = pF.y;
    d(5, 2) = pF.z;
    d(6, 0) = pG.x;
    d(6, 1) = pG.y;
    d(6, 2) = pG.z;
    d(7, 0) = pH.x;
    d(7, 1) = pH.y;
    d(7, 2) = pH.z;

    /// Initial nodal coordinates
    ChMatrixNM<double, 24, 1> InitialCoord;
    ChMatrixNM<double, 8, 3> d0;
    InitialCoord = GetInitialPos();
    d0(0, 0) = InitialCoord(0, 0);
    d0(0, 1) = InitialCoord(1, 0);
    d0(0, 2) = InitialCoord(2, 0);
    d0(1, 0) = InitialCoord(3, 0);
    d0(1, 1) = InitialCoord(4, 0);
    d0(1, 2) = InitialCoord(5, 0);
    d0(2, 0) = InitialCoord(6, 0);
    d0(2, 1) = InitialCoord(7, 0);
    d0(2, 2) = InitialCoord(8, 0);
    d0(3, 0) = InitialCoord(9, 0);
    d0(3, 1) = InitialCoord(10, 0);
    d0(3, 2) = InitialCoord(11, 0);
    d0(4, 0) = InitialCoord(12, 0);
    d0(4, 1) = InitialCoord(13, 0);
    d0(4, 2) = InitialCoord(14, 0);
    d0(5, 0) = InitialCoord(15, 0);
    d0(5, 1) = InitialCoord(16, 0);
    d0(5, 2) = InitialCoord(17, 0);
    d0(6, 0) = InitialCoord(18, 0);
    d0(6, 1) = InitialCoord(19, 0);
    d0(6, 2) = InitialCoord(20, 0);
    d0(7, 0) = InitialCoord(21, 0);
    d0(7, 1) = InitialCoord(22, 0);
    d0(7, 2) = InitialCoord(23, 0);

	double v = m_Material->Get_v();
	double E = m_Material->Get_E();

    Fi.Reset();

    /// If numerical differentiation is used, only the internal force and EAS stiffness
    /// will be calculated. If the numerical differentiation is not used, the jacobian
    /// will also be calculated.
    bool use_numerical_differentiation = false;

    /// Internal force and EAS parameters are caulculated for numerical differentiation.
    if (use_numerical_differentiation) {
        class MyForce : public ChIntegrable3D<ChMatrixNM<double, 330, 1> > {
          public:
            ChElementBrick* element;
            /// Pointers used for external values
            ChMatrixNM<double, 8, 3>* d;
            ChMatrixNM<double, 8, 3>* d0;
            ChMatrixNM<double, 6, 6>* T0;
            ChMatrixNM<double, 9, 1>* alpha_eas;
            double* detJ0C;
            double* E;
            double* v;

            ChMatrixNM<double, 24, 1> Fint;
            ChMatrixNM<double, 6, 6> E_eps;
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
            ChMatrixNM<double, 1, 3> Nxd;
            ChMatrixNM<double, 1, 3> Nyd;
            ChMatrixNM<double, 1, 3> Nzd;
            ChMatrixNM<double, 1, 1> tempA;
            ChMatrixNM<double, 1, 24> tempB;
            ChMatrixNM<double, 24, 6> tempC;
            ChMatrixNM<double, 1, 1> tempA1;  // for strain incase of initial curved
            ChMatrixNM<double, 8, 8> d0_d0;   // for strain incase of initial curved
            ChMatrixNM<double, 8, 1> d0d0Nx;  // for strain incase of initial curved
            ChMatrixNM<double, 8, 1> d0d0Ny;  // for strain incase of initial curved
            ChMatrixNM<double, 8, 1> d0d0Nz;  // for strain incase of initial curved
            double detJ0;
            // EAS
            ChMatrixNM<double, 6, 9> M;
            ChMatrixNM<double, 6, 9> G;
            ChMatrixNM<double, 9, 6> GT;
            ChMatrixNM<double, 6, 1> strain_EAS;

            /// Gaussian integration to calculate internal forces and EAS matrices
            virtual void Evaluate(ChMatrixNM<double, 330, 1>& result, const double x, const double y, const double z) {
                element->ShapeFunctionsDerivativeX(Nx, x, y, z);
                element->ShapeFunctionsDerivativeY(Ny, x, y, z);
                element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

                element->Basis_M(M, x, y, z);  // EAS

                int flag_Mooney = 1;  // 0 means use linear material; 1 means use nonlinear Mooney_Rivlin material

                if (flag_Mooney == 0) {  // 0 means use linear material
                    double DD = (*E) * (1.0 - (*v)) / ((1.0 + (*v)) * (1.0 - 2.0 * (*v)));
                    E_eps.FillDiag(1.0);
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
                ChMatrixNM<double, 3, 3> temp33;
                temp33.Reset();
                temp33 = (Nx * (*d0));
                temp33.MatrTranspose();
                rd0.PasteClippedMatrix(&temp33, 0, 0, 3, 1, 0, 0);
                temp33 = (Ny * (*d0));
                temp33.MatrTranspose();
                rd0.PasteClippedMatrix(&temp33, 0, 0, 3, 1, 0, 1);
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

                ////Tangent Frame
                ChVector<double> A1;
                ChVector<double> A2;
                ChVector<double> A3;
                A1 = G1 / sqrt(G1(0) * G1(0) + G1(1) * G1(1) + G1(2) * G1(2));
                A3 = G1xG2 / sqrt(G1xG2(0) * G1xG2(0) + G1xG2(1) * G1xG2(1) + G1xG2(2) * G1xG2(2));
                A2.Cross(A3, A1);

                ////Direction for orthotropic material//
                double theta = 0.0;
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
                tempA = Nz * ddNz;
                tempA1 = Nz * d0d0Nz;
                strain_til(3, 0) = 0.5 * (tempA(0, 0) - tempA1(0, 0));
                tempA = Nx * ddNz;
                tempA1 = Nx * d0d0Nz;
                strain_til(4, 0) = tempA(0, 0) - tempA1(0, 0);
                tempA = Ny * ddNz;
                tempA1 = Ny * d0d0Nz;
                strain_til(5, 0) = tempA(0, 0) - tempA1(0, 0);
                //// For orthotropic material ///
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
                tempB = Nz * (*d) * Sz;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 3, 0);
                tempB = Nx * (*d) * Sz + Nz * (*d) * Sx;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 4, 0);
                tempB = Ny * (*d) * Sz + Nz * (*d) * Sy;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 5, 0);

                //// For orthotropic material ///
                for (int ii = 0; ii < 24; ii++) {
                    strainD(0, ii) = strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                                     strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                                     strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
                    strainD(1, ii) = strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                                     strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                                     strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
                    strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) +
                                     strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                                     strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                                     strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                                     strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                                     strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
                    strainD(3, ii) = strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
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
                // for(int i=0;i<6;i++){
                //	for(int j=0; j<24; j++){
                //	GetLog() <<strainD(i,j)<<"\n";
                //	}
                //	system("pause");
                //}
                // system("pause");
                ///////////////////////////////////
                /// Enhanced Assumed Strain 2nd ///
                ///////////////////////////////////
                strain += strain_EAS;  // same as EPS in FORTRAN

                ChMatrixNM<double, 9, 6> temp56;  // same as TEMP1 in FORTRAN
                ChMatrixNM<double, 9, 1> HE1;
                ChMatrixNM<double, 9, 24> GDEPSP;
                ChMatrixNM<double, 9, 9> KALPHA;

                for (int ii = 0; ii < 9; ii++) {
                    for (int jj = 0; jj < 6; jj++) {
                        GT(ii, jj) = G(jj, ii);
                    }
                }

                // 1=use Iso_Nonlinear_Mooney-Rivlin Material (2-parameters=> 3 inputs)
                if (flag_Mooney == 1) {
                    ChMatrixNM<double, 3, 3> CG;  // CG: Right Cauchy-Green tensor  C=trans(F)*F
                    ChMatrixNM<double, 3, 3> INVCG;
                    ChMatrixNM<double, 3, 3> IMAT;
                    ChMatrixNM<double, 3, 3> I1PC;
                    ChMatrixNM<double, 3, 3> I2PC;
                    ChMatrixNM<double, 3, 3> JPC;
                    ChMatrixNM<double, 3, 3> STR;

                    ChMatrixNM<double, 3, 3> CGN;
                    ChMatrixNM<double, 3, 3> INVCGN;
                    ChMatrixNM<double, 3, 3> I1PCN;
                    ChMatrixNM<double, 3, 3> I2PCN;
                    ChMatrixNM<double, 3, 3> JPCN;
                    ChMatrixNM<double, 3, 3> STRN;

                    ChMatrixNM<double, 6, 1> strain_1;
                    ChMatrixNM<double, 6, 1> TEMP5;
                    ChMatrixNM<double, 6, 1> TEMP5N;
                    TEMP5.Reset();
                    TEMP5N.Reset();

                    CG(0, 0) = 2.0 * strain(0, 0) + 1.0;
                    CG(1, 1) = 2.0 * strain(1, 0) + 1.0;
                    CG(2, 2) = 2.0 * strain(3, 0) + 1.0;
                    CG(1, 0) = strain(2, 0);
                    CG(0, 1) = CG(1, 0);
                    CG(2, 0) = strain(4, 0);
                    CG(0, 2) = CG(2, 0);
                    CG(2, 1) = strain(5, 0);
                    CG(1, 2) = CG(2, 1);

                    INVCG = CG;
                    INVCG.MatrInverse();

                    double Deld = 0.000001;
                    double I1 = CG(0, 0) + CG(1, 1) + CG(2, 2);
                    double I2 = 0.5 * (pow(I1, 2) - (pow(CG(0, 0), 2) + pow(CG(1, 0), 2) + pow(CG(2, 0), 2) +
                                                     pow(CG(0, 1), 2) + pow(CG(1, 1), 2) + pow(CG(2, 1), 2) +
                                                     pow(CG(0, 2), 2) + pow(CG(1, 2), 2) + pow(CG(2, 2), 2)));
                    double I3 = CG(0, 0) * CG(1, 1) * CG(2, 2) - CG(0, 0) * CG(1, 2) * CG(2, 1) +
                                CG(0, 1) * CG(1, 2) * CG(2, 0) - CG(0, 1) * CG(1, 0) * CG(2, 2) +
                                CG(0, 2) * CG(1, 0) * CG(2, 1) - CG(2, 0) * CG(1, 1) * CG(0, 2);
                    double I1BAR = I1 / (pow(I3, 1.0 / 3.0));
                    double I2BAR = I2 / (pow(I3, 2.0 / 3.0));
                    double J = sqrt(I3);
                    double CCOM1 = 551584.0;                                    // C10   not 0.551584
                    double CCOM2 = 137896.0;                                    // C01   not 0.137896
                    double CCOM3 = 2.0 * (CCOM1 + CCOM2) / (1.0 - 2.0 * 0.49);  // K:bulk modulus
                    double StockEPS;

                    IMAT.Reset();
                    IMAT(0, 0) = 1.0;
                    IMAT(1, 1) = 1.0;
                    IMAT(2, 2) = 1.0;

                    I1PC = (IMAT - INVCG * (1.0 / 3.0 * I1)) * pow(I3, -1.0 / 3.0);
                    I2PC = (((IMAT * I1) - CG) - (INVCG * (2.0 / 3.0) * I2)) * pow(I3, -2.0 / 3.0);
                    JPC = INVCG * (J / 2.0);

                    STR = I1PC * (CCOM1 * 2.0) + I2PC * (CCOM2 * 2.0) + JPC * (CCOM3 * (J - 1.0) * 2.0);

                    TEMP5(0, 0) = STR(0, 0);
                    TEMP5(1, 0) = STR(1, 1);
                    TEMP5(2, 0) = STR(0, 1);
                    TEMP5(3, 0) = STR(2, 2);
                    TEMP5(4, 0) = STR(0, 2);
                    TEMP5(5, 0) = STR(1, 2);

                    E_eps.Reset();

                    strain_1 = strain;
                    for (int JJJ = 0; JJJ < 6; JJJ++) {
                        StockEPS = strain_1(JJJ, 0);
                        strain_1(JJJ, 0) = StockEPS + Deld;
                        CGN(0, 0) = 2.0 * strain_1(0, 0) + 1.0;
                        CGN(1, 1) = 2.0 * strain_1(1, 0) + 1.0;
                        CGN(2, 2) = 2.0 * strain_1(3, 0) + 1.0;
                        CGN(1, 0) = strain_1(2, 0);
                        CGN(0, 1) = CGN(1, 0);
                        CGN(2, 0) = strain_1(4, 0);
                        CGN(0, 2) = CGN(2, 0);
                        CGN(2, 1) = strain_1(5, 0);
                        CGN(1, 2) = CGN(2, 1);
                        INVCGN = CGN;
                        INVCGN.MatrInverse();
                        I1 = CGN(0, 0) + CGN(1, 1) + CGN(2, 2);
                        I2 = 0.5 * (pow(I1, 2) - (pow(CGN(0, 0), 2) + pow(CGN(1, 0), 2) + pow(CGN(2, 0), 2) +
                                                  pow(CGN(0, 1), 2) + pow(CGN(1, 1), 2) + pow(CGN(2, 1), 2) +
                                                  pow(CGN(0, 2), 2) + pow(CGN(1, 2), 2) + pow(CGN(2, 2), 2)));
                        I3 = CGN(0, 0) * CGN(1, 1) * CGN(2, 2) - CGN(0, 0) * CGN(1, 2) * CGN(2, 1) +
                             CGN(0, 1) * CGN(1, 2) * CGN(2, 0) - CGN(0, 1) * CGN(1, 0) * CGN(2, 2) +
                             CGN(0, 2) * CGN(1, 0) * CGN(2, 1) - CGN(2, 0) * CGN(1, 1) * CGN(0, 2);
                        J = sqrt(I3);
                        I1BAR = I1 / (pow(I3, 1.0 / 3.0));
                        I2BAR = I2 / (pow(I3, 2.0 / 3.0));
                        I1PCN = (IMAT - INVCGN * (1.0 / 3.0 * I1)) * pow(I3, -1.0 / 3.0);
                        I2PCN = (((IMAT * I1) - CGN) - (INVCGN * (2.0 / 3.0) * I2)) * pow(I3, -2.0 / 3.0);
                        JPCN = INVCGN * (J / 2.0);
                        STRN = I1PCN * (CCOM1 * 2.0) + I2PCN * (CCOM2 * 2.0) + JPCN * (CCOM3 * (J - 1.0) * 2.0);
                        TEMP5N(0, 0) = STRN(0, 0);
                        TEMP5N(1, 0) = STRN(1, 1);
                        TEMP5N(2, 0) = STRN(0, 1);
                        TEMP5N(3, 0) = STRN(2, 2);
                        TEMP5N(4, 0) = STRN(0, 2);
                        TEMP5N(5, 0) = STRN(1, 2);
                        strain_1(JJJ, 0) = StockEPS;
                        E_eps(JJJ, 0) = (TEMP5N(0, 0) - TEMP5(0, 0)) / Deld;
                        E_eps(JJJ, 1) = (TEMP5N(1, 0) - TEMP5(1, 0)) / Deld;
                        E_eps(JJJ, 2) = (TEMP5N(2, 0) - TEMP5(2, 0)) / Deld;
                        E_eps(JJJ, 3) = (TEMP5N(3, 0) - TEMP5(3, 0)) / Deld;
                        E_eps(JJJ, 4) = (TEMP5N(4, 0) - TEMP5(4, 0)) / Deld;
                        E_eps(JJJ, 5) = (TEMP5N(5, 0) - TEMP5(5, 0)) / Deld;
                    }
                    temp56.MatrMultiply(GT, E_eps);
                    Fint.MatrTMultiply(strainD, TEMP5);
                    Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                            (element->GetLengthZ() / 2.0);
                    HE1.MatrMultiply(GT, TEMP5);
                    HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                           (element->GetLengthZ() / 2.0);
                } else {
                    temp56.MatrMultiply(GT, E_eps);
                    tempC.MatrTMultiply(strainD, E_eps);
                    Fint.MatrMultiply(tempC, strain);
                    Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                            (element->GetLengthZ() / 2.0);
                    HE1.MatrMultiply(temp56, strain);
                    HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                           (element->GetLengthZ() / 2.0);
                }  // end of   if(*flag_Mooney==1)

                KALPHA.MatrMultiply(temp56, G);
                KALPHA *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                          (element->GetLengthZ() / 2.0);
                GDEPSP.MatrMultiply(temp56, strainD);
                GDEPSP *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                          (element->GetLengthZ() / 2.0);

                result.Reset();

                ChMatrixNM<double, 216, 1> GDEPSPVec;
                ChMatrixNM<double, 81, 1> KALPHAVec;
                GDEPSPVec = GDEPSP;
                KALPHAVec = KALPHA;
                result.PasteClippedMatrix(&Fint, 0, 0, 24, 1, 0, 0);
                result.PasteClippedMatrix(&HE1, 0, 0, 9, 1, 24, 0);
                result.PasteClippedMatrix(&GDEPSPVec, 0, 0, 216, 1, 33, 0);
                result.PasteClippedMatrix(&KALPHAVec, 0, 0, 81, 1, 249, 0);
            }
        };
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        ChMatrixNM<double, 330, 1> TempIntegratedResult;
        ChMatrixNM<double, 24, 1> Finternal;

        ChMatrixNM<double, 6, 6> T0;
        ChMatrixNM<double, 9, 1> HE;
        ChMatrixNM<double, 9, 24> GDEPSP;
        ChMatrixNM<double, 9, 9> KALPHA;
        ChMatrixNM<double, 9, 9> KALPHA1;
        ChMatrixNM<double, 9, 1> ResidHE;
        double detJ0C;
        ChMatrixNM<double, 9, 1> alpha_eas;
        ChMatrixNM<double, 9, 1> renewed_alpha_eas;
        ChMatrixNM<double, 9, 1> previous_alpha;

		previous_alpha = m_stock_alpha_EAS;
        alpha_eas = previous_alpha;
        ResidHE.Reset();
        int flag_M = 1;  // 0 means use linear material; 1 means use nonlinear Mooney_Rivlin material
        int count = 0;
        int fail = 1;
        /// Begin EAS loop
        while (fail == 1) {
            /// Update alpha EAS
            alpha_eas = alpha_eas - ResidHE;
            renewed_alpha_eas = alpha_eas;

            Finternal.Reset();
            HE.Reset();
            GDEPSP.Reset();
            KALPHA.Reset();

            // Enhanced Assumed Strain (EAS)
            T0.Reset();
            detJ0C = 0.0;
            T0DetJElementCenterForEAS(d0, T0, detJ0C);
            //== F_internal ==//
            MyForce myformula;
            myformula.d = &d;
            myformula.d0 = &d0;

            if (flag_M == 0) {
                myformula.E = &E;
                myformula.v = &v;
            }

            myformula.element = this;
            // EAS
            myformula.T0 = &T0;
            myformula.detJ0C = &detJ0C;
            myformula.alpha_eas = &alpha_eas;
            ChQuadrature::Integrate3D<ChMatrixNM<double, 330, 1> >(
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
            ChMatrixNM<double, 216, 1> GDEPSPvec;
            ChMatrixNM<double, 81, 1> KALPHAvec;
            Finternal.PasteClippedMatrix(&TempIntegratedResult, 0, 0, 24, 1, 0, 0);    //
            HE.PasteClippedMatrix(&TempIntegratedResult, 24, 0, 9, 1, 0, 0);           //
            GDEPSPvec.PasteClippedMatrix(&TempIntegratedResult, 33, 0, 216, 1, 0, 0);  //
            KALPHAvec.PasteClippedMatrix(&TempIntegratedResult, 249, 0, 81, 1, 0, 0);  //
            GDEPSP = GDEPSPvec;
            KALPHA = KALPHAvec;
            KALPHA1 = KALPHA;

			if (m_flag_HE == 1)
                break;  // When numerical jacobian loop, no need to calculate HE
            count = count + 1;
            double norm_HE = HE.NormTwo();

            if (norm_HE < 0.00001) {
                fail = 0;
            } else {
                ChMatrixNM<int, 9, 1> INDX;
                bool pivoting;
                ResidHE = HE;
                if (!LU_factor(KALPHA1, INDX, pivoting)) {
                    throw ChException("Singular matrix.");
                }
                LU_solve(KALPHA1, INDX, ResidHE);
            }
			if (m_flag_HE == 0 && count > 2) {
                GetLog() << i << "  count " << count << "  NormHE " << norm_HE << "\n";
            }
        }
        Fi = -Finternal;
        //== Stock_Alpha=================//
		if (m_flag_HE == 0) {
            SetStockAlpha(renewed_alpha_eas(0, 0), renewed_alpha_eas(1, 0), renewed_alpha_eas(2, 0),
                          renewed_alpha_eas(3, 0), renewed_alpha_eas(4, 0), renewed_alpha_eas(5, 0),
                          renewed_alpha_eas(6, 0), renewed_alpha_eas(7, 0), renewed_alpha_eas(8, 0));  // this->
        }
        //== Jacobian Matrix for alpha ==//
		if (m_flag_HE == 0) {
            ChMatrixNM<double, 9, 9> INV_KALPHA;
            ChMatrixNM<double, 9, 24> TEMP_GDEPSP;
            ChMatrixNM<double, 9, 9> INV_KALPHA_Temp;
            ChMatrixNM<double, 24, 24> stock_jac_EAS_elem;

            for (int ii = 0; ii < 9; ii++) {
                INV_KALPHA_Temp = KALPHA;
                ChMatrixNM<int, 9, 1> INDX;
                ChMatrixNM<double, 9, 1> DAMMY_vec;
                DAMMY_vec.Reset();
                DAMMY_vec(ii) = 1.0;
                bool pivoting;
                if (!LU_factor(INV_KALPHA_Temp, INDX, pivoting)) {
                    throw ChException("Singular matrix.");
                }
                LU_solve(INV_KALPHA_Temp, INDX, DAMMY_vec);
                INV_KALPHA.PasteClippedMatrix(&DAMMY_vec, 0, 0, 9, 1, 0, ii);  //
            }
            TEMP_GDEPSP.MatrMultiply(INV_KALPHA, GDEPSP);
            stock_jac_EAS_elem.MatrTMultiply(GDEPSP, TEMP_GDEPSP);
            SetStockJac(stock_jac_EAS_elem);
        }
    } else {
        /// Internal force, EAS stiffness, and analytical jacobian are calculated
        class MyForce : public ChIntegrable3D<ChMatrixNM<double, 906, 1> > {
          public:
            ChElementBrick* element;
            ChMatrixNM<double, 8, 3>* d;  // this is an external matrix, use pointer
            ChMatrixNM<double, 8, 3>* d0;
            ChMatrixNM<double, 6, 6>* T0;
            ChMatrixNM<double, 9, 1>* alpha_eas;
            double* detJ0C;
            double* E;
            double* v;

            ChMatrixNM<double, 24, 1> Fint;
            ChMatrixNM<double, 24, 24> JAC11;
            ChMatrixNM<double, 9, 24> Gd;
            ChMatrixNM<double, 6, 1> stress;
            ChMatrixNM<double, 9, 9> Sigm;
            ChMatrixNM<double, 24, 6> temp246;
            ChMatrixNM<double, 24, 9> temp249;
            ChMatrixNM<double, 6, 6> E_eps;
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
            ChMatrixNM<double, 1, 3> Nxd;
            ChMatrixNM<double, 1, 3> Nyd;
            ChMatrixNM<double, 1, 3> Nzd;
            ChMatrixNM<double, 1, 1> tempA;
            ChMatrixNM<double, 1, 24> tempB;
            ChMatrixNM<double, 24, 6> tempC;
            ChMatrixNM<double, 1, 1> tempA1;  // for strain incase of initial curved
            ChMatrixNM<double, 8, 8> d0_d0;   // for strain incase of initial curved
            ChMatrixNM<double, 8, 1> d0d0Nx;  // for strain incase of initial curved
            ChMatrixNM<double, 8, 1> d0d0Ny;  // for strain incase of initial curved
            ChMatrixNM<double, 8, 1> d0d0Nz;  // for strain incase of initial curved
            double detJ0;
            //	//EAS
            ChMatrixNM<double, 6, 9> M;
            ChMatrixNM<double, 6, 9> G;
            ChMatrixNM<double, 9, 6> GT;
            ChMatrixNM<double, 6, 1> strain_EAS;

            //	/// Evaluate (strainD'*strain)  at point
            virtual void Evaluate(ChMatrixNM<double, 906, 1>& result, const double x, const double y, const double z) {
                element->ShapeFunctionsDerivativeX(Nx, x, y, z);
                element->ShapeFunctionsDerivativeY(Ny, x, y, z);
                element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

                element->Basis_M(M, x, y, z);  // EAS

                int flag_Mooney = 1;  // 0 means use linear material; 1 means use nonlinear Mooney_Rivlin material

                if (flag_Mooney == 0) {  // 0 means use linear material
                    double DD = (*E) * (1.0 - (*v)) / ((1.0 + (*v)) * (1.0 - 2.0 * (*v)));
                    E_eps.FillDiag(1.0);
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
                //		// Sd=[Nd1*eye(3) Nd2*eye(3) Nd3*eye(3) Nd4*eye(3)]
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

                //		//==EAS and Initial Shape==//
                ChMatrixNM<double, 3, 3> rd0;
                ChMatrixNM<double, 3, 3> temp33;
                temp33.Reset();
                temp33 = (Nx * (*d0));
                temp33.MatrTranspose();
                rd0.PasteClippedMatrix(&temp33, 0, 0, 3, 1, 0, 0);
                temp33 = (Ny * (*d0));
                temp33.MatrTranspose();
                rd0.PasteClippedMatrix(&temp33, 0, 0, 3, 1, 0, 1);
                temp33 = (Nz * (*d0));
                temp33.MatrTranspose();
                rd0.PasteClippedMatrix(&temp33, 0, 0, 3, 1, 0, 2);
                detJ0 = rd0.Det();

                //		//////////////////////////////////////////////////////////////
                //		//// Transformation : Orthogonal transformation (A and J) ////
                //		//////////////////////////////////////////////////////////////
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

                //		////Tangent Frame
                ChVector<double> A1;
                ChVector<double> A2;
                ChVector<double> A3;
                A1 = G1 / sqrt(G1(0) * G1(0) + G1(1) * G1(1) + G1(2) * G1(2));
                A3 = G1xG2 / sqrt(G1xG2(0) * G1xG2(0) + G1xG2(1) * G1xG2(1) + G1xG2(2) * G1xG2(2));
                A2.Cross(A3, A1);
                ////Direction for orthotropic material//
                double theta = 0.0;
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
                //		//////////////////////////////////////////////////
                //		//// Enhanced Assumed Strain /////////////////////
                //		//////////////////////////////////////////////////
                G = (*T0) * M * ((*detJ0C) / (detJ0));
                strain_EAS = G * (*alpha_eas);
                //		//////////////////////////////////////////////////

                d_d.MatrMultiplyT(*d, *d);
                ddNx.MatrMultiplyT(d_d, Nx);
                ddNy.MatrMultiplyT(d_d, Ny);
                ddNz.MatrMultiplyT(d_d, Nz);

                d0_d0.MatrMultiplyT(*d0, *d0);
                d0d0Nx.MatrMultiplyT(d0_d0, Nx);
                d0d0Ny.MatrMultiplyT(d0_d0, Ny);
                d0d0Nz.MatrMultiplyT(d0_d0, Nz);
                //		///////////////////////////
                //		/// Strain component //////
                //		///////////////////////////
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
                tempA = Nz * ddNz;
                tempA1 = Nz * d0d0Nz;
                strain_til(3, 0) = 0.5 * (tempA(0, 0) - tempA1(0, 0));
                tempA = Nx * ddNz;
                tempA1 = Nx * d0d0Nz;
                strain_til(4, 0) = tempA(0, 0) - tempA1(0, 0);
                tempA = Ny * ddNz;
                tempA1 = Ny * d0d0Nz;
                strain_til(5, 0) = tempA(0, 0) - tempA1(0, 0);
                //		//// For orthotropic material ///
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

                //		////////////////////////////////////
                //		/// Straint derivative component ///
                //		////////////////////////////////////
                ChMatrixNM<double, 6, 24> strainD_til;
                strainD_til.Reset();
                tempB = Nx * (*d) * Sx;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 0, 0);
                tempB = Ny * (*d) * Sy;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 1, 0);
                tempB = Nx * (*d) * Sy + Ny * (*d) * Sx;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 2, 0);
                //== Compatible strain (No ANS)==//
                tempB = Nz * (*d) * Sz;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 3, 0);
                tempB = Nx * (*d) * Sz + Nz * (*d) * Sx;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 4, 0);
                tempB = Ny * (*d) * Sz + Nz * (*d) * Sy;
                strainD_til.PasteClippedMatrix(&tempB, 0, 0, 1, 24, 5, 0);
                //		//// For orthotropic material ///
                for (int ii = 0; ii < 24; ii++) {
                    strainD(0, ii) = strainD_til(0, ii) * beta(0) * beta(0) + strainD_til(1, ii) * beta(3) * beta(3) +
                                     strainD_til(2, ii) * beta(0) * beta(3) + strainD_til(3, ii) * beta(6) * beta(6) +
                                     strainD_til(4, ii) * beta(0) * beta(6) + strainD_til(5, ii) * beta(3) * beta(6);
                    strainD(1, ii) = strainD_til(0, ii) * beta(1) * beta(1) + strainD_til(1, ii) * beta(4) * beta(4) +
                                     strainD_til(2, ii) * beta(1) * beta(4) + strainD_til(3, ii) * beta(7) * beta(7) +
                                     strainD_til(4, ii) * beta(1) * beta(7) + strainD_til(5, ii) * beta(4) * beta(7);
                    strainD(2, ii) = strainD_til(0, ii) * 2.0 * beta(0) * beta(1) +
                                     strainD_til(1, ii) * 2.0 * beta(3) * beta(4) +
                                     strainD_til(2, ii) * (beta(1) * beta(3) + beta(0) * beta(4)) +
                                     strainD_til(3, ii) * 2.0 * beta(6) * beta(7) +
                                     strainD_til(4, ii) * (beta(1) * beta(6) + beta(0) * beta(7)) +
                                     strainD_til(5, ii) * (beta(4) * beta(6) + beta(3) * beta(7));
                    strainD(3, ii) = strainD_til(0, ii) * beta(2) * beta(2) + strainD_til(1, ii) * beta(5) * beta(5) +
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
                //		/// Gd (8x24) calculation
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
                //		///////////////////////////////////
                //		/// Enhanced Assumed Strain 2nd ///
                //		///////////////////////////////////
                strain += strain_EAS;

                ChMatrixNM<double, 9, 6> temp56;
                ChMatrixNM<double, 9, 1> HE1;
                ChMatrixNM<double, 9, 24> GDEPSP;
                ChMatrixNM<double, 9, 9> KALPHA;
                for (int ii = 0; ii < 9; ii++) {
                    for (int jj = 0; jj < 6; jj++) {
                        GT(ii, jj) = G(jj, ii);
                    }
                }
                if (flag_Mooney == 1) {
                    ChMatrixNM<double, 3, 3> CG;  // CG: Right Cauchy-Green tensor  C=trans(F)*F
                    ChMatrixNM<double, 3, 3> INVCG;
                    ChMatrixNM<double, 3, 3> IMAT;
                    ChMatrixNM<double, 3, 3> I1PC;
                    ChMatrixNM<double, 3, 3> I2PC;
                    ChMatrixNM<double, 3, 3> JPC;
                    ChMatrixNM<double, 3, 3> STR;

                    ChMatrixNM<double, 3, 3> CGN;
                    ChMatrixNM<double, 3, 3> INVCGN;
                    ChMatrixNM<double, 3, 3> I1PCN;
                    ChMatrixNM<double, 3, 3> I2PCN;
                    ChMatrixNM<double, 3, 3> JPCN;
                    ChMatrixNM<double, 3, 3> STRN;

                    ChMatrixNM<double, 6, 1> strain_1;
                    ChMatrixNM<double, 6, 1> TEMP5;
                    ChMatrixNM<double, 6, 1> TEMP5N;
                    TEMP5.Reset();
                    TEMP5N.Reset();

                    CG(0, 0) = 2.0 * strain(0, 0) + 1.0;
                    CG(1, 1) = 2.0 * strain(1, 0) + 1.0;
                    CG(2, 2) = 2.0 * strain(3, 0) + 1.0;
                    CG(1, 0) = strain(2, 0);
                    CG(0, 1) = CG(1, 0);
                    CG(2, 0) = strain(4, 0);
                    CG(0, 2) = CG(2, 0);
                    CG(2, 1) = strain(5, 0);
                    CG(1, 2) = CG(2, 1);

                    INVCG = CG;
                    INVCG.MatrInverse();

                    // GetLog() << " INVCG"<<INVCG<<"\n";

                    double Deld = 0.000001;
                    double I1 = CG(0, 0) + CG(1, 1) + CG(2, 2);
                    double I2 = 0.5 * (pow(I1, 2) - (pow(CG(0, 0), 2) + pow(CG(1, 0), 2) + pow(CG(2, 0), 2) +
                                                     pow(CG(0, 1), 2) + pow(CG(1, 1), 2) + pow(CG(2, 1), 2) +
                                                     pow(CG(0, 2), 2) + pow(CG(1, 2), 2) + pow(CG(2, 2), 2)));
                    double I3 = CG(0, 0) * CG(1, 1) * CG(2, 2) - CG(0, 0) * CG(1, 2) * CG(2, 1) +
                                CG(0, 1) * CG(1, 2) * CG(2, 0) - CG(0, 1) * CG(1, 0) * CG(2, 2) +
                                CG(0, 2) * CG(1, 0) * CG(2, 1) - CG(2, 0) * CG(1, 1) * CG(0, 2);
                    double I1BAR = I1 / (pow(I3, 1.0 / 3.0));
                    double I2BAR = I2 / (pow(I3, 2.0 / 3.0));
                    double J = sqrt(I3);
                    double CCOM1 = 551584.0;                                    // C10   not 0.551584
                    double CCOM2 = 137896.0;                                    // C01   not 0.137896
                    double CCOM3 = 2.0 * (CCOM1 + CCOM2) / (1.0 - 2.0 * 0.49);  // K:bulk modulus
                    double StockEPS;

                    IMAT.Reset();
                    IMAT(0, 0) = 1.0;
                    IMAT(1, 1) = 1.0;
                    IMAT(2, 2) = 1.0;
                    I1PC = (IMAT - INVCG * (1.0 / 3.0 * I1)) * pow(I3, -1.0 / 3.0);
                    I2PC = (((IMAT * I1) - CG) - (INVCG * (2.0 / 3.0) * I2)) * pow(I3, -2.0 / 3.0);
                    JPC = INVCG * (J / 2.0);
                    STR = I1PC * (CCOM1 * 2.0) + I2PC * (CCOM2 * 2.0) + JPC * (CCOM3 * (J - 1.0) * 2.0);

                    TEMP5(0, 0) = STR(0, 0);
                    TEMP5(1, 0) = STR(1, 1);
                    TEMP5(2, 0) = STR(0, 1);
                    TEMP5(3, 0) = STR(2, 2);
                    TEMP5(4, 0) = STR(0, 2);
                    TEMP5(5, 0) = STR(1, 2);

                    E_eps.Reset();

                    strain_1 = strain;
                    for (int JJJ = 0; JJJ < 6; JJJ++) {
                        StockEPS = strain_1(JJJ, 0);
                        strain_1(JJJ, 0) = StockEPS + Deld;
                        CGN(0, 0) = 2.0 * strain_1(0, 0) + 1.0;
                        CGN(1, 1) = 2.0 * strain_1(1, 0) + 1.0;
                        CGN(2, 2) = 2.0 * strain_1(3, 0) + 1.0;
                        CGN(1, 0) = strain_1(2, 0);
                        CGN(0, 1) = CGN(1, 0);
                        CGN(2, 0) = strain_1(4, 0);
                        CGN(0, 2) = CGN(2, 0);
                        CGN(2, 1) = strain_1(5, 0);
                        CGN(1, 2) = CGN(2, 1);
                        INVCGN = CGN;
                        INVCGN.MatrInverse();
                        // GetLog() << " INVCGN"<<INVCGN<<"\n";

                        I1 = CGN(0, 0) + CGN(1, 1) + CGN(2, 2);
                        I2 = 0.5 * (pow(I1, 2) - (pow(CGN(0, 0), 2) + pow(CGN(1, 0), 2) + pow(CGN(2, 0), 2) +
                                                  pow(CGN(0, 1), 2) + pow(CGN(1, 1), 2) + pow(CGN(2, 1), 2) +
                                                  pow(CGN(0, 2), 2) + pow(CGN(1, 2), 2) + pow(CGN(2, 2), 2)));
                        I3 = CGN(0, 0) * CGN(1, 1) * CGN(2, 2) - CGN(0, 0) * CGN(1, 2) * CGN(2, 1) +
                             CGN(0, 1) * CGN(1, 2) * CGN(2, 0) - CGN(0, 1) * CGN(1, 0) * CGN(2, 2) +
                             CGN(0, 2) * CGN(1, 0) * CGN(2, 1) - CGN(2, 0) * CGN(1, 1) * CGN(0, 2);
                        J = sqrt(I3);
                        I1BAR = I1 / (pow(I3, 1.0 / 3.0));
                        I2BAR = I2 / (pow(I3, 2.0 / 3.0));
                        I1PCN = (IMAT - INVCGN * (1.0 / 3.0 * I1)) * pow(I3, -1.0 / 3.0);
                        I2PCN = (((IMAT * I1) - CGN) - (INVCGN * (2.0 / 3.0) * I2)) * pow(I3, -2.0 / 3.0);
                        JPCN = INVCGN * (J / 2.0);
                        STRN = I1PCN * (CCOM1 * 2.0) + I2PCN * (CCOM2 * 2.0) + JPCN * (CCOM3 * (J - 1.0) * 2.0);
                        TEMP5N(0, 0) = STRN(0, 0);
                        TEMP5N(1, 0) = STRN(1, 1);
                        TEMP5N(2, 0) = STRN(0, 1);
                        TEMP5N(3, 0) = STRN(2, 2);
                        TEMP5N(4, 0) = STRN(0, 2);
                        TEMP5N(5, 0) = STRN(1, 2);
                        strain_1(JJJ, 0) = StockEPS;
                        E_eps(JJJ, 0) = (TEMP5N(0, 0) - TEMP5(0, 0)) / Deld;
                        E_eps(JJJ, 1) = (TEMP5N(1, 0) - TEMP5(1, 0)) / Deld;
                        E_eps(JJJ, 2) = (TEMP5N(2, 0) - TEMP5(2, 0)) / Deld;
                        E_eps(JJJ, 3) = (TEMP5N(3, 0) - TEMP5(3, 0)) / Deld;
                        E_eps(JJJ, 4) = (TEMP5N(4, 0) - TEMP5(4, 0)) / Deld;
                        E_eps(JJJ, 5) = (TEMP5N(5, 0) - TEMP5(5, 0)) / Deld;
                        // for(int i=0;i<6;i++){
                        //	GetLog() <<TEMP5N(i)<<"\n";}
                        // system("pause");
                        // for(int i=0;i<6;i++){
                        //	GetLog() <<TEMP5(i)<<"\n";}
                        // system("pause");
                    }
                    temp56.MatrMultiply(GT, E_eps);
                    Fint.MatrTMultiply(strainD, TEMP5);
                    Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                            (element->GetLengthZ() / 2.0);
                    HE1.MatrMultiply(GT, TEMP5);
                    HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                           (element->GetLengthZ() / 2.0);
                    Sigm(0, 0) = TEMP5(0, 0);
                    Sigm(1, 1) = TEMP5(0, 0);
                    Sigm(2, 2) = TEMP5(0, 0);
                    Sigm(0, 3) = TEMP5(2, 0);
                    Sigm(1, 4) = TEMP5(2, 0);
                    Sigm(2, 5) = TEMP5(2, 0);
                    Sigm(0, 6) = TEMP5(4, 0);
                    Sigm(1, 7) = TEMP5(4, 0);
                    Sigm(2, 8) = TEMP5(4, 0);
                    Sigm(3, 0) = TEMP5(2, 0);
                    Sigm(4, 1) = TEMP5(2, 0);
                    Sigm(5, 2) = TEMP5(2, 0);
                    Sigm(3, 3) = TEMP5(1, 0);
                    Sigm(4, 4) = TEMP5(1, 0);
                    Sigm(5, 5) = TEMP5(1, 0);
                    Sigm(3, 6) = TEMP5(5, 0);
                    Sigm(4, 7) = TEMP5(5, 0);
                    Sigm(5, 8) = TEMP5(5, 0);
                    Sigm(6, 0) = TEMP5(4, 0);
                    Sigm(7, 1) = TEMP5(4, 0);
                    Sigm(8, 2) = TEMP5(4, 0);
                    Sigm(6, 3) = TEMP5(5, 0);
                    Sigm(7, 4) = TEMP5(5, 0);
                    Sigm(8, 5) = TEMP5(5, 0);
                    Sigm(6, 6) = TEMP5(3, 0);
                    Sigm(7, 7) = TEMP5(3, 0);
                    Sigm(8, 8) = TEMP5(3, 0);
                } else {
                    /// Stress tensor calculation
                    stress.MatrMultiply(E_eps, strain);
                    Sigm(0, 0) = stress(0, 0);
                    Sigm(0, 3) = stress(2, 0);
                    Sigm(0, 6) = stress(4, 0);
                    Sigm(1, 1) = stress(0, 0);
                    Sigm(1, 4) = stress(2, 0);
                    Sigm(1, 7) = stress(4, 0);
                    Sigm(2, 2) = stress(0, 0);
                    Sigm(2, 5) = stress(2, 0);
                    Sigm(2, 8) = stress(4, 0);
                    // XX                      //XY                     //XZ

                    Sigm(3, 0) = stress(2, 0);
                    Sigm(3, 3) = stress(1, 0);
                    Sigm(3, 6) = stress(5, 0);
                    Sigm(4, 1) = stress(2, 0);
                    Sigm(4, 4) = stress(1, 0);
                    Sigm(4, 7) = stress(5, 0);
                    Sigm(5, 2) = stress(2, 0);
                    Sigm(5, 5) = stress(1, 0);
                    Sigm(5, 8) = stress(5, 0);
                    // XY                     //YY                     //YZ

                    Sigm(6, 0) = stress(4, 0);
                    Sigm(6, 3) = stress(5, 0);
                    Sigm(6, 6) = stress(3, 0);
                    Sigm(7, 1) = stress(4, 0);
                    Sigm(7, 4) = stress(5, 0);
                    Sigm(7, 7) = stress(3, 0);
                    Sigm(8, 2) = stress(4, 0);
                    Sigm(8, 5) = stress(5, 0);
                    Sigm(8, 8) = stress(3, 0);
                    // XZ                     //YZ                     //ZZ

                    temp56.MatrMultiply(GT, E_eps);
                    tempC.MatrTMultiply(strainD, E_eps);
                    Fint.MatrMultiply(tempC, strain);
                    Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                            (element->GetLengthZ() / 2.0);
                    HE1.MatrMultiply(temp56, strain);
                    HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                           (element->GetLengthZ() / 2.0);
                }  // end of   if(*flag_Mooney==1)

                /// Jacobian calculation
                temp246.MatrTMultiply(strainD, E_eps);
                temp249.MatrTMultiply(Gd, Sigm);
                JAC11 = temp246 * strainD + temp249 * Gd;
                JAC11 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                         (element->GetLengthZ() / 2.0);

                GDEPSP.MatrMultiply(temp56, strainD);
                GDEPSP *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                          (element->GetLengthZ() / 2.0);
                KALPHA.MatrMultiply(temp56, G);
                KALPHA *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) *
                          (element->GetLengthZ() / 2.0);

                ChMatrixNM<double, 216, 1> GDEPSPVec;
                ChMatrixNM<double, 81, 1> KALPHAVec;
                ChMatrixNM<double, 576, 1> JACVec;

                result.Reset();
                GDEPSPVec = GDEPSP;
                KALPHAVec = KALPHA;
                JACVec = JAC11;
                result.PasteClippedMatrix(&Fint, 0, 0, 24, 1, 0, 0);
                result.PasteClippedMatrix(&HE1, 0, 0, 9, 1, 24, 0);
                result.PasteClippedMatrix(&GDEPSPVec, 0, 0, 216, 1, 33, 0);
                result.PasteClippedMatrix(&KALPHAVec, 0, 0, 81, 1, 249, 0);
                result.PasteClippedMatrix(&JACVec, 0, 0, 576, 1, 330, 0);
            }
        };  // end of class MyForce
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ChMatrixNM<double, 906, 1> TempIntegratedResult;
        ChMatrixNM<double, 24, 1> Finternal;
        // Enhanced Assumed Strain (EAS)
        ChMatrixNM<double, 6, 6> T0;
        ChMatrixNM<double, 9, 1> HE;
        ChMatrixNM<double, 9, 24> GDEPSP;
        ChMatrixNM<double, 9, 9> KALPHA;
        ChMatrixNM<double, 24, 24> KTE;
        ChMatrixNM<double, 9, 9> KALPHA1;
        ChMatrixNM<double, 9, 1> ResidHE;
        double detJ0C;
        ChMatrixNM<double, 9, 1> alpha_eas;
        ChMatrixNM<double, 9, 1> renewed_alpha_eas;
        ChMatrixNM<double, 9, 1> previous_alpha;

		previous_alpha = m_stock_alpha_EAS;
        alpha_eas = previous_alpha;
        ResidHE.Reset();
        int flag_M = 1;  // 0 means use linear material; 1 means use nonlinear Mooney_Rivlin material
        int count = 0;
        int fail = 1;
        while (fail == 1) {
            alpha_eas = alpha_eas - ResidHE;
            renewed_alpha_eas = alpha_eas;

            Finternal.Reset();
            HE.Reset();
            GDEPSP.Reset();
            KALPHA.Reset();

            // Enhanced Assumed Strain (EAS)
            T0.Reset();
            detJ0C = 0.0;
            T0DetJElementCenterForEAS(d0, T0, detJ0C);
            //== F_internal ==//
            MyForce myformula;
            myformula.d = &d;
            myformula.d0 = &d0;

            if (flag_M == 0) {
                myformula.E = &E;
                myformula.v = &v;
            }

            myformula.element = this;
            //	//EAS
            myformula.T0 = &T0;
            myformula.detJ0C = &detJ0C;
            myformula.alpha_eas = &alpha_eas;
            ChQuadrature::Integrate3D<ChMatrixNM<double, 906, 1> >(
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
            ChMatrixNM<double, 216, 1> GDEPSPvec;
            ChMatrixNM<double, 81, 1> KALPHAvec;
            ChMatrixNM<double, 576, 1> JACvec;
            Finternal.PasteClippedMatrix(&TempIntegratedResult, 0, 0, 24, 1, 0, 0);    //
            HE.PasteClippedMatrix(&TempIntegratedResult, 24, 0, 9, 1, 0, 0);           //
            GDEPSPvec.PasteClippedMatrix(&TempIntegratedResult, 33, 0, 216, 1, 0, 0);  //
            KALPHAvec.PasteClippedMatrix(&TempIntegratedResult, 249, 0, 81, 1, 0, 0);  //
            JACvec.PasteClippedMatrix(&TempIntegratedResult, 330, 0, 576, 1, 0, 0);    //
            GDEPSP = GDEPSPvec;
            KALPHA = KALPHAvec;
            KTE = JACvec;
            KALPHA1 = KALPHA;
            //   GetLog() <<HE<<"\n";
            // system("pause");
            //      GetLog() <<Finternal<<"\n";
            // system("pause");
			if (m_flag_HE == 1)
                break;  // When numerical jacobian loop, no need to calculate HE
            count = count + 1;
            double norm_HE = HE.NormTwo();
            if (norm_HE < 0.00001) {
                fail = 0;
            } else {
                ChMatrixNM<int, 9, 1> INDX;
                ResidHE = HE;
                bool pivoting;
                if (!LU_factor(KALPHA1, INDX, pivoting)) {
                    throw ChException("Singualr matrix.");
                }
                LU_solve(KALPHA1, INDX, ResidHE);
            }
        }  // end of while
        Fi = -Finternal;
        ////===============================//
        ////== Stock_Alpha=================//
        ////===============================//
		if (m_flag_HE == 0) {
            SetStockAlpha(renewed_alpha_eas(0, 0), renewed_alpha_eas(1, 0), renewed_alpha_eas(2, 0),
                          renewed_alpha_eas(3, 0), renewed_alpha_eas(4, 0), renewed_alpha_eas(5, 0),
                          renewed_alpha_eas(6, 0), renewed_alpha_eas(7, 0), renewed_alpha_eas(8, 0));  // this->
        }
        ////===============================//
        ////== Jacobian Matrix for alpha ==//
        ////===============================//
		if (m_flag_HE == 0) {
            ChMatrixNM<double, 9, 9> INV_KALPHA;
            ChMatrixNM<double, 9, 24> TEMP_GDEPSP;
            ChMatrixNM<double, 9, 9> INV_KALPHA_Temp;
            ChMatrixNM<double, 24, 24> stock_jac_EAS_elem;

            for (int ii = 0; ii < 9; ii++) {
                INV_KALPHA_Temp = KALPHA;
                ChMatrixNM<int, 9, 1> INDX;
                ChMatrixNM<double, 9, 1> DAMMY_vec;
                DAMMY_vec.Reset();
                DAMMY_vec(ii) = 1.0;
                bool pivoting;
                if (!LU_factor(INV_KALPHA_Temp, INDX, pivoting)) {
                    throw ChException("Singular matrix.");
                }
                LU_solve(INV_KALPHA_Temp, INDX, DAMMY_vec);
                INV_KALPHA.PasteClippedMatrix(&DAMMY_vec, 0, 0, 9, 1, 0, ii);  //
            }
            TEMP_GDEPSP.MatrMultiply(INV_KALPHA, GDEPSP);
            stock_jac_EAS_elem.MatrTMultiply(GDEPSP, TEMP_GDEPSP);
            SetStockKTE(KTE);
            SetStockJac(stock_jac_EAS_elem);
        }
    }  // end of else for numerical or analytical

    // Add gravity force
    class MyGravity : public ChIntegrable3D<ChMatrixNM<double, 24, 1> > {
      public:
        ChElementBrick* element;
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
            double wx2 = (element->GetLengthX()) / 2;
            double wy2 = (element->GetLengthY()) / 2;
            double wz2 = (element->GetLengthZ()) / 2;

            // Set gravity acceleration
            LocalGravityForce(0, 0) = 0.0;
            LocalGravityForce(1, 0) = 0.0;
            LocalGravityForce(2, 0) = -9.81;

            // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)...]
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

			result *= detJ0 * wx2 * wy2 * wz2 * (element->m_Material->Get_density());
        }
    };

    MyGravity myformula1;
    myformula1.d0 = &d0;
    myformula1.element = this;

    ChMatrixNM<double, 24, 1> Fgravity;
    ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 1> >(Fgravity,    // result of integration will go there
                                                          myformula1,  // formula to integrate
                                                          -1,          // start of x
                                                          1,           // end of x
                                                          -1,          // start of y
                                                          1,           // end of y
                                                          -1,          // start of z
                                                          1,           // end of z
                                                          2            // order of integration
                                                          );
    Fi += Fgravity;
    ////check gravity force
    // for (i=0;i<24;i++){
    //	GetLog()<<Fgravity(i)<<"\n";
    //}
    // system("pause");
}  // end of ComputeInternalForces

// -----------------------------------------------------------------------------

void ChElementBrick::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
    N(0) = 0.125 * (1.0 - x) * (1.0 - y) * (1.0 - z);
    N(1) = 0.125 * (1.0 + x) * (1.0 - y) * (1.0 - z);
    N(2) = 0.125 * (1.0 - x) * (1.0 + y) * (1.0 - z);
    N(3) = 0.125 * (1.0 + x) * (1.0 + y) * (1.0 - z);
    N(4) = 0.125 * (1.0 - x) * (1.0 - y) * (1.0 + z);
    N(5) = 0.125 * (1.0 + x) * (1.0 - y) * (1.0 + z);
    N(6) = 0.125 * (1.0 - x) * (1.0 + y) * (1.0 + z);
    N(7) = 0.125 * (1.0 + x) * (1.0 + y) * (1.0 + z);
}

// -----------------------------------------------------------------------------

void ChElementBrick::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
    double a = GetLengthX();

    Nx(0) = 2.0 / a * 0.125 * (-1.0) * (1.0 - y) * (1.0 - z);
    Nx(1) = 2.0 / a * 0.125 * (1.0) * (1.0 - y) * (1.0 - z);
    Nx(2) = 2.0 / a * 0.125 * (-1.0) * (1.0 + y) * (1.0 - z);
    Nx(3) = 2.0 / a * 0.125 * (1.0) * (1.0 + y) * (1.0 - z);
    Nx(4) = 2.0 / a * 0.125 * (-1.0) * (1.0 - y) * (1.0 + z);
    Nx(5) = 2.0 / a * 0.125 * (1.0) * (1.0 - y) * (1.0 + z);
    Nx(6) = 2.0 / a * 0.125 * (-1.0) * (1.0 + y) * (1.0 + z);
    Nx(7) = 2.0 / a * 0.125 * (1.0) * (1.0 + y) * (1.0 + z);
}

// -----------------------------------------------------------------------------

void ChElementBrick::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
    double b = GetLengthY();

    Ny(0) = 2.0 / b * 0.125 * (1.0 - x) * (-1.0) * (1.0 - z);
    Ny(1) = 2.0 / b * 0.125 * (1.0 + x) * (-1.0) * (1.0 - z);
    Ny(2) = 2.0 / b * 0.125 * (1.0 - x) * (1.0) * (1.0 - z);
    Ny(3) = 2.0 / b * 0.125 * (1.0 + x) * (1.0) * (1.0 - z);
    Ny(4) = 2.0 / b * 0.125 * (1.0 - x) * (-1.0) * (1.0 + z);
    Ny(5) = 2.0 / b * 0.125 * (1.0 + x) * (-1.0) * (1.0 + z);
    Ny(6) = 2.0 / b * 0.125 * (1.0 - x) * (1.0) * (1.0 + z);
    Ny(7) = 2.0 / b * 0.125 * (1.0 + x) * (1.0) * (1.0 + z);
}

// -----------------------------------------------------------------------------

void ChElementBrick::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
    double c = GetLengthZ();

    Nz(0) = 2.0 / c * 0.125 * (1.0 - x) * (1.0 - y) * (-1.0);
    Nz(1) = 2.0 / c * 0.125 * (1.0 + x) * (1.0 - y) * (-1.0);
    Nz(2) = 2.0 / c * 0.125 * (1.0 - x) * (1.0 + y) * (-1.0);
    Nz(3) = 2.0 / c * 0.125 * (1.0 + x) * (1.0 + y) * (-1.0);
    Nz(4) = 2.0 / c * 0.125 * (1.0 - x) * (1.0 - y) * (1.0);
    Nz(5) = 2.0 / c * 0.125 * (1.0 + x) * (1.0 - y) * (1.0);
    Nz(6) = 2.0 / c * 0.125 * (1.0 - x) * (1.0 + y) * (1.0);
    Nz(7) = 2.0 / c * 0.125 * (1.0 + x) * (1.0 + y) * (1.0);
}

// ----------------------------------------------------------------------------
void ChElementBrick::Update() {
    // parent class update:
    ChElementGeneric::Update();
}

// -----------------------------------------------------------------------------

void ChElementBrick::GetStateBlock(ChMatrixDynamic<>& mD) {
	mD.PasteVector(m_nodes[0]->GetPos(), 0, 0);
	mD.PasteVector(m_nodes[1]->GetPos(), 3, 0);
	mD.PasteVector(m_nodes[2]->GetPos(), 6, 0);
	mD.PasteVector(m_nodes[3]->GetPos(), 9, 0);
	mD.PasteVector(m_nodes[4]->GetPos(), 12, 0);
	mD.PasteVector(m_nodes[5]->GetPos(), 15, 0);
	mD.PasteVector(m_nodes[6]->GetPos(), 18, 0);
	mD.PasteVector(m_nodes[7]->GetPos(), 21, 0);
}

// -----------------------------------------------------------------------------

void ChElementBrick::ComputeStiffnessMatrix() {
    bool use_numerical_differentiation = false;

    if (use_numerical_differentiation) {
        double diff = 1e-8;
        ChMatrixDynamic<> Kcolumn(24, 1);
        ChMatrixDynamic<> F0(24, 1);
        ChMatrixDynamic<> F1(24, 1);
		m_flag_HE = 0;  // flag_HE is defineded in  [class  ChElementBrick : public ChElementGeneric]
        ComputeInternalForces(F0);
		m_flag_HE = 1;  // flag_HE is defineded in  [class  ChElementBrick : public ChElementGeneric]
        for (int inode = 0; inode < 8; ++inode) {
			m_nodes[inode]->pos.x += diff;
            ComputeInternalForces(F1);  // Flag=1 > Jacobian of internal force calculation
            Kcolumn = (F0 - F1) * (1.0 / diff);
			m_StiffnessMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 0 + inode * 3);
			m_nodes[inode]->pos.x -= diff;

			m_nodes[inode]->pos.y += diff;
            ComputeInternalForces(F1);
            Kcolumn = (F0 - F1) * (1.0 / diff);
			m_StiffnessMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 1 + inode * 3);
			m_nodes[inode]->pos.y -= diff;

			m_nodes[inode]->pos.z += diff;
            ComputeInternalForces(F1);
            Kcolumn = (F0 - F1) * (1.0 / diff);
			m_StiffnessMatrix.PasteClippedMatrix(&Kcolumn, 0, 0, 24, 1, 0, 2 + inode * 3);
			m_nodes[inode]->pos.z -= diff;
        }

		m_flag_HE = 0;  // flag_HE=0 is default
		m_StiffnessMatrix -= m_stock_jac_EAS;  // For Enhanced Assumed Strain
    } else {
		m_flag_HE = 0;
		m_StiffnessMatrix = m_stock_KTE;
		m_StiffnessMatrix -= m_stock_jac_EAS;

    }
}

// -----------------------------------------------------------------------------

void ChElementBrick::ComputeMassMatrix() {
	double rho = m_Material->Get_density();
    ChMatrixNM<double, 24, 1> InitialCoord;
    ChMatrixNM<double, 8, 3> d0;
    InitialCoord = GetInitialPos();
    d0(0, 0) = InitialCoord(0, 0);
    d0(0, 1) = InitialCoord(1, 0);
    d0(0, 2) = InitialCoord(2, 0);
    d0(1, 0) = InitialCoord(3, 0);
    d0(1, 1) = InitialCoord(4, 0);
    d0(1, 2) = InitialCoord(5, 0);
    d0(2, 0) = InitialCoord(6, 0);
    d0(2, 1) = InitialCoord(7, 0);
    d0(2, 2) = InitialCoord(8, 0);
    d0(3, 0) = InitialCoord(9, 0);
    d0(3, 1) = InitialCoord(10, 0);
    d0(3, 2) = InitialCoord(11, 0);
    d0(4, 0) = InitialCoord(12, 0);
    d0(4, 1) = InitialCoord(13, 0);
    d0(4, 2) = InitialCoord(14, 0);
    d0(5, 0) = InitialCoord(15, 0);
    d0(5, 1) = InitialCoord(16, 0);
    d0(5, 2) = InitialCoord(17, 0);
    d0(6, 0) = InitialCoord(18, 0);
    d0(6, 1) = InitialCoord(19, 0);
    d0(6, 2) = InitialCoord(20, 0);
    d0(7, 0) = InitialCoord(21, 0);
    d0(7, 1) = InitialCoord(22, 0);
    d0(7, 2) = InitialCoord(23, 0);

    class MyMass : public ChIntegrable3D<ChMatrixNM<double, 24, 24> > {
      public:
        ChElementBrick* element;
        ChMatrixNM<double, 8, 3>* d0;
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

            // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)...]
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

            // perform  r = S'*S
            result.MatrTMultiply(S, S);

            result *= detJ0 * (element->GetLengthX() / 2) * (element->GetLengthY() / 2) * (element->GetLengthZ() / 2);
        }
    };

    MyMass myformula;
    myformula.d0 = &d0;
    myformula.element = this;

	ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 24> >(m_MassMatrix,  // result of integration will go there
                                                           myformula,         // formula to integrate
                                                           -1,                // start of x
                                                           1,                 // end of x
                                                           -1,                // start of y
                                                           1,                 // end of y
                                                           -1,                // start of z
                                                           1,                 // end of z
                                                           2                  // order of integration
                                                           );

	m_MassMatrix *= rho;
}

// -----------------------------------------------------------------------------

void ChElementBrick::SetupInitial() {
    // Compute mass matrix
    ComputeMassMatrix();
    // initial EAS parameters
	m_stock_jac_EAS.Reset();
    // Compute stiffness matrix
    // (this is not constant in ANCF and will be called automatically many times by ComputeKRMmatricesGlobal()
    // when the solver will run, yet maybe nice to privide an initial nonzero value)
    ComputeStiffnessMatrix();
}

// -----------------------------------------------------------------------------

void ChElementBrick::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 24) && (H.GetColumns() == 24));

    // Compute global stiffness matrix:
    ComputeStiffnessMatrix();
    // 1) Store  +kf*[K] +rf*[R]
    // For K stiffness matrix and R matrix: scale by factors
    // because [R] = r*[K] , so kf*[K]+rf*[R] = (kf+rf*r)*[K]
	double kr_factor = Kfactor + Rfactor * m_Material->Get_RayleighDampingK();

	ChMatrixDynamic<> temp(m_StiffnessMatrix);
    temp.MatrScale(kr_factor);

    // Paste scaled K stiffness matrix and R matrix in resulting H:
    H.PasteMatrix(&temp, 0, 0);

    // 2) Store  +mf*[M]
	temp = m_MassMatrix;
    temp.MatrScale(Mfactor);

    // Paste scaled M mass matrix in resulting H:
    H.PasteSumMatrix(&temp, 0, 0);
}

// -----------------------------------------------------------------------------

void ChElementBrick::T0DetJElementCenterForEAS(ChMatrixNM<double, 8, 3>& d0,
                                               ChMatrixNM<double, 6, 6>& T0,
                                               double& detJ0C) {
    double x = 0;
    double y = 0;
    double z = 0;
    ChMatrixNM<double, 1, 8> Nx;
    ChMatrixNM<double, 1, 8> Ny;
    ChMatrixNM<double, 1, 8> Nz;
    ChMatrixNM<double, 3, 3> rd0;
    ChMatrixNM<double, 3, 3> tempA;
    ShapeFunctionsDerivativeX(Nx, x, y, z);
    ShapeFunctionsDerivativeY(Ny, x, y, z);
    ShapeFunctionsDerivativeZ(Nz, x, y, z);
    tempA = (Nx * d0);
    tempA.MatrTranspose();
    rd0.PasteClippedMatrix(&tempA, 0, 0, 3, 1, 0, 0);
    tempA = (Ny * d0);
    tempA.MatrTranspose();
    rd0.PasteClippedMatrix(&tempA, 0, 0, 3, 1, 0, 1);
    tempA = (Nz * d0);
    tempA.MatrTranspose();
    rd0.PasteClippedMatrix(&tempA, 0, 0, 3, 1, 0, 2);
    detJ0C = rd0.Det();
    // Transformation : Orthogonal transformation (A and J) ////
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
    ;
    A1 = G1 / sqrt(G1(0) * G1(0) + G1(1) * G1(1) + G1(2) * G1(2));
    A3 = G1xG2 / sqrt(G1xG2(0) * G1xG2(0) + G1xG2(1) * G1xG2(1) + G1xG2(2) * G1xG2(2));
    A2.Cross(A3, A1);
    double theta = 0.0;
    ChVector<double> AA1;
    ChVector<double> AA2;
    ChVector<double> AA3;
    AA1 = A1 * cos(theta) + A2 * sin(theta);
    AA2 = -A1 * sin(theta) + A2 * cos(theta);
    AA3 = A3;

    // Beta
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
void ChElementBrick::Basis_M(ChMatrixNM<double, 6, 9>& M, double x, double y, double z) {
    M.Reset();
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

}  // end namespace fea
}  // end namespace chrono
