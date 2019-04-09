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
// Brick element with 8 nodes (with EAS)
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChElementBrick.h"
#include "chrono/fea/ChUtilsFEA.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------
ChElementBrick::ChElementBrick() : m_flag_HE(ANALYTICAL), m_gravity_on(false) {
    m_nodes.resize(8);
}
// -----------------------------------------------------------------------------
void ChElementBrick::SetNodes(std::shared_ptr<ChNodeFEAxyz> nodeA,
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
    ChVector<> pA = m_nodes[0]->GetPos();
    ChVector<> pB = m_nodes[1]->GetPos();
    ChVector<> pC = m_nodes[2]->GetPos();
    ChVector<> pD = m_nodes[3]->GetPos();
    ChVector<> pE = m_nodes[4]->GetPos();
    ChVector<> pF = m_nodes[5]->GetPos();
    ChVector<> pG = m_nodes[6]->GetPos();
    ChVector<> pH = m_nodes[7]->GetPos();
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

void ChElementBrick::SetStockAlpha(double a1,
                                   double a2,
                                   double a3,
                                   double a4,
                                   double a5,
                                   double a6,
                                   double a7,
                                   double a8,
                                   double a9) {
    m_stock_alpha_EAS(0, 0) = a1;
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

    double v = m_Material->Get_v();
    double E = m_Material->Get_E();

    Fi.Reset();

    /// If numerical differentiation is used, only the internal force and EAS stiffness
    /// will be calculated. If the numerical differentiation is not used, the jacobian
    /// will also be calculated.
    bool use_numerical_differentiation = false;

    /// Internal force and EAS parameters are calculated for numerical differentiation.
    if (use_numerical_differentiation) {
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
            T0DetJElementCenterForEAS(m_d0, T0, detJ0C);
            //== F_internal ==//
            // Choose constructors depending on m_isMooney
            MyForceNum myformula = !m_isMooney ? MyForceNum(&d, &m_d0, this, &T0, &detJ0C, &alpha_eas, &E, &v)
                                               : MyForceNum(&d, &m_d0, this, &T0, &detJ0C, &alpha_eas);
            TempIntegratedResult.Reset();
            ChQuadrature::Integrate3D<ChMatrixNM<double, 330, 1>>(
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
            Finternal.PasteClippedMatrix(TempIntegratedResult, 0, 0, 24, 1, 0, 0);    //
            HE.PasteClippedMatrix(TempIntegratedResult, 24, 0, 9, 1, 0, 0);           //
            GDEPSPvec.PasteClippedMatrix(TempIntegratedResult, 33, 0, 216, 1, 0, 0);  //
            KALPHAvec.PasteClippedMatrix(TempIntegratedResult, 249, 0, 81, 1, 0, 0);  //
            GDEPSP = GDEPSPvec;
            KALPHA = KALPHAvec;
            KALPHA1 = KALPHA;

            if (m_flag_HE == NUMERICAL)
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
            if (m_flag_HE == ANALYTICAL && count > 2) {
                GetLog() << i << "  count " << count << "  NormHE " << norm_HE << "\n";
            }
        }
        Fi = -Finternal;
        //== Stock_Alpha=================//
        if (m_flag_HE == ANALYTICAL) {
            SetStockAlpha(renewed_alpha_eas(0, 0), renewed_alpha_eas(1, 0), renewed_alpha_eas(2, 0),
                          renewed_alpha_eas(3, 0), renewed_alpha_eas(4, 0), renewed_alpha_eas(5, 0),
                          renewed_alpha_eas(6, 0), renewed_alpha_eas(7, 0), renewed_alpha_eas(8, 0));  // this->
        }
        //== Jacobian Matrix for alpha ==//
        if (m_flag_HE == ANALYTICAL) {
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
                INV_KALPHA.PasteClippedMatrix(DAMMY_vec, 0, 0, 9, 1, 0, ii);  //
            }
            TEMP_GDEPSP.MatrMultiply(INV_KALPHA, GDEPSP);
            stock_jac_EAS_elem.MatrTMultiply(GDEPSP, TEMP_GDEPSP);
            SetStockJac(stock_jac_EAS_elem);
        }
    } else {
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

            Finternal.Reset();  // Internal force vector
            HE.Reset();         // Internal force vector from EAS
            GDEPSP.Reset();     // Jacobian of EAS forces w.r.t. coordinates
            KALPHA.Reset();     // Jacobian of EAS forces w.r.t. EAS internal parameters

            // Enhanced Assumed Strain (EAS)
            T0.Reset();
            detJ0C = 0.0;
            T0DetJElementCenterForEAS(m_d0, T0, detJ0C);

            //== F_internal ==//
            MyForceAnalytical myformula = !m_isMooney
                                              ? MyForceAnalytical(&d, &m_d0, this, &T0, &detJ0C, &alpha_eas, &E, &v)
                                              : MyForceAnalytical(&d, &m_d0, this, &T0, &detJ0C, &alpha_eas);
            TempIntegratedResult.Reset();
            ChQuadrature::Integrate3D<ChMatrixNM<double, 906, 1>>(
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
            Finternal.PasteClippedMatrix(TempIntegratedResult, 0, 0, 24, 1, 0, 0);    //
            HE.PasteClippedMatrix(TempIntegratedResult, 24, 0, 9, 1, 0, 0);           //
            GDEPSPvec.PasteClippedMatrix(TempIntegratedResult, 33, 0, 216, 1, 0, 0);  //
            KALPHAvec.PasteClippedMatrix(TempIntegratedResult, 249, 0, 81, 1, 0, 0);  //
            JACvec.PasteClippedMatrix(TempIntegratedResult, 330, 0, 576, 1, 0, 0);    //
            for (int i = 0; i < 9; i++) {
                for (int j = 0; j < 24; j++) {
                    GDEPSP(i, j) = GDEPSPvec(i * 24 + j, 0);
                }
            }
            // GDEPSP = GDEPSPvec;
            for (int i = 0; i < 9; i++) {
                for (int j = 0; j < 9; j++) {
                    KALPHA(i, j) = KALPHAvec(i * 9 + j, 0);
                }
            }
            // KALPHA = KALPHAvec;
            for (int i = 0; i < 24; i++) {
                for (int j = 0; j < 24; j++) {
                    KTE(i, j) = JACvec(i * 24 + j, 0);
                }
            }
            // KTE = JACvec;

            // Calculation of the element Jacobian for implicit integrator
            // KTE and stock_jac_EAS_elem.
            KALPHA1 = KALPHA;
            if (m_flag_HE == NUMERICAL)
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
                    throw ChException("Singular matrix.");
                }
                LU_solve(KALPHA1, INDX, ResidHE);
            }
        }  // end of while
        Fi = -Finternal;
        ////== Stock_Alpha=================//
        if (m_flag_HE == ANALYTICAL) {
            SetStockAlpha(renewed_alpha_eas(0, 0), renewed_alpha_eas(1, 0), renewed_alpha_eas(2, 0),
                          renewed_alpha_eas(3, 0), renewed_alpha_eas(4, 0), renewed_alpha_eas(5, 0),
                          renewed_alpha_eas(6, 0), renewed_alpha_eas(7, 0), renewed_alpha_eas(8, 0));  // this->
        }
        ////== Jacobian Matrix for alpha ==//
        if (m_flag_HE == ANALYTICAL) {
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
                INV_KALPHA.PasteClippedMatrix(DAMMY_vec, 0, 0, 9, 1, 0, ii);  //
            }
            TEMP_GDEPSP.MatrMultiply(INV_KALPHA, GDEPSP);
            stock_jac_EAS_elem.MatrTMultiply(GDEPSP, TEMP_GDEPSP);
            SetStockKTE(KTE);
            SetStockJac(stock_jac_EAS_elem);
        }
    }  // end of else for numerical or analytical

    if (m_gravity_on) {
        Fi += m_GravForce;
    }
}

// -----------------------------------------------------------------------------

void ChElementBrick::ShapeFunctions(ChMatrix<>& N, double x, double y, double z) {
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

void ChElementBrick::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y, double z) {
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

void ChElementBrick::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y, double z) {
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

void ChElementBrick::ShapeFunctionsDerivativeZ(ChMatrix<>& Nz, double x, double y, double z) {
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
        ComputeInternalForces(F0);
        for (int inode = 0; inode < 8; ++inode) {
            m_nodes[inode]->pos.x() += diff;
            ComputeInternalForces(F1);  // Flag=1 > Jacobian of internal force calculation
            Kcolumn = (F0 - F1) * (1.0 / diff);
            m_StiffnessMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 24, 1, 0, 0 + inode * 3);
            m_nodes[inode]->pos.x() -= diff;

            m_nodes[inode]->pos.y() += diff;
            ComputeInternalForces(F1);
            Kcolumn = (F0 - F1) * (1.0 / diff);
            m_StiffnessMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 24, 1, 0, 1 + inode * 3);
            m_nodes[inode]->pos.y() -= diff;

            m_nodes[inode]->pos.z() += diff;
            ComputeInternalForces(F1);
            Kcolumn = (F0 - F1) * (1.0 / diff);
            m_StiffnessMatrix.PasteClippedMatrix(Kcolumn, 0, 0, 24, 1, 0, 2 + inode * 3);
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
void ChElementBrick::MyForceAnalytical::Evaluate(ChMatrixNM<double, 906, 1>& result,
                                                 const double x,
                                                 const double y,
                                                 const double z) {
    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

    element->Basis_M(M, x, y, z);  // EAS

    if (!element->m_isMooney) {  // m_isMooney == false means use linear material
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
    Sxi.Reset();

    Sxi.FillDiag(Nx(0));
    Sx.PasteMatrix(Sxi, 0, 0);
    Sxi.FillDiag(Nx(1));
    Sx.PasteMatrix(Sxi, 0, 3);
    Sxi.FillDiag(Nx(2));
    Sx.PasteMatrix(Sxi, 0, 6);
    Sxi.FillDiag(Nx(3));
    Sx.PasteMatrix(Sxi, 0, 9);
    Sxi.FillDiag(Nx(4));
    Sx.PasteMatrix(Sxi, 0, 12);
    Sxi.FillDiag(Nx(5));
    Sx.PasteMatrix(Sxi, 0, 15);
    Sxi.FillDiag(Nx(6));
    Sx.PasteMatrix(Sxi, 0, 18);
    Sxi.FillDiag(Nx(7));
    Sx.PasteMatrix(Sxi, 0, 21);

    ChMatrix33<> Syi;
    Syi.Reset();

    Syi.FillDiag(Ny(0));
    Sy.PasteMatrix(Syi, 0, 0);
    Syi.FillDiag(Ny(1));
    Sy.PasteMatrix(Syi, 0, 3);
    Syi.FillDiag(Ny(2));
    Sy.PasteMatrix(Syi, 0, 6);
    Syi.FillDiag(Ny(3));
    Sy.PasteMatrix(Syi, 0, 9);
    Syi.FillDiag(Ny(4));
    Sy.PasteMatrix(Syi, 0, 12);
    Syi.FillDiag(Ny(5));
    Sy.PasteMatrix(Syi, 0, 15);
    Syi.FillDiag(Ny(6));
    Sy.PasteMatrix(Syi, 0, 18);
    Syi.FillDiag(Ny(7));
    Sy.PasteMatrix(Syi, 0, 21);

    ChMatrix33<> Szi;
    Szi.Reset();

    Szi.FillDiag(Nz(0));
    Sz.PasteMatrix(Szi, 0, 0);
    Szi.FillDiag(Nz(1));
    Sz.PasteMatrix(Szi, 0, 3);
    Szi.FillDiag(Nz(2));
    Sz.PasteMatrix(Szi, 0, 6);
    Szi.FillDiag(Nz(3));
    Sz.PasteMatrix(Szi, 0, 9);
    Szi.FillDiag(Nz(4));
    Sz.PasteMatrix(Szi, 0, 12);
    Szi.FillDiag(Nz(5));
    Sz.PasteMatrix(Szi, 0, 15);
    Szi.FillDiag(Nz(6));
    Sz.PasteMatrix(Szi, 0, 18);
    Szi.FillDiag(Nz(7));
    Sz.PasteMatrix(Szi, 0, 21);

    // EAS and Initial Shape
    ChMatrixNM<double, 3, 3> rd0;
    ChMatrixNM<double, 1, 3> temp13;

    temp13.Reset();

    temp13 = (Nx * (*d0));
    temp13.MatrTranspose();
    rd0.PasteClippedMatrix(temp13, 0, 0, 3, 1, 0, 0);
    temp13.MatrTranspose();

    temp13 = (Ny * (*d0));
    temp13.MatrTranspose();
    rd0.PasteClippedMatrix(temp13, 0, 0, 3, 1, 0, 1);
    temp13.MatrTranspose();

    temp13 = (Nz * (*d0));
    temp13.MatrTranspose();
    rd0.PasteClippedMatrix(temp13, 0, 0, 3, 1, 0, 2);
    detJ0 = rd0.Det();

    // Transformation : Orthogonal transformation (A and J)

    ChVector<double> G1;
    ChVector<double> G2;
    ChVector<double> G3;
    ChVector<double> G1xG2;
    double G1dotG1;
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
    G1dotG1 = Vdot(G1, G1);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1 = G1 / sqrt(G1[0] * G1[0] + G1[1] * G1[1] + G1[2] * G1[2]);
    A3 = G1xG2 / sqrt(G1xG2[0] * G1xG2[0] + G1xG2[1] * G1xG2[1] + G1xG2[2] * G1xG2[2]);
    A2.Cross(A3, A1);

    // Direction for orthotropic material
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
    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);
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

    // Enhanced Assumed Strain
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

    // Strain component

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

    // Straint derivative component

    ChMatrixNM<double, 6, 24> strainD_til;
    strainD_til.Reset();
    tempB = Nx * (*d) * Sx;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 0, 0);
    tempB = Ny * (*d) * Sy;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 1, 0);
    tempB = Nx * (*d) * Sy + Ny * (*d) * Sx;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 2, 0);
    //== Compatible strain (No ANS)==//
    tempB = Nz * (*d) * Sz;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 3, 0);
    tempB = Nx * (*d) * Sz + Nz * (*d) * Sx;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 4, 0);
    tempB = Ny * (*d) * Sz + Nz * (*d) * Sy;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 5, 0);
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

    ChMatrixNM<double, 9, 6> temp56;
    ChMatrixNM<double, 9, 1> HE1;
    ChMatrixNM<double, 9, 24> GDEPSP;
    ChMatrixNM<double, 9, 9> KALPHA;
    for (int ii = 0; ii < 9; ii++) {
        for (int jj = 0; jj < 6; jj++) {
            GT(ii, jj) = G(jj, ii);
        }
    }
    // If Mooney-Rivlin Material is selected -> Calculates internal forces and their Jacobian accordingly (new E_eps)
    if (element->m_isMooney) {
        ChMatrixNM<double, 3, 3> CG;     // CG: Right Cauchy-Green deformation tensor  C=trans(F)*F
        ChMatrixNM<double, 3, 3> INVCG;  // INVCG: Inverse of Right Cauchy-Green deformation tensor  C=trans(F)*F
        ChMatrixNM<double, 3, 3> IMAT;   // Identity matrix
        ChMatrixNM<double, 3, 3> I1PC;   // Stress tensor from first term of Mooney-Rivlin strain energy
        ChMatrixNM<double, 3, 3> I2PC;   // Stress tensor from second term of Mooney-Rivlin strain energy
        ChMatrixNM<double, 3, 3> JPC;    // Stress tensor from penalty term to ensure incompressibility
        ChMatrixNM<double, 3, 3>
            STR;  // Definition of stress tensor from strain energy (including penalty for incompressibility CCOM3)

        // Same quantities for the numerical calculation of the Jacobian of Mooney-Rivlin forces
        ChMatrixNM<double, 3, 3> CGN;     // CG: Right Cauchy-Green deformation tensor  C=trans(F)*F
        ChMatrixNM<double, 3, 3> INVCGN;  // INVCG: Inverse of Right Cauchy-Green deformation tensor  C=trans(F)*F
        ChMatrixNM<double, 3, 3> I1PCN;   // Stress tensor from first term of Mooney-Rivlin strain energy
        ChMatrixNM<double, 3, 3> I2PCN;   // Stress tensor from second term of Mooney-Rivlin strain energy
        ChMatrixNM<double, 3, 3> JPCN;    // Stress tensor from penalty term to ensure incompressibility
        ChMatrixNM<double, 3, 3>
            STRN;  // Definition of stress tensor from strain energy (including penalty for incompressibility CCOM3)

        ChMatrixNM<double, 6, 1> strain_1;
        ChMatrixNM<double, 6, 1> TEMP5;
        ChMatrixNM<double, 6, 1> TEMP5N;
        TEMP5.Reset();
        TEMP5N.Reset();

        // Right Cauchy - Green deformation tensor
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
        // Calculation of invariants I1, I2, and I3 and its deviatoric counterparts I1bar, I2bar, and I3bar
        double Deld = 0.000001;
        // First invariant of Right Cauchy-Green deformation tensor
        double I1 = CG(0, 0) + CG(1, 1) + CG(2, 2);
        // Second invariant of Right Cauchy-Green deformation tensor
        double I2 = 0.5 * (pow(I1, 2) - (pow(CG(0, 0), 2) + pow(CG(1, 0), 2) + pow(CG(2, 0), 2) + pow(CG(0, 1), 2) +
                                         pow(CG(1, 1), 2) + pow(CG(2, 1), 2) + pow(CG(0, 2), 2) + pow(CG(1, 2), 2) +
                                         pow(CG(2, 2), 2)));
        // Third invariant of Right Cauchy-Green deformation tensor (must be very close to 1 for incompressible
        // material)
        double I3 = CG(0, 0) * CG(1, 1) * CG(2, 2) - CG(0, 0) * CG(1, 2) * CG(2, 1) + CG(0, 1) * CG(1, 2) * CG(2, 0) -
                    CG(0, 1) * CG(1, 0) * CG(2, 2) + CG(0, 2) * CG(1, 0) * CG(2, 1) - CG(2, 0) * CG(1, 1) * CG(0, 2);
        double I1BAR = I1 / (pow(I3, 1.0 / 3.0));  // First invariant of the deviatoric tensor
        double I2BAR = I2 / (pow(I3, 2.0 / 3.0));  // Second invariant of the deviatoric tensor
        double J = sqrt(I3);
        // double CCOM1 = 551584.0;                                    // C10   not 0.551584
        // double CCOM2 = 137896.0;                                    // C01   not 0.137896
        double CCOM3 = 2.0 * (element->CCOM1 + element->CCOM2) / (1.0 - 2.0 * 0.49);  // K:bulk modulus
        double StockEPS;

        /// Calculation of stress tensor STR term to term: I1PC, I2PC, and JPC.
        // Identity matrix
        IMAT.Reset();
        IMAT(0, 0) = 1.0;
        IMAT(1, 1) = 1.0;
        IMAT(2, 2) = 1.0;
        // Stress tensor from first term of Mooney-Rivlin strain energy
        I1PC = (IMAT - INVCG * (1.0 / 3.0 * I1)) * pow(I3, -1.0 / 3.0);
        // Stress tensor from second term of Mooney-Rivlin strain energy
        I2PC = (((IMAT * I1) - CG) - (INVCG * (2.0 / 3.0) * I2)) * pow(I3, -2.0 / 3.0);
        // Stress tensor from penalty for incompressibility
        JPC = INVCG * (J / 2.0);
        // Definition of stress tensor from strain energy (including penalty for incompressibility CCOM3)
        STR = I1PC * (element->CCOM1 * 2.0) + I2PC * (element->CCOM2 * 2.0) + JPC * (CCOM3 * (J - 1.0) * 2.0);

        // Put the stress in vector form
        TEMP5(0, 0) = STR(0, 0);
        TEMP5(1, 0) = STR(1, 1);
        TEMP5(2, 0) = STR(0, 1);
        TEMP5(3, 0) = STR(2, 2);
        TEMP5(4, 0) = STR(0, 2);
        TEMP5(5, 0) = STR(1, 2);

        E_eps.Reset();

        // Compatible plus enhanced assumed strain
        strain_1 = strain;

        // Loop to obtain our Mooney-Rivling E_eps (tangential matrix of elastic coefficients)
        // E_eps is necessary for obtaining the Jacobian of MR internal forces
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
            I2 = 0.5 * (pow(I1, 2) - (pow(CGN(0, 0), 2) + pow(CGN(1, 0), 2) + pow(CGN(2, 0), 2) + pow(CGN(0, 1), 2) +
                                      pow(CGN(1, 1), 2) + pow(CGN(2, 1), 2) + pow(CGN(0, 2), 2) + pow(CGN(1, 2), 2) +
                                      pow(CGN(2, 2), 2)));
            I3 = CGN(0, 0) * CGN(1, 1) * CGN(2, 2) - CGN(0, 0) * CGN(1, 2) * CGN(2, 1) +
                 CGN(0, 1) * CGN(1, 2) * CGN(2, 0) - CGN(0, 1) * CGN(1, 0) * CGN(2, 2) +
                 CGN(0, 2) * CGN(1, 0) * CGN(2, 1) - CGN(2, 0) * CGN(1, 1) * CGN(0, 2);
            J = sqrt(I3);
            I1BAR = I1 / (pow(I3, 1.0 / 3.0));
            I2BAR = I2 / (pow(I3, 2.0 / 3.0));
            I1PCN = (IMAT - INVCGN * (1.0 / 3.0 * I1)) * pow(I3, -1.0 / 3.0);
            I2PCN = (((IMAT * I1) - CGN) - (INVCGN * (2.0 / 3.0) * I2)) * pow(I3, -2.0 / 3.0);
            JPCN = INVCGN * (J / 2.0);
            STRN = I1PCN * (element->CCOM1 * 2.0) + I2PCN * (element->CCOM2 * 2.0) + JPCN * (CCOM3 * (J - 1.0) * 2.0);
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
        // Add internal forces to Fint and HE1 for Mooney-Rivlin
        temp56.MatrMultiply(GT, E_eps);
        Fint.MatrTMultiply(strainD, TEMP5);
        Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        HE1.MatrMultiply(GT, TEMP5);
        HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
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
        // Add internal forces to Fint and HE1 for linear elastic material
        // temp56 is actually 9x6 for the brick element (9 EAS internal parameters)
        temp56.MatrMultiply(GT, E_eps);
        tempC.MatrTMultiply(strainD, E_eps);
        // Add generalized internal force
        Fint.MatrMultiply(tempC, strain);
        Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        // Add EAS internal force (vector of 9 components for each element)
        HE1.MatrMultiply(temp56, strain);
        HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    }  // end of   if(isMooney==1)

    // Internal force (linear isotropic or Mooney-Rivlin) Jacobian calculation
    // First term for Jacobian matrix
    temp246.MatrTMultiply(strainD, E_eps);
    // Second term for Jacobian matrix
    temp249.MatrTMultiply(Gd, Sigm);
    JAC11 = temp246 * strainD + temp249 * Gd;
    // Final expression for the Jacobian
    JAC11 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    // Jacobian of EAS forces w.r.t. element coordinates
    GDEPSP.MatrMultiply(temp56, strainD);
    GDEPSP *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    // Jacobian of EAS forces (w.r.t. EAS internal parameters)
    KALPHA.MatrMultiply(temp56, G);
    KALPHA *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);

    ChMatrixNM<double, 216, 1> GDEPSPVec;
    ChMatrixNM<double, 81, 1> KALPHAVec;
    ChMatrixNM<double, 576, 1> JACVec;

    result.Reset();
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 24; j++) {
            GDEPSPVec(i * 24 + j, 0) = GDEPSP(i, j);
        }
    }
    // GDEPSP = GDEPSPvec;
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            KALPHAVec(i * 9 + j, 0) = KALPHA(i, j);
        }
    }
    // KALPHAVec = KALPHA;
    for (int i = 0; i < 24; i++) {
        for (int j = 0; j < 24; j++) {
            JACVec(i * 24 + j, 0) = JAC11(i, j);
        }
    }
    // JACVec = JAC11;
    result.PasteClippedMatrix(Fint, 0, 0, 24, 1, 0, 0);
    result.PasteClippedMatrix(HE1, 0, 0, 9, 1, 24, 0);
    result.PasteClippedMatrix(GDEPSPVec, 0, 0, 216, 1, 33, 0);
    result.PasteClippedMatrix(KALPHAVec, 0, 0, 81, 1, 249, 0);
    result.PasteClippedMatrix(JACVec, 0, 0, 576, 1, 330, 0);
}
// -----------------------------------------------------------------------------
void ChElementBrick::MyMass::Evaluate(ChMatrixNM<double, 24, 24>& result,
                                      const double x,
                                      const double y,
                                      const double z) {
    element->ShapeFunctions(N, x, y, z);
    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

    // S=[N1*eye(3) N2*eye(3) N3*eye(3) N4*eye(3)...]
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

    // Perform  r = S'*S
    result.MatrTMultiply(S, S);

    result *= detJ0 * (element->GetLengthX() / 2) * (element->GetLengthY() / 2) * (element->GetLengthZ() / 2);
}
// -----------------------------------------------------------------------------
void ChElementBrick::MyForceNum::Evaluate(ChMatrixNM<double, 330, 1>& result,
                                          const double x,
                                          const double y,
                                          const double z) {
    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);
    element->Basis_M(M, x, y, z);  // EAS

    if (!element->m_isMooney) {  // m_isMooney == false means linear elastic material
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
    Sxi.Reset();

    Sxi.FillDiag(Nx(0));
    Sx.PasteMatrix(Sxi, 0, 0);
    Sxi.FillDiag(Nx(1));
    Sx.PasteMatrix(Sxi, 0, 3);
    Sxi.FillDiag(Nx(2));
    Sx.PasteMatrix(Sxi, 0, 6);
    Sxi.FillDiag(Nx(3));
    Sx.PasteMatrix(Sxi, 0, 9);
    Sxi.FillDiag(Nx(4));
    Sx.PasteMatrix(Sxi, 0, 12);
    Sxi.FillDiag(Nx(5));
    Sx.PasteMatrix(Sxi, 0, 15);
    Sxi.FillDiag(Nx(6));
    Sx.PasteMatrix(Sxi, 0, 18);
    Sxi.FillDiag(Nx(7));
    Sx.PasteMatrix(Sxi, 0, 21);

    ChMatrix33<> Syi;
    Syi.Reset();

    Syi.FillDiag(Ny(0));
    Sy.PasteMatrix(Syi, 0, 0);
    Syi.FillDiag(Ny(1));
    Sy.PasteMatrix(Syi, 0, 3);
    Syi.FillDiag(Ny(2));
    Sy.PasteMatrix(Syi, 0, 6);
    Syi.FillDiag(Ny(3));
    Sy.PasteMatrix(Syi, 0, 9);
    Syi.FillDiag(Ny(4));
    Sy.PasteMatrix(Syi, 0, 12);
    Syi.FillDiag(Ny(5));
    Sy.PasteMatrix(Syi, 0, 15);
    Syi.FillDiag(Ny(6));
    Sy.PasteMatrix(Syi, 0, 18);
    Syi.FillDiag(Ny(7));
    Sy.PasteMatrix(Syi, 0, 21);

    ChMatrix33<> Szi;
    Szi.Reset();

    Szi.FillDiag(Nz(0));
    Sz.PasteMatrix(Szi, 0, 0);
    Szi.FillDiag(Nz(1));
    Sz.PasteMatrix(Szi, 0, 3);
    Szi.FillDiag(Nz(2));
    Sz.PasteMatrix(Szi, 0, 6);
    Szi.FillDiag(Nz(3));
    Sz.PasteMatrix(Szi, 0, 9);
    Szi.FillDiag(Nz(4));
    Sz.PasteMatrix(Szi, 0, 12);
    Szi.FillDiag(Nz(5));
    Sz.PasteMatrix(Szi, 0, 15);
    Szi.FillDiag(Nz(6));
    Sz.PasteMatrix(Szi, 0, 18);
    Szi.FillDiag(Nz(7));
    Sz.PasteMatrix(Szi, 0, 21);

    //==EAS and Initial Shape==//
    ChMatrixNM<double, 3, 3> rd0;
    ChMatrixNM<double, 3, 3> temp33;
    temp33.Reset();
    temp33 = (Nx * (*d0));
    temp33.MatrTranspose();
    rd0.PasteClippedMatrix(temp33, 0, 0, 3, 1, 0, 0);
    temp33 = (Ny * (*d0));
    temp33.MatrTranspose();
    rd0.PasteClippedMatrix(temp33, 0, 0, 3, 1, 0, 1);
    temp33 = (Nz * (*d0));
    temp33.MatrTranspose();
    rd0.PasteClippedMatrix(temp33, 0, 0, 3, 1, 0, 2);
    detJ0 = rd0.Det();

    //////////////////////////////////////////////////////////////
    //// Transformation : Orthogonal transformation (A and J) ////
    //////////////////////////////////////////////////////////////
    ChVector<double> G1;
    ChVector<double> G2;
    ChVector<double> G3;
    ChVector<double> G1xG2;
    double G1dotG1;
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
    G1dotG1 = Vdot(G1, G1);

    ////Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;
    A1 = G1 / sqrt(G1[0] * G1[0] + G1[1] * G1[1] + G1[2] * G1[2]);
    A3 = G1xG2 / sqrt(G1xG2[0] * G1xG2[0] + G1xG2[1] * G1xG2[1] + G1xG2[2] * G1xG2[2]);
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
    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);
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
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 0, 0);
    tempB = Ny * (*d) * Sy;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 1, 0);
    tempB = Nx * (*d) * Sy + Ny * (*d) * Sx;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 2, 0);
    //== Compatible strain (No ANS)==//
    tempB = Nz * (*d) * Sz;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 3, 0);
    tempB = Nx * (*d) * Sz + Nz * (*d) * Sx;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 4, 0);
    tempB = Ny * (*d) * Sz + Nz * (*d) * Sy;
    strainD_til.PasteClippedMatrix(tempB, 0, 0, 1, 24, 5, 0);

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
    ChMatrixNM<double, 9, 1> HE1;
    ChMatrixNM<double, 9, 24> GDEPSP;
    ChMatrixNM<double, 9, 9> KALPHA;

    for (int ii = 0; ii < 9; ii++) {
        for (int jj = 0; jj < 6; jj++) {
            GT(ii, jj) = G(jj, ii);
        }
    }

    // m_isMooney == 1 use Iso_Nonlinear_Mooney-Rivlin Material (2-parameters=> 3 inputs)
    if (element->m_isMooney == 1) {
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
        double I2 = 0.5 * (pow(I1, 2) - (pow(CG(0, 0), 2) + pow(CG(1, 0), 2) + pow(CG(2, 0), 2) + pow(CG(0, 1), 2) +
                                         pow(CG(1, 1), 2) + pow(CG(2, 1), 2) + pow(CG(0, 2), 2) + pow(CG(1, 2), 2) +
                                         pow(CG(2, 2), 2)));
        double I3 = CG(0, 0) * CG(1, 1) * CG(2, 2) - CG(0, 0) * CG(1, 2) * CG(2, 1) + CG(0, 1) * CG(1, 2) * CG(2, 0) -
                    CG(0, 1) * CG(1, 0) * CG(2, 2) + CG(0, 2) * CG(1, 0) * CG(2, 1) - CG(2, 0) * CG(1, 1) * CG(0, 2);
        double I1BAR = I1 / (pow(I3, 1.0 / 3.0));
        double I2BAR = I2 / (pow(I3, 2.0 / 3.0));
        double J = sqrt(I3);
        // double CCOM1 = 551584.0;                                    // C10   not 0.551584
        // double CCOM2 = 137896.0;                                    // C01   not 0.137896
        double CCOM3 = 2.0 * (element->CCOM1 + element->CCOM2) / (1.0 - 2.0 * 0.49);  // K:bulk modulus
        double StockEPS;

        IMAT.Reset();
        IMAT(0, 0) = 1.0;
        IMAT(1, 1) = 1.0;
        IMAT(2, 2) = 1.0;

        I1PC = (IMAT - INVCG * (1.0 / 3.0 * I1)) * pow(I3, -1.0 / 3.0);
        I2PC = (((IMAT * I1) - CG) - (INVCG * (2.0 / 3.0) * I2)) * pow(I3, -2.0 / 3.0);
        JPC = INVCG * (J / 2.0);

        STR = I1PC * (element->CCOM1 * 2.0) + I2PC * (element->CCOM2 * 2.0) + JPC * (CCOM3 * (J - 1.0) * 2.0);

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
            I2 = 0.5 * (pow(I1, 2) - (pow(CGN(0, 0), 2) + pow(CGN(1, 0), 2) + pow(CGN(2, 0), 2) + pow(CGN(0, 1), 2) +
                                      pow(CGN(1, 1), 2) + pow(CGN(2, 1), 2) + pow(CGN(0, 2), 2) + pow(CGN(1, 2), 2) +
                                      pow(CGN(2, 2), 2)));
            I3 = CGN(0, 0) * CGN(1, 1) * CGN(2, 2) - CGN(0, 0) * CGN(1, 2) * CGN(2, 1) +
                 CGN(0, 1) * CGN(1, 2) * CGN(2, 0) - CGN(0, 1) * CGN(1, 0) * CGN(2, 2) +
                 CGN(0, 2) * CGN(1, 0) * CGN(2, 1) - CGN(2, 0) * CGN(1, 1) * CGN(0, 2);
            J = sqrt(I3);
            I1BAR = I1 / (pow(I3, 1.0 / 3.0));
            I2BAR = I2 / (pow(I3, 2.0 / 3.0));
            I1PCN = (IMAT - INVCGN * (1.0 / 3.0 * I1)) * pow(I3, -1.0 / 3.0);
            I2PCN = (((IMAT * I1) - CGN) - (INVCGN * (2.0 / 3.0) * I2)) * pow(I3, -2.0 / 3.0);
            JPCN = INVCGN * (J / 2.0);
            STRN = I1PCN * (element->CCOM1 * 2.0) + I2PCN * (element->CCOM2 * 2.0) + JPCN * (CCOM3 * (J - 1.0) * 2.0);
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
        Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        HE1.MatrMultiply(GT, TEMP5);
        HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    } else {
        temp56.MatrMultiply(GT, E_eps);
        tempC.MatrTMultiply(strainD, E_eps);
        Fint.MatrMultiply(tempC, strain);
        Fint *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
        HE1.MatrMultiply(temp56, strain);
        HE1 *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    }  // end of   if(*flag_Mooney==1)

    KALPHA.MatrMultiply(temp56, G);
    KALPHA *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);
    GDEPSP.MatrMultiply(temp56, strainD);
    GDEPSP *= detJ0 * (element->GetLengthX() / 2.0) * (element->GetLengthY() / 2.0) * (element->GetLengthZ() / 2.0);

    result.Reset();

    ChMatrixNM<double, 216, 1> GDEPSPVec;
    ChMatrixNM<double, 81, 1> KALPHAVec;
    GDEPSPVec = GDEPSP;
    KALPHAVec = KALPHA;
    result.PasteClippedMatrix(Fint, 0, 0, 24, 1, 0, 0);
    result.PasteClippedMatrix(HE1, 0, 0, 9, 1, 24, 0);
    result.PasteClippedMatrix(GDEPSPVec, 0, 0, 216, 1, 33, 0);
    result.PasteClippedMatrix(KALPHAVec, 0, 0, 81, 1, 249, 0);
}

// -----------------------------------------------------------------------------

void ChElementBrick::MyGravity::Evaluate(ChMatrixNM<double, 24, 1>& result,
                                         const double x,
                                         const double y,
                                         const double z) {
    element->ShapeFunctions(N, x, y, z);
    element->ShapeFunctionsDerivativeX(Nx, x, y, z);
    element->ShapeFunctionsDerivativeY(Ny, x, y, z);
    element->ShapeFunctionsDerivativeZ(Nz, x, y, z);

    // Weights for Gaussian integration
    double wx2 = (element->GetLengthX()) / 2;
    double wy2 = (element->GetLengthY()) / 2;
    double wz2 = (element->GetLengthZ()) / 2;

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

    for (int i = 0; i < 8; i++) {
        result(i * 3 + 0, 0) = N(0, i) * gacc.x();
        result(i * 3 + 1, 0) = N(0, i) * gacc.y();
        result(i * 3 + 2, 0) = N(0, i) * gacc.z();
    }

    result *= detJ0 * wx2 * wy2 * wz2;
}

void ChElementBrick::ComputeGravityForce(const ChVector<>& g_acc) {
    MyGravity myformula1(&m_d0, this, g_acc);
    m_GravForce.Reset();
    ChQuadrature::Integrate3D<ChMatrixNM<double, 24, 1>>(m_GravForce,  // result of integration will go there
                                                         myformula1,   // formula to integrate
                                                         -1, 1,        // limits in x direction
                                                         -1, 1,        // limits in y direction
                                                         -1, 1,        // limits in z direction
                                                         2             // order of integration
    );

    m_GravForce *= m_Material->Get_density();
}

void ChElementBrick::ComputeMassMatrix() {
    double rho = m_Material->Get_density();
    MyMass myformula(&m_d0, this);
    m_MassMatrix.Reset();
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

void ChElementBrick::SetupInitial(ChSystem* system) {
    // Compute gravitational forces
    ComputeGravityForce(system->Get_G_acc());
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
    H.PasteMatrix(temp, 0, 0);

    // 2) Store  +mf*[M]
    temp = m_MassMatrix;
    temp.MatrScale(Mfactor);

    // Paste scaled M mass matrix in resulting H:
    H.PasteSumMatrix(temp, 0, 0);
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
    ChMatrixNM<double, 1, 3> tempVecA;
    ShapeFunctionsDerivativeX(Nx, x, y, z);
    ShapeFunctionsDerivativeY(Ny, x, y, z);
    ShapeFunctionsDerivativeZ(Nz, x, y, z);
    tempVecA = (Nx * d0);
    tempVecA.MatrTranspose();
    rd0.PasteClippedMatrix(tempVecA, 0, 0, 3, 1, 0, 0);
    tempVecA.MatrTranspose();
    tempVecA = (Ny * d0);
    tempVecA.MatrTranspose();
    rd0.PasteClippedMatrix(tempVecA, 0, 0, 3, 1, 0, 1);
    tempVecA.MatrTranspose();
    tempVecA = (Nz * d0);
    tempVecA.MatrTranspose();
    rd0.PasteClippedMatrix(tempVecA, 0, 0, 3, 1, 0, 2);
    tempVecA.MatrTranspose();
    detJ0C = rd0.Det();
    // Transformation : Orthogonal transformation (A and J) ////
    ChVector<double> G1;
    ChVector<double> G2;
    ChVector<double> G3;
    ChVector<double> G1xG2;
    double G1dotG1;
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
    G1dotG1 = Vdot(G1, G1);

    // Tangent Frame
    ChVector<double> A1;
    ChVector<double> A2;
    ChVector<double> A3;

    A1 = G1 / sqrt(G1[0] * G1[0] + G1[1] * G1[1] + G1[2] * G1[2]);
    A3 = G1xG2 / sqrt(G1xG2[0] * G1xG2[0] + G1xG2[1] * G1xG2[1] + G1xG2[2] * G1xG2[2]);
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
    j01[0] = j0(0, 0);
    j02[0] = j0(1, 0);
    j03[0] = j0(2, 0);
    j01[1] = j0(0, 1);
    j02[1] = j0(1, 1);
    j03[1] = j0(2, 1);
    j01[2] = j0(0, 2);
    j02[2] = j0(1, 2);
    j03[2] = j0(2, 2);
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
