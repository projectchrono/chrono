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
// Authors: Alessandro Tasora
// =============================================================================
// Four node shell with geom.exact kinematics
// =============================================================================

#include "chrono/core/ChException.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/timestepper/ChState.h"
#include "chrono/fea/ChElementShellReissner4.h"
#include "chrono/fea/ChUtilsFEA.h"
#include "chrono/fea/ChRotUtils.h"
#include <cmath>

#define CHUSE_ANS
//#define CHUSE_EAS
#define CHUSE_KGEOMETRIC
//#define CHSIMPLIFY_DROT

namespace chrono {
namespace fea {

//--------------------------------------------------------------
// utility functions
//--------------------------------------------------------------

static inline double L1(const double xi[2]) {
    return 0.25 * (1. + xi[0]) * (1. + xi[1]);
};

static inline double L2(const double xi[2]) {
    return 0.25 * (1. - xi[0]) * (1. + xi[1]);
};

static inline double L3(const double xi[2]) {
    return 0.25 * (1. - xi[0]) * (1. - xi[1]);
};

static inline double L4(const double xi[2]) {
    return 0.25 * (1. + xi[0]) * (1. - xi[1]);
};

typedef double (*LI_Type)(const double xi[2]);
static LI_Type LI[4] = {&L1, &L2, &L3, &L4};

static inline double L1_1(const double xi[2]) {
    return 0.25 * (1. + xi[1]);
}

static inline double L1_2(const double xi[2]) {
    return 0.25 * (1. + xi[0]);
}

static inline double L2_1(const double xi[2]) {
    return -0.25 * (1. + xi[1]);
}

static inline double L2_2(const double xi[2]) {
    return 0.25 * (1. - xi[0]);
}

static inline double L3_1(const double xi[2]) {
    return -0.25 * (1. - xi[1]);
}

static inline double L3_2(const double xi[2]) {
    return -0.25 * (1. - xi[0]);
}

static inline double L4_1(const double xi[2]) {
    return 0.25 * (1. - xi[1]);
}

static inline double L4_2(const double xi[2]) {
    return -0.25 * (1. + xi[0]);
}

typedef double (*LI_J_Type)(const double xi[2]);
static LI_J_Type LI_J[4][2] = {
    {&L1_1, &L1_2},
    {&L2_1, &L2_2},
    {&L3_1, &L3_2},
    {&L4_1, &L4_2},
};

static ChVector<> Interp(const ChVector<>* const v, const double xi[2]) {
    ChVector<> r = v[0] * L1(xi) + v[1] * L2(xi) + v[2] * L3(xi) + v[3] * L4(xi);
    return r;
}

static ChVector<> InterpDeriv1(const ChVector<>* const v, const ChMatrixNM<double, 4, 2>& der_mat) {
    ChVector<> r = v[0] * der_mat(0, 0) + v[1] * der_mat(1, 0) + v[2] * der_mat(2, 0) + v[3] * der_mat(3, 0);
    return r;
}

static ChVector<> InterpDeriv2(const ChVector<>* const v, const ChMatrixNM<double, 4, 2>& der_mat) {
    ChVector<> r = v[0] * der_mat(0, 1) + v[1] * der_mat(1, 1) + v[2] * der_mat(2, 1) + v[3] * der_mat(3, 1);
    return r;
}

static void InterpDeriv(const ChVector<>* const v,
                        const ChMatrixNM<double, 4, 2>& der_mat,
                        ChVector<>& der1,
                        ChVector<>& der2) {
    der1 = InterpDeriv1(v, der_mat);
    der2 = InterpDeriv2(v, der_mat);
    return;
}

static ChVector<> InterpDeriv_xi1(const ChVector<>* const v, const double xi[2]) {
    ChVector<> r = v[0] * L1_1(xi) + v[1] * L2_1(xi) + v[2] * L3_1(xi) + v[3] * L4_1(xi);
    return r;
}

static ChVector<> InterpDeriv_xi2(const ChVector<>* const v, const double xi[2]) {
    ChVector<> r = v[0] * L1_2(xi) + v[1] * L2_2(xi) + v[2] * L3_2(xi) + v[3] * L4_2(xi);
    return r;
}

//--------------------------------------------------------------
// ChElementShellReissner4
//--------------------------------------------------------------

// Static integration tables

double ChElementShellReissner4::xi_i[ChElementShellReissner4::NUMIP][2] = {{-1. / std::sqrt(3.), -1. / std::sqrt(3.)},
                                                                           {1. / std::sqrt(3.), -1. / std::sqrt(3.)},
                                                                           {1. / std::sqrt(3.), 1. / std::sqrt(3.)},
                                                                           {-1. / std::sqrt(3.), 1. / std::sqrt(3.)}};

double ChElementShellReissner4::w_i[ChElementShellReissner4::NUMIP] = {1., 1., 1., 1.};

double ChElementShellReissner4::xi_A[ChElementShellReissner4::NUMSSEP][2] = {{0., 1.}, {-1., 0.}, {0., -1.}, {1., 0.}};

double ChElementShellReissner4::xi_n[ChElementShellReissner4::NUMNODES][2] = {{1., 1.},
                                                                              {-1., 1.},
                                                                              {-1., -1.},
                                                                              {1., -1.}};

double ChElementShellReissner4::xi_0[2] = {0., 0.};

void ChElementShellReissner4::UpdateNodalAndAveragePosAndOrientation() {
    ChMatrix33<> Tn[NUMNODES];
    ChMatrix33<> T_avg;
    T_avg.Reset();
    for (int i = 0; i < NUMNODES; i++) {
        xa[i] = this->m_nodes[i]->GetPos();
        Tn[i] = this->m_nodes[i]->GetA() * iTa[i];
        T_avg += this->m_nodes[i]->GetA() * iTa[i];  //***TODO*** use predicted rot?
    }
    T_avg *= 0.25;
    T_overline = ChRotUtils::Rot(ChRotUtils::VecRot(T_avg));
    /*
        // ***ALEX*** test an alternative for T_overline:
        // average four rotations with quaternion averaging:
        ChQuaternion<> qTa = Tn[0].Get_A_quaternion();
        ChQuaternion<> qTb = Tn[1].Get_A_quaternion();
        if ( (qTa ^ qTb) < 0)
            qTb *= -1;
        ChQuaternion<> qTc = Tn[2].Get_A_quaternion();
        if ( (qTa ^ qTc) < 0)
            qTc *= -1;
        ChQuaternion<> qTd = Tn[3].Get_A_quaternion();
        if ( (qTa ^ qTd) < 0)
            qTd *= -1;
        ChQuaternion<> Tavg =(qTa + qTb + qTc + qTd ).GetNormalized();
        T_overline.Set_A_quaternion(Tavg);
    */

    ChMatrix33<> R_tilde_n[NUMNODES];
    for (int i = 0; i < NUMNODES; i++) {
        R_tilde_n[i].MatrTMultiply(T_overline, Tn[i]);  // = T_overline.MulTM(Tn[i]);
        phi_tilde_n[i] = ChRotUtils::VecRot(R_tilde_n[i]);
        // if (phi_tilde_n[i].Length()*CH_C_RAD_TO_DEG > 15)
        //    GetLog() << "WARNING phi_tilde_n[" << i << "]=" <<  phi_tilde_n[i].Length()*CH_C_RAD_TO_DEG << "°\n";
    }
}

void ChElementShellReissner4::ComputeInitialNodeOrientation() {
    for (int i = 0; i < NUMNODES; i++) {
        xa[i] = this->m_nodes[i]->GetPos();
    }
    for (int i = 0; i < NUMNODES; i++) {
        ChVector<> t1 = InterpDeriv_xi1(xa, xi_n[i]);
        t1 = t1 / t1.Length();
        ChVector<> t2 = InterpDeriv_xi2(xa, xi_n[i]);
        t2 = t2 / t2.Length();
        ChVector<> t3 = Vcross(t1, t2);
        t3 = t3 / t3.Length();
        t2 = Vcross(t3, t1);
        ChMatrix33<> t123;
        t123.Set_A_axis(t1, t2, t3);
        iTa[i].MatrTMultiply(this->m_nodes[i]->GetA(), t123);
    }
    for (int i = 0; i < NUMIP; i++) {
        iTa_i[i] = ChMatrix33<>(1);
    }
    for (int i = 0; i < NUMSSEP; i++) {
        iTa_A[i] = ChMatrix33<>(1);
    }
    UpdateNodalAndAveragePosAndOrientation();
    InterpolateOrientation();
    for (int i = 0; i < NUMIP; i++) {
        ChVector<> t1 = InterpDeriv_xi1(xa, xi_i[i]);
        t1 = t1 / t1.Length();
        ChVector<> t2 = InterpDeriv_xi2(xa, xi_i[i]);
        t2 = t2 / t2.Length();
        ChVector<> t3 = Vcross(t1, t2);
        t3 = t3 / t3.Length();
        t2 = Vcross(t3, t1);
        ChMatrix33<> t123;
        t123.Set_A_axis(t1, t2, t3);
        iTa_i[i].MatrTMultiply(T_i[i], t123);
    }
    for (int i = 0; i < NUMSSEP; i++) {
        ChVector<> t1 = InterpDeriv_xi1(xa, xi_A[i]);
        t1 = t1 / t1.Length();
        ChVector<> t2 = InterpDeriv_xi2(xa, xi_A[i]);
        t2 = t2 / t2.Length();
        ChVector<> t3 = Vcross(t1, t2);
        t3 = t3 / t3.Length();
        t2 = Vcross(t3, t1);
        ChMatrix33<> t123;
        t123.Set_A_axis(t1, t2, t3);
        iTa_A[i].MatrTMultiply(T_A[i], t123);
    }
    InterpolateOrientation();
}

void ChElementShellReissner4::InterpolateOrientation() {
    ChMatrix33<> DRot_I_phi_tilde_n_MT_T_overline[NUMNODES];
    ChMatrix33<> Ri, Gammai;
    for (int n = 0; n < NUMNODES; n++) {
        ChMatrix33<> mDrot_I = ChRotUtils::DRot_I(phi_tilde_n[n]);
#ifdef CHSIMPLIFY_DROT
        mDrot_I = ChMatrix33<>(1);
#endif
        DRot_I_phi_tilde_n_MT_T_overline[n].MatrMultiplyT(mDrot_I, T_overline);
    }
    for (int i = 0; i < NUMIP; i++) {
        phi_tilde_i[i] = Interp(phi_tilde_n, xi_i[i]);
        ChRotUtils::RotAndDRot(phi_tilde_i[i], Ri, Gammai);
#ifdef CHSIMPLIFY_DROT
        Gammai = ChMatrix33<>(1);
#endif
        T_i[i] = T_overline * Ri * iTa_i[i];
        ChMatrix33<> T_overline_Gamma_tilde_i(T_overline * Gammai);
        for (int n = 0; n < NUMNODES; n++) {
            Phi_Delta_i[i][n] = T_overline_Gamma_tilde_i * DRot_I_phi_tilde_n_MT_T_overline[n];
        }
    }
    ChVector<> phi_tilde_0 = Interp(phi_tilde_n, xi_0);
    T_0 = T_overline * ChRotUtils::Rot(phi_tilde_0);
    for (int i = 0; i < NUMSSEP; i++) {
        phi_tilde_A[i] = Interp(phi_tilde_n, xi_A[i]);
        ChRotUtils::RotAndDRot(phi_tilde_A[i], Ri, Gammai);
#ifdef CHSIMPLIFY_DROT
        Gammai = ChMatrix33<>(1);
#endif
        T_A[i] = T_overline * Ri * iTa_A[i];
        ChMatrix33<> T_overline_Gamma_tilde_A(T_overline * Gammai);
        for (int n = 0; n < NUMNODES; n++) {
            Phi_Delta_A[i][n] = T_overline_Gamma_tilde_A * DRot_I_phi_tilde_n_MT_T_overline[n];
        }
    }
}

void ChElementShellReissner4::ComputeIPCurvature() {
    ChMatrix33<> Gamma_I_n_MT_T_overline[NUMNODES];
    for (int n = 0; n < NUMNODES; n++) {
        ChMatrix33<> mDrot_I = ChRotUtils::DRot_I(phi_tilde_n[n]);
#ifdef CHSIMPLIFY_DROT
        mDrot_I = ChMatrix33<>(1);
#endif
        Gamma_I_n_MT_T_overline[n].MatrMultiplyT(mDrot_I, T_overline);
    }
    for (int i = 0; i < NUMIP; i++) {
        ChVector<> phi_tilde_1_i;
        ChVector<> phi_tilde_2_i;
        InterpDeriv(phi_tilde_n, L_alpha_beta_i[i], phi_tilde_1_i, phi_tilde_2_i);
        ChMatrix33<> mGamma_tilde_i = ChRotUtils::DRot(phi_tilde_i[i]);
#ifdef CHSIMPLIFY_DROT
        mGamma_tilde_i = ChMatrix33<>(1);
#endif
        ChMatrix33<> T_overlineGamma_tilde_i(T_overline * mGamma_tilde_i);
        k_1_i[i] = T_overlineGamma_tilde_i * phi_tilde_1_i;
        k_2_i[i] = T_overlineGamma_tilde_i * phi_tilde_2_i;
        ChMatrix33<> tmp1 = T_overline * ChRotUtils::Elle(phi_tilde_i[i], phi_tilde_1_i);
        ChMatrix33<> tmp2 = T_overline * ChRotUtils::Elle(phi_tilde_i[i], phi_tilde_2_i);
#ifdef CHSIMPLIFY_DROT
        tmp1 = T_overline;
        tmp2 = T_overline;
#endif
        for (int n = 0; n < NUMNODES; n++) {
            Kappa_delta_i_1[i][n] =
                tmp1 * Gamma_I_n_MT_T_overline[n] * LI[n](xi_i[i]) + Phi_Delta_i[i][n] * L_alpha_beta_i[i](n, 0);
            Kappa_delta_i_2[i][n] =
                tmp2 * Gamma_I_n_MT_T_overline[n] * LI[n](xi_i[i]) + Phi_Delta_i[i][n] * L_alpha_beta_i[i](n, 1);
        }
    }
}

ChElementShellReissner4::ChElementShellReissner4() : tot_thickness(0) {
    m_nodes.resize(4);
    m_Alpha = 0;

    // add here other constructor-time initializations
    for (int i = 0; i < NUMIP; i++) {
        L_alpha_beta_i[i].Reset();
        B_overline_i[i].Reset();
        D_overline_i[i].Reset();
        G_i[i].Reset();
        P_i[i].Reset();
    }

    for (int i = 0; i < NUMSSEP; i++)
        L_alpha_beta_A[i].Reset();
}

ChElementShellReissner4::~ChElementShellReissner4() {}

void ChElementShellReissner4::SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                       std::shared_ptr<ChNodeFEAxyzrot> nodeB,
                                       std::shared_ptr<ChNodeFEAxyzrot> nodeC,
                                       std::shared_ptr<ChNodeFEAxyzrot> nodeD) {
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
    mvars.push_back(&m_nodes[1]->Variables());
    mvars.push_back(&m_nodes[2]->Variables());
    mvars.push_back(&m_nodes[3]->Variables());
    Kmatr.SetVariables(mvars);
}

ChVector<> ChElementShellReissner4::EvaluateGP(int igp) {
    return GetNodeA()->GetPos() * L1(xi_i[igp]) + GetNodeB()->GetPos() * L2(xi_i[igp]) +
           GetNodeC()->GetPos() * L3(xi_i[igp]) + GetNodeD()->GetPos() * L4(xi_i[igp]);
}
ChVector<> ChElementShellReissner4::EvaluatePT(int ipt) {
    return GetNodeA()->GetPos() * L1(xi_n[ipt]) + GetNodeB()->GetPos() * L2(xi_n[ipt]) +
           GetNodeC()->GetPos() * L3(xi_n[ipt]) + GetNodeD()->GetPos() * L4(xi_n[ipt]);
}

// -----------------------------------------------------------------------------
// Add a layer.
// -----------------------------------------------------------------------------

void ChElementShellReissner4::AddLayer(double thickness,
                                       double theta,
                                       std::shared_ptr<ChMaterialShellReissner> material) {
    m_layers.push_back(Layer(this, thickness, theta, material));
    SetLayerZreferenceCentered();
}

void ChElementShellReissner4::SetLayerZreferenceCentered() {
    // accumulate element thickness.
    tot_thickness = 0;
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        tot_thickness += m_layers[kl].Get_thickness();
    }

    // Loop again over the layers and calculate the z levels of layers, by centering them
    m_layers_z.clear();
    m_layers_z.push_back(-0.5 * this->GetThickness());
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        m_layers_z.push_back(m_layers_z[kl] + m_layers[kl].Get_thickness());
    }
}

void ChElementShellReissner4::SetLayerZreference(double z_from_bottom) {
    // accumulate element thickness.
    tot_thickness = 0;
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        tot_thickness += m_layers[kl].Get_thickness();
    }

    // Loop again over the layers and calculate the z levels of layers, by centering them
    m_layers_z.clear();
    m_layers_z.push_back(z_from_bottom);
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        m_layers_z.push_back(m_layers_z[kl] + m_layers[kl].Get_thickness());
    }
}

// -----------------------------------------------------------------------------
// Set as neutral position
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellReissner4::SetAsNeutral() {
    GetNodeA()->GetX0ref().SetPos(GetNodeA()->GetPos());
    GetNodeB()->GetX0ref().SetPos(GetNodeB()->GetPos());
    GetNodeC()->GetX0ref().SetPos(GetNodeC()->GetPos());
    GetNodeD()->GetX0ref().SetPos(GetNodeD()->GetPos());
    GetNodeA()->GetX0ref().SetRot(GetNodeA()->GetRot());
    GetNodeB()->GetX0ref().SetRot(GetNodeB()->GetRot());
    GetNodeC()->GetX0ref().SetRot(GetNodeC()->GetRot());
    GetNodeD()->GetX0ref().SetRot(GetNodeD()->GetRot());
}

// -----------------------------------------------------------------------------
// Interface to ChElementBase base class
// -----------------------------------------------------------------------------

// Initial element setup.
void ChElementShellReissner4::SetupInitial(ChSystem* system) {
    // Align initial pos/rot of nodes to actual pos/rot
    SetAsNeutral();

    ComputeInitialNodeOrientation();
    // 	UpdateNodalAndAveragePosAndOrientation();
    // 	InterpolateOrientation();
    // copy ref values
    T0_overline = T_overline;
    T_0_0 = T_0;
    for (int i = 0; i < NUMNODES; i++) {
        xa_0[i] = xa[i];
    }
    for (int i = 0; i < NUMIP; i++) {
        T_0_i[i] = T_i[i];
    }
    for (int i = 0; i < NUMSSEP; i++) {
        T_0_A[i] = T_A[i];
    }

    ChMatrixNM<double, 4, 4> M_0;
    ChMatrixNM<double, 4, 4> M_0_Inv;
    {
        ChVector<> x_1 = InterpDeriv_xi1(xa, xi_0);
        ChVector<> x_2 = InterpDeriv_xi2(xa, xi_0);
        S_alpha_beta_0(0, 0) = T_0_0.Get_A_Xaxis() ^ x_1;
        S_alpha_beta_0(1, 0) = T_0_0.Get_A_Yaxis() ^ x_1;
        S_alpha_beta_0(0, 1) = T_0_0.Get_A_Xaxis() ^ x_2;
        S_alpha_beta_0(1, 1) = T_0_0.Get_A_Yaxis() ^ x_2;
        alpha_0 = S_alpha_beta_0(0, 0) * S_alpha_beta_0(1, 1) - S_alpha_beta_0(0, 1) * S_alpha_beta_0(1, 0);

        M_0(0, 0) = S_alpha_beta_0(0, 0) * S_alpha_beta_0(0, 0);
        M_0(0, 1) = S_alpha_beta_0(0, 1) * S_alpha_beta_0(0, 1);
        M_0(0, 2) = S_alpha_beta_0(0, 1) * S_alpha_beta_0(0, 0);
        M_0(0, 3) = S_alpha_beta_0(0, 0) * S_alpha_beta_0(0, 1);

        M_0(1, 0) = S_alpha_beta_0(1, 0) * S_alpha_beta_0(1, 0);
        M_0(1, 1) = S_alpha_beta_0(1, 1) * S_alpha_beta_0(1, 1);
        M_0(1, 2) = S_alpha_beta_0(1, 1) * S_alpha_beta_0(1, 0);
        M_0(1, 3) = S_alpha_beta_0(1, 0) * S_alpha_beta_0(1, 1);

        M_0(2, 0) = S_alpha_beta_0(0, 0) * S_alpha_beta_0(1, 0);
        M_0(2, 1) = S_alpha_beta_0(0, 1) * S_alpha_beta_0(1, 1);
        M_0(2, 2) = S_alpha_beta_0(0, 0) * S_alpha_beta_0(1, 1);
        M_0(2, 3) = S_alpha_beta_0(0, 1) * S_alpha_beta_0(1, 0);

        M_0(3, 0) = S_alpha_beta_0(0, 0) * S_alpha_beta_0(1, 0);
        M_0(3, 1) = S_alpha_beta_0(0, 1) * S_alpha_beta_0(1, 1);
        M_0(3, 2) = S_alpha_beta_0(0, 1) * S_alpha_beta_0(1, 0);
        M_0(3, 3) = S_alpha_beta_0(0, 0) * S_alpha_beta_0(1, 1);

        M_0_Inv = M_0;
        M_0_Inv.MatrInverse();
    }

    for (int i = 0; i < NUMIP; i++) {
        ChMatrixNM<double, 4, 2> L_alpha_B_i;
        ChVector<> x_1 = InterpDeriv_xi1(xa, xi_i[i]);
        ChVector<> x_2 = InterpDeriv_xi2(xa, xi_i[i]);
        S_alpha_beta_i[i](0, 0) = T_0_i[i].Get_A_Xaxis() ^ x_1;
        S_alpha_beta_i[i](1, 0) = T_0_i[i].Get_A_Yaxis() ^ x_1;
        S_alpha_beta_i[i](0, 1) = T_0_i[i].Get_A_Xaxis() ^ x_2;
        S_alpha_beta_i[i](1, 1) = T_0_i[i].Get_A_Yaxis() ^ x_2;
        // alpha_i = det(S_alpha_beta_i)
        alpha_i[i] =
            S_alpha_beta_i[i](0, 0) * S_alpha_beta_i[i](1, 1) - S_alpha_beta_i[i](0, 1) * S_alpha_beta_i[i](1, 0);

        ChMatrixNM<double, 2, 2> xi_i_i;
        xi_i_i = S_alpha_beta_i[i];
        xi_i_i.MatrInverse();

        for (int n = 0; n < NUMNODES; n++) {
            for (int ii = 0; ii < 2; ii++) {
                L_alpha_B_i(n, ii) = LI_J[n][ii](xi_i[i]);
            }
        }

        L_alpha_beta_i[i].MatrMultiply(L_alpha_B_i, xi_i_i);

        double t = xi_i[i][0] * xi_i[i][1];
        ChMatrixNM<double, 4, IDOFS> H;
        H.Reset();

        H(0, 0) = xi_i[i][0];
        H(0, 1) = t;

        H(1, 2) = xi_i[i][1];
        H(1, 3) = t;

        H(2, 4) = xi_i[i][0];
        H(2, 5) = t;

        H(3, 6) = xi_i[i][1];
        H(3, 5) = t;

        ChMatrixNM<double, 12, 4> Perm;
        Perm.Reset();

        // 1, 5, 4, 2, 3, 6
        Perm(0, 0) = 1.;
        Perm(1, 2) = 1.;
        Perm(3, 3) = 1.;
        Perm(4, 1) = 1.;

        ChMatrixNM<double, 4, IDOFS> tmpP;
        tmpP.MatrTMultiply(M_0_Inv, H);

        P_i[i].MatrMultiply(Perm, tmpP);
        P_i[i].MatrScale(alpha_0 / alpha_i[i]);
    }
    // save initial axial values
    ComputeIPCurvature();
    for (int i = 0; i < NUMIP; i++) {
        InterpDeriv(xa, L_alpha_beta_i[i], y_i_1[i], y_i_2[i]);
        eps_tilde_1_0_i[i] = T_i[i].MatrT_x_Vect(y_i_1[i]);
        eps_tilde_2_0_i[i] = T_i[i].MatrT_x_Vect(y_i_2[i]);
        k_tilde_1_0_i[i] = T_i[i].MatrT_x_Vect(k_1_i[i]);
        k_tilde_2_0_i[i] = T_i[i].MatrT_x_Vect(k_2_i[i]);
    }
    for (int i = 0; i < NUMSSEP; i++) {
        ChMatrixNM<double, 4, 2> L_alpha_B_A;
        ChVector<> x_1 = InterpDeriv_xi1(xa, xi_A[i]);
        ChVector<> x_2 = InterpDeriv_xi2(xa, xi_A[i]);
        S_alpha_beta_A[i](0, 0) = T_0_A[i].Get_A_Xaxis() ^ x_1;
        S_alpha_beta_A[i](1, 0) = T_0_A[i].Get_A_Yaxis() ^ x_1;
        S_alpha_beta_A[i](0, 1) = T_0_A[i].Get_A_Xaxis() ^ x_2;
        S_alpha_beta_A[i](1, 1) = T_0_A[i].Get_A_Yaxis() ^ x_2;

        ChVector<> y_A_1;
        ChVector<> y_A_2;
        InterpDeriv(xa, L_alpha_beta_A[i], y_A_1, y_A_2);
        eps_tilde_1_0_A[i] = T_A[i].MatrT_x_Vect(y_A_1);
        eps_tilde_2_0_A[i] = T_A[i].MatrT_x_Vect(y_A_2);

        // xi_A_i = S_alpha_beta_A^{-1}
        ChMatrixNM<double, 2, 2> xi_A_i;
        xi_A_i = S_alpha_beta_A[i];
        xi_A_i.MatrInverse();

        for (int n = 0; n < NUMNODES; n++) {
            for (int ii = 0; ii < 2; ii++) {
                L_alpha_B_A(n, ii) = LI_J[n][ii](xi_A[i]);
            }
        }

        L_alpha_beta_A[i].MatrMultiply(L_alpha_B_A, xi_A_i);
    }
    {
        ChVector<> y_A_1;
        ChVector<> y_A_2;
        for (int i = 0; i < NUMSSEP; i++) {
            InterpDeriv(xa, L_alpha_beta_A[i], y_A_1, y_A_2);
            eps_tilde_1_0_A[i] = T_A[i].MatrT_x_Vect(y_A_1);
            eps_tilde_2_0_A[i] = T_A[i].MatrT_x_Vect(y_A_2);
        }
    }

    // Perform layer initialization
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        m_layers[kl].SetupInitial();
    }

    // compute initial sizes (just for auxiliary information)
    m_lenX =
        (0.5 * (GetNodeA()->coord.pos + GetNodeD()->coord.pos) - 0.5 * (GetNodeB()->coord.pos + GetNodeC()->coord.pos))
            .Length();
    m_lenY =
        (0.5 * (GetNodeA()->coord.pos + GetNodeB()->coord.pos) - 0.5 * (GetNodeD()->coord.pos + GetNodeC()->coord.pos))
            .Length();

    // Compute mass matrix
    ComputeMassMatrix();
}

// State update.
void ChElementShellReissner4::Update() {
    ChElementGeneric::Update();
}

// Fill the D vector with the current field values at the element nodes.
void ChElementShellReissner4::GetStateBlock(ChMatrixDynamic<>& mD) {
    mD.Reset(4 * 7, 1);
    mD.PasteVector(m_nodes[0]->GetPos(), 0, 0);
    mD.PasteQuaternion(m_nodes[0]->GetRot(), 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos(), 7, 0);
    mD.PasteQuaternion(m_nodes[1]->GetRot(), 10, 0);
    mD.PasteVector(m_nodes[2]->GetPos(), 14, 0);
    mD.PasteQuaternion(m_nodes[2]->GetRot(), 17, 0);
    mD.PasteVector(m_nodes[3]->GetPos(), 21, 0);
    mD.PasteQuaternion(m_nodes[3]->GetRot(), 24, 0);
}

// Calculate the global matrix H as a linear combination of K, R, and M:
//   H = Mfactor * [M] + Kfactor * [K] + Rfactor * [R]
// NOTE! we assume that this function is computed after one computed
// ComputeInternalForces(), that updates inner data for the given node states.

void ChElementShellReissner4::ComputeKRMmatricesGlobal(ChMatrix<>& H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.GetRows() == 24) && (H.GetColumns() == 24));

    // Calculate the mass matrix
    ComputeMassMatrix();

    // Calculate the linear combination Kfactor*[K] + Rfactor*[R]
    ComputeInternalJacobians(Kfactor, Rfactor);

    // Load Jac + Mfactor*[M] into H
    for (int i = 0; i < 24; i++)
        for (int j = 0; j < 24; j++)
            H(i, j) = m_JacobianMatrix(i, j) + Mfactor * m_MassMatrix(i, j);
}

// Return the mass matrix.
void ChElementShellReissner4::ComputeMmatrixGlobal(ChMatrix<>& M) {
    // Calculate the mass matrix
    ComputeMassMatrix();

    M = m_MassMatrix;
}

// -----------------------------------------------------------------------------
// Mass matrix calculation
// -----------------------------------------------------------------------------

void ChElementShellReissner4::ComputeMassMatrix() {
    m_MassMatrix.Reset();

    // loop over all layers, to compute total "mass per area" = sum(rho_i*thickness_i) = average_rho * sum(thickness_i)
    double avg_density = this->GetDensity();
    double mass_per_area = avg_density * tot_thickness;

    // Heuristic, simplified 'lumped' mass matrix.
    // Split the mass in 4 pieces, weighting as the jacobian at integration point, but
    // lump at the node closest to integration point.
    // This is simpler than the stiffness-consistent mass matrix that would require
    // integration over gauss points.

    for (int n = 0; n < NUMNODES; n++) {
        int igp = (n + 2) % 4;  // id of closest gauss point to node
        double jacobian =
            alpha_i[igp];  // weight: jacobian at gauss point (another,brute force,option would be 0.25 for all nodes)

        double nodemass = (jacobian * mass_per_area);

        // Approximate (!) inertia of a quarter of tile, note *(1/4) because only the quarter tile,
        // in local system of Gauss point
        double Ixx = (pow(this->GetLengthY(), 2) + pow(tot_thickness, 2)) * (1. / 12.) * (1. / 4.) * nodemass;
        double Iyy = (pow(this->GetLengthX(), 2) + pow(tot_thickness, 2)) * (1. / 12.) * (1. / 4.) * nodemass;
        double Izz = (pow(this->GetLengthX(), 2) + pow(this->GetLengthY(), 2)) * (1. / 12.) * (1. / 4.) * nodemass;
        ChMatrix33<> box_inertia(Ixx, 0., 0., 0., Iyy, 0., 0., 0., Izz);
        // ..and rotate inertia in local system of node: (local because ystem-level rotational coords of nodes are
        // ang.vel in loc sys)
        // I' = A'* T * I * T' * A
        //    = Rot * I * Rot'
        ChMatrix33<> Rot;
        Rot.MatrTMultiply(this->m_nodes[n]->GetA(), this->T_i[igp]);
        ChMatrix33<> Rot_I(Rot * box_inertia);
        ChMatrix33<> inertia_n;
        inertia_n.MatrMultiplyT(Rot_I, Rot);

        int node_off = n * 6;
        m_MassMatrix(node_off + 0, node_off + 0) = nodemass;
        m_MassMatrix(node_off + 1, node_off + 1) = nodemass;
        m_MassMatrix(node_off + 2, node_off + 2) = nodemass;
        m_MassMatrix.PasteMatrix(inertia_n, node_off + 3, node_off + 3);

    }  // end loop on nodes
}

// -----------------------------------------------------------------------------
// Elastic force calculation
// -----------------------------------------------------------------------------

void ChElementShellReissner4::ComputeInternalForces(ChMatrixDynamic<>& Fi) {
    Fi.Reset();

    UpdateNodalAndAveragePosAndOrientation();
    InterpolateOrientation();

    /*
	for (unsigned int i = 1; i <= iGetNumDof(); i++) {
		beta(i) = XCurr(iFirstReactionIndex + i);
	}
    */  //***TODO*** EAS internal variables not yet implemented

    ComputeIPCurvature();
    for (int i = 0; i < NUMIP; i++) {
        InterpDeriv(xa, L_alpha_beta_i[i], y_i_1[i], y_i_2[i]);
        eps_tilde_1_i[i] = T_i[i].MatrT_x_Vect(y_i_1[i]) - eps_tilde_1_0_i[i];
        eps_tilde_2_i[i] = T_i[i].MatrT_x_Vect(y_i_2[i]) - eps_tilde_2_0_i[i];
        k_tilde_1_i[i] = T_i[i].MatrT_x_Vect(k_1_i[i]) - k_tilde_1_0_i[i];
        k_tilde_2_i[i] = T_i[i].MatrT_x_Vect(k_2_i[i]) - k_tilde_2_0_i[i];

        ChMatrix33<> T_i_t(T_i[i]);
        T_i_t.MatrTranspose();
        ChMatrix33<> myi_1_X;
        myi_1_X.Set_X_matrix(y_i_1[i]);
        ChMatrix33<> myi_2_X;
        myi_2_X.Set_X_matrix(y_i_2[i]);
        ChMatrix33<> mk_1_X;
        mk_1_X.Set_X_matrix(k_1_i[i]);
        ChMatrix33<> mk_2_X;
        mk_2_X.Set_X_matrix(k_2_i[i]);
        ChMatrix33<> block;

        // parte variabile di B_overline_i
        for (int n = 0; n < NUMNODES; n++) {
            ChMatrix33<> Phi_Delta_i_n_LI_i = Phi_Delta_i[i][n] * LI[n](xi_i[i]);

            // delta epsilon_tilde_1_i
            block = T_i[i] * L_alpha_beta_i[i](n, 0);
            B_overline_i[i].PasteTranspMatrix(block, 0, 6 * n);
            block = T_i_t * myi_1_X * Phi_Delta_i_n_LI_i;
            block = block * this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            B_overline_i[i].PasteMatrix(block, 0, 3 + 6 * n);

            // delta epsilon_tilde_2_i
            block = T_i[i] * L_alpha_beta_i[i](n, 1);
            B_overline_i[i].PasteTranspMatrix(block, 3, 6 * n);
            block = T_i_t * myi_2_X * Phi_Delta_i_n_LI_i;
            block = block * this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            B_overline_i[i].PasteMatrix(block, 3, 3 + 6 * n);

            ChVector<> phi_tilde_1_i;
            ChVector<> phi_tilde_2_i;
            InterpDeriv(phi_tilde_n, L_alpha_beta_i[i], phi_tilde_1_i, phi_tilde_2_i);

            // delta k_tilde_1_i
            block = T_i_t * mk_1_X * Phi_Delta_i_n_LI_i + T_i_t * Kappa_delta_i_1[i][n];
            block = block * this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            B_overline_i[i].PasteMatrix(block, 6, 3 + 6 * n);

            // delta k_tilde_2_i
            block = T_i_t * mk_2_X * Phi_Delta_i_n_LI_i + T_i_t * Kappa_delta_i_2[i][n];
            block = block * this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            B_overline_i[i].PasteMatrix(block, 9, 3 + 6 * n);

            ChMatrix33<> Ieye(1);
            // delta y_alpha_1
            block = Ieye * L_alpha_beta_i[i](n, 0);
            D_overline_i[i].PasteMatrix(block, 0, 6 * n);

            // delta y_alpha_2
            block = Ieye * L_alpha_beta_i[i](n, 1);
            D_overline_i[i].PasteMatrix(block, 3, 6 * n);

            // delta k_1_i
            block = Kappa_delta_i_1[i][n] *
                    this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            D_overline_i[i].PasteMatrix(block, 6, 3 + 6 * n);

            // delta k_2_i
            block = Kappa_delta_i_2[i][n] *
                    this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            D_overline_i[i].PasteMatrix(block, 9, 3 + 6 * n);

            // phi_delta
            block = Phi_Delta_i_n_LI_i *
                    this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            D_overline_i[i].PasteMatrix(block, 12, 3 + 6 * n);
        }
    }

// ANS
#ifdef CHUSE_ANS

    ChMatrixNM<double, 6, 24> B_overline_A;
    ChVector<> y_A_1;
    ChVector<> y_A_2;
    ChMatrixNM<double, 4, 24> B_overline_3_ABCD;
    ChMatrixNM<double, 4, 24> B_overline_6_ABCD;

    for (int i = 0; i < NUMSSEP; i++) {
        B_overline_A.Reset();
        InterpDeriv(xa, L_alpha_beta_A[i], y_A_1, y_A_2);
        eps_tilde_1_A[i] = T_A[i].MatrT_x_Vect(y_A_1) - eps_tilde_1_0_A[i];
        eps_tilde_2_A[i] = T_A[i].MatrT_x_Vect(y_A_2) - eps_tilde_2_0_A[i];

        ChMatrix33<> T_A_t(T_A[i]);
        T_A_t.MatrTranspose();
        ChMatrix33<> myA_1_X;
        myA_1_X.Set_X_matrix(y_A_1);
        ChMatrix33<> myA_2_X;
        myA_2_X.Set_X_matrix(y_A_2);
        ChMatrix33<> block;

        for (int n = 0; n < NUMNODES; n++) {
            ChMatrix33<> Phi_Delta_A_n_LI_i = Phi_Delta_A[i][n] * LI[n](xi_A[i]);

            // delta epsilon_tilde_1_A
            block = T_A_t * L_alpha_beta_A[i](n, 0);
            B_overline_A.PasteMatrix(block, 0, 6 * n);
            block = T_A_t * myA_1_X * Phi_Delta_A_n_LI_i;
            block = block * this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            B_overline_A.PasteMatrix(block, 0, 3 + 6 * n);

            // delta epsilon_tilde_2_A
            block = T_A_t * L_alpha_beta_A[i](n, 1);
            B_overline_A.PasteMatrix(block, 3, 6 * n);
            block = T_A_t * myA_2_X * Phi_Delta_A_n_LI_i;
            block = block * this->m_nodes[n]->GetA();  //***NEEDED because in chrono rotations are body-relative
            B_overline_A.PasteMatrix(block, 3, 3 + 6 * n);
        }

        B_overline_3_ABCD.PasteClippedMatrix(B_overline_A, 2, 0, 1, 24, i, 0);
        B_overline_6_ABCD.PasteClippedMatrix(B_overline_A, 5, 0, 1, 24, i, 0);
    }
    ChMatrixNM<double, 1, 24> tmp_B_ANS;
    for (int i = 0; i < NUMIP; i++) {
        ChMatrixNM<double, 1, 4> sh1;
        ChMatrixNM<double, 1, 4> sh2;
        sh1(0, 0) = (1. + xi_i[i][1]) * 0.5;
        sh1(0, 1) = 0;
        sh1(0, 2) = (1. - xi_i[i][1]) * 0.5;
        sh1(0, 3) = 0;

        sh2(0, 0) = 0;
        sh2(0, 1) = (1. - xi_i[i][0]) * 0.5;
        sh2(0, 2) = 0;
        sh2(0, 3) = (1. + xi_i[i][0]) * 0.5;

        eps_tilde_1_i[i].z() = sh1(0, 0) * eps_tilde_1_A[0].z() + sh1(0, 2) * eps_tilde_1_A[2].z();
        eps_tilde_2_i[i].z() = sh2(0, 1) * eps_tilde_2_A[1].z() + sh2(0, 3) * eps_tilde_2_A[3].z();

        tmp_B_ANS.MatrMultiply(sh1, B_overline_3_ABCD);

        B_overline_i[i].PasteClippedMatrix(tmp_B_ANS, 0, 0, 1, 24, 2, 0);

        tmp_B_ANS.MatrMultiply(sh2, B_overline_6_ABCD);

        B_overline_i[i].PasteClippedMatrix(tmp_B_ANS, 0, 0, 1, 24, 5, 0);
    }

#endif

    // EAS: B membranali
    // 	{
    // 		int tmpidx1[5] = {0, 1, 5, 4, 2};
    // 		for (int i = 0; i < NUMIP; i++) {
    // 			for (int n = 1; n <= 4; n++) {
    //#if 0
    // 				CopyMatrixRow(B_overline_m_i[i], n, B_overline_i[i], tmpidx1[n]);
    //#endif
    // 				B_overline_m_i[i].CopyMatrixRow(n, B_overline_i[i], tmpidx1[n]);
    // 			}
    // 		}
    // 	}

    /* Calcola le azioni interne */
    for (int i = 0; i < NUMIP; i++) {
        epsilon.PasteVector(eps_tilde_1_i[i], 0, 0);
        epsilon.PasteVector(eps_tilde_2_i[i], 3, 0);
        epsilon.PasteVector(k_tilde_1_i[i], 6, 0);
        epsilon.PasteVector(k_tilde_2_i[i], 9, 0);

        //***TODO*** add the EAS effect using the epsilon_hat
        // epsilon_hat.MatrMultiply(P_i[i], beta);
        // epsilon += epsilon_hat;

        ChVector<> eps_tot_1, eps_tot_2, k_tot_1, k_tot_2;
        eps_tot_1 = epsilon.ClipVector(0, 0);
        eps_tot_2 = epsilon.ClipVector(3, 0);
        k_tot_1 = epsilon.ClipVector(6, 0);
        k_tot_2 = epsilon.ClipVector(9, 0);

        // Compute strains using
        // constitutive law of material

        ChVector<> n1, n2, m1, m2;
        ChVector<> l_n1, l_n2, l_m1, l_m2;
        // loop on layers
        for (size_t il = 0; il < this->m_layers.size(); ++il) {
            // compute layer stresses (per-unit-length forces and torques), and accumulate
            m_layers[il].GetMaterial()->ComputeStress(l_n1, l_n2, l_m1, l_m2, eps_tot_1, eps_tot_2, k_tot_1, k_tot_2,
                                                      m_layers_z[il], m_layers_z[il + 1], m_layers[il].Get_theta());
            n1 += l_n1;
            n2 += l_n2;
            m1 += l_m1;
            m2 += l_m2;
        }

        stress_i[i].PasteVector(n1, 0, 0);
        stress_i[i].PasteVector(n2, 3, 0);
        stress_i[i].PasteVector(m1, 6, 0);
        stress_i[i].PasteVector(m2, 9, 0);

        ChMatrix33<> Hh;
        ChVector<> Tn1 = T_i[i] * n1;
        ChVector<> Tn2 = T_i[i] * n2;
        ChVector<> Tm1 = T_i[i] * m1;
        ChVector<> Tm2 = T_i[i] * m2;
        Hh = TensorProduct(Tn1, y_i_1[i]) - ChMatrix33<>(Tn1 ^ y_i_1[i]) + TensorProduct(Tn2, y_i_2[i]) -
             ChMatrix33<>(Tn2 ^ y_i_2[i]) + TensorProduct(Tm1, k_1_i[i]) - ChMatrix33<>(Tm1 ^ k_1_i[i]) +
             TensorProduct(Tm2, k_2_i[i]) - ChMatrix33<>(Tm2 ^ k_2_i[i]);

        ChMatrix33<> block;
        block.Set_X_matrix(Tn1);
        G_i[i].PasteMatrix(block, 12, 0);
        block.Set_X_matrix(Tn2);
        G_i[i].PasteMatrix(block, 12, 3);
        block.Set_X_matrix(Tm1);
        G_i[i].PasteMatrix(block, 12, 6);
        block.Set_X_matrix(Tm2);
        G_i[i].PasteMatrix(block, 12, 9);
        block.Set_X_matrix(Tn1);
        G_i[i].PasteTranspMatrix(block, 0, 12);
        block.Set_X_matrix(Tn2);
        G_i[i].PasteTranspMatrix(block, 3, 12);
        G_i[i].PasteMatrix(Hh, 12, 12);
    }

    // Residual

    ChMatrixNM<double, 24, 1> rd;
    ChMatrixNM<double, IDOFS, 1> rbeta;
    for (int i = 0; i < NUMIP; i++) {
        rd.MatrTMultiply(B_overline_i[i], stress_i[i]);

        rd *= (-alpha_i[i] * w_i[i]);
        Fi.PasteSumMatrix(rd, 0, 0);

#ifdef CHUSE_EAS
        rbeta.MatrTMultiply(P_i[i], stress_i[i]);
        double dCoef = 1.0;  //***TODO*** autoset this
        rbeta *= (-alpha_i[i] * w_i[i] / dCoef);
        Fi.PasteSumMatrix(rbeta, 24, 0);
#endif
    }

    // Add Rayleigh stiffness-proportional damping, as
    //   Fi_damping = Alpha*[Km]*v
    // where the Km matrix is the material stiffness matrix.
    // This is just one of the many structural damping approaches, here it is
    // used because, later, computing the jacobian of internal forces (the damping matrix -[R])
    // becomes a very easy formula [R] = Alpha*[Km].
    if (this->m_Alpha) {
        // fills velocities with speeds of nodes:
        // {dxdt_1, angular_vel_1,  dxdt_2, angular_vel_2, ...}
        // Note that angular velocities are in node local frame because in C::E angular
        // increments are assumed in local frame csys.
        ChStateDelta velocities(24, nullptr);
        this->LoadableGetStateBlock_w(0, velocities);

        // compute Fi_damping = -Alpha*[Km]*v   without explicitly building [Km],
        // rather exploit the factorization B'CB, as:
        //   Alpha*[Km]*v = Alpha * [sum_integraton_points ( [B_i]'*[C]*[B_i] * a_i * w_i )] *v
        //   Alpha*[Km]*v = sum_integraton_points ( [B_i]' * (Alpha * [C]*[B_i] *v) * a_i * w_i )
        //   Alpha*[Km]*v = sum_integraton_points ( [B_i]' * (   stress_damp_i    ) * a_i * w_i )
        // where for convenience we denoted stress_damp = Alpha*[C]*[B_i]*v

        ChMatrixNM<double, 12, 1> strain_dt;
        ChMatrixNM<double, 12, 1> stress_damp;

        for (int i = 0; i < NUMIP; i++) {
            strain_dt.MatrMultiply(B_overline_i[i], velocities);  // [B_i] *v

            ChVector<> eps_dt_1, eps_dt_2, k_dt_1, k_dt_2;
            eps_dt_1 = strain_dt.ClipVector(0, 0);
            eps_dt_2 = strain_dt.ClipVector(3, 0);
            k_dt_1 = strain_dt.ClipVector(6, 0);
            k_dt_2 = strain_dt.ClipVector(9, 0);
            ChVector<> n1, n2, m1, m2;
            ChVector<> l_n1, l_n2, l_m1, l_m2;
            // loop on layers
            for (size_t il = 0; il < this->m_layers.size(); ++il) {
                // compute layer stresses (per-unit-length forces and torques), and accumulate  [C]*[B_i]*v
                m_layers[il].GetMaterial()->ComputeStress(l_n1, l_n2, l_m1, l_m2, eps_dt_1, eps_dt_2, k_dt_1, k_dt_2,
                                                          m_layers_z[il], m_layers_z[il + 1], m_layers[il].Get_theta());
                n1 += l_n1;
                n2 += l_n2;
                m1 += l_m1;
                m2 += l_m2;
            }
            stress_damp.PasteVector(n1, 0, 0);
            stress_damp.PasteVector(n2, 3, 0);
            stress_damp.PasteVector(m1, 6, 0);
            stress_damp.PasteVector(m2, 9, 0);
            stress_damp.MatrScale(this->m_Alpha);

            ChMatrixNM<double, 24, 1> rd_damp;
            rd_damp.MatrTMultiply(B_overline_i[i], stress_damp);
            rd_damp *= (-alpha_i[i] * w_i[i]);
            Fi.PasteSumMatrix(rd_damp, 0, 0);
        }

    }  // end Rayleigh damping
}

// -----------------------------------------------------------------------------
// Jacobians of internal forces
// -----------------------------------------------------------------------------

void ChElementShellReissner4::ComputeInternalJacobians(double Kfactor, double Rfactor) {
    m_JacobianMatrix.Reset();

    // tangente

    ChMatrixNM<double, 24, 24> Kg;
    ChMatrixNM<double, 24, 24> Km;
    ChMatrixNM<double, IDOFS, 24> K_beta_q;
    ChMatrixNM<double, IDOFS, IDOFS> K_beta_beta;

    ChMatrixNM<double, 24, 15> Ktg;
    ChMatrixNM<double, 24, 12> Ktm;

    ChMatrixNM<double, IDOFS, 12> Ktbetaq;

    ChMatrixNM<double, 12, 12> C;

    for (int i = 0; i < NUMIP; i++) {
        // GEOMETRIC STIFFNESS Kg:

        Ktg.MatrTMultiply(D_overline_i[i], G_i[i]);
        Kg.MatrMultiply(Ktg, D_overline_i[i]);

        // MATERIAL STIFFNESS Km:

        ChMatrixNM<double, 12, 12> C;
        ChMatrixNM<double, 12, 12> l_C;
        C.Reset();
        l_C.Reset();
        // loop on layers
        for (size_t il = 0; il < this->m_layers.size(); ++il) {
            // compute layer tang. material stiff, and accumulate
            m_layers[il].GetMaterial()->ComputeTangentC(
                l_C, eps_tilde_1_i[i], eps_tilde_2_i[i], k_tilde_1_i[i], k_tilde_2_i[i], m_layers_z[il],
                m_layers_z[il + 1],
                m_layers[il].Get_theta());  // ***TODO*** use the total epsilon including the 'hat' component from EAS
            C.MatrInc(l_C);
        }

        Ktm.MatrTMultiply(B_overline_i[i], C);
        Km.MatrMultiply(Ktm, B_overline_i[i]);

        // EAS STIFFNESS K_beta_q:

        Ktbetaq.MatrTMultiply(P_i[i], C);
        K_beta_q.MatrMultiply(Ktbetaq, B_overline_i[i]);

        // EAS STIFFNESS K_beta_beta:

        K_beta_beta.MatrMultiply(Ktbetaq, P_i[i]);

        // Assembly the entire jacobian
        //  [ Km   Kbq' ]
        //  [ Kbq  Kbb  ]

        double dCoef = 1.0;  //***TODO*** autoset this

        Km *= (alpha_i[i] * w_i[i] * dCoef * (Kfactor + Rfactor * this->m_Alpha));
        this->m_JacobianMatrix.PasteSumMatrix(Km, 0, 0);

#ifdef CHUSE_KGEOMETRIC
        Kg *= (alpha_i[i] * w_i[i] * dCoef * Kfactor);
        this->m_JacobianMatrix.PasteSumMatrix(Kg, 0, 0);
#endif

#ifdef CHUSE_EAS
        K_beta_q *= (alpha_i[i] * w_i[i] * Kfactor);
        this->m_JacobianMatrix.PasteSumMatrix(K_beta_q, 24, 0);
        this->m_JacobianMatrix.PasteSumTranspMatrix(K_beta_q, 0, 24);

        K_beta_beta *= (alpha_i[i] * (w_i[i] / dCoef) * Kfactor);
        this->m_JacobianMatrix.PasteSumMatrix(K_beta_beta, 24, 4);
#endif
    }
}

// -----------------------------------------------------------------------------
// Shape functions
// -----------------------------------------------------------------------------

void ChElementShellReissner4::ShapeFunctions(ChMatrix<>& N, double x, double y) {
    double xi[2];
    xi[0] = x, xi[1] = y;

    N(0) = L1(xi);
    N(1) = L2(xi);
    N(2) = L3(xi);
    N(3) = L4(xi);
}

void ChElementShellReissner4::ShapeFunctionsDerivativeX(ChMatrix<>& Nx, double x, double y) {
    double xi[2];
    xi[0] = x, xi[1] = y;
    Nx(0) = L1_1(xi);
    Nx(1) = L2_1(xi);
    Nx(2) = L3_1(xi);
    Nx(3) = L4_1(xi);
}

void ChElementShellReissner4::ShapeFunctionsDerivativeY(ChMatrix<>& Ny, double x, double y) {
    double xi[2];
    xi[0] = x, xi[1] = y;
    Ny(0) = L1_2(xi);
    Ny(1) = L2_2(xi);
    Ny(2) = L3_2(xi);
    Ny(3) = L4_2(xi);
}

// -----------------------------------------------------------------------------
// Helper functions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Interface to ChElementShell base class
// -----------------------------------------------------------------------------

void ChElementShellReissner4::EvaluateSectionDisplacement(const double u,
                                                          const double v,
                                                          ChVector<>& u_displ,
                                                          ChVector<>& u_rotaz) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, u_displ);
    u_rotaz = VNULL;  // no angles.. this is ANCF (or maybe return here the slope derivatives?)
}

void ChElementShellReissner4::EvaluateSectionFrame(const double u,
                                                   const double v,
                                                   ChVector<>& point,
                                                   ChQuaternion<>& rot) {
    // this is not a corotational element, so just do:
    EvaluateSectionPoint(u, v, point);
    rot = QUNIT;  // or maybe use gram-schmidt to get csys of section from slopes?
}

void ChElementShellReissner4::EvaluateSectionPoint(const double u,
                                                   const double v,
                                                   ChVector<>& point) {
    ChMatrixNM<double, 1, 4> N;
    this->ShapeFunctions(N, u, v);

    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& pD = m_nodes[3]->GetPos();

    point = N(0) * pA + N(1) * pB + N(2) * pC + N(3) * pD;
}

// -----------------------------------------------------------------------------
// Functions for ChLoadable interface
// -----------------------------------------------------------------------------

// Gets all the DOFs packed in a single vector (position part).
void ChElementShellReissner4::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.PasteVector(m_nodes[0]->GetPos(), block_offset, 0);
    mD.PasteQuaternion(m_nodes[0]->GetRot(), block_offset + 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos(), block_offset + 7, 0);
    mD.PasteQuaternion(m_nodes[1]->GetRot(), block_offset + 10, 0);
    mD.PasteVector(m_nodes[2]->GetPos(), block_offset + 14, 0);
    mD.PasteQuaternion(m_nodes[2]->GetRot(), block_offset + 17, 0);
    mD.PasteVector(m_nodes[3]->GetPos(), block_offset + 21, 0);
    mD.PasteQuaternion(m_nodes[3]->GetRot(), block_offset + 24, 0);
}

// Gets all the DOFs packed in a single vector (velocity part).
void ChElementShellReissner4::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.PasteVector(m_nodes[0]->GetPos_dt(), block_offset, 0);
    mD.PasteVector(m_nodes[0]->GetWvel_loc(), block_offset + 3, 0);
    mD.PasteVector(m_nodes[1]->GetPos_dt(), block_offset + 6, 0);
    mD.PasteVector(m_nodes[1]->GetWvel_loc(), block_offset + 9, 0);
    mD.PasteVector(m_nodes[2]->GetPos_dt(), block_offset + 12, 0);
    mD.PasteVector(m_nodes[2]->GetWvel_loc(), block_offset + 15, 0);
    mD.PasteVector(m_nodes[3]->GetPos_dt(), block_offset + 18, 0);
    mD.PasteVector(m_nodes[3]->GetWvel_loc(), block_offset + 21, 0);
}

void ChElementShellReissner4::LoadableStateIncrement(const unsigned int off_x, ChState& x_new, const ChState& x, const unsigned int off_v, const ChStateDelta& Dv)  {
    for (int i = 0; i < 4; i++) {
        this->m_nodes[i]->NodeIntStateIncrement(off_x  + 7 * i  , x_new, x, off_v  + 6 * i  , Dv);
    }
}

void ChElementShellReissner4::EvaluateSectionVelNorm(double U, double V, ChVector<>& Result) {
    ChMatrixNM<double, 4, 1> N;
    ShapeFunctions(N, U, V);
    for (unsigned int ii = 0; ii < 4; ii++) {
        Result += N(ii) * m_nodes[ii]->GetPos_dt();
    }
}

// Get the pointers to the contained ChVariables, appending to the mvars vector.
void ChElementShellReissner4::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    for (int i = 0; i < m_nodes.size(); ++i) {
        mvars.push_back(&m_nodes[i]->Variables());
    }
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V) coordinates of the surface.
void ChElementShellReissner4::ComputeNF(
    const double U,              // parametric coordinate in surface
    const double V,              // parametric coordinate in surface
    ChVectorDynamic<>& Qi,       // Return result of Q = N'*F  here
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is =n. field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 4> N;
    ShapeFunctions(N, U, V);

    //***TODO*** exact det of jacobian at u,v
    detJ = GetLengthX() * GetLengthY() / 4.0;  // approximate

    ChVector<> tmp;
    ChVector<> Fv = F.ClipVector(0, 0);
    ChVector<> Mv = F.ClipVector(3, 0);
    tmp = N(0) * Fv;
    Qi.PasteVector(tmp, 0, 0);
    tmp = N(0) * Mv;
    Qi.PasteVector(tmp, 3, 0);
    tmp = N(1) * Fv;
    Qi.PasteVector(tmp, 6, 0);
    tmp = N(1) * Mv;
    Qi.PasteVector(tmp, 9, 0);
    tmp = N(2) * Fv;
    Qi.PasteVector(tmp, 12, 0);
    tmp = N(2) * Mv;
    Qi.PasteVector(tmp, 15, 0);
    tmp = N(3) * Fv;
    Qi.PasteVector(tmp, 18, 0);
    tmp = N(3) * Mv;
    Qi.PasteVector(tmp, 21, 0);
}

// Evaluate N'*F , where N is the shape function evaluated at (U,V,W) coordinates of the surface.
void ChElementShellReissner4::ComputeNF(
    const double U,              // parametric coordinate in volume
    const double V,              // parametric coordinate in volume
    const double W,              // parametric coordinate in volume
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is = n.field coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
    ) {
    ChMatrixNM<double, 1, 4> N;
    ShapeFunctions(N, U, V);

    detJ = GetLengthX() * GetLengthY() / 4.0;  // approximate
    detJ *= GetThickness();

    ChVector<> tmp;
    ChVector<> Fv = F.ClipVector(0, 0);
    ChVector<> Mv = F.ClipVector(3, 0);
    tmp = N(0) * Fv;
    Qi.PasteVector(tmp, 0, 0);
    tmp = N(0) * Mv;
    Qi.PasteVector(tmp, 3, 0);
    tmp = N(1) * Fv;
    Qi.PasteVector(tmp, 6, 0);
    tmp = N(1) * Mv;
    Qi.PasteVector(tmp, 9, 0);
    tmp = N(2) * Fv;
    Qi.PasteVector(tmp, 12, 0);
    tmp = N(2) * Mv;
    Qi.PasteVector(tmp, 15, 0);
    tmp = N(3) * Fv;
    Qi.PasteVector(tmp, 18, 0);
    tmp = N(3) * Mv;
    Qi.PasteVector(tmp, 21, 0);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------

// Calculate average element density (needed for ChLoaderVolumeGravity).
double ChElementShellReissner4::GetDensity() {
    double tot_density = 0;
    for (size_t kl = 0; kl < m_layers.size(); kl++) {
        double rho = m_layers[kl].GetMaterial()->Get_rho();
        double layerthick = m_layers[kl].Get_thickness();
        tot_density += rho * layerthick;
    }
    return tot_density / tot_thickness;
}

// Calculate normal to the surface at (U,V) coordinates.
ChVector<> ChElementShellReissner4::ComputeNormal(const double U, const double V) {
    ChMatrixNM<double, 1, 4> Nx;
    ChMatrixNM<double, 1, 4> Ny;
    ShapeFunctionsDerivativeX(Nx, U, V);
    ShapeFunctionsDerivativeY(Ny, U, V);

    const ChVector<>& pA = m_nodes[0]->GetPos();
    const ChVector<>& pB = m_nodes[1]->GetPos();
    const ChVector<>& pC = m_nodes[2]->GetPos();
    const ChVector<>& pD = m_nodes[3]->GetPos();

    ChVector<> Gx = Nx(0) * pA + Nx(1) * pB + Nx(2) * pC + Nx(3) * pD;
    ChVector<> Gy = Ny(0) * pA + Ny(1) * pB + Ny(2) * pC + Ny(3) * pD;

    ChVector<> mnorm = Vcross(Gx, Gy);
    return mnorm.GetNormalized();
}

// Private constructor (a layer can be created only by adding it to an element)
ChElementShellReissner4::Layer::Layer(ChElementShellReissner4* element,
                                      double thickness,
                                      double theta,
                                      std::shared_ptr<ChMaterialShellReissner> material)
    : m_element(element), m_thickness(thickness), m_theta(theta), m_material(material) {}

// Initial setup for this layer:
void ChElementShellReissner4::Layer::SetupInitial() {}

}  // end namespace fea
}  // end namespace chrono
