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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

//#define BEAM_VERBOSE

#include "chrono/fea/ChElementBeamEuler.h"

namespace chrono {
namespace fea {

ChElementBeamEuler::ChElementBeamEuler()
    : q_refrotA(QUNIT),
      q_refrotB(QUNIT),
      q_element_abs_rot(QUNIT),
      q_element_ref_rot(QUNIT),
      force_symmetric_stiffness(false),
      disable_corotate(false),
      disable_projector(true)  //***TO DO*** see why projectors work worse... ex. see lateral buckling test
{
    nodes.resize(2);

    StiffnessMatrix.setZero(this->GetNdofs(), this->GetNdofs());
}

void ChElementBeamEuler::SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA, std::shared_ptr<ChNodeFEAxyzrot> nodeB) {
    assert(nodeA);
    assert(nodeB);

    nodes[0] = nodeA;
    nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementBeamEuler::ShapeFunctions(ShapeVector& N, double eta) {
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);
    double Ny1 = (1. / 4.) * pow((1 - eta), 2) * (2 + eta);
    double Ny2 = (1. / 4.) * pow((1 + eta), 2) * (2 - eta);
    double Nr1 = (this->length / 8.) * pow((1 - eta), 2) * (1 + eta);
    double Nr2 = (this->length / 8.) * pow((1 + eta), 2) * (eta - 1);
    /*
    N(0) = Nx1;
    N(1) = Ny1;
    N(2) = Ny1;
    N(3) = Nx1;
    N(4) = -Nr1;
    N(5) = Nr1;
    N(6) = Nx2;
    N(7) = Ny2;
    N(8) = Ny2;
    N(9) = Nx2;
    N(10) = -Nr2;
    N(11) = Nr2;
    */
    double dN_ua = (1. / (2. * this->length)) * (-3. + 3 * eta * eta);
    double dN_ub = (1. / (2. * this->length)) * (3. - 3 * eta * eta);
    double dN_ra = (1. / 4.) * (-1. - 2 * eta + 3 * eta * eta);
    double dN_rb = -(1. / 4.) * (1. - 2 * eta - 3 * eta * eta);
    N(0) = Nx1;
    N(1) = Ny1;
    N(2) = Nr1;
    N(3) = Nx2;
    N(4) = Ny2;
    N(5) = Nr2;
    N(6) = dN_ua;
    N(7) = dN_ub;
    N(8) = dN_ra;
    N(9) = dN_rb;
}

void ChElementBeamEuler::Update() {
    // parent class update:
    ChElementGeneric::Update();

    // always keep updated the rotation matrix A:
    this->UpdateRotation();
}

void ChElementBeamEuler::UpdateRotation() {
    ChMatrix33<> A0(this->q_element_ref_rot);

    ChMatrix33<> Aabs;
    if (this->disable_corotate) {
        Aabs = A0;
        q_element_abs_rot = q_element_ref_rot;
    } else {
        ChVector<> mXele_w = nodes[1]->Frame().GetPos() - nodes[0]->Frame().GetPos();
        // propose Y_w as absolute dir of the Y axis of A node, removing the effect of Aref-to-A rotation if any:
        //    Y_w = [R Aref->w ]*[R Aref->A ]'*{0,1,0}
        ChVector<> myele_wA = nodes[0]->Frame().GetRot().Rotate(q_refrotA.RotateBack(ChVector<>(0, 1, 0)));
        // propose Y_w as absolute dir of the Y axis of B node, removing the effect of Bref-to-B rotation if any:
        //    Y_w = [R Bref->w ]*[R Bref->B ]'*{0,1,0}
        ChVector<> myele_wB = nodes[1]->Frame().GetRot().Rotate(q_refrotB.RotateBack(ChVector<>(0, 1, 0)));
        // Average the two Y directions to have midpoint torsion (ex -30° torsion A and +30° torsion B= 0°)
        ChVector<> myele_w = (myele_wA + myele_wB).GetNormalized();
        Aabs.Set_A_Xdir(mXele_w, myele_w);
        q_element_abs_rot = Aabs.Get_A_quaternion();
    }

    this->A = A0.transpose() * Aabs;
}

void ChElementBeamEuler::GetStateBlock(ChVectorDynamic<>& mD) {
    mD.resize(12);

    ChVector<> delta_rot_dir;
    double delta_rot_angle;

    // Node 0, displacement (in local element frame, corotated back)
    //     d = [Atw]' Xt - [A0w]'X0
    ChVector<> displ = this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos()) -
                       this->q_element_ref_rot.RotateBack(nodes[0]->GetX0().GetPos());
    mD.segment(0, 3) = displ.eigen();

    // Node 0, x,y,z small rotations (in local element frame)
    ChQuaternion<> q_delta0 = q_element_abs_rot.GetConjugate() % nodes[0]->Frame().GetRot() % q_refrotA.GetConjugate();
    // note, for small incremental rotations this is opposite of ChNodeFEAxyzrot::VariablesQbIncrementPosition
    q_delta0.Q_to_AngAxis(delta_rot_angle, delta_rot_dir);

    if (delta_rot_angle > CH_C_PI)
        delta_rot_angle -= CH_C_2PI;  // no 0..360 range, use -180..+180

    mD.segment(3, 3) = delta_rot_angle * delta_rot_dir.eigen();

    // Node 1, displacement (in local element frame, corotated back)
    //     d = [Atw]' Xt - [A0w]'X0
    displ = this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos()) -
            this->q_element_ref_rot.RotateBack(nodes[1]->GetX0().GetPos());
    mD.segment(6, 3) = displ.eigen();

    // Node 1, x,y,z small rotations (in local element frame)
    ChQuaternion<> q_delta1 = q_element_abs_rot.GetConjugate() % nodes[1]->Frame().GetRot() % q_refrotB.GetConjugate();
    // note, for small incremental rotations this is opposite of ChNodeFEAxyzrot::VariablesQbIncrementPosition
    q_delta1.Q_to_AngAxis(delta_rot_angle, delta_rot_dir);

    if (delta_rot_angle > CH_C_PI)
        delta_rot_angle -= CH_C_2PI;  // no 0..360 range, use -180..+180

    mD.segment(9, 3) = delta_rot_angle * delta_rot_dir.eigen();
}

void ChElementBeamEuler::GetField_dt(ChVectorDynamic<>& mD_dt) {
    mD_dt.resize(12);

    // Node 0, velocity (in local element frame, corotated back by A' )
    mD_dt.segment(0, 3) = q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos_dt()).eigen();

    // Node 0, x,y,z ang.velocity (in local element frame, corotated back by A' )
    mD_dt.segment(3, 3) = q_element_abs_rot.RotateBack(nodes[0]->Frame().GetWvel_par()).eigen();

    // Node 1, velocity (in local element frame, corotated back by A' )
    mD_dt.segment(6, 3) = q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos_dt()).eigen();

    // Node 1, x,y,z ang.velocity (in local element frame, corotated back by A' )
    mD_dt.segment(9, 3) = q_element_abs_rot.RotateBack(nodes[1]->Frame().GetWvel_par()).eigen();
}

void ChElementBeamEuler::ComputeStiffnessMatrix() {
    assert(section);
    /*
    double Area = section->Area;
    double E = section->E;
    double Izz = section->Izz;
    double Iyy = section->Iyy;
    double G = section->G;
    double Jpolar = section->J;
    */
    double EA = this->section->GetAxialRigidity();
    double EIyy = this->section->GetYbendingRigidity();
    double EIzz = this->section->GetZbendingRigidity();
    double GJ = this->section->GetXtorsionRigidity();
    double Cy = this->section->GetCentroidY();
    double Cz = this->section->GetCentroidZ();
    double Sy = this->section->GetShearCenterY();
    double Sz = this->section->GetShearCenterZ();

    double h = this->length;

    double om_xz = 0;  // For Euler-Bernoulli
    double om_xy = 0;  // For Euler-Bernoulli
    /*
    if (false) {
        //***TEST REDDY BEAMS***
        double Ks_z = section->Ks_z;
        double Ks_y = section->Ks_y;
        double om_xz = E * Iyy / (G * Area * Ks_z * h);  // For Reddy's RBT
        double om_xy = E * Izz / (G * Area * Ks_y * h);  // For Reddy's RBT
    }
    */
    double u_xz = 1 + 12 * om_xz;
    double l_xz = 1 + 3 * om_xz;
    double e_xz = 1 - 6 * om_xz;
    double u_xy = 1 + 12 * om_xy;
    double l_xy = 1 + 3 * om_xy;
    double e_xy = 1 - 6 * om_xy;

    double k_u = EA / h;
    double k_f = GJ / h;

    double k_w = 12 * EIyy / (u_xz * h * h * h);
    double k_v = 12 * EIzz / (u_xy * h * h * h);

    double k_t = 4 * EIyy * l_xz / (u_xz * h);
    double k_p = 4 * EIzz * l_xy / (u_xy * h);

    double k_wt = 6 * EIyy / (u_xz * h * h);
    double k_vp = 6 * EIzz / (u_xy * h * h);

    double k_tt = 2 * EIyy * e_xz / (u_xz * h);
    double k_pp = 2 * EIzz * e_xy / (u_xy * h);

    StiffnessMatrix(0, 0) = k_u;
    StiffnessMatrix(1, 1) = k_v;
    StiffnessMatrix(2, 2) = k_w;
    StiffnessMatrix(3, 3) = k_f;
    StiffnessMatrix(4, 4) = k_t;
    StiffnessMatrix(5, 5) = k_p;
    StiffnessMatrix(6, 6) = k_u;
    StiffnessMatrix(7, 7) = k_v;
    StiffnessMatrix(8, 8) = k_w;
    StiffnessMatrix(9, 9) = k_f;
    StiffnessMatrix(10, 10) = k_t;
    StiffnessMatrix(11, 11) = k_p;

    StiffnessMatrix(0, 6) = -k_u;
    StiffnessMatrix(1, 7) = -k_v;
    StiffnessMatrix(2, 8) = -k_w;
    StiffnessMatrix(3, 9) = -k_f;
    StiffnessMatrix(4, 10) = k_tt;
    StiffnessMatrix(5, 11) = k_pp;

    StiffnessMatrix(4, 8) = k_wt;
    StiffnessMatrix(5, 7) = -k_vp;
    StiffnessMatrix(1, 11) = k_vp;
    StiffnessMatrix(2, 10) = -k_wt;

    StiffnessMatrix(1, 5) = k_vp;
    StiffnessMatrix(2, 4) = -k_wt;
    StiffnessMatrix(7, 11) = -k_vp;
    StiffnessMatrix(8, 10) = k_wt;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = r + 1; c < 12; c++)
            StiffnessMatrix(c, r) = StiffnessMatrix(r, c);

    // In case the section is rotated:
    if (this->section->GetSectionRotation()) {
        // Do [K]^ = [R][K][R]'
        ChMatrix33<> Rotsect;
        Rotsect.Set_A_Rxyz(ChVector<>(-section->GetSectionRotation(), 0, 0));
        ChMatrixDynamic<> CKtemp(12, 12);
        CKtemp.setZero();
        ChMatrixCorotation::ComputeCK(this->StiffnessMatrix, Rotsect, 4, CKtemp);
        ChMatrixCorotation::ComputeKCt(CKtemp, Rotsect, 4, this->StiffnessMatrix);
    }
    // In case the section has a centroid displacement:

    if (Cy || Cz) {
        // Do [K]" = [T_c][K]^[T_c]'

        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(4, i) += Cz * this->StiffnessMatrix(0, i);
        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(5, i) += -Cy * this->StiffnessMatrix(0, i);

        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(10, i) += Cz * this->StiffnessMatrix(6, i);
        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(11, i) += -Cy * this->StiffnessMatrix(6, i);

        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(i, 4) += Cz * this->StiffnessMatrix(i, 0);
        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(i, 5) += -Cy * this->StiffnessMatrix(i, 0);

        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(i, 10) += Cz * this->StiffnessMatrix(i, 6);
        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(i, 11) += -Cy * this->StiffnessMatrix(i, 6);
    }

    // In case the section has a shear center displacement:
    if (Sy || Sz) {
        // Do [K]° = [T_s][K]"[T_s]'

        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(3, i) +=
                - Sz * this->StiffnessMatrix(1, i) + Sy * this->StiffnessMatrix(2, i);
        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(9, i) +=
                - Sz * this->StiffnessMatrix(7, i) + Sy * this->StiffnessMatrix(8, i);

        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(i, 3) +=
                - Sz * this->StiffnessMatrix(i, 1) + Sy * this->StiffnessMatrix(i, 2);
        for (int i = 0; i < 12; ++i)
            this->StiffnessMatrix(i, 9) +=
                - Sz * this->StiffnessMatrix(i, 7) + Sy * this->StiffnessMatrix(i, 8);
    }
}

void ChElementBeamEuler::SetupInitial(ChSystem* system) {
    assert(section);

    // Compute rest length, mass:
    this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
    this->mass = this->length * this->section->GetMassPerUnitLength();

    // Compute initial rotation
    ChMatrix33<> A0;
    ChVector<> mXele = nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos();
    ChVector<> myele = nodes[0]->GetX0().GetA().Get_A_Yaxis();
    A0.Set_A_Xdir(mXele, myele);
    q_element_ref_rot = A0.Get_A_quaternion();

    // Compute local stiffness matrix:
    ComputeStiffnessMatrix();
}

void ChElementBeamEuler::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 12) && (H.cols() == 12));
    assert(section);

    //
    // The K stiffness matrix and R damping matrix of this element:
    //

    if (Kfactor || Rfactor) {


        // Corotational K stiffness:
        ChMatrixDynamic<> CK(12, 12);
        ChMatrixDynamic<> CKCt(12, 12);  // the global, corotated, K matrix

        if (!disable_projector) {
            //
            // Corotational approach as in G.Felippa
            //

            // compute [H(theta)]'[K_loc] [H(theta]

            ChVectorDynamic<> displ(12);
            GetStateBlock(displ);

            ChVector<> VthetaA(displ(3), displ(4), displ(5));
            ChVector<> VthetaB(displ(9), displ(10), displ(11));
            ChStarMatrix33<> MthetaA(VthetaA);
            ChStarMatrix33<> MthetaB(VthetaB);
            double thetaA = VthetaA.Length();
            double thetaB = VthetaB.Length();

            double zetaA;
            double zetaB;
            if (fabs(thetaA) > 0.05)
                zetaA = (1.0 - 0.5 * thetaA * (1.0 / tan(0.5 * thetaA))) / (pow(thetaA, 2));
            else
                zetaA = 1.0 / 12.0 + pow(thetaA, 2) / 720.0 + pow(thetaA, 4) / 30240.0 + pow(thetaA, 6) / 1209600.0;
            if (fabs(thetaB) > 0.05)
                zetaB = (1.0 - 0.5 * thetaB * (1.0 / tan(0.5 * thetaB))) / (pow(thetaB, 2));
            else
                zetaB = 1.0 / 12.0 + pow(thetaB, 2) / 720.0 + pow(thetaB, 4) / 30240.0 + pow(thetaB, 6) / 1209600.0;

            ChMatrix33<> mI(1);
            ChMatrix33<> LambdaA = mI - MthetaA * 0.5 + (MthetaA * MthetaA) * zetaA;
            ChMatrix33<> LambdaB = mI - MthetaB * 0.5 + (MthetaB * MthetaB) * zetaB;
            LambdaA.transposeInPlace();
            LambdaB.transposeInPlace();

            ChMatrixDynamic<> HtKH(12, 12);
            std::vector<ChMatrix33<>*> Ht;
            Ht.push_back(&mI);
            Ht.push_back(&LambdaA);
            Ht.push_back(&mI);
            Ht.push_back(&LambdaB);

            ChMatrixCorotation::ComputeCK(this->StiffnessMatrix, Ht, 4, CK);  // CK = [H(theta)]'[K_loc]
            ChMatrixCorotation::ComputeKCt(CK, Ht, 4, HtKH);                  // HtKH = [H(theta)]'[K_loc] [H(theta)]

            // compute K_m, K_gr, K_gm, K_gp

            ChVector<> vC = 0.5 * (nodes[0]->Frame().GetPos() + nodes[1]->Frame().GetPos());  // centroid
            ChVector<> vX_a_loc = this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos() - vC);
            ChVector<> vX_b_loc = this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos() - vC);
            double Lel = (nodes[0]->Frame().GetPos() - nodes[1]->Frame().GetPos()).Length();

            ChMatrixDynamic<> mS(12, 3);  // [S] = [ -skew[X_a_loc];  [I];  -skew[X_b_loc];  [I] ]
            mS.block(0, 0, 3, 3) = ChStarMatrix33<>(-vX_a_loc);
            mS.block(3, 0, 3, 3) = mI;
            mS.block(6, 0, 3, 3) = ChStarMatrix33<>(-vX_b_loc);
            mS.block(9, 0, 3, 3) = mI;

            ChMatrixDynamic<> mG(3, 12);  // [G] = [dw_frame/du_a; dw_frame/dw_a; dw_frame/du_b; dw_frame/dw_b]
            mG.setZero();
            mG(2, 1) = -1. / Lel;
            mG(1, 2) = 1. / Lel;
            mG(2, 7) = 1. / Lel;
            mG(1, 8) = -1. / Lel;
            mG(0, 4) = 0.5;
            mG(0, 10) = 0.5;

            ChMatrixDynamic<> mP = ChMatrixDynamic<>::Identity(12, 12) - mS * mG;  // [P] = [I]-[S][G]
            ChVectorDynamic<> f_local = StiffnessMatrix * displ;                   // f_loc = [K_loc]*u_loc

            ChVectorDynamic<> f_h(12);  // f_h = [H(theta)]' [K_loc]*u_loc
            f_h.segment(0, 3) = f_local.segment(0, 3);
            f_h.segment(3, 3) = LambdaA * f_local.segment(3, 3);
            f_h.segment(6, 3) = f_local.segment(6, 3);
            f_h.segment(9, 3) = LambdaB * f_local.segment(9, 3);

            ChVectorDynamic<> f_p = mP.transpose() * f_h;  // f_p = [P]' [H(theta)]' [K_loc]*u_loc

            ChMatrixDynamic<> mFnm(12, 3);
            ChMatrixDynamic<> mFn(12, 3);

            if (!force_symmetric_stiffness) {
                {
                    ChStarMatrix33<> skew_f(f_p.segment(0, 3));
                    mFnm.block(0, 0, 3, 3) = skew_f;
                    mFn.block(0, 0, 3, 3) = skew_f;
                }
                {
                    mFnm.block(3, 0, 3, 3) = ChStarMatrix33<>(f_p.segment(3, 3));
                    mFn.block(3, 0, 3, 3) = ChMatrix33<>::Zero();
                }
                {
                    ChStarMatrix33<> skew_f(f_p.segment(6, 3));
                    mFnm.block(6, 0, 3, 3) = skew_f;
                    mFn.block(6, 0, 3, 3) = skew_f;
                }
                {
                    mFnm.block(9, 0, 3, 3) = ChStarMatrix33<>(f_p.segment(9, 3));
                    mFn.block(9, 0, 3, 3) = ChMatrix33<>::Zero();
                }
            }
            else {
                mFnm.block(0, 0, 3, 3) = ChStarMatrix33<>(f_p.segment(0, 3));
                mFnm.block(3, 0, 3, 3) = ChStarMatrix33<>(f_p.segment(3, 3) * 0.5);
                mFnm.block(6, 0, 3, 3) = ChStarMatrix33<>(f_p.segment(6, 3));
                mFnm.block(9, 0, 3, 3) = ChStarMatrix33<>(f_p.segment(9, 3) * 0.5);
                mFn = mFnm;
            }

            ChMatrixDynamic<> K_m = mP.transpose() * HtKH * mP;  // [K_m]  = [P]' [H(theta)]'[K_loc] [H(theta] [P]
            ChMatrixDynamic<> K_gr = mFnm * mG;                  // [K_gr] = [F_nm][G]
            ChMatrixDynamic<> K_gp = mG.transpose() * mFn.transpose() * mP;  // [K_gp] = [G]'[F_n]'[P] = ([F_n][G])'[P]

            // ...							// [K_gm] = [P]'[L][P]  (simplify: avoid computing this)

            ChMatrixDynamic<> K_tang = K_m - K_gr - K_gp;  // [K_tang] = [K_m] - [K_gr] - [K_gp] + [K_gm]

            // finally, do :   [K_tang_global] = [R][K_tang][R]'
            ChMatrix33<> Atoabs(this->q_element_abs_rot);
            ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            std::vector<ChMatrix33<>*> R;
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelA);
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelB);

            ChMatrixCorotation::ComputeCK(K_tang, R, 4, CK);
            ChMatrixCorotation::ComputeKCt(CK, R, 4, CKCt);
        }
        else {
            //
            // Simplified (inexact, faster) corotational approach
            //

            ChMatrix33<> Atoabs(this->q_element_abs_rot);
            ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            std::vector<ChMatrix33<>*> R;
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelA);
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelB);

            ChMatrixCorotation::ComputeCK(this->StiffnessMatrix, R, 4, CK);
            ChMatrixCorotation::ComputeKCt(CK, R, 4, CKCt);
        }

        // For strict symmetry, copy L=U because the computations above might
        // lead to small errors because of numerical roundoff even with force_symmetric_stiffness
        if (force_symmetric_stiffness) {
            for (int row = 0; row < CKCt.rows() - 1; ++row)
                for (int col = row + 1; col < CKCt.cols(); ++col)
                    CKCt(row, col) = CKCt(col, row);
        }

        //// RADU
        //// Check if the above can be done with the one-liner:
        ////CKCt.triangularView<Eigen::Upper>() = CKCt.transpose();

        // For K stiffness matrix and R matrix: scale by factors
        CKCt *= Kfactor + Rfactor * this->section->GetBeamRaleyghDamping();

        H.block(0, 0, 12, 12) = CKCt;  // because [R] = r*[K] , so kf*[K]+rf*[R] = (kf+rf*r)*[K]

    } 
    else
		H.setZero();

    //
    // The M mass matrix of this element:  
    //

    if (Mfactor) {

        ChMatrixDynamic<> Mloc(12, 12);
        Mloc.setZero();
        ChMatrix33<> Mxw;

        //
        // "lumped" M mass matrix
        //
        ChMatrixNM<double, 6, 6> sectional_mass;
        this->section->ComputeInertiaMatrix(sectional_mass);

        double node_multiplier_fact = 0.5 * length * Mfactor;
        for (int i = 0; i < nodes.size(); ++i) {
            int stride = i * 6;
            // if there is no mass center offset, the upper right and lower left blocks need not be rotated,
            // hence it can be the simple (constant) expression
            //   Mloc.block<6, 6>(stride, stride) += sectional_mass * node_multiplier_fact;
            // but the more general case needs the rotations, hence:
            Mloc.block<3, 3>(stride,   stride  ) += sectional_mass.block<3, 3>(0,0) * node_multiplier_fact;
            Mloc.block<3, 3>(stride+3, stride+3) += sectional_mass.block<3, 3>(3,3) * node_multiplier_fact;
            Mxw = nodes[i]->GetA() * sectional_mass.block<3, 3>(0,3) * node_multiplier_fact;
            Mloc.block<3, 3>(stride,   stride+3) += Mxw;
            Mloc.block<3, 3>(stride+3, stride)   += Mxw.transpose();
        }
        
        /* old
        double lmass = mass * 0.5;
        double lineryz = (1. / 50.) * mass * pow(length, 2);  // note: 1/50 can be even less (this is 0 in many texts)
        double linerx = (1. / 2.) * length * this->section->GetInertiaJxxPerUnitLength(); // for constant density would be (1. / 2.) * length * section->GetDensity() * (section->GetIyy() + section->GetIzz());  

        Mloc(0, 0) += Mfactor * lmass;  // node A x,y,z
        Mloc(1, 1) += Mfactor * lmass;
        Mloc(2, 2) += Mfactor * lmass;

        Mloc(6, 6) += Mfactor * lmass;  // node B x,y,z
        Mloc(7, 7) += Mfactor * lmass;
        Mloc(8, 8) += Mfactor * lmass;

        Mloc(3, 3) += Mfactor * linerx;  // node A Ixx,Iyy,Izz  
        Mloc(4, 4) += Mfactor * lineryz;
        Mloc(5, 5) += Mfactor * lineryz;

        Mloc(9, 9) += Mfactor * linerx;  // node B Ixx,Iyy,Izz 
        Mloc(10, 10) += Mfactor * lineryz;
        Mloc(11, 11) += Mfactor * lineryz;
        */
        /* The following would be needed if consistent mass matrix is used, but...
        ChMatrix33<> Atoabs(this->q_element_abs_rot);
        ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
        ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
        std::vector< ChMatrix33<>* > R;
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelA);
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelB);

        ChMatrixCorotation::ComputeCK(Mloc, R, 4, CK);
        ChMatrixCorotation::ComputeKCt(CK, R, 4, CKCt);

        H.block(0,0,12,12) += CKCt;
        */

        // ..rather do this because lumped mass matrix does not need rotation transf.
        H.block(0, 0, 12, 12) += Mloc;

        //***TO DO*** better per-node lumping, or 4x4 consistent mass matrices, maybe with integration if not uniform
        // materials.
    }

}

void ChElementBeamEuler::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 12);
    assert(section);

    // set up vector of nodal displacements and small rotations (in local element system)
    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);

    // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
    ChVectorDynamic<> FiK_local = StiffnessMatrix * displ;

    // set up vector of nodal velocities (in local element system)
    ChVectorDynamic<> displ_dt(12);
    this->GetField_dt(displ_dt);

    ChMatrixDynamic<> FiR_local = section->GetBeamRaleyghDamping() * StiffnessMatrix * displ_dt;

    FiK_local += FiR_local;
    FiK_local *= -1.0;

    if (!disable_projector) {
        //
        // Corotational approach as in G.Felippa
        //

        ChVectorDynamic<> displ(12);
        GetStateBlock(displ);

        ChVector<> VthetaA(displ(3), displ(4), displ(5));
        ChVector<> VthetaB(displ(9), displ(10), displ(11));
        ChStarMatrix33<> MthetaA(VthetaA);
        ChStarMatrix33<> MthetaB(VthetaB);
        double thetaA = VthetaA.Length();
        double thetaB = VthetaB.Length();

        double zetaA;
        double zetaB;
        if (fabs(thetaA) > 0.05)
            zetaA = (1.0 - 0.5 * thetaA * (1.0 / tan(0.5 * thetaA))) / (pow(thetaA, 2));
        else
            zetaA = 1.0 / 12.0 + pow(thetaA, 2) / 720.0 + pow(thetaA, 4) / 30240.0 + pow(thetaA, 6) / 1209600.0;
        if (fabs(thetaB) > 0.05)
            zetaB = (1.0 - 0.5 * thetaB * (1.0 / tan(0.5 * thetaB))) / (pow(thetaB, 2));
        else
            zetaB = 1.0 / 12.0 + pow(thetaB, 2) / 720.0 + pow(thetaB, 4) / 30240.0 + pow(thetaB, 6) / 1209600.0;

        ChMatrix33<> mI(1);
        ChMatrix33<> LambdaA = mI - MthetaA * 0.5 + (MthetaA * MthetaA) * zetaA;
        ChMatrix33<> LambdaB = mI - MthetaB * 0.5 + (MthetaB * MthetaB) * zetaB;
        LambdaA.transposeInPlace();
        LambdaB.transposeInPlace();

        ChVector<> vC = 0.5 * (nodes[0]->Frame().GetPos() + nodes[1]->Frame().GetPos());  // centroid
        ChVector<> vX_a_loc = this->q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos() - vC);
        ChVector<> vX_b_loc = this->q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos() - vC);
        double Lel = (nodes[0]->Frame().GetPos() - nodes[1]->Frame().GetPos()).Length();

        ChMatrixDynamic<> mS(12, 3);  // [S] = [ -skew[X_a_loc];  [I];  -skew[X_b_loc];  [I] ]
        mS.block(0, 0, 3, 3) = ChStarMatrix33<>(-vX_a_loc);
        mS.block(3, 0, 3, 3) = mI;
        mS.block(6, 0, 3, 3) = ChStarMatrix33<>(-vX_b_loc);
        mS.block(9, 0, 3, 3) = mI;

        ChMatrixDynamic<> mG(3, 12);  // [G] = [dw_frame/du_a; dw_frame/dw_a; dw_frame/du_b; dw_frame/dw_b]
        mG.setZero();
        mG(2, 1) = -1. / Lel;
        mG(1, 2) = 1. / Lel;
        mG(2, 7) = 1. / Lel;
        mG(1, 8) = -1. / Lel;
        mG(0, 4) = 0.5;
        mG(0, 10) = 0.5;

        ChMatrixDynamic<> mP = ChMatrixDynamic<>::Identity(12,12) - mS * mG;  // [P] = [I]-[S][G]

        ChVectorDynamic<> HF(12);  //  HF =  [H(theta)]' F_local
        HF.segment(0, 3) = FiK_local.segment(0, 3);
        HF.segment(3, 3) = LambdaA * FiK_local.segment(3, 3);
        HF.segment(6, 3) = FiK_local.segment(6, 3);
        HF.segment(9, 3) = LambdaB * FiK_local.segment(9, 3);

        ChVectorDynamic<> PHF = mP.transpose() * HF;

        ChMatrix33<> Atoabs(this->q_element_abs_rot);
        ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
        ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
        std::vector<ChMatrix33<>*> R;
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelA);
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelB);
        ChMatrixCorotation::ComputeCK(PHF, R, 4, Fi);
    } else {
        //
        // Simplified (inexact, faster) corotational approach
        //

        // Fi = C * Fi_local  with C block-diagonal rotations A  , for nodal forces in abs. frame
        ChMatrix33<> Atoabs(this->q_element_abs_rot);
        ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
        ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
        std::vector<ChMatrix33<>*> R;
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelA);
        R.push_back(&Atoabs);
        R.push_back(&AtolocwelB);
        ChMatrixCorotation::ComputeCK(FiK_local, R, 4, Fi);
    }


    // Add also inertial quadratic terms: gyroscopic and centrifugal
    
    // CASE OF LUMPED MASS - fast
    double node_multiplier = 0.5 * length;
    ChVector<> mFcent_i;
    ChVector<> mTgyro_i;
    for (int i = 0; i < nodes.size(); ++i) {
        int stride = i * 6;
        this->section->ComputeQuadraticTerms(mFcent_i, mTgyro_i, nodes[i]->GetWvel_loc());
        ChQuaternion<> q_i(nodes[i]->GetRot());
        Fi.segment(i * 6, 3)     -= node_multiplier * (nodes[i]->GetA() * mFcent_i).eigen();
        Fi.segment(3 + i * 6, 3) -= node_multiplier * mTgyro_i.eigen();
    }

#ifdef BEAM_VERBOSE
    GetLog() << "\nInternal forces (local): \n";
    for (int c = 0; c < 6; c++)
        GetLog() << FiK_local(c) << "  ";
    GetLog() << "\n";
    for (int c = 6; c < 12; c++)
        GetLog() << FiK_local(c) << "  ";
    GetLog() << "\n\nInternal forces (ABS) : \n";
    for (int c = 0; c < 6; c++)
        GetLog() << Fi(c) << "  ";
    GetLog() << "\n";
    for (int c = 6; c < 12; c++)
        GetLog() << Fi(c) << "  ";
    GetLog() << "\n";
#endif
}



void ChElementBeamEuler::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {
    
    // no so efficient... a temporary mass matrix here:
    ChMatrixDynamic<> mM(12, 12);
    this->ComputeMmatrixGlobal(mM);

    // a vector of G accelerations for the two nodes (for translation degrees of freedom)
    ChVectorDynamic<> mG(12);
    mG.setZero();
    mG.segment(0, 3) = G_acc.eigen();
    mG.segment(6, 3) = G_acc.eigen();

    // Gravity forces as M*g, always works, regardless of the way M 
    // is computed (lumped or consistent, with offset center of mass or centered, etc.)
    // [Maybe one can replace this function with a faster ad-hoc implementation in case of lumped masses.]
    Fg = mM * mG;

    //***TO DO*** for the lumped mass matrix case, the mM * mG product can be unrolled into few multiplications as mM mostly zero, and same for mG
}


void ChElementBeamEuler::EvaluateSectionDisplacement(const double eta, ChVector<>& u_displ, ChVector<>& u_rotaz) {
    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);

    ShapeVector N;
    ShapeFunctions(N, eta);  // Evaluate compressed shape functions

    u_displ.x() = N(0) * displ(0) + N(3) * displ(6);     // x_a   x_b
    u_displ.y() = N(1) * displ(1) + N(4) * displ(7)      // y_a   y_b
                  + N(2) * displ(5) + N(5) * displ(11);  // Rz_a  Rz_b
    u_displ.z() = N(1) * displ(2) + N(4) * displ(8)      // z_a   z_b
                  - N(2) * displ(4) - N(5) * displ(10);  // Ry_a  Ry_b

    u_rotaz.x() = N(0) * displ(3) + N(3) * displ(9);    // Rx_a  Rx_b
    u_rotaz.y() = -N(6) * displ(2) - N(7) * displ(8) +  // z_a   z_b   note - sign
                  N(8) * displ(4) + N(9) * displ(10);   // Ry_a  Ry_b
    u_rotaz.z() = N(6) * displ(1) + N(7) * displ(7) +   // y_a   y_b
                  N(8) * displ(5) + N(9) * displ(11);   // Rz_a  Rz_b
}

void ChElementBeamEuler::EvaluateSectionFrame(const double eta, ChVector<>& point, ChQuaternion<>& rot) {
    ChVector<> u_displ;
    ChVector<> u_rotaz;
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);

    this->EvaluateSectionDisplacement(eta, u_displ, u_rotaz);

    // Since   d = [Atw]' Xt - [A0w]'X0   , so we have
    //        Xt = [Atw] (d +  [A0w]'X0)

    point = this->q_element_abs_rot.Rotate(u_displ +
                                           this->q_element_ref_rot.RotateBack(Nx1 * this->nodes[0]->GetX0().GetPos() +
                                                                              Nx2 * this->nodes[1]->GetX0().GetPos()));

    ChQuaternion<> msectionrot;
    msectionrot.Q_from_AngAxis(u_rotaz.Length(), u_rotaz.GetNormalized());
    rot = this->q_element_abs_rot % msectionrot;
}

void ChElementBeamEuler::EvaluateSectionForceTorque(const double eta, ChVector<>& Fforce, ChVector<>& Mtorque) {
    assert(section);

    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);

    double EA = this->section->GetAxialRigidity();
    double EIyy = this->section->GetYbendingRigidity();
    double EIzz = this->section->GetZbendingRigidity();
    double GJ = this->section->GetXtorsionRigidity();
    double alpha = this->section->GetSectionRotation();
    double Cy = this->section->GetCentroidY();
    double Cz = this->section->GetCentroidZ();
    double Sy = this->section->GetShearCenterY();
    double Sz = this->section->GetShearCenterZ();

    // ShapeVector N;
    // ShapeFunctions(N, eta);  // Evaluate compressed shape functions

    // shape function derivatives are computed here on-the-fly
    double dN_xa = -(1. / length);
    double dN_xb = (1. / length);
    double ddN_ua = (6. / (length * length)) * (eta);
    double ddN_ub = -(6. / (length * length)) * (eta);
    double ddN_ra = -(1. / length) + ((3.0 / length) * eta);
    double ddN_rb = (1. / length) + ((3.0 / length) * eta);
    double dddN_ua = (12. / (length * length * length));
    double dddN_ub = -(12. / (length * length * length));
    double dddN_ra = (6.0 / (length * length));
    double dddN_rb = (6.0 / (length * length));

    // generalized strains/curvatures;
    ChVectorN<double, 6> sect_ek;

    // e_x
    sect_ek(0) = (dN_xa * displ(0) + dN_xb * displ(6));  // x_a   x_b
    // e_y
    sect_ek(1) = (dddN_ua * displ(1) + dddN_ub * displ(7) +   // y_a   y_b
                  dddN_ra * displ(5) + dddN_rb * displ(11));  // Rz_a  Rz_b
    // e_z
    sect_ek(2) = (-dddN_ua * displ(2) - dddN_ub * displ(8) +  // z_a   z_b   note - sign
                  dddN_ra * displ(4) + dddN_rb * displ(10));  // Ry_a  Ry_b

    // k_x
    sect_ek(3) = (dN_xa * displ(3) + dN_xb * displ(9));  // Rx_a  Rx_b
    // k_y
    sect_ek(4) = (-ddN_ua * displ(2) - ddN_ub * displ(8) +  // z_a   z_b   note - sign
                  ddN_ra * displ(4) + ddN_rb * displ(10));  // Ry_a  Ry_b
    // k_z
    sect_ek(5) = (ddN_ua * displ(1) + ddN_ub * displ(7) +   // y_a   y_b
                  ddN_ra * displ(5) + ddN_rb * displ(11));  // Rz_a  Rz_b

    if (false)  // section->alpha ==0 && section->Cy ==0 && section->Cz==0 && section->Sy==0 && section->Sz==0)
    {
        // Fast computation:
        Fforce.x() = EA   * sect_ek(0);
        Fforce.y() = EIzz * sect_ek(1);
        Fforce.z() = EIyy * sect_ek(2);

        Mtorque.x() = GJ   * sect_ek(3);
        Mtorque.y() = EIyy * sect_ek(4);
        Mtorque.z() = EIzz * sect_ek(5);
    } else {
        // Generic computation, by rotating and translating the constitutive
        // matrix of the beam:
        double ca = cos(alpha);
        double sa = sin(alpha);
        double cb = cos(alpha);  // could be beta if shear custom axes
        double sb = sin(alpha);

        double Klaw_d0 = EA;
        double Klaw_d1 = EIzz;
        double Klaw_d2 = EIyy;
        double Klaw_d3 = GJ;
        double Klaw_d4 = EIyy;
        double Klaw_d5 = EIzz;

        // ..unrolled rotated constitutive matrix..
        ChMatrixNM<double, 6, 6> Klaw_r;
        Klaw_r.setZero();

        Klaw_r(0, 0) = Klaw_d0;
        Klaw_r(1, 1) = Klaw_d1 * cb * cb + Klaw_d2 * sb * sb;
        Klaw_r(2, 2) = Klaw_d1 * sb * sb + Klaw_d2 * cb * cb;
        Klaw_r(1, 2) = Klaw_d1 * cb * sb - Klaw_d2 * cb * sb;
        Klaw_r(2, 1) = Klaw_r(1, 2);
        Klaw_r(3, 3) = Klaw_d3;
        Klaw_r(4, 4) = Klaw_d4 * ca * ca + Klaw_d5 * sa * sa;
        Klaw_r(5, 5) = Klaw_d4 * sa * sa + Klaw_d5 * ca * ca;
        Klaw_r(4, 5) = Klaw_d4 * ca * sa - Klaw_d5 * ca * sa;
        Klaw_r(5, 4) = Klaw_r(4, 5);

        // ..also translate for Cy Cz
        for (int i = 0; i < 6; ++i)
            Klaw_r(i, 4) +=  Cz * Klaw_r(i, 0);
        for (int i = 0; i < 6; ++i)
            Klaw_r(i, 5) += -Cy * Klaw_r(i, 0);

        // ..also translate for Sy Sz
        for (int i = 0; i < 6; ++i)
            Klaw_r(i, 3) += - Sz * Klaw_r(i, 1) + Sy * Klaw_r(i, 2);

        // .. compute wrench = Klaw_l * sect_ek
        ChVectorN<double, 6> wrench = Klaw_r * sect_ek;
        Fforce = wrench.segment(0, 3);
        Mtorque = wrench.segment(3, 3);

        // Note: to be checked.
    }
}

void ChElementBeamEuler::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 4) = nodes[0]->GetRot().eigen();

    mD.segment(block_offset + 7, 3) = nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 10, 4) = nodes[1]->GetRot().eigen();
}

void ChElementBeamEuler::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = nodes[0]->GetWvel_loc().eigen();

    mD.segment(block_offset + 6, 3) = nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = nodes[1]->GetWvel_loc().eigen();
}

void ChElementBeamEuler::LoadableStateIncrement(const unsigned int off_x,
                                                ChState& x_new,
                                                const ChState& x,
                                                const unsigned int off_v,
                                                const ChStateDelta& Dv) {
    nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    nodes[1]->NodeIntStateIncrement(off_x + 7, x_new, x, off_v + 6, Dv);
}

void ChElementBeamEuler::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&this->nodes[0]->Variables());
    mvars.push_back(&this->nodes[1]->Variables());
}

void ChElementBeamEuler::ComputeNF(const double U,
                                   ChVectorDynamic<>& Qi,
                                   double& detJ,
                                   const ChVectorDynamic<>& F,
                                   ChVectorDynamic<>* state_x,
                                   ChVectorDynamic<>* state_w) {
    ShapeVector N;
    ShapeFunctions(N, U);  // evaluate shape functions (in compressed vector), btw. not dependant on state

    detJ = this->GetRestLength() / 2.0;

    Qi(0) = N(0) * F(0);
    Qi(1) = N(1) * F(1) + N(6) * F(5);
    Qi(2) = N(1) * F(2) - N(6) * F(4);
    Qi(3) = N(0) * F(3);
    Qi(4) = -N(2) * F(2) + N(8) * F(4);
    Qi(5) = N(2) * F(1) + N(8) * F(5);

    Qi(6) = N(3) * F(0);
    Qi(7) = N(4) * F(1) + N(7) * F(5);
    Qi(8) = N(4) * F(2) - N(7) * F(4);
    Qi(9) = N(3) * F(3);
    Qi(10) = -N(5) * F(2) + N(9) * F(4);
    Qi(11) = N(5) * F(1) + N(9) * F(5);
}

void ChElementBeamEuler::ComputeNF(const double U,
                                   const double V,
                                   const double W,
                                   ChVectorDynamic<>& Qi,
                                   double& detJ,
                                   const ChVectorDynamic<>& F,
                                   ChVectorDynamic<>* state_x,
                                   ChVectorDynamic<>* state_w) {
    this->ComputeNF(U, Qi, detJ, F, state_x, state_w);
    detJ /= 4.0;  // because volume
}

double ChElementBeamEuler::GetDensity() {
    return this->section->GetMassPerUnitLength();
}

}  // end namespace fea
}  // end namespace chrono
