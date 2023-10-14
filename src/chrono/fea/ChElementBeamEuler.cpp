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
      disable_corotate(false),
      force_symmetric_stiffness(false),
      use_geometric_stiffness(true)
{
    nodes.resize(2);

    Km.setZero(this->GetNdofs(), this->GetNdofs());
    Kg.setZero(this->GetNdofs(), this->GetNdofs());
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
        // Average the two Y directions to have midpoint torsion (ex -30?torsion A and +30?torsion B= 0?
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

// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass
void ChElementBeamEuler::ComputeNodalMass() {
    for (int i = 0; i < nodes.size(); ++i)
        nodes[i]->m_TotalMass += this->mass / nodes.size();
}

void ChElementBeamEuler::ComputeStiffnessMatrix() {
    assert(section);

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
        //// TEST REDDY BEAMS
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

    Km(0, 0) = k_u;
    Km(1, 1) = k_v;
    Km(2, 2) = k_w;
    Km(3, 3) = k_f;
    Km(4, 4) = k_t;
    Km(5, 5) = k_p;
    Km(6, 6) = k_u;
    Km(7, 7) = k_v;
    Km(8, 8) = k_w;
    Km(9, 9) = k_f;
    Km(10, 10) = k_t;
    Km(11, 11) = k_p;

    Km(0, 6) = -k_u;
    Km(1, 7) = -k_v;
    Km(2, 8) = -k_w;
    Km(3, 9) = -k_f;
    Km(4, 10) = k_tt;
    Km(5, 11) = k_pp;

    Km(4, 8) = k_wt;
    Km(5, 7) = -k_vp;
    Km(1, 11) = k_vp;
    Km(2, 10) = -k_wt;

    Km(1, 5) = k_vp;
    Km(2, 4) = -k_wt;
    Km(7, 11) = -k_vp;
    Km(8, 10) = k_wt;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = r + 1; c < 12; c++)
            Km(c, r) = Km(r, c);

    // In case the section is rotated:
    if (this->section->GetSectionRotation()) {
        // Do [K]^ = [R][K][R]'
        ChMatrix33<> Rotsect;
        Rotsect.Set_A_Rxyz(ChVector<>(-section->GetSectionRotation(), 0, 0));
        ChMatrixDynamic<> CKtemp(12, 12);
        CKtemp.setZero();
        ChMatrixCorotation::ComputeCK(this->Km, Rotsect, 4, CKtemp);
        ChMatrixCorotation::ComputeKCt(CKtemp, Rotsect, 4, this->Km);
    }
    // In case the section has a centroid displacement:

    if (Cy || Cz) {
        // Do [K]" = [T_c][K]^[T_c]'

        for (int i = 0; i < 12; ++i)
            this->Km(4, i) += Cz * this->Km(0, i);
        for (int i = 0; i < 12; ++i)
            this->Km(5, i) += -Cy * this->Km(0, i);

        for (int i = 0; i < 12; ++i)
            this->Km(10, i) += Cz * this->Km(6, i);
        for (int i = 0; i < 12; ++i)
            this->Km(11, i) += -Cy * this->Km(6, i);

        for (int i = 0; i < 12; ++i)
            this->Km(i, 4) += Cz * this->Km(i, 0);
        for (int i = 0; i < 12; ++i)
            this->Km(i, 5) += -Cy * this->Km(i, 0);

        for (int i = 0; i < 12; ++i)
            this->Km(i, 10) += Cz * this->Km(i, 6);
        for (int i = 0; i < 12; ++i)
            this->Km(i, 11) += -Cy * this->Km(i, 6);
    }

    // In case the section has a shear center displacement:
    if (Sy || Sz) {
        // Do [K]?= [T_s][K]"[T_s]'

        for (int i = 0; i < 12; ++i)
            this->Km(3, i) +=
                - Sz * this->Km(1, i) + Sy * this->Km(2, i);
        for (int i = 0; i < 12; ++i)
            this->Km(9, i) +=
                - Sz * this->Km(7, i) + Sy * this->Km(8, i);

        for (int i = 0; i < 12; ++i)
            this->Km(i, 3) +=
                - Sz * this->Km(i, 1) + Sy * this->Km(i, 2);
        for (int i = 0; i < 12; ++i)
            this->Km(i, 9) +=
                - Sz * this->Km(i, 7) + Sy * this->Km(i, 8);
    }
}


void ChElementBeamEuler::ComputeGeometricStiffnessMatrix() {
    assert(section);
    
    // Compute the local geometric stiffness matrix Kg without the P multiplication term, that is Kg*(1/P), 
    // so that it is a constant matrix for performance reasons.
    // We used the analytical values from 
    //   [1] "Nonlinear finite element analysis of elastic frames", Kuo Mo, Hsiao Fang, Yu Hou
    //        Computers & Structures Volume 26, Issue 4, 1987, Pages 693-701
    // For the Reddy or timoshenko more detailed case with higher order terms, look also to:
    //   [2] "A Unified Approach to the Timoshenko Geometric Stiffness Matrix Considering Higher-Order Terms in the Strain Tensor"
    //        https://www.scielo.br/pdf/lajss/v16n4/1679-7825-lajss-16-04-e185.pdf
    // Look also at: https://enercalc.com/3d_help/toc161394033.html or in Plesha, Malkus, Cook �Concepts and Applications of Finite Element Analysis? 
    // or in W. McGuire & R.H. Gallagher & R.D. Ziemian, �Matrix Structural Analysis?
      
    //double EA = this->section->GetAxialRigidity();
    //double EIyy = this->section->GetYbendingRigidity();
    //double EIzz = this->section->GetZbendingRigidity();

    double L = this->length;

    double P6_5L_y  = 6. / (5.*L); // optional [2]: ...+ 12*IzA /(L*L*L);
    double P6_5L_z  = 6. / (5.*L); // optional [2]: ...+ 12*IyA /(L*L*L);
    double P_10_y   = 1. / (10.);  // optional [2]: ...+ 6*IzA /(L*L);
    double P_10_z   = 1. / (10.);  // optional [2]: ...+ 6*IyA /(L*L);
    double PL2_15_y = 2.*L / (15.);// optional [2]: ...+ 4*IzA /(L);
    double PL2_15_z = 2.*L / (15.);// optional [2]: ...+ 4*IyA /(L);
    double PL_30_y  = L / (30.);   // optional [2]: ...+ 2*IyA /(L);
    double PL_30_z  = L / (30.);   // optional [2]: ...+ 2*IyA /(L);
   /* 
    this->Kg(0, 0) =   1./L;
    this->Kg(6, 6) =   1./L;
    this->Kg(0, 6) = - 1./L;
    */
    
    this->Kg(1, 1) =  P6_5L_y;
    this->Kg(1, 5) =  P_10_y;
    this->Kg(1, 7) = -P6_5L_y;
    this->Kg(1,11) =  P_10_y;

    this->Kg(2, 2) =  P6_5L_z;
    this->Kg(2, 4) = -P_10_z;
    this->Kg(2, 8) = -P6_5L_z;
    this->Kg(2,10) = -P_10_z;

    this->Kg(4, 4) =  PL2_15_y;
    this->Kg(4, 8) =  P_10_y;
    this->Kg(4,10) = -PL_30_y;

    this->Kg(5, 5) =  PL2_15_z;
    this->Kg(5, 7) = -P_10_z;
    this->Kg(5,11) = -PL_30_z;

    this->Kg(7, 7) =  P6_5L_y;
    this->Kg(7,11) = -P_10_y;

    this->Kg(8, 8) =  P6_5L_z;
    this->Kg(8,10) =  P_10_y;

    this->Kg(10, 10) = PL2_15_y;

    this->Kg(11, 11) = PL2_15_z;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = r + 1; c < 12; c++)
            Kg(c, r) = Kg(r, c);



    double Cy = this->section->GetCentroidY();
    double Cz = this->section->GetCentroidZ();
    double Sy = this->section->GetShearCenterY();
    double Sz = this->section->GetShearCenterZ();

    // In case the section is rotated:
    if (this->section->GetSectionRotation()) {
        // Do [K]^ = [R][K][R]'
        ChMatrix33<> Rotsect;
        Rotsect.Set_A_Rxyz(ChVector<>(-section->GetSectionRotation(), 0, 0));
        ChMatrixDynamic<> CKtemp(12, 12);
        CKtemp.setZero();
        ChMatrixCorotation::ComputeCK(this->Kg, Rotsect, 4, CKtemp);
        ChMatrixCorotation::ComputeKCt(CKtemp, Rotsect, 4, this->Kg);
    }
    // In case the section has a centroid displacement:

    if (Cy || Cz) {
        // Do [K]" = [T_c][K]^[T_c]'

        for (int i = 0; i < 12; ++i)
            this->Kg(4, i) += Cz * this->Kg(0, i);
        for (int i = 0; i < 12; ++i)
            this->Kg(5, i) += -Cy * this->Kg(0, i);

        for (int i = 0; i < 12; ++i)
            this->Kg(10, i) += Cz * this->Kg(6, i);
        for (int i = 0; i < 12; ++i)
            this->Kg(11, i) += -Cy * this->Kg(6, i);

        for (int i = 0; i < 12; ++i)
            this->Kg(i, 4) += Cz * this->Kg(i, 0);
        for (int i = 0; i < 12; ++i)
            this->Kg(i, 5) += -Cy * this->Kg(i, 0);

        for (int i = 0; i < 12; ++i)
            this->Kg(i, 10) += Cz * this->Kg(i, 6);
        for (int i = 0; i < 12; ++i)
            this->Kg(i, 11) += -Cy * this->Kg(i, 6);
    }

    // In case the section has a shear center displacement:
    if (Sy || Sz) {
        // Do [K]?= [T_s][K]"[T_s]'

        for (int i = 0; i < 12; ++i)
            this->Kg(3, i) +=
                - Sz * this->Kg(1, i) + Sy * this->Kg(2, i);
        for (int i = 0; i < 12; ++i)
            this->Kg(9, i) +=
                - Sz * this->Kg(7, i) + Sy * this->Kg(8, i);

        for (int i = 0; i < 12; ++i)
            this->Kg(i, 3) +=
                - Sz * this->Kg(i, 1) + Sy * this->Kg(i, 2);
        for (int i = 0; i < 12; ++i)
            this->Kg(i, 9) +=
                - Sz * this->Kg(i, 7) + Sy * this->Kg(i, 8);
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
    ChVector<> myele =
        (nodes[0]->GetX0().GetA().Get_A_Yaxis() + nodes[1]->GetX0().GetA().Get_A_Yaxis()).GetNormalized();
    A0.Set_A_Xdir(mXele, myele);
    q_element_ref_rot = A0.Get_A_quaternion();

    // Compute local stiffness matrix:
    ComputeStiffnessMatrix();

    // Compute local geometric stiffness matrix normalized by pull force P: Kg/P
    ComputeGeometricStiffnessMatrix();
}

void ChElementBeamEuler::ComputeKRMmatricesGlobal(ChMatrixRef H, double Kfactor, double Rfactor, double Mfactor) {
    assert((H.rows() == 12) && (H.cols() == 12));
    assert(section);

    //
    // The K stiffness matrix and R damping matrix of this element:
    //

    if (Kfactor || Rfactor) {

        if (use_numerical_diff_for_KR) {

            // numerical evaluation of the K R  matrices
            double delta_p = 1e-5;
            double delta_r = 1e-3;

            ChVectorDynamic<> Fi0(12);
            ChVectorDynamic<> FiD(12);
            this->ComputeInternalForces(Fi0);
            ChMatrixDynamic<> H_num(12, 12);

            // K
            ChVector<>     pa0 = this->GetNodeA()->GetPos();
            ChQuaternion<> qa0 = this->GetNodeA()->GetRot();
            ChVector<>     pb0 = this->GetNodeB()->GetPos();
            ChQuaternion<> qb0 = this->GetNodeB()->GetRot();
            for (int i = 0; i < 3; ++i) {
                ChVector<> paD = pa0; 
                paD[i] += delta_p;
                this->GetNodeA()->SetPos(paD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i) = (FiD - Fi0) / delta_p;
                this->GetNodeA()->SetPos(pa0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector<> rotator(VNULL);  rotator[i] = delta_r;
                ChQuaternion<> mdeltarotL;  mdeltarotL.Q_from_Rotv(rotator); // rot.in local basis - as in system wide vectors
                ChQuaternion<> qaD = qa0 * mdeltarotL;
                this->GetNodeA()->SetRot(qaD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+3) = (FiD - Fi0) / delta_r;
                this->GetNodeA()->SetRot(qa0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector<> pbD = pb0; 
                pbD[i] += delta_p;
                this->GetNodeB()->SetPos(pbD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+6) = (FiD - Fi0) / delta_p;
                this->GetNodeB()->SetPos(pb0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector<> rotator(VNULL);  rotator[i] = delta_r;
                ChQuaternion<> mdeltarotL;  mdeltarotL.Q_from_Rotv(rotator); // rot.in local basis - as in system wide vectors
                ChQuaternion<> qbD = qb0 * mdeltarotL;
                this->GetNodeB()->SetRot(qbD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+9) = (FiD - Fi0) / delta_r;
                this->GetNodeB()->SetRot(qb0);
            }
            H.block(0, 0, 12, 12) = - H_num * Kfactor;

            // R
            ChVector<> va0 = this->GetNodeA()->GetPos_dt();
            ChVector<> wa0 = this->GetNodeA()->GetWvel_loc();
            ChVector<> vb0 = this->GetNodeB()->GetPos_dt();
            ChVector<> wb0 = this->GetNodeB()->GetWvel_loc();
            for (int i = 0; i < 3; ++i) {
                ChVector<> vaD = va0; 
                vaD[i] += delta_p;
                this->GetNodeA()->SetPos_dt(vaD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i) = (FiD - Fi0) / delta_p;
                this->GetNodeA()->SetPos_dt(va0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector<> waD = wa0; 
                waD[i] += delta_r;
                this->GetNodeA()->SetWvel_loc(waD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+3) = (FiD - Fi0) / delta_r;
                this->GetNodeA()->SetWvel_loc(wa0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector<> vbD = vb0; 
                vbD[i] += delta_p;
                this->GetNodeB()->SetPos_dt(vbD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+6) = (FiD - Fi0) / delta_p;
                this->GetNodeB()->SetPos_dt(vb0);
            }
            for (int i = 0; i < 3; ++i) {
                ChVector<> wbD = wb0; 
                wbD[i] += delta_r;
                this->GetNodeB()->SetWvel_loc(wbD);
                this->ComputeInternalForces(FiD);
                H_num.block<12,1>(0, i+9) = (FiD - Fi0) / delta_r;
                this->GetNodeB()->SetWvel_loc(wb0);
            }
            H.block(0, 0, 12, 12) += - H_num * Rfactor;

        }
        else {

            // Corotational K stiffness:
            ChMatrixDynamic<> CK(12, 12);
            ChMatrixDynamic<> CKCt(12, 12);  // the global, corotated, K matrix


            ChMatrixDynamic<> H_local;

            //
            // Corotate local stiffness matrix
            //

            ChMatrix33<> Atoabs(this->q_element_abs_rot);
            ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            std::vector<ChMatrix33<>*> R;
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelA);
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelB);

            if (this->use_geometric_stiffness) {
                // K = Km+Kg

                // For Kg, compute Px tension of the beam along centerline, using temporary but fast data structures:
                ChVectorDynamic<> displ(this->GetNdofs());
                this->GetStateBlock(displ);
                double Px = -this->Km.row(0) * displ;

                // Rayleigh damping (stiffness proportional part)  [R] = beta*[Km] , so H = kf*[Km+Kg]+rf*[R] = (kf+rf*beta)*[Km] + kf*Kg
                H_local = this->Km * (Kfactor + Rfactor * this->section->GetBeamRaleyghDampingBeta()) + this->Kg * Px * Kfactor;
            }
            else {
                // K = Km

                // Rayleigh damping (stiffness proportional part)  [R] = beta*[Km] , so H = kf*[Km]+rf*[R] = (kf+rf*beta)*[K]
                H_local = this->Km * (Kfactor + Rfactor * this->section->GetBeamRaleyghDampingBeta());
            }

            ChMatrixCorotation::ComputeCK(H_local, R, 4, CK);
            ChMatrixCorotation::ComputeKCt(CK, R, 4, CKCt);

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

            H.block(0, 0, 12, 12) = CKCt;

            // Add inertial stiffness matrix and inertial damping matrix (gyroscopic damping), 
            // if enabled in section material.
            // These matrices are not symmetric. Also note
            if (this->section->compute_inertia_damping_matrix || this->section->compute_inertia_stiffness_matrix) {
                ChMatrixNM<double, 6, 6> matr_loc;
                ChMatrixNM<double, 6, 6> KRi_loc;
                KRi_loc.setZero();
                // A lumped version of the inertial damping/stiffness matrix computation is used here, on a per-node basis:
                double node_multiplier_fact_R = 0.5 * length * Rfactor;
                double node_multiplier_fact_K = 0.5 * length * Kfactor;
                for (int i = 0; i < nodes.size(); ++i) {
                    int stride = i * 6;
                    if (this->section->compute_inertia_damping_matrix) {
                        this->section->ComputeInertiaDampingMatrix(matr_loc, nodes[i]->GetWvel_loc());
                        KRi_loc += matr_loc * node_multiplier_fact_R;
                    }
                    if (this->section->compute_inertia_stiffness_matrix) {
                        this->section->ComputeInertiaStiffnessMatrix(matr_loc, nodes[i]->GetWvel_loc(), nodes[i]->GetWacc_loc(), (nodes[i]->GetA().transpose()) * nodes[i]->GetPos_dtdt()); // assume x_dtdt in local frame!
                        KRi_loc += matr_loc * node_multiplier_fact_K;
                    }
                    // corotate the local damping and stiffness matrices (at once, already scaled) into absolute one
                    //H.block<3, 3>(stride,   stride  ) += nodes[i]->GetA() * KRi_loc.block<3, 3>(0,0) * (nodes[i]->GetA().transpose()); // NOTE: not needed as KRi_loc.block<3, 3>(0,0) is null by construction
                    H.block<3, 3>(stride + 3, stride + 3) += KRi_loc.block<3, 3>(3, 3);
                    H.block<3, 3>(stride, stride + 3) += nodes[i]->GetA() * KRi_loc.block<3, 3>(0, 3);
                    // H.block<3, 3>(stride+3, stride)   +=                    KRi_loc.block<3, 3>(3,0) * (nodes[i]->GetA().transpose()); // NOTE: not needed as KRi_loc.block<3, 3>(3,0) is null by construction
                }
            }
        }

    } 
    else
		H.setZero();

    //
    // The M mass matrix of this element:  
    //

    if (Mfactor || (Rfactor && this->section->GetBeamRaleyghDampingAlpha()) ) {

        ChMatrixDynamic<> Mloc(12, 12);
        Mloc.setZero();
        ChMatrix33<> Mxw;

        //
        // "lumped" M mass matrix
        //
        ChMatrixNM<double, 6, 6> sectional_mass;
        this->section->ComputeInertiaMatrix(sectional_mass);

        // Rayleigh damping (stiffness proportional part)  [Rm] = alpha*[M] , so H += km*[M]+rf*[Rm]  H += (km+rf*alpha)*[M]

        double node_multiplier_fact = 0.5 * length * (Mfactor + Rfactor * this->section->GetBeamRaleyghDampingAlpha());
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

        //// TODO better per-node lumping, or 4x4 consistent mass matrices, maybe with integration if not uniform
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
    ChVectorDynamic<> Fi_local = Km * displ;

    // set up vector of nodal velocities (in local element system)
    ChVectorDynamic<> displ_dt(12);
    this->GetField_dt(displ_dt);

    // Rayleigh damping - stiffness proportional
    ChMatrixDynamic<> FiR_local = section->GetBeamRaleyghDampingBeta() * Km * displ_dt;

    Fi_local += FiR_local;

    // Rayleigh damping - mass proportional
    if (this->section->GetBeamRaleyghDampingAlpha()) {
        ChMatrixDynamic<> Mloc(12, 12);
        Mloc.setZero();
        ChMatrix33<> Mxw;

        // the "lumped" M mass matrix must be computed
        ChMatrixNM<double, 6, 6> sectional_mass;
        this->section->ComputeInertiaMatrix(sectional_mass);

        // Rayleigh damping (stiffness proportional part)  [Rm] = alpha*[M] 
        double node_multiplier_fact = 0.5 * length * (this->section->GetBeamRaleyghDampingAlpha());
        for (int i = 0; i < nodes.size(); ++i) {
            int stride = i * 6; 
            Mloc.block<6, 6>(stride, stride) += sectional_mass * node_multiplier_fact;
        }
        FiR_local = Mloc * displ_dt;
        Fi_local += FiR_local;
    }

    Fi_local *= -1.0;

    //
    // Corotate local internal loads
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
    ChMatrixCorotation::ComputeCK(Fi_local, R, 4, Fi);


    // Add also inertial quadratic terms: gyroscopic and centrifugal
    
    // CASE OF LUMPED MASS - fast
    double node_multiplier = 0.5 * length;
    ChVector<> mFcent_i;
    ChVector<> mTgyro_i;
    for (int i = 0; i < nodes.size(); ++i) {
        this->section->ComputeQuadraticTerms(mFcent_i, mTgyro_i, nodes[i]->GetWvel_loc());
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

    //// TODO for the lumped mass matrix case, the mM * mG product can be unrolled into few multiplications as mM mostly zero, and same for mG
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

    double EA = this->section->GetAxialRigidity();
    double EIyy = this->section->GetYbendingRigidity();
    double EIzz = this->section->GetZbendingRigidity();
    double GJ = this->section->GetXtorsionRigidity();
    double alpha = this->section->GetSectionRotation();
    double Cy = this->section->GetCentroidY();
    double Cz = this->section->GetCentroidZ();
    double Sy = this->section->GetShearCenterY();
    double Sz = this->section->GetShearCenterZ();

    // The shear center offset is respect to the centerline.
    // In case the section has a shear center displacement:
    //Sy = Sy - Cy;   // Unnecessary to do this substraction
    //Sz = Sz - Cz;

    ChMatrix33<> Rotsect0;
    Rotsect0.Set_A_Rxyz(ChVector<>(alpha, 0, 0));
    ChMatrixNM<double, 6, 6> RotsectA;
    RotsectA.setZero();
    RotsectA.block<3, 3>(0, 0) = Rotsect0;
    RotsectA.block<3, 3>(3, 3) = Rotsect0;
    ChMatrixNM<double, 12, 12> Rotsect;
    Rotsect.setZero();
    Rotsect.block<3, 3>(0, 0) = Rotsect0;
    Rotsect.block<3, 3>(3, 3) = Rotsect0;
    Rotsect.block<3, 3>(6, 6) = Rotsect0;
    Rotsect.block<3, 3>(9, 9) = Rotsect0;

    ChMatrixNM<double, 6, 6> TcA;
    TcA.setIdentity();
    TcA(0, 4) = Cz;
    TcA(0, 5) = -Cy;
    TcA(1, 3) = -Cz;
    TcA(2, 3) = Cy;
    ChMatrixNM<double, 12, 12> Tc;
    Tc.setIdentity();
    Tc.block<6, 6>(0, 0) = TcA;
    Tc.block<6, 6>(6, 6) = TcA;

    ChMatrixNM<double, 6, 6> TsA;
    TsA.setIdentity();
    TsA(1, 3) = -Sz;
    TsA(2, 3) = Sy;
    ChMatrixNM<double, 12, 12> Ts;
    Ts.setIdentity();
    Ts.block<6, 6>(0, 0) = TsA;
    Ts.block<6, 6>(6, 6) = TsA;


    ChMatrixNM<double, 6, 6> TA;
    TA = RotsectA * TsA * TcA;
    ChMatrixNM<double, 12, 12> T;
    T = Rotsect * Ts * Tc;

    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);
    ChVectorDynamic<> displ_ec = T * displ;

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
    sect_ek(0) = (dN_xa * displ_ec(0) + dN_xb * displ_ec(6));  // x_a   x_b
    // e_y
    sect_ek(1) = (dddN_ua * displ_ec(1) + dddN_ub * displ_ec(7) +  // y_a   y_b
                  dddN_ra * displ_ec(5) + dddN_rb * displ_ec(11));  // Rz_a  Rz_b
    sect_ek(1) *= -1.0;
    // e_z
    sect_ek(2) = (-dddN_ua * displ_ec(2) - dddN_ub * displ_ec(8) +  // z_a   z_b   note - sign
                  dddN_ra * displ_ec(4) + dddN_rb * displ_ec(10));  // Ry_a  Ry_b
    
    // k_x
    sect_ek(3) = (dN_xa * displ_ec(3) + dN_xb * displ_ec(9));  // Rx_a  Rx_b
    // k_y
    sect_ek(4) = (-ddN_ua * displ_ec(2) - ddN_ub * displ_ec(8) +  // z_a   z_b   note - sign
                  ddN_ra * displ_ec(4) + ddN_rb * displ_ec(10));  // Ry_a  Ry_b
    // k_z
    sect_ek(5) = (ddN_ua * displ_ec(1) + ddN_ub * displ_ec(7) +  // y_a   y_b
                  ddN_ra * displ_ec(5) + ddN_rb * displ_ec(11));  // Rz_a  Rz_b

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
        ChMatrixNM<double, 6, 6> Klaw_d;
        Klaw_d.setZero();
        Klaw_d(0, 0) = EA;
        Klaw_d(1, 1) = EIzz;
        Klaw_d(2, 2) = EIyy;
        Klaw_d(3, 3) = GJ;
        Klaw_d(4, 4) = EIyy;
        Klaw_d(5, 5) = EIzz;

        // ..unrolled rotated constitutive matrix..
        ChMatrixNM<double, 6, 6> Klaw_r;
        Klaw_r.setZero();
        Klaw_r = TA.transpose() * Klaw_d;

        // .. compute wrench = Klaw_l * sect_ek
        ChVectorN<double, 6> wrench = Klaw_r * sect_ek;
        Fforce = wrench.segment(0, 3);
        Mtorque = wrench.segment(3, 3);

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
