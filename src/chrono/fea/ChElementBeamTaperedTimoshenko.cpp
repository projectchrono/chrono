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

#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"

namespace chrono {
namespace fea {

ChElementBeamTaperedTimoshenko::ChElementBeamTaperedTimoshenko()
    : q_refrotA(QUNIT),
      q_refrotB(QUNIT),
      q_element_abs_rot(QUNIT),
      q_element_ref_rot(QUNIT),
      force_symmetric_stiffness(false),
      disable_corotate(false),
      use_geometric_stiffness(true),
      use_Rc(true),
      use_Rs(true) {
    nodes.resize(2);

    Km.setZero(this->GetNdofs(), this->GetNdofs());
    Kg.setZero(this->GetNdofs(), this->GetNdofs());
    M.setZero(this->GetNdofs(), this->GetNdofs());
    Rm.setZero(this->GetNdofs(), this->GetNdofs());
    Ri.setZero(this->GetNdofs(), this->GetNdofs());
    Ki.setZero(this->GetNdofs(), this->GetNdofs());

    T.setZero(this->GetNdofs(), this->GetNdofs());
    Rs.setIdentity(6, 6);
    Rc.setIdentity(6, 6);
}

void ChElementBeamTaperedTimoshenko::SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                                              std::shared_ptr<ChNodeFEAxyzrot> nodeB) {
    assert(nodeA);
    assert(nodeB);

    nodes[0] = nodeA;
    nodes[1] = nodeB;
    std::vector<ChVariables*> mvars;
    mvars.push_back(&nodes[0]->Variables());
    mvars.push_back(&nodes[1]->Variables());
    Kmatr.SetVariables(mvars);
}

void ChElementBeamTaperedTimoshenko::ShapeFunctionsTimoshenko(ShapeFunctionGroup& NN, double eta) {
    // eta = 2 * x/L;
    // x = (-L/2, L/2),  hence eta = (-1, 1)

    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;
    double phiy = this->tapered_section->GetAverageSectionParameters()->phiy;
    double phiz = this->tapered_section->GetAverageSectionParameters()->phiz;

    double eta2 = eta * eta;
    double eta3 = eta2 * eta;

    double ay = 1. / (1. + phiy);
    double by = phiy / (1. + phiy);
    double az = 1. / (1. + phiz);
    double bz = phiz / (1. + phiz);

    // axial, torsion
    double Nx1 = 1. / 2. * (1 - eta);
    double Nx2 = 1. / 2. * (1 + eta);
    // shear
    double Nsu1 = 1. / 2. * (1 - eta);
    double Nsu2 = 1. / 2. * (1 + eta);
    double Nsr = -1. / 4. * (1 + eta) * L;
    // bending
    double Nbu1 = 1. / 4. * (eta3 - 3. * eta + 2.);
    double Nbu2 = 1. / 4. * (-eta3 + 3. * eta + 2.);
    double Nbr1 = 1. / 8. * (eta3 - eta2 - eta + 1.) * L;
    double Nbr2 = 1. / 8. * (-eta2 + 2. * eta + 3.) * L;
    double Nbr3 = 1. / 8. * (eta3 + eta2 - eta - 1.) * L;
    double Nbr4 = 1. / 8. * (eta2 + 2. * eta + 1.) * L;

    // shape functions, reduced from three-dimensional shape functions.
    // hence, only valid for the points at centerline
    ShapeFunctionN N;
    N.setZero();
    // x
    N(0, 0) = Nx1;
    N(0, 6) = Nx2;
    // y
    N(1, 1) = Nbu1 * ay + Nsu1 * by;
    N(1, 5) = Nbr1 * ay + Nbr2 * by + Nsr * by;
    N(1, 7) = Nbu2 * ay + Nsu2 * by;
    N(1, 11) = Nbr3 * ay + Nbr4 * by + Nsr * by;
    // z
    N(2, 2) = Nbu1 * az + Nsu1 * bz;
    N(2, 4) = -Nbr1 * az - Nbr2 * bz - Nsr * bz;
    N(2, 8) = Nbu2 * az + Nsu2 * bz;
    N(2, 10) = -Nbr3 * az - Nbr4 * bz - Nsr * bz;
    // rotx
    N(3, 3) = Nx1;
    N(3, 9) = Nx2;

    // Variables named with initial 'k' means block of shape function matrix
    // Variables named with initial 'd' means derivatives respect to x
    // 'd'   --> first derivative
    // 'dd'  --> second derivative
    // 'ddd' --> third derivative

    // some important blocks from shape function matrix
    // bending y
    SFBlock kNby;
    kNby.setZero();
    kNby << Nbu1 * ay, Nbr1 * ay + Nbr2 * by, Nbu2 * ay, Nbr3 * ay + Nbr4 * by;
    // shear y
    SFBlock kNsy;
    kNsy.setZero();
    kNsy << Nsu1 * by, Nsr * by, Nsu2 * by, Nsr * by;
    // bending z
    SFBlock kNbz;
    kNbz.setZero();
    kNbz << Nbu1 * az, -Nbr1 * az - Nbr2 * bz, Nbu2 * az, -Nbr3 * az - Nbr4 * bz;
    // shear z
    SFBlock kNsz;
    kNsz.setZero();
    kNsz << Nsu1 * bz, -Nsr * bz, Nsu2 * bz, -Nsr * bz;
    // axial, torsion
    ChMatrixNM<double, 1, 2> kNx;
    kNx.setZero();
    kNx << Nx1, Nx2;

    // first derivatives
    double dNx1_dx = -1. / L;
    double dNx2_dx = 1. / L;
    double dNsu1_dx = -1. / L;
    double dNsu2_dx = 1. / L;
    double dNsr_dx = -1. / 2.;

    double dNbu1_dx = 1. / (2. * L) * (3. * eta2 - 3.);
    double dNbu2_dx = 1. / (2. * L) * (-3. * eta2 + 3.);

    double dNbr1_dx = 1. / 4. * (3. * eta2 - 2. * eta - 1.);
    double dNbr2_dx = 1. / 2. * (1. - eta);
    double dNbr3_dx = 1. / 4. * (3. * eta2 + 2. * eta - 1.);
    double dNbr4_dx = 1. / 2. * (1. + eta);

    SFBlock dkNby_dx;
    dkNby_dx.setZero();
    dkNby_dx << dNbu1_dx * ay, dNbr1_dx * ay + dNbr2_dx * by, dNbu2_dx * ay, dNbr3_dx * ay + dNbr4_dx * by;

    SFBlock dkNsy_dx;
    dkNsy_dx.setZero();
    dkNsy_dx << dNsu1_dx * by, dNsr_dx * by, dNsu2_dx * by, dNsr_dx * by;

    SFBlock dkNbz_dx;
    dkNbz_dx.setZero();
    dkNbz_dx << dNbu1_dx * az, -dNbr1_dx * az - dNbr2_dx * bz, dNbu2_dx * az, -dNbr3_dx * az - dNbr4_dx * bz;

    SFBlock dkNsz_dx;
    dkNsz_dx.setZero();
    dkNsz_dx << dNsu1_dx * bz, -dNsr_dx * bz, dNsu2_dx * bz, -dNsr_dx * bz;

    ChMatrixNM<double, 1, 2> dkNx_dx;
    dkNx_dx.setZero();
    dkNx_dx << dNx1_dx, dNx2_dx;

    // Continue to fill in the shape functions to finish it.
    // roty
    N(4, 2) = -dNbu1_dx * az;                  // - dNsu1_dx * bz;
    N(4, 4) = dNbr1_dx * az + dNbr2_dx * bz;   // + dNsr_dx  * bz;
    N(4, 8) = -dNbu2_dx * az;                  // - dNsu2_dx * bz;
    N(4, 10) = dNbr3_dx * az + dNbr4_dx * bz;  // + dNsr_dx  * bz;
    // rotz
    N(5, 1) = dNbu1_dx * ay;                   // + dNsu1_dx * by;
    N(5, 5) = dNbr1_dx * ay + dNbr2_dx * by;   // + dNsr_dx  * by;
    N(5, 7) = dNbu2_dx * ay;                   // + dNsu2_dx * by;
    N(5, 11) = dNbr3_dx * ay + dNbr4_dx * by;  // + dNsr_dx  * by;

    // Second derivatives
    double ddNbu1_dx = 6. * eta / LL;
    double ddNbu2_dx = -6. * eta / LL;

    double ddNbr1_dx = 1. / L * (3. * eta - 1.);
    double ddNbr2_dx = -1. / L;
    double ddNbr3_dx = 1. / L * (3. * eta + 1.);
    double ddNbr4_dx = 1. / L;

    SFBlock ddkNby_dx;
    ddkNby_dx.setZero();
    ddkNby_dx << ddNbu1_dx * ay, ddNbr1_dx * ay + ddNbr2_dx * by, ddNbu2_dx * ay, ddNbr3_dx * ay + ddNbr4_dx * by;

    SFBlock ddkNbz_dx;
    ddkNbz_dx.setZero();
    ddkNbz_dx << ddNbu1_dx * az, -ddNbr1_dx * az - ddNbr2_dx * bz, ddNbu2_dx * az, -ddNbr3_dx * az - ddNbr4_dx * bz;

    // Third derivatives
    double dddNbu1_dx = 12. / LLL;
    double dddNbu2_dx = -12. / LLL;

    double dddNbr1_dx = 6. / LL;
    double dddNbr2_dx = 0.;
    double dddNbr3_dx = 6. / LL;
    double dddNbr4_dx = 0.;

    SFBlock dddkNby_dx;
    dddkNby_dx.setZero();
    dddkNby_dx << dddNbu1_dx * ay, dddNbr1_dx * ay + dddNbr2_dx * by, dddNbu2_dx * ay,
        dddNbr3_dx * ay + dddNbr4_dx * by;

    SFBlock dddkNbz_dx;
    dddkNbz_dx.setZero();
    dddkNbz_dx << dddNbu1_dx * az, -dddNbr1_dx * az - dddNbr2_dx * bz, dddNbu2_dx * az,
        -dddNbr3_dx * az - dddNbr4_dx * bz;

    ShapeFunction5Blocks SFblk = std::make_tuple(kNby, kNsy, kNbz, kNsz, kNx);
    ShapeFunction5Blocks SFblk1d = std::make_tuple(dkNby_dx, dkNsy_dx, dkNbz_dx, dkNsz_dx, dkNx_dx);
    ShapeFunction2Blocks SFblk2d = std::make_tuple(ddkNby_dx, ddkNbz_dx);
    ShapeFunction2Blocks SFblk3d = std::make_tuple(dddkNby_dx, dddkNbz_dx);

    std::get<0>(NN) = N;
    std::get<1>(NN) = SFblk;
    std::get<2>(NN) = SFblk1d;
    std::get<3>(NN) = SFblk2d;
    std::get<4>(NN) = SFblk3d;
}

void ChElementBeamTaperedTimoshenko::Update() {
    // parent class update:
    ChElementGeneric::Update();

    // always keep updated the rotation matrix A:
    this->UpdateRotation();
}

void ChElementBeamTaperedTimoshenko::UpdateRotation() {
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

void ChElementBeamTaperedTimoshenko::GetStateBlock(ChVectorDynamic<>& mD) {
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

void ChElementBeamTaperedTimoshenko::GetField_dt(ChVectorDynamic<>& mD_dt) {
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

void ChElementBeamTaperedTimoshenko::GetField_dtdt(ChVectorDynamic<>& mD_dtdt) {
    mD_dtdt.resize(12);

    // Node 0, accelaration (in local element frame, corotated back by A' )
    mD_dtdt.segment(0, 3) = q_element_abs_rot.RotateBack(nodes[0]->Frame().GetPos_dtdt()).eigen();

    // Node 0, x,y,z ang.accelaration (in local element frame, corotated back by A' )
    mD_dtdt.segment(3, 3) = q_element_abs_rot.RotateBack(nodes[0]->Frame().GetWacc_par()).eigen();

    // Node 1, accelaration (in local element frame, corotated back by A' )
    mD_dtdt.segment(6, 3) = q_element_abs_rot.RotateBack(nodes[1]->Frame().GetPos_dtdt()).eigen();

    // Node 1, x,y,z ang.accelaration (in local element frame, corotated back by A' )
    mD_dtdt.segment(9, 3) = q_element_abs_rot.RotateBack(nodes[1]->Frame().GetWacc_par()).eigen();
}

// This class computes and adds corresponding masses to ElementGeneric member m_TotalMass
void ChElementBeamTaperedTimoshenko::ComputeNodalMass() {
   nodes[0]->m_TotalMass += 0.5 * this->length * this->tapered_section->GetSectionA()->GetMassPerUnitLength();
   nodes[1]->m_TotalMass += 0.5 * this->length * this->tapered_section->GetSectionB()->GetMassPerUnitLength();
}

void ChElementBeamTaperedTimoshenko::ComputeTransformMatrix() {
    T.setZero(this->GetNdofs(), this->GetNdofs());

    double alpha1 = this->tapered_section->GetSectionA()->GetSectionRotation();
    double Cy1 = this->tapered_section->GetSectionA()->GetCentroidY();
    double Cz1 = this->tapered_section->GetSectionA()->GetCentroidZ();
    double Sy1 = this->tapered_section->GetSectionA()->GetShearCenterY();
    double Sz1 = this->tapered_section->GetSectionA()->GetShearCenterZ();

    double alpha2 = this->tapered_section->GetSectionB()->GetSectionRotation();
    double Cy2 = this->tapered_section->GetSectionB()->GetCentroidY();
    double Cz2 = this->tapered_section->GetSectionB()->GetCentroidZ();
    double Sy2 = this->tapered_section->GetSectionB()->GetShearCenterY();
    double Sz2 = this->tapered_section->GetSectionB()->GetShearCenterZ();

    double L = this->length;

    // In case the section is rotated:
    ChMatrix33<> RotsectA;
    RotsectA.Set_A_Rxyz(ChVector<>(alpha1, 0, 0));
    ChMatrix33<> RotsectB;
    RotsectB.Set_A_Rxyz(ChVector<>(alpha2, 0, 0));
    ChMatrixNM<double, 12, 12> Rotsect;
    Rotsect.setZero();
    Rotsect.block<3, 3>(0, 0) = RotsectA;
    Rotsect.block<3, 3>(3, 3) = RotsectA;
    Rotsect.block<3, 3>(6, 6) = RotsectB;
    Rotsect.block<3, 3>(9, 9) = RotsectB;

    // In case the section has a centroid displacement:
    // double dCx = 0;
    double dCy = Cy2 - Cy1;
    double dCz = Cz2 - Cz1;
    double LN = L;
    double LG = pow(LN * LN - dCy * dCy - dCz * dCz, 0.5);
    double LA = LG;
    double LB = pow(LG * LG + dCy * dCy, 0.5);
    ChMatrix33<> rc;
    rc.setIdentity();
    rc(0, 0) = LA / LN;
    rc(0, 1) = dCy / LB;
    rc(0, 2) = LA * dCz / (LN * LB);
    rc(1, 0) = -dCy / LN;
    rc(1, 1) = LA / LB;
    rc(1, 2) = -dCy * dCz / (LN * LB);
    rc(2, 0) = -dCz / LN;
    rc(2, 1) = 0.0;
    rc(2, 2) = LB / LN;
    // ChMatrixNM<double, 6, 6> Rc;
    // Rc.setIdentity();
    if (use_Rc) {
        this->Rc.block<3, 3>(0, 0) = rc;
        this->Rc.block<3, 3>(3, 3) = rc;
    }

    ChMatrixNM<double, 6, 6> Tc1;
    Tc1.setIdentity();
    Tc1(0, 4) = Cz1;
    Tc1(0, 5) = -Cy1;
    Tc1(1, 3) = -Cz1;
    Tc1(2, 3) = Cy1;

    ChMatrixNM<double, 6, 6> Tc2;
    Tc2.setIdentity();
    Tc2(0, 4) = Cz2;
    Tc2(0, 5) = -Cy2;
    Tc2(1, 3) = -Cz2;
    Tc2(2, 3) = Cy2;

    ChMatrixNM<double, 12, 12> Tc;
    Tc.setIdentity();
    Tc.block<6, 6>(0, 0) = Tc1 * Rc;
    Tc.block<6, 6>(6, 6) = Tc2 * Rc;

    // In case the section has a shear center displacement:
    // The shear center offset is respect to the centerline of beam element.
    /*Sy1 = Sy1 - Cy1;  // Unnecessary to do this substraction
    Sz1 = Sz1 - Cz1;
    Sy2 = Sy2 - Cy2;
    Sz2 = Sz2 - Cz2;*/
    double dSy = Sy1 - Sy2;
    double dSz = Sz1 - Sz2;

    double Lsy = pow(LG * LG + dSy * dSy, 0.5);
    double Lsyz = pow(LG * LG + dSy * dSy + dSz * dSz, 0.5);
    double C1 = Lsyz / LG;
    double C2 = dSy * Lsyz / (LG * Lsy);
    double C3 = dSz / Lsy;
    if (this->use_simplified_correction_for_inclined_shear_axis) {
        C1 = Lsyz / LG;
        C2 = dSy / LG;
        C3 = dSz / LG;
    }

    ChMatrixNM<double, 6, 6> Ts1;
    Ts1.setIdentity();
    Ts1(1, 3) = -Sz1;
    Ts1(2, 3) = Sy1;

    ChMatrixNM<double, 6, 6> Ts2;
    Ts2.setIdentity();
    Ts2(1, 3) = -Sz2;
    Ts2(2, 3) = Sy2;

    // ChMatrixNM<double, 6, 6> Rs;
    // Rs.setIdentity();
    if (use_Rs) {
        this->Rs(3, 3) = C1;
        this->Rs(4, 3) = C2;
        this->Rs(5, 3) = C3;
    }

    ChMatrixNM<double, 12, 12> Ts;
    Ts.setIdentity();
    Ts.block<6, 6>(0, 0) = Rs * Ts1;
    Ts.block<6, 6>(6, 6) = Rs * Ts2;

    this->T = Rotsect * Ts * Tc;
}

void ChElementBeamTaperedTimoshenko::ComputeTransformMatrixAtPoint(ChMatrixDynamic<>& mT, const double eta) {
    mT.resize(6, 6);

    double alpha1 = this->tapered_section->GetSectionA()->GetSectionRotation();
    double Cy1 = this->tapered_section->GetSectionA()->GetCentroidY();
    double Cz1 = this->tapered_section->GetSectionA()->GetCentroidZ();
    double Sy1 = this->tapered_section->GetSectionA()->GetShearCenterY();
    double Sz1 = this->tapered_section->GetSectionA()->GetShearCenterZ();

    double alpha2 = this->tapered_section->GetSectionB()->GetSectionRotation();
    double Cy2 = this->tapered_section->GetSectionB()->GetCentroidY();
    double Cz2 = this->tapered_section->GetSectionB()->GetCentroidZ();
    double Sy2 = this->tapered_section->GetSectionB()->GetShearCenterY();
    double Sz2 = this->tapered_section->GetSectionB()->GetShearCenterZ();

    // double L = this->length;

    // The shear center offset is respect to the centerline of beam element.
    /*Sy1 = Sy1 - Cy1;  // Unnecessary to do this substraction
    Sz1 = Sz1 - Cz1;
    Sy2 = Sy2 - Cy2;
    Sz2 = Sz2 - Cz2;*/

    // calculate the orientation angle and centers by linear interpolation
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);
    double alpha = Nx1 * alpha1 + Nx2 * alpha2;
    double Cy = Nx1 * Cy1 + Nx2 * Cy2;
    double Cz = Nx1 * Cz1 + Nx2 * Cz2;
    double Sy = Nx1 * Sy1 + Nx2 * Sy2;
    double Sz = Nx1 * Sz1 + Nx2 * Sz2;

    // In case the section is rotated:
    ChMatrix33<> RotsectA;
    RotsectA.Set_A_Rxyz(ChVector<>(alpha, 0, 0));
    ChMatrixNM<double, 6, 6> Rotsect;
    Rotsect.setZero();
    Rotsect.block<3, 3>(0, 0) = RotsectA;
    Rotsect.block<3, 3>(3, 3) = RotsectA;

    // In case the section has a centroid displacement:
    ChMatrixNM<double, 6, 6> Tc;
    Tc.setIdentity();
    Tc(0, 4) = Cz;
    Tc(0, 5) = -Cy;
    Tc(1, 3) = -Cz;
    Tc(2, 3) = Cy;

    Tc = Tc * this->Rc;

    // In case the section has a shear center displacement:
    ChMatrixNM<double, 6, 6> Ts;
    Ts.setIdentity();
    Ts(1, 3) = -Sz;
    Ts(2, 3) = Sy;

    Ts = this->Rs * Ts;

    // the transformation matrix at point eta:
    mT = Rotsect * Ts * Tc;
}

void ChElementBeamTaperedTimoshenko::ComputeStiffnessMatrix() {
    assert(tapered_section);

    Km.setZero(this->GetNdofs(), this->GetNdofs());

    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;

    double EA = this->tapered_section->GetAverageSectionParameters()->EA;
    double GJ = this->tapered_section->GetAverageSectionParameters()->GJ;
    // double GAyy = this->tapered_section->GetAverageSectionParameters()->GAyy;
    // double GAzz = this->tapered_section->GetAverageSectionParameters()->GAzz;
    double EIyy = this->tapered_section->GetAverageSectionParameters()->EIyy;
    double EIzz = this->tapered_section->GetAverageSectionParameters()->EIzz;

    double phiy = this->tapered_section->GetAverageSectionParameters()->phiy;
    double phiz = this->tapered_section->GetAverageSectionParameters()->phiz;

    double az = 12.0 * EIzz / (LLL * (1 + phiy));
    double ay = 12.0 * EIyy / (LLL * (1 + phiz));
    double cz = 6.0 * EIzz / (LL * (1 + phiy));
    double cy = 6.0 * EIyy / (LL * (1 + phiz));
    double ez = (4 + phiy) * EIzz / (L * (1 + phiy));
    double ey = (4 + phiz) * EIyy / (L * (1 + phiz));
    double fz = (2 - phiy) * EIzz / (L * (1 + phiy));
    double fy = (2 - phiz) * EIyy / (L * (1 + phiz));

    Km(0, 0) = EA / L;
    Km(1, 1) = az;
    Km(2, 2) = ay;
    Km(3, 3) = GJ / L;
    Km(4, 4) = ey;
    Km(5, 5) = ez;
    Km(6, 6) = EA / L;
    Km(7, 7) = az;
    Km(8, 8) = ay;
    Km(9, 9) = GJ / L;
    Km(10, 10) = ey;
    Km(11, 11) = ez;

    Km(0, 6) = -EA / L;
    Km(1, 7) = -az;
    Km(2, 8) = -ay;
    Km(3, 9) = -GJ / L;
    Km(4, 10) = fy;
    Km(5, 11) = fz;

    Km(4, 8) = cy;
    Km(5, 7) = -cz;
    Km(1, 11) = cz;
    Km(2, 10) = -cy;

    Km(1, 5) = cz;
    Km(2, 4) = -cy;
    Km(7, 11) = -cz;
    Km(8, 10) = cy;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = r + 1; c < 12; c++)
            Km(c, r) = Km(r, c);

    Km = this->T.transpose() * Km * this->T;
}

void ChElementBeamTaperedTimoshenko::ComputeDampingMatrix() {
    assert(tapered_section);

    Rm.setZero(this->GetNdofs(), this->GetNdofs());

    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;

    double EA = this->tapered_section->GetAverageSectionParameters()->EA;
    double GJ = this->tapered_section->GetAverageSectionParameters()->GJ;
    // double GAyy = this->tapered_section->GetAverageSectionParameters()->GAyy;  // not used here
    // double GAzz = this->tapered_section->GetAverageSectionParameters()->GAzz;  // not used here
    double EIyy = this->tapered_section->GetAverageSectionParameters()->EIyy;
    double EIzz = this->tapered_section->GetAverageSectionParameters()->EIzz;

    double phiy = this->tapered_section->GetAverageSectionParameters()->phiy;
    double phiz = this->tapered_section->GetAverageSectionParameters()->phiz;

    double bx2 = pow(this->tapered_section->GetAverageSectionParameters()->rdamping_coeff.bx, 2.0);
    double by2 = pow(this->tapered_section->GetAverageSectionParameters()->rdamping_coeff.by, 2.0);
    double bz2 = pow(this->tapered_section->GetAverageSectionParameters()->rdamping_coeff.bz, 2.0);
    double bt2 = pow(this->tapered_section->GetAverageSectionParameters()->rdamping_coeff.bt, 2.0);
    double rdamping_alpha = this->tapered_section->GetAverageSectionParameters()->rdamping_coeff.alpha;

    // Correction for the structural damping in the shear deformation, to improve the numerical stability in long-time
    // simulation. It might be helpful, also possible to be useless at all.
    double cc2 = pow(this->tapered_section->GetAverageSectionParameters()->artificial_factor_for_shear_damping, 2.0);
    double ccphiy = (1 + phiy * cc2) / (1. + phiy);
    double ccphiz = (1 + phiz * cc2) / (1. + phiz);
    double ccbz = (EIyy / L * phiz * bz2 * (cc2 - 1)) / (1. + phiz);
    double ccby = (EIzz / L * phiy * by2 * (cc2 - 1)) / (1. + phiy);

    double az = 12.0 * EIzz / (LLL * (1. + phiy));
    double ay = 12.0 * EIyy / (LLL * (1. + phiz));
    double cz = 6.0 * EIzz / (LL * (1. + phiy));
    double cy = 6.0 * EIyy / (LL * (1. + phiz));
    double ez = (4.0 + phiy) * EIzz / (L * (1. + phiy));
    double ey = (4.0 + phiz) * EIyy / (L * (1. + phiz));
    double fz = (2.0 - phiy) * EIzz / (L * (1. + phiy));
    double fy = (2.0 - phiz) * EIyy / (L * (1. + phiz));

    Rm(0, 0) = EA / L * bx2;
    Rm(1, 1) = az * by2 * ccphiy;
    Rm(2, 2) = ay * bz2 * ccphiz;
    Rm(3, 3) = GJ / L * bt2;
    Rm(4, 4) = ey * bz2 * ccphiz;
    Rm(5, 5) = ez * by2 * ccphiy;
    Rm(6, 6) = EA / L * bx2;
    Rm(7, 7) = az * by2 * ccphiy;
    Rm(8, 8) = ay * bz2 * ccphiz;
    Rm(9, 9) = GJ / L * bt2;
    Rm(10, 10) = ey * bz2 * ccphiz;
    Rm(11, 11) = ez * by2 * ccphiy;

    Rm(0, 6) = -EA / L * bx2;
    Rm(1, 7) = -az * by2 * ccphiy;
    Rm(2, 8) = -ay * bz2 * ccphiz;
    Rm(3, 9) = -GJ / L * bt2;
    Rm(4, 10) = fy * bz2 * ccphiz;
    Rm(5, 11) = fz * by2 * ccphiy;

    Rm(4, 8) = cy * bz2 * ccphiz;
    Rm(5, 7) = -cz * by2 * ccphiy;
    Rm(1, 11) = cz * by2 * ccphiy;
    Rm(2, 10) = -cy * bz2 * ccphiz;

    Rm(1, 5) = cz * by2 * ccphiy;
    Rm(2, 4) = -cy * bz2 * ccphiz;
    Rm(7, 11) = -cz * by2 * ccphiy;
    Rm(8, 10) = cy * bz2 * ccphiz;

    Rm(4, 4) += -ccbz;
    Rm(5, 5) += -ccby;
    Rm(10, 4) += ccbz;
    Rm(4, 10) += ccbz;
    Rm(11, 5) += ccby;
    Rm(5, 11) += ccby;
    Rm(10, 10) += -ccbz;
    Rm(11, 11) += -ccby;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = r + 1; c < 12; c++)
            Rm(c, r) = Rm(r, c);

    // The stiffness-proportional term.
    // The geometric stiffness Kg is NOT considered in the damping matrix Rm.
    Rm = this->T.transpose() * Rm * this->T;

    // The mass-proportional term
    if (this->tapered_section->GetLumpedMassMatrixType()) {
        double node_multiplier_fact = 0.5 * this->length;
        Rm += rdamping_alpha * this->M * node_multiplier_fact;
    } else {
        Rm += rdamping_alpha * this->M;
    }
}

void ChElementBeamTaperedTimoshenko::ComputeGeometricStiffnessMatrix() {
    assert(tapered_section);

    Kg.setZero(this->GetNdofs(), this->GetNdofs());

    // Compute the local geometric stiffness matrix Kg without the P multiplication term, that is Kg*(1/P),
    // so that it is a constant matrix for performance reasons.
    // We used the analytical values from
    //   [1] "Nonlinear finite element analysis of elastic frames", Kuo Mo, Hsiao Fang, Yu Hou
    //        Computers & Structures Volume 26, Issue 4, 1987, Pages 693-701
    // For the Reddy or timoshenko more detailed case with higher order terms, look also to:
    //   [2] "A Unified Approach to the Timoshenko Geometric Stiffness Matrix Considering Higher-Order Terms in the
    //   Strain Tensor"
    //        https://www.scielo.br/pdf/lajss/v16n4/1679-7825-lajss-16-04-e185.pdf
    // Look also at: https://enercalc.com/3d_help/toc161394033.html or in Plesha, Malkus, Cook �Concepts and
    // Applications of Finite Element Analysis? or in W. McGuire & R.H. Gallagher & R.D. Ziemian, �Matrix Structural
    // Analysis?

    // double L = this->tapered_section->GetLength();
    double L = this->length;

    // double P_L = 1. / L;
    double P6_5L_y = 6. / (5. * L);    // optional [2]: ...+ 12*IzA /(L*L*L);
    double P6_5L_z = 6. / (5. * L);    // optional [2]: ...+ 12*IyA /(L*L*L);
    double P_10_y = 1. / (10.);        // optional [2]: ...+ 6*IzA /(L*L);
    double P_10_z = 1. / (10.);        // optional [2]: ...+ 6*IyA /(L*L);
    double PL2_15_y = 2. * L / (15.);  // optional [2]: ...+ 4*IzA /(L);
    double PL2_15_z = 2. * L / (15.);  // optional [2]: ...+ 4*IyA /(L);
    double PL_30_y = L / (30.);        // optional [2]: ...+ 2*IyA /(L);
    double PL_30_z = L / (30.);        // optional [2]: ...+ 2*IyA /(L);
    /*  DONOT use the axial terms:
    this->Kg(0, 0) =  P_L;
    this->Kg(6, 6) =  P_L;
    this->Kg(0, 6) = -P_L;
    */

    this->Kg(1, 1) = P6_5L_y;
    this->Kg(1, 5) = P_10_y;
    this->Kg(1, 7) = -P6_5L_y;
    this->Kg(1, 11) = P_10_y;

    this->Kg(2, 2) = P6_5L_z;
    this->Kg(2, 4) = -P_10_z;
    this->Kg(2, 8) = -P6_5L_z;
    this->Kg(2, 10) = -P_10_z;

    this->Kg(4, 4) = PL2_15_y;
    this->Kg(4, 8) = P_10_y;
    this->Kg(4, 10) = -PL_30_y;

    this->Kg(5, 5) = PL2_15_z;
    this->Kg(5, 7) = -P_10_z;
    this->Kg(5, 11) = -PL_30_z;

    this->Kg(7, 7) = P6_5L_y;
    this->Kg(7, 11) = -P_10_y;

    this->Kg(8, 8) = P6_5L_z;
    this->Kg(8, 10) = P_10_y;

    this->Kg(10, 10) = PL2_15_y;

    this->Kg(11, 11) = PL2_15_z;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = r + 1; c < 12; c++)
            Kg(c, r) = Kg(r, c);

    Kg = this->T.transpose() * Kg * this->T;
}

void ChElementBeamTaperedTimoshenko::ComputeAccurateTangentStiffnessMatrix(ChMatrixRef Kt_accurate,
                                                               double Km_factor,
                                                               double Kg_factor) {
    ChMatrix33<> I33;
    I33.setIdentity();
    ChMatrix33<> O33;
    O33.setZero();

    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);
    ChVectorDynamic<> Fi_local = Km * displ;
    double L = this->length;

    ChMatrixNM<double, 12, 12> H;
    H.setZero();
    for (int i = 0; i < nodes.size(); ++i) {
        double rot_angle = displ.segment(3 + 6 * i, 3).norm();
        ChVector<> rot_vector = displ.segment(3 + 6 * i, 3).normalized();
        ChVector<> Theta = displ.segment(3 + 6 * i, 3);
        // double eta = (1 - 0.5 * rot_angle * cos(0.5 * rot_angle) / sin(0.5 * rot_angle)) / (rot_angle * rot_angle);
        double eta = 1.0 / 12.0 + 1.0 / 720.0 * pow(rot_angle, 2) + 1.0 / 30240.0 * pow(rot_angle, 4) +
                     1.0 / 1209600.0 * pow(rot_angle, 6);
        ChMatrix33<> Lambda =
            I33 - 0.5 * ChStarMatrix33<>(Theta) + eta * ChStarMatrix33<>(Theta) * ChStarMatrix33<>(Theta);
        ChMatrixNM<double, 6, 6> Hn;
        Hn << I33, O33, O33, Lambda;
        H.block<6, 6>(i * 6, i * 6) = Hn;
    }

    ChVector<> xA = nodes[0]->Frame().GetPos();
    ChVector<> xB = nodes[1]->Frame().GetPos();
    ChVector<> xF = 0.5 * (xA + xB);
    ChVector<> xA_loc = this->q_element_abs_rot.RotateBack(xA - xF);
    ChVector<> xB_loc = this->q_element_abs_rot.RotateBack(xB - xF);
    ChMatrixNM<double, 12, 3> SD;
    SD.topRows(3) = -ChStarMatrix33<>(xA_loc);
    SD.middleRows(3, 3) = I33;
    SD.middleRows(6, 3) = -ChStarMatrix33<>(xB_loc);
    SD.bottomRows(3) = I33;

    ChMatrixNM<double, 3, 12> G;
    G << 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 1 / L, 0, 0, 0, 0, 0, -1 / L, 0, 0, 0, 0, -1 / L, 0, 0, 0, 0, 0,
        1 / L, 0, 0, 0, 0;

    ChMatrixNM<double, 12, 12> I12;
    I12.setIdentity();
    ChMatrixNM<double, 12, 12> P = I12 - SD * G;

    ChVectorDynamic<> Fi_temp = P.transpose() * H.transpose() * Fi_local;
    ChVectorDynamic<> nA_loc = Fi_temp.segment(0, 3);
    ChVectorDynamic<> mA_loc = Fi_temp.segment(3, 3);
    ChVectorDynamic<> nB_loc = Fi_temp.segment(6, 3);
    ChVectorDynamic<> mB_loc = Fi_temp.segment(9, 3);

    ChMatrixNM<double, 12, 3> Fnm;
    Fnm.topRows(3) = ChStarMatrix33<>(nA_loc);
    Fnm.middleRows(3, 3) = ChStarMatrix33<>(mA_loc);
    Fnm.middleRows(6, 3) = ChStarMatrix33<>(nB_loc);
    Fnm.bottomRows(3) = ChStarMatrix33<>(mB_loc);

    ChMatrixNM<double, 12, 3> Fn;
    Fn.topRows(3) = ChStarMatrix33<>(nA_loc);
    Fn.middleRows(3, 3) = O33;
    Fn.middleRows(6, 3) = ChStarMatrix33<>(nB_loc);
    Fn.bottomRows(3) = O33;

    ChMatrixNM<double, 12, 12> LH;
    LH.setZero();
    for (int i = 0; i < nodes.size(); ++i) {
        double rot_angle = displ.segment(3 + 6 * i, 3).norm();
        ChVector<> rot_vector = displ.segment(3 + 6 * i, 3).normalized();
        ChVector<> Theta = displ.segment(3 + 6 * i, 3);
        // double eta = (1 - 0.5 * rot_angle * cos(0.5 * rot_angle) / sin(0.5 * rot_angle)) / (rot_angle * rot_angle);
        double eta = 1.0 / 12.0 + 1.0 / 720.0 * pow(rot_angle, 2) + 1.0 / 30240.0 * pow(rot_angle, 4) +
                     1.0 / 1209600.0 * pow(rot_angle, 6);
        ChMatrix33<> Lambda =
            I33 - 0.5 * ChStarMatrix33<>(Theta) + eta * ChStarMatrix33<>(Theta) * ChStarMatrix33<>(Theta);

        // double miu = (rot_angle*rot_angle+4*cos(rot_angle)+rot_angle*sin(rot_angle)-4)/(4*pow(rot_angle,
        // 4)*pow(0.5*rot_angle, 2));
        double miu = 1.0 / 360.0 + 1.0 / 7560.0 * pow(rot_angle, 2) + 1.0 / 201600.0 * pow(rot_angle, 4) +
                     1.0 / 5987520.0 * pow(rot_angle, 6);
        ChVector<> m_loc = Fi_temp.segment(3 + 6 * i, 3);

        ChMatrix33<> Li =
            (eta * (Theta.Dot(m_loc) * I33 + TensorProduct(Theta, m_loc) - 2.0 * TensorProduct(m_loc, Theta)) +
             miu * ChStarMatrix33<>(Theta) * ChStarMatrix33<>(Theta) * TensorProduct(m_loc, Theta) -
             0.5 * ChStarMatrix33<>(m_loc)) * Lambda;
        LH.block<3, 3>(i * 6 + 3, i * 6 + 3) = Li;
    }

    ChMatrixNM<double, 12, 12> KM = P.transpose() * H.transpose() * Km * H * P;
    ChMatrixNM<double, 12, 12> KGR = Fnm * G;
    ChMatrixNM<double, 12, 12> KGP = G.transpose() * Fn.transpose() * P;
    ChMatrixNM<double, 12, 12> KGH = P.transpose() * LH * P;
    ChMatrixNM<double, 12, 12> Kt_loc = KM * Km_factor + (-KGR - KGP + KGH) * Kg_factor;

    ChMatrixNM<double, 12, 12> R_rhombus;
    R_rhombus.setZero();
    R_rhombus.block<3, 3>(0, 0) = ChMatrix33<>(this->q_element_abs_rot);
    R_rhombus.block<3, 3>(3, 3) =
        ChMatrix33<>(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
    R_rhombus.block<3, 3>(6, 6) = ChMatrix33<>(this->q_element_abs_rot);
    R_rhombus.block<3, 3>(9, 9) =
        ChMatrix33<>(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);

    Kt_accurate.setZero();
    Kt_accurate = R_rhombus * Kt_loc * R_rhombus.transpose();
}

void ChElementBeamTaperedTimoshenko::ComputeKiRimatricesLocal(bool inertial_damping, bool inertial_stiffness) {
    assert(tapered_section);

    if (inertial_damping) {
        ChVector<> mW_A = this->GetNodeA()->GetWvel_loc();
        ChVector<> mW_B = this->GetNodeB()->GetWvel_loc();
        ChMatrixNM<double, 12, 12> mH;
        this->tapered_section->ComputeInertiaDampingMatrix(mH, mW_A, mW_B);
        this->Ri = 0.5 * this->length * mH;
    } else {
        this->Ri.setZero(this->GetNdofs(), this->GetNdofs());
    }

    if (inertial_stiffness) {
        // current angular velocity of section of node A, in material frame
        ChVector<> mWvel_A = this->GetNodeA()->GetWvel_loc();
        // current angular acceleration of section of node A, in material frame
        ChVector<> mWacc_A = this->GetNodeA()->GetWacc_loc();
        // current acceleration of section of node A, in material frame)
        ChVector<> mXacc_A = this->GetNodeA()->TransformDirectionParentToLocal(this->GetNodeA()->GetPos_dtdt());
        // current angular velocity of section of node B, in material frame
        ChVector<> mWvel_B = this->GetNodeB()->GetWvel_loc();
        // current angular acceleration of section of node B, in material frame
        ChVector<> mWacc_B = this->GetNodeB()->GetWacc_loc();
        // current acceleration of section of node B, in material frame
        ChVector<> mXacc_B = this->GetNodeB()->TransformDirectionParentToLocal(this->GetNodeB()->GetPos_dtdt());

        ChMatrixNM<double, 12, 12> mH;
        this->tapered_section->ComputeInertiaStiffnessMatrix(mH, mWvel_A, mWacc_A, mXacc_A, mWvel_B, mWacc_B, mXacc_B);
        this->Ki = 0.5 * this->length * mH;
    } else {
        this->Ki.setZero(this->GetNdofs(), this->GetNdofs());
    }
}

void ChElementBeamTaperedTimoshenko::SetupInitial(ChSystem* system) {
    assert(tapered_section);

    // Compute rest length, mass:
    this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
    this->mass = 0.5 * this->length * this->tapered_section->GetSectionA()->GetMassPerUnitLength() +
                 0.5 * this->length * this->tapered_section->GetSectionB()->GetMassPerUnitLength();

    // Compute initial rotation
    ChMatrix33<> A0;
    ChVector<> mXele = nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos();
    ChVector<> myele =
        (nodes[0]->GetX0().GetA().Get_A_Yaxis() + nodes[1]->GetX0().GetA().Get_A_Yaxis()).GetNormalized();
    A0.Set_A_Xdir(mXele, myele);
    q_element_ref_rot = A0.Get_A_quaternion();

    // Compute local mass matrix
    // It could be lumped or consistent mass matrix, depends on SetLumpedMassMatrix(true/false)
    // If it is lumped mass matrix, you need to multiple 0.5 * length to obtain the final mass matrix
    // For consistent mass matrix, don't need to multiple anything.
    this->tapered_section->ComputeInertiaMatrix(this->M);

    // Compute transformation matrix
    ComputeTransformMatrix();

    // Compute local stiffness matrix:
    ComputeStiffnessMatrix();

    // Compute local geometric stiffness matrix normalized by pull force P: Kg/P
    ComputeGeometricStiffnessMatrix();

    // Compute local damping matrix:
    ComputeDampingMatrix();
}

void ChElementBeamTaperedTimoshenko::ComputeKRMmatricesGlobal(ChMatrixRef H,
                                                              double Kfactor,
                                                              double Rfactor,
                                                              double Mfactor) {
    assert((H.rows() == 12) && (H.cols() == 12));
    assert(tapered_section);

    //
    // The K stiffness matrix and R damping matrix of this element:
    //

    if (Kfactor || Rfactor) {
        // Corotational K stiffness:
        ChMatrixDynamic<> CK(12, 12);
        ChMatrixDynamic<> CKCt(12, 12);  // the global, corotated, K matrix

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
            // compute Px tension of the beam along centerline, using temporary but fast data structures:
            ChVectorDynamic<> displ(this->GetNdofs());
            this->GetStateBlock(displ);
            double Px = -this->Km.row(0) * displ;
            // ChVector<> mFo, mTo;
            // this->EvaluateSectionForceTorque(0, mFo, mTo);  // for double checking the Px value
            // GetLog() << "   Px = " << Px << "  Px_eval = " << mFo.x() << " \n";

            // corotate Km + Kg  (where Kg = this->Kg * Px)
            ChMatrixCorotation::ComputeCK(this->Km + Px * this->Kg, R, 4, CK);
        } else {
            ChMatrixCorotation::ComputeCK(this->Km, R, 4, CK);
        }
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

        // The code below may be quite slow!!
        ChMatrixDynamic<> CR(12, 12);
        ChMatrixDynamic<> CRCt(12, 12);  // the global, corotated, R matrix
        ChMatrixCorotation::ComputeCK(this->Rm, R, 4, CR);
        ChMatrixCorotation::ComputeKCt(CR, R, 4, CRCt);
        if (force_symmetric_stiffness) {
            for (int row = 0; row < CRCt.rows() - 1; ++row)
                for (int col = row + 1; col < CRCt.cols(); ++col)
                    CRCt(row, col) = CRCt(col, row);
        }

        //// For K stiffness matrix and R matrix: scale by factors
        // CKCt *= Kfactor + Rfactor * this->tapered_section->GetBeamRaleyghDamping();

        H.block(0, 0, 12, 12) = CKCt * Kfactor + CRCt * Rfactor;

        // Add inertial stiffness matrix and inertial damping matrix (gyroscopic damping),
        // if enabled in this beam element.
        // These matrices are not symmetric. Also note.
        if (this->tapered_section->compute_inertia_damping_matrix ||
            this->tapered_section->compute_inertia_stiffness_matrix) {
            ChMatrixNM<double, 12, 12> matr_loc;
            ChMatrixNM<double, 12, 12> KRi_loc;
            KRi_loc.setZero();
            // A lumped version of the inertial damping/stiffness matrix computation is used here, on a per-node basis:
            double node_multiplier_fact_R = 0.5 * length * Rfactor;
            double node_multiplier_fact_K = 0.5 * length * Kfactor;

            ///< current angular velocity of section of node A, in material frame
            ChVector<> mWvel_A = this->GetNodeA()->GetWvel_loc();
            ///< current angular acceleration of section of node A, in material frame
            ChVector<> mWacc_A = this->GetNodeA()->GetWacc_loc();
            ///< current acceleration of section of node A, in material frame)
            ChVector<> mXacc_A = this->GetNodeA()->TransformDirectionParentToLocal(this->GetNodeA()->GetPos_dtdt());
            ///< current angular velocity of section of node B, in material frame
            ChVector<> mWvel_B = this->GetNodeB()->GetWvel_loc();
            ///< current angular acceleration of section of node B, in material frame
            ChVector<> mWacc_B = this->GetNodeB()->GetWacc_loc();
            ///< current acceleration of section of node B, in material frame
            ChVector<> mXacc_B = this->GetNodeB()->TransformDirectionParentToLocal(this->GetNodeB()->GetPos_dtdt());

            if (this->tapered_section->compute_inertia_damping_matrix) {
                this->tapered_section->ComputeInertiaDampingMatrix(matr_loc, mWvel_A, mWacc_A);
                KRi_loc += matr_loc * node_multiplier_fact_R;
                // check the inertial damping matrix, it should be skew symmetric
                // if ((matr_loc + matr_loc.transpose()).norm() < 1E-9) {
                //    std::cout << "Gyroscopic damping matrix is skew symmetric, that's correct！" << std::endl;
                //} else {
                //    std::cout << "Gyroscopic damping matrix is NOT skew symmetric, something is wrong!" << std::endl;
                //}
            }

            if (this->tapered_section->compute_inertia_stiffness_matrix) {
                this->tapered_section->ComputeInertiaStiffnessMatrix(matr_loc, mWvel_A, mWacc_A, mXacc_A, mWvel_B,
                                                                     mWacc_B, mXacc_B);
                KRi_loc += matr_loc * node_multiplier_fact_K;
                // check the inertial stiffness matrix, is it must be skew symmetric as the stiffness matrix?
                // if ((matr_loc + matr_loc.transpose()).norm() < 1E-9) {
                //    std::cout << "Inertial stiffness matrix is skew symmetric" << std::endl;
                //} else {
                //    std::cout << "Inertial stiffness matrix is NOT skew symmetric" << std::endl;
                //}
            }

            for (int i = 0; i < nodes.size(); ++i) {
                int stride = i * 6;
                // corotate the local damping and stiffness matrices (at once, already scaled) into absolute one
                // H.block<3, 3>(stride,   stride  ) += nodes[i]->GetA() * KRi_loc.block<3, 3>(0,0) *
                // (nodes[i]->GetA().transpose()); // NOTE: not needed as KRi_loc.block<3, 3>(0,0) is null by
                // construction
                H.block<3, 3>(stride + 3, stride + 3) += KRi_loc.block<3, 3>(3, 3);
                H.block<3, 3>(stride, stride + 3) += nodes[i]->GetA() * KRi_loc.block<3, 3>(0, 3);
                // H.block<3, 3>(stride+3, stride)   +=                    KRi_loc.block<3, 3>(3,0) *
                // (nodes[i]->GetA().transpose()); // NOTE: not needed as KRi_loc.block<3, 3>(3,0) is null by
                // construction
            }
        }
    } else
        H.setZero();

    //
    // The M mass matrix of this element:
    //

    if (Mfactor) {
        ChMatrixDynamic<> Mloc(12, 12);
        Mloc.setZero();
        ChMatrix33<> Mxw;

        // ChMatrixNM<double, 12, 12> sectional_mass = this->M;  // it could be consistent mass matrix, depends on
        // SetLumpedMassMatrix(true/false)

        if (this->tapered_section->GetLumpedMassMatrixType()) {
            double node_multiplier_fact = 0.5 * this->length * Mfactor;
            for (int i = 0; i < nodes.size(); ++i) {
                int stride = i * 6;
                // if there is no mass center offset, the upper right and lower left blocks need not be rotated,
                // hence it can be the simple (constant) expression
                //   Mloc.block<6, 6>(stride, stride) += sectional_mass * node_multiplier_fact;
                // but the more general case needs the rotations, hence:
                Mloc.block<3, 3>(stride, stride) += this->M.block<3, 3>(stride, stride) * node_multiplier_fact;
                Mloc.block<3, 3>(stride + 3, stride + 3) +=
                    this->M.block<3, 3>(stride + 3, stride + 3) * node_multiplier_fact;
                Mxw = nodes[i]->GetA() * this->M.block<3, 3>(stride, stride + 3) * node_multiplier_fact;
                Mloc.block<3, 3>(stride, stride + 3) += Mxw;
                Mloc.block<3, 3>(stride + 3, stride) += Mxw.transpose();
            }
            // ..rather do this because lumped mass matrix does not need rotation transf.
            H.block(0, 0, 12, 12) += Mloc;

        } else {
            Mloc = this->M * Mfactor;

            // The following would be needed if consistent mass matrix is used, but...
            ChMatrix33<> Atoabs(this->q_element_abs_rot);
            ChMatrix33<> AtolocwelA(this->GetNodeA()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            ChMatrix33<> AtolocwelB(this->GetNodeB()->Frame().GetRot().GetConjugate() % this->q_element_abs_rot);
            std::vector<ChMatrix33<>*> R;
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelA);
            R.push_back(&Atoabs);
            R.push_back(&AtolocwelB);

            ChMatrixNM<double, 12, 12> CK;
            ChMatrixNM<double, 12, 12> CKCt;
            ChMatrixCorotation::ComputeCK(Mloc, R, 4, CK);
            ChMatrixCorotation::ComputeKCt(CK, R, 4, CKCt);

            H.block(0, 0, 12, 12) += CKCt;
        }

        //// TODO better per-node lumping, or 4x4 consistent mass matrices, maybe with integration if not uniform
        // materials.
    }
}

void ChElementBeamTaperedTimoshenko::GetKRMmatricesLocal(ChMatrixRef H,
                                                         double Kmfactor,
                                                         double Kgfactor,
                                                         double Rmfactor,
                                                         double Mfactor) {
    assert((H.rows() == 12) && (H.cols() == 12));

    H.block(0, 0, 12, 12) = this->Km * Kmfactor + this->Kg * Kgfactor + this->Rm * Rmfactor + this->M * Mfactor;

}

void ChElementBeamTaperedTimoshenko::ComputeInternalForces(ChVectorDynamic<>& Fi) {
    assert(Fi.size() == 12);
    assert(tapered_section);

    // set up vector of nodal displacements and small rotations (in local element system)
    ChVectorDynamic<> displ(12);
    this->GetStateBlock(displ);

    // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
    ChVectorDynamic<> Fi_local = this->Km * displ;

    // set up vector of nodal velocities (in local element system)
    ChVectorDynamic<> displ_dt(12);
    this->GetField_dt(displ_dt);

    // ChMatrixDynamic<> FiR_local = this->tapered_section->GetBeamRaleyghDamping() * this->Km * displ_dt;
    ChMatrixDynamic<> FiR_local = this->Rm * displ_dt;

    Fi_local += FiR_local;
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
        // int stride = i * 6;
        if (i == 0) {
            this->tapered_section->GetSectionA()->ComputeQuadraticTerms(mFcent_i, mTgyro_i, nodes[i]->GetWvel_loc());
        } else {  // i==1
            this->tapered_section->GetSectionB()->ComputeQuadraticTerms(mFcent_i, mTgyro_i, nodes[i]->GetWvel_loc());
        }
        ChQuaternion<> q_i(nodes[i]->GetRot());
        Fi.segment(i * 6, 3) -= node_multiplier * (nodes[i]->GetA() * mFcent_i).eigen();
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

void ChElementBeamTaperedTimoshenko::ComputeInternalForces(ChVectorDynamic<>& Fi,
                                                           bool Mfactor,
                                                           bool Kfactor,
                                                           bool Rfactor,
                                                           bool Gfactor) {
    assert(Fi.size() == 12);
    assert(tapered_section);

    ChVectorDynamic<> FiMKR_local(12);
    FiMKR_local.setZero();

    if (Mfactor && true) {
        // set up vector of nodal accelerations (in local element system)
        ChVectorDynamic<> displ_dtdt(12);
        this->GetField_dtdt(displ_dtdt);

        ChVectorDynamic<> FiM_local = this->M * displ_dtdt;  // this->M is the local mass matrix of element
        FiMKR_local -= FiM_local;
    }

    if (Kfactor) {
        // set up vector of nodal displacements and small rotations (in local element system)
        ChVectorDynamic<> displ(12);
        this->GetStateBlock(displ);

        // [local Internal Forces] = [Klocal] * displ + [Rlocal] * displ_dt
        ChVectorDynamic<> FiK_local = this->Km * displ;
        FiMKR_local -= FiK_local;
    }
    if (Rfactor) {
        // set up vector of nodal velocities (in local element system)
        ChVectorDynamic<> displ_dt(12);
        this->GetField_dt(displ_dt);

        // ChMatrixDynamic<> FiR_local = this->tapered_section->GetBeamRaleyghDamping() * this->Km * displ_dt;
        ChMatrixDynamic<> FiR_local = this->Rm * displ_dt;
        FiMKR_local -= FiR_local;
    }

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
    ChMatrixCorotation::ComputeCK(FiMKR_local, R, 4, Fi);

    // DO NOT USE THIS PIECE OF CODE.
    // if (Mfactor && false) {  // Is this correct? Or the code in lines 1029~1036
    //    // set up vector of nodal accelarations (in absolute element system)
    //    ChVectorDynamic<> displ_dtdt(12);
    //    displ_dtdt.segment(0, 3) = nodes[0]->Frame().GetPos_dtdt().eigen();
    //    // displ_dtdt.segment(3, 3) = nodes[0]->Frame().GetWacc_par().eigen();        //但是之后dynamics求解发散
    //    displ_dtdt.segment(3, 3) = nodes[0]->Frame().GetWacc_loc().eigen();  //但是之后dynamics求解发散

    //    displ_dtdt.segment(6, 3) = nodes[1]->Frame().GetPos_dtdt().eigen();
    //    // displ_dtdt.segment(9, 3) = nodes[1]->Frame().GetWacc_par().eigen();
    //    displ_dtdt.segment(9, 3) = nodes[1]->Frame().GetWacc_loc().eigen();
    //    // this->GetField_dtdt(displ_dtdt);

    //    ChMatrixDynamic<> Mabs(12, 12);
    //    Mabs.setZero();
    //    this->ComputeKRMmatricesGlobal(Mabs, false, false,
    //                                   true);  // Mabs is the mass matrix in absolute coordinate for this element

    //    ChVectorDynamic<> FiM_abs = Mabs * displ_dtdt;
    //    Fi -= FiM_abs;
    //}

    if (Gfactor) {
        // Add also inertial quadratic terms: gyroscopic and centrifugal

        // CASE OF LUMPED MASS - fast
        double node_multiplier = 0.5 * length;
        ChVector<> mFcent_i;
        ChVector<> mTgyro_i;
        for (int i = 0; i < nodes.size(); ++i) {
            // int stride = i * 6;
            if (i == 0) {
                this->tapered_section->GetSectionA()->ComputeQuadraticTerms(mFcent_i, mTgyro_i,
                                                                            nodes[i]->GetWvel_loc());
            } else {  // i==1
                this->tapered_section->GetSectionB()->ComputeQuadraticTerms(mFcent_i, mTgyro_i,
                                                                            nodes[i]->GetWvel_loc());
            }
            ChQuaternion<> q_i(nodes[i]->GetRot());
            Fi.segment(i * 6, 3) -= node_multiplier * (nodes[i]->GetA() * mFcent_i).eigen();
            Fi.segment(3 + i * 6, 3) -= node_multiplier * mTgyro_i.eigen();
        }
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

void ChElementBeamTaperedTimoshenko::ComputeGravityForces(ChVectorDynamic<>& Fg, const ChVector<>& G_acc) {
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

    //// TODO for the lumped mass matrix case, the mM * mG product can be unrolled into few multiplications as mM mostly
    /// zero, and same for mG
}

void ChElementBeamTaperedTimoshenko::EvaluateSectionDisplacement(const double eta,
                                                                 ChVector<>& u_displ,
                                                                 ChVector<>& u_rotaz) {
    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);
    // No transformation for the displacement of two nodes,
    // so the section displacement is evaluated at the centerline of beam

    ShapeFunctionGroup NN;
    ShapeFunctionsTimoshenko(NN, eta);
    ShapeFunction5Blocks sfblk = std::get<1>(NN);
    ShapeFunction5Blocks sfblk1d = std::get<2>(NN);
    auto kNby = std::get<0>(sfblk);
    auto kNsy = std::get<1>(sfblk);
    auto kNbz = std::get<2>(sfblk);
    auto kNsz = std::get<3>(sfblk);
    auto kNx = std::get<4>(sfblk);
    auto dkNby = std::get<0>(sfblk1d);
    auto dkNbz = std::get<2>(sfblk1d);

    ChVectorN<double, 4> qey;
    qey << displ(1), displ(5), displ(7), displ(11);
    ChVectorN<double, 4> qez;
    qez << displ(2), displ(4), displ(8), displ(10);
    ChVectorN<double, 2> qeux;
    qeux << displ(0), displ(6);
    ChVectorN<double, 2> qerx;
    qerx << displ(3), displ(9);

    u_displ.x() = kNx * qeux;
    u_displ.y() = (kNby + kNsy) * qey;
    u_displ.z() = (kNbz + kNsz) * qez;
    u_rotaz.x() = kNx * qerx;
    u_rotaz.y() = -dkNbz * qez;
    u_rotaz.z() = dkNby * qey;
}

void ChElementBeamTaperedTimoshenko::EvaluateSectionFrame(const double eta, ChVector<>& point, ChQuaternion<>& rot) {
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

void ChElementBeamTaperedTimoshenko::EvaluateSectionForceTorque(const double eta,
                                                                ChVector<>& Fforce,
                                                                ChVector<>& Mtorque) {
    assert(tapered_section);

    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);

    ChVectorDynamic<> displ_ec = this->T * displ;  // transform the displacement of two nodes to elastic axis

    ChVectorN<double, 4> qey;
    qey << displ_ec(1), displ_ec(5), displ_ec(7), displ_ec(11);
    ChVectorN<double, 4> qez;
    qez << displ_ec(2), displ_ec(4), displ_ec(8), displ_ec(10);
    ChVectorN<double, 2> qeux;
    qeux << displ_ec(0), displ_ec(6);
    ChVectorN<double, 2> qerx;
    qerx << displ_ec(3), displ_ec(9);

    ShapeFunctionGroup NN;
    ShapeFunctionsTimoshenko(NN, eta);
    ShapeFunction5Blocks sfblk1d = std::get<2>(NN);
    ShapeFunction2Blocks sfblk2d = std::get<3>(NN);
    ShapeFunction2Blocks sfblk3d = std::get<4>(NN);
    auto dkNx1 = std::get<4>(sfblk1d);
    auto dkNsy = std::get<1>(sfblk1d);
    auto dkNsz = std::get<3>(sfblk1d);
    auto ddkNby = std::get<0>(sfblk2d);
    auto ddkNbz = std::get<1>(sfblk2d);
    auto dddkNby = std::get<0>(sfblk3d);
    auto dddkNbz = std::get<1>(sfblk3d);

    double EA = this->tapered_section->GetAverageSectionParameters()->EA;
    double GJ = this->tapered_section->GetAverageSectionParameters()->GJ;
    double GAyy = this->tapered_section->GetAverageSectionParameters()->GAyy;
    double GAzz = this->tapered_section->GetAverageSectionParameters()->GAzz;
    double EIyy = this->tapered_section->GetAverageSectionParameters()->EIyy;
    double EIzz = this->tapered_section->GetAverageSectionParameters()->EIzz;

    double eps = 1.0e-3;
    bool use_shear_stain = true;  // As default, use shear strain to evaluate the shear forces
    if (abs(GAyy) < eps ||
        abs(GAzz) < eps) {  // Sometimes, the user will not input GAyy, GAzz, so GAyy, GAzz may be zero.
        use_shear_stain = false;
    }

    // generalized strains/curvatures;
    ChVectorN<double, 6> sect_ek;
    sect_ek(0) = dkNx1 * qeux;   // ux
    sect_ek(3) = dkNx1 * qerx;   // rotx
    sect_ek(4) = -ddkNbz * qez;  // roty
    sect_ek(5) = ddkNby * qey;   // rotz

    if (use_shear_stain) {
        // Strictly speaking, dkNsy * qey,dkNsz * qez are the real shear strains.
        sect_ek(1) = dkNsy * qey;  // gamma_y = dkNsy * qey; Fy = GAyy * gamma_y;
        sect_ek(2) = dkNsz * qez;  // gamma_z = dkNsz * qez; Fz = GAzz * gamma_z;
    } else {
        // Calculate the shear strain through third-differentian of bending displacement
        sect_ek(1) = -dddkNby * qey;  // Fy == -EIzz * dddkNby * qey == GAyy * dkNsy * qey;
        sect_ek(2) = -dddkNbz * qez;  // Fz == -EIyy *  dddkNbz * qez == GAzz * dkNsz * qez;
    }

    if (false)  // section->alpha ==0 && section->Cy ==0 && section->Cz==0 && section->Sy==0 && section->Sz==0)
    {
        // Fast computation:
        Fforce.x() = EA * sect_ek(0);
        if (use_shear_stain) {
            Fforce.y() = GAyy * sect_ek(1);
            Fforce.z() = GAzz * sect_ek(2);
        } else {
            Fforce.y() = EIzz * sect_ek(1);
            Fforce.z() = EIyy * sect_ek(2);
        }

        Mtorque.x() = GJ * sect_ek(3);
        Mtorque.y() = EIyy * sect_ek(4);
        Mtorque.z() = EIzz * sect_ek(5);
    } else {
        // Constitutive matrix of the beam:
        ChMatrixNM<double, 6, 6> Klaw_d;
        Klaw_d.setZero();
        Klaw_d(0, 0) = EA;
        Klaw_d(3, 3) = GJ;
        Klaw_d(4, 4) = EIyy;
        Klaw_d(5, 5) = EIzz;

        if (use_shear_stain) {
            Klaw_d(1, 1) = GAyy;
            Klaw_d(2, 2) = GAzz;
        } else {
            Klaw_d(1, 1) = EIzz;
            Klaw_d(2, 2) = EIyy;
        }

        ChMatrixDynamic<> Teta;
        ComputeTransformMatrixAtPoint(Teta, eta);

        // ..unrolled rotated constitutive matrix..
        ChMatrixNM<double, 6, 6> Klaw_r;
        Klaw_r.setZero();
        Klaw_r = Teta.transpose() * Klaw_d;

        // .. compute wrench = Klaw_r * sect_ek
        ChVectorN<double, 6> wrench = Klaw_r * sect_ek;
        Fforce = wrench.segment(0, 3);
        Mtorque = wrench.segment(3, 3);
    }
}

void ChElementBeamTaperedTimoshenko::EvaluateSectionStrain(const double eta,
                                                           ChVector<>& StrainV_trans,
                                                           ChVector<>& StrainV_rot) {
    assert(tapered_section);

    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);

    ChVectorDynamic<> displ_ec = this->T * displ;  // transform the displacement of two nodes to elastic axis

    ChVectorN<double, 4> qey;
    qey << displ_ec(1), displ_ec(5), displ_ec(7), displ_ec(11);
    ChVectorN<double, 4> qez;
    qez << displ_ec(2), displ_ec(4), displ_ec(8), displ_ec(10);
    ChVectorN<double, 2> qeux;
    qeux << displ_ec(0), displ_ec(6);
    ChVectorN<double, 2> qerx;
    qerx << displ_ec(3), displ_ec(9);

    ShapeFunctionGroup NN;
    ShapeFunctionsTimoshenko(NN, eta);
    ShapeFunction5Blocks sfblk1d = std::get<2>(NN);
    ShapeFunction2Blocks sfblk2d = std::get<3>(NN);
    ShapeFunction2Blocks sfblk3d = std::get<4>(NN);
    auto dkNx1 = std::get<4>(sfblk1d);
    auto dkNsy = std::get<1>(sfblk1d);
    auto dkNsz = std::get<3>(sfblk1d);
    auto ddkNby = std::get<0>(sfblk2d);
    auto ddkNbz = std::get<1>(sfblk2d);
    auto dddkNby = std::get<0>(sfblk3d);
    auto dddkNbz = std::get<1>(sfblk3d);

    ////double EA = this->tapered_section->GetAverageSectionParameters()->EA;
    ////double GJ = this->tapered_section->GetAverageSectionParameters()->GJ;
    double GAyy = this->tapered_section->GetAverageSectionParameters()->GAyy;
    double GAzz = this->tapered_section->GetAverageSectionParameters()->GAzz;
    ////double EIyy = this->tapered_section->GetAverageSectionParameters()->EIyy;
    ////double EIzz = this->tapered_section->GetAverageSectionParameters()->EIzz;

    double eps = 1.0e-3;
    bool use_shear_stain = true;  // As default, use shear strain to evaluate the shear forces
    if (abs(GAyy) < eps ||
        abs(GAzz) < eps) {  // Sometimes, the user will not input GAyy, GAzz, so GAyy, GAzz may be zero.
        use_shear_stain = false;
    }

    // generalized strains/curvatures;
    ChVectorN<double, 6> sect_ek;
    sect_ek(0) = dkNx1 * qeux;   // ux
    sect_ek(3) = dkNx1 * qerx;   // rotx
    sect_ek(4) = -ddkNbz * qez;  // roty
    sect_ek(5) = ddkNby * qey;   // rotz

    if (use_shear_stain) {
        // Strictly speaking, dkNsy * qey,dkNsz * qez are the real shear strains.
        sect_ek(1) = dkNsy * qey;  // gamma_y = dkNsy * qey; Fy = GAyy * gamma_y;
        sect_ek(2) = dkNsz * qez;  // gamma_z = dkNsz * qez; Fz = GAzz * gamma_z;
    } else {
        // Calculate the shear strain through third-differentian of bending displacement
        sect_ek(1) = -dddkNby * qey;  // Fy == -EIzz * dddkNby * qey == GAyy * dkNsy * qey;
        sect_ek(2) = -dddkNbz * qez;  // Fz == -EIyy *  dddkNbz * qez == GAzz * dkNsz * qez;
    }

    StrainV_trans = sect_ek.segment(0, 3);
    StrainV_rot = sect_ek.segment(3, 3);
}

void ChElementBeamTaperedTimoshenko::EvaluateElementStrainEnergy(ChVector<>& StrainEnergyV_trans,
                                                                 ChVector<>& StrainEnergyV_rot) {
    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);

    ChVectorN<double, 12> strain_energy = 1.0 / 2.0 * displ.asDiagonal() * this->Km * displ;
    ChVectorN<double, 6> strain_energy_v;
    // double strain_energy_sum = 0;
    for (int i = 0; i < 6; i++) {
        strain_energy_v(i) = strain_energy(i) + strain_energy(i + 6);
        // strain_energy_sum += strain_energy_v(i);
    }

    StrainEnergyV_trans = strain_energy_v.segment(0, 3);
    StrainEnergyV_rot = strain_energy_v.segment(3, 3);
}

void ChElementBeamTaperedTimoshenko::EvaluateElementDampingEnergy(ChVector<>& DampingEnergyV_trans,
                                                                  ChVector<>& DampingEnergyV_rot) {
    ChVectorDynamic<> displ(this->GetNdofs());
    this->GetStateBlock(displ);
    ChVectorDynamic<> displ_dt(this->GetNdofs());
    this->GetField_dt(displ_dt);

    ChVectorN<double, 12> damping_energy = 1.0 / 2.0 * displ.asDiagonal() * this->Rm * displ_dt;
    ChVectorN<double, 6> damping_energy_v;
    // double damping_energy_sum = 0;
    for (int i = 0; i < 6; i++) {
        damping_energy_v(i) = damping_energy(i) + damping_energy(i + 6);
        // damping_energy_sum += damping_energy_v(i);
    }

    DampingEnergyV_trans = damping_energy_v.segment(0, 3);
    DampingEnergyV_rot = damping_energy_v.segment(3, 3);
}

void ChElementBeamTaperedTimoshenko::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos().eigen();
    mD.segment(block_offset + 3, 4) = nodes[0]->GetRot().eigen();

    mD.segment(block_offset + 7, 3) = nodes[1]->GetPos().eigen();
    mD.segment(block_offset + 10, 4) = nodes[1]->GetRot().eigen();
}

void ChElementBeamTaperedTimoshenko::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = nodes[0]->GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = nodes[0]->GetWvel_loc().eigen();

    mD.segment(block_offset + 6, 3) = nodes[1]->GetPos_dt().eigen();
    mD.segment(block_offset + 9, 3) = nodes[1]->GetWvel_loc().eigen();
}

void ChElementBeamTaperedTimoshenko::LoadableStateIncrement(const unsigned int off_x,
                                                            ChState& x_new,
                                                            const ChState& x,
                                                            const unsigned int off_v,
                                                            const ChStateDelta& Dv) {
    nodes[0]->NodeIntStateIncrement(off_x, x_new, x, off_v, Dv);
    nodes[1]->NodeIntStateIncrement(off_x + 7, x_new, x, off_v + 6, Dv);
}

void ChElementBeamTaperedTimoshenko::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&this->nodes[0]->Variables());
    mvars.push_back(&this->nodes[1]->Variables());
}

void ChElementBeamTaperedTimoshenko::ComputeNF(const double U,
                                               ChVectorDynamic<>& Qi,
                                               double& detJ,
                                               const ChVectorDynamic<>& F,
                                               ChVectorDynamic<>* state_x,
                                               ChVectorDynamic<>* state_w) {
    ShapeFunctionGroup NN;
    ShapeFunctionsTimoshenko(NN, U);
    ShapeFunctionN N = std::get<0>(NN);

    // eta = 2*x/L;
    // ---> Deta/dx = 2./L;
    // ---> detJ = dx/Deta = L/2.;
    detJ = this->GetRestLength() / 2.0;

    Qi = N.transpose() * F;
}

void ChElementBeamTaperedTimoshenko::ComputeNF(const double U,
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

double ChElementBeamTaperedTimoshenko::GetDensity() {
    double mu1 = this->tapered_section->GetSectionA()->GetMassPerUnitLength();
    double mu2 = this->tapered_section->GetSectionB()->GetMassPerUnitLength();
    double mu = (mu1 + mu2) / 2.0;

    return mu;
}

}  // end namespace fea
}  // end namespace chrono
