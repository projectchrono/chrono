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

#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"

namespace chrono {
namespace fea {


void ChElementBeamTaperedTimoshenkoFPM::ComputeStiffnessMatrix() {
    assert(taperedSectionFPM);

    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;

    ChMatrixNM<double,6,6> Klaw = this->taperedSectionFPM->GetAverageFPM();
    double EA = this->taperedSectionFPM->GetAverageSectionParameters()->EA;
    double GJ = this->taperedSectionFPM->GetAverageSectionParameters()->GJ;
    double GAyy = this->taperedSectionFPM->GetAverageSectionParameters()->GAyy;
    double GAzz = this->taperedSectionFPM->GetAverageSectionParameters()->GAzz;
    double EIyy = this->taperedSectionFPM->GetAverageSectionParameters()->EIyy;
    double EIzz = this->taperedSectionFPM->GetAverageSectionParameters()->EIzz;

    double phiy = this->taperedSectionFPM->GetAverageSectionParameters()->phiy;
    double phiz = this->taperedSectionFPM->GetAverageSectionParameters()->phiz;

    double ay = 1. / (1. + phiy);
    double by = phiy * ay;
    double az = 1. / (1. + phiz);
    double bz = phiz * az;

    double k12 = Klaw(0, 1);
    double k13 = Klaw(0, 2);
    double k14 = Klaw(0, 3);
    double k15 = Klaw(0, 4);
    double k16 = Klaw(0, 5);
    double k23 = Klaw(1, 2);
    double k24 = Klaw(1, 3);
    double k25 = Klaw(1, 4);
    double k26 = Klaw(1, 5);
    double k34 = Klaw(2, 3);
    double k35 = Klaw(2, 4);
    double k36 = Klaw(2, 5);
    double k45 = Klaw(3, 4);
    double k46 = Klaw(3, 5);
    double k56 = Klaw(4, 5);
    

    //TODO: need to check whether this stiffness matrix has shear locking issue!
    Km(0, 0) = EA / L;
    Km(0, 1) = by * k12 / L;
    Km(0, 2) = bz * k13 / L;
    Km(0, 3) = k14 / L;
    Km(0, 4) = -bz * k13 / 2. + k15 / L;
    Km(0, 5) = by * k12 / 2. + k16 / L;
    Km(0, 6) = -EA / L;
    Km(0, 7) = -by * k12 / L;
    Km(0, 8) = -bz * k13 / L;
    Km(0, 9) = -k14 / L;
    Km(0, 10) = -bz * k13 / 2. - k15 / L;
    Km(0, 11) = by * k12 / 2. - k16 / L;
    Km(1, 1) = 12. * EIzz * ay / LLL;
    Km(1, 2) = (by * bz * k23 * LL - 12. * ay * az * k56) / LLL;
    Km(1, 3) = by * k24 / L;
    Km(1, 4) = (L * by * k25 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Km(1, 5) = 6. * EIzz * ay / LL + k26 * by / L;
    Km(1, 6) = -by * k12 / L;
    Km(1, 7) = -12. * EIzz * ay / LLL;
    Km(1, 8) = -Km(1, 2);
    Km(1, 9) = -by * k24 / L;
    Km(1, 10) = (-L * by * k25 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Km(1, 11) = 6. * EIzz * ay / LL - k26 * by / L;
    Km(2, 2) = 12. * EIyy * az / LLL;
    Km(2, 3) = bz * k34 / L;
    Km(2, 4) = k35 * bz / L - 6. * EIyy * az / LL;
    Km(2, 5) = (L * bz * k36 - 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Km(2, 6) = -bz * k13 / L;
    Km(2, 7) = (-by * bz * k23 * LL + 12. * ay * az * k56) / LLL;
    Km(2, 8) = -12. * EIyy * az / LLL;
    Km(2, 9) = -bz * k34 / L;
    Km(2, 10) = -6. * EIyy * az / LL - k35 * bz / L;
    Km(2, 11) = -(L * bz * k36 + 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Km(3, 3) = GJ / L;
    Km(3, 4) = k45 / L - bz * k34 / 2.;
    Km(3, 5) = k46 / L + by * k24 / 2.;
    Km(3, 6) = -k14 / L;
    Km(3, 7) = -by * k24 / L;
    Km(3, 8) = -bz * k34 / L;
    Km(3, 9) = -GJ / L;
    Km(3, 10) = -k45 / L - bz * k34 / 2.;
    Km(3, 11) = -k46 / L + by * k24 / 2.;
    Km(4, 4) = EIyy * (4. + phiz) * az / L - bz * k35;
    Km(4, 5) = -L * by * bz * k23 / 4. + by * k25 / 2. - bz * k36 / 2. + (3. * ay * az + 1.) * k56 / L;
    Km(4, 6) = bz * k13 / 2. - k15 / L;
    Km(4, 7) = by * bz * k23 / 2. - (L * by * k25 + 6. * ay * az * k56) / LL;
    Km(4, 8) = 6. * EIyy * az / LL - k35 * bz / L;
    Km(4, 9) = bz * k34 / 2. - k45 / L;
    Km(4, 10) = EIyy * (2. - phiz) * az / L;
    Km(4, 11) = -L * by * bz * k23 / 4. + by * k25 / 2. + bz * k36 / 2. + (3. * ay * az - 1.) * k56 / L;
    Km(5, 5) = EIzz * (4. + phiy) * ay / L + by * k26;
    Km(5, 6) = -by * k12 / 2. - k16 / L;
    Km(5, 7) = -6. * EIzz * ay / LL - k26 * by / L;
    Km(5, 8) = -(L * bz * k36 - 6. * ay * az * k56) / LL - by * bz * k23 / 2.;
    Km(5, 9) = -by * k24 / 2. - k46 / L;
    Km(5, 10) = (3. * ay * az - 1.) * k56 / L - by * k25 / 2. - bz * k36 / 2. - L * bz * by * k23 / 4.;
    Km(5, 11) = EIzz * (2. - phiy) * ay / L;
    Km(6, 6) = EA / L;
    Km(6, 7) = by * k12 / L;
    Km(6, 8) = bz * k13 / L;
    Km(6, 9) = k14 / L;
    Km(6, 10) = bz * k13 / 2. + k15 / L;
    Km(6, 11) = k16 / L - by * k12 / 2.;
    Km(7, 7) = 12. * EIzz * ay / LLL;
    Km(7, 8) = (by * bz * k23 * LL - 12. * ay * az * k56) / LLL;
    Km(7, 9) = by * k24 / L;
    Km(7, 10) = (L * by * k25 - 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Km(7, 11) = -6. * EIzz * ay / LL + k26 * by / L;
    Km(8, 8) = 12. * EIyy * az / LLL;
    Km(8, 9) = bz * k34 / L;
    Km(8, 10) = 6. * EIyy * az / LL + k35 * bz / L;
    Km(8, 11) = (L * bz * k36 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Km(9, 9) = GJ / L;
    Km(9, 10) = bz * k34 / 2. + k45 / L;
    Km(9, 11) = k46 / L - by * k24 / 2.;
    Km(10, 10) = bz * k35 + EIyy * (4. + phiz) * az / L;
    Km(10, 11) = bz * k36 / 2. - by * k25 / 2. - L * by * bz * k23 / 4. + (3. * ay * az + 1.) * k56 / L;
    Km(11, 11) = EIzz * (4. + phiy) * ay / L - by * k26;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = 0; c < r; c++)
            Km(r, c) = Km(c, r);

    Km = this->T.transpose() * Km * this->T;
}

void ChElementBeamTaperedTimoshenkoFPM::ComputeDampingMatrix() {
    assert(taperedSectionFPM);

    double L = this->length;
    double LL = L * L;
    double LLL = LL * L;

    double mbx = this->taperedSectionFPM->GetAverageSectionParameters()->rdamping_coeff.bx;
    double mby = this->taperedSectionFPM->GetAverageSectionParameters()->rdamping_coeff.by;
    double mbz = this->taperedSectionFPM->GetAverageSectionParameters()->rdamping_coeff.bz;
    double mbt = this->taperedSectionFPM->GetAverageSectionParameters()->rdamping_coeff.bt;
    ChMatrixNM<double, 6, 6> mb;
    mb.setIdentity();
    mb(0, 0) = mbx;
    mb(1, 1) = mby;
    mb(2, 2) = mbz;
    mb(3, 3) = mbt;
    mb(4, 4) = mbz;
    mb(5, 5) = mby;

    ChMatrixNM<double, 6, 6> Klaw = this->taperedSectionFPM->GetAverageFPM();
    ChMatrixNM<double, 6, 6> Rlaw = mb.transpose() * Klaw * mb;  // material damping matrix

    double rEA = Rlaw(0,0);
    double rGAyy = Rlaw(1, 1);
    double rGAzz = Rlaw(2, 2);
    double rGJ = Rlaw(3, 3);
    double rEIyy = Rlaw(4, 4);
    double rEIzz = Rlaw(5, 5);

    double phiy = this->taperedSectionFPM->GetAverageSectionParameters()->phiy;
    double phiz = this->taperedSectionFPM->GetAverageSectionParameters()->phiz;

    double ay = 1. / (1. + phiy);
    double by = phiy * ay;
    double az = 1. / (1. + phiz);
    double bz = phiz * az;


    double k12 = Rlaw(0, 1);
    double k13 = Rlaw(0, 2);
    double k14 = Rlaw(0, 3);
    double k15 = Rlaw(0, 4);
    double k16 = Rlaw(0, 5);
    double k23 = Rlaw(1, 2);
    double k24 = Rlaw(1, 3);
    double k25 = Rlaw(1, 4);
    double k26 = Rlaw(1, 5);
    double k34 = Rlaw(2, 3);
    double k35 = Rlaw(2, 4);
    double k36 = Rlaw(2, 5);
    double k45 = Rlaw(3, 4);
    double k46 = Rlaw(3, 5);
    double k56 = Rlaw(4, 5);

    Rm(0, 0) = rEA / L;
    Rm(0, 1) = by * k12 / L;
    Rm(0, 2) = bz * k13 / L;
    Rm(0, 3) = k14 / L;
    Rm(0, 4) = -bz * k13 / 2. + k15 / L;
    Rm(0, 5) = by * k12 / 2. + k16 / L;
    Rm(0, 6) = -rEA / L;
    Rm(0, 7) = -by * k12 / L;
    Rm(0, 8) = -bz * k13 / L;
    Rm(0, 9) = -k14 / L;
    Rm(0, 10) = -bz * k13 / 2. - k15 / L;
    Rm(0, 11) = by * k12 / 2. - k16 / L;
    Rm(1, 1) = 12. * rEIzz * ay / LLL;
    Rm(1, 2) = (by * bz * k23 * LL - 12. * ay * az * k56) / LLL;
    Rm(1, 3) = by * k24 / L;
    Rm(1, 4) = (L * by * k25 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Rm(1, 5) = 6. * rEIzz * ay / LL + k26 * by / L;
    Rm(1, 6) = -by * k12 / L;
    Rm(1, 7) = -12. * rEIzz * ay / LLL;
    Rm(1, 8) = -Rm(1, 2);
    Rm(1, 9) = -by * k24 / L;
    Rm(1, 10) = (-L * by * k25 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Rm(1, 11) = 6. * rEIzz * ay / LL - k26 * by / L;
    Rm(2, 2) = 12. * rEIyy * az / LLL;
    Rm(2, 3) = bz * k34 / L;
    Rm(2, 4) = k35 * bz / L - 6. * rEIyy * az / LL;
    Rm(2, 5) = (L * bz * k36 - 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Rm(2, 6) = -bz * k13 / L;
    Rm(2, 7) = (-by * bz * k23 * LL + 12. * ay * az * k56) / LLL;
    Rm(2, 8) = -12. * rEIyy * az / LLL;
    Rm(2, 9) = -bz * k34 / L;
    Rm(2, 10) = -6. * rEIyy * az / LL - k35 * bz / L;
    Rm(2, 11) = -(L * bz * k36 + 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Rm(3, 3) = rGJ / L;
    Rm(3, 4) = k45 / L - bz * k34 / 2.;
    Rm(3, 5) = k46 / L + by * k24 / 2.;
    Rm(3, 6) = -k14 / L;
    Rm(3, 7) = -by * k24 / L;
    Rm(3, 8) = -bz * k34 / L;
    Rm(3, 9) = -rGJ / L;
    Rm(3, 10) = -k45 / L - bz * k34 / 2.;
    Rm(3, 11) = -k46 / L + by * k24 / 2.;
    Rm(4, 4) = rEIyy * (4. + phiz) * az / L - bz * k35;
    Rm(4, 5) = -L * by * bz * k23 / 4. + by * k25 / 2. - bz * k36 / 2. + (3. * ay * az + 1.) * k56 / L;
    Rm(4, 6) = bz * k13 / 2. - k15 / L;
    Rm(4, 7) = by * bz * k23 / 2. - (L * by * k25 + 6. * ay * az * k56) / LL;
    Rm(4, 8) = 6. * rEIyy * az / LL - k35 * bz / L;
    Rm(4, 9) = bz * k34 / 2. - k45 / L;
    Rm(4, 10) = rEIyy * (2. - phiz) * az / L;
    Rm(4, 11) = -L * by * bz * k23 / 4. + by * k25 / 2. + bz * k36 / 2. + (3. * ay * az - 1.) * k56 / L;
    Rm(5, 5) = rEIzz * (4. + phiy) * ay / L + by * k26;
    Rm(5, 6) = -by * k12 / 2. - k16 / L;
    Rm(5, 7) = -6. * rEIzz * ay / LL - k26 * by / L;
    Rm(5, 8) = -(L * bz * k36 - 6. * ay * az * k56) / LL - by * bz * k23 / 2.;
    Rm(5, 9) = -by * k24 / 2. - k46 / L;
    Rm(5, 10) = (3. * ay * az - 1.) * k56 / L - by * k25 / 2. - bz * k36 / 2. - L * bz * by * k23 / 4.;
    Rm(5, 11) = rEIzz * (2. - phiy) * ay / L;
    Rm(6, 6) = rEA / L;
    Rm(6, 7) = by * k12 / L;
    Rm(6, 8) = bz * k13 / L;
    Rm(6, 9) = k14 / L;
    Rm(6, 10) = bz * k13 / 2. + k15 / L;
    Rm(6, 11) = k16 / L - by * k12 / 2.;
    Rm(7, 7) = 12. * rEIzz * ay / LLL;
    Rm(7, 8) = (by * bz * k23 * LL - 12. * ay * az * k56) / LLL;
    Rm(7, 9) = by * k24 / L;
    Rm(7, 10) = (L * by * k25 - 6. * ay * az * k56) / LL + (by * bz * k23) / 2.;
    Rm(7, 11) = -6. * rEIzz * ay / LL + k26 * by / L;
    Rm(8, 8) = 12. * rEIyy * az / LLL;
    Rm(8, 9) = bz * k34 / L;
    Rm(8, 10) = 6. * rEIyy * az / LL + k35 * bz / L;
    Rm(8, 11) = (L * bz * k36 + 6. * ay * az * k56) / LL - (by * bz * k23) / 2.;
    Rm(9, 9) = rGJ / L;
    Rm(9, 10) = bz * k34 / 2. + k45 / L;
    Rm(9, 11) = k46 / L - by * k24 / 2.;
    Rm(10, 10) = bz * k35 + rEIyy * (4. + phiz) * az / L;
    Rm(10, 11) = bz * k36 / 2. - by * k25 / 2. - L * by * bz * k23 / 4. + (3. * ay * az + 1.) * k56 / L;
    Rm(11, 11) = rEIzz * (4. + phiy) * ay / L - by * k26;

    // symmetric part;
    for (int r = 0; r < 12; r++)
        for (int c = 0; c < r; c++)
            Rm(r, c) = Rm(c, r);

    Rm = this->T.transpose() * Rm * this->T;
}


void ChElementBeamTaperedTimoshenkoFPM::SetupInitial(ChSystem* system) {
    assert(taperedSectionFPM);

    // Compute rest length, mass:
    this->length = (nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos()).Length();
    this->mass = this->length / 2 * this->taperedSectionFPM->GetSectionA()->GetMassPerUnitLength() +
                 this->length / 2 * this->taperedSectionFPM->GetSectionB()->GetMassPerUnitLength();

    // Compute initial rotation
    ChMatrix33<> A0;
    ChVector<> mXele = nodes[1]->GetX0().GetPos() - nodes[0]->GetX0().GetPos();
    ChVector<> myele = nodes[0]->GetX0().GetA().Get_A_Yaxis();
    A0.Set_A_Xdir(mXele, myele);
    q_element_ref_rot = A0.Get_A_quaternion();


    // Compute local mass matrix
    // It could be lumped or consistent mass matrix, depends on SetLumpedMassMatrix(true/false)
    // If it is lumped mass matrix, you need to multiple 0.5 * length to obtain the final mass matrix
    // For consistent mass matrix, don't need to multiple anything.
    this->taperedSectionFPM->ComputeInertiaMatrix(this->M);

    // Compute transformation matrix
    ComputeTransformMatrix();

    // Compute local stiffness matrix:
    ComputeStiffnessMatrix();

    // Compute local geometric stiffness matrix normalized by pull force P: Kg/P
    ComputeGeometricStiffnessMatrix();

    // Compute local damping matrix:
    ComputeDampingMatrix();
}

void ChElementBeamTaperedTimoshenkoFPM::EvaluateSectionForceTorque(const double eta,
                                                                ChVector<>& Fforce,
                                                                ChVector<>& Mtorque) {
    assert(taperedSectionFPM);

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

    double EA = this->taperedSectionFPM->GetAverageSectionParameters()->EA;
    double GJ = this->taperedSectionFPM->GetAverageSectionParameters()->GJ;
    double GAyy = this->taperedSectionFPM->GetAverageSectionParameters()->GAyy;
    double GAzz = this->taperedSectionFPM->GetAverageSectionParameters()->GAzz;
    double EIyy = this->taperedSectionFPM->GetAverageSectionParameters()->EIyy;
    double EIzz = this->taperedSectionFPM->GetAverageSectionParameters()->EIzz;
    
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
        // 6*6 fully populated constitutive matrix of the beam:
        ChMatrixNM<double, 6, 6> Klaw_d = this->taperedSectionFPM->GetAverageFPM();
        if (use_shear_stain == false) {
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


}  // end namespace fea
}  // end namespace chrono
