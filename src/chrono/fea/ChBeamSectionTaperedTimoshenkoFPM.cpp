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

#include "chrono/fea/ChBeamSectionTaperedTimoshenkoFPM.h"

namespace chrono {
namespace fea {

void ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::ComputeAverageFPM() {
    // the elements off the diagonal are averaged by arithmetic mean
    this->average_fpm =
        0.5 * (this->section_fpmA->GetStiffnessMatrixFPM() + this->section_fpmB->GetStiffnessMatrixFPM());

    // The diagonal terms are averaged by geometrical mean, to be consistent with previous algorithm
    this->average_fpm(0, 0) = this->avg_sec_par->EA;
    this->average_fpm(1, 1) = this->avg_sec_par->GAyy;
    this->average_fpm(2, 2) = this->avg_sec_par->GAzz;
    this->average_fpm(3, 3) = this->avg_sec_par->GJ;
    this->average_fpm(4, 4) = this->avg_sec_par->EIyy;
    this->average_fpm(5, 5) = this->avg_sec_par->EIzz;
}

ChMatrixNM<double, 6, 6> ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::GetAverageKlaw() {
    ChMatrixNM<double, 6, 6> Klaw_average =
        0.5 * (this->section_fpmA->GetStiffnessMatrixFPM() + this->section_fpmB->GetStiffnessMatrixFPM());
    return Klaw_average;
}

ChMatrixNM<double, 6, 6> ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::GetAverageMlaw() {
    ChMatrixNM<double, 6, 6> Mlaw_average =
        0.5 * (this->section_fpmA->GetMassMatrixFPM() + this->section_fpmB->GetMassMatrixFPM());
    return Mlaw_average;
}

ChMatrixNM<double, 6, 6> ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::GetKlawAtPoint(const double eta) {
    // calculate cross-sectional material stiffness matrix Klaw at dimensionless point eta, by linear interpolation
    // eta = (-1,1)
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);
    ChMatrixNM<double, 6, 6> Klaw_point =
        Nx1 * this->section_fpmA->GetStiffnessMatrixFPM() + Nx2 * this->section_fpmB->GetStiffnessMatrixFPM();

    return Klaw_point;
}

ChMatrixNM<double, 6, 6> ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::GetMlawAtPoint(const double eta) {
    // calculate cross-sectional material mass matrix Mlaw at dimensionless point eta, by linear interpolation
    // eta = (-1,1)
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);
    ChMatrixNM<double, 6, 6> Mlaw_point =
        Nx1 * this->section_fpmA->GetMassMatrixFPM() + Nx2 * this->section_fpmB->GetMassMatrixFPM();

    return Mlaw_point;
}

ChMatrixNM<double, 6, 6> ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::GetRlawAtPoint(const double eta) {
    DampingCoefficients rdamping_coeff_A = this->section_fpmA->GetBeamRaleyghDamping();
    DampingCoefficients rdamping_coeff_B = this->section_fpmB->GetBeamRaleyghDamping();
    double artificial_factor_for_shear_damping_A = this->section_fpmA->GetArtificialFactorForShearDamping();
    double artificial_factor_for_shear_damping_B = this->section_fpmB->GetArtificialFactorForShearDamping();

    // linear interpolation
    double Nx1 = (1. / 2.) * (1 - eta);
    double Nx2 = (1. / 2.) * (1 + eta);
    double mbx = Nx1 * rdamping_coeff_A.bx + Nx2 * rdamping_coeff_B.bx;
    double mby = Nx1 * rdamping_coeff_A.by + Nx2 * rdamping_coeff_B.by;
    double mbz = Nx1 * rdamping_coeff_A.bz + Nx2 * rdamping_coeff_B.bz;
    double mbt = Nx1 * rdamping_coeff_A.bt + Nx2 * rdamping_coeff_B.bt;
    double artificial_factor_for_shear_damping =
        Nx1 * artificial_factor_for_shear_damping_A + Nx2 * artificial_factor_for_shear_damping_B;

    ChMatrixNM<double, 6, 6> mb;
    mb.setIdentity();
    mb(0, 0) = mbx;
    mb(1, 1) = mby * artificial_factor_for_shear_damping;
    mb(2, 2) = mbz * artificial_factor_for_shear_damping;
    mb(3, 3) = mbt;
    mb(4, 4) = mbz;
    mb(5, 5) = mby;

    ChMatrixNM<double, 6, 6> Klaw_point = this->GetKlawAtPoint(eta);
    ChMatrixNM<double, 6, 6> Rlaw_point = mb.transpose() * Klaw_point * mb;  // material damping matrix

    return Rlaw_point;
}

void ChBeamSectionTaperedTimoshenkoAdvancedGenericFPM::ComputeInertiaMatrix(ChMatrixDynamic<>& M) {
    // Inherit from base class
    ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeInertiaMatrix(M);

    // FPM doesnot influence the Inertia(/Mass) matrix of beam element,
    // so it can be evaluated after calculating the inertia matrix
    ComputeAverageFPM();
}

}  // end namespace fea
}  // end namespace chrono
