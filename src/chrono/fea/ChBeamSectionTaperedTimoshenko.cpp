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

#include "chrono/fea/ChBeamSectionTaperedTimoshenko.h"

namespace chrono {
namespace fea {

using EigenMat5x1 = Eigen::Matrix<double, 5, 1>;  // ChMatrixNM<double, 5, 1> cannot be used directly, building error
using EigenMat5x5 = Eigen::Matrix<double, 5, 5>;  // ChMatrixNM<double, 5, 5> cannot be used directly, building error

void ChBeamSectionTimoshenkoAdvancedGeneric::SetInertiasPerUnitLength(const double mJyy,
                                                                      const double mJzz,
                                                                      const double mJyz,
                                                                      const double mQy,
                                                                      const double mQz) {
    this->Jyy = mJyy;
    this->Jzz = mJzz;
    this->Jyz = mJyz;
    // automatically set parent Jxx value
    this->Jxx = (this->Jyy + this->Jzz);

    this->Qy = mQy;
    this->Qz = mQz;
}

void ChBeamSectionTimoshenkoAdvancedGeneric::SetMainInertiasInMassReference(const double Jmyy,
                                                                            const double Jmzz,
                                                                            const double Jmyz,
                                                                            const double mass_phi,
                                                                            const double Qmy,
                                                                            const double Qmz) {
    // define a vector of local mass properties expressed in mass center coordinate system
    EigenMat5x1 Jm_vec;
    // Jm_vec << Qmy, Qmz, Jmyy, Jmzz, Jmyz;
    Jm_vec(0) = Qmy;
    Jm_vec(1) = Qmz;
    Jm_vec(2) = Jmyy;
    Jm_vec(3) = Jmzz;
    Jm_vec(4) = Jmyz;

    // A constant vector, which is from the mass center offset (My,Mz)
    EigenMat5x1 const_vec;
    const_vec.setZero();
    const_vec(0) = this->mu * this->Mz;
    const_vec(1) = this->mu * this->My;
    const_vec(2) = this->mu * this->Mz * this->Mz;
    const_vec(3) = this->mu * this->My * this->My;
    const_vec(4) = this->mu * this->My * this->Mz;

    // Transformation matrix from the mass center coordinate system to the centerline of beam
    EigenMat5x5 Acog2cl;
    Acog2cl.setZero();
    double cosphi = cos(mass_phi);
    double sinphi = sin(mass_phi);
    double cos2phi = cos(2. * mass_phi);
    double sin2phi = sin(2. * mass_phi);
    double cosphi2 = cosphi * cosphi;
    double sinphi2 = sinphi * sinphi;

    Acog2cl(0, 0) = cosphi;
    Acog2cl(0, 1) = sinphi;

    Acog2cl(1, 0) = -sinphi;
    Acog2cl(1, 1) = cosphi;

    Acog2cl(2, 0) = 2. * this->Mz * cosphi;
    Acog2cl(2, 1) = 2. * this->Mz * sinphi;
    Acog2cl(2, 2) = cosphi2;
    Acog2cl(2, 3) = sinphi2;
    Acog2cl(2, 4) = sin2phi;

    Acog2cl(3, 0) = -2. * this->My * sinphi;
    Acog2cl(3, 1) = 2. * this->My * cosphi;
    Acog2cl(3, 2) = sinphi2;
    Acog2cl(3, 3) = cosphi2;
    Acog2cl(3, 4) = -sin2phi;

    Acog2cl(4, 0) = this->My * cosphi - this->Mz * sinphi;
    Acog2cl(4, 1) = this->Mz * cosphi + this->My * sinphi;
    Acog2cl(4, 2) = -1. / 2. * sin2phi;
    Acog2cl(4, 3) = 1. / 2. * sin2phi;
    Acog2cl(4, 4) = cos2phi;

    EigenMat5x1 J_vec;
    J_vec.setZero();
    J_vec = const_vec + Acog2cl * Jm_vec;

    this->Qy = J_vec(0);
    this->Qz = J_vec(1);
    this->Jyy = J_vec(2);
    this->Jzz = J_vec(3);
    this->Jyz = J_vec(4);
    // Automatically set parent Jxx value
    this->Jxx = (this->Jyy + this->Jzz);
}

void ChBeamSectionTimoshenkoAdvancedGeneric::GetMainInertiasInMassReference(double& Jmyy,
                                                                            double& Jmzz,
                                                                            double& Jmyz,
                                                                            double& mass_phi,
                                                                            double& Qmy,
                                                                            double& Qmz) {
    // inherit the algorithm from Rayleigh beam class, ignoring Qy,Qz to solve the mass_phi approximately.
    // remove inertia transport
    double Tyy_rot = this->Jyy - this->mu * this->Mz * this->Mz;
    double Tzz_rot = this->Jzz - this->mu * this->My * this->My;
    double Tyz_rot = -this->Jyz + this->mu * this->Mz * this->My;
    // tensor de-rotation up to principal axes
    double argum = pow((Tyy_rot - Tzz_rot) * 0.5, 2) + pow(Tyz_rot, 2);
    if (argum <= 0) {
        mass_phi = 0;
        // Jmyy = 0.5 * (Tzz_rot + Tyy_rot);
        // Jmzz = 0.5 * (Tzz_rot + Tyy_rot);
        // return;
    } else {
        double discr = sqrt(pow((Tyy_rot - Tzz_rot) * 0.5, 2) + pow(Tyz_rot, 2));
        mass_phi = -0.5 * atan2(Tyz_rot / discr, (Tzz_rot - Tyy_rot) / (2. * discr));
        // Jmyy = 0.5 * (Tzz_rot + Tyy_rot) - discr;
        // Jmzz = 0.5 * (Tzz_rot + Tyy_rot) + discr;
    }

    // a constant vector, which is from the mass center offset (My,Mz)
    EigenMat5x1 const_vec;
    const_vec.setZero();
    const_vec(0) = this->mu * this->Mz;
    const_vec(1) = this->mu * this->My;
    const_vec(2) = this->mu * this->Mz * this->Mz;
    const_vec(3) = this->mu * this->My * this->My;
    const_vec(4) = this->mu * this->My * this->Mz;

    // transformation matrix from the centerline of beam to the mass center coordinate system
    EigenMat5x5 Acl2cog;
    Acl2cog.setZero();
    double cosphi = cos(mass_phi);
    double sinphi = sin(mass_phi);
    double cos2phi = cos(2. * mass_phi);
    double sin2phi = sin(2. * mass_phi);
    double cosphi2 = cosphi * cosphi;
    double sinphi2 = sinphi * sinphi;

    Acl2cog(0, 0) = cosphi;
    Acl2cog(0, 1) = -sinphi;

    Acl2cog(1, 0) = sinphi;
    Acl2cog(1, 1) = cosphi;

    Acl2cog(2, 0) = this->My * sin2phi - 2. * this->Mz * cosphi2;
    Acl2cog(2, 1) = -2. * this->My * sinphi2 + this->Mz * sin2phi;
    Acl2cog(2, 2) = cosphi2;
    Acl2cog(2, 3) = sinphi2;
    Acl2cog(2, 4) = -sin2phi;

    Acl2cog(3, 0) = -this->My * sin2phi - 2. * this->Mz * sinphi2;
    Acl2cog(3, 1) = -2. * this->My * cosphi2 - this->Mz * sin2phi;
    Acl2cog(3, 2) = sinphi2;
    Acl2cog(3, 3) = cosphi2;
    Acl2cog(3, 4) = sin2phi;

    Acl2cog(4, 0) = -this->My * cos2phi - this->Mz * sin2phi;
    Acl2cog(4, 1) = this->My * sin2phi - this->Mz * cos2phi;
    Acl2cog(4, 2) = 1. / 2. * sin2phi;
    Acl2cog(4, 3) = -1. / 2. * sin2phi;
    Acl2cog(4, 4) = cos2phi;

    // a vector of mass properties expressed in the centerline of beam
    EigenMat5x1 J_vec;
    J_vec.setZero();

    J_vec(0) = this->Qy;
    J_vec(1) = this->Qz;
    J_vec(2) = this->Jyy;
    J_vec(3) = this->Jzz;
    J_vec(4) = this->Jyz;

    // define a vector of local mass properties expressed in mass center coordinate system
    EigenMat5x1 Jm_vec;
    Jm_vec.setZero();
    Jm_vec = Acl2cog * (J_vec - const_vec);

    Qmy = Jm_vec(0);
    Qmz = Jm_vec(1);
    Jmyy = Jm_vec(2);
    Jmzz = Jm_vec(3);
    Jmyz = Jm_vec(4);
}

// Three Lambda functions to evalute the average parameters conveniently.
auto GetAverageValue = [](const double mv1, const double mv2) { return (mv1 + mv2) / 2.0; };

// For more information, please refer to ANSYS theory document in the chapters of tapered beam element.
auto GetAverageValue3 = [](const double mv1, const double mv2) {
    if (mv1 * mv2 < 0.) {
        // GetLog() << "WARNING: negative value, error!\n";
        return GetAverageValue(mv1, mv2);
    }
    return (mv1 + pow(mv1 * mv2, 0.5) + mv2) / 3.0;
};
// For more information, please refer to ANSYS theory document in the chapters of tapered beam element.
auto GetAverageValue5 = [](const double mv1, const double mv2) {
    if (mv1 * mv2 < 0.) {
        // GetLog() << "WARNING: negative value, error!\n";
        return GetAverageValue(mv1, mv2);
    }
    return (mv1 + pow(mv1 * mv1 * mv1 * mv2, 0.25) + pow(mv1 * mv2, 0.5) + pow(mv1 * mv2 * mv2 * mv2, 0.25) + mv2) /
           5.0;
};

void ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeAverageSectionParameters() {
    if (compute_ave_sec_par) {
        return;
    }

    double mu1 = this->sectionA->GetMassPerUnitLength();
    // double Jxx1 = this->sectionA->GetInertiaJxxPerUnitLength();
    double Jyy1 = this->sectionA->GetInertiaJyyPerUnitLength();
    double Jzz1 = this->sectionA->GetInertiaJzzPerUnitLength();
    double Jyz1 = this->sectionA->GetInertiaJyzPerUnitLength();
    double Qy1 = this->sectionA->GetInertiaQyPerUnitLength();
    double Qz1 = this->sectionA->GetInertiaQzPerUnitLength();
    // double Jmxx1 = this->sectionA->GetInertiaJxxPerUnitLengthInMassReference();
    double EA1 = this->sectionA->GetAxialRigidity();
    double GJ1 = this->sectionA->GetXtorsionRigidity();
    double EIyy1 = this->sectionA->GetYbendingRigidity();
    double EIzz1 = this->sectionA->GetZbendingRigidity();
    double GAyy1 = this->sectionA->GetYshearRigidity();
    double GAzz1 = this->sectionA->GetZshearRigidity();
    double alpha1 = this->sectionA->GetSectionRotation();
    double Cy1 = this->sectionA->GetCentroidY();
    double Cz1 = this->sectionA->GetCentroidZ();
    double Sy1 = this->sectionA->GetShearCenterY();
    double Sz1 = this->sectionA->GetShearCenterZ();
    double My1 = this->sectionA->GetCenterOfMassY();
    double Mz1 = this->sectionA->GetCenterOfMassZ();
    double mass_phi1;
    double Jmyy1;
    double Jmzz1;
    double Jmyz1;
    double Qmy1;
    double Qmz1;
    this->sectionA->GetMainInertiasInMassReference(Jmyy1, Jmzz1, Jmyz1, mass_phi1, Qmy1, Qmz1);
    // double Jmxx1 = Jmyy1 + Jmzz1;
    // rotate the bending and shear stiffnesses from elastic axis to mass axis
    double cosphi1 = cos(mass_phi1);
    double sinphi1 = sin(mass_phi1);
    double EImyy1 = EIyy1 * cosphi1 + EIzz1 * sinphi1;
    double EImzz1 = -EIyy1 * sinphi1 + EIzz1 * cosphi1;
    double GAmyy1 = GAyy1 * cosphi1 + GAzz1 * sinphi1;
    double GAmzz1 = -GAyy1 * sinphi1 + GAzz1 * cosphi1;
    DampingCoefficients rdamping_coeff1 = this->sectionA->GetBeamRaleyghDamping();
    double artificial_factor_for_shear_damping1 = this->sectionA->GetArtificialFactorForShearDamping();

    double mu2 = this->sectionB->GetMassPerUnitLength();
    // double Jxx2 = this->sectionB->GetInertiaJxxPerUnitLength();
    double Jyy2 = this->sectionB->GetInertiaJyyPerUnitLength();
    double Jzz2 = this->sectionB->GetInertiaJzzPerUnitLength();
    double Jyz2 = this->sectionB->GetInertiaJyzPerUnitLength();
    double Qy2 = this->sectionB->GetInertiaQyPerUnitLength();
    double Qz2 = this->sectionB->GetInertiaQzPerUnitLength();
    // double Jmxx2 = this->sectionB->GetInertiaJxxPerUnitLengthInMassReference();
    double EA2 = this->sectionB->GetAxialRigidity();
    double GJ2 = this->sectionB->GetXtorsionRigidity();
    double EIyy2 = this->sectionB->GetYbendingRigidity();
    double EIzz2 = this->sectionB->GetZbendingRigidity();
    double GAyy2 = this->sectionB->GetYshearRigidity();
    double GAzz2 = this->sectionB->GetZshearRigidity();
    double alpha2 = this->sectionB->GetSectionRotation();
    double Cy2 = this->sectionB->GetCentroidY();
    double Cz2 = this->sectionB->GetCentroidZ();
    double Sy2 = this->sectionB->GetShearCenterY();
    double Sz2 = this->sectionB->GetShearCenterZ();
    double My2 = this->sectionB->GetCenterOfMassY();
    double Mz2 = this->sectionB->GetCenterOfMassZ();
    double mass_phi2;
    double Jmyy2;
    double Jmzz2;
    double Jmyz2;
    double Qmy2;
    double Qmz2;
    this->sectionB->GetMainInertiasInMassReference(Jmyy2, Jmzz2, Jmyz2, mass_phi2, Qmy2, Qmz2);
    // double Jmxx2 = Jmyy2 + Jmzz2;
    // rotate the bending and shear stiffnesses from elastic axis to mass axis
    double cosphi2 = cos(mass_phi2);
    double sinphi2 = sin(mass_phi2);
    double EImyy2 = EIyy2 * cosphi2 + EIzz2 * sinphi2;
    double EImzz2 = -EIyy2 * sinphi2 + EIzz2 * cosphi2;
    double GAmyy2 = GAyy2 * cosphi2 + GAzz2 * sinphi2;
    double GAmzz2 = -GAyy2 * sinphi2 + GAzz2 * cosphi2;
    DampingCoefficients rdamping_coeff2 = this->sectionB->GetBeamRaleyghDamping();
    double artificial_factor_for_shear_damping2 = this->sectionB->GetArtificialFactorForShearDamping();

    double L = this->GetLength();
    double LL = L * L;

    this->avg_sec_par->mu = GetAverageValue(mu1, mu2);
    this->avg_sec_par->alpha = GetAverageValue(alpha1, alpha2);
    this->avg_sec_par->Cy = GetAverageValue(Cy1, Cy2);
    this->avg_sec_par->Cz = GetAverageValue(Cz1, Cz2);
    this->avg_sec_par->Sy = GetAverageValue(Sy1, Sy2);
    this->avg_sec_par->Sz = GetAverageValue(Sz1, Sz2);
    this->avg_sec_par->My = GetAverageValue(My1, My2);
    this->avg_sec_par->Mz = GetAverageValue(Mz1, Mz2);

    this->avg_sec_par->Jyy = GetAverageValue5(Jyy1, Jyy2);
    this->avg_sec_par->Jzz = GetAverageValue5(Jzz1, Jzz2);
    this->avg_sec_par->Jyz = GetAverageValue5(Jyz1, Jyz2);  // Jyz may be negative
    this->avg_sec_par->Jxx = this->avg_sec_par->Jyy + this->avg_sec_par->Jzz;
    this->avg_sec_par->Qy = GetAverageValue3(Qy1, Qy2);  // Qy may be negative
    this->avg_sec_par->Qz = GetAverageValue3(Qz1, Qz2);  // Qz may be negative

    this->avg_sec_par->mass_phi = GetAverageValue(mass_phi1, mass_phi2);
    this->avg_sec_par->Jmyy = GetAverageValue5(Jmyy1, Jmyy2);
    this->avg_sec_par->Jmzz = GetAverageValue5(Jmzz1, Jmzz2);
    this->avg_sec_par->Jmyz = GetAverageValue5(Jmyz1, Jmyz2);  // Jyz may be negative
    this->avg_sec_par->Jmxx = this->avg_sec_par->Jmyy + this->avg_sec_par->Jmzz;
    this->avg_sec_par->Qmy = GetAverageValue3(Qmy1, Qmy2);  // Qy may be negative
    this->avg_sec_par->Qmz = GetAverageValue3(Qmz1, Qmz2);  // Qz may be negative

    this->avg_sec_par->EA = GetAverageValue3(EA1, EA2);
    this->avg_sec_par->GJ = GetAverageValue3(GJ1, GJ2);
    this->avg_sec_par->GAyy = GetAverageValue5(GAyy1, GAyy2);
    this->avg_sec_par->GAzz = GetAverageValue5(GAzz1, GAzz2);
    this->avg_sec_par->EIyy = GetAverageValue5(EIyy1, EIyy2);
    this->avg_sec_par->EIzz = GetAverageValue5(EIzz1, EIzz2);

    this->avg_sec_par->GAmyy = GetAverageValue5(GAmyy1, GAmyy2);
    this->avg_sec_par->GAmzz = GetAverageValue5(GAmzz1, GAmzz2);
    this->avg_sec_par->EImyy = GetAverageValue5(EImyy1, EImyy2);
    this->avg_sec_par->EImzz = GetAverageValue5(EImzz1, EImzz2);

    this->avg_sec_par->rdamping_coeff.bx = GetAverageValue(rdamping_coeff1.bx, rdamping_coeff2.bx);
    this->avg_sec_par->rdamping_coeff.by = GetAverageValue(rdamping_coeff1.by, rdamping_coeff2.by);
    this->avg_sec_par->rdamping_coeff.bz = GetAverageValue(rdamping_coeff1.bz, rdamping_coeff2.bz);
    this->avg_sec_par->rdamping_coeff.bt = GetAverageValue(rdamping_coeff1.bt, rdamping_coeff2.bt);
    this->avg_sec_par->rdamping_coeff.alpha = GetAverageValue(rdamping_coeff1.alpha, rdamping_coeff2.alpha);
    this->avg_sec_par->artificial_factor_for_shear_damping =
        GetAverageValue(artificial_factor_for_shear_damping1, artificial_factor_for_shear_damping2);

    this->avg_sec_par->phimy =
        0.;  // If the shear stiffness of section is not input by user, the shear deformation effect is ignored
    this->avg_sec_par->phimz =
        0.;  // If the shear stiffness of section is not input by user, the shear deformation effect is ignored
    double eps = 0.01;
    if (abs(this->avg_sec_par->GAmyy) > eps) {  // avoid dividing zero
        this->avg_sec_par->phimy = 12.0 * this->avg_sec_par->EImzz / (this->avg_sec_par->GAmyy * LL);
    }
    if (abs(this->avg_sec_par->GAmzz) > eps) {  // avoid dividing zero
        this->avg_sec_par->phimz = 12.0 * this->avg_sec_par->EImyy / (this->avg_sec_par->GAmzz * LL);
    }

    this->avg_sec_par->phiy =
        0.;  // If the shear stiffness of section is not input by user, the shear deformation effect is ignored
    this->avg_sec_par->phiz =
        0.;  // If the shear stiffness of section is not input by user, the shear deformation effect is ignored
    if (abs(this->avg_sec_par->GAyy) > eps) {  // avoid dividing zero
        this->avg_sec_par->phiy = 12.0 * this->avg_sec_par->EIzz / (this->avg_sec_par->GAyy * LL);
    }
    if (abs(this->avg_sec_par->GAzz) > eps) {  // avoid dividing zero
        this->avg_sec_par->phiz = 12.0 * this->avg_sec_par->EIyy / (this->avg_sec_par->GAzz * LL);
    }
    
    // update the status of lock, to avoid computing again.
    compute_ave_sec_par = true;
}

void ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeLumpedInertiaMatrix(ChMatrixNM<double, 12, 12>& M) {
    M.setZero(12, 12);

    ChMatrixNM<double, 6, 6> MA;
    ChMatrixNM<double, 6, 6> MB;
    this->sectionA->ComputeInertiaMatrix(MA);
    this->sectionB->ComputeInertiaMatrix(MB);

    M.block<6, 6>(0, 0) = MA;
    M.block<6, 6>(6, 6) = MB;
}

void ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeSimpleConsistentInertiaMatrix(
    ChMatrixNM<double, 12, 12>& M) {
    M.setZero();
    double mu1 = this->sectionA->GetMassPerUnitLength();
    double My1 = this->sectionA->GetCenterOfMassY();
    double Mz1 = this->sectionA->GetCenterOfMassZ();
    double mass_phi1;
    double Jmzz1;
    double Jmyy1;
    double Jmyz1;
    double Qmy1;
    double Qmz1;
    this->sectionA->GetMainInertiasInMassReference(Jmyy1, Jmzz1, Jmyz1, mass_phi1, Qmy1, Qmz1);
    double Jmxx1 = Jmyy1 + Jmzz1;

    double mu2 = this->sectionB->GetMassPerUnitLength();
    double My2 = this->sectionB->GetCenterOfMassY();
    double Mz2 = this->sectionB->GetCenterOfMassZ();
    double mass_phi2;
    double Jmzz2;
    double Jmyy2;
    double Jmyz2;
    double Qmy2;
    double Qmz2;
    this->sectionB->GetMainInertiasInMassReference(Jmyy2, Jmzz2, Jmyz2, mass_phi2, Qmy2, Qmz2);
    double Jmxx2 = Jmyy2 + Jmzz2;

    double L = this->GetLength();
    double LL = L * L;
    double phiy = this->avg_sec_par->phimy;
    double phiz = this->avg_sec_par->phimz;
    double mu = this->avg_sec_par->mu;
    double Jmxx = this->avg_sec_par->Jmxx;

    // The radii of gyration ry,rz are:
    // double ry = pow(Iyy / A, 0.5);
    // double rz = pow(Izz / A, 0.5);
    // We have: Iyy / A == Iyy * pho / (A * pho) = Jyy / mu
    // For wind turbine blade, the above equations are not true, but can be a good approximation.
    // double ry = pow(Jmyy / mu, 0.5);   // wrong modal results, error up to 5%
    // double rz = pow(Jmzz / mu, 0.5);   // wrong modal results, error up to 5%

    // double ry = pow(EImyy / EA, 0.5);  // run error
    // double rz = pow(EImzz / EA, 0.5);  // run error

    // Note: bending inertia has to be switched off!
    double ry = 0;  // Only this setting could give correct modal results
    double rz = 0;  // But, why? TODO: need an explanation

    double phiy2 = phiy * phiy;
    double phiz2 = phiz * phiz;
    double ry_L2 = pow(ry / L, 2.0);
    double rz_L2 = pow(rz / L, 2.0);
    double oneplusphiy2 = pow(1 + phiy, 2.0);
    double oneplusphiz2 = pow(1 + phiz, 2.0);
    double mAz = (13. / 35. + 7. / 10. * phiy + 1. / 3. * phiy2 + 6. / 5. * rz_L2) / oneplusphiy2;
    double mAy = (13. / 35. + 7. / 10. * phiz + 1. / 3. * phiz2 + 6. / 5. * ry_L2) / oneplusphiz2;
    double mBy = (9. / 70. + 3. / 10. * phiy + 1. / 6. * phiy2 - 6. / 5. * rz_L2) / oneplusphiy2;
    double mBz = (9. / 70. + 3. / 10. * phiz + 1. / 6. * phiz2 - 6. / 5. * ry_L2) / oneplusphiz2;
    double mCy =
        (11. / 210. + 11. / 120. * phiy + 1. / 24. * phiy2 + (1. / 10. - 1. / 2. * phiy) * rz_L2) * L / oneplusphiy2;
    double mCz =
        (11. / 210. + 11. / 120. * phiz + 1. / 24. * phiz2 + (1. / 10. - 1. / 2. * phiz) * ry_L2) * L / oneplusphiz2;
    double mDy =
        (13. / 420. + 3. / 40. * phiy + 1. / 24. * phiy2 - (1. / 10. - 1. / 2. * phiy) * rz_L2) * L / oneplusphiy2;
    double mDz =
        (13. / 420. + 3. / 40. * phiz + 1. / 24. * phiz2 - (1. / 10. - 1. / 2. * phiz) * ry_L2) * L / oneplusphiz2;
    double mEy =
        (1. / 105. + 1. / 60. * phiy + 1. / 120. * phiy2 + (2. / 15. + 1. / 6. * phiy + 1. / 3. * phiy2) * rz_L2) * LL /
        oneplusphiy2;
    double mEz =
        (1. / 105. + 1. / 60. * phiz + 1. / 120. * phiz2 + (2. / 15. + 1. / 6. * phiz + 1. / 3. * phiz2) * ry_L2) * LL /
        oneplusphiz2;
    double mFy =
        (1. / 140. + 1. / 60. * phiy + 1. / 120. * phiy2 + (1. / 30. + 1. / 6. * phiy - 1. / 6. * phiy2) * rz_L2) * LL /
        oneplusphiy2;
    double mFz =
        (1. / 140. + 1. / 60. * phiz + 1. / 120. * phiz2 + (1. / 30. + 1. / 6. * phiz - 1. / 6. * phiz2) * ry_L2) * LL /
        oneplusphiz2;

    double mt1 = mu1 * L;
    double mt2 = mu2 * L;
    double mt = mu * L;

    M(0, 0) = mt1 / 3.0;
    M(6, 6) = mt2 / 3.0;
    M(6, 0) = mt / 6.0;
    M(0, 6) = M(6, 0);

    M(3, 3) = Jmxx1 * L / 3.0;
    M(9, 9) = Jmxx2 * L / 3.0;
    M(9, 3) = Jmxx * L / 6.0;
    M(3, 9) = M(9, 3);

    M(1, 1) = mt1 * mAz;
    M(2, 2) = mt1 * mAy;
    M(4, 4) = mt1 * mEy;
    M(5, 5) = mt1 * mEz;
    M(7, 7) = mt2 * mAz;
    M(8, 8) = mt2 * mAy;
    M(10, 10) = mt2 * mEy;
    M(11, 11) = mt2 * mEz;

    M(4, 2) = -mt1 * mCy;
    M(5, 1) = mt1 * mCz;
    M(7, 1) = mt * mBz;
    M(8, 2) = mt * mBy;
    M(7, 5) = mt * mDz;
    M(8, 4) = -mt * mDy;
    M(10, 2) = mt * mDy;
    M(11, 1) = -mt * mDz;
    M(10, 4) = -mt * mFy;
    M(11, 5) = -mt * mFz;
    M(10, 8) = mt2 * mCy;
    M(11, 7) = -mt2 * mCz;

    M(2, 4) = M(4, 2);
    M(1, 5) = M(5, 1);
    M(1, 7) = M(7, 1);
    M(2, 8) = M(8, 2);
    M(5, 7) = M(7, 5);
    M(4, 8) = M(8, 4);
    M(2, 10) = M(10, 2);
    M(1, 11) = M(11, 1);
    M(4, 10) = M(10, 4);
    M(5, 11) = M(11, 5);
    M(8, 10) = M(10, 8);
    M(7, 11) = M(11, 7);

    // In case the section is rotated:
    ChMatrix33<> RotsectA;
    RotsectA.Set_A_Rxyz(ChVector<>(mass_phi1, 0, 0));
    ChMatrix33<> RotsectB;
    RotsectB.Set_A_Rxyz(ChVector<>(mass_phi2, 0, 0));
    ChMatrixNM<double, 12, 12> Rotsect;
    Rotsect.setZero();
    Rotsect.block<3, 3>(0, 0) = RotsectA;
    Rotsect.block<3, 3>(3, 3) = RotsectA;
    Rotsect.block<3, 3>(6, 6) = RotsectB;
    Rotsect.block<3, 3>(9, 9) = RotsectB;
    M = Rotsect.transpose() * M * Rotsect;

    // transformation matrix for section A
    ChMatrixNM<double, 6, 6> Tm1;
    Tm1.setIdentity();
    Tm1(0, 4) = Mz1;
    Tm1(0, 5) = -My1;
    Tm1(1, 3) = -Mz1;
    Tm1(2, 3) = My1;

    // transformation matrix for section B
    ChMatrixNM<double, 6, 6> Tm2;
    Tm2.setIdentity();
    Tm2(0, 4) = Mz2;
    Tm2(0, 5) = -My2;
    Tm2(1, 3) = -Mz2;
    Tm2(2, 3) = My2;

    // whole transformation matrix
    ChMatrixNM<double, 12, 12> Tm;
    Tm.setZero();
    Tm.block<6, 6>(0, 0) = Tm1;
    Tm.block<6, 6>(6, 6) = Tm2;

    // do the transformation for mass matrix
    M = Tm.transpose() * M * Tm;
}

void ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeConsistentInertiaMatrix(ChMatrixNM<double, 12, 12>& M) {
    M.setZero();

    double mu1 = this->sectionA->GetMassPerUnitLength();
    double Jxx1 = this->sectionA->GetInertiaJxxPerUnitLength();
    double Jyy1 = this->sectionA->GetInertiaJyyPerUnitLength();
    double Jzz1 = this->sectionA->GetInertiaJzzPerUnitLength();
    double Jyz1 = this->sectionA->GetInertiaJyzPerUnitLength();
    double Qy1 = this->sectionA->GetInertiaQyPerUnitLength();
    double Qz1 = this->sectionA->GetInertiaQzPerUnitLength();

    double mu2 = this->sectionB->GetMassPerUnitLength();
    double Jxx2 = this->sectionB->GetInertiaJxxPerUnitLength();
    double Jyy2 = this->sectionB->GetInertiaJyyPerUnitLength();
    double Jzz2 = this->sectionB->GetInertiaJzzPerUnitLength();
    double Jyz2 = this->sectionB->GetInertiaJyzPerUnitLength();
    double Qy2 = this->sectionB->GetInertiaQyPerUnitLength();
    double Qz2 = this->sectionB->GetInertiaQzPerUnitLength();

    double L = this->GetLength();
    double LL = L * L;
    double LLL = LL * L;
    double mu = this->avg_sec_par->mu;
    double Jyy = this->avg_sec_par->Jyy;
    double Jzz = this->avg_sec_par->Jzz;
    double Jyz = this->avg_sec_par->Jyz;
    double Jxx = this->avg_sec_par->Jxx;
    double Qy = this->avg_sec_par->Qy;
    double Qz = this->avg_sec_par->Qz;
    double phiy = this->avg_sec_par->phimy;
    double phiz = this->avg_sec_par->phimz;

    double phiy2 = phiy * phiy;
    double phiz2 = phiz * phiz;
    double oneplusphiy = 1.0 / (phiy + 1.);
    double oneplusphiz = 1.0 / (phiz + 1.);
    double oneplusphiy2 = oneplusphiy * oneplusphiy;
    double oneplusphiz2 = oneplusphiz * oneplusphiz;
    double oneplusphiyz = oneplusphiy * oneplusphiz;

    double rAy1 =
        6. * Jyy1 / (5. * L) * oneplusphiz2 + mu1 * L * (1. / 3. * phiz2 + 7. / 10. * phiz + 13. / 35.) * oneplusphiz2;
    double rAz1 =
        6. * Jzz1 / (5. * L) * oneplusphiy2 + mu1 * L * (1. / 3. * phiy2 + 7. / 10. * phiy + 13. / 35.) * oneplusphiy2;
    double rAy2 =
        6. * Jyy2 / (5. * L) * oneplusphiz2 + mu2 * L * (1. / 3. * phiz2 + 7. / 10. * phiz + 13. / 35.) * oneplusphiz2;
    double rAz2 =
        6. * Jzz2 / (5. * L) * oneplusphiy2 + mu2 * L * (1. / 3. * phiy2 + 7. / 10. * phiy + 13. / 35.) * oneplusphiy2;
    // double rAy =
    // 6. * Jyy / (5. * L) * oneplusphiz2 + mu * L * (1. / 3. * phiz2 + 7. / 10. * phiz + 13. / 35.) * oneplusphiz2;
    // double rAz =
    // 6. * Jzz / (5. * L) * oneplusphiy2 + mu * L * (1. / 3. * phiy2 + 7. / 10. * phiy + 13. / 35.) * oneplusphiy2;

    double rBy =
        (mu * L * (1. / 6. * phiz2 + 3. / 10. * phiz + 9. / 70.)) * oneplusphiz2 - 6. * Jyy / (5. * L) * oneplusphiz2;
    double rBz =
        (mu * L * (1. / 6. * phiy2 + 3. / 10. * phiy + 9. / 70.)) * oneplusphiy2 - 6. * Jzz / (5. * L) * oneplusphiy2;

    double rCy1 = (mu1 * LL * (1. / 24. * phiz2 + 11. / 120. * phiz + 11. / 210.)) * oneplusphiz2 -
                  (1. / 2. * phiz - 1. / 10.) * Jyy1 * oneplusphiz2;
    double rCz1 = (mu1 * LL * (1. / 24. * phiy2 + 11. / 120. * phiy + 11. / 210.)) * oneplusphiy2 -
                  (1. / 2. * phiy - 1. / 10.) * Jzz1 * oneplusphiy2;
    double rCy2 = (mu2 * LL * (1. / 24. * phiz2 + 11. / 120. * phiz + 11. / 210.)) * oneplusphiz2 -
                  (1. / 2. * phiz - 1. / 10.) * Jyy2 * oneplusphiz2;
    double rCz2 = (mu2 * LL * (1. / 24. * phiy2 + 11. / 120. * phiy + 11. / 210.)) * oneplusphiy2 -
                  (1. / 2. * phiy - 1. / 10.) * Jzz2 * oneplusphiy2;
    // double rCy = (mu * LL * (1. / 24. * phiz2 + 11. / 120. * phiz + 11. / 210.)) * oneplusphiz2 -
    //(1. / 2. * phiz - 1. / 10.) * Jyy * oneplusphiz2;
    // double rCz = (mu * LL * (1. / 24. * phiy2 + 11. / 120. * phiy + 11. / 210.)) * oneplusphiy2 -
    //(1. / 2. * phiy - 1. / 10.) * Jzz * oneplusphiy2;

    double rDy = (mu * LL * (1. / 24. * phiz2 + 3. / 40. * phiz + 13. / 420.)) * oneplusphiz2 +
                 (1. / 2. * phiz - 1. / 10.) * Jyy * oneplusphiz2;
    double rDz = (mu * LL * (1. / 24. * phiy2 + 3. / 40. * phiy + 13. / 420.)) * oneplusphiy2 +
                 (1. / 2. * phiy - 1. / 10.) * Jzz * oneplusphiy2;

    double rEy1 = mu1 * LLL * (1. / 120. * phiz2 + 1. / 60. * phiz + 1. / 105.) * oneplusphiz2 +
                  L * (1. / 3. * phiz2 + 1. / 6. * phiz + 2. / 15.) * Jyy1 * oneplusphiz2;
    double rEz1 = mu1 * LLL * (1. / 120. * phiy2 + 1. / 60. * phiy + 1. / 105.) * oneplusphiy2 +
                  L * (1. / 3. * phiy2 + 1. / 6. * phiy + 2. / 15.) * Jzz1 * oneplusphiy2;
    double rEy2 = mu2 * LLL * (1. / 120. * phiz2 + 1. / 60. * phiz + 1. / 105.) * oneplusphiz2 +
                  L * (1. / 3. * phiz2 + 1. / 6. * phiz + 2. / 15.) * Jyy2 * oneplusphiz2;
    double rEz2 = mu2 * LLL * (1. / 120. * phiy2 + 1. / 60. * phiy + 1. / 105.) * oneplusphiy2 +
                  L * (1. / 3. * phiy2 + 1. / 6. * phiy + 2. / 15.) * Jzz2 * oneplusphiy2;
    // double rEy = mu * LLL * (1. / 120. * phiz2 + 1. / 60. * phiz + 1. / 105.) * oneplusphiz2 +
    // L * (1. / 3. * phiz2 + 1. / 6. * phiz + 2. / 15.) * Jyy * oneplusphiz2;
    // double rEz = mu * LLL * (1. / 120. * phiy2 + 1. / 60. * phiy + 1. / 105.) * oneplusphiy2 +
    // L * (1. / 3. * phiy2 + 1. / 6. * phiy + 2. / 15.) * Jzz * oneplusphiy2;

    double rFy = (mu * LLL * (1. / 120. * phiz2 + 1. / 60. * phiz + 1. / 140.)) * oneplusphiz2 +
                 L * (-1. / 6. * phiz2 + 1. / 6. * phiz + 1. / 30.) * Jyy * oneplusphiz2;
    double rFz = (mu * LLL * (1. / 120. * phiy2 + 1. / 60. * phiy + 1. / 140.)) * oneplusphiy2 +
                 L * (-1. / 6. * phiy2 + 1. / 6. * phiy + 1. / 30.) * Jzz * oneplusphiy2;

    double rGy1 = Qy1 / 2. * oneplusphiz;
    double rGz1 = Qz1 / 2. * oneplusphiy;
    double rGy2 = Qy2 / 2. * oneplusphiz;
    double rGz2 = Qz2 / 2. * oneplusphiy;
    double rGy = Qy / 2. * oneplusphiz;
    double rGz = Qz / 2. * oneplusphiy;

    double rHy1 = L * (1. / 3. * phiy + 1. / 12.) * Qz1 * oneplusphiy;
    double rHz1 = L * (1. / 3. * phiz + 1. / 12.) * Qy1 * oneplusphiz;
    double rHy2 = L * (1. / 3. * phiy + 1. / 12.) * Qz2 * oneplusphiy;
    double rHz2 = L * (1. / 3. * phiz + 1. / 12.) * Qy2 * oneplusphiz;
    // double rHy = L * (1. / 3. * phiy + 1. / 12.) * Qz * oneplusphiy;
    // double rHz = L * (1. / 3. * phiz + 1. / 12.) * Qy * oneplusphiz;

    double rIy = L * (1. / 6. * phiy - 1. / 12.) * Qz * oneplusphiy;
    double rIz = L * (1. / 6. * phiz - 1. / 12.) * Qy * oneplusphiz;

    double rJyz1 = 6. * Jyz1 / (5. * L) * oneplusphiyz;
    double rJyz2 = 6. * Jyz2 / (5. * L) * oneplusphiyz;
    double rJyz = 6. * Jyz / (5. * L) * oneplusphiyz;

    double rKy = L * (1. / 6. * phiy + 3. / 20.) * Qy * oneplusphiy;
    double rKz = L * (1. / 6. * phiz + 3. / 20.) * Qz * oneplusphiz;

    double rLy1 = L * (1. / 3. * phiy + 7. / 20.) * Qy1 * oneplusphiy;
    double rLz1 = L * (1. / 3. * phiz + 7. / 20.) * Qz1 * oneplusphiz;
    double rLy2 = L * (1. / 3. * phiy + 7. / 20.) * Qy2 * oneplusphiy;
    double rLz2 = L * (1. / 3. * phiz + 7. / 20.) * Qz2 * oneplusphiz;
    // double rLy = L * (1. / 3. * phiy + 7. / 20.) * Qy * oneplusphiy;
    // double rLz = L * (1. / 3. * phiz + 7. / 20.) * Qz * oneplusphiz;

    double rMy1 = (1. / 2. * phiy - 1. / 10.) * Jyz1 * oneplusphiyz;
    double rMz1 = (1. / 2. * phiz - 1. / 10.) * Jyz1 * oneplusphiyz;
    double rMy2 = (1. / 2. * phiy - 1. / 10.) * Jyz2 * oneplusphiyz;
    double rMz2 = (1. / 2. * phiz - 1. / 10.) * Jyz2 * oneplusphiyz;
    double rMy = (1. / 2. * phiy - 1. / 10.) * Jyz * oneplusphiyz;
    double rMz = (1. / 2. * phiz - 1. / 10.) * Jyz * oneplusphiyz;

    double rNyz1 = L * (1. / 12. * phiy + 1. / 12. * phiz + 1. / 3. * phiy * phiz + 2. / 15.) * Jyz1 * oneplusphiyz;
    double rNyz2 = L * (1. / 12. * phiy + 1. / 12. * phiz + 1. / 3. * phiy * phiz + 2. / 15.) * Jyz2 * oneplusphiyz;
    // double rNyz = L * (1. / 12. * phiy + 1. / 12. * phiz + 1. / 3. * phiy * phiz + 2. / 15.) * Jyz * oneplusphiyz;

    double rOyz = L * (1. / 12. * phiy + 1. / 12. * phiz - 1. / 6. * phiy * phiz + 1. / 30.) * Jyz * oneplusphiyz;
    double rPy = LL * (1. / 24. * phiy + 1. / 30.) * Qy * oneplusphiy;
    double rPz = LL * (1. / 24. * phiz + 1. / 30.) * Qz * oneplusphiz;

    double rQy1 = LL * (1. / 24. * phiy + 1. / 20.) * Qy1 * oneplusphiy;
    double rQz1 = LL * (1. / 24. * phiz + 1. / 20.) * Qz1 * oneplusphiz;
    double rQy2 = LL * (1. / 24. * phiy + 1. / 20.) * Qy2 * oneplusphiy;
    double rQz2 = LL * (1. / 24. * phiz + 1. / 20.) * Qz2 * oneplusphiz;
    // double rQy = LL * (1. / 24. * phiy + 1. / 20.) * Qy * oneplusphiy;
    // double rQz = LL * (1. / 24. * phiz + 1. / 20.) * Qz * oneplusphiz;

    M(0, 0) = mu1 * L / 3.;
    M(1, 0) = rGz1;
    M(2, 0) = rGy1;
    M(3, 0) = 0.;
    M(4, 0) = rHz1;
    M(5, 0) = -rHy1;

    M(6, 0) = mu * L / 6.;
    M(7, 0) = -rGz;
    M(8, 0) = -rGy;
    M(9, 0) = 0.;
    M(10, 0) = rIz;
    M(11, 0) = -rIy;

    M(1, 1) = rAz1;
    M(2, 1) = rJyz1;
    M(3, 1) = -rLy1;
    M(4, 1) = rMz1;
    M(5, 1) = rCz1;

    M(6, 1) = rGz;
    M(7, 1) = rBz;
    M(8, 1) = -rJyz;
    M(9, 1) = -rKy;
    M(10, 1) = rMz;
    M(11, 1) = -rDz;

    M(2, 2) = rAy1;
    M(3, 2) = rLz1;
    M(4, 2) = -rCy1;
    M(5, 2) = -rMy1;

    M(6, 2) = rGy;
    M(7, 2) = -rJyz;
    M(8, 2) = rBy;
    M(9, 2) = rKz;
    M(10, 2) = rDy;
    M(11, 2) = -rMy;

    M(3, 3) = L * Jxx1 / 3.;
    M(4, 3) = -rQz1;
    M(5, 3) = -rQy1;

    M(6, 3) = 0.;
    M(7, 3) = -rKy;
    M(8, 3) = rKz;
    M(9, 3) = L * Jxx / 6.;
    M(10, 3) = rPz;
    M(11, 3) = rPy;

    M(4, 4) = rEy1;
    M(5, 4) = -rNyz1;

    M(6, 4) = rIz;
    M(7, 4) = -rMz;
    M(8, 4) = -rDy;
    M(9, 4) = -rPz;
    M(10, 4) = -rFy;
    M(11, 4) = rOyz;

    M(5, 5) = rEz1;

    M(6, 5) = -rIy;
    M(7, 5) = rDz;
    M(8, 5) = rMy;
    M(9, 5) = -rPy;
    M(10, 5) = rOyz;
    M(11, 5) = -rFz;

    M(6, 6) = mu2 * L / 3.;
    M(7, 6) = -rGz2;
    M(8, 6) = -rGy2;
    M(9, 6) = 0.;
    M(10, 6) = rHz2;
    M(11, 6) = -rHy2;
    M(7, 7) = rAz2;
    M(8, 7) = rJyz2;
    M(9, 7) = -rLy2;
    M(10, 7) = -rMz2;
    M(11, 7) = -rCz2;
    M(8, 8) = rAy2;
    M(9, 8) = rLz2;
    M(10, 8) = rCy2;
    M(11, 8) = rMy2;
    M(9, 9) = L * Jxx2 / 3.;
    M(10, 9) = rQz2;
    M(11, 9) = rQy2;
    M(10, 10) = rEy2;
    M(11, 10) = -rNyz2;
    M(11, 11) = rEz2;

    // obtain the symmetric right-top elements by mirror
    for (int i = 0; i < 12; i++) {
        for (int j = i; j < 12; j++) {
            M(i, j) = M(j, i);
        }
    }
}

void ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeInertiaMatrix(ChMatrixDynamic<>& M) {
    M.setZero(12, 12);

    ComputeAverageSectionParameters();

    ChMatrixNM<double, 12, 12> Mtmp;

    if (use_lumped_mass_matrix) {
        ComputeLumpedInertiaMatrix(Mtmp);
    } else {
        // The most generic function for consistent mass matrix
        ComputeConsistentInertiaMatrix(Mtmp);

        // A simple function for consistent mass matrix, do quick transformation by matrix manipulation
        // Has been validated to be wrong! NEVER use it again
        // ComputeSimpleConsistentInertiaMatrix(Mtmp);
    }
    M = Mtmp;  // transform from ChMatrixNM<double, 12, 12> to ChMatrixDynamic<>
}

void ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeInertiaDampingMatrix(ChMatrixNM<double, 12, 12>& Ri,
                                                                                const ChVector<>& mW_A,
                                                                                const ChVector<>& mW_B) {
    Ri.setZero(12, 12);

    if (this->compute_inertia_damping_matrix == false)
        return;

    this->sectionA->compute_inertia_damping_matrix = this->compute_inertia_damping_matrix;
    this->sectionA->compute_inertia_stiffness_matrix = this->compute_inertia_stiffness_matrix;
    this->sectionA->compute_Ri_Ki_by_num_diff = this->compute_Ri_Ki_by_num_diff;

    this->sectionB->compute_inertia_damping_matrix = this->compute_inertia_damping_matrix;
    this->sectionB->compute_inertia_stiffness_matrix = this->compute_inertia_stiffness_matrix;
    this->sectionB->compute_Ri_Ki_by_num_diff = this->compute_Ri_Ki_by_num_diff;

    ChMatrixNM<double, 6, 6> Ri_A;
    ChMatrixNM<double, 6, 6> Ri_B;
    this->sectionA->ComputeInertiaDampingMatrix(Ri_A, mW_A);
    this->sectionB->ComputeInertiaDampingMatrix(Ri_B, mW_B);

    Ri.block<6, 6>(0, 0) = Ri_A;
    Ri.block<6, 6>(6, 6) = Ri_B;
}

void ChBeamSectionTaperedTimoshenkoAdvancedGeneric::ComputeInertiaStiffnessMatrix(
    ChMatrixNM<double, 12, 12>& Ki,
    const ChVector<>& mWvel_A,  ///< current angular velocity of section of node A, in material frame
    const ChVector<>& mWacc_A,  ///< current angular acceleration of section of node A, in material frame
    const ChVector<>& mXacc_A,  ///< current acceleration of section of node A, in material frame)
    const ChVector<>& mWvel_B,  ///< current angular velocity of section of node B, in material frame
    const ChVector<>& mWacc_B,  ///< current angular acceleration of section of node B, in material frame
    const ChVector<>& mXacc_B   ///< current acceleration of section of node B, in material frame
) {
    Ki.setZero(12, 12);

    if (this->compute_inertia_stiffness_matrix == false)
        return;

    this->sectionA->compute_inertia_damping_matrix = this->compute_inertia_damping_matrix;
    this->sectionA->compute_inertia_stiffness_matrix = this->compute_inertia_stiffness_matrix;
    this->sectionA->compute_Ri_Ki_by_num_diff = this->compute_Ri_Ki_by_num_diff;

    this->sectionB->compute_inertia_damping_matrix = this->compute_inertia_damping_matrix;
    this->sectionB->compute_inertia_stiffness_matrix = this->compute_inertia_stiffness_matrix;
    this->sectionB->compute_Ri_Ki_by_num_diff = this->compute_Ri_Ki_by_num_diff;

    ChMatrixNM<double, 6, 6> Ki_A;
    ChMatrixNM<double, 6, 6> Ki_B;
    this->sectionA->ComputeInertiaStiffnessMatrix(Ki_A, mWvel_A, mWacc_A, mXacc_A);
    this->sectionB->ComputeInertiaStiffnessMatrix(Ki_B, mWvel_B, mWacc_B, mXacc_B);

    Ki.block<6, 6>(0, 0) = Ki_A;
    Ki.block<6, 6>(6, 6) = Ki_B;
}

DampingCoefficients ChBeamSectionTaperedTimoshenkoAdvancedGeneric::GetBeamRaleyghDamping() const {
    return this->avg_sec_par->rdamping_coeff;
};

}  // end namespace fea
}  // end namespace chrono
