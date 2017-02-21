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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono_fea/ChMaterialShellReissner.h"

namespace chrono {
namespace fea {

//--------------------------------------------------------------

void ChMaterialShellReissner::ComputeTangentC(ChMatrix<>& mC,
                                              const ChVector<>& eps_u,
                                              const ChVector<>& eps_v,
                                              const ChVector<>& kur_u,
                                              const ChVector<>& kur_v,
                                              const double z_inf,
                                              const double z_sup,
                                              const double angle) {
    assert(mC.GetRows() == 12);
    assert(mC.GetColumns() == 12);

    mC.Reset(12, 12);

    ChMatrixNM<double, 12, 1> strain_0;
    strain_0.PasteVector(eps_u, 0, 0);
    strain_0.PasteVector(eps_v, 3, 0);
    strain_0.PasteVector(kur_u, 6, 0);
    strain_0.PasteVector(kur_v, 9, 0);

    ChVector<> nu, nv, mu, mv;

    this->ComputeStress(nu, nv, mu, mv, eps_u, eps_v, kur_u, kur_v, z_inf, z_sup, angle);

    ChMatrixNM<double, 12, 1> stress_0;
    stress_0.PasteVector(nu, 0, 0);
    stress_0.PasteVector(nv, 3, 0);
    stress_0.PasteVector(mu, 6, 0);
    stress_0.PasteVector(mv, 9, 0);

    double delta = 1e-9;
    for (int i = 0; i < 12; ++i) {
        strain_0(i, 0) += delta;
        ChVector<> deps_u, deps_v, dkur_u, dkur_v;
        deps_u = strain_0.ClipVector(0, 0);
        deps_v = strain_0.ClipVector(3, 0);
        dkur_u = strain_0.ClipVector(6, 0);
        dkur_v = strain_0.ClipVector(9, 0);
        this->ComputeStress(nu, nv, mu, mv, deps_u, deps_v, dkur_u, dkur_v, z_inf, z_sup, angle);
        ChMatrixNM<double, 12, 1> stress_1;
        stress_1.PasteVector(nu, 0, 0);
        stress_1.PasteVector(nv, 3, 0);
        stress_1.PasteVector(mu, 6, 0);
        stress_1.PasteVector(mv, 9, 0);
        ChMatrixNM<double, 12, 1> stress_d = stress_1 - stress_0;
        stress_d *= (1. / delta);
        mC.PasteMatrix(stress_d, 0, i);
        strain_0(i, 0) -= delta;
    }
}

//--------------------------------------------------------------

ChMaterialShellReissnerIsothropic::ChMaterialShellReissnerIsothropic(double rho,    ///< material density
                                                                     double E,      ///< Young's modulus
                                                                     double nu,     ///< Poisson ratio
                                                                     double alpha,  ///< shear factor
                                                                     double beta    ///< torque factor
                                                                     ) {
    m_rho = rho;
    m_E = E;
    m_nu = nu;
    m_alpha = alpha;
    m_beta = beta;
}

void ChMaterialShellReissnerIsothropic::ComputeStress(
    ChVector<>& n_u,
    ChVector<>& n_v,
    ChVector<>& m_u,
    ChVector<>& m_v,
    const ChVector<>& eps_u,
    const ChVector<>& eps_v,
    const ChVector<>& kur_u,
    const ChVector<>& kur_v,
    const double z_inf,  ///< layer lower z value (along thickness coord)
    const double z_sup,  ///< layer upper z value (along thickness coord)
    const double angle   ///< layer angle respect to x (if needed)
    ) {
    if (z_inf == -z_sup) {
        // simplified computation for centered layer
        double h = z_sup - z_inf;
        double G = m_E / (2. * (1. + m_nu));
        double C = m_E * h / (1. - m_nu * m_nu);
        double D = C * h * h / 12.;
        double F = G * h * h * h / 12.;

        n_u.x() = eps_u.x() * C + eps_v.y() * m_nu * C;
        n_u.y() = eps_u.y() * 2 * G * h;
        n_u.z() = eps_u.z() * m_alpha * G * h;
        n_v.x() = eps_v.x() * 2 * G * h;
        n_v.y() = eps_v.y() * C + eps_u.x() * m_nu * C;
        n_v.z() = eps_v.z() * m_alpha * G * h;

        m_u.x() = kur_u.x() * 2 * F;
        m_u.y() = kur_u.y() * D + kur_v.x() * (-m_nu * D);
        m_u.z() = kur_u.z() * m_beta * F;
        m_v.x() = kur_v.x() * D + kur_u.y() * (-m_nu * D);
        m_v.y() = kur_v.y() * 2 * F;
        m_v.z() = kur_v.z() * m_beta * F;
    } else {
        // throw ChException("ComputeTangentC not yet implemented for non-centered layers");
        double G = m_E / (2. * (1. + m_nu));
        double Q11 = m_E / (1. - m_nu * m_nu);
        double Q22 = Q11;
        double Q12 = m_nu * Q11;
        double Q33 = 2 * G;
        double Q44 = 2 * G;
        double Qss = m_alpha * 2 * G;
        double Qdd = m_beta * 2 * G;
        double h1 = z_sup - z_inf;
        double h2 = 0.5 * (pow(z_sup, 2) - pow(z_inf, 2));
        double h3 = (1. / 3.) * (pow(z_sup, 3) - pow(z_inf, 3));

        n_u.x() = h1 * (eps_u.x() * Q11 + eps_v.y() * Q12) + h2 * (kur_u.y() * Q11 + kur_v.x() * Q12);
        n_u.y() = h1 * (eps_u.y() * Q33) + h2 * (kur_u.x() * Q33);
        n_u.z() = h1 * (eps_u.z() * Qss);
        n_v.x() = h1 * (eps_v.x() * Q44) + h2 * (kur_v.y() * Q44);
        n_v.y() = h1 * (eps_u.x() * Q12 + eps_v.y() * Q22) + h2 * (kur_u.y() * Q12 + kur_v.x() * Q22);
        n_v.z() = h1 * (eps_v.z() * Qss);

        m_u.x() = h2 * (eps_u.y() * Q33) + h3 * (kur_u.x() * Q33);
        m_u.y() = h2 * (eps_u.x() * Q11 + eps_v.y() * Q12) + h3 * (kur_u.y() * Q11 + kur_v.x() * Q12);
        m_u.z() = h3 * (eps_u.z() * Qdd);
        m_v.x() = h2 * (eps_u.x() * Q12 + eps_v.y() * Q22) + h3 * (kur_u.y() * Q12 + kur_v.x() * Q22);
        m_v.y() = h2 * (eps_v.x() * Q44) + h3 * (kur_v.y() * Q44);
        m_v.z() = h3 * (eps_v.z() * Qdd);
    }
}

void ChMaterialShellReissnerIsothropic::ComputeTangentC(
    ChMatrix<>& mC,
    const ChVector<>& eps_u,
    const ChVector<>& eps_v,
    const ChVector<>& kur_u,
    const ChVector<>& kur_v,
    const double z_inf,  ///< layer lower z value (along thickness coord)
    const double z_sup,  ///< layer upper z value (along thickness coord)
    const double angle   ///< layer angle respect to x (if needed)
    ) {
    assert(mC.GetRows() == 12);
    assert(mC.GetColumns() == 12);

    mC.Reset(12, 12);

    if (z_inf == -z_sup) {
        // simplified computation for centered layer
        double h = z_sup - z_inf;
        double G = m_E / (2. * (1. + m_nu));
        double C = m_E * h / (1. - m_nu * m_nu);
        double D = C * h * h / 12.;
        double F = G * h * h * h / 12.;
        mC(0, 0) = C;
        mC(0, 4) = m_nu * C;
        mC(4, 0) = m_nu * C;
        mC(1, 1) = 2. * G * h;
        mC(2, 2) = m_alpha * G * h;
        mC(3, 3) = 2. * G * h;
        mC(4, 4) = C;
        mC(5, 5) = m_alpha * G * h;
        mC(6, 6) = 2. * F;
        mC(7, 7) = D;
        mC(7, 9) = -m_nu * D;
        mC(9, 7) = -m_nu * D;
        mC(8, 8) = m_beta * F;
        mC(9, 9) = D;
        mC(10, 10) = 2. * F;
        mC(11, 11) = m_beta * F;
    } else {
        // throw ChException("ComputeTangentC not yet implemented for non-centered layers");
        double G = m_E / (2. * (1. + m_nu));
        double Q11 = m_E / (1. - m_nu * m_nu);
        double Q22 = Q11;
        double Q12 = m_nu * Q11;
        double Q33 = 2 * G;
        double Q44 = 2 * G;
        double Qss = m_alpha * 2 * G;
        double Qdd = m_beta * 2 * G;
        double h1 = z_sup - z_inf;
        double h2 = 0.5 * (pow(z_sup, 2) - pow(z_inf, 2));
        double h3 = (1. / 3.) * (pow(z_sup, 3) - pow(z_inf, 3));

        mC(0, 0) = h1 * Q11;
        mC(0, 4) = h1 * Q12;
        mC(0, 7) = h2 * Q11;
        mC(0, 9) = h2 * Q12;
        mC(1, 1) = h1 * Q33;
        mC(1, 6) = h2 * Q33;
        mC(2, 2) = h1 * Qss;
        mC(3, 3) = h1 * Q44;
        mC(3, 10) = h2 * Q44;
        mC(4, 0) = h1 * Q12;
        mC(4, 4) = h1 * Q22;
        mC(4, 7) = h2 * Q12;
        mC(4, 9) = h2 * Q22;
        mC(5, 5) = h1 * Qss;
        mC(6, 1) = h2 * Q33;
        mC(6, 6) = h3 * Q33;
        mC(7, 0) = h2 * Q11;
        mC(7, 4) = h2 * Q12;
        mC(7, 7) = h3 * Q11;
        mC(7, 9) = h3 * Q12;
        mC(8, 8) = h3 * Qdd;
        mC(9, 0) = h2 * Q12;
        mC(9, 4) = h2 * Q22;
        mC(9, 7) = h3 * Q12;
        mC(9, 9) = h3 * Q22;
        mC(10, 3) = h2 * Q44;
        mC(10, 10) = h3 * Q44;
        mC(11, 11) = h3 * Qdd;
    }
}

//--------------------------------------------------------------

/// Construct an orthotropic material
ChMaterialShellReissnerOrthotropic::ChMaterialShellReissnerOrthotropic(
    double a_rho,    ///< material density
    double m_E_x,    ///< Young's modulus on x
    double m_E_y,    ///< Young's modulus on y
    double m_nu_xy,  ///< Poisson ratio (for yx it holds: nu_yx*E_x = nu_xy*E_y)
    double m_G_xy,   ///< Shear modulus, in plane
    double m_G_xz,   ///< Shear modulus, transverse
    double m_G_yz,   ///< Shear modulus, transverse
    double m_alpha,  ///< shear factor
    double m_beta    ///< torque factor
    ) {
    m_rho = a_rho;
    E_x = m_E_x;
    E_y = m_E_y;
    nu_xy = m_nu_xy;
    G_xy = m_G_xy;
    G_xz = m_G_xz;
    G_yz = m_G_yz;
    alpha = m_alpha;
    beta = m_beta;
}

ChMaterialShellReissnerOrthotropic::ChMaterialShellReissnerOrthotropic(double a_rho,    ///< material density
                                                                       double m_E,      ///< Young's modulus
                                                                       double m_nu,     ///< Poisson ratio
                                                                       double m_alpha,  ///< shear factor
                                                                       double m_beta    ///< torque factor
                                                                       ) {
    this->m_rho = a_rho;
    double m_G = m_E / (2. * (1. + m_nu));  // defult value of G for special subcase of isotropic constructor
    this->E_x = m_E;
    this->E_y = m_E;
    this->nu_xy = m_nu;
    this->G_xy = m_G;
    this->G_xz = m_G;
    this->G_yz = m_G;
    this->alpha = m_alpha;
    this->beta = m_beta;
}

void ChMaterialShellReissnerOrthotropic::ComputeStress(
    ChVector<>& n_u,
    ChVector<>& n_v,
    ChVector<>& m_u,
    ChVector<>& m_v,
    const ChVector<>& eps_u,
    const ChVector<>& eps_v,
    const ChVector<>& kur_u,
    const ChVector<>& kur_v,
    const double z_inf,  ///< layer lower z value (along thickness coord)
    const double z_sup,  ///< layer upper z value (along thickness coord)
    const double angle   ///< layer angle respect to x (if needed)
    ) {
    // Since it is a linear material, just compute S by using the
    // constitutive matrix, as S = C*eps, where S={n_u, n_v, m_u, m_v}
    ChMatrixNM<double, 12, 12> mC;
    this->ComputeTangentC(mC, eps_u, eps_v, kur_u, kur_v, z_inf, z_sup, angle);
    ChMatrixNM<double, 12, 1> eps;
    eps.PasteVector(eps_u, 0, 0);
    eps.PasteVector(eps_v, 3, 0);
    eps.PasteVector(kur_u, 6, 0);
    eps.PasteVector(kur_v, 9, 0);
    ChMatrixNM<double, 12, 1> Sigma;
    Sigma.MatrMultiply(mC, eps);
    n_u = Sigma.ClipVector(0, 0);
    n_v = Sigma.ClipVector(3, 0);
    m_u = Sigma.ClipVector(6, 0);
    m_v = Sigma.ClipVector(9, 0);
}

void ChMaterialShellReissnerOrthotropic::ComputeTangentC(
    ChMatrix<>& mC,
    const ChVector<>& eps_u,
    const ChVector<>& eps_v,
    const ChVector<>& kur_u,
    const ChVector<>& kur_v,
    const double z_inf,  ///< layer lower z value (along thickness coord)
    const double z_sup,  ///< layer upper z value (along thickness coord)
    const double angle   ///< layer angle respect to x (if needed)
    ) {
    assert(mC.GetRows() == 12);
    assert(mC.GetColumns() == 12);

    mC.Reset(12, 12);

    // Compute Qm_local for inplane stresses as in sigma_local = Qm_local * eps_local
    double nu_yx = this->Get_nu_yx();  // follows xy as it must be nu_yx*E_x = nu_xy*E_y
    ChMatrixNM<double, 4, 4> Qm_local;
    Qm_local(0, 0) = E_x / (1. - nu_xy * nu_yx);
    Qm_local(0, 1) = (nu_xy * E_y) / (1. - nu_xy * nu_yx);
    Qm_local(1, 0) = (nu_yx * E_x) / (1. - nu_xy * nu_yx);
    Qm_local(1, 1) = E_y / (1. - nu_xy * nu_yx);
    Qm_local(2, 2) = 2. * G_xy;
    Qm_local(3, 3) = 2. * G_xy;
    // Compute Qs_local for transverse shear stresses as in sigma_local = Qs_local * eps_local
    ChMatrixNM<double, 2, 2> Qs_local;
    Qs_local(0, 0) = 2. * G_xz;
    Qs_local(1, 1) = 2. * G_yz;

    // Rotate Qm_local into Qm, as Qm = Tm'*Qm_local*Tm
    double Co = cos(angle);
    double Si = sin(angle);
    double CC = Co * Co;
    double SS = Si * Si;
    double SC = Si * Co;
    ChMatrixNM<double, 4, 4> Tm;
    Tm(0, 0) = CC;
    Tm(0, 1) = SS;
    Tm(0, 2) = SC;
    Tm(0, 3) = SC;
    Tm(1, 0) = SS;
    Tm(1, 1) = CC;
    Tm(1, 2) = -SC;
    Tm(1, 3) = -SC;
    Tm(2, 0) = -SC;
    Tm(2, 1) = SC;
    Tm(2, 2) = CC;
    Tm(2, 3) = -SS;
    Tm(3, 0) = -SC;
    Tm(3, 1) = SC;
    Tm(3, 2) = -SS;
    Tm(3, 3) = CC;

    ChMatrixNM<double, 4, 4> tmp44;
    ChMatrixNM<double, 4, 4> Qm;
    tmp44.MatrMultiply(Qm_local, Tm);
    Qm.MatrTMultiply(Tm, tmp44);

    // Rotate Qs_local into Qs, as Qs = Ts'*Qs_local*Ts
    ChMatrixNM<double, 2, 2> Ts;
    Ts(0, 0) = Co;
    Ts(0, 1) = -Si;
    Ts(1, 0) = Si;
    Ts(1, 1) = Co;

    ChMatrixNM<double, 2, 2> tmp22;
    ChMatrixNM<double, 2, 2> Qs;
    tmp22.MatrMultiply(Qs_local, Ts);
    Qs.MatrTMultiply(Ts, tmp22);

    // Fill the 12x12 constitutive matrix
    // upper left part
    double h = z_sup - z_inf;
    mC(0, 0) = h * Qm(0, 0);
    mC(0, 1) = h * Qm(0, 2);
    mC(0, 3) = h * Qm(0, 3);
    mC(0, 4) = h * Qm(0, 1);
    mC(1, 0) = h * Qm(2, 0);
    mC(1, 1) = h * Qm(2, 2);
    mC(1, 3) = h * Qm(2, 3);
    mC(1, 4) = h * Qm(2, 1);
    mC(3, 0) = h * Qm(3, 0);
    mC(3, 1) = h * Qm(3, 2);
    mC(3, 3) = h * Qm(3, 3);
    mC(3, 4) = h * Qm(3, 1);
    mC(4, 0) = h * Qm(1, 0);
    mC(4, 1) = h * Qm(1, 2);
    mC(4, 3) = h * Qm(1, 3);
    mC(4, 4) = h * Qm(1, 1);
    mC(2, 2) = h * alpha * Qs(0, 0);
    mC(2, 5) = h * alpha * Qs(0, 1);
    mC(5, 2) = h * alpha * Qs(1, 0);
    mC(5, 5) = h * alpha * Qs(1, 1);
    // lower right part
    double H = (1. / 3.) * (pow(z_sup, 3) - pow(z_inf, 3));
    mC(6, 6) = H * Qm(2, 2);
    mC(6, 7) = H * Qm(2, 0);
    mC(6, 9) = H * Qm(2, 1);
    mC(6, 10) = H * Qm(2, 3);
    mC(7, 6) = H * Qm(0, 2);
    mC(7, 7) = H * Qm(0, 0);
    mC(7, 9) = H * Qm(0, 1);
    mC(7, 10) = H * Qm(0, 3);
    mC(9, 6) = H * Qm(1, 2);
    mC(9, 7) = H * Qm(1, 0);
    mC(9, 9) = H * Qm(1, 1);
    mC(9, 10) = H * Qm(1, 3);
    mC(10, 6) = H * Qm(3, 2);
    mC(10, 7) = H * Qm(3, 0);
    mC(10, 9) = H * Qm(3, 1);
    mC(10, 10) = H * Qm(3, 3);
    mC(8, 8) = H * beta * Qs(0, 0);
    mC(8, 11) = H * beta * Qs(0, 1);
    mC(11, 8) = H * beta * Qs(1, 0);
    mC(11, 11) = H * beta * Qs(1, 1);
    // upper right part and lower right part
    double hh = (0.5) * (pow(z_sup, 2) - pow(z_inf, 2));
    mC(0, 6) = hh * Qm(0, 2);
    mC(0, 7) = hh * Qm(0, 0);
    mC(0, 9) = hh * Qm(0, 1);
    mC(0, 10) = hh * Qm(0, 3);
    mC(1, 6) = hh * Qm(2, 2);
    mC(1, 7) = hh * Qm(2, 0);
    mC(1, 9) = hh * Qm(2, 1);
    mC(1, 10) = hh * Qm(2, 3);
    mC(3, 6) = hh * Qm(3, 2);
    mC(3, 7) = hh * Qm(3, 0);
    mC(3, 9) = hh * Qm(3, 1);
    mC(3, 10) = hh * Qm(3, 3);
    mC(4, 6) = hh * Qm(1, 2);
    mC(4, 7) = hh * Qm(1, 0);
    mC(4, 9) = hh * Qm(1, 1);
    mC(4, 10) = hh * Qm(1, 3);
    mC(6, 0) = hh * Qm(2, 0);
    mC(6, 1) = hh * Qm(2, 2);
    mC(6, 3) = hh * Qm(2, 3);
    mC(6, 4) = hh * Qm(2, 1);
    mC(7, 0) = hh * Qm(0, 0);
    mC(7, 1) = hh * Qm(0, 2);
    mC(7, 3) = hh * Qm(0, 3);
    mC(7, 4) = hh * Qm(0, 1);
    mC(9, 0) = hh * Qm(1, 0);
    mC(9, 1) = hh * Qm(1, 2);
    mC(9, 3) = hh * Qm(1, 3);
    mC(9, 4) = hh * Qm(1, 1);
    mC(10, 0) = hh * Qm(3, 0);
    mC(10, 1) = hh * Qm(3, 2);
    mC(10, 3) = hh * Qm(3, 3);
    mC(10, 4) = hh * Qm(3, 1);
}

}  // end of namespace fea
}  // end of namespace chrono
