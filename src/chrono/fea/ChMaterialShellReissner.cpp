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

#include <cmath>

#include "chrono/fea/ChMaterialShellReissner.h"

namespace chrono {
namespace fea {

void ChElasticityReissner::ComputeStiffnessMatrix(ChMatrixRef mC,
                                                  const ChVector3d& eps_u,
                                                  const ChVector3d& eps_v,
                                                  const ChVector3d& kur_u,
                                                  const ChVector3d& kur_v,
                                                  const double z_inf,
                                                  const double z_sup,
                                                  const double angle) {
    assert(mC.rows() == 12);
    assert(mC.cols() == 12);

    mC.setZero();

    ChVectorN<double, 12> strain_0;
    strain_0.segment(0, 3) = eps_u.eigen();
    strain_0.segment(3, 3) = eps_v.eigen();
    strain_0.segment(6, 3) = kur_u.eigen();
    strain_0.segment(9, 3) = kur_v.eigen();

    ChVector3d nu, nv, mu, mv;
    this->ComputeStress(nu, nv, mu, mv, eps_u, eps_v, kur_u, kur_v, z_inf, z_sup, angle);

    ChVectorN<double, 12> stress_0;
    stress_0.segment(0, 3) = nu.eigen();
    stress_0.segment(3, 3) = nv.eigen();
    stress_0.segment(6, 3) = mu.eigen();
    stress_0.segment(9, 3) = mv.eigen();

    double delta = 1e-9;
    for (int i = 0; i < 12; ++i) {
        strain_0(i, 0) += delta;
        ChVector3d deps_u(strain_0.segment(0, 3));
        ChVector3d deps_v(strain_0.segment(3, 3));
        ChVector3d dkur_u(strain_0.segment(6, 3));
        ChVector3d dkur_v(strain_0.segment(9, 3));
        this->ComputeStress(nu, nv, mu, mv, deps_u, deps_v, dkur_u, dkur_v, z_inf, z_sup, angle);
        ChVectorN<double, 12> stress_1;
        stress_1.segment(0, 3) = nu.eigen();
        stress_1.segment(3, 3) = nv.eigen();
        stress_1.segment(6, 3) = mu.eigen();
        stress_1.segment(9, 3) = mv.eigen();
        ChVectorN<double, 12> stress_d = (1. / delta) * (stress_1 - stress_0);
        mC.block(0, i, 12, 1) = stress_d;
        strain_0(i, 0) -= delta;
    }
}

//--------------------------------------------------------------

ChElasticityReissnerIsothropic::ChElasticityReissnerIsothropic(double E, double nu, double alpha, double beta) {
    m_E = E;
    m_nu = nu;
    m_alpha = alpha;
    m_beta = beta;
}

void ChElasticityReissnerIsothropic::ComputeStress(ChVector3d& n_u,
                                                   ChVector3d& n_v,
                                                   ChVector3d& m_u,
                                                   ChVector3d& m_v,
                                                   const ChVector3d& eps_u,
                                                   const ChVector3d& eps_v,
                                                   const ChVector3d& kur_u,
                                                   const ChVector3d& kur_v,
                                                   const double z_inf,
                                                   const double z_sup,
                                                   const double angle) {
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
        double G = m_E / (2. * (1. + m_nu));
        double Q11 = m_E / (1. - m_nu * m_nu);
        double Q22 = Q11;
        double Q12 = m_nu * Q11;
        double Q33 = 2 * G;
        double Q44 = 2 * G;
        double Qss = m_alpha * 2 * G;
        double Qdd = m_beta * 2 * G;
        double h1 = z_sup - z_inf;
        double h2 = 0.5 * (std::pow(z_sup, 2) - std::pow(z_inf, 2));
        double h3 = CH_1_3 * (std::pow(z_sup, 3) - std::pow(z_inf, 3));

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

void ChElasticityReissnerIsothropic::ComputeStiffnessMatrix(ChMatrixRef mC,
                                                            const ChVector3d& eps_u,
                                                            const ChVector3d& eps_v,
                                                            const ChVector3d& kur_u,
                                                            const ChVector3d& kur_v,
                                                            const double z_inf,
                                                            const double z_sup,
                                                            const double angle) {
    assert(mC.rows() == 12);
    assert(mC.cols() == 12);

    mC.setZero();

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
        double G = m_E / (2. * (1. + m_nu));
        double Q11 = m_E / (1. - m_nu * m_nu);
        double Q22 = Q11;
        double Q12 = m_nu * Q11;
        double Q33 = 2 * G;
        double Q44 = 2 * G;
        double Qss = m_alpha * 2 * G;
        double Qdd = m_beta * 2 * G;
        double h1 = z_sup - z_inf;
        double h2 = 0.5 * (std::pow(z_sup, 2) - std::pow(z_inf, 2));
        double h3 = CH_1_3 * (std::pow(z_sup, 3) - std::pow(z_inf, 3));

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
ChElasticityReissnerOrthotropic::ChElasticityReissnerOrthotropic(double m_E_x,
                                                                 double m_E_y,
                                                                 double m_nu_xy,
                                                                 double m_G_xy,
                                                                 double m_G_xz,
                                                                 double m_G_yz,
                                                                 double m_alpha,
                                                                 double m_beta) {
    E_x = m_E_x;
    E_y = m_E_y;
    nu_xy = m_nu_xy;
    G_xy = m_G_xy;
    G_xz = m_G_xz;
    G_yz = m_G_yz;
    alpha = m_alpha;
    beta = m_beta;
}

ChElasticityReissnerOrthotropic::ChElasticityReissnerOrthotropic(double m_E,
                                                                 double m_nu,
                                                                 double m_alpha,
                                                                 double m_beta) {
    double m_G = m_E / (2. * (1. + m_nu));  // default value of G for special subcase of isotropic constructor
    this->E_x = m_E;
    this->E_y = m_E;
    this->nu_xy = m_nu;
    this->G_xy = m_G;
    this->G_xz = m_G;
    this->G_yz = m_G;
    this->alpha = m_alpha;
    this->beta = m_beta;
}

void ChElasticityReissnerOrthotropic::ComputeStress(ChVector3d& n_u,
                                                    ChVector3d& n_v,
                                                    ChVector3d& m_u,
                                                    ChVector3d& m_v,
                                                    const ChVector3d& eps_u,
                                                    const ChVector3d& eps_v,
                                                    const ChVector3d& kur_u,
                                                    const ChVector3d& kur_v,
                                                    const double z_inf,
                                                    const double z_sup,
                                                    const double angle) {
    // Since it is a linear material, just compute S by using the
    // constitutive matrix, as S = C*eps, where S={n_u, n_v, m_u, m_v}
    ChMatrixNM<double, 12, 12> mC;
    this->ComputeStiffnessMatrix(mC, eps_u, eps_v, kur_u, kur_v, z_inf, z_sup, angle);

    ChVectorN<double, 12> eps;
    eps.segment(0, 3) = eps_u.eigen();
    eps.segment(3, 3) = eps_v.eigen();
    eps.segment(6, 3) = kur_u.eigen();
    eps.segment(9, 3) = kur_v.eigen();

    ChVectorN<double, 12> Sigma = mC * eps;
    n_u = Sigma.segment(0, 3);
    n_v = Sigma.segment(3, 3);
    m_u = Sigma.segment(6, 3);
    m_v = Sigma.segment(9, 3);
}

void ChElasticityReissnerOrthotropic::ComputeStiffnessMatrix(ChMatrixRef mC,
                                                             const ChVector3d& eps_u,
                                                             const ChVector3d& eps_v,
                                                             const ChVector3d& kur_u,
                                                             const ChVector3d& kur_v,
                                                             const double z_inf,
                                                             const double z_sup,
                                                             const double angle) {
    assert(mC.rows() == 12);
    assert(mC.cols() == 12);

    mC.setZero();

    // Compute Qm_local for inplane stresses as in sigma_local = Qm_local * eps_local
    double nu_yx = this->GetPoissonRatioYX();  // follows xy as it must be nu_yx*E_x = nu_xy*E_y
    ChMatrixNM<double, 4, 4> Qm_local;
    Qm_local.setZero();
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
    Qs_local(0, 1) = 0;
    Qs_local(1, 0) = 0;

    // Rotate Qm_local into Qm, as Qm = Tm'*Qm_local*Tm
    double Co = std::cos(angle);
    double Si = std::sin(angle);
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

    ChMatrixNM<double, 4, 4> Qm = Tm.transpose() * Qm_local * Tm;

    // Rotate Qs_local into Qs, as Qs = Ts'*Qs_local*Ts
    ChMatrixNM<double, 2, 2> Ts;
    Ts(0, 0) = Co;
    Ts(0, 1) = -Si;
    Ts(1, 0) = Si;
    Ts(1, 1) = Co;

    ChMatrixNM<double, 2, 2> Qs = Ts.transpose() * Qs_local * Ts;

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
    double H = CH_1_3 * (std::pow(z_sup, 3) - std::pow(z_inf, 3));
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
    double hh = (0.5) * (std::pow(z_sup, 2) - std::pow(z_inf, 2));
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

//----------------------------------------------------------------

ChElasticityReissnerGeneric::ChElasticityReissnerGeneric() {
    mE.setIdentity();
}

void ChElasticityReissnerGeneric::ComputeStress(
    ChVector3d& n_u,          ///< forces along \e u direction (per unit length)
    ChVector3d& n_v,          ///< forces along \e v direction (per unit length)
    ChVector3d& m_u,          ///< torques along \e u direction (per unit length)
    ChVector3d& m_v,          ///< torques along \e v direction (per unit length)
    const ChVector3d& eps_u,  ///< strains along \e u direction
    const ChVector3d& eps_v,  ///< strains along \e v direction
    const ChVector3d& kur_u,  ///< curvature along \e u direction
    const ChVector3d& kur_v,  ///< curvature along \e v direction
    const double z_inf,       ///< layer lower z value (along thickness coord)
    const double z_sup,       ///< layer upper z value (along thickness coord)
    const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
) {
    ChVectorN<double, 12> mstrain;
    ChVectorN<double, 12> mstress;
    mstrain.segment(0, 3) = eps_u.eigen();
    mstrain.segment(3, 3) = eps_v.eigen();
    mstrain.segment(6, 3) = kur_u.eigen();
    mstrain.segment(9, 3) = kur_v.eigen();
    mstress = this->mE * mstrain;
    n_u = mstress.segment(0, 3);
    n_v = mstress.segment(3, 3);
    m_u = mstress.segment(6, 3);
    m_v = mstress.segment(9, 3);
}

void ChElasticityReissnerGeneric::ComputeStiffnessMatrix(
    ChMatrixRef mC,           ///< tangent matrix
    const ChVector3d& eps_u,  ///< strains along \e u direction
    const ChVector3d& eps_v,  ///< strains along \e v direction
    const ChVector3d& kur_u,  ///< curvature along \e u direction
    const ChVector3d& kur_v,  ///< curvature along \e v direction
    const double z_inf,       ///< layer lower z value (along thickness coord)
    const double z_sup,       ///< layer upper z value (along thickness coord)
    const double angle        ///< layer angle respect to x (if needed) -not used in this, isotropic
) {
    mC = this->mE;
}

//----------------------------------------------------------------

ChPlasticityReissner::ChPlasticityReissner() : section(nullptr), nr_yeld_tolerance(1e-7), nr_yeld_maxiters(5) {}

void ChPlasticityReissner::ComputeStiffnessMatrixElastoplastic(
    ChMatrixRef K,            ///< 12x12 material elastoplastic stiffness matrix values here
    const ChVector3d& eps_u,  ///< strains along \e u direction
    const ChVector3d& eps_v,  ///< strains along \e v direction
    const ChVector3d& kur_u,  ///< curvature along \e u direction
    const ChVector3d& kur_v,  ///< curvature along \e v direction
    const ChShellReissnerInternalData&
        data,  ///< updated material internal variables, at this point including {p_strain_e, p_strain_k, p_strain_acc}
    const double z_inf,  ///< layer lower z value (along thickness coord)
    const double z_sup,  ///< layer upper z value (along thickness coord)
    const double angle   ///< layer angle respect to x (if needed)
) {
    ChVector3d n_u;
    ChVector3d n_v;
    ChVector3d m_u;
    ChVector3d m_v;

    std::vector<std::unique_ptr<ChShellReissnerInternalData>> a_plastic_data;
    this->CreatePlasticityData(1, a_plastic_data);
    std::vector<std::unique_ptr<ChShellReissnerInternalData>> b_plastic_data;
    this->CreatePlasticityData(1, b_plastic_data);

    bool in_plastic = ComputeStressWithReturnMapping(n_u, n_v, m_u, m_v, *a_plastic_data[0], eps_u, eps_v, kur_u, kur_v,
                                                     data, z_inf, z_sup, angle);

    if (!in_plastic) {
        // if no return mapping is needed at this strain state, just use elastic matrix:

        return this->section->GetElasticity()->ComputeStiffnessMatrix(K, eps_u, eps_v, kur_u, kur_v, z_inf, z_sup,
                                                                      angle);

    } else {
        // if return mapping is needed at this strain state, compute the elastoplastic stiffness by brute force BDF

        ChVectorN<double, 12> strain_0;
        strain_0.segment(0, 3) = eps_u.eigen();
        strain_0.segment(3, 3) = eps_v.eigen();
        strain_0.segment(6, 3) = kur_u.eigen();
        strain_0.segment(9, 3) = kur_v.eigen();

        ChVectorN<double, 12> stress_0;
        stress_0.segment(0, 3) = n_u.eigen();
        stress_0.segment(3, 3) = n_v.eigen();
        stress_0.segment(6, 3) = m_u.eigen();
        stress_0.segment(9, 3) = m_v.eigen();

        double delta = 1e-6;
        double invdelta = 1.0 / delta;
        for (int i = 0; i < 12; ++i) {
            strain_0(i, 0) += delta;
            ChVector3d deps_u(strain_0.segment(0, 3));
            ChVector3d deps_v(strain_0.segment(3, 3));
            ChVector3d dkur_u(strain_0.segment(6, 3));
            ChVector3d dkur_v(strain_0.segment(9, 3));
            this->ComputeStressWithReturnMapping(n_u, n_v, m_u, m_v, *b_plastic_data[0], deps_u, deps_v, dkur_u, dkur_v,
                                                 data, z_inf, z_sup, angle);
            ChVectorN<double, 12> stress_1;
            stress_1.segment(0, 3) = n_u.eigen();
            stress_1.segment(3, 3) = n_v.eigen();
            stress_1.segment(6, 3) = m_u.eigen();
            stress_1.segment(9, 3) = m_v.eigen();
            ChVectorN<double, 12> stress_d = invdelta * (stress_1 - stress_0);
            K.block(0, i, 12, 1) = stress_d;
            strain_0(i, 0) -= delta;
        }
    }
}

void ChPlasticityReissner::CreatePlasticityData(
    int numpoints,
    std::vector<std::unique_ptr<ChShellReissnerInternalData>>& plastic_data) {
    plastic_data.resize(numpoints);
    for (int i = 0; i < numpoints; ++i) {
        plastic_data[i] = std::unique_ptr<ChShellReissnerInternalData>(new ChShellReissnerInternalData());
    }
}

//-----------------------------------------------------------------------

void ChDampingReissner::ComputeDampingMatrix(
    ChMatrixRef R,             // 12x12 material damping matrix values here
    const ChVector3d& deps_u,  // time derivative of strains along \e u direction
    const ChVector3d& deps_v,  // time derivative of strains along \e v direction
    const ChVector3d& dkur_u,  // time derivative of curvature along \e u direction
    const ChVector3d& dkur_v,  // time derivative of curvature along \e v direction
    const double z_inf,        // layer lower z value (along thickness coord)
    const double z_sup,        // layer upper z value (along thickness coord)
    const double angle         // layer angle respect to x (if needed) -not used in this, isotropic+
) {
    assert(R.rows() == 12);
    assert(R.cols() == 12);

    R.setZero();

    ChVectorN<double, 12> dstrain_0;
    dstrain_0.segment(0, 3) = deps_u.eigen();
    dstrain_0.segment(3, 3) = deps_v.eigen();
    dstrain_0.segment(6, 3) = dkur_u.eigen();
    dstrain_0.segment(9, 3) = dkur_v.eigen();

    ChVector3d nu, nv, mu, mv;
    this->ComputeStress(nu, nv, mu, mv, deps_u, deps_v, dkur_u, dkur_v, z_inf, z_sup, angle);

    ChVectorN<double, 12> stress_0;
    stress_0.segment(0, 3) = nu.eigen();
    stress_0.segment(3, 3) = nv.eigen();
    stress_0.segment(6, 3) = mu.eigen();
    stress_0.segment(9, 3) = mv.eigen();

    double delta = 1e-9;
    for (int i = 0; i < 12; ++i) {
        dstrain_0(i, 0) += delta;
        ChVector3d ddeps_u(dstrain_0.segment(0, 3));
        ChVector3d ddeps_v(dstrain_0.segment(3, 3));
        ChVector3d ddkur_u(dstrain_0.segment(6, 3));
        ChVector3d ddkur_v(dstrain_0.segment(9, 3));
        this->ComputeStress(nu, nv, mu, mv, ddeps_u, ddeps_v, ddkur_u, ddkur_v, z_inf, z_sup, angle);
        ChVectorN<double, 12> stress_1;
        stress_1.segment(0, 3) = nu.eigen();
        stress_1.segment(3, 3) = nv.eigen();
        stress_1.segment(6, 3) = mu.eigen();
        stress_1.segment(9, 3) = mv.eigen();
        ChVectorN<double, 12> stress_d = (1. / delta) * (stress_1 - stress_0);
        R.block(0, i, 12, 1) = stress_d;
        dstrain_0(i, 0) -= delta;
    }
}

// -----------------------------------------------------------------------------

ChDampingReissnerRayleigh::ChDampingReissnerRayleigh(std::shared_ptr<ChElasticityReissner> melasticity,
                                                     const double& mbeta) {
    this->beta = mbeta;
    this->section_elasticity = melasticity;
    this->updated = false;
}

void ChDampingReissnerRayleigh::ComputeStress(
    ChVector3d& n_u,           ///< forces along \e u direction (per unit length)
    ChVector3d& n_v,           ///< forces along \e v direction (per unit length)
    ChVector3d& m_u,           ///< torques along \e u direction (per unit length)
    ChVector3d& m_v,           ///< torques along \e v direction (per unit length)
    const ChVector3d& deps_u,  ///< time derivative of strains along \e u direction
    const ChVector3d& deps_v,  ///< time derivative of strains along \e v direction
    const ChVector3d& dkur_u,  ///< time derivative of curvature along \e u direction
    const ChVector3d& dkur_v,  ///< time derivative of curvature along \e v direction
    const double z_inf,        ///< layer lower z value (along thickness coord)
    const double z_sup,        ///< layer upper z value (along thickness coord)
    const double angle         ///< layer angle respect to x (if needed)
) {
    if (!this->updated && this->section_elasticity->section) {
        this->section_elasticity->ComputeStiffnessMatrix(this->E_const, VNULL, VNULL, VNULL, VNULL, z_inf, z_sup,
                                                         angle);
        this->updated = true;
    }
    ChVectorN<double, 12> mdstrain;
    ChVectorN<double, 12> mstress;
    mdstrain.segment(0, 3) = deps_u.eigen();
    mdstrain.segment(3, 3) = deps_v.eigen();
    mdstrain.segment(6, 3) = dkur_u.eigen();
    mdstrain.segment(9, 3) = dkur_v.eigen();
    mstress = this->beta * this->E_const * mdstrain;
    n_u = mstress.segment(0, 3);
    n_v = mstress.segment(3, 3);
    m_u = mstress.segment(6, 3);
    m_v = mstress.segment(9, 3);
}

void ChDampingReissnerRayleigh::ComputeDampingMatrix(
    ChMatrixRef R,             ///< 12x12 material damping matrix values here
    const ChVector3d& deps_u,  ///< time derivative of strains along \e u direction
    const ChVector3d& deps_v,  ///< time derivative of strains along \e v direction
    const ChVector3d& dkur_u,  ///< time derivative of curvature along \e u direction
    const ChVector3d& dkur_v,  ///< time derivative of curvature along \e v direction
    const double z_inf,        ///< layer lower z value (along thickness coord)
    const double z_sup,        ///< layer upper z value (along thickness coord)
    const double angle         ///< layer angle respect to x (if needed) -not used in this, isotropic
) {
    R = this->beta * this->E_const;
}

// -----------------------------------------------------------------------------

ChMaterialShellReissner::ChMaterialShellReissner(std::shared_ptr<ChElasticityReissner> melasticity) {
    this->SetElasticity(melasticity);
}

ChMaterialShellReissner::ChMaterialShellReissner(std::shared_ptr<ChElasticityReissner> melasticity,
                                                 std::shared_ptr<ChPlasticityReissner> mplasticity) {
    this->SetElasticity(melasticity);
    this->SetPlasticity(mplasticity);
}

ChMaterialShellReissner::ChMaterialShellReissner(std::shared_ptr<ChElasticityReissner> melasticity,
                                                 std::shared_ptr<ChPlasticityReissner> mplasticity,
                                                 std::shared_ptr<ChDampingReissner> mdamping) {
    this->SetElasticity(melasticity);

    if (mplasticity)
        this->SetPlasticity(mplasticity);

    if (mdamping)
        this->SetDamping(mdamping);
}

void ChMaterialShellReissner::ComputeStress(ChVector3d& n_u,
                                            ChVector3d& n_v,
                                            ChVector3d& m_u,
                                            ChVector3d& m_v,
                                            const ChVector3d& eps_u,
                                            const ChVector3d& eps_v,
                                            const ChVector3d& kur_u,
                                            const ChVector3d& kur_v,
                                            const double z_inf,
                                            const double z_sup,
                                            const double angle,
                                            ChShellReissnerInternalData* mdata_new,
                                            const ChShellReissnerInternalData* mdata) {
    if (!plasticity || !mdata || !mdata_new)
        this->elasticity->ComputeStress(n_u, n_v, m_u, m_v, eps_u, eps_v, kur_u, kur_v, z_inf, z_sup, angle);
    else {
        this->plasticity->ComputeStressWithReturnMapping(n_u, n_v, m_u, m_v, *mdata_new, eps_u, eps_v, kur_u, kur_v,
                                                         *mdata, z_inf, z_sup, angle);
    }
}

void ChMaterialShellReissner::ComputeStiffnessMatrix(ChMatrixRef K,
                                                     const ChVector3d& eps_u,
                                                     const ChVector3d& eps_v,
                                                     const ChVector3d& kur_u,
                                                     const ChVector3d& kur_v,
                                                     const double z_inf,
                                                     const double z_sup,
                                                     const double angle,
                                                     const ChShellReissnerInternalData* mdata) {
    if (!plasticity || !mdata)
        this->elasticity->ComputeStiffnessMatrix(K, eps_u, eps_v, kur_u, kur_v, z_inf, z_sup, angle);
    else {
        this->plasticity->ComputeStiffnessMatrixElastoplastic(K, eps_u, eps_v, kur_u, kur_v, *mdata, z_inf, z_sup,
                                                              angle);
    }
}

void ChMaterialShellReissner::SetElasticity(std::shared_ptr<ChElasticityReissner> melasticity) {
    elasticity = melasticity;
    elasticity->section = this;
}

void ChMaterialShellReissner::SetPlasticity(std::shared_ptr<ChPlasticityReissner> mplasticity) {
    plasticity = mplasticity;
    mplasticity->section = this;
}

void ChMaterialShellReissner::SetDamping(std::shared_ptr<ChDampingReissner> mdamping) {
    damping = mdamping;
    damping->section = this;
}

}  // end of namespace fea
}  // end of namespace chrono
