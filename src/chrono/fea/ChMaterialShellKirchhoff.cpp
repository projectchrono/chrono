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

#include <cmath>

#include "chrono/fea/ChMaterialShellKirchhoff.h"

namespace chrono {
namespace fea {

void ChElasticityKirchhoff::ComputeStiffnessMatrix(ChMatrixRef mC,
                                                   const ChVector3d& eps,
                                                   const ChVector3d& kur,
                                                   const double z_inf,
                                                   const double z_sup,
                                                   const double angle) {
    assert(mC.rows() == 6);
    assert(mC.cols() == 6);

    mC.setZero();

    ChVectorN<double, 6> strain_0;
    strain_0.segment(0, 3) = eps.eigen();
    strain_0.segment(3, 3) = kur.eigen();

    ChVector3d n, m;
    this->ComputeStress(n, m, eps, kur, z_inf, z_sup, angle);

    ChVectorN<double, 6> stress_0;
    stress_0.segment(0, 3) = n.eigen();
    stress_0.segment(3, 3) = m.eigen();

    double delta = 1e-9;
    for (int i = 0; i < 6; ++i) {
        strain_0(i, 0) += delta;
        ChVector3d deps(strain_0.segment(0, 3));
        ChVector3d dkur(strain_0.segment(3, 3));
        this->ComputeStress(n, m, deps, dkur, z_inf, z_sup, angle);
        ChVectorN<double, 6> stress_1;
        stress_1.segment(0, 3) = n.eigen();
        stress_1.segment(3, 3) = m.eigen();
        ChVectorN<double, 6> stress_d = (1. / delta) * (stress_1 - stress_0);
        mC.block(0, i, 6, 1) = stress_d;
        strain_0(i, 0) -= delta;
    }
}

//--------------------------------------------------------------

ChElasticityKirchhoffIsothropic::ChElasticityKirchhoffIsothropic(double E, double nu) {
    m_E = E;
    m_nu = nu;
}

void ChElasticityKirchhoffIsothropic::ComputeStress(ChVector3d& n,
                                                    ChVector3d& m,
                                                    const ChVector3d& eps,
                                                    const ChVector3d& kur,
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

        n.x() = eps.x() * C + eps.y() * m_nu * C;
        n.y() = eps.y() * C + eps.x() * m_nu * C;
        n.z() = eps.z() * G * h;

        m.x() = kur.x() * D + kur.y() * (m_nu * D);
        m.y() = kur.y() * D + kur.x() * (m_nu * D);
        m.z() = kur.z() * F;
    } else {
        double G = m_E / (2. * (1. + m_nu));
        double h1 = z_sup - z_inf;
        double h2 = 0.5 * (std::pow(z_sup, 2) - std::pow(z_inf, 2));
        double h3 = CH_1_3 * (std::pow(z_sup, 3) - std::pow(z_inf, 3));
        ChMatrix33<> Q;
        Q(0, 0) = m_E / (1. - m_nu * m_nu);
        Q(0, 1) = m_nu * Q(0, 0);
        Q(0, 2) = 0;
        Q(1, 0) = Q(0, 1);
        Q(1, 1) = Q(0, 0);
        Q(1, 2) = 0;
        Q(2, 0) = 0;
        Q(2, 1) = 0;
        Q(2, 2) = G;
        n = h1 * (Q * eps) + h2 * (Q * kur);
        m = h2 * (Q * eps) + h3 * (Q * kur);
    }
}

void ChElasticityKirchhoffIsothropic::ComputeStiffnessMatrix(ChMatrixRef mC,
                                                             const ChVector3d& eps,
                                                             const ChVector3d& kur,
                                                             const double z_inf,
                                                             const double z_sup,
                                                             const double angle) {
    assert(mC.rows() == 6);
    assert(mC.cols() == 6);

    mC.setZero();

    if (z_inf == -z_sup) {
        // simplified computation for centered layer
        double h = z_sup - z_inf;
        double G = m_E / (2. * (1. + m_nu));
        double C = m_E * h / (1. - m_nu * m_nu);
        double D = C * h * h / 12.;
        double F = G * h * h * h / 12.;
        mC(0, 0) = C;
        mC(1, 1) = C;
        mC(0, 1) = m_nu * C;
        mC(1, 0) = m_nu * C;
        mC(2, 2) = G * h;
        mC(3, 3) = D;
        mC(4, 4) = D;
        mC(3, 4) = m_nu * D;
        mC(4, 3) = m_nu * D;
        mC(5, 5) = F;
    } else {
        double G = m_E / (2. * (1. + m_nu));
        double h1 = z_sup - z_inf;
        double h2 = 0.5 * (std::pow(z_sup, 2) - std::pow(z_inf, 2));
        double h3 = CH_1_3 * (std::pow(z_sup, 3) - std::pow(z_inf, 3));
        ChMatrix33<> Q;
        Q(0, 0) = m_E / (1. - m_nu * m_nu);
        Q(0, 1) = m_nu * Q(0, 0);
        Q(0, 2) = 0;
        Q(1, 0) = Q(0, 1);
        Q(1, 1) = Q(0, 0);
        Q(1, 2) = 0;
        Q(2, 0) = 0;
        Q(2, 1) = 0;
        Q(2, 2) = G;
        mC.block<3, 3>(0, 0) = Q * h1;
        mC.block<3, 3>(0, 3) = Q * h2;
        mC.block<3, 3>(3, 0) = Q * h2;
        mC.block<3, 3>(3, 3) = Q * h3;
    }
}

//--------------------------------------------------------------

/// Construct an orthotropic material
ChElasticityKirchhoffOrthotropic::ChElasticityKirchhoffOrthotropic(double m_E_x,
                                                                   double m_E_y,
                                                                   double m_nu_xy,
                                                                   double m_G_xy) {
    E_x = m_E_x;
    E_y = m_E_y;
    nu_xy = m_nu_xy;
    G_xy = m_G_xy;
}

ChElasticityKirchhoffOrthotropic::ChElasticityKirchhoffOrthotropic(double m_E, double m_nu) {
    double m_G = m_E / (2. * (1. + m_nu));  // default value of G for special subcase of isotropic constructor
    this->E_x = m_E;
    this->E_y = m_E;
    this->nu_xy = m_nu;
    this->G_xy = m_G;
}

void ChElasticityKirchhoffOrthotropic::ComputeStress(ChVector3d& n,
                                                     ChVector3d& m,
                                                     const ChVector3d& eps,
                                                     const ChVector3d& kur,
                                                     const double z_inf,
                                                     const double z_sup,
                                                     const double angle) {
    // Since it is a linear material, just compute S by using the
    // constitutive matrix, as S = C*eps, where S={n, m}
    ChMatrix66d mC;
    this->ComputeStiffnessMatrix(mC, eps, kur, z_inf, z_sup, angle);

    ChVectorN<double, 6> ek;
    ek.segment(0, 3) = eps.eigen();
    ek.segment(3, 3) = kur.eigen();

    ChVectorN<double, 6> Sigma = mC * ek;
    n = Sigma.segment(0, 3);
    m = Sigma.segment(3, 3);
}

void ChElasticityKirchhoffOrthotropic::ComputeStiffnessMatrix(ChMatrixRef mC,
                                                              const ChVector3d& eps,
                                                              const ChVector3d& kur,
                                                              const double z_inf,
                                                              const double z_sup,
                                                              const double angle) {
    assert(mC.rows() == 6);
    assert(mC.cols() == 6);

    mC.setZero();

    // Compute Qm_local for inplane stresses in material coordinate, as in sigma_local = Qm_local * eps_local
    double nu_yx = this->GetPoissonRatioYX();  // follows xy as it must be nu_yx*E_x = nu_xy*E_y
    ChMatrix33<> Qm_local;
    Qm_local.setZero();
    Qm_local(0, 0) = E_x / (1. - nu_xy * nu_yx);
    Qm_local(0, 1) = (nu_xy * E_y) / (1. - nu_xy * nu_yx);
    Qm_local(1, 0) = (nu_yx * E_x) / (1. - nu_xy * nu_yx);
    Qm_local(1, 1) = E_y / (1. - nu_xy * nu_yx);
    Qm_local(2, 2) = G_xy;

    // Rotate Qm_local into Q, as Q = Ts'*Qm_local*Te
    double Co = std::cos(angle);
    double Si = std::sin(angle);
    double CC = Co * Co;
    double SS = Si * Si;
    double SC = Si * Co;
    ChMatrix33<> Tm;  //  T^{-1} , transforms sigma from rotated material direction to local direction
    Tm << CC, SS, -2 * SC, SS, CC, 2 * SC, SC, -SC, CC - SS;

    // Q = T^{-1}*Qm_local*T^{-1 T}
    ChMatrix33<> Q = Tm * Qm_local * Tm.transpose();

    double h1 = z_sup - z_inf;
    double h2 = 0.5 * (std::pow(z_sup, 2) - std::pow(z_inf, 2));
    double h3 = CH_1_3 * (std::pow(z_sup, 3) - std::pow(z_inf, 3));

    mC.block<3, 3>(0, 0) = Q * h1;
    mC.block<3, 3>(0, 3) = Q * h2;
    mC.block<3, 3>(3, 0) = Q * h2;
    mC.block<3, 3>(3, 3) = Q * h3;
}

//--------------------------------------------------------------

ChElasticityKirchhoffGeneric::ChElasticityKirchhoffGeneric() {
    mE.setIdentity();
}

void ChElasticityKirchhoffGeneric::ComputeStress(
    ChVector3d& n,          ///< forces  n_11, n_22, n_12 (per unit length)
    ChVector3d& m,          ///< torques m_11, m_22, m_12 (per unit length)
    const ChVector3d& eps,  ///< strains   e_11, e_22, e_12
    const ChVector3d& kur,  ///< curvature k_11, k_22, k_12
    const double z_inf,     ///< layer lower z value (along thickness coord)
    const double z_sup,     ///< layer upper z value (along thickness coord)
    const double angle      ///< layer angle respect to x (if needed) -not used in this, isotropic
) {
    ChVectorN<double, 6> mstrain;
    ChVectorN<double, 6> mstress;
    mstrain.segment(0, 3) = eps.eigen();
    mstrain.segment(3, 3) = kur.eigen();
    mstress = this->mE * mstrain;
    n = mstress.segment(0, 3);
    m = mstress.segment(3, 3);
}

void ChElasticityKirchhoffGeneric::ComputeStiffnessMatrix(
    ChMatrixRef mC,         ///< tangent matrix
    const ChVector3d& eps,  ///< strains   e_11, e_22, e_12
    const ChVector3d& kur,  ///< curvature k_11, k_22, k_12
    const double z_inf,     ///< layer lower z value (along thickness coord)
    const double z_sup,     ///< layer upper z value (along thickness coord)
    const double angle      ///< layer angle respect to x (if needed)
) {
    mC = this->mE;
}

//----------------------------------------------------------------

ChPlasticityKirchhoff::ChPlasticityKirchhoff() : section(nullptr), nr_yeld_tolerance(1e-7), nr_yeld_maxiters(5) {}

void ChPlasticityKirchhoff::ComputeStiffnessMatrixElastoplastic(
    ChMatrixRef K,          ///< 6x6 material elastoplastic stiffness matrix values here
    const ChVector3d& eps,  ///< strains
    const ChVector3d& kur,  ///< curvature
    const ChShellKirchhoffInternalData&
        data,  ///< updated material internal variables, at this point including {p_strain_e, p_strain_k, p_strain_acc}
    const double z_inf,  ///< layer lower z value (along thickness coord)
    const double z_sup,  ///< layer upper z value (along thickness coord)
    const double angle   ///< layer angle respect to x (if needed)
) {
    ChVector3d n;
    ChVector3d m;

    std::vector<std::unique_ptr<ChShellKirchhoffInternalData>> a_plastic_data;
    this->CreatePlasticityData(1, a_plastic_data);
    std::vector<std::unique_ptr<ChShellKirchhoffInternalData>> b_plastic_data;
    this->CreatePlasticityData(1, b_plastic_data);

    bool in_plastic = ComputeStressWithReturnMapping(n, m, *a_plastic_data[0], eps, kur, data, z_inf, z_sup, angle);

    if (!in_plastic) {
        // if no return mapping is needed at this strain state, just use elastic matrix:

        return this->section->GetElasticity()->ComputeStiffnessMatrix(K, eps, kur, z_inf, z_sup, angle);

    } else {
        // if return mapping is needed at this strain state, compute the elastoplastic stiffness by brute force BDF

        ChVectorN<double, 6> strain_0;
        strain_0.segment(0, 3) = eps.eigen();
        strain_0.segment(3, 3) = kur.eigen();

        ChVectorN<double, 6> stress_0;
        stress_0.segment(0, 3) = n.eigen();
        stress_0.segment(3, 3) = m.eigen();

        double delta = 1e-6;
        double invdelta = 1.0 / delta;
        for (int i = 0; i < 6; ++i) {
            strain_0(i, 0) += delta;
            ChVector3d deps(strain_0.segment(0, 3));
            ChVector3d dkur(strain_0.segment(3, 3));
            this->ComputeStressWithReturnMapping(n, m, *b_plastic_data[0], deps, dkur, data, z_inf, z_sup, angle);
            ChVectorN<double, 6> stress_1;
            stress_1.segment(0, 3) = n.eigen();
            stress_1.segment(3, 3) = m.eigen();
            ChVectorN<double, 6> stress_d = invdelta * (stress_1 - stress_0);
            K.block(0, i, 6, 1) = stress_d;
            strain_0(i, 0) -= delta;
        }
    }
}

void ChPlasticityKirchhoff::CreatePlasticityData(
    int numpoints,
    std::vector<std::unique_ptr<ChShellKirchhoffInternalData>>& plastic_data) {
    plastic_data.resize(numpoints);
    for (int i = 0; i < numpoints; ++i) {
        plastic_data[i] = std::unique_ptr<ChShellKirchhoffInternalData>(new ChShellKirchhoffInternalData());
    }
}

//-----------------------------------------------------------------------

void ChDampingKirchhoff::ComputeDampingMatrix(
    ChMatrixRef R,           // 6x6 material damping matrix values here
    const ChVector3d& deps,  // time derivative of strains
    const ChVector3d& dkur,  // time derivative of curvatures
    const double z_inf,      // layer lower z value (along thickness coord)
    const double z_sup,      // layer upper z value (along thickness coord)
    const double angle       // layer angle respect to x (if needed) -not used in this, isotropic+
) {
    assert(R.rows() == 6);
    assert(R.cols() == 6);

    R.setZero();

    ChVectorN<double, 6> dstrain_0;
    dstrain_0.segment(0, 3) = deps.eigen();
    dstrain_0.segment(3, 3) = dkur.eigen();

    ChVector3d n, m;
    this->ComputeStress(n, m, deps, dkur, z_inf, z_sup, angle);

    ChVectorN<double, 6> stress_0;
    stress_0.segment(0, 3) = n.eigen();
    stress_0.segment(3, 3) = m.eigen();

    double delta = 1e-9;
    for (int i = 0; i < 6; ++i) {
        dstrain_0(i, 0) += delta;
        ChVector3d ddeps(dstrain_0.segment(0, 3));
        ChVector3d ddkur(dstrain_0.segment(3, 3));
        this->ComputeStress(n, m, ddeps, ddkur, z_inf, z_sup, angle);
        ChVectorN<double, 6> stress_1;
        stress_1.segment(0, 3) = n.eigen();
        stress_1.segment(3, 3) = m.eigen();
        ChVectorN<double, 6> stress_d = (1. / delta) * (stress_1 - stress_0);
        R.block(0, i, 6, 1) = stress_d;
        dstrain_0(i, 0) -= delta;
    }
}

// -----------------------------------------------------------------------------

ChDampingKirchhoffRayleigh::ChDampingKirchhoffRayleigh(std::shared_ptr<ChElasticityKirchhoff> melasticity,
                                                       const double& mbeta) {
    this->beta = mbeta;
    this->section_elasticity = melasticity;
    this->updated = false;
}

void ChDampingKirchhoffRayleigh::ComputeStress(
    ChVector3d& n,           ///< forces  n_11, n_22, n_12 (per unit length)
    ChVector3d& m,           ///< torques m_11, m_22, m_12 (per unit length)
    const ChVector3d& deps,  ///< time derivative of strains   de_11/dt, de_22/dt, de_12/dt
    const ChVector3d& dkur,  ///< time derivative of curvature dk_11/dt, dk_22/dt, dk_12/dt
    const double z_inf,      ///< layer lower z value (along thickness coord)
    const double z_sup,      ///< layer upper z value (along thickness coord)
    const double angle       ///< layer angle respect to x (if needed)
) {
    if (!this->updated && this->section_elasticity->section) {
        this->section_elasticity->ComputeStiffnessMatrix(this->E_const, VNULL, VNULL, z_inf, z_sup, angle);
        this->updated = true;
    }
    ChVectorN<double, 6> mdstrain;
    ChVectorN<double, 6> mstress;
    mdstrain.segment(0, 3) = deps.eigen();
    mdstrain.segment(3, 3) = dkur.eigen();
    mstress = this->beta * this->E_const * mdstrain;
    n = mstress.segment(0, 3);
    m = mstress.segment(3, 3);
}

void ChDampingKirchhoffRayleigh::ComputeDampingMatrix(
    ChMatrixRef R,           ///< 6x6 material damping matrix values here
    const ChVector3d& deps,  ///< time derivative of strains   de_11/dt, de_22/dt, de_12/dt
    const ChVector3d& dkur,  ///< time derivative of curvature dk_11/dt, dk_22/dt, dk_12/dt
    const double z_inf,      ///< layer lower z value (along thickness coord)
    const double z_sup,      ///< layer upper z value (along thickness coord)
    const double angle       ///< layer angle respect to x (if needed) -not used in this, isotropic
) {
    R = this->beta * this->E_const;
}

// -----------------------------------------------------------------------------

ChMaterialShellKirchhoff::ChMaterialShellKirchhoff(std::shared_ptr<ChElasticityKirchhoff> melasticity) {
    this->SetElasticity(melasticity);
}

ChMaterialShellKirchhoff::ChMaterialShellKirchhoff(std::shared_ptr<ChElasticityKirchhoff> melasticity,
                                                   std::shared_ptr<ChPlasticityKirchhoff> mplasticity) {
    this->SetElasticity(melasticity);
    this->SetPlasticity(mplasticity);
}

ChMaterialShellKirchhoff::ChMaterialShellKirchhoff(std::shared_ptr<ChElasticityKirchhoff> melasticity,
                                                   std::shared_ptr<ChPlasticityKirchhoff> mplasticity,
                                                   std::shared_ptr<ChDampingKirchhoff> mdamping) {
    this->SetElasticity(melasticity);

    if (mplasticity)
        this->SetPlasticity(mplasticity);

    if (mdamping)
        this->SetDamping(mdamping);
}

void ChMaterialShellKirchhoff::ComputeStress(ChVector3d& n,
                                             ChVector3d& m,
                                             const ChVector3d& eps,
                                             const ChVector3d& kur,
                                             const double z_inf,
                                             const double z_sup,
                                             const double angle,
                                             ChShellKirchhoffInternalData* mdata_new,
                                             const ChShellKirchhoffInternalData* mdata) {
    if (!plasticity || !mdata || !mdata_new)
        this->elasticity->ComputeStress(n, m, eps, kur, z_inf, z_sup, angle);
    else {
        this->plasticity->ComputeStressWithReturnMapping(n, m, *mdata_new, eps, kur, *mdata, z_inf, z_sup, angle);
    }
}

void ChMaterialShellKirchhoff::ComputeStiffnessMatrix(ChMatrixRef K,
                                                      const ChVector3d& eps,
                                                      const ChVector3d& kur,
                                                      const double z_inf,
                                                      const double z_sup,
                                                      const double angle,
                                                      const ChShellKirchhoffInternalData* mdata) {
    if (!plasticity || !mdata)
        this->elasticity->ComputeStiffnessMatrix(K, eps, kur, z_inf, z_sup, angle);
    else {
        this->plasticity->ComputeStiffnessMatrixElastoplastic(K, eps, kur, *mdata, z_inf, z_sup, angle);
    }
}

void ChMaterialShellKirchhoff::SetElasticity(std::shared_ptr<ChElasticityKirchhoff> melasticity) {
    elasticity = melasticity;
    elasticity->section = this;
}

void ChMaterialShellKirchhoff::SetPlasticity(std::shared_ptr<ChPlasticityKirchhoff> mplasticity) {
    plasticity = mplasticity;
    mplasticity->section = this;
}

void ChMaterialShellKirchhoff::SetDamping(std::shared_ptr<ChDampingKirchhoff> mdamping) {
    damping = mdamping;
    damping->section = this;
}

}  // end of namespace fea
}  // end of namespace chrono
