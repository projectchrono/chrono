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

#include "chrono/fea/ChContinuumMaterial.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------

ChContinuumMaterial::ChContinuumMaterial(const ChContinuumMaterial& other) {
    m_density = other.m_density;
}

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContinuumMaterial)

void ChContinuumMaterial::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContinuumMaterial>();
    // serialize parent class
    // serialize all member data:
    archive_out << CHNVP(m_density);
}

void ChContinuumMaterial::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContinuumMaterial>();
    // deserialize parent class
    // stream in all member data:
    archive_in >> CHNVP(m_density);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContinuumElastic)

ChContinuumElastic::ChContinuumElastic(double young, double poisson, double density) : ChContinuumMaterial(density) {
    m_E = young;
    SetPoissonRatio(poisson);     // sets also Lamé
    ComputeStressStrainMatrix();  // sets Elasticity matrix
    this->m_rayl_damping_alpha = 0;
    this->m_rayl_damping_beta = 0;
}

ChContinuumElastic::ChContinuumElastic(const ChContinuumElastic& other) : ChContinuumMaterial(other) {
    m_E = other.m_E;
    m_poisson = other.m_poisson;
    m_lamefirst = other.m_lamefirst;
    m_rayl_damping_alpha = other.m_rayl_damping_alpha;
    m_rayl_damping_beta = other.m_rayl_damping_beta;

    StressStrainMatrix = other.StressStrainMatrix;
}

void ChContinuumElastic::SetYoungModulus(double E) {
    m_E = E;
    m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
    ComputeStressStrainMatrix();                                                // updates Elasticity matrix
}

void ChContinuumElastic::SetPoissonRatio(double v) {
    m_poisson = v;
    m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
    ComputeStressStrainMatrix();                                                // updates Elasticity matrix
}

void ChContinuumElastic::SetShearModulus(double G) {
    m_poisson = (m_E / (2 * G)) - 1;                                            // fixed G, E, get v
    m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
    ComputeStressStrainMatrix();                                                // updates Elasticity matrix
}

void ChContinuumElastic::ComputeStressStrainMatrix() {
    StressStrainMatrix.setZero(6, 6);
    StressStrainMatrix(0, 0) = (m_E * (1 - m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    // StressStrainMatrix(1,1)=StressStrainMatrix(0,0);	//
    // StressStrainMatrix(2,2)=StressStrainMatrix(0,0);	//per non ricalcolare; qual'� meglio?
    StressStrainMatrix(1, 1) = (m_E * (1 - m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    StressStrainMatrix(2, 2) = (m_E * (1 - m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    StressStrainMatrix(0, 1) = (m_E * (m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    StressStrainMatrix(0, 2) = (m_E * (m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    StressStrainMatrix(1, 0) = (m_E * (m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    StressStrainMatrix(1, 2) = (m_E * (m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    StressStrainMatrix(2, 0) = (m_E * (m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    StressStrainMatrix(2, 1) = (m_E * (m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson);
    StressStrainMatrix(3, 3) = (m_E * (1 - 2 * m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson) / 2;
    StressStrainMatrix(4, 4) = (m_E * (1 - 2 * m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson) / 2;
    StressStrainMatrix(5, 5) = (m_E * (1 - 2 * m_poisson)) / (1 + m_poisson) / (1 - 2 * m_poisson) / 2;
}

void ChContinuumElastic::ComputeElasticStress(ChStressTensor<>& stress, const ChStrainTensor<>& strain) const {
    double G = GetShearModulus();
    stress.XX() = strain.XX() * (m_lamefirst + 2 * G) + strain.YY() * m_lamefirst + strain.ZZ() * m_lamefirst;
    stress.YY() = strain.XX() * m_lamefirst + strain.YY() * (m_lamefirst + 2 * G) + strain.ZZ() * m_lamefirst;
    stress.ZZ() = strain.XX() * m_lamefirst + strain.YY() * m_lamefirst + strain.ZZ() * (m_lamefirst + 2 * G);
    stress.XY() = strain.XY() * 2 * G;
    stress.XZ() = strain.XZ() * 2 * G;
    stress.YZ() = strain.YZ() * 2 * G;
}

void ChContinuumElastic::ComputeElasticStrain(ChStrainTensor<>& strain, const ChStressTensor<>& stress) const {
    double invE = 1. / m_E;
    double invhG = 0.5 / GetShearModulus();
    strain.XX() = invE * (stress.XX() - stress.YY() * m_poisson - stress.ZZ() * m_poisson);
    strain.YY() = invE * (-stress.XX() * m_poisson + stress.YY() - stress.ZZ() * m_poisson);
    strain.ZZ() = invE * (-stress.XX() * m_poisson - stress.YY() * m_poisson + stress.ZZ());
    strain.XY() = stress.XY() * invhG;
    strain.XZ() = stress.XZ() * invhG;
    strain.YZ() = stress.YZ() * invhG;
}

void ChContinuumElastic::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContinuumElastic>();
    // serialize parent class
    ChContinuumMaterial::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_E);
    archive_out << CHNVP(m_poisson);
    archive_out << CHNVP(m_rayl_damping_alpha);
    archive_out << CHNVP(m_rayl_damping_beta);
}

void ChContinuumElastic::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContinuumElastic>();
    // deserialize parent class
    ChContinuumMaterial::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_E);
    archive_in >> CHNVP(m_poisson);
    this->SetPoissonRatio(m_poisson);  // G and l from v
    archive_in >> CHNVP(m_rayl_damping_alpha);
    archive_in >> CHNVP(m_rayl_damping_beta);
}

// -----------------------------------------------------------------------------

void ChContinuumElastoplastic::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContinuumElastoplastic>();
    // serialize parent class
    ChContinuumElastic::ArchiveOut(archive_out);
    // serialize all member data:
}

void ChContinuumElastoplastic::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContinuumElastoplastic>();
    // deserialize parent class
    ChContinuumElastic::ArchiveIn(archive_in);
    // stream in all member data:
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContinuumPlasticVonMises)

ChContinuumPlasticVonMises::ChContinuumPlasticVonMises(double young,
                                                       double poisson,
                                                       double density,
                                                       double elastic_yeld,
                                                       double plastic_yeld)
    : ChContinuumElastoplastic(young, poisson, density),
      m_elastic_yield(elastic_yeld),
      m_plastic_yield(plastic_yeld),
      m_plastic_flow_rate(1) {}

ChContinuumPlasticVonMises::ChContinuumPlasticVonMises(const ChContinuumPlasticVonMises& other)
    : ChContinuumElastoplastic(other) {
    m_elastic_yield = other.m_elastic_yield;
    m_plastic_yield = other.m_plastic_yield;
    m_plastic_flow_rate = other.m_plastic_flow_rate;
}

double ChContinuumPlasticVonMises::ComputeYieldFunction(const ChStressTensor<>& stress) const {
    return (stress.GetEquivalentVonMises() - this->m_elastic_yield);
}

void ChContinuumPlasticVonMises::ComputeReturnMapping(ChStrainTensor<>& plasticstrainflow,
                                                      const ChStrainTensor<>& incrementstrain,
                                                      const ChStrainTensor<>& lastelasticstrain,
                                                      const ChStrainTensor<>& lastplasticstrain) const {
    ChStrainTensor<> guesselstrain(lastelasticstrain);
    guesselstrain += incrementstrain;  // assume increment is all elastic

    double vonm = guesselstrain.GetEquivalentVonMises();
    if (vonm > this->m_elastic_yield) {
        ChVoightTensor<> mdev;
        guesselstrain.GetDeviatoricPart(mdev);
        plasticstrainflow = mdev * ((vonm - this->m_elastic_yield) / (vonm));
    } else {
        plasticstrainflow.setZero();
    }
}

void ChContinuumPlasticVonMises::ComputePlasticStrainFlow(ChStrainTensor<>& plasticstrainflow,
                                                          const ChStrainTensor<>& totstrain) const {
    double vonm = totstrain.GetEquivalentVonMises();
    if (vonm > this->m_elastic_yield) {
        ChVoightTensor<> mdev;
        totstrain.GetDeviatoricPart(mdev);
        plasticstrainflow = mdev * ((vonm - this->m_elastic_yield) / (vonm));
    } else {
        plasticstrainflow.setZero();
    }
}

void ChContinuumPlasticVonMises::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContinuumPlasticVonMises>();
    // serialize parent class
    ChContinuumElastoplastic::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_elastic_yield);
    archive_out << CHNVP(m_plastic_yield);
    archive_out << CHNVP(m_plastic_flow_rate);
}

void ChContinuumPlasticVonMises::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContinuumPlasticVonMises>();
    // deserialize parent class
    ChContinuumElastoplastic::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_elastic_yield);
    archive_in >> CHNVP(m_plastic_yield);
    archive_in >> CHNVP(m_plastic_flow_rate);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContinuumDruckerPrager)

ChContinuumDruckerPrager::ChContinuumDruckerPrager(double young,
                                                   double poisson,
                                                   double density,
                                                   double elastic_yield,
                                                   double alpha,
                                                   double dilatancy)
    : ChContinuumElastoplastic(young, poisson, density),
      m_elastic_yield(elastic_yield),
      m_alpha(alpha),
      m_dilatancy(dilatancy),
      m_hardening_limit(elastic_yield),
      m_hardening_speed(0),
      m_plastic_flow_rate(1) {}

ChContinuumDruckerPrager::ChContinuumDruckerPrager(const ChContinuumDruckerPrager& other)
    : ChContinuumElastoplastic(other) {
    m_elastic_yield = other.m_elastic_yield;
    m_alpha = other.m_alpha;
    m_dilatancy = other.m_dilatancy;
    m_hardening_speed = other.m_hardening_speed;
    m_hardening_limit = other.m_hardening_limit;
    m_plastic_flow_rate = other.m_plastic_flow_rate;
}

void ChContinuumDruckerPrager::SetFromMohrCoulomb(double phi, double cohesion, bool inner_approx) {
    if (inner_approx) {
        m_alpha = (2 * std::sin(phi)) / (std::sqrt(3.0) * (3.0 - std::sin(phi)));
        m_elastic_yield = (6 * cohesion * std::cos(phi)) / (std::sqrt(3.0) * (3.0 - std::sin(phi)));
    } else {
        m_alpha = (2 * std::sin(phi)) / (std::sqrt(3.0) * (3.0 + std::sin(phi)));
        m_elastic_yield = (6 * cohesion * std::cos(phi)) / (std::sqrt(3.0) * (3.0 + std::sin(phi)));
    }
}

double ChContinuumDruckerPrager::ComputeYieldFunction(const ChStressTensor<>& mstress) const {
    return (mstress.GetInvariant_I1() * this->m_alpha + std::sqrt(mstress.GetInvariant_J2()) - this->m_elastic_yield);
}

void ChContinuumDruckerPrager::ComputeReturnMapping(ChStrainTensor<>& plasticstrainflow,
                                                    const ChStrainTensor<>& incrementstrain,
                                                    const ChStrainTensor<>& lastelasticstrain,
                                                    const ChStrainTensor<>& lastplasticstrain) const {
    ChStrainTensor<> guesselstrain(lastelasticstrain);
    guesselstrain += incrementstrain;  // assume increment is all elastic

    ChStressTensor<> mstress;
    this->ComputeElasticStress(mstress, guesselstrain);
    double fprager = this->ComputeYieldFunction(mstress);

    if (fprager > 0) {
        if (mstress.GetInvariant_I1() * this->m_alpha -
                std::sqrt(mstress.GetInvariant_J2()) * this->m_alpha * this->m_alpha - this->m_elastic_yield >
            0) {
            // Case: tentative stress is in polar cone; a singular region where the gradient of
            // the yield function (or flow potential) is not defined. Just project to vertex.
            ChStressTensor<> vertexstress;
            double vertcoord = this->m_elastic_yield / (3 * this->m_alpha);
            vertexstress.XX() = vertcoord;
            vertexstress.YY() = vertcoord;
            vertexstress.ZZ() = vertcoord;
            ChStrainTensor<> vertexstrain;
            this->ComputeElasticStrain(vertexstrain, vertexstress);
            plasticstrainflow = guesselstrain - vertexstrain;
        } else {
            // Case: tentative stress is out of the yield cone.
            // Just project using the yield (or flow potential) gradient.
            ChStrainTensor<> dFdS;
            ChStrainTensor<> dGdS;
            double devsq = std::sqrt(mstress.GetInvariant_J2());
            if (devsq > 10e-16) {
                double sixdevsq = 6 * devsq;

                dFdS.XX() = this->m_alpha + (2 * mstress.XX() - mstress.YY() - mstress.ZZ()) / sixdevsq;
                dFdS.YY() = this->m_alpha + (-mstress.XX() + 2 * mstress.YY() - mstress.ZZ()) / sixdevsq;
                dFdS.ZZ() = this->m_alpha + (-mstress.XX() - mstress.YY() + 2 * mstress.ZZ()) / sixdevsq;
                dFdS.XY() = mstress.XY() / devsq;
                dFdS.YZ() = mstress.YZ() / devsq;
                dFdS.XZ() = mstress.XZ() / devsq;

                dGdS.XX() = this->m_dilatancy + (2 * mstress.XX() - mstress.YY() - mstress.ZZ()) / sixdevsq;
                dGdS.YY() = this->m_dilatancy + (-mstress.XX() + 2 * mstress.YY() - mstress.ZZ()) / sixdevsq;
                dGdS.ZZ() = this->m_dilatancy + (-mstress.XX() - mstress.YY() + 2 * mstress.ZZ()) / sixdevsq;
                dGdS.XY() = mstress.XY() / devsq;
                dGdS.YZ() = mstress.YZ() / devsq;
                dGdS.XZ() = mstress.XZ() / devsq;
            } else {
                std::cerr << "Error: axial singularity - SHOULD NEVER OCCUR  - handled by polar cone" << std::endl;
                dFdS.setZero();
                dFdS.XX() = 1;
                dFdS.YY() = 1;
                dFdS.ZZ() = 1;
                dGdS.setZero();
                dGdS.XX() = 1;
                dGdS.YY() = 1;
                dGdS.ZZ() = 1;
            }
            ChStressTensor<> aux_dFdS_C;
            this->ComputeElasticStress(aux_dFdS_C, dFdS);

            ChMatrixNM<double, 1, 1> inner_up;
            inner_up = aux_dFdS_C.transpose() * incrementstrain;
            ChMatrixNM<double, 1, 1> inner_dw;
            inner_dw = aux_dFdS_C.transpose() * dGdS;

            plasticstrainflow = dGdS;
            plasticstrainflow *= inner_up(0) / inner_dw(0);
        }
    } else {
        plasticstrainflow.setZero();
    }
}

//// OBSOLETE
void ChContinuumDruckerPrager::ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow,
                                                        const ChStrainTensor<>& mestrain) const {
    ChStressTensor<> mstress;
    this->ComputeElasticStress(mstress, mestrain);
    double prager = mstress.GetInvariant_I1() * this->m_alpha + std::sqrt(mstress.GetInvariant_J2());
    if (prager > this->m_elastic_yield) {
        ChVoightTensor<> mdev;
        mstress.GetDeviatoricPart(mdev);
        double divisor = 2. * std::sqrt(mstress.GetInvariant_J2());
        if (divisor > 10e-20)
            mdev *= 1. / divisor;
        mdev.XX() += this->m_dilatancy;
        mdev.YY() += this->m_dilatancy;
        mdev.ZZ() += this->m_dilatancy;
        mplasticstrainflow = mdev;
    } else {
        mplasticstrainflow.setZero();
    }
}

void ChContinuumDruckerPrager::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContinuumDruckerPrager>();
    // serialize parent class
    ChContinuumElastoplastic::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(m_elastic_yield);
    archive_out << CHNVP(m_alpha);
    archive_out << CHNVP(m_dilatancy);
    archive_out << CHNVP(m_hardening_speed);
    archive_out << CHNVP(m_hardening_limit);
    archive_out << CHNVP(m_plastic_flow_rate);
}

void ChContinuumDruckerPrager::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContinuumDruckerPrager>();
    // deserialize parent class
    ChContinuumElastoplastic::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(m_elastic_yield);
    archive_in >> CHNVP(m_alpha);
    archive_in >> CHNVP(m_dilatancy);
    archive_in >> CHNVP(m_hardening_speed);
    archive_in >> CHNVP(m_hardening_limit);
    archive_in >> CHNVP(m_plastic_flow_rate);
}

}  // end namespace fea
}  // end namespace chrono
