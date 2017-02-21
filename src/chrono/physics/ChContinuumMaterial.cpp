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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChContinuumMaterial.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------

ChContinuumMaterial::ChContinuumMaterial(const ChContinuumMaterial& other) {
    density = other.density;
}

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContinuumMaterial)

void ChContinuumMaterial::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContinuumMaterial>();
    // serialize parent class
    // serialize all member data:
    marchive << CHNVP(density, "density");
}

void ChContinuumMaterial::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChContinuumMaterial>();
    // deserialize parent class
    // stream in all member data:
    marchive >> CHNVP(density, "density");
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContinuumElastic)

ChContinuumElastic::ChContinuumElastic(double myoung, double mpoisson, double mdensity)
    : ChContinuumMaterial(mdensity) {
    E = myoung;
    Set_v(mpoisson);              // sets also G and l
    ComputeStressStrainMatrix();  // sets Elasticity matrix
    this->damping_M = 0;
    this->damping_K = 0;
}

ChContinuumElastic::ChContinuumElastic(const ChContinuumElastic& other) : ChContinuumMaterial(other) {
    E = other.E;
    v = other.v;
    G = other.G;
    l = other.l;
    damping_M = other.damping_M;
    damping_K = other.damping_K;

    StressStrainMatrix.CopyFromMatrix(other.StressStrainMatrix);
}

void ChContinuumElastic::Set_E(double m_E) {
    E = m_E;
    G = E / (2 * (1 + v));                  // fixed v, E, get G
    l = (v * E) / ((1 + v) * (1 - 2 * v));  // Lame's constant l
    ComputeStressStrainMatrix();            // updates Elasticity matrix
}

void ChContinuumElastic::Set_v(double m_v) {
    v = m_v;
    G = E / (2 * (1 + v));                  // fixed v, E, get G
    l = (v * E) / ((1 + v) * (1 - 2 * v));  // Lame's constant l
    ComputeStressStrainMatrix();            // updates Elasticity matrix
}

void ChContinuumElastic::Set_G(double m_G) {
    G = m_G;
    v = (E / (2 * G)) - 1;                  // fixed G, E, get v
    l = (v * E) / ((1 + v) * (1 - 2 * v));  // Lame's constant l
    ComputeStressStrainMatrix();            // updates Elasticity matrix
}

void ChContinuumElastic::ComputeStressStrainMatrix() {
    StressStrainMatrix.Reset(6, 6);
    StressStrainMatrix.SetElement(0, 0, (E * (1 - v)) / (1 + v) / (1 - 2 * v));
    // StressStrainMatrix.SetElement(1,1,StressStrainMatrix.GetElement(0,0));	//
    // StressStrainMatrix.SetElement(2,2,StressStrainMatrix.GetElement(0,0));	//per non ricalcolare; qual'è meglio?
    StressStrainMatrix.SetElement(1, 1, (E * (1 - v)) / (1 + v) / (1 - 2 * v));
    StressStrainMatrix.SetElement(2, 2, (E * (1 - v)) / (1 + v) / (1 - 2 * v));
    StressStrainMatrix.SetElement(0, 1, (E * (v)) / (1 + v) / (1 - 2 * v));
    StressStrainMatrix.SetElement(0, 2, (E * (v)) / (1 + v) / (1 - 2 * v));
    StressStrainMatrix.SetElement(1, 0, (E * (v)) / (1 + v) / (1 - 2 * v));
    StressStrainMatrix.SetElement(1, 2, (E * (v)) / (1 + v) / (1 - 2 * v));
    StressStrainMatrix.SetElement(2, 0, (E * (v)) / (1 + v) / (1 - 2 * v));
    StressStrainMatrix.SetElement(2, 1, (E * (v)) / (1 + v) / (1 - 2 * v));
    StressStrainMatrix.SetElement(3, 3, (E * (1 - 2 * v)) / (1 + v) / (1 - 2 * v) / 2);
    StressStrainMatrix.SetElement(4, 4, (E * (1 - 2 * v)) / (1 + v) / (1 - 2 * v) / 2);
    StressStrainMatrix.SetElement(5, 5, (E * (1 - 2 * v)) / (1 + v) / (1 - 2 * v) / 2);
}

void ChContinuumElastic::ComputeElasticStress(ChStressTensor<>& mstress, const ChStrainTensor<>& mstrain) const {
    mstress.XX() = mstrain.XX() * (l + 2 * G) + mstrain.YY() * l + mstrain.ZZ() * l;
    mstress.YY() = mstrain.XX() * l + mstrain.YY() * (l + 2 * G) + mstrain.ZZ() * l;
    mstress.ZZ() = mstrain.XX() * l + mstrain.YY() * l + mstrain.ZZ() * (l + 2 * G);
    mstress.XY() = mstrain.XY() * 2 * G;
    mstress.XZ() = mstrain.XZ() * 2 * G;
    mstress.YZ() = mstrain.YZ() * 2 * G;
}

void ChContinuumElastic::ComputeElasticStrain(ChStrainTensor<>& mstrain, const ChStressTensor<>& mstress) const {
    double invE = 1. / E;
    double invhG = 0.5 / G;
    mstrain.XX() = invE * (mstress.XX() - mstress.YY() * v - mstress.ZZ() * v);
    mstrain.YY() = invE * (-mstress.XX() * v + mstress.YY() - mstress.ZZ() * v);
    mstrain.ZZ() = invE * (-mstress.XX() * v - mstress.YY() * v + mstress.ZZ());
    mstrain.XY() = mstress.XY() * invhG;
    mstrain.XZ() = mstress.XZ() * invhG;
    mstrain.YZ() = mstress.YZ() * invhG;
}

void ChContinuumElastic::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContinuumElastic>();
    // serialize parent class
    ChContinuumMaterial::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(this->E);
    marchive << CHNVP(this->v);
    marchive << CHNVP(this->damping_M);
    marchive << CHNVP(this->damping_K);
}

void ChContinuumElastic::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChContinuumElastic>();
    // deserialize parent class
    ChContinuumMaterial::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(this->E);
    marchive >> CHNVP(this->v);
    this->Set_v(this->v);  // G and l from v
    marchive >> CHNVP(this->damping_M);
    marchive >> CHNVP(this->damping_K);
}

// -----------------------------------------------------------------------------

void ChContinuumElastoplastic::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContinuumElastoplastic>();
    // serialize parent class
    ChContinuumElastic::ArchiveOUT(marchive);
    // serialize all member data:
}

void ChContinuumElastoplastic::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChContinuumElastoplastic>();
    // deserialize parent class
    ChContinuumElastic::ArchiveIN(marchive);
    // stream in all member data:
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContinuumPlasticVonMises)

ChContinuumPlasticVonMises::ChContinuumPlasticVonMises(double myoung,
                                                       double mpoisson,
                                                       double mdensity,
                                                       double melastic_yeld,
                                                       double mplastic_yeld)
    : ChContinuumElastoplastic(myoung, mpoisson, mdensity),
      elastic_yeld(melastic_yeld),
      plastic_yeld(mplastic_yeld),
      flow_rate(1) {}

ChContinuumPlasticVonMises::ChContinuumPlasticVonMises(const ChContinuumPlasticVonMises& other)
    : ChContinuumElastoplastic(other) {
    elastic_yeld = other.elastic_yeld;
    plastic_yeld = other.plastic_yeld;
    flow_rate = other.flow_rate;
}

double ChContinuumPlasticVonMises::ComputeYeldFunction(const ChStressTensor<>& mstress) const {
    return (mstress.GetEquivalentVonMises() - this->elastic_yeld);
}

void ChContinuumPlasticVonMises::ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow,
                                                      const ChStrainTensor<>& mincrementstrain,
                                                      const ChStrainTensor<>& mlastelasticstrain,
                                                      const ChStrainTensor<>& mlastplasticstrain) const {
    ChStrainTensor<> guesselstrain(mlastelasticstrain);
    guesselstrain.MatrInc(mincrementstrain);  // assume increment is all elastic

    double vonm = guesselstrain.GetEquivalentVonMises();
    if (vonm > this->elastic_yeld) {
        ChVoightTensor<> mdev;
        guesselstrain.GetDeviatoricPart(mdev);
        mplasticstrainflow.CopyFromMatrix(mdev * ((vonm - this->elastic_yeld) / (vonm)));
    } else {
        mplasticstrainflow.FillElem(0);
    }
}

void ChContinuumPlasticVonMises::ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow,
                                                          const ChStrainTensor<>& mtotstrain) const {
    double vonm = mtotstrain.GetEquivalentVonMises();
    if (vonm > this->elastic_yeld) {
        ChVoightTensor<> mdev;
        mtotstrain.GetDeviatoricPart(mdev);
        mplasticstrainflow.CopyFromMatrix(mdev * ((vonm - this->elastic_yeld) / (vonm)));
    } else {
        mplasticstrainflow.FillElem(0);
    }
}

void ChContinuumPlasticVonMises::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContinuumPlasticVonMises>();
    // serialize parent class
    ChContinuumElastoplastic::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(this->elastic_yeld);
    marchive << CHNVP(this->plastic_yeld);
    marchive << CHNVP(this->flow_rate);
}

void ChContinuumPlasticVonMises::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChContinuumPlasticVonMises>();
    // deserialize parent class
    ChContinuumElastoplastic::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(this->elastic_yeld);
    marchive >> CHNVP(this->plastic_yeld);
    marchive >> CHNVP(this->flow_rate);
}

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContinuumDruckerPrager)

ChContinuumDruckerPrager::ChContinuumDruckerPrager(double myoung,
                                                   double mpoisson,
                                                   double mdensity,
                                                   double melastic_yeld,
                                                   double malpha,
                                                   double mdilatancy)
    : ChContinuumElastoplastic(myoung, mpoisson, mdensity),
      elastic_yeld(melastic_yeld),
      alpha(malpha),
      dilatancy(mdilatancy),
      hardening_limit(elastic_yeld),
      hardening_speed(0),
      flow_rate(1) {}

ChContinuumDruckerPrager::ChContinuumDruckerPrager(const ChContinuumDruckerPrager& other)
    : ChContinuumElastoplastic(other) {
    elastic_yeld = other.elastic_yeld;
    alpha = other.alpha;
    dilatancy = other.dilatancy;
    hardening_speed = other.hardening_speed;
    hardening_limit = other.hardening_limit;
    flow_rate = other.flow_rate;
}

void ChContinuumDruckerPrager::Set_from_MohrCoulomb(double phi, double cohesion, bool inner_approx) {
    if (inner_approx) {
        alpha = (2 * sin(phi)) / (sqrt(3.0) * (3.0 - sin(phi)));
        elastic_yeld = (6 * cohesion * cos(phi)) / (sqrt(3.0) * (3.0 - sin(phi)));
    } else {
        alpha = (2 * sin(phi)) / (sqrt(3.0) * (3.0 + sin(phi)));
        elastic_yeld = (6 * cohesion * cos(phi)) / (sqrt(3.0) * (3.0 + sin(phi)));
    }
}

double ChContinuumDruckerPrager::ComputeYeldFunction(const ChStressTensor<>& mstress) const {
    return (mstress.GetInvariant_I1() * this->alpha + sqrt(mstress.GetInvariant_J2()) - this->elastic_yeld);
}

void ChContinuumDruckerPrager::ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow,
                                                    const ChStrainTensor<>& mincrementstrain,
                                                    const ChStrainTensor<>& mlastelasticstrain,
                                                    const ChStrainTensor<>& mlastplasticstrain) const {
    ChStrainTensor<> guesselstrain(mlastelasticstrain);
    guesselstrain.MatrInc(mincrementstrain);  // assume increment is all elastic

    ChStressTensor<> mstress;
    this->ComputeElasticStress(mstress, guesselstrain);
    double fprager = this->ComputeYeldFunction(mstress);

    if (fprager > 0) {
        if (mstress.GetInvariant_I1() * this->alpha - sqrt(mstress.GetInvariant_J2()) * this->alpha * this->alpha -
                this->elastic_yeld >
            0) {
            // Case: tentative stress is in polar cone; a singular region where the gradient of
            // the yeld function (or flow potential) is not defined. Just project to vertex.
            ChStressTensor<> vertexstress;
            double vertcoord = this->elastic_yeld / (3 * this->alpha);
            vertexstress.XX() = vertcoord;
            vertexstress.YY() = vertcoord;
            vertexstress.ZZ() = vertcoord;
            ChStrainTensor<> vertexstrain;
            this->ComputeElasticStrain(vertexstrain, vertexstress);
            mplasticstrainflow.MatrSub(guesselstrain, vertexstrain);
        } else {
            // Case: tentative stress is out of the yeld cone.
            // Just project using the yeld (or flow potential) gradient.
            ChStrainTensor<> dFdS;
            ChStrainTensor<> dGdS;
            double devsq = sqrt(mstress.GetInvariant_J2());
            if (devsq > 10e-16) {
                double sixdevsq = 6 * devsq;

                dFdS.XX() = this->alpha + (2 * mstress.XX() - mstress.YY() - mstress.ZZ()) / sixdevsq;
                dFdS.YY() = this->alpha + (-mstress.XX() + 2 * mstress.YY() - mstress.ZZ()) / sixdevsq;
                dFdS.ZZ() = this->alpha + (-mstress.XX() - mstress.YY() + 2 * mstress.ZZ()) / sixdevsq;
                dFdS.XY() = mstress.XY() / devsq;
                dFdS.YZ() = mstress.YZ() / devsq;
                dFdS.XZ() = mstress.XZ() / devsq;

                dGdS.XX() = this->dilatancy + (2 * mstress.XX() - mstress.YY() - mstress.ZZ()) / sixdevsq;
                dGdS.YY() = this->dilatancy + (-mstress.XX() + 2 * mstress.YY() - mstress.ZZ()) / sixdevsq;
                dGdS.ZZ() = this->dilatancy + (-mstress.XX() - mstress.YY() + 2 * mstress.ZZ()) / sixdevsq;
                dGdS.XY() = mstress.XY() / devsq;
                dGdS.YZ() = mstress.YZ() / devsq;
                dGdS.XZ() = mstress.XZ() / devsq;
            } else {
                GetLog() << "      ... axial singularity - SHOULD NEVER OCCUR  - handled by polar cone\n";
                dFdS.FillElem(0);
                dFdS.XX() = 1;
                dFdS.YY() = 1;
                dFdS.ZZ() = 1;
                dGdS.FillElem(0);
                dGdS.XX() = 1;
                dGdS.YY() = 1;
                dGdS.ZZ() = 1;
            }
            ChStressTensor<> aux_dFdS_C;
            this->ComputeElasticStress(aux_dFdS_C, dFdS);

            ChMatrixNM<double, 1, 1> inner_up;
            inner_up.MatrTMultiply(aux_dFdS_C, mincrementstrain);
            ChMatrixNM<double, 1, 1> inner_dw;
            inner_dw.MatrTMultiply(aux_dFdS_C, dGdS);

            mplasticstrainflow.CopyFromMatrix(dGdS);
            mplasticstrainflow.MatrScale(inner_up(0) / inner_dw(0));
        }
    } else {
        mplasticstrainflow.FillElem(0);
    }
}

//***OBSOLETE***
void ChContinuumDruckerPrager::ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow,
                                                        const ChStrainTensor<>& mestrain) const {
    ChStressTensor<> mstress;
    this->ComputeElasticStress(mstress, mestrain);
    double prager = mstress.GetInvariant_I1() * this->alpha + sqrt(mstress.GetInvariant_J2());
    if (prager > this->elastic_yeld) {
        ChVoightTensor<> mdev;
        mstress.GetDeviatoricPart(mdev);
        double divisor = 2. * sqrt(mstress.GetInvariant_J2());
        if (divisor > 10e-20)
            mdev.MatrScale(1. / divisor);
        mdev.XX() += this->dilatancy;
        mdev.YY() += this->dilatancy;
        mdev.ZZ() += this->dilatancy;
        mplasticstrainflow.CopyFromMatrix(mdev);
    } else {
        mplasticstrainflow.FillElem(0);
    }
}

void ChContinuumDruckerPrager::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContinuumDruckerPrager>();
    // serialize parent class
    ChContinuumElastoplastic::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(this->elastic_yeld);
    marchive << CHNVP(this->alpha);
    marchive << CHNVP(this->dilatancy);
    marchive << CHNVP(this->hardening_speed);
    marchive << CHNVP(this->hardening_limit);
    marchive << CHNVP(this->flow_rate);
}

void ChContinuumDruckerPrager::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChContinuumDruckerPrager>();
    // deserialize parent class
    ChContinuumElastoplastic::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(this->elastic_yeld);
    marchive >> CHNVP(this->alpha);
    marchive >> CHNVP(this->dilatancy);
    marchive >> CHNVP(this->hardening_speed);
    marchive >> CHNVP(this->hardening_limit);
    marchive >> CHNVP(this->flow_rate);
}

}  // end namespace fea
}  // end namespace chrono
