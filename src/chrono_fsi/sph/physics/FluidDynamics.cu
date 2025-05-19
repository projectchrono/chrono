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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Class for performing time integration in fluid system.
// =============================================================================

#include <thrust/execution_policy.h>
#include <thrust/scan.h>
#include <thrust/logical.h>

#include "chrono/utils/ChConstants.h"
#include "chrono_fsi/sph/physics/FluidDynamics.cuh"
#include "chrono_fsi/sph/physics/FsiForceWCSPH.cuh"
#include "chrono_fsi/sph/physics/FsiForceISPH.cuh"
#include "chrono_fsi/sph/physics/SphGeneral.cuh"

using std::cout;
using std::endl;

namespace chrono {
namespace fsi {
namespace sph {

FluidDynamics::FluidDynamics(FsiDataManager& data_mgr, BceManager& bce_mgr, bool verbose, bool check_errors)
    : m_data_mgr(data_mgr), m_verbose(verbose), m_check_errors(check_errors) {
    collisionSystem = chrono_types::make_shared<CollisionSystem>(data_mgr);

    if (m_data_mgr.paramsH->integration_scheme == IntegrationScheme::IMPLICIT_SPH)
        forceSystem = chrono_types::make_shared<FsiForceISPH>(data_mgr, bce_mgr, verbose);
    else
        forceSystem = chrono_types::make_shared<FsiForceWCSPH>(data_mgr, bce_mgr, verbose);
}

FluidDynamics::~FluidDynamics() {}

// -----------------------------------------------------------------------------

void FluidDynamics::Initialize() {
    cudaMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(ChFsiParamsSPH));
    cudaMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));
    cudaMemcpyFromSymbol(m_data_mgr.paramsH.get(), paramsD, sizeof(ChFsiParamsSPH));

    forceSystem->Initialize();
    collisionSystem->Initialize();
}

// -----------------------------------------------------------------------------

void FluidDynamics::ProximitySearch() {
    collisionSystem->ArrangeData(m_data_mgr.sphMarkers_D, m_data_mgr.sortedSphMarkers2_D);
    collisionSystem->NeighborSearch(m_data_mgr.sortedSphMarkers2_D);
}

// -----------------------------------------------------------------------------

void FluidDynamics::CopySortedMarkers(const std::shared_ptr<SphMarkerDataD>& in, std::shared_ptr<SphMarkerDataD>& out) {
    thrust::copy(in->posRadD.begin(), in->posRadD.end(), out->posRadD.begin());
    thrust::copy(in->velMasD.begin(), in->velMasD.end(), out->velMasD.begin());
    thrust::copy(in->rhoPresMuD.begin(), in->rhoPresMuD.end(), out->rhoPresMuD.begin());
    if (m_data_mgr.paramsH->elastic_SPH) {
        thrust::copy(in->tauXxYyZzD.begin(), in->tauXxYyZzD.end(), out->tauXxYyZzD.begin());
        thrust::copy(in->tauXyXzYzD.begin(), in->tauXyXzYzD.end(), out->tauXyXzYzD.begin());
    }
}

//// TODO - revisit application of particle shifting (explicit schemes)
////        currently, a new v_XSPH is calculated at every force evaluation and used in the subsequent position update
////        should this be done only once per step?

void FluidDynamics::DoStepDynamics(std::shared_ptr<SphMarkerDataD> y, Real t, Real h, IntegrationScheme scheme) {
    switch (scheme) {
        case IntegrationScheme::EULER: {
            Real dummy = 0;  // force calculation for WCSPH does not need the step size

            forceSystem->ForceSPH(y, t, dummy);  // f(t_n, y_n)
            EulerStep(y, h);                     // y <==  y_{n+1} = y_n + h * f(t_n, y_n)
            ApplyBoundaryConditions(y);

            break;
        }

        case IntegrationScheme::RK2: {
            Real dummy = 0;  // force calculation for WCSPH does not need the step size

            auto& y_tmp = m_data_mgr.sortedSphMarkers1_D;
            CopySortedMarkers(y, y_tmp);  // y_tmp <- y_n

            forceSystem->ForceSPH(y, t, dummy);  // f(t_n, y_n)
            EulerStep(y_tmp, h / 2);             // y_tmp <==  K1 = y_n + (h/2) * f(t_n, y_n)
            ApplyBoundaryConditions(y_tmp);

            forceSystem->ForceSPH(y_tmp, t + h / 2, dummy);  // f(t_n + h/2, K1)
            EulerStep(y, h);                                 // y <== y_{n+1} = y_n + h * f(t_n + h/2, K1)
            ApplyBoundaryConditions(y);

            break;
        }

        case IntegrationScheme::SYMPLECTIC: {
            Real dummy = 0;  // force calculation for WCSPH does not need the step size

            auto& y_tmp = m_data_mgr.sortedSphMarkers1_D;
            CopySortedMarkers(y, y_tmp);  // y_tmp <- y_n

            forceSystem->ForceSPH(y, t, dummy);  // f(t_n, y_n)
            EulerStep(y_tmp, h / 2);             // y_tmp <== y_{n+1/2} = y_n + (h/2) * f(t_n, y_n)
            ApplyBoundaryConditions(y_tmp);

            forceSystem->ForceSPH(y_tmp, t + h / 2, dummy);  // f_{n+1/2} = f(t_n + h/2, y_{n+1/2})
            MidpointStep(y, h);                              // y_{n+1} = y_n + h * f_{n+1/2}

            break;
        }

        case IntegrationScheme::IMPLICIT_SPH: {
            forceSystem->ForceSPH(y, t, h);
            ApplyBoundaryConditions(y);

            break;
        }
    }
}

// -----------------------------------------------------------------------------

__global__ void UpdateActivityD(const Real4* posRadD,
                                Real3* velMasD,
                                const Real3* pos_bodies_D,
                                const Real3* pos_nodes1D_D,
                                const Real3* pos_nodes2D_D,
                                int32_t* activityIdentifierD,
                                int32_t* extendedActivityIdD,
                                const Real4* rhoPreMuD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers) {
        return;
    }

    // Set the particle as an active particle
    activityIdentifierD[index] = 1;
    extendedActivityIdD[index] = 1;

    size_t numFsiBodies = countersD.numFsiBodies;
    size_t numFsiNodes1D = countersD.numFsiNodes1D;
    size_t numFsiNodes2D = countersD.numFsiNodes2D;
    size_t numTotal = numFsiBodies + numFsiNodes1D + numFsiNodes2D;

    // Check the activity of this particle
    uint isNotActive = 0;
    uint isNotExtended = 0;
    Real3 Acdomain = paramsD.bodyActiveDomain;
    Real3 ExAcdomain = paramsD.bodyActiveDomain + mR3(2 * paramsD.h_multiplier * paramsD.h);
    Real3 domainDims = paramsD.boxDims;
    Real3 domainOrigin = paramsD.worldOrigin;
    bool x_periodic = paramsD.x_periodic;
    bool y_periodic = paramsD.y_periodic;
    bool z_periodic = paramsD.z_periodic;

    Real3 posRadA = mR3(posRadD[index]);

    for (uint num = 0; num < numFsiBodies; num++) {
        Real3 detPos = posRadA - pos_bodies_D[num];
        if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || abs(detPos.z) > Acdomain.z)
            isNotActive = isNotActive + 1;
        if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || abs(detPos.z) > ExAcdomain.z)
            isNotExtended = isNotExtended + 1;
    }

    for (uint num = 0; num < numFsiNodes1D; num++) {
        Real3 detPos = posRadA - pos_nodes1D_D[num];
        if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || abs(detPos.z) > Acdomain.z)
            isNotActive = isNotActive + 1;
        if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || abs(detPos.z) > ExAcdomain.z)
            isNotExtended = isNotExtended + 1;
    }

    for (uint num = 0; num < numFsiNodes2D; num++) {
        Real3 detPos = posRadA - pos_nodes2D_D[num];
        if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || abs(detPos.z) > Acdomain.z)
            isNotActive = isNotActive + 1;
        if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || abs(detPos.z) > ExAcdomain.z)
            isNotExtended = isNotExtended + 1;
    }

    // Set the particle as an inactive particle if needed
    if (isNotActive == numTotal && numTotal > 0) {
        activityIdentifierD[index] = 0;
        velMasD[index] = mR3(0.0);
    }
    if (isNotExtended == numTotal && numTotal > 0)
        extendedActivityIdD[index] = 0;

    // Check if the particle is outside the zombie domain
    if (IsFluidParticle(rhoPreMuD[index].w)) {
        bool outside_domain = false;

        // Check X boundaries - only inactivate if not periodic
        if (!x_periodic && (posRadA.x < domainOrigin.x || posRadA.x > domainOrigin.x + domainDims.x)) {
            outside_domain = true;
        }

        // Check Y boundaries - only inactivate if not periodic
        if (!y_periodic && (posRadA.y < domainOrigin.y || posRadA.y > domainOrigin.y + domainDims.y)) {
            outside_domain = true;
        }

        // Check Z boundaries - only inactivate if not periodic
        if (!z_periodic && (posRadA.z < domainOrigin.z || posRadA.z > domainOrigin.z + domainDims.z)) {
            outside_domain = true;
        }

        if (outside_domain) {
            activityIdentifierD[index] = -1;
            extendedActivityIdD[index] = -1;
            velMasD[index] = mR3(0.0);
        }
    }

    return;
}

void FluidDynamics::UpdateActivity(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    uint numBlocks, numThreads;
    computeGridSize((uint)m_data_mgr.countersH->numAllMarkers, 1024, numBlocks, numThreads);

    UpdateActivityD<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
        mR3CAST(m_data_mgr.fsiMesh1DState_D->pos), mR3CAST(m_data_mgr.fsiMesh2DState_D->pos),
        INT_32CAST(m_data_mgr.activityIdentifierOriginalD), INT_32CAST(m_data_mgr.extendedActivityIdentifierOriginalD),
        mR4CAST(sphMarkersD->rhoPresMuD));
    cudaCheckError();
}

// -----------------------------------------------------------------------------

// Resize data based on the active particles
// Custom functor for exclusive scan that treats -1 (zombie particles) the same as 0 (sleep particles)
struct ActivityScanOp {
    __host__ __device__ int operator()(const int& a, const int& b) const {
        // Treat -1 the same as 0 (only add positive values)
        int b_value = (b <= 0) ? 0 : b;
        return a + b_value;
    }
};

bool FluidDynamics::CheckActivityArrayResize() {
    auto& countersH = m_data_mgr.countersH;

    // Exclusive scan for extended activity identifier using custom functor to handle -1 values
    thrust::exclusive_scan(thrust::device,                                          // execution policy
                           m_data_mgr.extendedActivityIdentifierOriginalD.begin(),  // in start
                           m_data_mgr.extendedActivityIdentifierOriginalD.end(),    // in end
                           m_data_mgr.prefixSumExtendedActivityIdD.begin(),         // out start
                           0,                                                       // initial value
                           ActivityScanOp());

    // Copy the last element of prefixSumD to host and since we used exclusive scan, need to add the last flag
    uint lastPrefixVal = m_data_mgr.prefixSumExtendedActivityIdD[countersH->numAllMarkers - 1];
    int32_t lastFlagInt32;
    cudaMemcpy(&lastFlagInt32,
               thrust::raw_pointer_cast(&m_data_mgr.extendedActivityIdentifierOriginalD[countersH->numAllMarkers - 1]),
               sizeof(int32_t), cudaMemcpyDeviceToHost);
    uint lastFlag = (lastFlagInt32 > 0) ? 1 : 0;  // Only count positive values

    countersH->numExtendedParticles = lastPrefixVal + lastFlag;

    return countersH->numExtendedParticles < countersH->numAllMarkers;
}

// -----------------------------------------------------------------------------

__device__ void PositionEulerStep(Real dT, const Real3& vel, Real4& pos) {
    Real3 p = mR3(pos);
    p += dT * vel;
    pos = mR4(p, pos.w);
}

__device__ void PositionMidpointStep(Real dT, const Real3& vel, const Real3& acc, Real4& pos) {
    Real3 p = mR3(pos);
    p += dT * vel + 0.5 * dT * dT * acc;
}

__device__ void VelocityEulerStep(Real dT, const Real3& acc, Real3& vel) {
    vel += dT * acc;
}

__device__ void DensityEulerStep(Real dT, const Real& deriv, EosType eos, Real4& rho_p) {
    rho_p.x += dT * deriv;
    rho_p.y = Eos(rho_p.x, eos);
}

__device__ void TauEulerStep(Real dT,
                             const Real3& deriv_tau_diag,
                             const Real3& deriv_tau_offdiag,
                             bool close_to_surface,
                             Real3& tau_diag,
                             Real3& tau_offdiag,
                             Real4& rho_p) {
    Real3 new_tau_diag = tau_diag + dT * deriv_tau_diag;
    Real3 new_tau_offdiag = tau_offdiag + dT * deriv_tau_offdiag;

    // Check for plastic flow
    Real p_n = -CH_1_3 * (tau_diag.x + tau_diag.y + tau_diag.z);
    Real p_tr = -CH_1_3 * (new_tau_diag.x + new_tau_diag.y + new_tau_diag.z);
    tau_diag += mR3(p_n);
    new_tau_diag += mR3(p_tr);

    Real tau_n = square(tau_diag.x) + square(tau_diag.y) + square(tau_diag.z) +                             //
                 2 * (square(tau_offdiag.x) + square(tau_offdiag.y) + square(tau_offdiag.z));               //
    Real tau_tr = square(new_tau_diag.x) + square(new_tau_diag.y) + square(new_tau_diag.z) +                //
                  2 * (square(new_tau_offdiag.x) + square(new_tau_offdiag.y) + square(new_tau_offdiag.z));  //
    tau_n = sqrt(0.5 * tau_n);
    tau_tr = sqrt(0.5 * tau_tr);
    Real Chi = abs(tau_tr - tau_n) * paramsD.INV_G_shear / dT;

    // Should use the positive magnitude according to "A constitutive law for dense granular flows" Nature 2006
    Real mu_s = paramsD.mu_fric_s;
    Real mu_2 = paramsD.mu_fric_2;
    // Real s_0 = mu_s * p_tr;
    // Real s_2 = mu_2 * p_tr;
    // Real xi = 1.1;
    Real dia = paramsD.ave_diam;
    Real I0 = paramsD.mu_I0;  // xi*dia*sqrt(rhoPresMu.x);//
    Real I = Chi * dia * sqrt(paramsD.rho0 / (p_tr + 1.0e-9));

    Real coh = paramsD.Coh_coeff;
    // Real Chi_cri = 0.1;
    // if (Chi < Chi_cri){
    //     coh = paramsD.Coh_coeff * (1.0 - sin(-1.57 + 3.14 * (Chi / Chi_cri))) / 2.0;
    //     // coh = paramsD.Coh_coeff * (1.0 - I / I_cri);
    // } else {
    //     coh = 0.0;
    // }

    Real inv_mus = 1.0 / paramsD.mu_fric_s;
    Real p_cri = -coh * inv_mus;
    if (p_tr < p_cri) {
        new_tau_diag = mR3(0.0);
        new_tau_offdiag = mR3(0.0);
        p_tr = 0.0;
    } else {
        Real mu = mu_s + (mu_2 - mu_s) * (I + 1.0e-9) / (I0 + I + 1.0e-9);
        // Real G0 = paramsD.G_shear;
        // Real alpha = xi*G0*I0*(dT)*sqrt(p_tr);
        // Real B0 = s_2 + tau_tr + alpha;
        // Real H0 = s_2*tau_tr + s_0*alpha;
        // Real tau_n1 = (B0+sqrt(B0*B0-4*H0))/(2*H0+1e-9);
        // if(tau_tr>s_0){
        //     Real coeff = tau_n1/(tau_tr+1e-9);
        //     updatedTauXxYyZz = updatedTauXxYyZz*coeff;
        //     updatedTauXyXzYz = updatedTauXyXzYz*coeff;
        // }
        Real tau_max = p_tr * mu + coh;  // p_tr*paramsD.Q_FA;
        // should use tau_max instead of s_0 according to
        // "A constitutive law for dense granular flows" Nature 2006
        if (tau_tr > tau_max) {
            Real coeff = tau_max / (tau_tr + 1e-9);
            new_tau_diag *= coeff;
            new_tau_offdiag *= coeff;
        }
    }

    // Set stress to zero if the particle is close to free surface
    if (close_to_surface == 1) {
        new_tau_diag = mR3(0.0);
        new_tau_offdiag = mR3(0.0);
        p_tr = 0.0;
    }

    tau_diag = new_tau_diag - mR3(p_tr);
    tau_offdiag = new_tau_offdiag;

    rho_p.y = p_tr;
    rho_p.x = paramsD.rho0;
}

// Kernel to update the fluid properities of a particle, using an explicit Euler step.
// First, update the particle position and velocity. Next,
// - For a CFD problem, advance the density and calculate pressure from the Equation of State;
// - For a CRM problem, update the stress tensor and the pressure (density is kept constant).
//
// Important note: the derivVelRhoD calculated by ChForceExplicitSPH is the negative of actual time
// derivative. That is important to keep the derivVelRhoD to be the force/mass for fsi forces.
// - calculate the force, that is f=m dv/dt
// - derivVelRhoD[index] *= paramsD.markerMass;
__global__ void EulerStep_D(Real4* posRadD,
                            Real3* velMasD,
                            Real4* rhoPresMuD,
                            Real3* tauXxYyZzD,
                            Real3* tauXyXzYzD,
                            const Real3* vel_XSPH_D,
                            const Real4* derivVelRhoD,
                            const Real3* derivTauXxYyZzD,
                            const Real3* derivTauXyXzYzD,
                            const uint* freeSurfaceIdD,
                            const int32_t* activityIdentifierSortedD,
                            const uint numActive,
                            Real dT) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    // Only update active SPH particles, not extended active particles
    if (IsBceMarker(rhoPresMuD[index].w) || activityIdentifierSortedD[index] <= 0)
        return;

    // Euler step for position
    PositionEulerStep(dT, velMasD[index] + vel_XSPH_D[index], posRadD[index]);

    // Euler step for velocity
    VelocityEulerStep(dT, mR3(derivVelRhoD[index]), velMasD[index]);

    if (paramsD.elastic_SPH) {
        // Euler step for tau and pressure update
        TauEulerStep(dT, derivTauXxYyZzD[index], derivTauXyXzYzD[index], freeSurfaceIdD[index], tauXxYyZzD[index],
                     tauXyXzYzD[index], rhoPresMuD[index]);
    } else {
        // Euler step for density and pressure update from EOS
        DensityEulerStep(dT, derivVelRhoD[index].w, paramsD.eos_type, rhoPresMuD[index]);
    }
}

// Kernel to update the fluid properities of a particle, using an mid-point step.
// Note: the derivatives (provided in input vectors) are assumed to have been calculated at the mid-point!
// The mid-point updates for position and velocitie are:
//    v_{n+1} = v_n + h * F_{n+1/2}
//    r_{n+1} = r_n + h * (v_{n+1} + v_n) / 2
// These are implemented in reverse order (because the velocity update would overwrite v_n) as:
//    r_{n+1} = r_n + h * v_n + 0.5 * h^2 * F_{n+1/2}
//    v_{n+1} = v_n + h * F_{n+1/2}
// After the position and velocity updates, the mid-point update for density (CFD) or stress (CRM) are equivalent to the
// velocity update above (i.e., an Euler step).
__global__ void MidpointStep_D(Real4* posRadD,
                               Real3* velMasD,
                               Real4* rhoPresMuD,
                               Real3* tauXxYyZzD,
                               Real3* tauXyXzYzD,
                               const Real3* vel_XSPH_D,
                               const Real4* derivVelRhoD,
                               const Real3* derivTauXxYyZzD,
                               const Real3* derivTauXyXzYzD,
                               const uint* freeSurfaceIdD,
                               const int32_t* activityIdentifierSortedD,
                               const uint numActive,
                               Real dT) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    // Only update active SPH particles, not extended active particles
    if (IsBceMarker(rhoPresMuD[index].w) || activityIdentifierSortedD[index] <= 0)
        return;

    // Advance position
    //// TODO: what about XSPH?
    PositionMidpointStep(dT, velMasD[index] + vel_XSPH_D[index], mR3(derivVelRhoD[index]), posRadD[index]);
    
    // Advance velocity
    VelocityEulerStep(dT, mR3(derivVelRhoD[index]), velMasD[index]);

    if (paramsD.elastic_SPH) {
        // Euler step for tau and pressure update
        TauEulerStep(dT, derivTauXxYyZzD[index], derivTauXyXzYzD[index], freeSurfaceIdD[index], tauXxYyZzD[index],
                     tauXyXzYzD[index], rhoPresMuD[index]);
    } else {
        // Euler step for density and pressure update from EOS
        DensityEulerStep(dT, derivVelRhoD[index].w, paramsD.eos_type, rhoPresMuD[index]);
    }
}

template <typename T>
struct check_infinite {
    __host__ __device__ bool operator()(const T& v) { return !IsFinite(v); }
};

void FluidDynamics::EulerStep(std::shared_ptr<SphMarkerDataD> sortedMarkers, Real dT) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 256, numBlocks, numThreads);

    EulerStep_D<<<numBlocks, numThreads>>>(
        mR4CAST(sortedMarkers->posRadD), mR3CAST(sortedMarkers->velMasD), mR4CAST(sortedMarkers->rhoPresMuD),
        mR3CAST(sortedMarkers->tauXxYyZzD), mR3CAST(sortedMarkers->tauXyXzYzD), mR3CAST(m_data_mgr.vel_XSPH_D),
        mR4CAST(m_data_mgr.derivVelRhoD), mR3CAST(m_data_mgr.derivTauXxYyZzD), mR3CAST(m_data_mgr.derivTauXyXzYzD),
        U1CAST(m_data_mgr.freeSurfaceIdD), INT_32CAST(m_data_mgr.activityIdentifierSortedD), numActive, dT);
    cudaCheckError();

    if (m_check_errors) {
        if (thrust::any_of(sortedMarkers->posRadD.begin(), sortedMarkers->posRadD.end(), check_infinite<Real4>()))
            cudaThrowError("A particle position is NaN");
        if (thrust::any_of(sortedMarkers->rhoPresMuD.begin(), sortedMarkers->rhoPresMuD.end(), check_infinite<Real4>()))
            cudaThrowError("A particle density is NaN");
    }
}

void FluidDynamics::MidpointStep(std::shared_ptr<SphMarkerDataD> sortedMarkers, Real dT) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 256, numBlocks, numThreads);

    MidpointStep_D<<<numBlocks, numThreads>>>(
        mR4CAST(sortedMarkers->posRadD), mR3CAST(sortedMarkers->velMasD), mR4CAST(sortedMarkers->rhoPresMuD),
        mR3CAST(sortedMarkers->tauXxYyZzD), mR3CAST(sortedMarkers->tauXyXzYzD), mR3CAST(m_data_mgr.vel_XSPH_D),
        mR4CAST(m_data_mgr.derivVelRhoD), mR3CAST(m_data_mgr.derivTauXxYyZzD), mR3CAST(m_data_mgr.derivTauXyXzYzD),
        U1CAST(m_data_mgr.freeSurfaceIdD), INT_32CAST(m_data_mgr.activityIdentifierSortedD), numActive, dT);
    cudaCheckError();

    if (m_check_errors) {
        if (thrust::any_of(sortedMarkers->posRadD.begin(), sortedMarkers->posRadD.end(), check_infinite<Real4>()))
            cudaThrowError("A particle position is NaN");
        if (thrust::any_of(sortedMarkers->rhoPresMuD.begin(), sortedMarkers->rhoPresMuD.end(), check_infinite<Real4>()))
            cudaThrowError("A particle density is NaN");
    }

}

// -----------------------------------------------------------------------------

// Kernel to copy sorted data back to original order (ISPH)
__global__ void CopySortedToOriginalISPH_D(MarkerGroup group,
                                           const Real4* sortedPosRad,
                                           const Real3* sortedVelMas,
                                           const Real4* sortedRhoPresMu,
                                           const Real3* sortedTauXxYyZz,
                                           const Real3* sortedTauXyXXzYz,
                                           const Real4* derivVelRho,
                                           const Real4* sr_tau_I_mu_i,
                                           const uint numActive,
                                           Real4* posRadOriginal,
                                           Real3* velMasOriginal,
                                           Real4* rhoPresMuOriginal,
                                           Real3* tauXxYyZzOriginal,
                                           Real3* tauXyXzYzOriginal,
                                           Real4* derivVelRhoOriginal,
                                           Real4* sr_tau_I_mu_i_Original,
                                           uint* gridMarkerIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numActive)
        return;

    Real type = sortedRhoPresMu[id].w;
    if (!IsInMarkerGroup(group, type))
        return;

    uint index = gridMarkerIndex[id];
    posRadOriginal[index] = sortedPosRad[id];
    velMasOriginal[index] = sortedVelMas[id];
    rhoPresMuOriginal[index] = sortedRhoPresMu[id];
    derivVelRhoOriginal[index] = derivVelRho[id];
    tauXxYyZzOriginal[index] = sortedTauXxYyZz[id];
    tauXyXzYzOriginal[index] = sortedTauXyXXzYz[id];
    sr_tau_I_mu_i_Original[index] = sr_tau_I_mu_i[id];
}

// Kernel to copy sorted data back to original order (WCSPH)
__global__ void CopySortedToOriginalWCSPH_D(MarkerGroup group,
                                            const Real4* sortedPosRad,
                                            const Real3* sortedVelMas,
                                            const Real4* sortedRhoPresMu,
                                            const Real3* sortedTauXxYyZz,
                                            const Real3* sortedTauXyXXzYz,
                                            const Real4* derivVelRho,
                                            const uint numActive,
                                            Real4* posRadOriginal,
                                            Real3* velMasOriginal,
                                            Real4* rhoPresMuOriginal,
                                            Real3* tauXxYyZzOriginal,
                                            Real3* tauXyXzYzOriginal,
                                            Real4* derivVelRhoOriginal,
                                            uint* gridMarkerIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numActive)
        return;

    Real type = sortedRhoPresMu[id].w;
    if (!IsInMarkerGroup(group, type))
        return;

    uint index = gridMarkerIndex[id];
    posRadOriginal[index] = sortedPosRad[id];
    velMasOriginal[index] = sortedVelMas[id];
    rhoPresMuOriginal[index] = sortedRhoPresMu[id];
    derivVelRhoOriginal[index] = derivVelRho[id];
    tauXxYyZzOriginal[index] = sortedTauXxYyZz[id];
    tauXyXzYzOriginal[index] = sortedTauXyXXzYz[id];
}

void FluidDynamics::CopySortedToOriginal(MarkerGroup group,
                                         std::shared_ptr<SphMarkerDataD> sortedSphMarkersD,
                                         std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 1024, numBlocks, numThreads);
    if (m_data_mgr.paramsH->integration_scheme == IntegrationScheme::IMPLICIT_SPH) {
        CopySortedToOriginalISPH_D<<<numBlocks, numThreads>>>(
            group, mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
            mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
            mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR4CAST(m_data_mgr.derivVelRhoD), mR4CAST(m_data_mgr.sr_tau_I_mu_i),
            numActive, mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR4CAST(sphMarkersD->rhoPresMuD),
            mR3CAST(sphMarkersD->tauXxYyZzD), mR3CAST(sphMarkersD->tauXyXzYzD),
            mR4CAST(m_data_mgr.derivVelRhoOriginalD), mR4CAST(m_data_mgr.sr_tau_I_mu_i_Original),
            U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD));
    } else {
        CopySortedToOriginalWCSPH_D<<<numBlocks, numThreads>>>(
            group, mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
            mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
            mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR4CAST(m_data_mgr.derivVelRhoD), numActive,
            mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR4CAST(sphMarkersD->rhoPresMuD),
            mR3CAST(sphMarkersD->tauXxYyZzD), mR3CAST(sphMarkersD->tauXyXzYzD),
            mR4CAST(m_data_mgr.derivVelRhoOriginalD), U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD));
    }
    cudaCheckError();
}

// -----------------------------------------------------------------------------

// Kernel to apply inlet/outlet BC along x
__global__ void ApplyInletBoundaryX_D(Real4* posRadD, Real3* VelMassD, Real4* rhoPresMuD, const uint numActive) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    // no need to do anything if it is a BCE marker
    if (IsBceMarker(rhoPresMu.w))
        return;

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > paramsD.cMax.x) {
        posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y + paramsD.delta_pressure.x;
        rhoPresMuD[index] = rhoPresMu;
    }
    if (posRad.x < paramsD.cMin.x) {
        posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        VelMassD[index] = mR3(paramsD.V_in.x, 0, 0);
        rhoPresMu.y = rhoPresMu.y - paramsD.delta_pressure.x;
        rhoPresMuD[index] = rhoPresMu;
    }

    if (posRad.x > -paramsD.x_in)
        rhoPresMuD[index].y = 0;

    if (posRad.x < paramsD.x_in)
        VelMassD[index] = mR3(paramsD.V_in.x, 0, 0);
}

// Kernel to apply periodic BC along x
__global__ void ApplyPeriodicBoundaryX_D(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    // no need to do anything if it is a BCE marker
    if (IsBceMarker(rhoPresMu.w))
        return;

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > paramsD.cMax.x) {
        posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        rhoPresMuD[index].y += paramsD.delta_pressure.x;
        return;
    }
    if (posRad.x < paramsD.cMin.x) {
        posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        rhoPresMuD[index].y -= paramsD.delta_pressure.x;
        return;
    }
}

// Kernel to apply periodic BC along y
__global__ void ApplyPeriodicBoundaryY_D(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    // no need to do anything if it is a BCE marker
    if (IsBceMarker(rhoPresMu.w))
        return;

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.y > paramsD.cMax.y) {
        posRad.y -= (paramsD.cMax.y - paramsD.cMin.y);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y + paramsD.delta_pressure.y;
        rhoPresMuD[index] = rhoPresMu;
        return;
    }
    if (posRad.y < paramsD.cMin.y) {
        posRad.y += (paramsD.cMax.y - paramsD.cMin.y);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y - paramsD.delta_pressure.y;
        rhoPresMuD[index] = rhoPresMu;
        return;
    }
}

// Kernel to apply periodic BC along z
__global__ void ApplyPeriodicBoundaryZ_D(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    // no need to do anything if it is a BCE marker
    if (IsBceMarker(rhoPresMu.w))
        return;

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.z > paramsD.cMax.z) {
        posRad.z -= (paramsD.cMax.z - paramsD.cMin.z);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y + paramsD.delta_pressure.z;
        rhoPresMuD[index] = rhoPresMu;
        return;
    }
    if (posRad.z < paramsD.cMin.z) {
        posRad.z += (paramsD.cMax.z - paramsD.cMin.z);
        posRadD[index] = mR4(posRad, h);
        rhoPresMu.y = rhoPresMu.y - paramsD.delta_pressure.z;
        rhoPresMuD[index] = rhoPresMu;
        return;
    }
}

// Apply boundary conditions in x, y, and z directions
void FluidDynamics::ApplyBoundaryConditions(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 1024, numBlocks, numThreads);

    switch (m_data_mgr.paramsH->bc_type.x) {
        case BCType::PERIODIC:
            ApplyPeriodicBoundaryX_D<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                                mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
            cudaCheckError();
            break;
        case BCType::INLET_OUTLET:
            //// TODO - check this and modify as appropriate
            //ApplyInletBoundaryX_D<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD),
            //                                                 mR3CAST(sphMarkersD->velMasD),
            //                                                 mR4CAST(sphMarkersD->rhoPresMuD), numActive);
            //cudaCheckError();
            break;
    }

    switch (m_data_mgr.paramsH->bc_type.y) {
        case BCType::PERIODIC:
            ApplyPeriodicBoundaryY_D<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                                mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
            cudaCheckError();
            break;
    }

    switch (m_data_mgr.paramsH->bc_type.z) {
        case BCType::PERIODIC:
            ApplyPeriodicBoundaryZ_D<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                                mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
            cudaCheckError();
            break;
    }
}

// -----------------------------------------------------------------------------

// Device function to calculate the share of density influence on a given
// particle from all other particle in a given cell
__device__ void collideCellDensityReInit(Real& numerator,
                                         Real& denominator,
                                         int3 gridPos,
                                         uint index,
                                         Real3 posRadA,
                                         Real4* sortedPosRad,
                                         Real3* sortedVelMas,
                                         Real4* sortedRhoPreMu,
                                         uint* cellStart,
                                         uint* cellEnd) {
    uint gridHash = calcGridHash(gridPos);
    uint startIndex = cellStart[gridHash];
    if (startIndex != 0xffffffff) {  // cell is not empty
        // iterate over particles in this cell
        uint endIndex = cellEnd[gridHash];
        for (uint j = startIndex; j < endIndex; j++) {
            Real3 posRadB = mR3(sortedPosRad[j]);
            Real4 rhoPreMuB = sortedRhoPreMu[j];
            Real3 dist3 = Distance(posRadA, posRadB);
            Real d = length(dist3);
            if (d > paramsD.h_multiplier * paramsD.h)
                continue;
            Real w = W3h(paramsD.kernel_type, d, paramsD.ooh);
            numerator += paramsD.markerMass * w;
            denominator += paramsD.markerMass / rhoPreMuB.x * w;
        }
    }
}

// Kernel for updating the density.
// It calculates the density of the particle. It does include the normalization
// close to the boundaries and free surface.
__global__ void ReCalcDensityD_F1(Real4* dummySortedRhoPreMu,
                                  Real4* sortedPosRad,
                                  Real3* sortedVelMas,
                                  Real4* sortedRhoPreMu,
                                  uint* gridMarkerIndex,
                                  uint* cellStart,
                                  uint* cellEnd) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

    // read particle data from sorted arrays
    Real3 posRadA = mR3(sortedPosRad[index]);
    Real4 rhoPreMuA = sortedRhoPreMu[index];

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);

    Real numerator = 0.0;
    Real denominator = 0.0;
    // examine neighbouring cells
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                collideCellDensityReInit(numerator, denominator, neighbourPos, index, posRadA, sortedPosRad,
                                         sortedVelMas, sortedRhoPreMu, cellStart, cellEnd);
            }
        }
    }

    rhoPreMuA.x = numerator;  // denominator;
    //    rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
    dummySortedRhoPreMu[index] = rhoPreMuA;
}

void FluidDynamics::DensityReinitialization() {
    uint numBlocks, numThreads;
    computeGridSize((uint)m_data_mgr.countersH->numAllMarkers, 256, numBlocks, numThreads);

    thrust::device_vector<Real4> dummySortedRhoPreMu(m_data_mgr.countersH->numAllMarkers);
    thrust::fill(dummySortedRhoPreMu.begin(), dummySortedRhoPreMu.end(), mR4(0.0));

    ReCalcDensityD_F1<<<numBlocks, numThreads>>>(
        mR4CAST(dummySortedRhoPreMu), mR4CAST(m_data_mgr.sortedSphMarkers1_D->posRadD),
        mR3CAST(m_data_mgr.sortedSphMarkers1_D->velMasD), mR4CAST(m_data_mgr.sortedSphMarkers1_D->rhoPresMuD),
        U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD), U1CAST(m_data_mgr.markersProximity_D->cellStartD),
        U1CAST(m_data_mgr.markersProximity_D->cellEndD));

    cudaCheckError();
    FsiForce::CopySortedToOriginal_NonInvasive_R4(m_data_mgr.sphMarkers_D->rhoPresMuD, dummySortedRhoPreMu,
                                                  m_data_mgr.markersProximity_D->gridMarkerIndexD);
    dummySortedRhoPreMu.clear();
}

}  // namespace sph
}  // namespace fsi
}  // end namespace chrono
