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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================
//
// Class for performing time integration in fluid system.
// =============================================================================

#include "chrono/utils/ChConstants.h"
#include "chrono_fsi/sph/physics/FluidDynamics.cuh"
#include "chrono_fsi/sph/physics/SphGeneral.cuh"

using std::cout;
using std::endl;

namespace chrono {
namespace fsi {
namespace sph {

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

// -----------------------------------------------------------------------------
// Kernel to apply periodic BC along x
__global__ void ApplyPeriodicBoundaryXKernel(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
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

// -----------------------------------------------------------------------------
// Kernel to apply inlet/outlet BC along x
__global__ void ApplyInletBoundaryXKernel(Real4* posRadD, Real3* VelMassD, Real4* rhoPresMuD, const uint numActive) {
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

// -----------------------------------------------------------------------------
// Kernel to apply periodic BC along y
__global__ void ApplyPeriodicBoundaryYKernel(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
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

// -----------------------------------------------------------------------------
// Kernel to apply periodic BC along z
__global__ void ApplyPeriodicBoundaryZKernel(Real4* posRadD, Real4* rhoPresMuD, const uint numActive) {
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

// -----------------------------------------------------------------------------
// Kernel to update the fluid properities. It updates the stress tensor,
// density, velocity and position relying on explicit Euler scheme.
// Pressure is obtained from the density and an Equation of State.
__global__ void UpdateFluidD(Real4* posRadD,
                             Real3* velMasD,
                             Real4* rhoPresMuD,
                             Real3* tauXxYyZzD,
                             Real3* tauXyXzYzD,
                             Real3* vel_XSPH_D,
                             Real4* derivVelRhoD,
                             Real3* derivTauXxYyZzD,
                             Real3* derivTauXyXzYzD,
                             Real4* sr_tau_I_mu_iD,
                             uint* freeSurfaceIdD,
                             int32_t* activityIdentifierSortedD,
                             const uint numActive,
                             Real dT,
                             volatile bool* error_flag) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numActive) {
        return;
    }

    // Only update active particles not extended active particles
    if (activityIdentifierSortedD[index] <= 0) {
        return;
    }

    Real4 rhoPresMu = rhoPresMuD[index];
    if (IsBceMarker(rhoPresMu.w)) {
        return;
    }

    Real4 derivVelRho = derivVelRhoD[index];
    Real h = posRadD[index].w;

    // This is only implemented for granular material
    Real p_tr, p_n;

    if (paramsD.elastic_SPH) {
        //--------------------------------
        // ** total stress tau
        //--------------------------------
        Real3 tauXxYyZz = tauXxYyZzD[index];
        Real3 tauXyXzYz = tauXyXzYzD[index];
        Real3 derivTauXxYyZz = derivTauXxYyZzD[index];
        Real3 derivTauXyXzYz = derivTauXyXzYzD[index];
        Real3 updatedTauXxYyZz = tauXxYyZz + mR3(derivTauXxYyZz) * dT;
        Real3 updatedTauXyXzYz = tauXyXzYz + mR3(derivTauXyXzYz) * dT;

        // check if there is a plastic flow
        p_n = -CH_1_3 * (tauXxYyZz.x + tauXxYyZz.y + tauXxYyZz.z);
        tauXxYyZz.x += p_n;
        tauXxYyZz.y += p_n;
        tauXxYyZz.z += p_n;
        p_tr = -CH_1_3 * (updatedTauXxYyZz.x + updatedTauXxYyZz.y + updatedTauXxYyZz.z);
        updatedTauXxYyZz.x += p_tr;
        updatedTauXxYyZz.y += p_tr;
        updatedTauXxYyZz.z += p_tr;

        Real tau_tr = square(updatedTauXxYyZz.x) + square(updatedTauXxYyZz.y) + square(updatedTauXxYyZz.z) +
                      2.0 * square(updatedTauXyXzYz.x) + 2.0 * square(updatedTauXyXzYz.y) +
                      2.0 * square(updatedTauXyXzYz.z);
        Real tau_n = square(tauXxYyZz.x) + square(tauXxYyZz.y) + square(tauXxYyZz.z) + 2.0 * square(tauXyXzYz.x) +
                     2.0 * square(tauXyXzYz.y) + 2.0 * square(tauXyXzYz.z);
        tau_tr = sqrt(0.5 * tau_tr);
        tau_n = sqrt(0.5 * tau_n);
        Real Chi = abs(tau_tr - tau_n) * paramsD.INV_G_shear / dT;
        // should use the positive magnitude according to "A
        // constitutive law for dense granular flows" Nature 2006
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
        if (p_tr > p_cri) {
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
                updatedTauXxYyZz = updatedTauXxYyZz * coeff;
                updatedTauXyXzYz = updatedTauXyXzYz * coeff;
            }
        }
        // Set stress to zero if the pressure is smaller than the threshold
        if (p_tr < p_cri) {
            updatedTauXxYyZz = mR3(0.0);
            updatedTauXyXzYz = mR3(0.0);
            p_tr = 0.0;
            // Real coeff = abs(p_cri / (p_tr + 1e-9));
            // if (p_tr < 2.0 * p_cri){
            //     coeff = 0.0;
            // } else {
            //     coeff = abs(1.0 - (p_tr - p_cri) / p_cri);
            // }
            // updatedTauXxYyZz = updatedTauXxYyZz * coeff;
            // updatedTauXyXzYz = updatedTauXyXzYz * coeff;
            // p_tr = p_cri * coeff;
        }
        // Set stress to zero if the particle is close to free surface
        if (freeSurfaceIdD[index] == 1) {
            updatedTauXxYyZz = mR3(0.0);
            updatedTauXyXzYz = mR3(0.0);
            p_tr = 0.0;
        }

        tauXxYyZzD[index] = updatedTauXxYyZz - mR3(p_tr);
        tauXyXzYzD[index] = updatedTauXyXzYz;
    }

    //-------------
    // ** position
    //-------------
    Real3 vel_XSPH = velMasD[index] + vel_XSPH_D[index];
    Real3 posRad = mR3(posRadD[index]);
    Real3 updatedPositon = posRad + vel_XSPH * dT;
    if (!IsFinite(updatedPositon)) {
        printf("Error! particle position is NAN: thrown from FluidDynamics.cu, UpdateFluidDKernel !\n");
        *error_flag = true;
        return;
    }
    posRadD[index] = mR4(updatedPositon, h);

    //-------------
    // ** velocity
    //-------------
    // Note that the velocity update should not use the XSPH contribution
    // It adds dissipation to the solution, and provides numerical damping
    Real3 velMas = velMasD[index];
    Real3 updatedVelocity = velMas + mR3(derivVelRho) * dT;
    velMasD[index] = updatedVelocity;

    //-------------
    // ** density
    //-------------
    if (paramsD.elastic_SPH) {  // This is only implemented for granular material
        rhoPresMu.y = p_tr;
        rhoPresMu.x = paramsD.rho0;
    } else {
        Real rho2 = rhoPresMu.x + derivVelRho.w * dT;
        rhoPresMu.y = Eos(rho2, paramsD.eos_type);
        rhoPresMu.x = rho2;
    }
    if (!IsFinite(rhoPresMu)) {
        printf("Error! particle rho pressure is NAN: thrown from FluidDynamics.cu, UpdateFluidDKernel !\n");
        *error_flag = true;
        return;
    }
    rhoPresMuD[index] = rhoPresMu;

    // Important note: the derivVelRhoD that is calculated by the ChForceExplicitSPH is the negative of actual time
    // derivative. That is important to keep the derivVelRhoD to be the force/mass for fsi forces.
    // calculate the force that is f=m dv/dt
    // derivVelRhoD[index] *= paramsD.markerMass;
}

// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Kernel for updating the activity of all particles.
__global__ void UpdateActivityD(const Real4* posRadD,
                                Real3* velMasD,
                                const Real3* pos_bodies_D,
                                const Real3* pos_nodes1D_D,
                                const Real3* pos_nodes2D_D,
                                int32_t* activityIdentifierD,
                                int32_t* extendedActivityIdD,
                                const Real4* rhoPreMuD,
                                volatile bool* error_flag) {
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

// -----------------------------------------------------------------------------
__global__ void CopySortedToOriginal_D(MarkerGroup group,
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
                                       uint* gridMarkerIndex,
                                       volatile bool* error_flag) {
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

// -----------------------------------------------------------------------------
// CLASS FOR FLUID DYNAMICS SYSTEM
// -----------------------------------------------------------------------------
FluidDynamics::FluidDynamics(FsiDataManager& data_mgr, BceManager& bce_mgr, bool verbose)
    : m_data_mgr(data_mgr), m_verbose(verbose) {
    switch (m_data_mgr.paramsH->sph_method) {
        default:
        case SPHMethod::WCSPH:
            forceSystem = chrono_types::make_shared<FsiForceWCSPH>(data_mgr, bce_mgr, verbose);
            break;
        case SPHMethod::I2SPH:
            forceSystem = chrono_types::make_shared<FsiForceISPH>(data_mgr, bce_mgr, verbose);
            break;
    }
}

FluidDynamics::~FluidDynamics() {}

// -----------------------------------------------------------------------------

void FluidDynamics::Initialize() {
    forceSystem->Initialize();
    cudaMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(ChFsiParamsSPH));
    cudaMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));
    cudaMemcpyFromSymbol(m_data_mgr.paramsH.get(), paramsD, sizeof(ChFsiParamsSPH));
}

// -----------------------------------------------------------------------------
void FluidDynamics::SortParticles() {
    forceSystem->fsiCollisionSystem->ArrangeData(m_data_mgr.sphMarkers_D);
}

// -----------------------------------------------------------------------------
void FluidDynamics::IntegrateSPH(std::shared_ptr<SphMarkerDataD> sortedSphMarkers2_D,
                                   std::shared_ptr<SphMarkerDataD> sortedSphMarkers1_D,
                                   Real dT,
                                   Real time,
                                   bool firstHalfStep) {
    if (m_data_mgr.paramsH->sph_method == SPHMethod::WCSPH) {
        forceSystem->ForceSPH(sortedSphMarkers2_D, time, firstHalfStep);
        UpdateFluid(sortedSphMarkers1_D, dT);
    } else {
        forceSystem->ForceSPH(sortedSphMarkers1_D, time, firstHalfStep);
    }

    ApplyBoundarySPH_Markers(sortedSphMarkers2_D);
}

// -----------------------------------------------------------------------------
void FluidDynamics::UpdateActivity(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    bool* error_flagD;
    cudaMallocErrorFlag(error_flagD);
    cudaResetErrorFlag(error_flagD);

    uint numBlocks, numThreads;
    computeGridSize((uint)m_data_mgr.countersH->numAllMarkers, 1024, numBlocks, numThreads);

    UpdateActivityD<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR3CAST(m_data_mgr.fsiBodyState_D->pos),
        mR3CAST(m_data_mgr.fsiMesh1DState_D->pos_fsi_fea_D), mR3CAST(m_data_mgr.fsiMesh2DState_D->pos_fsi_fea_D),
        INT_32CAST(m_data_mgr.activityIdentifierOriginalD), INT_32CAST(m_data_mgr.extendedActivityIdentifierOriginalD),
        mR4CAST(sphMarkersD->rhoPresMuD), error_flagD);
    cudaCheckErrorFlag(error_flagD, "UpdateActivityD");

    cudaFreeErrorFlag(error_flagD);
}

// -----------------------------------------------------------------------------
void FluidDynamics::UpdateFluid(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, Real dT) {
    bool* error_flagD;
    cudaMallocErrorFlag(error_flagD);
    cudaResetErrorFlag(error_flagD);

    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 256, numBlocks, numThreads);

    UpdateFluidD<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
        mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR3CAST(m_data_mgr.vel_XSPH_D), mR4CAST(m_data_mgr.derivVelRhoD),
        mR3CAST(m_data_mgr.derivTauXxYyZzD), mR3CAST(m_data_mgr.derivTauXyXzYzD), mR4CAST(m_data_mgr.sr_tau_I_mu_i),
        U1CAST(m_data_mgr.freeSurfaceIdD), INT_32CAST(m_data_mgr.activityIdentifierSortedD), numActive, dT,
        error_flagD);
    cudaCheckErrorFlag(error_flagD, "UpdateFluidD");

    cudaFreeErrorFlag(error_flagD);
}

// -----------------------------------------------------------------------------
void FluidDynamics::CopySortedToOriginal(MarkerGroup group,
                                           std::shared_ptr<SphMarkerDataD> sortedSphMarkersD,
                                           std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    bool* error_flagD;
    cudaMallocErrorFlag(error_flagD);
    cudaResetErrorFlag(error_flagD);

    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 1024, numBlocks, numThreads);

    CopySortedToOriginal_D<<<numBlocks, numThreads>>>(
        group, mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
        mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR4CAST(m_data_mgr.derivVelRhoD), mR4CAST(m_data_mgr.sr_tau_I_mu_i),
        numActive, mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), mR4CAST(sphMarkersD->rhoPresMuD),
        mR3CAST(sphMarkersD->tauXxYyZzD), mR3CAST(sphMarkersD->tauXyXzYzD), mR4CAST(m_data_mgr.derivVelRhoOriginalD),
        mR4CAST(m_data_mgr.sr_tau_I_mu_i_Original), U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD),
        error_flagD);
    cudaCheckErrorFlag(error_flagD, "CopySortedToOriginal_D");

    cudaFreeErrorFlag(error_flagD);
}

// -----------------------------------------------------------------------------
// Apply periodic boundary conditions in x, y, and z directions
void FluidDynamics::ApplyBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 1024, numBlocks, numThreads);

    if (m_data_mgr.paramsH->x_periodic) {
        ApplyPeriodicBoundaryXKernel<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                                mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
        cudaDeviceSynchronize();
        cudaCheckError();
    }
    if (m_data_mgr.paramsH->y_periodic) {
        ApplyPeriodicBoundaryYKernel<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                                mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
        cudaDeviceSynchronize();
        cudaCheckError();
    }
    if (m_data_mgr.paramsH->z_periodic) {
        ApplyPeriodicBoundaryZKernel<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                                mR4CAST(sortedSphMarkersD->rhoPresMuD), numActive);
        cudaDeviceSynchronize();
        cudaCheckError();
    }
}

// -----------------------------------------------------------------------------
// Apply periodic boundary conditions in y, and z.
// The inlet/outlet BC is applied in the x direction.
// This functions needs to be tested.
void FluidDynamics::ApplyModifiedBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    uint numActive = (uint)m_data_mgr.countersH->numExtendedParticles;
    uint numBlocks, numThreads;
    computeGridSize(numActive, 256, numBlocks, numThreads);
    ApplyInletBoundaryXKernel<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD),
                                                         mR4CAST(sphMarkersD->rhoPresMuD), numActive);
    cudaDeviceSynchronize();
    cudaCheckError();

    // these are useful anyway for out of bound particles
    ApplyPeriodicBoundaryYKernel<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD),
                                                            mR4CAST(sphMarkersD->rhoPresMuD), numActive);
    cudaDeviceSynchronize();
    cudaCheckError();

    ApplyPeriodicBoundaryZKernel<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD),
                                                            mR4CAST(sphMarkersD->rhoPresMuD), numActive);
    cudaDeviceSynchronize();
    cudaCheckError();
}

// -----------------------------------------------------------------------------
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

    cudaDeviceSynchronize();
    cudaCheckError();
    FsiForce::CopySortedToOriginal_NonInvasive_R4(m_data_mgr.sphMarkers_D->rhoPresMuD, dummySortedRhoPreMu,
                                                    m_data_mgr.markersProximity_D->gridMarkerIndexD);
    dummySortedRhoPreMu.clear();
}

}  // namespace sph
}  // namespace fsi
}  // end namespace chrono
