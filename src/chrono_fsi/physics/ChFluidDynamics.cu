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

#include "chrono_fsi/physics/ChFluidDynamics.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"

using std::cout;
using std::endl;

namespace chrono {
namespace fsi {

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
            if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
                continue;
            numerator += paramsD.markerMass * W3h(d, sortedPosRad[j].w);
            denominator += paramsD.markerMass / rhoPreMuB.x * W3h(d, sortedPosRad[j].w);
        }
    }
}

// -----------------------------------------------------------------------------
// Kernel to apply periodic BC along x
__global__ void ApplyPeriodicBoundaryXKernel(Real4* posRadD, Real4* rhoPresMuD, uint* activityIdentifierD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    uint activity = activityIdentifierD[index];
    if (activity == 0)
        return;  // no need to do anything if it is not an active particle

    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1)
        return;  // no need to do anything if it is a boundary particle

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > paramsD.cMax.x) {
        posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1)
            rhoPresMuD[index].y += paramsD.deltaPress.x;
        return;
    }
    if (posRad.x < paramsD.cMin.x) {
        posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1)
            rhoPresMuD[index].y -= paramsD.deltaPress.x;
        return;
    }
}

// -----------------------------------------------------------------------------
// Kernel to apply inlet/outlet BC along x
__global__ void ApplyInletBoundaryXKernel(Real4* posRadD, Real3* VelMassD, Real4* rhoPresMuD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    if (rhoPresMu.w > 0.0)
        return;  // no need to do anything if it is a boundary particle

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > paramsD.cMax.x) {
        posRad.x -= (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w <= 0.0) {
            rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.x;
            rhoPresMuD[index] = rhoPresMu;
        }
    }
    if (posRad.x < paramsD.cMin.x) {
        posRad.x += (paramsD.cMax.x - paramsD.cMin.x);
        posRadD[index] = mR4(posRad, h);
        VelMassD[index] = mR3(paramsD.V_in.x, 0, 0);
        if (rhoPresMu.w <= -.1) {
            rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.x;
            rhoPresMuD[index] = rhoPresMu;
        }
    }

    if (posRad.x > -paramsD.x_in)
        rhoPresMuD[index].y = 0;

    if (posRad.x < paramsD.x_in)
        VelMassD[index] = mR3(paramsD.V_in.x, 0, 0);
}

// -----------------------------------------------------------------------------
// Kernel to apply periodic BC along y
__global__ void ApplyPeriodicBoundaryYKernel(Real4* posRadD, Real4* rhoPresMuD, uint* activityIdentifierD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    uint activity = activityIdentifierD[index];
    if (activity == 0)
        return;  // no need to do anything if it is not an active particle

    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1)
        return;  // no need to do anything if it is a boundary particle

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.y > paramsD.cMax.y) {
        posRad.y -= (paramsD.cMax.y - paramsD.cMin.y);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.y;
            rhoPresMuD[index] = rhoPresMu;
        }
        return;
    }
    if (posRad.y < paramsD.cMin.y) {
        posRad.y += (paramsD.cMax.y - paramsD.cMin.y);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.y;
            rhoPresMuD[index] = rhoPresMu;
        }
        return;
    }
}

// -----------------------------------------------------------------------------
// Kernel to apply periodic BC along z
__global__ void ApplyPeriodicBoundaryZKernel(Real4* posRadD, Real4* rhoPresMuD, uint* activityIdentifierD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    uint activity = activityIdentifierD[index];
    if (activity == 0)
        return;  // no need to do anything if it is not an active particle

    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1)
        return;  // no need to do anything if it is a boundary particle

    Real3 posRad = mR3(posRadD[index]);
    Real h = posRadD[index].w;

    if (posRad.z > paramsD.cMax.z) {
        posRad.z -= (paramsD.cMax.z - paramsD.cMin.z);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMu.y = rhoPresMu.y + paramsD.deltaPress.z;
            rhoPresMuD[index] = rhoPresMu;
        }
        return;
    }
    if (posRad.z < paramsD.cMin.z) {
        posRad.z += (paramsD.cMax.z - paramsD.cMin.z);
        posRadD[index] = mR4(posRad, h);
        if (rhoPresMu.w < -.1) {
            rhoPresMu.y = rhoPresMu.y - paramsD.deltaPress.z;
            rhoPresMuD[index] = rhoPresMu;
        }
        return;
    }
}

// -----------------------------------------------------------------------------
// Kernel to keep particle inside the simulation domain
__global__ void ApplyOutOfBoundaryKernel(Real4* posRadD, Real4* rhoPresMuD, Real3* velMasD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1)
        return;  // no need to do anything if it is a boundary particle

    Real3 posRad = mR3(posRadD[index]);
    Real3 vel = mR3(velMasD[index]);
    Real h = posRadD[index].w;

    if (posRad.x > 0.5 * paramsD.boxDimX)
        posRad.x = 0.5 * paramsD.boxDimX;
    if (posRad.x < -0.5 * paramsD.boxDimX)
        posRad.x = -0.5 * paramsD.boxDimX;
    if (posRad.y > 0.5 * paramsD.boxDimY)
        posRad.y = 0.5 * paramsD.boxDimY;
    if (posRad.y < -0.5 * paramsD.boxDimY)
        posRad.y = -0.5 * paramsD.boxDimY;
    if (posRad.z > 1.0 * paramsD.boxDimZ)
        posRad.z = 1.0 * paramsD.boxDimZ;
    if (posRad.z < -0.0 * paramsD.boxDimZ)
        posRad.z = -0.0 * paramsD.boxDimZ;

    posRadD[index] = mR4(posRad, h);
    velMasD[index] = mR3(vel);
    return;
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
                             uint* activityIdentifierD,
                             uint* freeSurfaceIdD,
                             int2 updatePortion,
                             Real dT,
                             volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= updatePortion.y - updatePortion.x) {
        return;
    }

    uint activity = activityIdentifierD[index];
    if (activity == 0)
        return;

    Real4 derivVelRho = derivVelRhoD[index];
    Real4 rhoPresMu = rhoPresMuD[index];
    Real h = posRadD[index].w;
    Real p_tr, p_n;
    if (rhoPresMu.w > -0.5) {
        return;
    }

    // This is only implemented for granular material
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
        p_n = -1.0 / 3.0 * (tauXxYyZz.x + tauXxYyZz.y + tauXxYyZz.z);
        tauXxYyZz.x += p_n;
        tauXxYyZz.y += p_n;
        tauXxYyZz.z += p_n;
        p_tr = -1.0 / 3.0 * (updatedTauXxYyZz.x + updatedTauXxYyZz.y + updatedTauXxYyZz.z);
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

        if (paramsD.output_length == 2) {
            Real tau_tr = square(updatedTauXxYyZz.x) + square(updatedTauXxYyZz.y) + square(updatedTauXxYyZz.z) +
                          2.0 * (square(updatedTauXyXzYz.x) + square(updatedTauXyXzYz.y) + square(updatedTauXyXzYz.z));
            tau_tr = sqrt(0.5 * tau_tr);
            sr_tau_I_mu_iD[index].y = tau_tr;
        }

        tauXxYyZzD[index] = updatedTauXxYyZz - mR3(p_tr);
        tauXyXzYzD[index] = updatedTauXyXzYz;
    }

    //-------------
    // ** position
    //-------------
    Real3 vel_XSPH = velMasD[index] + vel_XSPH_D[index];  // paramsD.EPS_XSPH *
    Real3 posRad = mR3(posRadD[index]);
    Real3 updatedPositon = posRad + vel_XSPH * dT;
    if (!(isfinite(updatedPositon.x) && isfinite(updatedPositon.y) && isfinite(updatedPositon.z))) {
        printf("Error! particle position is NAN: thrown from ChFluidDynamics.cu, UpdateFluidDKernel !\n");
        *isErrorD = true;
        return;
    }
    posRadD[index] = mR4(updatedPositon, h);

    //-------------
    // ** velocity
    //-------------
    // Note that the velocity update should not use the XSPH contribution
    // It adds dissipation to the solution, and provides numerical damping
    Real3 velMas = velMasD[index] + 0.0 * vel_XSPH_D[index];  // paramsD.EPS_XSPH * vel_XSPH_D[index]
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
        rhoPresMu.y = Eos(rho2, rhoPresMu.w);
        rhoPresMu.x = rho2;
    }
    if (!(isfinite(rhoPresMu.x) && isfinite(rhoPresMu.y) && isfinite(rhoPresMu.z) && isfinite(rhoPresMu.w))) {
        printf("Error! particle rho pressure is NAN: thrown from ChFluidDynamics.cu, UpdateFluidDKernel !\n");
        *isErrorD = true;
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
    if (index >= numObjectsD.numAllMarkers)
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
                                uint* activityIdentifierD,
                                uint* extendedActivityIdD,
                                const int2 updatePortion,
                                const Real Time,
                                volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= updatePortion.y - updatePortion.x) {
        return;
    }

    // Set the particle as an active particle
    activityIdentifierD[index] = 1;
    extendedActivityIdD[index] = 1;

    // If during the settling phase, all particles are active
    if (Time < paramsD.settlingTime)
        return;

    size_t numRigidBodies = numObjectsD.numRigidBodies;
    size_t numFlexNodes1D = numObjectsD.numFlexNodes1D;
    size_t numFlexNodes2D = numObjectsD.numFlexNodes2D;
    size_t numTotal = numRigidBodies + numFlexNodes1D + numFlexNodes2D;

    // Check the activity of this particle
    uint isNotActive = 0;
    uint isNotExtended = 0;

    Real3 Acdomain = paramsD.bodyActiveDomain;
    Real3 ExAcdomain = paramsD.bodyActiveDomain + mR3(2 * RESOLUTION_LENGTH_MULT * paramsD.HSML);

    Real3 posRadA = mR3(posRadD[index]);

    for (uint num = 0; num < numRigidBodies; num++) {
        Real3 detPos = posRadA - pos_bodies_D[num];
        if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || abs(detPos.z) > Acdomain.z)
            isNotActive = isNotActive + 1;
        if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || abs(detPos.z) > ExAcdomain.z)
            isNotExtended = isNotExtended + 1;
    }

    for (uint num = 0; num < numFlexNodes1D; num++) {
        Real3 detPos = posRadA - pos_nodes1D_D[num];
        if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || abs(detPos.z) > Acdomain.z)
            isNotActive = isNotActive + 1;
        if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || abs(detPos.z) > ExAcdomain.z)
            isNotExtended = isNotExtended + 1;
    }

    for (uint num = 0; num < numFlexNodes2D; num++) {
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

    return;
}


// -----------------------------------------------------------------------------
__global__ void CopySortedToOriginal_D(const Real4* sortedPosRad,
                                       const Real3* sortedVelMas,
                                       const Real4* sortedRhoPresMu,
                                       const Real3* sortedTauXxYyZz,
                                       const Real3* sortedTauXyXXzYz,
                                       const Real4* derivVelRho,
                                       const Real4* sr_tau_I_mu_i,
                                       Real4* posRadOriginal,
                                       Real3* velMasOriginal,
                                       Real4* rhoPresMuOriginal,
                                       Real3* tauXxYyZzOriginal,
                                       Real3* tauXyXzYzOriginal,
                                       Real4* derivVelRhoOriginal,
                                       Real4* sr_tau_I_mu_i_Original,
                                       uint* gridMarkerIndex,
                                       uint* activityIdentifierD,
                                       volatile bool* isErrorD) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numObjectsD.numAllMarkers)
        return;

    // Check the activity of this particle
    uint activity = activityIdentifierD[id];
    if (activity == 0)
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
ChFluidDynamics::ChFluidDynamics(std::shared_ptr<ChBce> otherBceWorker,
                                 ChSystemFsi_impl& otherFsiSystem,
                                 std::shared_ptr<SimParams> otherParamsH,
                                 std::shared_ptr<ChCounters> otherNumObjects,
                                 bool verb)
    : ChFsiBase(otherParamsH, otherNumObjects), fsiSystem(otherFsiSystem), verbose(verb) {
    switch (paramsH->sph_method) {
        default:
            cout << "Selected integrator type not implemented, reverting to WCSPH" << endl;

        case SPHMethod::WCSPH:
            forceSystem = chrono_types::make_shared<ChFsiForceExplicitSPH>(
                otherBceWorker, fsiSystem.sortedSphMarkers2_D, fsiSystem.markersProximity_D, fsiSystem.fsiData, paramsH,
                numObjectsH, verb);
            if (verbose) {
                cout << "====== Created a WCSPH framework" << endl;
            }

            break;

        case SPHMethod::I2SPH:
            forceSystem = chrono_types::make_shared<ChFsiForceI2SPH>(otherBceWorker, fsiSystem.sortedSphMarkers2_D,
                                                                     fsiSystem.markersProximity_D, fsiSystem.fsiData,
                                                                     paramsH, numObjectsH, verb);
            if (verbose) {
                cout << "====== Created an I2SPH framework" << endl;
            }

            break;
    }
}

// -----------------------------------------------------------------------------
ChFluidDynamics::~ChFluidDynamics() {}

// -----------------------------------------------------------------------------
void ChFluidDynamics::Initialize() {
    forceSystem->Initialize();
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(ChCounters));
    cudaMemcpyFromSymbol(paramsH.get(), paramsD, sizeof(SimParams));
}

// -----------------------------------------------------------------------------
void ChFluidDynamics::SortParticles() {
    forceSystem->fsiCollisionSystem->ArrangeData(fsiSystem.sphMarkers_D);
}

// -----------------------------------------------------------------------------
void ChFluidDynamics::IntegrateSPH(std::shared_ptr<SphMarkerDataD> sortedSphMarkers2_D,
                                   std::shared_ptr<SphMarkerDataD> sortedSphMarkers1_D,
                                   std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                                   std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,
                                   std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D,
                                   Real dT,
                                   Real time,
                                   bool firstHalfStep) {
    if (paramsH->sph_method == SPHMethod::WCSPH) {
        // Explicit SPH
        UpdateActivity(sortedSphMarkers1_D, sortedSphMarkers2_D, fsiBodyState_D, fsiMesh1DState_D, fsiMesh2DState_D,
                       time);
        forceSystem->ForceSPH(sortedSphMarkers2_D, fsiBodyState_D, fsiMesh1DState_D, fsiMesh2DState_D, time,
                              firstHalfStep);
        UpdateFluid(sortedSphMarkers1_D, dT);
    } else {
        // Implicit SPH
        forceSystem->ForceSPH(sortedSphMarkers1_D, fsiBodyState_D, fsiMesh1DState_D, fsiMesh2DState_D, time,
                              firstHalfStep);
    }
    ApplyBoundarySPH_Markers(sortedSphMarkers2_D);
}

// -----------------------------------------------------------------------------
void ChFluidDynamics::UpdateActivity(std::shared_ptr<SphMarkerDataD> sortedSphMarkers1_D,
                                     std::shared_ptr<SphMarkerDataD> sortedSphMarkers2_D,
                                     std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                                     std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,
                                     std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D,
                                     Real time) {
    // Update portion of the SPH particles (should be all particles here)
    int2 updatePortion = mI2(0, (int)numObjectsH->numAllMarkers);

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    //------------------------
    uint numBlocks, numThreads;
    computeGridSize(updatePortion.y - updatePortion.x, 256, numBlocks, numThreads);
    UpdateActivityD<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkers2_D->posRadD), mR3CAST(sortedSphMarkers1_D->velMasD), mR3CAST(fsiBodyState_D->pos),
        mR3CAST(fsiMesh1DState_D->pos_fsi_fea_D), mR3CAST(fsiMesh2DState_D->pos_fsi_fea_D),
        U1CAST(fsiSystem.fsiData->activityIdentifierD), U1CAST(fsiSystem.fsiData->extendedActivityIdD), updatePortion,
        time, isErrorD);
    cudaDeviceSynchronize();
    cudaCheckError();
    //------------------------

    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true)
        throw std::runtime_error("Error! program crashed in UpdateActivityD!\n");
    cudaFree(isErrorD);
    free(isErrorH);
}

// -----------------------------------------------------------------------------
void ChFluidDynamics::UpdateFluid(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, Real dT) {
    // Update portion of the SPH particles (should be fluid particles only here)
    int2 updatePortion = mI2(0, (int)numObjectsH->numAllMarkers);

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    //------------------------
    uint numBlocks, numThreads;
    computeGridSize(updatePortion.y - updatePortion.x, 256, numBlocks, numThreads);
    UpdateFluidD<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
        mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR3CAST(fsiSystem.fsiData->vel_XSPH_D),
        mR4CAST(fsiSystem.fsiData->derivVelRhoD), mR3CAST(fsiSystem.fsiData->derivTauXxYyZzD),
        mR3CAST(fsiSystem.fsiData->derivTauXyXzYzD), mR4CAST(fsiSystem.fsiData->sr_tau_I_mu_i),
        U1CAST(fsiSystem.fsiData->activityIdentifierD), U1CAST(fsiSystem.fsiData->freeSurfaceIdD), updatePortion, dT,
        isErrorD);
    cudaDeviceSynchronize();
    cudaCheckError();
    //------------------------
    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true)
        throw std::runtime_error("Error! program crashed in UpdateFluidD!\n");
    cudaFree(isErrorD);
    free(isErrorH);
}

// -----------------------------------------------------------------------------
void ChFluidDynamics::CopySortedToOriginal(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD,
                                           std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    int2 updatePortion = mI2(0, (int)numObjectsH->numAllMarkers);

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    //------------------------
    uint numBlocks, numThreads;
    computeGridSize(updatePortion.y - updatePortion.x, 256, numBlocks, numThreads);
    CopySortedToOriginal_D<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD),
        mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
        mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR4CAST(fsiSystem.fsiData->derivVelRhoD),
        mR4CAST(fsiSystem.fsiData->sr_tau_I_mu_i), mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD),
        mR4CAST(sphMarkersD->rhoPresMuD), mR3CAST(sphMarkersD->tauXxYyZzD), mR3CAST(sphMarkersD->tauXyXzYzD),
        mR4CAST(fsiSystem.fsiData->derivVelRhoOriginalD), mR4CAST(fsiSystem.fsiData->sr_tau_I_mu_i_Original),
        U1CAST(fsiSystem.markersProximity_D->gridMarkerIndexD), U1CAST(fsiSystem.fsiData->activityIdentifierD),
        isErrorD);
    cudaDeviceSynchronize();
    cudaCheckError();

    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true)
        throw std::runtime_error("Error! program crashed in CopySortedToOriginal!\n");
    cudaFree(isErrorD);
    free(isErrorH);
}

// -----------------------------------------------------------------------------
// Apply periodic boundary conditions in x, y, and z directions
void ChFluidDynamics::ApplyBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD) {
    uint numBlocks, numThreads;

    computeGridSize((int)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);
    ApplyPeriodicBoundaryXKernel<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                            mR4CAST(sortedSphMarkersD->rhoPresMuD),
                                                            U1CAST(fsiSystem.fsiData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();

    ApplyPeriodicBoundaryYKernel<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                            mR4CAST(sortedSphMarkersD->rhoPresMuD),
                                                            U1CAST(fsiSystem.fsiData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();

    ApplyPeriodicBoundaryZKernel<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->posRadD),
                                                            mR4CAST(sortedSphMarkersD->rhoPresMuD),
                                                            U1CAST(fsiSystem.fsiData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();

    // ApplyOutOfBoundaryKernel<<<numBlocks, numThreads>>>
    //     (mR4CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD), mR3CAST(sphMarkersD->velMasD));
    // cudaDeviceSynchronize();
    // cudaCheckError();
}

// -----------------------------------------------------------------------------
// TODO (Huzaifa): Can this function be deprecated? Does not seem to be used anywhere
// Apply periodic boundary conditions in y, and z.
// The inlet/outlet BC is applied in the x direction.
// This functions needs to be tested.
void ChFluidDynamics::ApplyModifiedBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    uint numBlocks, numThreads;
    computeGridSize((int)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);
    ApplyInletBoundaryXKernel<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD),
                                                         mR4CAST(sphMarkersD->rhoPresMuD));
    cudaDeviceSynchronize();
    cudaCheckError();

    // these are useful anyway for out of bound particles
    ApplyPeriodicBoundaryYKernel<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD),
                                                            mR4CAST(sphMarkersD->rhoPresMuD),
                                                            U1CAST(fsiSystem.fsiData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();

    ApplyPeriodicBoundaryZKernel<<<numBlocks, numThreads>>>(mR4CAST(sphMarkersD->posRadD),
                                                            mR4CAST(sphMarkersD->rhoPresMuD),
                                                            U1CAST(fsiSystem.fsiData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();
}

// -----------------------------------------------------------------------------
void ChFluidDynamics::DensityReinitialization() {
    uint numBlocks, numThreads;
    computeGridSize((int)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);

    thrust::device_vector<Real4> dummySortedRhoPreMu(numObjectsH->numAllMarkers);
    thrust::fill(dummySortedRhoPreMu.begin(), dummySortedRhoPreMu.end(), mR4(0.0));

    ReCalcDensityD_F1<<<numBlocks, numThreads>>>(
        mR4CAST(dummySortedRhoPreMu), mR4CAST(fsiSystem.sortedSphMarkers1_D->posRadD),
        mR3CAST(fsiSystem.sortedSphMarkers1_D->velMasD), mR4CAST(fsiSystem.sortedSphMarkers1_D->rhoPresMuD),
        U1CAST(fsiSystem.markersProximity_D->gridMarkerIndexD), U1CAST(fsiSystem.markersProximity_D->cellStartD),
        U1CAST(fsiSystem.markersProximity_D->cellEndD));

    cudaDeviceSynchronize();
    cudaCheckError();
    ChFsiForce::CopySortedToOriginal_NonInvasive_R4(fsiSystem.sphMarkers_D->rhoPresMuD, dummySortedRhoPreMu,
                                                    fsiSystem.markersProximity_D->gridMarkerIndexD);
    dummySortedRhoPreMu.clear();
}

}  // namespace fsi
}  // end namespace chrono
