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
__global__ void ApplyPeriodicBoundaryXKernel(Real4* posRadD, 
                                             Real4* rhoPresMuD, 
                                             uint* activityIdentifierD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    uint activity = activityIdentifierD[index];
    if (activity == 0)
        return; // no need to do anything if it is not an active particle

    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1)
        return; // no need to do anything if it is a boundary particle

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
__global__ void ApplyInletBoundaryXKernel(Real4* posRadD, 
                                          Real3* VelMassD, 
                                          Real4* rhoPresMuD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    if (rhoPresMu.w > 0.0)
        return; // no need to do anything if it is a boundary particle
 
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
__global__ void ApplyPeriodicBoundaryYKernel(Real4* posRadD, 
                                             Real4* rhoPresMuD, 
                                             uint* activityIdentifierD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    uint activity = activityIdentifierD[index];
    if (activity == 0)
        return; // no need to do anything if it is not an active particle

    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1)
        return; // no need to do anything if it is a boundary particle

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
__global__ void ApplyPeriodicBoundaryZKernel(Real4* posRadD, 
                                             Real4* rhoPresMuD, 
                                             uint* activityIdentifierD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    uint activity = activityIdentifierD[index];
    if (activity == 0)
        return; // no need to do anything if it is not an active particle

    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1)
        return; // no need to do anything if it is a boundary particle

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
__global__ void ApplyOutOfBoundaryKernel(Real4* posRadD, 
                                         Real4* rhoPresMuD, 
                                         Real3* velMasD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    Real4 rhoPresMu = rhoPresMuD[index];
    if (fabs(rhoPresMu.w) < .1)
        return; // no need to do anything if it is a boundary particle

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
    index += updatePortion.x;
    if (index >= updatePortion.y)
        return;

    uint activity = activityIdentifierD[index];
    if (activity == 0)
        return;

    Real4 derivVelRho = derivVelRhoD[index];
    Real4 rhoPresMu = rhoPresMuD[index];
    Real h = posRadD[index].w;
    Real p_tr, p_n;

    if (rhoPresMu.w < 0) {
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

            Real coh = paramsD.Coh_coeff;
            Real inv_mus = 1.0 / paramsD.mu_fric_s;
            Real P_cri = - coh * inv_mus;
            if (p_tr > P_cri) {
                Real tau_tr = square(updatedTauXxYyZz.x) + square(updatedTauXxYyZz.y) + 
                    square(updatedTauXxYyZz.z) + 2.0 * square(updatedTauXyXzYz.x) + 
                    2.0 * square(updatedTauXyXzYz.y) + 2.0 * square(updatedTauXyXzYz.z);
                Real tau_n = square(tauXxYyZz.x) + square(tauXxYyZz.y) + square(tauXxYyZz.z) +
                    2.0 * square(tauXyXzYz.x) + 2.0 * square(tauXyXzYz.y) + 2.0 * square(tauXyXzYz.z);
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
                Real I = Chi * dia * sqrt(paramsD.rho0 / p_tr);
                Real mu = mu_s + (mu_2 - mu_s) * (I + 1.0E-9) / (I0 + I + 1.0E-9);
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
            // Set stress to zero if the pressure is smaller than a critical value
            if (p_tr < P_cri) {
                updatedTauXxYyZz = mR3(0.0);
                updatedTauXyXzYz = mR3(0.0);
                p_tr = P_cri;
            }
            // Set stress to zero if the particle is close to free surface
            if (freeSurfaceIdD[index] == 1) {
                updatedTauXxYyZz = mR3(0.0);
                updatedTauXyXzYz = mR3(0.0);
                p_tr = 0.0;
            }

            if (paramsD.output_length == 2) {
                Real tau_tr = square(updatedTauXxYyZz.x) + square(updatedTauXxYyZz.y) + 
                    square(updatedTauXxYyZz.z) + 2.0 * (square(updatedTauXyXzYz.x) + 
                    square(updatedTauXyXzYz.y) + square(updatedTauXyXzYz.z));
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
    }

    // Important note: the derivVelRhoD that is calculated by the ChForceExplicitSPH is the negative of actual time
    // derivative. That is important to keep the derivVelRhoD to be the force/mass for fsi forces.
    // calculate the force that is f=m dv/dt
    // derivVelRhoD[index] *= paramsD.markerMass;
}

//------------------------------------------------------------------------------
__global__ void Update_Fluid_State(Real3* new_vel,
                                   Real4* posRad,
                                   Real3* velMas,
                                   Real4* rhoPreMu,
                                   int4 updatePortion,
                                   const size_t numAllMarkers,
                                   double dT,
                                   volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= updatePortion.y)
        return;

    velMas[i_idx] = new_vel[i_idx];

    Real3 newpos = mR3(posRad[i_idx]) + dT * velMas[i_idx];
    Real h = posRad[i_idx].w;
    posRad[i_idx] = mR4(newpos, h);

    if (!(isfinite(posRad[i_idx].x) && 
        isfinite(posRad[i_idx].y) && isfinite(posRad[i_idx].z))) {
        printf("Error! particle %d position is NAN: thrown from UpdateFluidDKernel  %f,%f,%f,%f\n",
            i_idx, posRad[i_idx].x, posRad[i_idx].y, posRad[i_idx].z, posRad[i_idx].w);
    }
    if (!(isfinite(rhoPreMu[i_idx].x) && 
        isfinite(rhoPreMu[i_idx].y) && isfinite(rhoPreMu[i_idx].z))) {
        printf("Error! particle %d rhoPreMu is NAN: thrown from UpdateFluidDKernel ! %f,%f,%f,%f\n",
            i_idx, rhoPreMu[i_idx].x, rhoPreMu[i_idx].y, rhoPreMu[i_idx].z, rhoPreMu[i_idx].w);
    }

    if (!(isfinite(velMas[i_idx].x) && 
        isfinite(velMas[i_idx].y) && isfinite(velMas[i_idx].z))) {
        printf("Error! particle %d velocity is NAN: thrown from UpdateFluidDKernel !%f,%f,%f\n",
            i_idx, velMas[i_idx].x, velMas[i_idx].y, velMas[i_idx].z);
    }
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
                                  uint* cellEnd,
                                  size_t numAllMarkers) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numAllMarkers)
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
                collideCellDensityReInit(numerator, denominator, neighbourPos, index, 
                    posRadA, sortedPosRad, sortedVelMas, sortedRhoPreMu, cellStart, cellEnd);
            }
        }
    }

    rhoPreMuA.x = numerator;  // denominator;
    //    rhoPreMuA.y = Eos(rhoPreMuA.x, rhoPreMuA.w);
    dummySortedRhoPreMu[index] = rhoPreMuA;
}

// -----------------------------------------------------------------------------
// Kernel for updating the activity of all particles.
__global__ void UpdateActivityD(Real4* posRadD,
                                Real3* velMasD,
                                Real3* posRigidBodiesD,
                                uint* activityIdentifierD,
                                uint* extendedActivityIdD,
                                int2 updatePortion,
                                size_t numRigidBodies,
                                Real Time,
                                volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    index += updatePortion.x;
    if (index >= updatePortion.y)
        return;

    // Set the particle as an active particle
    activityIdentifierD[index] = 1;
    extendedActivityIdD[index] = 1;

    // If during the settling phase, all particles are active
    if (Time < paramsD.settlingTime)
        return;

    // Check the activity of this particle
    uint isNotActive = 0;
    uint isNotExtended = 0;
    Real3 posRadA = mR3(posRadD[index]);
    for (uint num = 0; num < numRigidBodies; num++) {
        Real3 detPos = posRadA - posRigidBodiesD[num];
        Real3 Acdomain = paramsD.bodyActiveDomain;
        Real3 ExAcdomain = paramsD.bodyActiveDomain + 
            mR3(2 * RESOLUTION_LENGTH_MULT * paramsD.HSML);
        if (abs(detPos.x) > Acdomain.x || abs(detPos.y) > Acdomain.y || 
            abs(detPos.z) > Acdomain.z)
            isNotActive = isNotActive + 1;
        if (abs(detPos.x) > ExAcdomain.x || abs(detPos.y) > ExAcdomain.y || 
            abs(detPos.z) > ExAcdomain.z)
            isNotExtended = isNotExtended + 1;
    }

    // Set the particle as an inactive particle if needed
    if (isNotActive == numRigidBodies && numRigidBodies > 0) {
        activityIdentifierD[index] = 0;
        velMasD[index] = mR3(0.0);
    }
    if (isNotExtended == numRigidBodies && numRigidBodies > 0)
        extendedActivityIdD[index] = 0;

    return;
}

// -----------------------------------------------------------------------------
// CLASS FOR FLUID DYNAMICS SYSTEM
// -----------------------------------------------------------------------------
ChFluidDynamics::ChFluidDynamics(std::shared_ptr<ChBce> otherBceWorker,
                                 ChSystemFsi_impl& otherFsiSystem,
                                 std::shared_ptr<SimParams> otherParamsH,
                                 std::shared_ptr<ChCounters> otherNumObjects,
                                 TimeIntegrator type,
                                 bool verb)
    : fsiSystem(otherFsiSystem),
      paramsH(otherParamsH),
      numObjectsH(otherNumObjects),
      integrator_type(type),
      verbose(verb) {
    switch (integrator_type) {
        case TimeIntegrator::I2SPH:
            forceSystem = chrono_types::make_shared<ChFsiForceI2SPH>(
                otherBceWorker, fsiSystem.sortedSphMarkersD, fsiSystem.markersProximityD, 
                fsiSystem.fsiGeneralData, paramsH, numObjectsH, verb);
            if (verbose) {
                cout << "============================================" << endl;
                cout << "======   Created an I2SPH framework   ======" << endl;
                cout << "============================================" << endl;
            }
            break;

        case TimeIntegrator::IISPH:
            forceSystem = chrono_types::make_shared<ChFsiForceIISPH>(
                otherBceWorker, fsiSystem.sortedSphMarkersD, fsiSystem.markersProximityD,
                fsiSystem.fsiGeneralData, paramsH, numObjectsH, verb);
            if (verbose) {
                cout << "============================================" << endl;
                cout << "======   Created an IISPH framework   ======" << endl;
                cout << "============================================" << endl;
            }
            break;

        case TimeIntegrator::EXPLICITSPH:
            forceSystem = chrono_types::make_shared<ChFsiForceExplicitSPH>(
                otherBceWorker, fsiSystem.sortedSphMarkersD, fsiSystem.markersProximityD, 
                fsiSystem.fsiGeneralData, paramsH, numObjectsH, verb);
            if (verbose) {
                cout << "============================================" << endl;
                cout << "======   Created a WCSPH framework   =======" << endl;
                cout << "============================================" << endl;
            }
            break;

        // Extend this function with your own linear solvers
        default:
            forceSystem = chrono_types::make_shared<ChFsiForceExplicitSPH>(
                otherBceWorker, fsiSystem.sortedSphMarkersD, fsiSystem.markersProximityD, 
                fsiSystem.fsiGeneralData, paramsH, numObjectsH, verb);
            cout << "Selected integrator type not implemented, reverting back to WCSPH" << endl;
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
void ChFluidDynamics::IntegrateSPH(std::shared_ptr<SphMarkerDataD> sphMarkersD2,
                                   std::shared_ptr<SphMarkerDataD> sphMarkersD1,
                                   std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                                   std::shared_ptr<FsiMeshDataD> fsiMeshD,
                                   Real dT,
                                   Real Time) {
    if (GetIntegratorType() == TimeIntegrator::EXPLICITSPH) {
        this->UpdateActivity(sphMarkersD1, sphMarkersD2, fsiBodiesD, Time);
        forceSystem->ForceSPH(sphMarkersD2, fsiBodiesD, fsiMeshD);
    } else
        forceSystem->ForceSPH(sphMarkersD1, fsiBodiesD, fsiMeshD);

    if (integrator_type == TimeIntegrator::IISPH)
        this->UpdateFluid_Implicit(sphMarkersD2);
    else if (GetIntegratorType() == TimeIntegrator::EXPLICITSPH)
        this->UpdateFluid(sphMarkersD1, dT);

    this->ApplyBoundarySPH_Markers(sphMarkersD2);
}

// -----------------------------------------------------------------------------
void ChFluidDynamics::UpdateActivity(std::shared_ptr<SphMarkerDataD> sphMarkersD1,
                                     std::shared_ptr<SphMarkerDataD> sphMarkersD2,
                                     std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                                     Real Time) {
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
        mR4CAST(sphMarkersD2->posRadD), mR3CAST(sphMarkersD1->velMasD), 
        mR3CAST(fsiBodiesD->posRigid_fsiBodies_D),
        U1CAST(fsiSystem.fsiGeneralData->activityIdentifierD), 
        U1CAST(fsiSystem.fsiGeneralData->extendedActivityIdD),
        updatePortion, numObjectsH->numRigidBodies, Time, isErrorD);
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
void ChFluidDynamics::UpdateFluid(std::shared_ptr<SphMarkerDataD> sphMarkersD, Real dT) {
    // Update portion of the SPH particles (should be fluid particles only here)
    int2 updatePortion = mI2(0, fsiSystem.fsiGeneralData->referenceArray[0].y);

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    //------------------------
    uint numBlocks, numThreads;
    computeGridSize(updatePortion.y - updatePortion.x, 256, numBlocks, numThreads);
    UpdateFluidD<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), 
        mR3CAST(sphMarkersD->velMasD), 
        mR4CAST(sphMarkersD->rhoPresMuD), 
        mR3CAST(sphMarkersD->tauXxYyZzD), 
        mR3CAST(sphMarkersD->tauXyXzYzD), 
        mR3CAST(fsiSystem.fsiGeneralData->vel_XSPH_D), 
        mR4CAST(fsiSystem.fsiGeneralData->derivVelRhoD_old),
        mR3CAST(fsiSystem.fsiGeneralData->derivTauXxYyZzD), 
        mR3CAST(fsiSystem.fsiGeneralData->derivTauXyXzYzD),
        mR4CAST(fsiSystem.fsiGeneralData->sr_tau_I_mu_i), 
        U1CAST(fsiSystem.fsiGeneralData->activityIdentifierD),
        U1CAST(fsiSystem.fsiGeneralData->freeSurfaceIdD), 
        updatePortion, dT, isErrorD);
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
void ChFluidDynamics::UpdateFluid_Implicit(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    uint numThreads, numBlocks;
    computeGridSize((int)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);

    int haveGhost = (numObjectsH->numGhostMarkers > 0) ? 1 : 0;
    int haveHelper = (numObjectsH->numHelperMarkers > 0) ? 1 : 0;

    int4 updatePortion = mI4(fsiSystem.fsiGeneralData->referenceArray[haveHelper].x,
        fsiSystem.fsiGeneralData->referenceArray[haveHelper + haveGhost].y, 0, 0);

    cout << "time step in UpdateFluid_Implicit " << paramsH->dT << endl;
    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    Update_Fluid_State<<<numBlocks, numThreads>>>(
        mR3CAST(fsiSystem.fsiGeneralData->vel_XSPH_D), 
        mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD), 
        mR4CAST(sphMarkersD->rhoPresMuD), updatePortion,
        numObjectsH->numAllMarkers, paramsH->dT, isErrorD);
    cudaDeviceSynchronize();
    cudaCheckError();

    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true)
        throw std::runtime_error("Error! program crashed in Update_Fluid_State!\n");
    cudaFree(isErrorD);
    free(isErrorH);
}

// -----------------------------------------------------------------------------
// Apply periodic boundary conditions in x, y, and z directions
void ChFluidDynamics::ApplyBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    uint numBlocks, numThreads;

    computeGridSize((int)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);
    ApplyPeriodicBoundaryXKernel<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD),
        U1CAST(fsiSystem.fsiGeneralData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();

    ApplyPeriodicBoundaryYKernel<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD),
        U1CAST(fsiSystem.fsiGeneralData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();

    ApplyPeriodicBoundaryZKernel<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD),
        U1CAST(fsiSystem.fsiGeneralData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();

    // ApplyOutOfBoundaryKernel<<<numBlocks, numThreads>>>
    //     (mR4CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD), mR3CAST(sphMarkersD->velMasD));
    // cudaDeviceSynchronize();
    // cudaCheckError();
}

// -----------------------------------------------------------------------------
// Apply periodic boundary conditions in y, and z. 
// The inlet/outlet BC is applied in the x direction.
// This functions needs to be tested.
void ChFluidDynamics::ApplyModifiedBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    uint numBlocks, numThreads;
    computeGridSize((int)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);
    ApplyInletBoundaryXKernel<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD),
        mR4CAST(sphMarkersD->rhoPresMuD));
    cudaDeviceSynchronize();
    cudaCheckError();

    // these are useful anyway for out of bound particles
    ApplyPeriodicBoundaryYKernel<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD),
        U1CAST(fsiSystem.fsiGeneralData->activityIdentifierD));
    cudaDeviceSynchronize();
    cudaCheckError();

    ApplyPeriodicBoundaryZKernel<<<numBlocks, numThreads>>>(
        mR4CAST(sphMarkersD->posRadD), mR4CAST(sphMarkersD->rhoPresMuD),
        U1CAST(fsiSystem.fsiGeneralData->activityIdentifierD));
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
        mR4CAST(dummySortedRhoPreMu), 
        mR4CAST(fsiSystem.sortedSphMarkersD->posRadD),
        mR3CAST(fsiSystem.sortedSphMarkersD->velMasD), 
        mR4CAST(fsiSystem.sortedSphMarkersD->rhoPresMuD),
        U1CAST(fsiSystem.markersProximityD->gridMarkerIndexD), 
        U1CAST(fsiSystem.markersProximityD->cellStartD),
        U1CAST(fsiSystem.markersProximityD->cellEndD), 
        numObjectsH->numAllMarkers);

    cudaDeviceSynchronize();
    cudaCheckError();
    ChFsiForce::CopySortedToOriginal_NonInvasive_R4(
        fsiSystem.sphMarkersD1->rhoPresMuD, dummySortedRhoPreMu,
        fsiSystem.markersProximityD->gridMarkerIndexD);
    ChFsiForce::CopySortedToOriginal_NonInvasive_R4(
        fsiSystem.sphMarkersD2->rhoPresMuD, dummySortedRhoPreMu,
        fsiSystem.markersProximityD->gridMarkerIndexD);
    dummySortedRhoPreMu.clear();
}

}  // namespace fsi
}  // end namespace chrono
