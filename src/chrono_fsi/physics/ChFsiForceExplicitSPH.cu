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
// Author: Arman Pazouki, Wei Hu
// =============================================================================
#include <thrust/extrema.h>
#include <thrust/sort.h>
#include "chrono_fsi/physics/ChFsiForceExplicitSPH.cuh"

//================================================================================================================================
namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Shear_Stress_Rate(Real4* sortedPosRad,
                                  Real4* sortedRhoPreMu,
                                  Real3* sortedVelMas,
                                  Real3* velMas_ModifiedBCE,
                                  Real4* rhoPreMu_ModifiedBCE,
                                  Real3* sortedTauXxYyZz,
                                  Real3* sortedTauXyXzYz,
                                  Real3* sortedDerivTauXxYyZz,
                                  Real3* sortedDerivTauXyXzYz,
                                  uint* gridMarkerIndex,
                                  uint* cellStart,
                                  uint* cellEnd,
                                  const size_t numAllMarkers) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numAllMarkers) {
        return;
    }

    Real3 posRadA = mR3(sortedPosRad[index]);
    Real3 velMasA = sortedVelMas[index];
    Real hA = sortedPosRad[index].w;

    Real tauxx = sortedTauXxYyZz[index].x;
    Real tauyy = sortedTauXxYyZz[index].y;
    Real tauzz = sortedTauXxYyZz[index].z;
    Real tauxy = sortedTauXyXzYz[index].x;
    Real tauxz = sortedTauXyXzYz[index].y;
    Real tauyz = sortedTauXyXzYz[index].z;
    Real tauzx = tauxz;
    Real tauzy = tauyz;
    Real tauyx = tauxy;
    Real dTauxx = 0.0;
    Real dTauyy = 0.0;
    Real dTauzz = 0.0;
    Real dTauxy = 0.0;
    Real dTauxz = 0.0;
    Real dTauyz = 0.0;

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                uint startIndex = cellStart[gridHash];
                uint endIndex = cellEnd[gridHash];
                for (uint j = startIndex; j < endIndex; j++) {
                    if (j != index) {
                        Real3 posRadB = mR3(sortedPosRad[j]);
                        Real3 dist3 = Distance(posRadA, posRadB);
                        Real d = length(dist3);

                        if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
                            continue;
                        Real3 velMasB = sortedVelMas[j];
                        Real4 rhoPresMuB = sortedRhoPreMu[j];
                        if (rhoPresMuB.w > -1.0) {
                            int bceIndexB = gridMarkerIndex[j] - (numObjectsD.numFluidMarkers);
                            if (!(bceIndexB >= 0 &&
                                  bceIndexB < numObjectsD.numBoundaryMarkers + numObjectsD.numRigid_SphMarkers)) {
                                printf("Error! bceIndex out of bound, collideCell !\n");
                            }
                            rhoPresMuB = rhoPreMu_ModifiedBCE[bceIndexB];
                            velMasB = velMas_ModifiedBCE[bceIndexB];
                        }
                        Real rhoB = rhoPresMuB.x;
                        Real hB = sortedPosRad[j].w;
                        Real mB = paramsD.markerMass;
                        Real3 gradW = GradWh(dist3, (hA + hB) * 0.5);
                        // start to calculate the rate
                        Real Gm = paramsD.G_shear;  // shear modulus of the material
                        Real half_mB_over_rhoB = 0.5 * (mB / rhoB);
                        // entries of strain rate tensor
                        Real exx = -half_mB_over_rhoB *
                                   ((velMasA.x - velMasB.x) * gradW.x + (velMasA.x - velMasB.x) * gradW.x);
                        Real eyy = -half_mB_over_rhoB *
                                   ((velMasA.y - velMasB.y) * gradW.y + (velMasA.y - velMasB.y) * gradW.y);
                        Real ezz = -half_mB_over_rhoB *
                                   ((velMasA.z - velMasB.z) * gradW.z + (velMasA.z - velMasB.z) * gradW.z);
                        Real exy = -half_mB_over_rhoB *
                                   ((velMasA.x - velMasB.x) * gradW.y + (velMasA.y - velMasB.y) * gradW.x);
                        Real exz = -half_mB_over_rhoB *
                                   ((velMasA.x - velMasB.x) * gradW.z + (velMasA.z - velMasB.z) * gradW.x);
                        Real eyz = -half_mB_over_rhoB *
                                   ((velMasA.y - velMasB.y) * gradW.z + (velMasA.z - velMasB.z) * gradW.y);
                        // entries of rotation rate (spin) tensor
                        Real wxx = 0.0;
                        Real wyy = 0.0;
                        Real wzz = 0.0;
                        Real wxy = -half_mB_over_rhoB *
                                   ((velMasA.x - velMasB.x) * gradW.y - (velMasA.y - velMasB.y) * gradW.x);
                        Real wxz = -half_mB_over_rhoB *
                                   ((velMasA.x - velMasB.x) * gradW.z - (velMasA.z - velMasB.z) * gradW.x);
                        Real wyz = -half_mB_over_rhoB *
                                   ((velMasA.y - velMasB.y) * gradW.z - (velMasA.z - velMasB.z) * gradW.y);
                        Real wyx = -wxy;
                        Real wzx = -wxz;
                        Real wzy = -wyz;

                        Real edia = 1.0 / 3.0 * (exx + eyy + ezz);
                        dTauxx += 2.0 * Gm * (exx - edia) - (tauxx * wxx + tauxy * wyx + tauxz * wzx) +
                                  (wxx * tauxx + wxy * tauyx + wxz * tauzx);
                        dTauyy += 2.0 * Gm * (eyy - edia) - (tauyx * wxy + tauyy * wyy + tauyz * wzy) +
                                  (wyx * tauxy + wyy * tauyy + wyz * tauzy);
                        dTauzz += 2.0 * Gm * (ezz - edia) - (tauzx * wxz + tauzy * wyz + tauzz * wzz) +
                                  (wzx * tauxz + wzy * tauyz + wzz * tauzz);
                        dTauxy += 2.0 * Gm * (exy - 0.0) - (tauxx * wxy + tauxy * wyy + tauxz * wzy) +
                                  (wxx * tauxy + wxy * tauyy + wxz * tauzy);
                        dTauxz += 2.0 * Gm * (exz - 0.0) - (tauxx * wxz + tauxy * wyz + tauxz * wzz) +
                                  (wxx * tauxz + wxy * tauyz + wxz * tauzz);
                        dTauyz += 2.0 * Gm * (eyz - 0.0) - (tauyx * wxz + tauyy * wyz + tauyz * wzz) +
                                  (wyx * tauxz + wyy * tauyz + wyz * tauzz);
                    }
                }
            }
        }
    }
    sortedDerivTauXxYyZz[index] = mR3(dTauxx, dTauyy, dTauzz);
    sortedDerivTauXyXzYz[index] = mR3(dTauxy, dTauxz, dTauyz);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void calcRho_kernel(Real4* sortedPosRad,
                               Real4* sortedRhoPreMu,
                               Real4* sortedRhoPreMu_old,
                               Real* _sumWij_rhoi,
                               uint* cellStart,
                               uint* cellEnd,
                               const size_t numAllMarkers,
                               int density_reinit,
                               volatile bool* isErrorD) {
    uint i_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (i_idx >= numAllMarkers) {
        return;
    }
    sortedRhoPreMu_old[i_idx].y = Eos(sortedRhoPreMu_old[i_idx].x, sortedRhoPreMu_old[i_idx].w);

    Real3 posRadA = mR3(sortedPosRad[i_idx]);
    Real h_i = sortedPosRad[i_idx].w;

    Real sum_mW = 0;
    Real sum_mW_rho = 0.0000001;

    Real sum_W = 0.0;
    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                uint startIndex = cellStart[gridHash];
                if (startIndex != 0xffffffff) {
                    uint endIndex = cellEnd[gridHash];
                    for (uint j = startIndex; j < endIndex; j++) {
                        Real3 posRadB = mR3(sortedPosRad[j]);
                        Real3 dist3 = Distance(posRadA, posRadB);
                        Real d = length(dist3);
                        if (d > RESOLUTION_LENGTH_MULT * h_i)
                            continue;
                        if (sortedRhoPreMu_old[j].w == -1) {  //
                            Real h_j = sortedPosRad[j].w;
                            Real m_j = paramsD.markerMass;  // pow(h_j * paramsD.MULT_INITSPACE, 3) * paramsD.rho0;
                            Real W3 = W3h(d, 0.5 * (h_j + h_i));
                            sum_mW += m_j * W3;
                            sum_W += W3;
                            sum_mW_rho += m_j * W3 / sortedRhoPreMu_old[j].x;
                        }
                    }
                }
            }
        }
    }
    //    sumWij_inv[i_idx] = paramsD.markerMass / sum_mW;

    // sortedRhoPreMu[i_idx].x = sum_mW;
    if ((density_reinit == 0) && (sortedRhoPreMu[i_idx].w == -1))
        sortedRhoPreMu[i_idx].x = sum_mW / sum_mW_rho;

    if ((sortedRhoPreMu[i_idx].x > 3 * paramsD.rho0 || sortedRhoPreMu[i_idx].x < 0.01 * paramsD.rho0) &&
        sortedRhoPreMu[i_idx].w == -1)
        printf("(calcRho_kernel)density marker %d, sum_mW=%f, sum_W=%f, h_i=%f\n", i_idx, sum_mW, sum_W, h_i);
}

//--------------------------------------------------------------------------------------------------------------------------------
// modify pressure for body force
__device__ __inline__ void modifyPressure(Real4& rhoPresMuB, const Real3& dist3Alpha) {
    // body force in x direction
    rhoPresMuB.y = (dist3Alpha.x > 0.5 * paramsD.boxDims.x) ? (rhoPresMuB.y - paramsD.deltaPress.x) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.x < -0.5 * paramsD.boxDims.x) ? (rhoPresMuB.y + paramsD.deltaPress.x) : rhoPresMuB.y;
    // body force in x direction
    rhoPresMuB.y = (dist3Alpha.y > 0.5 * paramsD.boxDims.y) ? (rhoPresMuB.y - paramsD.deltaPress.y) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.y < -0.5 * paramsD.boxDims.y) ? (rhoPresMuB.y + paramsD.deltaPress.y) : rhoPresMuB.y;
    // body force in x direction
    rhoPresMuB.y = (dist3Alpha.z > 0.5 * paramsD.boxDims.z) ? (rhoPresMuB.y - paramsD.deltaPress.z) : rhoPresMuB.y;
    rhoPresMuB.y = (dist3Alpha.z < -0.5 * paramsD.boxDims.z) ? (rhoPresMuB.y + paramsD.deltaPress.z) : rhoPresMuB.y;
}

//--------------------------------------------------------------------------------------------------------------------------------
__device__ inline Real3 CubicSolve(Real aa, Real bb, Real cc, Real dd) {
    Real disc, q, r, dum1, dum2, term1, r13;
    bb /= aa;
    cc /= aa;
    dd /= aa;
    if (aa == 0) {
        return mR3(0, 0, 0);
    }
    if (abs(bb) < 1e-9) {
        return mR3(0, 0, 0);
    }
    if (abs(cc) < 1e-9) {
        return mR3(0, 0, 0);
    }
    if (abs(dd) < 1e-9) {
        return mR3(0, 0, 0);
    }
    q = (3.0 * cc - (bb * bb)) / 9.0;
    r = -(27.0 * dd) + bb * (9.0 * cc - 2.0 * (bb * bb));
    r /= 54.0;
    disc = q * q * q + r * r;
    term1 = (bb / 3.0);

    /*     dataForm.x1Im.value = 0; //The first root is always real.
        if (disc > 0) { // one root real, two are complex
            s = r + Math.sqrt(disc);
            s = ((s < 0) ? -Math.pow(-s, (1.0/3.0)) : Math.pow(s, (1.0/3.0)));
            t = r - Math.sqrt(disc);
            t = ((t < 0) ? -Math.pow(-t, (1.0/3.0)) : Math.pow(t, (1.0/3.0)));
            dataForm.x1Re.value = -term1 + s + t;
            term1 += (s + t)/2.0;
            dataForm.x3Re.value = dataForm.x2Re.value = -term1;
            term1 = Math.sqrt(3.0)*(-t + s)/2;
            dataForm.x2Im.value = term1;
            dataForm.x3Im.value = -term1;
            return;
        }
        // End if (disc > 0)
        // The remaining options are all real
        dataForm.x3Im.value = dataForm.x2Im.value = 0;
        if (disc == 0){ // All roots real, at least two are equal.
            r13 = ((r < 0) ? -Math.pow(-r,(1.0/3.0)) : Math.pow(r,(1.0/3.0)));
            dataForm.x1Re.value = -term1 + 2.0*r13;
            dataForm.x3Re.value = dataForm.x2Re.value = -(r13 + term1);
            return;
        } // End if (disc == 0)
    */

    Real xRex, xRey, xRez;
    // have complex root
    if (disc > 0) {
        xRex = 0.0;
        xRey = 0.0;
        xRez = 0.0;
        return mR3(xRex, xRey, xRez);
    }
    // All roots real, at least two are equal.
    if (disc == 0) {
        if (r < 0) {
            r13 = pow(-r, (1.0 / 3.0));
        } else {
            r13 = pow(r, (1.0 / 3.0));
        }
        xRex = -term1 + 2.0 * r13;
        xRey = -(r13 + term1);
        xRez = xRey;
        return mR3(xRex, xRey, xRez);
    }
    // All roots are real and unequal (to get here, q < 0)
    q = -q;
    dum1 = q * q * q;
    dum2 = r / (sqrt(dum1 + 1.0e-9));
    if ((dum2 >= 0) && (dum2 <= 1)) {
        dum1 = acos(dum2);
    } else {
        xRex = 0.0;
        xRey = 0.0;
        xRez = 0.0;
        return mR3(xRex, xRey, xRez);
    }
    r13 = 2.0 * sqrt(q);
    xRex = -term1 + r13 * cos(dum1 / 3.0);
    xRey = -term1 + r13 * cos((dum1 + 2.0 * 3.1415926) / 3.0);
    xRez = -term1 + r13 * cos((dum1 + 4.0 * 3.1415926) / 3.0);

    return mR3(xRex, xRey, xRez);
}
__device__ inline Real3 CubicEigen(Real4 c1, Real4 c2, Real4 c3) {
    Real a = c1.x;
    Real b = c1.y;
    Real c = c1.z;
    Real d = c1.w;

    Real l = c2.x;
    Real m = c2.y;
    Real n = c2.z;
    Real k = c2.w;

    Real p = c3.x;
    Real q = c3.y;
    Real r = c3.z;
    Real s = c3.w;

    Real D = (a * m * r + b * p * n + c * l * q) - (a * n * q + b * l * r + c * m * p) + 1.0e-9;
    Real x = ((b * r * k + c * m * s + d * n * q) - (b * n * s + c * q * k + d * m * r)) / D;
    Real y = ((a * n * s + c * p * k + d * l * r) - (a * r * k + c * l * s + d * n * p)) / D;
    Real z = ((a * q * k + b * l * s + d * m * p) - (a * m * s + b * p * k + d * l * q)) / D;

    b = b + 1.0e-9;
    x = 1.0e0;
    z = (-l + a * m / b) / (n - c * m / b);
    y = (-a - c * z) / b;
    Real R = sqrt(x * x + y * y + z * z);
    x = x / R;
    y = y / R;
    z = z / R;

    // if(abs(D) < 1){
    //     return mR3(0,0,0);
    // }

    // if(abs(m) < 0.1){
    //     x=0;
    //     y=1;
    //     z=0;
    //     return mR3(x,y,z);
    // }
    // else{
    //     y=0;
    //     if(abs(c) > 0.1){
    //         x=1;
    //         z=-a/c;
    //         return mR3(x,y,z);
    //     }
    //     if(abs(a) > 0.1){
    //         z=1;
    //         x=-c/a;
    //         return mR3(x,y,z);
    //     }
    // }

    return mR3(x, y, z);
}

//--------------------------------------------------------------------------------------------------------------------------------
/**	/* Old version was commented by Wei Hu
 * @brief DifVelocityRho
 * @details  See SDKCollisionSystem.cuh
 */
__device__ inline Real4 DifVelocityRho(Real3 dist3,
                                       Real d,
                                       Real4 posRadA,
                                       Real4 posRadB,
                                       Real3 velMasA,
                                       Real3 vel_XSPH_A,
                                       Real3 velMasB,
                                       Real3 vel_XSPH_B,
                                       Real4 rhoPresMuA,
                                       Real4 rhoPresMuB,
                                       Real multViscosity) {
    Real3 gradW = GradWh(dist3, (posRadA.w + posRadB.w) * 0.5);

    //    Real vAB_Dot_rAB = dot(velMasA - velMasB, dist3);
    //
    //    //	//*** Artificial viscosity type 1.1
    //    Real alpha = .001;
    //    Real c_ab = 10 * paramsD.v_Max;  // Ma = .1;//sqrt(7.0f * 10000 /
    //                                     //    ((rhoPresMuA.x + rhoPresMuB.x) / 2.0f));
    //                                     // Real h = paramsD.HSML;
    //    Real rho = .5f * (rhoPresMuA.x + rhoPresMuB.x);
    //    Real nu = alpha * paramsD.HSML * c_ab / rho;
    //
    //    //*** Artificial viscosity type 1.2
    //    //    Real nu = 22.8f * paramsD.mu0 / 2.0f / (rhoPresMuA.x * rhoPresMuB.x);
    //    Real3 derivV = -paramsD.markerMass *
    //                   (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x) -
    //                    nu * vAB_Dot_rAB / (d * d + paramsD.epsMinMarkersDis * paramsD.HSML * paramsD.HSML)) *
    //                   gradW;
    //    return mR4(derivV, rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));

    //*** Artificial viscosity type 2
    if (rhoPresMuA.w > -1 && rhoPresMuB.w > -1)
        return mR4(0.0);

    Real rAB_Dot_GradWh = dot(dist3, gradW);
    Real rAB_Dot_GradWh_OverDist = rAB_Dot_GradWh / (d * d + paramsD.epsMinMarkersDis * paramsD.HSML * paramsD.HSML);
    Real3 derivV = - paramsD.markerMass *(rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x * rhoPresMuB.x)) * gradW
                   + paramsD.markerMass * (8.0f * multViscosity) * paramsD.mu0 
                   * pow(rhoPresMuA.x + rhoPresMuB.x, Real(-2)) * rAB_Dot_GradWh_OverDist * (velMasA - velMasB);

    //    Real derivRho = rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW);
    //	Real zeta = 0;//.05;//.1;
    //	Real derivRho = rhoPresMuA.x * paramsD.markerMass * invrhoPresMuBx *
    //(dot(vel_XSPH_A - vel_XSPH_B, gradW)
    //			+ zeta * paramsD.HSML * (10 * paramsD.v_Max) * 2 * (rhoPresMuB.x
    /// rhoPresMuA.x - 1) *
    // rAB_Dot_GradWh_OverDist
    //			);

    //--------------------------------
    // Ferrari Modification
    Real derivRho = paramsD.markerMass * dot(vel_XSPH_A - vel_XSPH_B, gradW);
    //    Real cA = FerrariCi(rhoPresMuA.x);
    //    Real cB = FerrariCi(rhoPresMuB.x);
    //    derivRho += rAB_Dot_GradWh / (d + paramsD.epsMinMarkersDis * paramsD.HSML) * max(cA, cB) / rhoPresMuB.x *
    //                (rhoPresMuB.x - rhoPresMuA.x);

    //    --------------------------------
    return mR4(derivV, derivRho);

    //	//*** Artificial viscosity type 1.3
    //    Real rAB_Dot_GradWh = dot(dist3, gradW);
    //    Real3 derivV = -paramsD.markerMass *
    //                       (rhoPresMuA.y / (rhoPresMuA.x * rhoPresMuA.x) + rhoPresMuB.y / (rhoPresMuB.x *
    //                       rhoPresMuB.x)) * gradW +
    //                   paramsD.markerMass / (rhoPresMuA.x * rhoPresMuB.x) * 2.0f * paramsD.mu0 * rAB_Dot_GradWh /
    //                       (d * d + paramsD.epsMinMarkersDis * paramsD.HSML * paramsD.HSML) * (velMasA - velMasB);
    //    return mR4(derivV, rhoPresMuA.x * paramsD.markerMass / rhoPresMuB.x * dot(vel_XSPH_A - vel_XSPH_B, gradW));
}

/// Only for modelling elastic and granular problems
__device__ inline Real4 DifVelocityRho_ElasticSPH(Real3 dist3,
                                                  Real d,
                                                  Real4 posRadA,
                                                  Real4 posRadB,
                                                  Real3 velMasA,
                                                  Real3 vel_XSPH_A,
                                                  Real3 velMasB,
                                                  Real3 vel_XSPH_B,
                                                  Real4 rhoPresMuA,
                                                  Real4 rhoPresMuB,
                                                  Real multViscosity,
                                                  Real3 tauXxYyZz_A,
                                                  Real3 tauXyXzYz_A,
                                                  Real3 tauXxYyZz_B,
                                                  Real3 tauXyXzYz_B) {
    Real3 gradW = GradWh(dist3, (posRadA.w + posRadB.w) * 0.5);

    // if (rhoPresMuA.w > -1 )
    //     return mR4(0.0);
    // if (rhoPresMuB.w > -1 )
    //     return mR4(0.0);
    if (rhoPresMuA.w > -1 && rhoPresMuB.w > -1)
        return mR4(0.0);

    Real txxA = tauXxYyZz_A.x;
    Real tyyA = tauXxYyZz_A.y;
    Real tzzA = tauXxYyZz_A.z;
    Real txyA = tauXyXzYz_A.x;
    Real txzA = tauXyXzYz_A.y;
    Real tyzA = tauXyXzYz_A.z;

    Real txxB = tauXxYyZz_B.x;
    Real tyyB = tauXxYyZz_B.y;
    Real tzzB = tauXxYyZz_B.z;
    Real txyB = tauXyXzYz_B.x;
    Real txzB = tauXyXzYz_B.y;
    Real tyzB = tauXyXzYz_B.z;

    Real PA = rhoPresMuA.y;
    Real PB = rhoPresMuB.y;
    Real rhoA = rhoPresMuA.x;
    Real rhoB = rhoPresMuB.x;

    Real Mass = paramsD.markerMass;

    Real derivVx = -Mass * (PA / (rhoA * rhoA) + PB / (rhoB * rhoB)) * gradW.x +
                   Mass * (txxA * gradW.x + txyA * gradW.y + txzA * gradW.z) / (rhoA * rhoA) +
                   Mass * (txxB * gradW.x + txyB * gradW.y + txzB * gradW.z) / (rhoB * rhoB);
    Real derivVy = -Mass * (PA / (rhoA * rhoA) + PB / (rhoB * rhoB)) * gradW.y +
                   Mass * (txyA * gradW.x + tyyA * gradW.y + tyzA * gradW.z) / (rhoA * rhoA) +
                   Mass * (txyB * gradW.x + tyyB * gradW.y + tyzB * gradW.z) / (rhoB * rhoB);
    Real derivVz = -Mass * (PA / (rhoA * rhoA) + PB / (rhoB * rhoB)) * gradW.z +
                   Mass * (txzA * gradW.x + tyzA * gradW.y + tzzA * gradW.z) / (rhoA * rhoA) +
                   Mass * (txzB * gradW.x + tyzB * gradW.y + tzzB * gradW.z) / (rhoB * rhoB);

    //*** Artificial viscosity type 1.1
    Real vAB_Dot_rAB = dot(velMasA - velMasB, dist3);
    if (vAB_Dot_rAB < 0.0) {
        Real alpha = paramsD.Ar_vis_alpha;
        Real c_ab = paramsD.Cs;
        Real rho = 0.5f * (rhoA + rhoB);
        Real nu = -alpha * paramsD.HSML * c_ab / rho;
        Real derivM1 = -Mass * (nu * vAB_Dot_rAB / (d * d + paramsD.epsMinMarkersDis * paramsD.HSML * paramsD.HSML));
        derivVx += derivM1 * gradW.x;
        derivVy += derivM1 * gradW.y;
        derivVz += derivM1 * gradW.z;
    }

    // damping force
    if (1 == 1) {
        Real xi0 = paramsD.Vis_Dam;
        Real E0 = paramsD.E_young;
        Real h0 = paramsD.HSML;
        Real Cd = xi0 * sqrt(E0 / (rhoA * h0 * h0));
        derivVx -= Cd * velMasA.x;
        derivVy -= Cd * velMasA.y;
        derivVz -= Cd * velMasA.z;
    }

    Real derivRho = Mass * dot(vel_XSPH_A - vel_XSPH_B, gradW);
    return mR4(derivVx, derivVy, derivVz, derivRho);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void EOS(Real4* sortedRhoPreMu, uint numAllMarkers, volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numAllMarkers)
        return;
    sortedRhoPreMu[index].y = Eos(sortedRhoPreMu[index].x, sortedRhoPreMu[index].w);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Navier_Stokes(Real4* sortedDerivVelRho,  // output: new velocity
                              Real3* shift_r,
                              Real4* sortedPosRad,
                              Real3* sortedVelMas,
                              Real4* sortedRhoPreMu,
                              Real3* velMas_ModifiedBCE,
                              Real4* rhoPreMu_ModifiedBCE,
                              Real3* sortedTauXxYyZz,  //
                              Real3* sortedTauXyXzYz,  //
                              uint* gridMarkerIndex,
                              uint* cellStart,
                              uint* cellEnd,
                              const size_t numAllMarkers,
                              Real MaxVel,
                              volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numAllMarkers)
        return;
    Real3 posRadA = mR3(sortedPosRad[index]);
    Real3 velMasA = sortedVelMas[index];
    Real4 rhoPresMuA = sortedRhoPreMu[index];
    Real4 derivVelRho = mR4(0.0);

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    Real3 inner_sum = mR3(0.0);
    Real mi_bar = 0.0, r0 = 0.0;
    int N_ = 1;
    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            for (int z = -1; z <= 1; z++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                uint startIndex = cellStart[gridHash];
                uint endIndex = cellEnd[gridHash];
                for (uint j = startIndex; j < endIndex; j++) {
                    if (j != index) {
                        Real3 posRadB = mR3(sortedPosRad[j]);
                        // Real3 dist3Alpha = posRadA - posRadB;
                        Real3 dist3 = Distance(posRadA, posRadB);  // change from B-A to A-B
                        Real d = length(dist3);
                        if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
                            continue;
                        Real4 rhoPresMuB = sortedRhoPreMu[j];
                        if (rhoPresMuA.w > -.1 && rhoPresMuB.w > -.1) {  // no rigid-rigid force
                            continue;
                        }
                        // modifyPressure(rhoPresMuB, dist3Alpha);
                        if (!(isfinite(rhoPresMuB.x) && isfinite(rhoPresMuB.y) && isfinite(rhoPresMuB.z))) {
                            printf("Error! particle rhoPresMuB is NAN: thrown from modifyPressure !\n");
                        }
                        Real3 velMasB = sortedVelMas[j];
                        if (rhoPresMuB.w > -1.0) {
                            int bceIndexB = gridMarkerIndex[j] - (numObjectsD.numFluidMarkers);
                            if (!(bceIndexB >= 0 &&
                                  bceIndexB < numObjectsD.numBoundaryMarkers + numObjectsD.numRigid_SphMarkers)) {
                                printf("Error! bceIndex out of bound, collideCell !\n");
                            }
                            rhoPresMuB = rhoPreMu_ModifiedBCE[bceIndexB];
                            velMasB = velMas_ModifiedBCE[bceIndexB];
                        }
                        Real multViscosit = 1;
                        if (!(isfinite(rhoPresMuB.x) && isfinite(rhoPresMuB.y) && isfinite(rhoPresMuB.z))) {
                            printf("Error! particle rhoPresMuB is NAN: thrown from collideCell ! type=%f\n",
                                   rhoPresMuB.w);
                        }
                        // change from "-=" to "+="
                        if(paramsD.elastic_SPH){
                            derivVelRho += DifVelocityRho_ElasticSPH(dist3, d, sortedPosRad[index], sortedPosRad[j], velMasA, velMasA,
                                                      velMasB, velMasB, rhoPresMuA, rhoPresMuB, multViscosit,
                                                      sortedTauXxYyZz[index], sortedTauXyXzYz[index],  //
                                                      sortedTauXxYyZz[j], sortedTauXyXzYz[j]);}         //}

                        else{
                            derivVelRho += DifVelocityRho(dist3, d, sortedPosRad[index], sortedPosRad[j], velMasA, velMasA,
                                                       velMasB, velMasB, rhoPresMuA, rhoPresMuB, multViscosit);}

                        if (d > EPSILON) {
                            Real m_j = pow(sortedPosRad[j].w * paramsD.MULT_INITSPACE, 3) * paramsD.rho0;
                            mi_bar += m_j;
                            r0 += d;
                            inner_sum += m_j * (-dist3) / (d * d * d);  // change from dist3 to -dist3
                        }
                    }
                }
            }
        }
    }

    if (!(isfinite(derivVelRho.x) && isfinite(derivVelRho.y) && isfinite(derivVelRho.z))) {
        printf("Error! particle derivVel is NAN: thrown from ChFsiForceExplicitSPH.cu, collideD !\n");
        *isErrorD = true;
    }
    if (!(isfinite(derivVelRho.w))) {
        printf("Error! particle derivRho is NAN: thrown from ChFsiForceExplicitSPH.cu, collideD !\n");
        *isErrorD = true;
    }

    sortedDerivVelRho[index] = derivVelRho;

    r0 /= N_;
    mi_bar /= N_;
    if (abs(mi_bar) > EPSILON && sortedRhoPreMu[index].w == -1.0)
        shift_r[index] = paramsD.beta_shifting * r0 * r0 * MaxVel * paramsD.dT * inner_sum / mi_bar;
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void CalcVel_XSPH_D(Real3* vel_XSPH_Sorted_D,  // output: new velocity
                               Real4* sortedPosRad_old,   // input: sorted positions
                               Real4* sortedPosRad,       // input: sorted positions
                               Real3* sortedVelMas,       // input: sorted velocities
                               Real4* sortedRhoPreMu,
                               Real3* shift_r,

                               uint* gridMarkerIndex,  // input: sorted particle indices
                               uint* cellStart,
                               uint* cellEnd,
                               const size_t numAllMarkers,
                               volatile bool* isErrorD) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numAllMarkers)
        return;

    Real4 rhoPreMuA = sortedRhoPreMu[index];
    Real3 velMasA = sortedVelMas[index];

    Real3 posRadA = mR3(sortedPosRad_old[index]);
    Real3 deltaV = mR3(0);

    // get address in grid
    int3 gridPos = calcGridPos(posRadA);
    Real3 inner_sum = mR3(0.0);
    //    Real mi_bar = 0.0, r0 = 0.0;
    Real3 dV = mR3(0.0f);
    // examine neighbouring cells
    for (int z = -1; z <= 1; z++) {
        for (int y = -1; y <= 1; y++) {
            for (int x = -1; x <= 1; x++) {
                int3 neighbourPos = gridPos + mI3(x, y, z);
                uint gridHash = calcGridHash(neighbourPos);
                uint startIndex = cellStart[gridHash];
                uint endIndex = cellEnd[gridHash];
                for (uint j = startIndex; j < endIndex; j++) {
                    if (j != index) {  // check not colliding with self
                        Real3 posRadB = mR3(sortedPosRad_old[j]);
                        Real3 dist3 = Distance(posRadA, posRadB);
                        Real d = length(dist3);
                        if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
                            continue;
                        Real4 rhoPresMuB = sortedRhoPreMu[j];

                        if (rhoPresMuB.w != -1.0)
                            continue;
                        Real3 velMasB = sortedVelMas[j];
                        Real rho_bar = 0.5 * (rhoPreMuA.x + rhoPresMuB.x);
                        deltaV += paramsD.markerMass * (velMasB - velMasA) *
                                  W3h(d, (sortedPosRad_old[index].w + sortedPosRad_old[j].w) * 0.5) / rho_bar;
                    }
                }
            }
        }
    }

    vel_XSPH_Sorted_D[index] = deltaV;

    // sortedPosRad[index] -= mR4(shift_r[index], 0.0); //

    if (!(isfinite(vel_XSPH_Sorted_D[index].x) && isfinite(vel_XSPH_Sorted_D[index].y) &&
          isfinite(vel_XSPH_Sorted_D[index].z))) {
        printf("Error! particle vXSPH is NAN: thrown from ChFsiForceExplicitSPH.cu, newVel_XSPH_D !\n");
        *isErrorD = true;
    }
}

//--------------------------------------------------------------------------------------------------------------------------------
ChFsiForceExplicitSPH::ChFsiForceExplicitSPH(std::shared_ptr<ChBce> otherBceWorker,
                                             std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,
                                             std::shared_ptr<ProximityDataD> otherMarkersProximityD,
                                             std::shared_ptr<FsiGeneralData> otherFsiGeneralData,
                                             std::shared_ptr<SimParams> otherParamsH,
                                             std::shared_ptr<NumberOfObjects> otherNumObjects)
    : ChFsiForce(otherBceWorker,
                 otherSortedSphMarkersD,
                 otherMarkersProximityD,
                 otherFsiGeneralData,
                 otherParamsH,
                 otherNumObjects) {
    CopyParams_NumberOfObjects(paramsH, numObjectsH);
    density_initialization = 0;
}

//--------------------------------------------------------------------------------------------------------------------------------
ChFsiForceExplicitSPH::~ChFsiForceExplicitSPH() {}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::Finalize() {
    ChFsiForce::Finalize();
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(NumberOfObjects));
    cudaMemcpyFromSymbol(paramsH.get(), paramsD, sizeof(SimParams));
    cudaDeviceSynchronize();
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::ForceSPH(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                                     std::shared_ptr<FsiBodiesDataD> otherFsiBodiesD,
                                     std::shared_ptr<FsiMeshDataD> fsiMeshD) {
    sphMarkersD = otherSphMarkersD;
    fsiCollisionSystem->ArrangeData(sphMarkersD);
    bceWorker->ModifyBceVelocity(sphMarkersD, otherFsiBodiesD);
    CollideWrapper();
    CalculateXSPH_velocity();
    AddGravityToFluid();
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::CollideWrapper() {
    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    //------------------------------------------------------------------------
    // thread per particle
    uint numThreads, numBlocks;
    computeGridSize((int)numObjectsH->numAllMarkers, 64, numBlocks, numThreads);
    /* Execute the kernel */
    thrust::device_vector<Real> _sumWij_rhoi(numObjectsH->numAllMarkers);
    thrust::device_vector<Real4> sortedDerivVelRho(numObjectsH->numAllMarkers);
    thrust::device_vector<Real3> sortedDerivTauXxYyZz(numObjectsH->numAllMarkers);
    thrust::device_vector<Real3> sortedDerivTauXyXzYz(numObjectsH->numAllMarkers);
    shift_r.resize(numObjectsH->numAllMarkers);
    thrust::fill(_sumWij_rhoi.begin(), _sumWij_rhoi.end(), 0.);
    thrust::fill(shift_r.begin(), shift_r.end(), mR3(0.0));
    thrust::fill(sortedDerivVelRho.begin(), sortedDerivVelRho.end(), mR4(0.0));
    thrust::fill(sortedDerivTauXxYyZz.begin(), sortedDerivTauXxYyZz.end(), mR3(0.0));
    thrust::fill(sortedDerivTauXyXzYz.begin(), sortedDerivTauXyXzYz.end(), mR3(0.0));

    thrust::device_vector<Real4> rhoPresMuD_old = sortedSphMarkersD->rhoPresMuD;

    if (density_initialization == 0)
        printf("Re-initializing density after %d steps.", paramsH->densityReinit);
    calcRho_kernel<<<numBlocks, numThreads>>>(
        mR4CAST(sortedSphMarkersD->posRadD), mR4CAST(sortedSphMarkersD->rhoPresMuD), mR4CAST(rhoPresMuD_old),
        R1CAST(_sumWij_rhoi), U1CAST(markersProximityD->cellStartD), U1CAST(markersProximityD->cellEndD),
        numObjectsH->numAllMarkers, density_initialization, isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "calcRho_kernel");

    //    EOS<<<numBlocks, numThreads>>>(mR4CAST(sortedSphMarkersD->rhoPresMuD),
    //    numObjectsH->numAllMarkers,
    //                                                 isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "EOS");

    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    thrust::device_vector<Real3>::iterator iter =
        thrust::max_element(sortedSphMarkersD->velMasD.begin(), sortedSphMarkersD->velMasD.end(), compare_Real3_mag());
    auto position = iter - sortedSphMarkersD->velMasD.begin();
    Real MaxVel = length(*iter);

    if(paramsH->elastic_SPH){
        // calculate the rate of shear stress tau
        Shear_Stress_Rate<<<numBlocks, numThreads>>>(
            mR4CAST(sortedSphMarkersD->posRadD), mR4CAST(sortedSphMarkersD->rhoPresMuD),
            mR3CAST(sortedSphMarkersD->velMasD), mR3CAST(bceWorker->velMas_ModifiedBCE),
            mR4CAST(bceWorker->rhoPreMu_ModifiedBCE), mR3CAST(sortedSphMarkersD->tauXxYyZzD),
            mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR3CAST(sortedDerivTauXxYyZz), mR3CAST(sortedDerivTauXyXzYz),
            U1CAST(markersProximityD->gridMarkerIndexD), U1CAST(markersProximityD->cellStartD),
            U1CAST(markersProximityD->cellEndD), numObjectsH->numAllMarkers);
        ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Shear_Stress_Rate");
    }

    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    // execute the kernel
    Navier_Stokes<<<numBlocks, numThreads>>>(
        mR4CAST(sortedDerivVelRho), mR3CAST(shift_r), mR4CAST(sortedSphMarkersD->posRadD),
        mR3CAST(sortedSphMarkersD->velMasD), mR4CAST(sortedSphMarkersD->rhoPresMuD),
        mR3CAST(bceWorker->velMas_ModifiedBCE), mR4CAST(bceWorker->rhoPreMu_ModifiedBCE),
        mR3CAST(sortedSphMarkersD->tauXxYyZzD), mR3CAST(sortedSphMarkersD->tauXyXzYzD),  //
        U1CAST(markersProximityD->gridMarkerIndexD), U1CAST(markersProximityD->cellStartD),
        U1CAST(markersProximityD->cellEndD), numObjectsH->numAllMarkers, MaxVel, isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "Navier_Stokes");

    CopySortedToOriginal_NonInvasive_R4(fsiGeneralData->derivVelRhoD_old, sortedDerivVelRho,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R3(fsiGeneralData->derivTauXxYyZzD, sortedDerivTauXxYyZz,  //
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R3(fsiGeneralData->derivTauXyXzYzD, sortedDerivTauXyXzYz,  //
                                        markersProximityD->gridMarkerIndexD);
    sortedDerivVelRho.clear();
    sortedDerivTauXxYyZz.clear();  //
    sortedDerivTauXyXzYz.clear();  //
    cudaFree(isErrorD);
    free(isErrorH);
    density_initialization++;
    if (density_initialization >= paramsH->densityReinit)
        density_initialization = 0;
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::CalculateXSPH_velocity() {
    /* Calculate vel_XSPH */
    if (vel_XSPH_Sorted_D.size() != numObjectsH->numAllMarkers) {
        printf("vel_XSPH_Sorted_D.size() %zd numObjectsH->numAllMarkers %zd \n", vel_XSPH_Sorted_D.size(),
               numObjectsH->numAllMarkers);
        throw std::runtime_error(
            "Error! size error vel_XSPH_Sorted_D Thrown from "
            "CalculateXSPH_velocity!\n");
    }

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    //------------------------------------------------------------------------
    /* thread per particle */
    uint numThreads, numBlocks;
    computeGridSize((uint)numObjectsH->numAllMarkers, 128, numBlocks, numThreads);
    thrust::device_vector<Real4> sortedPosRad_old = sortedSphMarkersD->posRadD;
    thrust::fill(vel_XSPH_Sorted_D.begin(), vel_XSPH_Sorted_D.end(), mR3(0.0));

    /* Execute the kernel */
    CalcVel_XSPH_D<<<numBlocks, numThreads>>>(
        mR3CAST(vel_XSPH_Sorted_D), mR4CAST(sortedPosRad_old), mR4CAST(sortedSphMarkersD->posRadD),
        mR3CAST(sortedSphMarkersD->velMasD), mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(shift_r),
        U1CAST(markersProximityD->gridMarkerIndexD), U1CAST(markersProximityD->cellStartD),
        U1CAST(markersProximityD->cellEndD), numObjectsH->numAllMarkers, isErrorD);
    ChUtilsDevice::Sync_CheckError(isErrorH, isErrorD, "CalcVel_XSPH_D");

    CopySortedToOriginal_NonInvasive_R3(fsiGeneralData->vel_XSPH_D, vel_XSPH_Sorted_D,
                                        markersProximityD->gridMarkerIndexD);
    CopySortedToOriginal_NonInvasive_R4(sphMarkersD->posRadD, sortedSphMarkersD->posRadD,
                                        markersProximityD->gridMarkerIndexD);

    if (density_initialization % paramsH->densityReinit == 0)
        CopySortedToOriginal_NonInvasive_R4(sphMarkersD->rhoPresMuD, sortedSphMarkersD->rhoPresMuD,
                                            markersProximityD->gridMarkerIndexD);
    cudaFree(isErrorD);
    free(isErrorH);
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceExplicitSPH::AddGravityToFluid() {
    // add gravity to fluid markers
    /* Add outside forces. Don't add gravity to rigids, BCE, and boundaries, it is
     * added in ChSystem */
    Real3 totalFluidBodyForce3 = paramsH->bodyForce3 + paramsH->gravity;
    thrust::device_vector<Real4> bodyForceD(numObjectsH->numAllMarkers);
    thrust::fill(bodyForceD.begin(), bodyForceD.end(), mR4(totalFluidBodyForce3));
    thrust::transform(
        fsiGeneralData->derivVelRhoD_old.begin() + fsiGeneralData->referenceArray[0].x,
        fsiGeneralData->derivVelRhoD_old.begin() + fsiGeneralData->referenceArray[0].y, bodyForceD.begin(),
        fsiGeneralData->derivVelRhoD_old.begin() + fsiGeneralData->referenceArray[0].x, thrust::plus<Real4>());
    bodyForceD.clear();
}

}  // namespace fsi
}  // namespace chrono
//================================================================================================================================
