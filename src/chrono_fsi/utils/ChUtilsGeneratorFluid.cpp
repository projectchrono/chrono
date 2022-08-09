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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Utility class for generating fluid markers.
// =============================================================================

#include "chrono_fsi/utils/ChUtilsGeneratorFluid.h"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace utils {

// Kernel function
//// TODO: provide more options for the kernel function
Real W3h_Spline(Real d, Real h) {
    Real invh = 1.0 / h;
    Real q = fabs(d) * invh;
    if (q < 1) {
        return (0.25f * (INVPI * invh * invh * invh) * (cube(2 - q) - 4 * cube(1 - q)));
    }
    if (q < 2) {
        return (0.25f * (INVPI * invh * invh * invh) * cube(2 - q));
    }
    return 0;
}

// Particle mass calculator based on the initial spacing and density
Real massCalculator(Real Kernel_h, Real InitialSpacing, Real rho0) {
    int IDX = 10;
    Real sum_wij = 0;
    for (int i = -IDX; i <= IDX; i++)
        for (int j = -IDX; j <= IDX; j++)
            for (int k = -IDX; k <= IDX; k++) {
                Real3 pos = mR3(i, j, k) * InitialSpacing;
                Real W = W3h_Spline(length(pos), Kernel_h);
                sum_wij += W;
            }
    return rho0 / sum_wij;
}

// Particle number calculator based on the initial spacing and kernel length
Real IniNeiNum(Real Kernel_h, Real InitialSpacing) {
    int IDX = 10;
    int count = 0;
    for (int i = -IDX; i <= IDX; i++)
        for (int j = -IDX; j <= IDX; j++)
            for (int k = -IDX; k <= IDX; k++) {
                Real3 pos = mR3(i, j, k) * InitialSpacing;
                Real W = W3h_Spline(length(pos), Kernel_h);
                if (W > 0)
                    count++;
            }
    return count;
}

// Create fluid/granular SPH particles for the simulation.
int2 CreateFluidMarkers(std::shared_ptr<SphMarkerDataH> sphMarkersH,
                        std::shared_ptr<FsiGeneralData> fsiGeneralData,
                        std::shared_ptr<SimParams> paramsH) {
    /* Number of fluid particles */
    int num_FluidMarkers = 0;
    /* Number of boundary particles */
    int num_BoundaryMarkers = 0;
    srand(964);
    Real initSpace0 =
        paramsH->MULT_INITSPACE * paramsH->HSML; /* Initial separation of both fluid and boundary particles */
    int nFX =
        (int)ceil((paramsH->cMaxInit.x - paramsH->cMinInit.x) / (initSpace0)); /* Number of particles per x dimension */
    Real initSpaceX =
        (paramsH->cMaxInit.x - paramsH->cMinInit.x) / nFX; /* Spacing between center of particles in x dimension*/

    int nFY =
        (int)ceil((paramsH->cMaxInit.y - paramsH->cMinInit.y) / (initSpace0)); /* Number of particles per y dimension */
    Real initSpaceY =
        (paramsH->cMaxInit.y - paramsH->cMinInit.y) / nFY; /* Spacing between center of particles in y dimension*/

    int nFZ =
        (int)ceil((paramsH->cMaxInit.z - paramsH->cMinInit.z) / (initSpace0)); /* Number of particles per z dimension */
    Real initSpaceZ =
        (paramsH->cMaxInit.z - paramsH->cMinInit.z) / nFZ; /* Spacing between center of particles in z dimension*/

    printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ, (nFY - 1) * initSpaceY, initSpaceY);
    /* Mass of a small cube in the fluid = (dx*dy*dz) * density */
    paramsH->markerMass = (initSpaceX * initSpaceY * initSpaceZ) * paramsH->rho0;

    for (size_t i = 0; i < nFX; i++) {
        for (size_t j = 0; j < nFY; j++) {
            for (size_t k = 0; k < nFZ; k++) {
                Real3 posRad;
                //					printf("initSpace X, Y, Z %f %f %f \n",
                // initSpaceX, initSpaceY,
                // initSpaceZ);
                posRad = paramsH->cMinInit + mR3(i * initSpaceX, j * initSpaceY, k * initSpaceZ) +
                         mR3(.5 * initSpace0) /* + mR3(sphR) + initSpace * .05 * (Real(rand()) / RAND_MAX)*/
                    ;
                if ((posRad.x > paramsH->straightChannelBoundaryMin.x &&
                     posRad.x < paramsH->straightChannelBoundaryMax.x) &&
                    (posRad.y > paramsH->straightChannelBoundaryMin.y &&
                     posRad.y < paramsH->straightChannelBoundaryMax.y) &&
                    (posRad.z > paramsH->straightChannelBoundaryMin.z &&
                     posRad.z < paramsH->straightChannelBoundaryMax.z)) {
                    if (i < nFX) {
                        num_FluidMarkers++;
                        sphMarkersH->posRadH.push_back(mR4(posRad, initSpace0));
                        sphMarkersH->velMasH.push_back(mR3(0));
                        sphMarkersH->rhoPresMuH.push_back(mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, -1));
                    }
                } else {
                    //					num_BoundaryMarkers++;
                    //					mPosRadBoundary.push_back(posRad);
                    //					mVelMasBoundary.push_back(mR4(0, 0, 0,
                    // paramsH->markerMass));
                    //					mRhoPresMuBoundary.push_back(mR4(paramsH->rho0,
                    // paramsH->LARGE_PRES,
                    // paramsH->mu0,
                    // 0));
                }
            }
        }
    }
    int2 num_fluidOrBoundaryMarkers = mI2(num_FluidMarkers, num_BoundaryMarkers);
    // *** copy boundary markers to the end of the markers arrays
    sphMarkersH->posRadH.resize(num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
    sphMarkersH->velMasH.resize(num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
    sphMarkersH->rhoPresMuH.resize(num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);

    //    int numAllMarkers = num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y;
    return num_fluidOrBoundaryMarkers;
}

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono
