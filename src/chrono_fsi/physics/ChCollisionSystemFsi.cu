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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Base class for processing proximity in fsi system.
// =============================================================================

#include <thrust/sort.h>
#include "chrono_fsi/physics/ChCollisionSystemFsi.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {

// calcHashD :
// 1. Get particle index determined by the block and thread we are in.
// 2. From x, y, z position, determine which bin it is in.
// 3. Calculate hash from bin index.
// 4. Store hash and particle index associated with it.
__global__ void calcHashD(
    uint* gridMarkerHashD,   // gridMarkerHash Store particle hash here
    uint* gridMarkerIndexD,  // gridMarkerIndex Store particle index here
    Real4* posRad,           // posRad Vector containing the positions of all particles (SPH and BCE)
    const size_t numAllMarkers,  // Total number of particles (fluid + boundary)
    volatile bool* isErrorD) {
    /* Calculate the index of where the particle is stored in posRad. */
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numAllMarkers)
        return;

    Real3 p = mR3(posRad[index]);

    if (!(isfinite(p.x) && isfinite(p.y) && isfinite(p.z))) {
        printf(
            "Error! particle position is NAN: thrown from "
            "ChCollisionSystemFsi.cu, calcHashD !\n");
        *isErrorD = true;
        return;
    }

    /* Check particle is inside the domain. */
    Real3 boxCorner = paramsD.worldOrigin - mR3(40 * paramsD.HSML);
    if (p.x < boxCorner.x || p.y < boxCorner.y || p.z < boxCorner.z) {
        printf(
            "Out of Min Boundary, point %f %f %f, boundary min: %f %f %f. "
            "Thrown from ChCollisionSystemFsi.cu, calcHashD !\n",
            p.x, p.y, p.z, boxCorner.x, boxCorner.y, boxCorner.z);
        *isErrorD = true;
        return;
    }
    boxCorner = paramsD.worldOrigin + paramsD.boxDims + mR3(40 * paramsD.HSML);
    if (p.x > boxCorner.x || p.y > boxCorner.y || p.z > boxCorner.z) {
        printf(
            "Out of max Boundary, point %f %f %f, boundary max: %f %f %f. "
            "Thrown from ChCollisionSystemFsi.cu, calcHashD !\n",
            p.x, p.y, p.z, boxCorner.x, boxCorner.y, boxCorner.z);
        *isErrorD = true;
        return;
    }

    /* Get x,y,z bin index in grid */
    int3 gridPos = calcGridPos(p);
    /* Calculate a hash from the bin index */
    uint hash = calcGridHash(gridPos);

    /* Store grid hash */
    gridMarkerHashD[index] = hash;
    /* Store particle index associated to the hash we stored in gridMarkerHashD */
    gridMarkerIndexD[index] = index;
}

/**
 * @brief reorderDataAndFindCellStartD
 * @details See ChCollisionSystemFsi.cuh for more info
 */
__global__ void reorderDataAndFindCellStartD(uint* cellStartD,          // output: cell start index
                                             uint* cellEndD,            // output: cell end index
                                             Real4* sortedPosRadD,      // output: sorted positions
                                             Real3* sortedVelMasD,      // output: sorted velocities
                                             Real4* sortedRhoPreMuD,    // output: sorted density pressure
                                             Real3* sortedTauXxYyZzD,   // output: sorted total stress xxyyzz
                                             Real3* sortedTauXyXzYzD,   // output: sorted total stress xyzxyz
                                             Real3* tauXxYyZzD,         // input: original total stress xxyyzz
                                             Real3* tauXyXzYzD,         // input: original total stress xyzxyz
                                             uint* gridMarkerHashD,     // input: sorted grid hashes
                                             uint* gridMarkerIndexD,    // input: sorted particle indices
                                             Real4* posRadD,            // input: original position array
                                             Real3* velMasD,            // input: original velocity array
                                             Real4* rhoPresMuD,         // input: original density pressure
                                             const size_t numAllMarkers) {
    extern __shared__ uint sharedHash[];  // blockSize + 1 elements
    /* Get the particle index the current thread is supposed to be looking at. */
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    uint hash;
    /* handle case when no. of particles not multiple of block size */
    if (index < numAllMarkers) {
        hash = gridMarkerHashD[index];
        /* Load hash data into shared memory so that we can look at neighboring
         * particle's hash
         * value without loading two hash values per thread
         */
        sharedHash[threadIdx.x + 1] = hash;

        if (index > 0 && threadIdx.x == 0) {
            /* first thread in block must load neighbor particle hash */
            sharedHash[0] = gridMarkerHashD[index - 1];
        }
    }

    __syncthreads();

    if (index < numAllMarkers) {
        /* If this particle has a different cell index to the previous particle then
         * it must be
         * the first particle in the cell, so store the index of this particle in
         * the cell. As it
         * isn't the first particle, it must also be the cell end of the previous
         * particle's cell
         */
        if (index == 0 || hash != sharedHash[threadIdx.x]) {
            cellStartD[hash] = index;
            if (index > 0)
                cellEndD[sharedHash[threadIdx.x]] = index;
        }

        if (index == numAllMarkers - 1) {
            cellEndD[hash] = index + 1;
        }

        /* Now use the sorted index to reorder the pos and vel data */
        uint originalIndex = gridMarkerIndexD[index];  // map sorted to original
        Real3 posRad = mR3(posRadD[originalIndex]);  // macro does either global read or
                                                     // texture fetch
        Real3 velMas = velMasD[originalIndex];       // see particles_kernel.cuh
        Real4 rhoPreMu = rhoPresMuD[originalIndex];

        if (!(isfinite(posRad.x) && isfinite(posRad.y) && isfinite(posRad.z))) {
            printf(
                "Error! particle position is NAN: thrown from "
                "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
        }
        if (!(isfinite(velMas.x) && isfinite(velMas.y) && isfinite(velMas.z))) {
            printf(
                "Error! particle velocity is NAN: thrown from "
                "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
        }
        if (!(isfinite(rhoPreMu.x) && isfinite(rhoPreMu.y) && isfinite(rhoPreMu.z) && isfinite(rhoPreMu.w))) {
            printf(
                "Error! particle rhoPreMu is NAN: thrown from "
                "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
        }

        sortedPosRadD[index] = mR4(posRad, posRadD[originalIndex].w);
        sortedVelMasD[index] = velMas;
        sortedRhoPreMuD[index] = rhoPreMu;

        // For granular material
        if( paramsD.elastic_SPH ) {
            Real3 tauXxYyZz = tauXxYyZzD[originalIndex];  
            Real3 tauXyXzYz = tauXyXzYzD[originalIndex]; 
            if (!(isfinite(tauXxYyZz.x) && isfinite(tauXxYyZz.y) && isfinite(tauXxYyZz.z))) {  
                printf(
                    "Error! particle tauXxYyZz is NAN: thrown from "
                    "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
            }
            if (!(isfinite(tauXyXzYz.x) && isfinite(tauXyXzYz.y) && isfinite(tauXyXzYz.z))) { 
                printf(
                    "Error! particle tauXyXzYz is NAN: thrown from "
                    "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
            }
            sortedTauXxYyZzD[index] = tauXxYyZz;  
            sortedTauXyXzYzD[index] = tauXyXzYz; 
        }
    }
}

__global__ void findCellStartEndD(uint* cellStartD,         // output: cell start index
                                  uint* cellEndD,           // output: cell end index
                                  uint* gridMarkerHashD,    // input: sorted grid hashes
                                  uint* gridMarkerIndexD,   // input: sorted particle indices
                                  const size_t numAllMarkers) {
    extern __shared__ uint sharedHash[];  // blockSize + 1 elements
    /* Get the particle index the current thread is supposed to be looking at. */
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    uint hash;
    /* handle case when no. of particles not multiple of block size */
    if (index < numAllMarkers) {
        hash = gridMarkerHashD[index];
        /* Load hash data into shared memory so that we can look at neighboring
         * particle's hash
         * value without loading two hash values per thread
         */
        sharedHash[threadIdx.x + 1] = hash;

        if (index > 0 && threadIdx.x == 0) {
            /* first thread in block must load neighbor particle hash */
            sharedHash[0] = gridMarkerHashD[index - 1];
        }
    }

    __syncthreads();

    if (index < numAllMarkers) {
        /* If this particle has a different cell index to the previous particle then
         * it must be
         * the first particle in the cell, so store the index of this particle in
         * the cell. As it
         * isn't the first particle, it must also be the cell end of the previous
         * particle's cell
         */
        if (index == 0 || hash != sharedHash[threadIdx.x]) {
            cellStartD[hash] = index;
            if (index > 0)
                cellEndD[sharedHash[threadIdx.x]] = index;
        }

        if (index == numAllMarkers - 1) {
            cellEndD[hash] = index + 1;
        }
    }
}

__global__ void reorderDataD(uint* gridMarkerIndexD,     // input: sorted particle indices
                             uint* extendedActivityIdD,  // input: particles in an extended active sub-domain
                             uint* mapOriginalToSorted,  // input: original index to sorted index
                             Real4* sortedPosRadD,       // output: sorted positions
                             Real3* sortedVelMasD,       // output: sorted velocities
                             Real4* sortedRhoPreMuD,     // output: sorted density pressure
                             Real3* sortedTauXxYyZzD,    // output: sorted total stress xxyyzz
                             Real3* sortedTauXyXzYzD,    // output: sorted total stress xyzxyz
                             Real4* posRadD,             // input: original position array
                             Real3* velMasD,             // input: original velocity array
                             Real4* rhoPresMuD,          // input: original density pressure
                             Real3* tauXxYyZzD,          // input: original total stress xxyyzz
                             Real3* tauXyXzYzD,          // input: original total stress xyzxyz
                             const size_t numAllMarkers) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numAllMarkers)
        return;

    // Now use the sorted index to reorder the pos and vel data
    uint originalIndex = id;

    // no need to do anything if it is not an active particle
    uint activity = extendedActivityIdD[originalIndex];
    if(activity == 0)
        return; 

    // map original to sorted
    uint index = mapOriginalToSorted[originalIndex]; 

    Real3 posRad = mR3(posRadD[originalIndex]);
    Real3 velMas = velMasD[originalIndex];
    Real4 rhoPreMu = rhoPresMuD[originalIndex];

    if (!(isfinite(posRad.x) && isfinite(posRad.y) && isfinite(posRad.z))) {
        printf(
            "Error! particle position is NAN: thrown from "
            "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
    }
    if (!(isfinite(velMas.x) && isfinite(velMas.y) && isfinite(velMas.z))) {
        printf(
            "Error! particle velocity is NAN: thrown from "
            "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
    }
    if (!(isfinite(rhoPreMu.x) && isfinite(rhoPreMu.y) && isfinite(rhoPreMu.z) && isfinite(rhoPreMu.w))) {
        printf(
            "Error! particle rhoPreMu is NAN: thrown from "
            "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
    }

    sortedPosRadD[index] = mR4(posRad, posRadD[originalIndex].w);
    sortedVelMasD[index] = velMas;
    sortedRhoPreMuD[index] = rhoPreMu;

    // For granular material
    if( paramsD.elastic_SPH ) {
        Real3 tauXxYyZz = tauXxYyZzD[originalIndex];  
        Real3 tauXyXzYz = tauXyXzYzD[originalIndex]; 
        if (!(isfinite(tauXxYyZz.x) && isfinite(tauXxYyZz.y) && isfinite(tauXxYyZz.z))) {  
            printf(
                "Error! particle tauXxYyZz is NAN: thrown from "
                "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
        }
        if (!(isfinite(tauXyXzYz.x) && isfinite(tauXyXzYz.y) && isfinite(tauXyXzYz.z))) { 
            printf(
                "Error! particle tauXyXzYz is NAN: thrown from "
                "ChCollisionSystemFsi.cu, reorderDataAndFindCellStartD !\n");
        }
        sortedTauXxYyZzD[index] = tauXxYyZz;  
        sortedTauXyXzYzD[index] = tauXyXzYz; 
    }
}

__global__ void OriginalToSortedD(uint* mapOriginalToSorted,
                                  uint* gridMarkerIndex,
                                  const size_t numAllMarkers) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numAllMarkers)
        return;

    uint index = gridMarkerIndex[id];

    mapOriginalToSorted[index] = id;
}

//--------------------------------------------------------------------------------------------------------------------------------
ChCollisionSystemFsi::ChCollisionSystemFsi(std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,
                                           std::shared_ptr<ProximityDataD> otherMarkersProximityD,
                                           std::shared_ptr<FsiGeneralData> otherFsiGeneralData,
                                           std::shared_ptr<SimParams> otherParamsH,
                                           std::shared_ptr<ChCounters> otherNumObjects)
    : sortedSphMarkersD(otherSortedSphMarkersD),
      markersProximityD(otherMarkersProximityD),
      fsiGeneralData(otherFsiGeneralData),
      paramsH(otherParamsH),
      numObjectsH(otherNumObjects) {
    sphMarkersD = NULL;
}
ChCollisionSystemFsi::~ChCollisionSystemFsi() {}

//--------------------------------------------------------------------------------------------------------------------------------
void ChCollisionSystemFsi::Initialize() {
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(ChCounters));
}

//--------------------------------------------------------------------------------------------------------------------------------
void ChCollisionSystemFsi::calcHash() {
    if (!(markersProximityD->gridMarkerHashD.size() == numObjectsH->numAllMarkers &&
          markersProximityD->gridMarkerIndexD.size() == numObjectsH->numAllMarkers)) {
        printf(
            "mError! calcHash!, gridMarkerHashD.size() %zu "
            "gridMarkerIndexD.size() %zu numObjectsH->numAllMarkers %zu \n",
            markersProximityD->gridMarkerHashD.size(), markersProximityD->gridMarkerIndexD.size(),
            numObjectsH->numAllMarkers);
        throw std::runtime_error("Error! size error, calcHash!");
    }

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
    //------------------------------------------------------------------------
    /* Is there a need to optimize the number of threads used at once? */
    uint numThreads, numBlocks;
    computeGridSize((int)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);

    /* Execute Kernel */
    calcHashD<<<numBlocks, numThreads>>>(U1CAST(markersProximityD->gridMarkerHashD),
                                         U1CAST(markersProximityD->gridMarkerIndexD), 
                                         mR4CAST(sphMarkersD->posRadD),
                                         numObjectsH->numAllMarkers, isErrorD);

    /* Check for errors in kernel execution */
    cudaDeviceSynchronize();
    cudaCheckError();
    //------------------------------------------------------------------------
    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true) {
        throw std::runtime_error("Error! program crashed in  calcHashD!\n");
    }
    cudaFree(isErrorD);
    free(isErrorH);
}

void ChCollisionSystemFsi::ResetCellSize(int s) {
    markersProximityD->cellStartD.resize(s);
    markersProximityD->cellEndD.resize(s);
}

void ChCollisionSystemFsi::reorderDataAndFindCellStart() {
    int3 cellsDim = paramsH->gridSize;
    int numCells = cellsDim.x * cellsDim.y * cellsDim.z;
    if (!(markersProximityD->cellStartD.size() == numCells && markersProximityD->cellEndD.size() == numCells)) {
        throw std::runtime_error("Error! size error, reorderDataAndFindCellStart!\n");
    }

    thrust::fill(markersProximityD->cellStartD.begin(), markersProximityD->cellStartD.end(), 0);
    thrust::fill(markersProximityD->cellEndD.begin(), markersProximityD->cellEndD.end(), 0);

    uint numThreads, numBlocks;
    computeGridSize((uint)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);  // 256 is blockSize

    uint smemSize = sizeof(uint) * (numThreads + 1);
    // Find the start index and the end index of the sorted array in each cell
    findCellStartEndD<<<numBlocks, numThreads, smemSize>>>(
        U1CAST(markersProximityD->cellStartD), U1CAST(markersProximityD->cellEndD),          
        U1CAST(markersProximityD->gridMarkerHashD), U1CAST(markersProximityD->gridMarkerIndexD),
        numObjectsH->numAllMarkers);
    cudaDeviceSynchronize();
    cudaCheckError();

    // Launch a kernel to find the location of original particles in the sorted arrays.
    // This is faster than using thrust::sort_by_key()
    OriginalToSortedD<<<numBlocks, numThreads>>>(
        U1CAST(markersProximityD->mapOriginalToSorted),
        U1CAST(markersProximityD->gridMarkerIndexD), numObjectsH->numAllMarkers);

    // Reorder the arrays according to the sorted index of all particles
    reorderDataD<<<numBlocks, numThreads>>>(
        U1CAST(markersProximityD->gridMarkerIndexD),
        U1CAST(fsiGeneralData->extendedActivityIdD),
        U1CAST(markersProximityD->mapOriginalToSorted),
        mR4CAST(sortedSphMarkersD->posRadD), mR3CAST(sortedSphMarkersD->velMasD), 
        mR4CAST(sortedSphMarkersD->rhoPresMuD), mR3CAST(sortedSphMarkersD->tauXxYyZzD), 
        mR3CAST(sortedSphMarkersD->tauXyXzYzD), mR4CAST(sphMarkersD->posRadD), 
        mR3CAST(sphMarkersD->velMasD), mR4CAST(sphMarkersD->rhoPresMuD), 
        mR3CAST(sphMarkersD->tauXxYyZzD), mR3CAST(sphMarkersD->tauXyXzYzD), 
        numObjectsH->numAllMarkers);
    cudaDeviceSynchronize();
    cudaCheckError();
}

void ChCollisionSystemFsi::ArrangeData(std::shared_ptr<SphMarkerDataD> otherSphMarkersD) {
    sphMarkersD = otherSphMarkersD;
    int3 cellsDim = paramsH->gridSize;
    int numCells = cellsDim.x * cellsDim.y * cellsDim.z;
    ResetCellSize(numCells);
    calcHash();
    thrust::sort_by_key(markersProximityD->gridMarkerHashD.begin(), markersProximityD->gridMarkerHashD.end(),
                        markersProximityD->gridMarkerIndexD.begin());
    reorderDataAndFindCellStart();
}

}  // end namespace fsi
}  // end namespace chrono
