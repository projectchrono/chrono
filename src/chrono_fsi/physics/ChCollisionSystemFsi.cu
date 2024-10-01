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
__global__ void calcHashD(uint* gridMarkerHashD,   // gridMarkerHash Store particle hash here
                          uint* gridMarkerIndexD,  // gridMarkerIndex Store particle index here
                          const Real4* posRad,  // posRad Vector containing the positions of all particles (SPH and BCE)
                          volatile bool* isErrorD) {
    // Calculate the index of where the particle is stored in posRad.
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= numObjectsD.numAllMarkers)
        return;

    Real3 p = mR3(posRad[index]);

    if (!(isfinite(p.x) && isfinite(p.y) && isfinite(p.z))) {
        printf(
            "Error! particle position is NAN: thrown from "
            "ChCollisionSystemFsi.cu, calcHashD !\n");
        *isErrorD = true;
        return;
    }

    // Check particle is inside the domain.
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

    // Get x,y,z bin index in grid
    int3 gridPos = calcGridPos(p);
    // Calculate a hash from the bin index
    uint hash = calcGridHash(gridPos);
    // Store grid hash
    // grid hash is a scalar cell ID
    gridMarkerHashD[index] = hash;
    // Store particle index associated to the hash we stored in gridMarkerHashD
    gridMarkerIndexD[index] = index;
}
// ------------------------------------------------------------------------------
__global__ void findCellStartEndD(uint* cellStartD,       // output: cell start index
                                  uint* cellEndD,         // output: cell end index
                                  uint* gridMarkerHashD,  // input: sorted grid hashes
                                  uint* gridMarkerIndexD  // input: sorted particle indices
) {
    extern __shared__ uint sharedHash[];  // blockSize + 1 elements
    // Get the particle index the current thread is supposed to be looking at.
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    uint hash;
    // handle case when no. of particles not multiple of block size
    if (index < numObjectsD.numAllMarkers) {
        hash = gridMarkerHashD[index];
        // Load hash data into shared memory so that we can look at neighboring
        // particle's hash value without loading two hash values per thread
        sharedHash[threadIdx.x + 1] = hash;

        // first thread in block must load neighbor particle hash
        if (index > 0 && threadIdx.x == 0)
            sharedHash[0] = gridMarkerHashD[index - 1];
    }

    __syncthreads();

    if (index < numObjectsD.numAllMarkers) {
        // If this particle has a different cell index to the previous
        // particle then it must be the first particle in the cell,
        // so store the index of this particle in the cell. As it
        // isn't the first particle, it must also be the cell end of
        // the previous particle's cell.
        if (index == 0 || hash != sharedHash[threadIdx.x]) {
            cellStartD[hash] = index;
            if (index > 0)
                cellEndD[sharedHash[threadIdx.x]] = index;
        }

        if (index == numObjectsD.numAllMarkers - 1)
            cellEndD[hash] = index + 1;
    }
}
// ------------------------------------------------------------------------------
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
                             Real3* tauXyXzYzD           // input: original total stress xyzxyz
) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numObjectsD.numAllMarkers)
        return;

    // Now use the sorted index to reorder the pos and vel data
    uint originalIndex = id;

    // no need to do anything if it is not an active particle
    // uint activity = extendedActivityIdD[originalIndex];
    // if (activity == 0)
    //     return;

    // map original to sorted
    uint index = mapOriginalToSorted[originalIndex];

    Real3 posRad = mR3(posRadD[originalIndex]);
    Real3 velMas = velMasD[originalIndex];
    Real4 rhoPreMu = rhoPresMuD[originalIndex];

    if (!(isfinite(posRad.x) && isfinite(posRad.y) && isfinite(posRad.z))) {
        printf(
            "Error! particle position is NAN: thrown from "
            "ChCollisionSystemFsi.cu, reorderDataD !\n");
    }
    if (!(isfinite(velMas.x) && isfinite(velMas.y) && isfinite(velMas.z))) {
        printf(
            "Error! particle velocity is NAN: thrown from "
            "ChCollisionSystemFsi.cu, reorderDataD !\n");
    }
    if (!(isfinite(rhoPreMu.x) && isfinite(rhoPreMu.y) && isfinite(rhoPreMu.z) && isfinite(rhoPreMu.w))) {
        printf(
            "Error! particle rhoPreMu is NAN: thrown from "
            "ChCollisionSystemFsi.cu, reorderDataD !\n");
    }

    sortedPosRadD[index] = mR4(posRad, posRadD[originalIndex].w);
    sortedVelMasD[index] = velMas;
    sortedRhoPreMuD[index] = rhoPreMu;

    // For granular material
    if (paramsD.elastic_SPH) {
        Real3 tauXxYyZz = tauXxYyZzD[originalIndex];
        Real3 tauXyXzYz = tauXyXzYzD[originalIndex];
        if (!(isfinite(tauXxYyZz.x) && isfinite(tauXxYyZz.y) && isfinite(tauXxYyZz.z))) {
            printf(
                "Error! particle tauXxYyZz is NAN: thrown from "
                "ChCollisionSystemFsi.cu, reorderDataD !\n");
        }
        if (!(isfinite(tauXyXzYz.x) && isfinite(tauXyXzYz.y) && isfinite(tauXyXzYz.z))) {
            printf(
                "Error! particle tauXyXzYz is NAN: thrown from "
                "ChCollisionSystemFsi.cu, reorderDataD !\n");
        }
        sortedTauXxYyZzD[index] = tauXxYyZz;
        sortedTauXyXzYzD[index] = tauXyXzYz;
    }
}
// ------------------------------------------------------------------------------
__global__ void OriginalToSortedD(uint* mapOriginalToSorted, uint* gridMarkerIndex) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numObjectsD.numAllMarkers)
        return;

    uint index = gridMarkerIndex[id];

    mapOriginalToSorted[index] = id;
}
// ------------------------------------------------------------------------------
ChCollisionSystemFsi::ChCollisionSystemFsi(std::shared_ptr<SphMarkerDataD> sortedSphMarkers_D,
                                           std::shared_ptr<ProximityDataD> markersProximity_D,
                                           std::shared_ptr<FsiData> fsiData,
                                           std::shared_ptr<SimParams> paramsH,
                                           std::shared_ptr<ChCounters> numObjects)
    : ChFsiBase(paramsH, numObjects),
      m_sortedSphMarkersD(sortedSphMarkers_D),
      m_markersProximityD(markersProximity_D),
      m_fsiData(fsiData),
      m_sphMarkersD(nullptr) {}

ChCollisionSystemFsi::~ChCollisionSystemFsi() {}
// ------------------------------------------------------------------------------
void ChCollisionSystemFsi::Initialize() {
    cudaMemcpyToSymbolAsync(paramsD, paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(numObjectsD, numObjectsH.get(), sizeof(ChCounters));
}

// ------------------------------------------------------------------------------

void ChCollisionSystemFsi::ArrangeData(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    m_sphMarkersD = sphMarkersD;
    int3 cellsDim = paramsH->gridSize;
    int numCells = cellsDim.x * cellsDim.y * cellsDim.z;

    uint numThreads, numBlocks;
    computeGridSize((uint)numObjectsH->numAllMarkers, 256, numBlocks, numThreads);

    // Reset cell size
    m_markersProximityD->cellStartD.resize(numCells);
    m_markersProximityD->cellEndD.resize(numCells);

    // =========================================================================================================
    // Calculate Hash
    // =========================================================================================================
    if (!(m_markersProximityD->gridMarkerHashD.size() == numObjectsH->numAllMarkers &&
          m_markersProximityD->gridMarkerIndexD.size() == numObjectsH->numAllMarkers)) {
        printf(
            "mError! calcHash!, gridMarkerHashD.size() %zu "
            "gridMarkerIndexD.size() %zu numObjectsH->numAllMarkers %zu \n",
            m_markersProximityD->gridMarkerHashD.size(), m_markersProximityD->gridMarkerIndexD.size(),
            numObjectsH->numAllMarkers);
        throw std::runtime_error("Error! size error, calcHash!");
    }

    bool *isErrorH, *isErrorD;
    isErrorH = (bool*)malloc(sizeof(bool));
    cudaMalloc((void**)&isErrorD, sizeof(bool));
    *isErrorH = false;
    cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);

    // Execute Kernel
    calcHashD<<<numBlocks, numThreads>>>(U1CAST(m_markersProximityD->gridMarkerHashD),
                                         U1CAST(m_markersProximityD->gridMarkerIndexD), mR4CAST(m_sphMarkersD->posRadD),
                                         isErrorD);

    cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
    if (*isErrorH == true)
        throw std::runtime_error("Error! program crashed in  calcHashD!\n");
    cudaFree(isErrorD);
    free(isErrorH);

    // =========================================================================================================
    // Sort Particles based on Hash
    // =========================================================================================================
    thrust::sort_by_key(m_markersProximityD->gridMarkerHashD.begin(), m_markersProximityD->gridMarkerHashD.end(),
                        m_markersProximityD->gridMarkerIndexD.begin());

    // =========================================================================================================
    // Find the start index and the end index of the sorted array in each cell
    // =========================================================================================================
    // Reset proximity cell data
    if (!(m_markersProximityD->cellStartD.size() == numCells && m_markersProximityD->cellEndD.size() == numCells)) {
        throw std::runtime_error("Error! size error, ArrangeData!\n");
    }

    thrust::fill(m_markersProximityD->cellStartD.begin(), m_markersProximityD->cellStartD.end(), 0);
    thrust::fill(m_markersProximityD->cellEndD.begin(), m_markersProximityD->cellEndD.end(), 0);

    uint smemSize = sizeof(uint) * (numThreads + 1);
    findCellStartEndD<<<numBlocks, numThreads, smemSize>>>(
        U1CAST(m_markersProximityD->cellStartD), U1CAST(m_markersProximityD->cellEndD),
        U1CAST(m_markersProximityD->gridMarkerHashD), U1CAST(m_markersProximityD->gridMarkerIndexD));

    // =========================================================================================================
    // Launch a kernel to find the location of original particles in the sorted arrays.
    // This is faster than using thrust::sort_by_key()
    // =========================================================================================================
    OriginalToSortedD<<<numBlocks, numThreads>>>(U1CAST(m_markersProximityD->mapOriginalToSorted),
                                                 U1CAST(m_markersProximityD->gridMarkerIndexD));

    // =========================================================================================================
    // Reorder the arrays according to the sorted index of all particles
    // =========================================================================================================
    reorderDataD<<<numBlocks, numThreads>>>(
        U1CAST(m_markersProximityD->gridMarkerIndexD), U1CAST(m_fsiData->extendedActivityIdD),
        U1CAST(m_markersProximityD->mapOriginalToSorted), mR4CAST(m_sortedSphMarkersD->posRadD),
        mR3CAST(m_sortedSphMarkersD->velMasD), mR4CAST(m_sortedSphMarkersD->rhoPresMuD),
        mR3CAST(m_sortedSphMarkersD->tauXxYyZzD), mR3CAST(m_sortedSphMarkersD->tauXyXzYzD),
        mR4CAST(m_sphMarkersD->posRadD), mR3CAST(m_sphMarkersD->velMasD), mR4CAST(m_sphMarkersD->rhoPresMuD),
        mR3CAST(m_sphMarkersD->tauXxYyZzD), mR3CAST(m_sphMarkersD->tauXyXzYzD));

    cudaDeviceSynchronize();

    cudaCheckError();
}

}  // end namespace fsi
}  // end namespace chrono
