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
#include <fstream>
#include "chrono_fsi/sph/physics/CollisionSystem.cuh"
#include "chrono_fsi/sph/physics/SphGeneral.cuh"
#include "chrono_fsi/sph/utils/UtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

// Create the active list
// Writes only if active - thus active list has all active partices at the front
// The index's are the index of the original particle arrangement
// After the active particles, random values that weere initialized are stored
__global__ void fillActiveListD(const uint* __restrict__ prefixSum,
                                const int32_t* __restrict__ extendedActivityIdD,
                                uint* __restrict__ activeListD,
                                uint numAllMarkers) {
    uint tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= numAllMarkers)
        return;

    // Check if the value is 1 (active)
    if (extendedActivityIdD[tid] == 1) {
        uint writePos = prefixSum[tid];  // an integer in [0..(numActive-1)]
        activeListD[writePos] = tid;
    }
}

// calcHashD :
// 1. Get particle index determined by the block and thread we are in.
// 2. From x, y, z position, determine which bin it is in.
// 3. Calculate hash from bin index.
// 4. Store hash and particle index associated with it.
// Again only the active particles are stored upfront in gridMarkerHashD and gridMarkerIndexD
__global__ void calcHashD(uint* gridMarkerHashD,    // gridMarkerHash Store particle hash here
                          uint* gridMarkerIndexD,   // gridMarkerIndex Store particle index here
                          const uint* activeListD,  // active list
                          const Real4* posRad,      // positions of all particles (SPH and BCE)
                          uint numActive,           // number of active particles
                          volatile bool* error_flag) {
    // Calculate the index of where the particle is stored in posRad.
    uint globalIndex = blockIdx.x * blockDim.x + threadIdx.x;
    if (globalIndex >= numActive)
        return;

    uint index = activeListD[globalIndex];
    Real3 p = mR3(posRad[index]);

    if (!IsFinite(p)) {
        printf("[calcHashD] index %d position is NaN\n", index);
        *error_flag = true;
        return;
    }

    // Check particle is inside the domain.
    Real3 boxCorner = paramsD.worldOrigin - mR3(40 * paramsD.h);
    if (p.x < boxCorner.x || p.y < boxCorner.y || p.z < boxCorner.z) {
        printf("[calcHashD] index %u (%f %f %f) out of min boundary (%f %f %f)\n",  //
               index, p.x, p.y, p.z, boxCorner.x, boxCorner.y, boxCorner.z);
        *error_flag = true;
        return;
    }
    boxCorner = paramsD.worldOrigin + paramsD.boxDims + mR3(40 * paramsD.h);
    if (p.x > boxCorner.x || p.y > boxCorner.y || p.z > boxCorner.z) {
        printf("[calcHashD] index %u (%f %f %f) out of max boundary (%f %f %f)\n",  //
               index, p.x, p.y, p.z, boxCorner.x, boxCorner.y, boxCorner.z);
        *error_flag = true;
        return;
    }

    // Get x,y,z bin index in grid
    int3 gridPos = calcGridPos(p);
    // Calculate a hash from the bin index
    uint hash = calcGridHash(gridPos);
    // Store grid hash
    // grid hash is a scalar cell ID
    gridMarkerHashD[globalIndex] = hash;
    // Store particle index associated to the hash we stored in gridMarkerHashD
    gridMarkerIndexD[globalIndex] = index;
}
// ------------------------------------------------------------------------------
__global__ void findCellStartEndD(uint* cellStartD,        // output: cell start index
                                  uint* cellEndD,          // output: cell end index
                                  uint* gridMarkerHashD,   // input: sorted grid hashes
                                  uint* gridMarkerIndexD,  // input: sorted particle indices
                                  uint numActive) {
    extern __shared__ uint sharedHash[];  // blockSize + 1 elements
    // Get the particle index the current thread is supposed to be looking at.
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    uint hash;

    if (index >= numActive)
        return;

    hash = gridMarkerHashD[index];
    // Load hash data into shared memory so that we can look at neighboring
    // particle's hash value without loading two hash values per thread
    sharedHash[threadIdx.x + 1] = hash;

    // first thread in block must load neighbor particle hash
    if (index > 0 && threadIdx.x == 0)
        sharedHash[0] = gridMarkerHashD[index - 1];

    __syncthreads();

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

    if (index == numActive - 1)
        cellEndD[hash] = index + 1;
}
// ------------------------------------------------------------------------------
__global__ void reorderDataD(const uint* __restrict__ gridMarkerIndexD,
                             Real4* __restrict__ sortedPosRadD,
                             Real3* __restrict__ sortedVelMasD,
                             Real4* __restrict__ sortedRhoPreMuD,
                             Real3* __restrict__ sortedTauXxYyZzD,
                             Real3* __restrict__ sortedTauXyXzYzD,
                             int32_t* __restrict__ activityIdentifierSortedD,
                             const Real4* __restrict__ posRadD,
                             const Real3* __restrict__ velMasD,
                             const Real4* __restrict__ rhoPresMuD,
                             const Real3* __restrict__ tauXxYyZzD,
                             const Real3* __restrict__ tauXyXzYzD,
                             const int32_t* __restrict__ activityIdentifierOriginalD,
                             const uint numActive) {
    uint tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= numActive)
        return;

    uint originalIndex = gridMarkerIndexD[tid];

    // Read from original arrays
    Real4 posRadVal = posRadD[originalIndex];
    Real3 velMasVal = velMasD[originalIndex];
    Real4 rhoPreMuVal = rhoPresMuD[originalIndex];
    int32_t activityIdentifierVal = activityIdentifierOriginalD[originalIndex];

    if (!IsFinite(mR3(posRadVal))) {
        printf("Error! reorderDataD_ActiveOnly: posRad is NAN at original index %u\n", originalIndex);
    }

    // Write to sorted arrays at index 'tid'
    sortedPosRadD[tid] = posRadVal;
    sortedVelMasD[tid] = velMasVal;
    sortedRhoPreMuD[tid] = rhoPreMuVal;
    activityIdentifierSortedD[tid] = activityIdentifierVal;

    // For elastic SPH or granular
    if (paramsD.elastic_SPH) {
        Real3 tauXxYyZzVal = tauXxYyZzD[originalIndex];
        Real3 tauXyXzYzVal = tauXyXzYzD[originalIndex];

        if (!IsFinite(tauXxYyZzVal)) {
            printf("Error! reorderDataD_ActiveOnly: tauXxYyZz is NAN at original index %u\n", originalIndex);
        }

        sortedTauXxYyZzD[tid] = tauXxYyZzVal;
        sortedTauXyXzYzD[tid] = tauXyXzYzVal;
    }
}

// ------------------------------------------------------------------------------
__global__ void OriginalToSortedD(uint* mapOriginalToSorted, uint* gridMarkerIndex, uint numActive) {
    uint id = blockIdx.x * blockDim.x + threadIdx.x;
    if (id >= numActive)
        return;

    uint index = gridMarkerIndex[id];

    mapOriginalToSorted[index] = id;
}
// ------------------------------------------------------------------------------

CollisionSystem::CollisionSystem(FsiDataManager& data_mgr) : m_data_mgr(data_mgr), m_sphMarkersD(nullptr) {}

CollisionSystem::~CollisionSystem() {}
// ------------------------------------------------------------------------------
void CollisionSystem::Initialize() {
    cudaMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(ChFsiParamsSPH));
    cudaMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));
}

// ------------------------------------------------------------------------------

void CollisionSystem::ArrangeData(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    bool* error_flagD;
    cudaMallocErrorFlag(error_flagD);
    cudaResetErrorFlag(error_flagD);

    m_sphMarkersD = sphMarkersD;

    //=========================================================================================================
    // Create active list where all active particles are at the front of the array
    //=========================================================================================================

    uint numThreads, numBlocks;
    computeGridSize((uint)m_data_mgr.countersH->numAllMarkers, 1024, numBlocks, numThreads);

    fillActiveListD<<<numBlocks, numThreads>>>(
        U1CAST(m_data_mgr.prefixSumExtendedActivityIdD), INT_32CAST(m_data_mgr.extendedActivityIdentifierOriginalD),
        U1CAST(m_data_mgr.activeListD), (uint)m_data_mgr.countersH->numAllMarkers);
    cudaDeviceSynchronize();
    cudaCheckErrorFlag(error_flagD, "fillActiveListD");

    // Reset cell size
    int3 cellsDim = m_data_mgr.paramsH->gridSize;
    int numCells = cellsDim.x * cellsDim.y * cellsDim.z;
    m_data_mgr.markersProximity_D->cellStartD.resize(numCells);
    m_data_mgr.markersProximity_D->cellEndD.resize(numCells);

    // =========================================================================================================
    // Calculate Hash
    // =========================================================================================================
    // Now only need to launch active particles
    computeGridSize((uint)m_data_mgr.countersH->numExtendedParticles, 1024, numBlocks, numThreads);
    // Execute Kernel
    calcHashD<<<numBlocks, numThreads>>>(U1CAST(m_data_mgr.markersProximity_D->gridMarkerHashD),
                                         U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD),
                                         U1CAST(m_data_mgr.activeListD), mR4CAST(m_sphMarkersD->posRadD),
                                         m_data_mgr.countersH->numExtendedParticles, error_flagD);
    cudaCheckErrorFlag(error_flagD, "calcHashD");

    // =========================================================================================================
    // Sort Particles based on Hash
    // =========================================================================================================
    thrust::sort_by_key(
        m_data_mgr.markersProximity_D->gridMarkerHashD.begin(),
        m_data_mgr.markersProximity_D->gridMarkerHashD.begin() + m_data_mgr.countersH->numExtendedParticles,
        m_data_mgr.markersProximity_D->gridMarkerIndexD.begin());

    // =========================================================================================================
    // Find the start index and the end index of the sorted array in each cell
    // =========================================================================================================

    thrust::fill(m_data_mgr.markersProximity_D->cellStartD.begin(), m_data_mgr.markersProximity_D->cellStartD.end(), 0);
    thrust::fill(m_data_mgr.markersProximity_D->cellEndD.begin(), m_data_mgr.markersProximity_D->cellEndD.end(), 0);

    // TODO - Check if 256 is optimal here
    computeGridSize((uint)m_data_mgr.countersH->numExtendedParticles, 256, numBlocks, numThreads);
    uint smemSize = sizeof(uint) * (numThreads + 1);
    findCellStartEndD<<<numBlocks, numThreads, smemSize>>>(
        U1CAST(m_data_mgr.markersProximity_D->cellStartD), U1CAST(m_data_mgr.markersProximity_D->cellEndD),
        U1CAST(m_data_mgr.markersProximity_D->gridMarkerHashD), U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD),
        m_data_mgr.countersH->numExtendedParticles);

    // =========================================================================================================
    // Launch a kernel to find the location of original particles in the sorted arrays.
    // This is faster than using thrust::sort_by_key()
    // =========================================================================================================
    computeGridSize((uint)m_data_mgr.countersH->numExtendedParticles, 1024, numBlocks, numThreads);
    OriginalToSortedD<<<numBlocks, numThreads>>>(U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted),
                                                 U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD),
                                                 m_data_mgr.countersH->numExtendedParticles);

    // =========================================================================================================
    // Reorder the arrays according to the sorted index of all particles
    // =========================================================================================================
    computeGridSize((uint)m_data_mgr.countersH->numExtendedParticles, 1024, numBlocks, numThreads);
    reorderDataD<<<numBlocks, numThreads>>>(
        U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD), mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD),
        mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD), mR4CAST(m_data_mgr.sortedSphMarkers2_D->rhoPresMuD),
        mR3CAST(m_data_mgr.sortedSphMarkers2_D->tauXxYyZzD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->tauXyXzYzD),
        INT_32CAST(m_data_mgr.activityIdentifierSortedD), mR4CAST(m_sphMarkersD->posRadD),
        mR3CAST(m_sphMarkersD->velMasD), mR4CAST(m_sphMarkersD->rhoPresMuD), mR3CAST(m_sphMarkersD->tauXxYyZzD),
        mR3CAST(m_sphMarkersD->tauXyXzYzD), INT_32CAST(m_data_mgr.activityIdentifierOriginalD),
        m_data_mgr.countersH->numExtendedParticles);

    cudaDeviceSynchronize();
    cudaCheckError();

    cudaFreeErrorFlag(error_flagD);
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
