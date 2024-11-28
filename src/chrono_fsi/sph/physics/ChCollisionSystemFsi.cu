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

#include "chrono_fsi/sph/physics/ChCollisionSystemFsi.cuh"
#include "chrono_fsi/sph/physics/ChSphGeneral.cuh"
#include "chrono_fsi/sph/utils/ChUtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

// calcHashD :
// 1. Get particle index determined by the block and thread we are in.
// 2. From x, y, z position, determine which bin it is in.
// 3. Calculate hash from bin index.
// 4. Store hash and particle index associated with it.
__global__ void calcHashD(uint* gridMarkerHashD,   // gridMarkerHash Store particle hash here
                          uint* gridMarkerIndexD,  // gridMarkerIndex Store particle index here
                          const Real4* posRad,     // positions of all particles (SPH and BCE)
                          volatile bool* error_flag) {
    // Calculate the index of where the particle is stored in posRad.
    uint index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= countersD.numAllMarkers)
        return;

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
    if (index < countersD.numAllMarkers) {
        hash = gridMarkerHashD[index];
        // Load hash data into shared memory so that we can look at neighboring
        // particle's hash value without loading two hash values per thread
        sharedHash[threadIdx.x + 1] = hash;

        // first thread in block must load neighbor particle hash
        if (index > 0 && threadIdx.x == 0)
            sharedHash[0] = gridMarkerHashD[index - 1];
    }

    __syncthreads();

    if (index < countersD.numAllMarkers) {
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

        if (index == countersD.numAllMarkers - 1)
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
    if (id >= countersD.numAllMarkers)
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

    if (!IsFinite(posRad)) {
        printf(
            "Error! particle position is NAN: thrown from "
            "ChCollisionSystemFsi.cu, reorderDataD !\n");
    }
    if (!IsFinite(velMas)) {
        printf(
            "Error! particle velocity is NAN: thrown from "
            "ChCollisionSystemFsi.cu, reorderDataD !\n");
    }
    if (!IsFinite(rhoPreMu)) {
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
        if (!IsFinite(tauXxYyZz)) {
            printf(
                "Error! particle tauXxYyZz is NAN: thrown from "
                "ChCollisionSystemFsi.cu, reorderDataD !\n");
        }
        if (!IsFinite(tauXyXzYz)) {
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
    if (id >= countersD.numAllMarkers)
        return;

    uint index = gridMarkerIndex[id];

    mapOriginalToSorted[index] = id;
}
// ------------------------------------------------------------------------------
ChCollisionSystemFsi::ChCollisionSystemFsi(FsiDataManager& data_mgr) : m_data_mgr(data_mgr), m_sphMarkersD(nullptr) {}

ChCollisionSystemFsi::~ChCollisionSystemFsi() {}
// ------------------------------------------------------------------------------
void ChCollisionSystemFsi::Initialize() {
    cudaMemcpyToSymbolAsync(paramsD, m_data_mgr.paramsH.get(), sizeof(SimParams));
    cudaMemcpyToSymbolAsync(countersD, m_data_mgr.countersH.get(), sizeof(Counters));
}

// ------------------------------------------------------------------------------

void ChCollisionSystemFsi::ArrangeData(std::shared_ptr<SphMarkerDataD> sphMarkersD) {
    bool* error_flagD;
    cudaMallocErrorFlag(error_flagD);
    cudaResetErrorFlag(error_flagD);

    m_sphMarkersD = sphMarkersD;
    int3 cellsDim = m_data_mgr.paramsH->gridSize;
    int numCells = cellsDim.x * cellsDim.y * cellsDim.z;

    uint numThreads, numBlocks;
    computeGridSize((uint)m_data_mgr.countersH->numAllMarkers, 256, numBlocks, numThreads);

    // Reset cell size
    m_data_mgr.markersProximity_D->cellStartD.resize(numCells);
    m_data_mgr.markersProximity_D->cellEndD.resize(numCells);

    // =========================================================================================================
    // Calculate Hash
    // =========================================================================================================
    if (!(m_data_mgr.markersProximity_D->gridMarkerHashD.size() == m_data_mgr.countersH->numAllMarkers &&
          m_data_mgr.markersProximity_D->gridMarkerIndexD.size() == m_data_mgr.countersH->numAllMarkers)) {
        printf("[calcHashD] marker hash size: %u | marker index size: %u\n            num markers: %u\n",  //
               (uint)m_data_mgr.markersProximity_D->gridMarkerHashD.size(),                                //
               (uint)m_data_mgr.markersProximity_D->gridMarkerIndexD.size(),                               //
               (uint)m_data_mgr.countersH->numAllMarkers);                                                 //
        throw std::runtime_error("Error! size error, calcHash!");
    }

    // Execute Kernel
    calcHashD<<<numBlocks, numThreads>>>(U1CAST(m_data_mgr.markersProximity_D->gridMarkerHashD),
                                         U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD),
                                         mR4CAST(m_sphMarkersD->posRadD), error_flagD);
    cudaCheckErrorFlag(error_flagD, "calcHashD");

    // =========================================================================================================
    // Sort Particles based on Hash
    // =========================================================================================================
    thrust::sort_by_key(m_data_mgr.markersProximity_D->gridMarkerHashD.begin(),
                        m_data_mgr.markersProximity_D->gridMarkerHashD.end(),
                        m_data_mgr.markersProximity_D->gridMarkerIndexD.begin());

    // =========================================================================================================
    // Find the start index and the end index of the sorted array in each cell
    // =========================================================================================================
    // Reset proximity cell data
    if (!(m_data_mgr.markersProximity_D->cellStartD.size() == numCells &&
          m_data_mgr.markersProximity_D->cellEndD.size() == numCells)) {
        throw std::runtime_error("Error! size error, ArrangeData!\n");
    }

    thrust::fill(m_data_mgr.markersProximity_D->cellStartD.begin(), m_data_mgr.markersProximity_D->cellStartD.end(), 0);
    thrust::fill(m_data_mgr.markersProximity_D->cellEndD.begin(), m_data_mgr.markersProximity_D->cellEndD.end(), 0);

    uint smemSize = sizeof(uint) * (numThreads + 1);
    findCellStartEndD<<<numBlocks, numThreads, smemSize>>>(U1CAST(m_data_mgr.markersProximity_D->cellStartD),
                                                           U1CAST(m_data_mgr.markersProximity_D->cellEndD),
                                                           U1CAST(m_data_mgr.markersProximity_D->gridMarkerHashD),
                                                           U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD));

    // =========================================================================================================
    // Launch a kernel to find the location of original particles in the sorted arrays.
    // This is faster than using thrust::sort_by_key()
    // =========================================================================================================
    OriginalToSortedD<<<numBlocks, numThreads>>>(U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted),
                                                 U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD));

    // =========================================================================================================
    // Reorder the arrays according to the sorted index of all particles
    // =========================================================================================================
    reorderDataD<<<numBlocks, numThreads>>>(
        U1CAST(m_data_mgr.markersProximity_D->gridMarkerIndexD), U1CAST(m_data_mgr.extendedActivityIdD),
        U1CAST(m_data_mgr.markersProximity_D->mapOriginalToSorted), mR4CAST(m_data_mgr.sortedSphMarkers2_D->posRadD),
        mR3CAST(m_data_mgr.sortedSphMarkers2_D->velMasD), mR4CAST(m_data_mgr.sortedSphMarkers2_D->rhoPresMuD),
        mR3CAST(m_data_mgr.sortedSphMarkers2_D->tauXxYyZzD), mR3CAST(m_data_mgr.sortedSphMarkers2_D->tauXyXzYzD),
        mR4CAST(m_sphMarkersD->posRadD), mR3CAST(m_sphMarkersD->velMasD), mR4CAST(m_sphMarkersD->rhoPresMuD),
        mR3CAST(m_sphMarkersD->tauXxYyZzD), mR3CAST(m_sphMarkersD->tauXyXzYzD));

    cudaDeviceSynchronize();
    cudaCheckError();

    cudaFreeErrorFlag(error_flagD);
}

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono
