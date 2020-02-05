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
// Author: Milad Rakhsha, Arman Pazouki
// =============================================================================
//
// Base class for processing SPH force in a FSI system.
//
// =============================================================================

#ifndef CH_FSI_FORCE_H_
#define CH_FSI_FORCE_H_

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include "chrono_fsi/ChBce.cuh"
#include "chrono_fsi/ChCollisionSystemFsi.cuh"
#include "chrono_fsi/ChFsiLinearSolver.h"
#include "chrono_fsi/ChFsiLinearSolverBiCGStab.h"
#include "chrono_fsi/ChSphGeneral.cuh"
#include "chrono_fsi/ExactLinearSolvers.cuh"

namespace chrono {
namespace fsi {

struct compare_Real4_x {
    __host__ __device__ bool operator()(Real4 lhs, Real4 rhs) { return lhs.x < rhs.x; }
};
struct compare_Real4_y {
    __host__ __device__ bool operator()(Real4 lhs, Real4 rhs) { return lhs.y < rhs.y; }
};
struct compare_Real3_mag {
    __host__ __device__ bool operator()(Real3 lhs, Real3 rhs) { return length(lhs) < length(rhs); }
};
struct Real4_x {
    const Real rest_val;
    Real4_x(Real _a) : rest_val(_a) {}
    __host__ __device__ Real operator()(const Real4& input) const {
        return (input.w != -1.0) ? 0.0 : abs(input.x - rest_val);
    }
};

struct Real4_y {
    __host__ __device__ Real operator()(const Real4& input) const { return (input.w != -1.0) ? 0.0 : input.y; }
};

__device__ inline void clearRow(uint i_idx, uint csrStartIdx, uint csrEndIdx, Real* A_Matrix, Real* Bi) {
    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        A_Matrix[count] = 0;
        Bi[i_idx] = 0;
    }
}
__device__ inline void clearRow3(uint i_idx, uint csrStartIdx, uint csrEndIdx, Real* A_Matrix, Real3* Bi) {
    for (int count = csrStartIdx; count < csrEndIdx; count++) {
        A_Matrix[count] = 0;
        Bi[i_idx] = mR3(0.0);
    }
}
/// @addtogroup fsi_physics
/// @{

/// @brief Class to calculate force between SPH markers in Weakly Compressible SPH.
///
/// This is an abstract class that defines an interface that various SPH method
/// should implement. The class owns a collision system fsi which takes care of GPU based
/// proximity computation of the markers. It also holds a pointer to external
/// data of SPH markers, proximity data, parameters, and numbers.
/// Child class must implement Finalize and ForceSPH methods

class CH_FSI_API ChFsiForce : public ChFsiGeneral {
  public:
    /// Base constructor for FSI force class.
    /// The constructor instantiates the collision system (ChCollisionSystemFsi)
    /// and initializes the pointer to external data.
    //    ChFsiForce() {}
    ChFsiForce(ChBce* otherBceWorker,                   ///< Pointer to the ChBce object that handles BCE markers
               SphMarkerDataD* otherSortedSphMarkersD,  ///< Information of markers in the sorted array on device
               ProximityDataD*
                   otherMarkersProximityD,  ///< Pointer to the object that holds the proximity of the markers on device
               FsiGeneralData* otherFsiGeneralData,  ///< Pointer to the sph general data
               SimParams* otherParamsH,              ///< Pointer to the simulation parameters on host
               NumberOfObjects* otherNumObjects      ///< Pointer to number of objects, fluid and boundary markers, etc.
    );
    /// Destructor. Deletes the collision system.
    virtual ~ChFsiForce();

    /// Function calculate the force on SPH markers.
    /// This is a basic force computation relying on WCSPH approach.
    //    void ForceSPH(SphMarkerDataD* otherSphMarkersD, FsiBodiesDataD* otherFsiBodiesD);

    /// This is a virtual method that needs to be overridden by the child classes to compute fluid forces in an
    /// implicit integrator.
    virtual void ForceSPH(SphMarkerDataD* otherSphMarkersD,
                          FsiBodiesDataD* otherFsiBodiesD,
                          FsiMeshDataD* fsiMeshD) = 0;

    /// Synchronize the copy of the data (parameters and number of objects)
    /// between device (GPU) and host (CPU).
    /// This function needs to be called once the host data are modified.
    virtual void Finalize();

    /// Copy sorted data into original data.
    /// This function copies the data that are sorted in the collision system, into the
    /// original data, where data is real3. The class is invasive, meaning that the sorted
    /// data will be modified (and will be equivalent to the original). Therefore,  this
    /// function should be used whenever sorted data is not needed, but efficiency is preferred.
    static void CopySortedToOriginal_Invasive_R3(thrust::device_vector<Real3>& original,
                                                 thrust::device_vector<Real3>& sorted,
                                                 const thrust::device_vector<uint>& gridMarkerIndex);

    /// Copy sorted data into original data.
    /// This function copies the data that are sorted in the collision system,  into the
    /// original data, where data is real3. The class is non-invasive, meaning  that the
    /// sorted data will not be modified. This comes at the expense of lower efficiency.
    static void CopySortedToOriginal_NonInvasive_R3(thrust::device_vector<Real3>& original,
                                                    const thrust::device_vector<Real3>& sorted,
                                                    const thrust::device_vector<uint>& gridMarkerIndex);

    /// Copy sorted data into original data.
    /// This function copies the data that are sorted in the collision system, into the
    /// original data, where data is real4. The class is invasive, meaning that the sorted
    /// data will be modified (and will be equivalent to the original). Therefore,  this
    /// function should be used whenever sorted data is not needed, but efficiency is preferred.
    static void CopySortedToOriginal_Invasive_R4(thrust::device_vector<Real4>& original,
                                                 thrust::device_vector<Real4>& sorted,
                                                 const thrust::device_vector<uint>& gridMarkerIndex);

    /// Copy sorted data into original data.
    /// This function copies the data that are sorted in the collision system, into the
    /// original data, where data is real4. The class is non-invasive, meaning that the
    /// sorted data will not be modified. This comes at the expense of lower efficiency.
    static void CopySortedToOriginal_NonInvasive_R4(thrust::device_vector<Real4>& original,
                                                    thrust::device_vector<Real4>& sorted,
                                                    const thrust::device_vector<uint>& gridMarkerIndex);

    /// Set the linear solver used in the simulation
    void SetLinearSolver(ChFsiLinearSolver::SolverType other_solverType);

  public:
    ChFsiLinearSolver* myLinearSolver;  ///< A pointer to the linear solver class used for ChFsiForce.

    ChBce* bceWorker;                          ///< pointer to Boundary Condition Enforcing markers class.
    ChCollisionSystemFsi* fsiCollisionSystem;  ///< collision system; takes care of  constructing neighbors list

    // The class takes care of BCE related computations. It is needed here, however,
    // for the implemetation of the ADAMI boundary condition

    SphMarkerDataD* sphMarkersD;        ///< device copy of the sph markers data
    SphMarkerDataD* sortedSphMarkersD;  ///< device copy of the sorted sph markers data
    ProximityDataD* markersProximityD;  ///< pointer object that holds the proximity of the markers
    FsiGeneralData* fsiGeneralData;     ///< pointer to sph general data

    SimParams* paramsH;            ///< pointer to simulation parameters
    NumberOfObjects* numObjectsH;  ///< pointer to number of objects, fluid and boundary markers

    thrust::device_vector<Real3> vel_vis_Sorted_D;       ///< sorted visualization velocity data
    thrust::device_vector<Real3> vel_XSPH_Sorted_D;      ///< sorted xsph velocity data
    thrust::device_vector<Real4> derivVelRhoD_Sorted_D;  ///< sorted derivVelRhoD
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
