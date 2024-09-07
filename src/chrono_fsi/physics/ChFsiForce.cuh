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
// Author: Milad Rakhsha, Wei Hu
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

#include "chrono_fsi/physics/ChBce.cuh"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChCollisionSystemFsi.cuh"

namespace chrono {
namespace fsi {

struct compare_Real4_x {
    __host__ __device__ bool operator()(Real4 lhs, Real4 rhs) { return lhs.x < rhs.x; }
};
struct compare_Real4_y {
    __host__ __device__ bool operator()(Real4 lhs, Real4 rhs) { return lhs.y < rhs.y; }
};
struct compare_Real4_z {
    __host__ __device__ bool operator()(Real4 lhs, Real4 rhs) { return lhs.z < rhs.z; }
};
struct compare_Real4_w {
    __host__ __device__ bool operator()(Real4 lhs, Real4 rhs) { return lhs.w < rhs.w; }
};
struct compare_Real3_mag {
    __host__ __device__ bool operator()(Real3 lhs, Real3 rhs) { return length(lhs) < length(rhs); }
};
struct equal_invalid {
    uint invalidParticleID;
    equal_invalid(uint y) { invalidParticleID = y; }
    __host__ __device__ bool operator()(const uint& x) { return x == invalidParticleID; }
};
struct Real4_x {
    const Real rest_val;
    Real4_x(Real _a) : rest_val(_a) {}
    __host__ __device__ Real operator()(const Real4& input) const {
        return (input.w != -1.0) ? 0.0 : abs(input.x - rest_val);
    }
};
struct saxpy_functor {
    const Real a;
    saxpy_functor(Real _a) : a(_a) {}
    __host__ __device__ Real operator()(const Real& x, const Real& y) const { return x + a * y; }
};
template <typename T>
struct square_functor {
    __host__ __device__ T operator()(const T& x) const { return x * x; }
};
struct Real4_z {
    Real4_z() {}
    __host__ __device__ Real operator()(const Real4& input) const { return (input.w == -1) ? input.z : 0; }
};

struct Real4_boundary {
    __host__ __device__ bool operator()(const Real4& input) const { return input.w == 1.0; }
};

struct Real4_y {
    __host__ __device__ Real operator()(const Real4& input) const { return (input.w != -1.0) ? 0.0 : input.y; }
};

struct Real4_y_min {
    __host__ __device__ Real operator()(const Real4& input) const { return (input.w != -1.0) ? 1e9 : input.y; }
};
__device__ inline void clearRow(uint i_idx, uint csrStartIdx, uint csrEndIdx, Real* A_Matrix, Real* Bi) {
    for (uint count = csrStartIdx; count < csrEndIdx; count++) {
        A_Matrix[count] = 0;
        Bi[i_idx] = 0;
    }
}
__device__ inline void clearRow3(uint i_idx, uint csrStartIdx, uint csrEndIdx, Real* A_Matrix, Real3* Bi) {
    for (uint count = csrStartIdx; count < csrEndIdx; count++) {
        A_Matrix[count] = 0;
        Bi[i_idx] = mR3(0.0);
    }
}
/// @addtogroup fsi_physics
/// @{

/// Base class to calculate force between SPH particles.
///
/// This is an abstract class that defines an interface that various SPH methods should implement. The class owns a
/// collision system fsi which takes care of GPU based proximity computation of the particles. It also holds a pointer
/// to external data of SPH particles, proximity data, parameters, and numbers.
class ChFsiForce : public ChFsiBase {
  public:
    /// Base constructor for the ChFsiForce class.
    /// The constructor instantiates the force system
    /// and initializes the pointer to external data.
    ChFsiForce(
        std::shared_ptr<ChBce> otherBceWorker,                   ///< object that handles BCE particles
        std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,  ///< information of particle in the sorted device array
        std::shared_ptr<ProximityDataD> otherMarkersProximityD,  ///< object that holds device proximity info
        std::shared_ptr<FsiData> otherFsiData,  ///< SPH general data
        std::shared_ptr<SimParams> params,             ///< simulation parameters
        std::shared_ptr<ChCounters> numObjects,        ///< problem counters
        bool verb                                      ///< verbose output
    );

    /// Destructor of the ChFsiForce.
    virtual ~ChFsiForce();

    /// Function to calculate forces on SPH particles.
    /// Implemented by derived classes to compute forces in an implicit integrator using ISPH method (see
    /// ChFsiForceI2SPH) or an explicit integrator using WCPSH method (see ChFsiForceExplicitSPH).
    virtual void ForceSPH(std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,
                          std::shared_ptr<FsiBodyStateD> fsiBodyStateD,
                          std::shared_ptr<FsiMeshStateD> fsiMesh1DStateD,
                          std::shared_ptr<FsiMeshStateD> fsiMesh2DStateD,
                          Real time,
                          bool firstHalfStep) = 0;

    /// Synchronize the copy of the data (parameters and number of objects)
    /// between device (GPU) and host (CPU).
    /// This function needs to be called once the host data are modified.
    virtual void Initialize();

    /// Copy sorted data into original data (real3).
    /// This function copies the data that are sorted in the collision system, into the
    /// original data, where data is real3. The class is invasive, meaning that the sorted
    /// data will be modified (and will be equivalent to the original). Therefore, this
    /// function should be used whenever sorted data is not needed, but efficiency is preferred.
    static void CopySortedToOriginal_Invasive_R3(thrust::device_vector<Real3>& original,
                                                 thrust::device_vector<Real3>& sorted,
                                                 const thrust::device_vector<uint>& gridMarkerIndex);

    /// Copy sorted data into original data (real3).
    /// This function copies the data that are sorted in the collision system, into the
    /// original data, where data is real3. The class is non-invasive, meaning that the
    /// sorted data will not be modified. This comes at the expense of lower efficiency.
    static void CopySortedToOriginal_NonInvasive_R3(thrust::device_vector<Real3>& original,
                                                    const thrust::device_vector<Real3>& sorted,
                                                    const thrust::device_vector<uint>& gridMarkerIndex);

    /// Copy sorted data into original data (real4).
    /// This function copies the data that are sorted in the collision system, into the
    /// original data, where data is real4. The class is invasive, meaning that the sorted
    /// data will be modified (and will be equivalent to the original). Therefore,  this
    /// function should be used whenever sorted data is not needed, but efficiency is preferred.
    static void CopySortedToOriginal_Invasive_R4(thrust::device_vector<Real4>& original,
                                                 thrust::device_vector<Real4>& sorted,
                                                 const thrust::device_vector<uint>& gridMarkerIndex);

    /// Copy sorted data into original data (real4).
    /// This function copies the data that are sorted in the collision system, into the
    /// original data, where data is real4. The class is non-invasive, meaning that the
    /// sorted data will not be modified. This comes at the expense of lower efficiency.
    static void CopySortedToOriginal_NonInvasive_R4(thrust::device_vector<Real4>& original,
                                                    thrust::device_vector<Real4>& sorted,
                                                    const thrust::device_vector<uint>& gridMarkerIndex);

    /// Function to set the linear solver type for the solver implemented using the ISPH method (ChFsiForceI2SPH).
    void SetLinearSolver(SolverType type);

    std::shared_ptr<ChCollisionSystemFsi> fsiCollisionSystem;  ///< collision system for building neighbors list

  protected:
    std::shared_ptr<ChBce> bceWorker;  ///< pointer to Boundary Condition Enforcing particles class


    std::shared_ptr<SphMarkerDataD> sphMarkersD;         ///< device copy of the SPH particles data
    std::shared_ptr<SphMarkerDataD> sortedSphMarkers_D;  ///< device copy of the sorted sph particles data
    std::shared_ptr<ProximityDataD> markersProximity_D;  ///< pointer object that holds the proximity of the particles
    std::shared_ptr<FsiData> fsiData;  ///< pointer to sph general data

    thrust::device_vector<Real3> vel_vis_Sorted_D;       ///< sorted visualization velocity data
    thrust::device_vector<Real3> vel_XSPH_Sorted_D;      ///< sorted xsph velocity data
    thrust::device_vector<Real4> derivVelRhoD_Sorted_D;  ///< sorted derivVelRhoD

    bool verbose;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
