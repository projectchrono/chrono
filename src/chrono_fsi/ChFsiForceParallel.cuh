// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Base class for processing SPH force in a FSI system.
//
// =============================================================================

#ifndef CH_FSI_FORCEPARALLEL_H_
#define CH_FSI_FORCEPARALLEL_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChBce.cuh"
#include "chrono_fsi/ChCollisionSystemFsi.cuh"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChFsiGeneral.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief Class to calculate force between SPH markers in Weakly Compressible SPH.
///
/// This class implements the necessary functions for a WCSPH force calculation
/// method. The class owns a collision system fsi which takes care of GPU based
/// proximity computation of the markers. It also holds a pointer to external
/// data of SPH markers, proximity data, parameters, and numbers.
class CH_FSI_API ChFsiForceParallel : public ChFsiGeneral {
  public:
    /// Base constructor for FSI force class.
    /// The constructor instantiates the collision system (ChCollisionSystemFsi)
    /// and initializes the pointer to external data.
    ChFsiForceParallel(
        ChBce* otherBceWorker,                   ///< Pointer to the ChBce object that handles BCE markers
        SphMarkerDataD* otherSortedSphMarkersD,  ///< Information of markers in the sorted array on device
        ProximityDataD*
            otherMarkersProximityD,  ///< Pointer to the object that holds the proximity of the markers on device
        FsiGeneralData* otherFsiGeneralData,  ///< Pointer to the sph general data
        SimParams* otherParamsH,              ///< Pointer to the simulation parameters on host
        NumberOfObjects* otherNumObjects      ///< Pointer to number of objects, fluid and boundary markers, etc.
        );

    /// Destructor. Deletes the collision system.
    ~ChFsiForceParallel();

    /// Function calculate the force on SPH markers.
    /// This is a basic force computation relying on WCSPH approach.
    virtual void ForceSPH(SphMarkerDataD* otherSphMarkersD, FsiBodiesDataD* otherFsiBodiesD);

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

  private:
    /// Function to calculate the xsph velocity of the particles.
    /// XSPH velocity is a compromise between Eulerian and Lagrangian velocities, used
    /// to regularize the markers velocity and reduce noise.
    void CalculateXSPH_velocity();

    /// A wrapper around collide function, where calculates the force on markers, and copies the
    /// sorted xsph velocities to the original. The latter is needed later for position update.
    void CollideWrapper();

    ChCollisionSystemFsi* fsiCollisionSystem;  ///< collision system; takes care of  constructing neighbors list
    ChBce* bceWorker;                          ///< pointer to Boundary Condition Enforcing markers class.

    // The class takes care of BCE related computations. It is needed here, however,
    // for the implemetation of the ADAMI boundary condition

    SphMarkerDataD* sphMarkersD;        ///< device copy of the sph markers data
    SphMarkerDataD* sortedSphMarkersD;  ///< device copy of the sorted sph markers data
    ProximityDataD* markersProximityD;  ///< pointer object that holds the proximity of the markers
    FsiGeneralData* fsiGeneralData;     ///< pointer to sph general data

    SimParams* paramsH;            ///< pointer to simulation parameters
    NumberOfObjects* numObjectsH;  ///< pointer to number of objects, fluid and boundary markers

    thrust::device_vector<Real3> vel_XSPH_Sorted_D;  ///< sorted xsph velocity data

    /// Function to calculate the force terms for SPH markers.
    /// This function calculates the derivatives of the density and velocity in a WCSPH fashion.
    void collide(
        thrust::device_vector<Real4>& sortedDerivVelRho_fsi_D,  ///< dv/dt, and d(rho)/dt in the sorted array
        thrust::device_vector<Real3>& sortedPosRad,             ///< position of markers in the sorted array
        thrust::device_vector<Real3>& sortedVelMas,             ///< velocity of markers in the sorted array
        thrust::device_vector<Real3>& vel_XSPH_Sorted_D,        ///< XSPH velocity of markers in the sorted array
        thrust::device_vector<Real4>& sortedRhoPreMu,  ///< (rho,pressure,mu,type) of markers in the sorted array
        thrust::device_vector<Real3>& velMas_ModifiedBCE,
        thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,

        thrust::device_vector<uint>& gridMarkerIndex,  ///< To get the original index based on the sorted index
        thrust::device_vector<uint>& cellStart,        ///< Index of the first marker in a cell
        thrust::device_vector<uint>& cellEnd);         ///< Index of the last marker in a cell

    /// Function to add gravity force (acceleration) to other forces on SPH  markers.
    void AddGravityToFluid();
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
