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
// Author: Arman Pazouki
// =============================================================================

#ifndef CH_FSI_FORCE_EXPLICITSPH_H_
#define CH_FSI_FORCE_EXPLICITSPH_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChFsiForce.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief Child class of ChFsiForce
class CH_FSI_API ChFsiForceExplicitSPH : public ChFsiForce {
  public:
    ChFsiForceExplicitSPH(
        std::shared_ptr<ChBce> otherBceWorker,                    ///< Pointer to the ChBce object that handles BCE markers
        std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,   ///< Information of markers in the sorted array on device
        std::shared_ptr<ProximityDataD> otherMarkersProximityD,   ///< Pointer to the object that holds the proximity of the markers on device
        std::shared_ptr<FsiGeneralData> otherFsiGeneralData,      ///< Pointer to the sph general data
        std::shared_ptr<SimParams> otherParamsH,                  ///< Pointer to the simulation parameters on host
        std::shared_ptr<NumberOfObjects> otherNumObjects          ///< Pointer to number of objects, fluid and boundary markers, etc.
    );

    ~ChFsiForceExplicitSPH();
    void Finalize() override;

  private:
    int density_initialization;

    void ForceSPH(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                  std::shared_ptr<FsiBodiesDataD> otherFsiBodiesD,
                  std::shared_ptr<FsiMeshDataD> fsiMeshD) override;
    thrust::device_vector<Real3> shift_r;

    /// Function to calculate the xsph velocity of the particles.
    /// XSPH velocity is a compromise between Eulerian and Lagrangian velocities, used
    /// to regularize the markers velocity and reduce noise.
    void CalculateXSPH_velocity();

    /// A wrapper around collide function, where calculates the force on markers, and copies the
    /// sorted xsph velocities to the original. The latter is needed later for position update.
    void CollideWrapper();

    /// Function to calculate the force terms for SPH markers.
    /// This function calculates the derivatives of the density and velocity in a WCSPH fashion.
    void collide(thrust::device_vector<Real4>& sortedDerivVelRho_fsi_D,
                 thrust::device_vector<Real4>& sortedPosRad,
                 thrust::device_vector<Real3>& sortedVelMas,
                 thrust::device_vector<Real4>& sortedRhoPreMu,
                 thrust::device_vector<Real3>& velMas_ModifiedBCE,
                 thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,

                 thrust::device_vector<uint>& gridMarkerIndex,
                 thrust::device_vector<uint>& cellStart,
                 thrust::device_vector<uint>& cellEnd);

    /// Function to add gravity force (acceleration) to other forces on SPH  markers.
    void AddGravityToFluid();
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
