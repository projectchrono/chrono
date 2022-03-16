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

#ifndef CH_FSI_FORCE_EXPLICITSPH_H_
#define CH_FSI_FORCE_EXPLICITSPH_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChFsiForce.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief Child class of ChFsiForce.
class ChFsiForceExplicitSPH : public ChFsiForce {
  public:
    /// Constructor of the ChFsiForceExplicitSPH class.
    /// Force class implemented using WCSPH with explicit integretor,
    /// supports for both fluid and granular material dynamics.
    ChFsiForceExplicitSPH(
        std::shared_ptr<ChBce> otherBceWorker,                    ///< Pointer to the ChBce object that handles BCE particles
        std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,   ///< Information of SPH particles in the sorted array on device
        std::shared_ptr<ProximityDataD> otherMarkersProximityD,   ///< Pointer to the object that holds the proximity of the particles on device
        std::shared_ptr<FsiGeneralData> otherFsiGeneralData,      ///< Pointer to the SPH general data
        std::shared_ptr<SimParams> otherParamsH,                  ///< Pointer to the simulation parameters on host
        std::shared_ptr<NumberOfObjects> otherNumObjects          ///< Pointer to number of objects, fluid, solid body, and boundary particles, etc.
    );

    /// Destructor of the ChFsiForceExplicitSPH class
    ~ChFsiForceExplicitSPH();

    void Finalize() override;

  private:
    int density_initialization;
    thrust::device_vector<Real3> sortedXSPHandShift;

    /// Function to find neighbor particles and calculate the interactions between SPH particles
    void ForceSPH(std::shared_ptr<SphMarkerDataD> otherSphMarkersD,
                  std::shared_ptr<FsiBodiesDataD> otherFsiBodiesD,
                  std::shared_ptr<FsiMeshDataD> fsiMeshD) override;
    
    /// Function to calculate the XSPH velocity of the particles.
    /// XSPH velocity is a compromise between Eulerian and Lagrangian velocities, used
    /// to regularize the particles velocity and reduce noise.
    void CalculateXSPH_velocity();

    /// A wrapper around collide function, where calculates the force on particles, and copies the
    /// sorted XSPH velocities to the original. The latter is needed later for position update.
    void CollideWrapper();

    /// Function to add gravity force (acceleration) to other forces on SPH  particles.
    void AddGravityToFluid();
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
