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
// Base class for processing boundary condition enforcing (BCE) particles forces
// in FSI system.
//
// =============================================================================

#ifndef CH_BCE_CUH_
#define CH_BCE_CUH_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChFsiGeneral.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief  Base class for processing boundary condition enforcing (BCE) particle forces in an FSI system.
///
/// This class handles the Fluid-Solid Interaction by enforcing i) forces from the fluid/granular dynamics
/// system to the MBD system, and ii) displacement from the MBD system to the fluid dynamics system.
class ChBce : public ChFsiGeneral {
  public:
    /// Modified velocity information for BCE particles.
    thrust::device_vector<Real3> velMas_ModifiedBCE;

    /// Modified density, pressure information for BCE particles.
    thrust::device_vector<Real4> rhoPreMu_ModifiedBCE;

    /// Modified stress tensor for BCE particles.
    thrust::device_vector<Real3> tauXxYyZz_ModifiedBCE;
    thrust::device_vector<Real3> tauXyXzYz_ModifiedBCE;

    /// Constructor of the ChBce class
    ChBce(std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,  ///< data for SPH particles
          std::shared_ptr<ProximityDataD> otherMarkersProximityD,  ///< information for neighbor search
          std::shared_ptr<FsiGeneralData> otherFsiGeneralData,     ///< general information, e.g, ordering of the phases
          std::shared_ptr<SimParams> otherParamsH,                 ///< simulation parameters
          std::shared_ptr<ChCounters> otherNumObjects,             ///< number of sph particles on each phase
          bool verb                                                ///< verbose terminal output
    );

    /// Destructor of the ChBce class
    ~ChBce();

    /// Updates the position and velocity of the particles on the rigid bodies based on the state of the body.
    void UpdateRigidMarkersPositionVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                            std::shared_ptr<FsiBodiesDataD> fsiBodiesD);

    /// Updates the position and velocity of the particles on the flexible bodies based on the state of the body.
    void UpdateFlexMarkersPositionVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                           std::shared_ptr<FsiMeshDataD> fsiMeshD);

    /// Calculates the forces from the fluid/granular dynamics system to the FSI system on rigid bodies.
    void Rigid_Forces_Torques(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<FsiBodiesDataD> fsiBodiesD);

    /// Calculates the forces from the fluid/granular dynamics system to the FSI system on flexible bodies.
    void Flex_Forces(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<FsiMeshDataD> fsiMeshD);

    /// Modify the velocity of BCE particles according to the SPH particles around.
    void ModifyBceVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<FsiBodiesDataD> fsiBodiesD);

    /// Populates the BCE particles on the rigid bodies at the initial configuration of the system.
    /// The local coordinates w.r.t to the coordinate system of the rigid bodies is saved and is used
    /// during the update stage. In such a condition the position and orientation of the body is
    /// enough to update the position of all the particles attached to it.
    void Populate_RigidSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                       std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                                       std::vector<int> fsiBodyBceNum);

    /// Populates the BCE particles on the flexible bodies at the initial configuration of the system.
    /// The local coordinates w.r.t to the coordinate system of the flexible bodies is saved and is used
    /// during the update stage. In such a condition the position and orientation of the body is enough
    /// to update the position of all the particles attached to it.
    void Populate_FlexSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                      std::shared_ptr<FsiMeshDataD> fsiMeshD,
                                      std::vector<int> fsiShellBceNum,
                                      std::vector<int> fsiCableBceNum);

    /// Complete construction of the BCE at the intial configuration of the system.
    void Initialize(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                    std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                    std::shared_ptr<FsiMeshDataD> fsiMeshD,
                    std::vector<int> fsiBodyBceNum,
                    std::vector<int> fsiShellBceNum,
                    std::vector<int> fsiCableBceNum);

  private:
    std::shared_ptr<FsiGeneralData> fsiGeneralData;              ///< General information of the simulation
    std::shared_ptr<SphMarkerDataD> sortedSphMarkersD;           ///< Particle state, properties, type
    std::shared_ptr<ProximityDataD> markersProximityD;           ///< Information for neighbor search
    std::shared_ptr<SimParams> paramsH;                          ///< Parameters of the simulation
    std::shared_ptr<ChCounters> numObjectsH;                     ///< Holds the number of SPH particles on each phase
    thrust::device_vector<Real4> totalSurfaceInteractionRigid4;  ///< Total forces from fluid to bodies
    thrust::device_vector<Real3> torqueMarkersD;                 ///< Total torques from fluid to bodies

    thrust::device_vector<int> dummyIdentify;

    bool verbose;

    /// Calculates the acceleration of the BCE particles based on the information of the ChSystem.
    void CalcBceAcceleration(thrust::device_vector<Real3>& bceAcc,
                             const thrust::device_vector<Real4>& q_fsiBodies_D,
                             const thrust::device_vector<Real3>& accRigid_fsiBodies_D,
                             const thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,
                             const thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,
                             const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
                             const thrust::device_vector<uint>& rigidIdentifierD,
                             int numRigidMarkers);

    /// Calculates pressure and velocity of the BCE particles.
    void RecalcSortedVelocityPressure_BCE(std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                                          thrust::device_vector<Real3>& velMas_ModifiedBCE,
                                          thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,
                                          thrust::device_vector<Real3>& tauXxYyZz_ModifiedBCE,
                                          thrust::device_vector<Real3>& tauXyXzYz_ModifiedBCE,
                                          const thrust::device_vector<Real4>& sortedPosRad,
                                          const thrust::device_vector<Real3>& sortedVelMas,
                                          const thrust::device_vector<Real4>& sortedRhoPreMu,
                                          const thrust::device_vector<Real3>& sortedTauXxYyZz,
                                          const thrust::device_vector<Real3>& sortedTauXyXzYz,
                                          const thrust::device_vector<uint>& cellStart,
                                          const thrust::device_vector<uint>& cellEnd,
                                          const thrust::device_vector<uint>& mapOriginalToSorted,
                                          const thrust::device_vector<Real3>& bceAcc,
                                          int3 updatePortion);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
