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
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Base class for processing boundary condition enforcing (bce) markers forces
// in fsi system.//
//
// =============================================================================

#ifndef CH_BCE_CUH_
#define CH_BCE_CUH_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChFsiGeneral.cuh"
#include "chrono_fsi/physics/ChSphGeneral.cuh"

namespace chrono {
namespace fsi {

// extern __constant__ SimParams paramsD;
// extern __constant__ NumberOfObjects numObjectsD;
/// @addtogroup fsi_physics
/// @{

/// @brief  Base class for processing boundary condition enforcing (BCE) marker forces
/// in an FSI system.
/// This class handles the Fluid-Solid Interaction by enforcing i) forces from the fluid dynamics system to the MBD
/// system and ii) displacement from the MBD system to the fluid dynamics system.
class CH_FSI_API ChBce : public ChFsiGeneral {
  public:
    thrust::device_vector<Real3> velMas_ModifiedBCE;
    thrust::device_vector<Real4> rhoPreMu_ModifiedBCE;

    /// Constructor of the ChBce
    ChBce(std::shared_ptr<SphMarkerDataD> otherSortedSphMarkersD,
          ///< Holds the position, velocity, denisty, pressure, viscousity and types of the SPH markers
          std::shared_ptr<ProximityDataD> otherMarkersProximityD,
          ///< Holds the information for the neighbor search and mapping from the sorted variables to an unsorted ones
          std::shared_ptr<FsiGeneralData> otherFsiGeneralData,
          ///< Some general information, e.g, ordering of the phases.
          std::shared_ptr<SimParams> otherParamsH,          ///< Parameter of the simulation
          std::shared_ptr<NumberOfObjects> otherNumObjects  ///< Holds the number of sph markers on each phase

    );

    /// Destructor of the ChBce
    virtual ~ChBce();

    /// Updates the position and velocity of the markers on the rigid bodies based on the state of the body
    virtual void UpdateRigidMarkersPositionVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                                    std::shared_ptr<FsiBodiesDataD> fsiBodiesD);
    virtual void UpdateFlexMarkersPositionVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                                   std::shared_ptr<FsiMeshDataD> fsiMeshD);

    /// Calculates the forces from the fluid dynamics system to the fsi system on rigid bodies
    virtual void Rigid_Forces_Torques(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                      std::shared_ptr<FsiBodiesDataD> fsiBodiesD);

    /// Calculates the forces from the fluid dynamics system to the fsi system on flexible bodies
    virtual void Flex_Forces(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<FsiMeshDataD> fsiMeshD);

    void ModifyBceVelocity(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<FsiBodiesDataD> fsiBodiesD);

    /// Populates the BCE markers on the rigid bodies at the initial configuration of the system. The local coordinates
    /// w.r.t to the coordinate system of the rigid body is saved and is used during the update stage. In such a
    /// condition the position and orientation of the body is enough to update the position of all the markers attached
    /// to it.
    virtual void Populate_RigidSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                               std::shared_ptr<FsiBodiesDataD> fsiBodiesD);

    /// Populates the BCE markers on the flexible bodies at the initial configuration of the system. The local
    /// coordinates w.r.t to the coordinate system of the rigid body is saved and is used during the update stage. In
    /// such a condition the position and orientation of the body is enough to update the position of all the markers
    /// attached to it.
    virtual void Populate_FlexSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                                              std::shared_ptr<FsiMeshDataD> fsiMeshD);

    /// Finalizes the construction of the ChBce at the intial configuration of the system.
    virtual void Finalize(std::shared_ptr<SphMarkerDataD> sphMarkersD,
                          std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                          std::shared_ptr<FsiMeshDataD> fsiMeshD);

  private:
    std::shared_ptr<FsiGeneralData>
        fsiGeneralData;  ///< General information of the simulation, e.g, ordering of the phases.
    std::shared_ptr<SphMarkerDataD>
        sortedSphMarkersD;  ///< Holds the position, velocity, denisty, pressure, viscousity and types of
                            /// the SPH markers
    std::shared_ptr<ProximityDataD>
        markersProximityD;               ///< Holds the information for the neighbor search and mapping from the  sorted
                                         /// variables to an unsorted ones
    std::shared_ptr<SimParams> paramsH;  ///< Parameters of the simulation
    std::shared_ptr<NumberOfObjects> numObjectsH;  ///< Holds the number of sph markers on each phase
    thrust::device_vector<Real4>
        totalSurfaceInteractionRigid4;  ///< Total surface-integrated forces from the fluid dynamics to Chbodies
    thrust::device_vector<Real3>
        torqueMarkersD;  ///< Total surface-integrated torques from the fluid dynamics to Chbodies

    thrust::device_vector<int> dummyIdentify;

    /// Calculates the acceleration of the BCE markers based on the information of the ChSystem
    void CalcBceAcceleration(thrust::device_vector<Real3>& bceAcc,
                             const thrust::device_vector<Real4>& q_fsiBodies_D,
                             const thrust::device_vector<Real3>& accRigid_fsiBodies_D,
                             const thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,
                             const thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,
                             const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
                             const thrust::device_vector<uint>& rigidIdentifierD,
                             int numRigid_SphMarkers);

    void RecalcSortedVelocityPressure_BCE(std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                                          thrust::device_vector<Real3>& velMas_ModifiedBCE,
                                          thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,
                                          const thrust::device_vector<Real4>& sortedPosRad,
                                          const thrust::device_vector<Real3>& sortedVelMas,
                                          const thrust::device_vector<Real4>& sortedRhoPreMu,
                                          const thrust::device_vector<uint>& cellStart,
                                          const thrust::device_vector<uint>& cellEnd,
                                          const thrust::device_vector<uint>& mapOriginalToSorted,
                                          const thrust::device_vector<Real3>& bceAcc,
                                          int3 updatePortion);

    /// At the initial configuration of the system, identifies the index of the rigid body to which a BCE marker is
    /// attached.
    void MakeRigidIdentifier();

    /// At the initial configuration of the system, identifies the index of the flexible body to which a BCE marker is
    /// attached.
    void MakeFlexIdentifier();
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
