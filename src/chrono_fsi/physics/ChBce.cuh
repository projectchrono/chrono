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
#include "chrono_fsi/physics/ChFsiBase.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief  Base class for processing boundary condition enforcing (BCE) particle forces in an FSI system.
///
/// This class handles the Fluid-Solid Interaction by enforcing i) forces from the fluid/granular dynamics
/// system to the MBD system, and ii) displacement from the MBD system to the fluid dynamics system.
class ChBce : public ChFsiBase {
  public:
    /// Modified velocity information for BCE particles.
    thrust::device_vector<Real3> velMas_ModifiedBCE;

    /// Modified density, pressure information for BCE particles.
    thrust::device_vector<Real4> rhoPreMu_ModifiedBCE;

    /// Modified stress tensor for BCE particles, diagonal entries.
    thrust::device_vector<Real3> tauXxYyZz_ModifiedBCE;

    /// Modified stress tensor for BCE particles， non-diagonal entries.
    thrust::device_vector<Real3> tauXyXzYz_ModifiedBCE;

    /// Constructor of the ChBce class
    ChBce(std::shared_ptr<SphMarkerDataD> sortedSphMarkers_D,  ///< data for SPH particles
          std::shared_ptr<ProximityDataD> markersProximity_D,  ///< information for neighbor search
          std::shared_ptr<FsiData> fsiData,                    ///< general information, e.g, ordering of the phases
          std::shared_ptr<SimParams> paramsH,                  ///< simulation parameters
          std::shared_ptr<ChCounters> numObjects,              ///< number of sph particles on each phase
          bool verbose                                         ///< verbose terminal output
    );

    /// Destructor of the ChBce class
    ~ChBce();

    /// Updates the position and velocity of the particles on the rigid bodies based on the state of the body.
    void UpdateBodyMarkerState(std::shared_ptr<FsiBodyStateD> fsiBodyState_D);

    /// Updates the position and velocity of the particles on the flexible solids based on the state of the mesh.
    void UpdateMeshMarker1DState(std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D);
    void UpdateMeshMarker2DState(std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D);

    /// Updates the position and velocity of the particles on the rigid bodies based on the state of the body.
    void UpdateBodyMarkerStateInitial(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                               std::shared_ptr<FsiBodyStateD> fsiBodyState_D);

    /// Updates the position and velocity of the particles on the flexible solids based on the state of the mesh.
    void UpdateMeshMarker1DStateInitial(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                 std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D);
    void UpdateMeshMarker2DStateInitial(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                 std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D);

    /// Calculates the forces from the fluid/granular dynamics system to the FSI system on rigid bodies.
    void Rigid_Forces_Torques(std::shared_ptr<FsiBodyStateD> fsiBodyState_D);

    /// Calculates the forces from the fluid/granular dynamics system to the FSI system on flexible bodies.
    void Flex1D_Forces(std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D);
    void Flex2D_Forces(std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D);

    void CalcMeshMarker1DAcceleration(thrust::device_vector<Real3>& bceAcc,
                                      std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D);
    void CalcMeshMarker2DAcceleration(thrust::device_vector<Real3>& bceAcc,
                                      std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D);
    void updateBCEAcc(std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                             std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,
                             std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D);

    /// Populates the BCE particles on the rigid bodies at the initial configuration of the system.
    /// The local coordinates w.r.t to the coordinate system of the rigid bodies is saved and is used
    /// during the update stage. In such a condition the position and orientation of the body is
    /// enough to update the position of all the particles attached to it.
    void Populate_RigidSPH_MeshPos_LRF(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                                       std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                                       std::vector<int> fsiBodyBceNum);

    /// Complete construction of the BCE at the intial configuration of the system.
    void Initialize(std::shared_ptr<SphMarkerDataD> sphMarkers_D,
                    std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                    std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,
                    std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D,
                    std::vector<int> fsiBodyBceNum);

  private:
    std::shared_ptr<FsiData> m_fsiData;                   ///< General information of the simulation
    std::shared_ptr<SphMarkerDataD> m_sortedSphMarkersD;  ///< Particle state, properties, type
    std::shared_ptr<ProximityDataD> m_markersProximityD;  ///< Information for neighbor search
    thrust::device_vector<Real3> m_totalForceRigid;       ///< Total forces from fluid to bodies
    thrust::device_vector<Real3> m_totalTorqueRigid;      ///< Total torques from fluid to bodies

    bool m_verbose;

    /// Calculates the acceleration of the rigid BCE particles based on the information of the ChSystem.
    void CalcRigidBceAcceleration(
        thrust::device_vector<Real3>& bceAcc,                         ///< acceleration of BCE particles
        const thrust::device_vector<Real4>& q_fsiBodies_D,            ///< quaternion of rigid bodies
        const thrust::device_vector<Real3>& accRigid_fsiBodies_D,     ///< acceleration of rigid bodies
        const thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,  ///< body ang. vel. in local reference frame
        const thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,  ///< body ang. acc. in local reference frame
        const thrust::device_vector<Real3>& rigid_BCEcoords_D,        ///< position of BCE in body local ref.
        const thrust::device_vector<uint>& rigid_BCEsolids_D          ///< ID of associated body
    );
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
