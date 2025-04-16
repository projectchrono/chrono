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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu, Radu Serban
// =============================================================================
//
// Base class for processing boundary condition enforcing (BCE) particles forces
// in FSI system.
//
// =============================================================================

#ifndef CH_BCE_MANAGER_H
#define CH_BCE_MANAGER_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Manager for processing boundary condition enforcing (BCE) particle forces in an FSI system.
/// This class handles the Fluid-Solid Interaction by enforcing:
/// - forces from the fluid/granular dynamics system to the MBD system
/// - displacements from the MBD system to the fluid dynamics system
class BceManager {
  public:
    BceManager(FsiDataManager& data_mgr,  ///< FSI data
               bool use_node_directions,  ///< use higher-order interpolation for flex solid BCEs
               bool verbose,              ///< verbose terminal output
               bool check_errors          ///< check CUDA errors
    );

    ~BceManager();

    // Update position and velocity of BCE markers on rigid solids
    void UpdateBodyMarkerState();
    void UpdateBodyMarkerStateInitial();

    // Update position and velocity of BCE markers on flex 1-D solids
    void CalcNodeDirections1D(thrust::device_vector<Real3>& dirs);
    void UpdateMeshMarker1DState();
    void UpdateMeshMarker1DStateInitial();

    // Update position and velocity of BCE markers on flex 2-D solids
    void CalcNodeDirections2D(thrust::device_vector<Real3>& dirs);
    void UpdateMeshMarker2DState();
    void UpdateMeshMarker2DStateInitial();

    /// Calculate fluid forces on rigid bodies.
    void Rigid_Forces_Torques();

    /// Calculates fluid forces on nodes of 1-D flexible solids.
    void Flex1D_Forces();

    /// Calculates fluid forces on nodes of 2-D flexible solids.
    void Flex2D_Forces();

    void updateBCEAcc();

    /// Populate the BCE markers on the rigid bodies at the initial configuration of the system.
    /// The local coordinates w.r.t to the coordinate system of the rigid bodies is saved and is used
    /// during the update stage. In such a condition the position and orientation of the body is
    /// enough to update the position of all the particles attached to it.
    void Populate_RigidSPH_MeshPos_LRF(std::vector<int> fsiBodyBceNum);

    /// Complete construction of the BCE at the intial configuration of the system.
    void Initialize(std::vector<int> fsiBodyBceNum);

  private:
    FsiDataManager& m_data_mgr;  ///< FSI data manager

    thrust::device_vector<Real3> m_totalForceRigid;   ///< Total forces from fluid to bodies
    thrust::device_vector<Real3> m_totalTorqueRigid;  ///< Total torques from fluid to bodies

    uint m_rigidBodyBlockSize;  ///< Block size used for kernel that accumulates force from Rigid BCE markers
    uint m_rigidBodyGridSize;   ///< Grid size used for kernel that accumulates force from Rigid BCE markers
    thrust::device_vector<uint> m_rigidBodyBlockValidThreads;  ///< Vector of size number of blocks that holds the
                                                               ///< number of valid (non-padding) threads in the block
    thrust::device_vector<uint>
        m_rigidBodyAccumulatedPaddedThreads;  ///< Vector of size number of blocks that holds the accumulated number of
                                              ///< padded threads uptill that block

    bool m_use_node_directions;
    bool m_verbose;
    bool m_check_errors;

    // Calculate accelerations of solid BCE markers based on the information of the ChSystem.
    void CalcRigidBceAcceleration();
    void CalcFlex1DBceAcceleration();
    void CalcFlex2DBceAcceleration();
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
