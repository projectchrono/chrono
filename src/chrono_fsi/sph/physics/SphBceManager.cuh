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

#ifndef CH_SPH_BCE_MANAGER_H
#define CH_SPH_BCE_MANAGER_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/physics/SphDataManager.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Manager for processing boundary condition enforcing (BCE) particle forces in an FSI system.
/// This class handles the Fluid-Solid Interaction by enforcing:
/// - forces from the fluid/granular dynamics system to the MBD system
/// - displacements from the MBD system to the fluid dynamics system
class SphBceManager {
  public:
    SphBceManager(FsiDataManager& data_mgr,             ///< FSI data
               NodeDirections node_directions_mode,  ///< enable/disable use of FEA node direction info
               bool verbose,                         ///< verbose terminal output
               bool check_errors                     ///< check CUDA errors
    );

    ~SphBceManager();

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

    /// Complete construction of the BCE at the intial configuration of the system.
    void Initialize(std::vector<int> fsiBodyBceNum);

  private:
    /// Set up block sizes for rigid body force accumulation.
    void SetForceAccumulationBlocks(std::vector<int> fsiBodyBceNum);

    // Calculate accelerations of solid BCE markers based on the information of the ChSystem.
    void CalcRigidBceAcceleration();
    void CalcFlex1DBceAcceleration();
    void CalcFlex2DBceAcceleration();

  private:
    FsiDataManager& m_data_mgr;  ///< FSI data manager

    thrust::device_vector<Real3> m_totalForceRigid;   ///< Total forces from fluid to bodies
    thrust::device_vector<Real3> m_totalTorqueRigid;  ///< Total torques from fluid to bodies

    uint m_rigid_block_size;                                  ///< Block size for the rigid force accumulator kernel
    uint m_rigid_grid_size;                                   ///< Grid size for the rigid force accumulator kernel
    thrust::device_vector<uint> m_rigid_valid_threads;        ///< numbers of valid (non-padding) threads in the block
    thrust::device_vector<uint> m_rigid_accumulated_threads;  ///< accumulated numbers of padded threads before a block

    NodeDirections m_node_directions_mode;
    bool m_verbose;
    bool m_check_errors;
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
