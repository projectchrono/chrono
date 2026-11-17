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

#include <vector>

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
                  bool check_errors                     ///< check GPU errors
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
    void CalcRigidBodyForces();

    /// Calculates fluid forces on nodes of 1-D flexible solids.
    void CalcFeaMesh1DForces();

    /// Calculates fluid forces on nodes of 2-D flexible solids.
    void CalcFeaMesh2DForces();

    /// Calculate accelerations of solid BCE markers, in sorted order, into the data manager's bceAcc array.
    void updateBCEAcc();

    /// Return the accelerations of BCE markers on solids, in original marker order.
    /// The accelerations follow from the current solid states; the first entry is that of the first rigid BCE marker.
    std::vector<Real3> GetSolidMarkerAccelerations();

    /// Complete construction of the BCE at the initial configuration of the system.
    void Initialize(std::vector<int> fsiBodyBceNum);

  private:
    /// Set up block sizes for rigid body force accumulation.
    void SetForceAccumulationBlocks(std::vector<int> fsiBodyBceNum);

    /// Carry the reference normals of 1-D flex segments over to the current segment configurations.
    void UpdateSegmentNormals1D();

    // Calculate accelerations of solid BCE markers based on the information of the ChSystem.
    // Output goes to accelerations[map[i]] for marker i if a sort map is provided, else to accelerations[i - offset].
    void CalcRigidBodyBceAcceleration(Real3* accelerations, const uint* map, uint offset);
    void CalcFeaMesh1DBceAcceleration(Real3* accelerations, const uint* map, uint offset);
    void CalcFeaMesh2DBceAcceleration(Real3* accelerations, const uint* map, uint offset);

  private:
    FsiDataManager& m_data_mgr;  ///< FSI data manager

    thrust::device_vector<Real3> m_rigid_block_forces;   ///< Force partial sums for each rigid marker block
    thrust::device_vector<Real3> m_rigid_block_torques;  ///< Torque partial sums for each rigid marker block

    uint m_rigid_block_size;                                  ///< Block size for the rigid force accumulator kernel
    uint m_rigid_grid_size;                                   ///< Grid size for the rigid force accumulator kernel
    thrust::device_vector<uint> m_rigid_valid_threads;        ///< numbers of valid (non-padding) threads in the block
    thrust::device_vector<uint> m_rigid_accumulated_threads;  ///< accumulated numbers of padded threads before a block
    thrust::device_vector<uint> m_rigid_body_offsets;         ///< first reduction block of each body (numFsiBodies + 1 entries)

    // Node-to-marker maps for the flexible force gather, in CSR form: the contributions to node i are entries
    // node_offsets[i] to node_offsets[i+1]-1 of contributions, each encoded as marker * nodes_per_marker + local node
    // (2 nodes per marker in 1D, 3 in 2D) and kept in the original marker order.
    thrust::device_vector<uint> m_flex1D_node_offsets;   ///< first contribution of each 1D node (numFsiNodes1D + 1 entries)
    thrust::device_vector<uint> m_flex1D_contributions;  ///< encoded (marker, local node) pairs, grouped by 1D node
    thrust::device_vector<uint> m_flex2D_node_offsets;   ///< first contribution of each 2D node (numFsiNodes2D + 1 entries)
    thrust::device_vector<uint> m_flex2D_contributions;  ///< encoded (marker, local node) pairs, grouped by 2D node

    NodeDirections m_node_directions_mode;
    bool m_verbose;
    bool m_check_errors;
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
