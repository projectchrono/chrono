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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================
//
// Base class for processing the interface between Chrono and FSI modules
// =============================================================================
#ifndef CH_FSI_INTERFACE_H
#define CH_FSI_INTERFACE_H

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChFsiGeneral.h"

namespace chrono {

// Forward declarations
namespace fea {
class ChNodeFEAxyzD;
class ChMesh;
class ChElementCableANCF;
class ChElementShellANCF_3423;
}

namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for processing the interface between Chrono and FSI modules.
class ChFsiInterface : public ChFsiGeneral {
  public:
    /// Constructor of the FSI interface class.
    ChFsiInterface(ChSystem& other_mphysicalSystem,
                   std::shared_ptr<fea::ChMesh> other_fsiMesh,
                   std::shared_ptr<SimParams> other_paramsH,
                   std::shared_ptr<FsiBodiesDataH> other_fsiBodiesH,
                   std::shared_ptr<FsiMeshDataH> other_fsiMeshH,
                   std::vector<std::shared_ptr<ChBody>>& other_fsiBodies,
                   std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& other_fsiNodes,
                   std::vector<std::shared_ptr<fea::ChElementCableANCF>>& other_fsiCables,
                   std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& other_fsiShells,
                   thrust::host_vector<int2>& other_CableElementsNodesH,
                   thrust::device_vector<int2>& other_CableElementsNodes,
                   thrust::host_vector<int4>& other_ShellElementsNodesH,
                   thrust::device_vector<int4>& other_ShellElementsNodes,
                   thrust::device_vector<Real3>& other_rigid_FSI_ForcesD,
                   thrust::device_vector<Real3>& other_rigid_FSI_TorquesD,
                   thrust::device_vector<Real3>& other_Flex_FSI_ForcesD);

    /// Destructor of the FSI interface class.
    ~ChFsiInterface();

    /// Read the surface-integrated pressure and viscous forces form the fluid/granular dynamics system, 
    /// and add these forces and torques as external forces to the ChSystem rigid bodies.
    void Add_Rigid_ForceTorques_To_ChSystem();

    /// Use an external configuration to set the generalized coordinates of the ChSystem.
    void Copy_External_To_ChSystem();

    /// Use the generalized coordinates of the ChSystem to set the configuration state in the FSI system.
    void Copy_ChSystem_to_External();

    /// Copy the ChSystem rigid bodies from CPU to GPU.
    void Copy_fsiBodies_ChSystem_to_FluidSystem(std::shared_ptr<FsiBodiesDataD> fsiBodiesD);

    /// Resize the number of ChSystem rigid bodies.
    void ResizeChronoBodiesData();

    /// Set the FSI mesh for flexible elements.
    void SetFsiMesh(std::shared_ptr<fea::ChMesh> other_fsi_mesh) { fsi_mesh = other_fsi_mesh; };

    /// Add forces and torques as external forces to the ChSystem flexible bodies.
    void Add_Flex_Forces_To_ChSystem();

    /// Resize number of nodes used in the flexible elements
    void ResizeChronoNodesData();

    /// Resize number of cable elements used in the flexible elements
    void ResizeChronoCablesData(std::vector<std::vector<int>> CableElementsNodesSTDVector,
                                        thrust::host_vector<int2>& CableElementsNodesH);

    /// Resize number of shell elements used in the flexible elements
    void ResizeChronoShellsData(std::vector<std::vector<int>> ShellElementsNodesSTDVector,
                                        thrust::host_vector<int4>& ShellElementsNodesH);

    /// Resize number of nodes used in the flexible elements
    void ResizeChronoFEANodesData();

    /// Copy the nodes information in ChSystem from CPU to GPU.
    void Copy_fsiNodes_ChSystem_to_FluidSystem(std::shared_ptr<FsiMeshDataD> FsiMeshD);

  private:
    ChSystem& mphysicalSystem;                              ///< Chrono system handled by the FSI system
    std::shared_ptr<FsiBodiesDataH> fsiBodiesH;             ///< states of the FSI rigid bodies
    std::shared_ptr<ChronoBodiesDataH> chronoRigidBackup;   ///< backup for the Chrono system state
    std::shared_ptr<FsiMeshDataH> fsiMeshH;                 ///< information of the FEA mesh participating in FSI
    std::shared_ptr<ChronoMeshDataH> chronoFlexMeshBackup;  ///< backup for the Chrono system state
    std::shared_ptr<SimParams> paramsH;                     ///< simulation parameters

    thrust::device_vector<Real3>& rigid_FSI_ForcesD;   ///< forces from the fluid dynamics system to rigid bodies
    thrust::device_vector<Real3>& rigid_FSI_TorquesD;  ///< torques from the fluid dynamics system to rigid bodies
    thrust::device_vector<Real3>& Flex_FSI_ForcesD;    ///< forces from the fluid dynamics system to flexible bodies

    std::shared_ptr<fea::ChMesh> fsi_mesh;
    std::vector<std::shared_ptr<ChBody>>& fsiBodies;                        ///< bodies handled by the FSI system
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& fsiNodes;             ///< FEA nodes available in FSI system
    std::vector<std::shared_ptr<fea::ChElementCableANCF>>& fsiCables;       ///< FEA cable elements in FSI system
    std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& fsiShells;  ///< FEA shell elements in FSI system
    thrust::host_vector<int2>& CableElementsNodesH;                         ///< indices of nodes of each element
    thrust::device_vector<int2>& CableElementsNodes;                        ///< indices of nodes of each element
    thrust::host_vector<int4>& ShellElementsNodesH;                         ///< indices of nodes of each element
    thrust::device_vector<int4>& ShellElementsNodes;                        ///< indices of nodes of each element
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
