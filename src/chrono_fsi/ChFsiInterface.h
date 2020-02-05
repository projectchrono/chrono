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
// Base class for processing the interface between chrono and fsi modules
// =============================================================================
#ifndef CH_FSIINTERFACE_H_
#define CH_FSIINTERFACE_H_


#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/ChFsiGeneral.cuh"

namespace chrono {

// Forward declaration
namespace fea {
class ChNodeFEAxyzD;
class ChMesh;
class ChElementCableANCF;
class ChElementShellANCF;
}  // namespace fea

namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for processing the interface between chrono and fsi modules
class CH_FSI_API ChFsiInterface : public ChFsiGeneral {
  public:
    ChFsiInterface(SimParams* other_paramsH,
                   FsiBodiesDataH* other_fsiBodiesH,
                   chrono::ChSystem* other_mphysicalSystem,
                   std::vector<std::shared_ptr<chrono::ChBody>>* other_fsiBodeisPtr,
                   thrust::host_vector<int2>* other_CableElementsNodesH,
                   thrust::device_vector<int2>* other_CableElementsNodes,
                   thrust::host_vector<int4>* other_ShellElementsNodesH,
                   thrust::device_vector<int4>* other_ShellElementsNodes,
                   thrust::device_vector<Real3>* other_rigid_FSI_ForcesD,
                   thrust::device_vector<Real3>* other_rigid_FSI_TorquesD,
                   thrust::device_vector<Real3>* other_Flex_FSI_ForcesD);

    /// Destructor of the FSI interface.
    ~ChFsiInterface();

    /// Reads the surface-integrated pressure and viscous forces form the fluid dynamics system, and add these forces
    /// and torques as external forces to the ChSystem bodies.
    virtual void Add_Rigid_ForceTorques_To_ChSystem();

    /// Uses an external configuration to set the generalized coordinates of the ChSystem.
    virtual void Copy_External_To_ChSystem();

    /// Uses the generalized coordinates of the ChSystem to set the configuration state in the FSI system.
    virtual void Copy_ChSystem_to_External();
    virtual void Copy_fsiBodies_ChSystem_to_FluidSystem(FsiBodiesDataD* fsiBodiesD);
    virtual void ResizeChronoBodiesData();
    ChFsiInterface(SimParams* other_paramsH,
                   FsiBodiesDataH* other_fsiBodiesH,
                   FsiMeshDataH* other_fsiMeshH,
                   chrono::ChSystem* other_mphysicalSystem,
                   std::vector<std::shared_ptr<chrono::ChBody>>* other_fsiBodeisPtr,
                   std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>* other_fsiNodesPtr,
                   std::vector<std::shared_ptr<fea::ChElementCableANCF>>* other_fsiCablesPtr,
                   std::vector<std::shared_ptr<fea::ChElementShellANCF>>* other_fsiShellsPtr,
                   std::shared_ptr<chrono::fea::ChMesh> other_fsiMesh,
                   thrust::host_vector<int2>* other_CableElementsNodesH,
                   thrust::device_vector<int2>* other_CableElementsNodes,
                   thrust::host_vector<int4>* other_ShellElementsNodesH,
                   thrust::device_vector<int4>* other_ShellElementsNodes,
                   thrust::device_vector<Real3>* other_rigid_FSI_ForcesD,
                   thrust::device_vector<Real3>* other_rigid_FSI_TorquesD,
                   thrust::device_vector<Real3>* other_Flex_FSI_ForcesD);
    virtual void SetFsiMesh(std::shared_ptr<chrono::fea::ChMesh> other_fsi_mesh) { fsi_mesh = other_fsi_mesh; };
    virtual void Add_Flex_Forces_To_ChSystem();
    virtual void ResizeChronoNodesData();
    virtual void ResizeChronoCablesData(std::vector<std::vector<int>> CableElementsNodesSTDVector,
                                        thrust::host_vector<int2>* CableElementsNodesH);
    virtual void ResizeChronoShellsData(std::vector<std::vector<int>> ShellElementsNodesSTDVector,
                                        thrust::host_vector<int4>* ShellElementsNodesH);
    virtual void ResizeChronoFEANodesData();
    virtual void Copy_fsiNodes_ChSystem_to_FluidSystem(FsiMeshDataD* FsiMeshD);

  private:
    FsiBodiesDataH* fsiBodiesH;  ///< State of the FSI rigid bodies at the position, velocity and acceleration level.
    ChronoBodiesDataH* chronoRigidBackup;  ///< A backup data structure to save the state of the chrono system
    chrono::ChSystem* mphysicalSystem;     ///< Pointer to the Chrono system handled by the FSI system.

    std::vector<std::shared_ptr<chrono::ChBody>>*
        fsiBodeisPtr;  ///< Pointer to the vector of the ChBody shared pointers handled by the FSI system.

    thrust::device_vector<Real3>*
        rigid_FSI_ForcesD;  ///< Surface-integrated forces from the fluid dynamics system to rigid bodies.
    thrust::device_vector<Real3>*
        rigid_FSI_TorquesD;  ///< Surface-integrated torques from the fluid dynamics system to rigid bodies.

    std::shared_ptr<chrono::fea::ChMesh> fsi_mesh;                  ///< These are all the nodes available to fsi
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>* fsiNodesPtr;  ///<  These are all the FE nodes available to fsi
    std::vector<std::shared_ptr<fea::ChElementCableANCF>>*
        fsiCablesPtr;  ///< Pointer to the vector of ChElementCableANCF participating in FSI
    std::vector<std::shared_ptr<fea::ChElementShellANCF>>*
        fsiShellsPtr;  ///< Pointer to the vector of ChElementShellANCF participating in FSI

    thrust::host_vector<int2>* CableElementsNodesH;   ///< These are the indices of nodes of each Element
    thrust::device_vector<int2>* CableElementsNodes;  ///< These are the indices of nodes of each Element
    thrust::host_vector<int4>* ShellElementsNodesH;   ///< These are the indices of nodes of each Element
    thrust::device_vector<int4>* ShellElementsNodes;  ///< These are the indices of nodes of each Element
    thrust::device_vector<Real3>* Flex_FSI_ForcesD;
    FsiMeshDataH* fsiMeshH;
    ChronoMeshDataH* chronoFlexMeshBackup;  ///< A backup data structure to save the state of the chrono system
    SimParams* paramsH;                     ///< pointer to the simulation parameters
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
