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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Base class for processing the interface between Chrono and FSI modules
// =============================================================================
#ifndef CH_FSI_INTERFACE_H
#define CH_FSI_INTERFACE_H

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChFsiGeneral.h"

namespace chrono {

// Forward declarations
namespace fea {
class ChNodeFEAxyzD;
class ChMesh;
class ChElementCableANCF;
class ChElementShellANCF_3423;
}  // namespace fea

namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for processing the interface between Chrono and FSI modules.
class ChFsiInterface : public ChFsiGeneral {
  public:
    /// Constructor of the FSI interface class.
    ChFsiInterface(ChSystemFsi_impl& fsi,
                   std::shared_ptr<SimParams> params);

    /// Destructor of the FSI interface class.
    ~ChFsiInterface();

    /// Read the surface-integrated pressure and viscous forces form the fluid/granular dynamics system,
    /// and add these forces and torques as external forces to the ChSystem rigid bodies.
    void Add_Rigid_ForceTorques_To_ChSystem();

    /// Copy rigid bodies' information from ChSystem to FsiSystem, then to the GPU memory.
    void Copy_FsiBodies_ChSystem_to_FsiSystem(std::shared_ptr<FsiBodiesDataD> fsiBodiesD);

    /// Add forces and torques as external forces to the ChSystem flexible bodies.
    void Add_Flex_Forces_To_ChSystem();

    /// Resize number of cable elements used in the flexible elements.
    void ResizeChronoCablesData(const std::vector<std::vector<int>>& CableElementsNodesSTDVector);

    /// Resize number of shell elements used in the flexible elements.
    void ResizeChronoShellsData(const std::vector<std::vector<int>>& ShellElementsNodesSTDVector);

    /// Copy flexible nodes' information from ChSystem to FsiSystem, then to the GPU memory.
    void Copy_FsiNodes_ChSystem_to_FsiSystem(std::shared_ptr<FsiMeshDataD> FsiMeshD);

  private:
    ChSystemFsi_impl& m_sysFSI;            ///< FSI system
    std::shared_ptr<SimParams> m_paramsH;  ///< simulation parameters
    bool m_verbose;                        ///< enable/disable m_verbose terminal output (default: true)

    std::shared_ptr<fea::ChMesh> m_fsi_mesh;
    std::vector<std::shared_ptr<ChBody>> m_fsi_bodies;             ///< bodies handled by the FSI system
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> m_fsi_nodes;  ///< FEA nodes available in FSI system

    friend class ChSystemFsi;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
