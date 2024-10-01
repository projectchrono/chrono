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
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceSegmentSet.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"
#include "chrono_fsi/physics/ChFsiBase.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for processing the interface between Chrono and FSI modules.
class ChFsiInterface : public ChFsiBase {
  public:
    /// Constructor of the FSI interface class.
    ChFsiInterface(ChSystemFsi_impl& fsi, std::shared_ptr<SimParams> params);

    /// Destructor of the FSI interface class.
    ~ChFsiInterface();

    /// Copy rigid body states from ChSystem to FsiSystem, then to the GPU memory.
    void LoadBodyState_Chrono2Fsi(std::shared_ptr<FsiBodyStateD> fsiBodyStateD);

    /// Copy FEA mesh states from ChSystem to FsiSystem, then to the GPU memory.
    void LoadMesh1DState_Chrono2Fsi(std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D);
    void LoadMesh2DState_Chrono2Fsi(std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D);

    /// Read the surface-integrated pressure and viscous forces form the fluid/granular dynamics system,
    /// and add these forces and torques as external forces to the ChSystem rigid bodies.
    void ApplyBodyForce_Fsi2Chrono();

    /// Add forces and torques as external forces to the ChSystem flexible bodies.
    void ApplyMesh1DForce_Fsi2Chrono();
    void ApplyMesh2DForce_Fsi2Chrono();

  private:
    /// Description of a rigid body exposed to the FSI system.
    struct FsiBody {
        std::shared_ptr<ChBody> body;  ///< rigid body exposed to FSI system
        ChVector3d fsi_force;          ///< fluid force at body COM (expressed in absolute frame)
        ChVector3d fsi_torque;         ///< induced torque (expressed in absolute frame)
    };

    /// Description of an FEA mesh with 1-D segments exposed to the FSI system.
    struct FsiMesh1D {
        std::shared_ptr<fea::ChContactSurfaceSegmentSet> contact_surface;  ///< FEA contact segments
        std::map<std::shared_ptr<fea::ChNodeFEAxyz>, int> ptr2ind_map;     ///< pointer-based to index-based mapping
        std::map<int, std::shared_ptr<fea::ChNodeFEAxyz>> ind2ptr_map;     ///< index-based to pointer-based mapping
        int num_bce;                                                       ///< number of BCE markers for this mesh
    };

    /// Description of an FEA mesh with 2-D faces exposed to the FSI system.
    struct FsiMesh2D {
        std::shared_ptr<fea::ChContactSurfaceMesh> contact_surface;     ///< FEA trimesh skin
        std::map<std::shared_ptr<fea::ChNodeFEAxyz>, int> ptr2ind_map;  ///< pointer-based to index-based mapping
        std::map<int, std::shared_ptr<fea::ChNodeFEAxyz>> ind2ptr_map;  ///< index-based to pointer-based mapping
        int num_bce;                                                    ///< number of BCE markers for this mesh
    };

    ChSystemFsi_impl& m_sysFSI;  ///< FSI system
    bool m_verbose;              ///< terminal output (default: true)

    std::vector<FsiBody> m_fsi_bodies;      ///< rigid bodies exposed to the FSI system
    std::vector<FsiMesh1D> m_fsi_meshes1D;  ///< FEA meshes with 1-D segments exposed to the FSI system
    std::vector<FsiMesh2D> m_fsi_meshes2D;  ///< FEA meshes with 2-D faces exposed to the FSI system

    friend class ChSystemFsi;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
