// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// 1. Definition of the base class for interfacing between a Chrono system and
//    an FSI-aware fluid system.
// 2. Implementation of a generic FSI interface that relies on copying data to
//    intermediate buffers.
//
// =============================================================================

#ifndef CH_FSI_INTERFACE_H
#define CH_FSI_INTERFACE_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiDefinitions.h"
#include "chrono_fsi/ChFsiFluidSystem.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_base
/// @{

// =============================================================================

/// Base class for interfacing between a Chrono system and an FSI-aware fluid system.
/// This class provides the functionality of extracting state information from the Chrono MBS and loading fluid forces
/// acting on the MBS solids. This base class also provides utilities for allocating the exchange data structures,
/// verifying sizes of the exchange data vectors, and a default implementation for calculating FEA node direction
/// vectors. A derived class must implement the actual functions for exchanging data between the two systems.
class CH_FSI_API ChFsiInterface {
  public:
    virtual ~ChFsiInterface();

    void SetVerbose(bool verbose) { m_verbose = verbose; }

    void AttachMultibodySystem(ChSystem* sys);

    // ------------

    /// Add a rigid body.
    /// The fluid-solid interaction is based on the provided rigid geometry.
    /// If geometry=nullptr, it is assumed that the interaction geometry is provided separately.
    std::shared_ptr<FsiBody> AddFsiBody(std::shared_ptr<ChBody> body,
                                        std::shared_ptr<utils::ChBodyGeometry> geometry,
                                        bool check_embedded);

    /// Add a flexible solid with segment set contact to the FSI system.
    std::shared_ptr<FsiMesh1D> AddFsiMesh1D(std::shared_ptr<fea::ChContactSurfaceSegmentSet> surface,
                                            bool check_embedded);

    /// Add a flexible solid with surface mesh contact to the FSI system.
    std::shared_ptr<FsiMesh2D> AddFsiMesh2D(std::shared_ptr<fea::ChContactSurfaceMesh> surface,
                                            bool check_embedded);

    /// Initialize the FSI interface.
    virtual void Initialize();

    // ------------

    /// Get the number of FSI bodies.
    unsigned int GetNumBodies() const;

    /// Get the number of FSI 1-D meshes.
    unsigned int GetNumMeshes1D() const;

    /// Get the number of FSI 1-D mesh elements (segments).
    unsigned int GetNumElements1D() const;

    /// Get the number of FSI 1-D mesh nodes.
    unsigned int GetNumNodes1D() const;

    /// Get the number of FSI 2-D meshes.
    unsigned int GetNumMeshes2D() const;

    /// Get the number of FSI 2-D mesh elements (segments).
    unsigned int GetNumElements2D() const;

    /// Get the number of FSI 2-D mesh nodes.
    unsigned int GetNumNodes2D() const;

    // ------------

    /// Get the set of bodies added to the FSI interface.
    const std::vector<std::shared_ptr<FsiBody>>& GetBodies() const { return m_fsi_bodies; }
    
    /// Get the set of 1D meshes added to the FSI interface.
    const std::vector<std::shared_ptr<FsiMesh1D>>& GetMeshes1D() const { return m_fsi_meshes1D; }
    
    /// Get the set of 2D meshes added to the FSI interface.
    const std::vector<std::shared_ptr<FsiMesh2D>>& GetMeshes2D() const { return m_fsi_meshes2D; }

    // ------------

    /// Return the FSI applied force on the body with specified index.
    /// The force is applied at the body COM and is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyForce(size_t i) const;

    /// Return the FSI applied torque on the body with specified index.
    /// The torque is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyTorque(size_t i) const;

    // ------------

    /// Enable use and set method of obtaining FEA node directions. Default: NodeDirectionsMode::NONE.
    void UseNodeDirections(NodeDirectionsMode mode) { m_node_directions_mode = mode; }

    /// Utility function to allocate state vectors.
    /// If use of node directions is enabled, also resize the vectors of node directions for FSI meshes.
    void AllocateStateVectors(std::vector<FsiBodyState>& body_states,
                              std::vector<FsiMeshState>& mesh1D_states,
                              std::vector<FsiMeshState>& mesh2D_states) const;

    /// Utility function to allocate force vectors.
    void AllocateForceVectors(std::vector<FsiBodyForce>& body_forces,
                              std::vector<FsiMeshForce>& mesh_forces1D,
                              std::vector<FsiMeshForce>& mesh_forces2D) const;

    /// Utility function to check sizes of state vectors.
    bool CheckStateVectors(const std::vector<FsiBodyState>& body_states,
                           const std::vector<FsiMeshState>& mesh1D_states,
                           const std::vector<FsiMeshState>& mesh2D_states) const;

    /// Utility function to check sizes of force vectors.
    bool CheckForceVectors(const std::vector<FsiBodyForce>& body_forces,
                           const std::vector<FsiMeshForce>& mesh_forces1D,
                           const std::vector<FsiMeshForce>& mesh_forces2D) const;

    /// Utility function to get current solid phase states from the multibody system in the provided structures.
    /// This function is always called once during initialization of the containing ChFsiSystem.
    /// During simulation, a derived class may use this function in its implementation of `ExchangeSolidStates`.
    /// A runtime exception is thrown if the output vectors do not have the appropriate sizes.
    void StoreSolidStates(std::vector<FsiBodyState>& body_states,
                          std::vector<FsiMeshState>& mesh1D_states,
                          std::vector<FsiMeshState>& mesh2D_states);

    /// Utility function to apply forces in the provided structures to the multibody system.
    /// A derived class may use this function in its implementation of `ExchangeSolidForces`.
    /// A runtime exception is thrown if the output vectors do not have the appropriate sizes.
    void LoadSolidForces(std::vector<FsiBodyForce>& body_forces,
                         std::vector<FsiMeshForce>& mesh1D_forces,
                         std::vector<FsiMeshForce>& mesh2D_forces);

    // ------------

    /// Exchange solid phase state information between the MBS and fluid system.
    /// - Extract FSI body states from MBS and apply them to the fluid system.
    /// - Extract FSI mesh node states from MBS and apply them to the fluid system.
    virtual void ExchangeSolidStates() = 0;

    /// Exchange solid phase force information between the multibody and fluid systems.
    /// - Extract fluid forces on rigid bodies from fluid system and apply them as external loads to the MBS.
    /// - Extract fluid forces on mesh nodes from fluid system and apply them as external loads to the MBS.
    virtual void ExchangeSolidForces() = 0;

  protected:
    ChFsiInterface(ChSystem* sysMBS, ChFsiFluidSystem* sysCFD);

    bool m_verbose;
    bool m_initialized;
    NodeDirectionsMode m_node_directions_mode;
    ChSystem* m_sysMBS;
    ChFsiFluidSystem* m_sysCFD;

    std::vector<std::shared_ptr<FsiBody>> m_fsi_bodies;      ///< rigid bodies exposed to the FSI system
    std::vector<std::shared_ptr<FsiMesh1D>> m_fsi_meshes1D;  ///< FEA meshes with 1-D segments exposed to the FSI system
    std::vector<std::shared_ptr<FsiMesh2D>> m_fsi_meshes2D;  ///< FEA meshes with 2-D faces exposed to the FSI system
};

// =============================================================================

/// Generic FSI interface between a Chrono multibody system and a fluid system.
/// This implementation relies on copying data to intermediate buffers.
class CH_FSI_API ChFsiInterfaceGeneric : public ChFsiInterface {
  public:
    ChFsiInterfaceGeneric(ChSystem* sysMBS, ChFsiFluidSystem* sysCFD);
    ~ChFsiInterfaceGeneric();

    /// Initialize the generic FSI interface.
    virtual void Initialize() override;

  private:
    /// Exchange solid phase state information between the MBS and fluid system.
    /// The implementation of the generic FSI interface uses intermediate buffers for the body and flex mesh states.
    virtual void ExchangeSolidStates() override;

    /// Exchange solid phase force information between the multibody and fluid systems.
    /// The implementation of the generic FSI interface uses intermediate buffers for the body and nodal forces.
    virtual void ExchangeSolidForces() override;

    std::vector<FsiBodyState> m_body_states;
    std::vector<FsiBodyForce> m_body_forces;
    std::vector<FsiMeshState> m_mesh1D_states;
    std::vector<FsiMeshForce> m_mesh1D_forces;
    std::vector<FsiMeshState> m_mesh2D_states;
    std::vector<FsiMeshForce> m_mesh2D_forces;
};

// =============================================================================

/// @} fsi_base

}  // end namespace fsi
}  // end namespace chrono

#endif
