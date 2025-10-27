// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Implementation of an FSI-aware TDPF fluid solver.
//
// =============================================================================

#ifndef CH_FLUID_SYSTEM_TDPF_H
#define CH_FLUID_SYSTEM_TDPF_H

#include "chrono_fsi/ChFsiFluidSystem.h"

namespace chrono {
namespace fsi {
namespace tdpf {

/// @addtogroup fsitdpf
/// @{

/// Physical system for an FSI-aware TDPF fluid solver.
class CH_FSI_API ChFsiFluidSystemTDPF : public ChFsiFluidSystem {
  public:
    ChFsiFluidSystemTDPF();
    ~ChFsiFluidSystemTDPF();

    /// Set gravity for the FSI syatem.
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) override;

    /// Return gravitational acceleration.
    ChVector3d GetGravitationalAcceleration() const;

  private:
    /// TDPF solver-specific actions taken when a rigid solid is added as an FSI object.
    virtual void OnAddFsiBody(std::shared_ptr<FsiBody> fsi_body, bool check_embedded) override;

    /// TDPF solver-specific actions taken when a 1D deformable solid is added as an FSI object.
    virtual void OnAddFsiMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh, bool check_embedded) override;

    /// TDPF solver-specific actions taken when a 2D deformable solid is added as an FSI object.
    virtual void OnAddFsiMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh, bool check_embedded) override;

    // ----------

    /// Initialize the TDPF fluid system with FSI support.
    virtual void Initialize(const std::vector<FsiBodyState>& body_states,
                            const std::vector<FsiMeshState>& mesh1D_states,
                            const std::vector<FsiMeshState>& mesh2D_states) override;

    // ----------

    /// Load the given body and mesh node states in the TDPF data manager structures.
    /// This function converts FEA mesh states from the provided AOS records to the SOA layout used by the TDPF data
    /// manager. LoadSolidStates is always called once during initialization. If the TDPF fluid solver is paired with
    /// the generic FSI interface, LoadSolidStates is also called from ChFsiInterfaceGeneric::ExchangeSolidStates at
    /// each co-simulation data exchange. If using a custom TDPF FSI interface, MBS states are copied directly...
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                 const std::vector<FsiMeshState>& mesh1D_states,
                                 const std::vector<FsiMeshState>& mesh2D_states) override;

    /// Store the body and mesh node forces from the TDPF data manager to the given vectors.
    /// If the TDPF fluid solver is paired with the generic FSI interface, StoreSolidForces is also called from
    /// ChFsiInterfaceGeneric::ExchangeSolidForces at each co-simulation data exchange. If using a custom TDPF FSI
    /// interface, MBS forces are copied directly...
    virtual void StoreSolidForces(std::vector<FsiBodyForce> body_forces,
                                  std::vector<FsiMeshForce> mesh1D_forces,
                                  std::vector<FsiMeshForce> mesh2D_forces) override;

    // ----------

    /// Function to integrate the fluid system from `time` to `time + step`.
    virtual void OnDoStepDynamics(double time, double step) override;

    /// Additional actions taken before applying fluid forces to the solid phase.
    virtual void OnExchangeSolidForces() override;

    /// Additional actions taken after loading new solid phase states.
    virtual void OnExchangeSolidStates() override;

    // ----------

    ChVector3d m_g;

    unsigned int m_num_rigid_bodies;     ///< number of rigid bodies
    unsigned int m_num_flex1D_nodes;     ///< number of 1-D flexible nodes (across all meshes)
    unsigned int m_num_flex2D_nodes;     ///< number of 2-D flexible nodes (across all meshes)
    unsigned int m_num_flex1D_elements;  ///< number of 1-D flexible segments (across all meshes)
    unsigned int m_num_flex2D_elements;  ///< number of 2-D flexible faces (across all meshes)
};

/// @} fsitdpf

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono

#endif
