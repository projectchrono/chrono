// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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
// Definition of an FSI-aware TDPF fluid solver.
//
// =============================================================================

#ifndef CH_FLUID_SYSTEM_TDPF_H
#define CH_FLUID_SYSTEM_TDPF_H

#include "chrono_fsi/ChFsiFluidSystem.h"

#include "chrono_fsi/tdpf/ChFsiTdpfTypes.h"

namespace chrono {
namespace fsi {
namespace tdpf {

// Forward declaration of TDPF implementation
class ChFsiFluidSystemTDPF_impl;

/// @addtogroup fsitdpf
/// @{

/// Physical system for an FSI-aware TDPF fluid solver.
class CH_FSI_API ChFsiFluidSystemTDPF : public ChFsiFluidSystem {
  public:
    ChFsiFluidSystemTDPF();
    ~ChFsiFluidSystemTDPF();

    /// Set input file name with hydro data (HDF5 format).
    void SetHydroFilename(const std::string& filename);

    /// Set gravity for the FSI system.
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) override;

    /// Return gravitational acceleration.
    ChVector3d GetGravitationalAcceleration() const;

    /// Set the sea state (still water, regular, or irregular waves).
    /// Note that the number of bodies is set during initialization.
    void SetSeaState(const ChTdpfSeaState& sea_state);

    /// Select the radiation damping method (default: RIRF convolution).
    void SetRadiationMethod(ChTdpfRadiationMethod method);

    /// Configure RIRF kernel processing (smoothing / tapering).
    /// Only applies with ChTdpfRadiationMethod::RIRF_CONVOLUTION.
    void SetRadiationKernelProcessing(const ChTdpfRadiationKernelProcessing& opts);

    /// Configure state-space fitting parameters.
    /// Only applies with ChTdpfRadiationMethod::STATE_SPACE.
    void SetStateSpaceOptions(const ChTdpfStateSpaceOptions& opts);

    /// Select the wave excitation force method (default: automatic).
    void SetExcitationMethod(ChTdpfExcitationMethod method);

    /// Select the excitation transfer function interpolation method.
    void SetExcitationInterpolation(ChTdpfExcitationInterpolation interp);

    /// Set the excitation ramp duration [s]. 0 = no ramp.
    void SetRampDuration(double seconds);

    /// Truncate the radiation RIRF to [0, T] seconds. 0 = use full RIRF.
    void SetRadiationTruncationTime(double seconds);

    /// Truncate the excitation IRF to [-T, T] seconds. 0 = use full IRF.
    void SetExcitationTruncationTime(double seconds);

    /// Set the directory for diagnostics output (kernel CSVs, etc.).
    void SetDiagnosticsOutputDir(const std::string& dir);

    /// Get current wave elevation at specified position (in X-Y plane).
    double GetWaveElevation(const ChVector3d& pos);

    /// Get current wave velocity at specified position (in X-Y plane).
    ChVector3d GetWaveVelocity(const ChVector3d& pos);

    /// Get current wave velocity at specified position (in X-Y plane).
    ChVector3d GetWaveVelocity(const ChVector3d& pos, double elevation);

public:
    /// Load the given body and mesh node states in the TDPF data manager structures.
    /// This function converts FEA mesh states from the provided AOS records to the SOA layout used by the TDPF data
    /// manager. LoadSolidStates is always called once during initialization. If the TDPF fluid solver is paired with
    /// the generic FSI interface, LoadSolidStates is also called from ChFsiInterfaceGeneric::ExchangeSolidStates at
    /// each co-simulation data exchange. If using a custom TDPF FSI interface, MBS states are copied directly...
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states) override;

    /// Store the body and mesh node forces from the TDPF data manager to the given vectors.
    /// If the TDPF fluid solver is paired with the generic FSI interface, StoreSolidForces is called from
    /// ChFsiInterfaceGeneric::ExchangeSolidForces at each co-simulation data exchange. If using a custom TDPF FSI
    /// interface, MBS forces are copied directly...
    virtual void StoreSolidForces(std::vector<FsiBodyForce>& body_forces) override;

    #ifdef CHRONO_FEA
    /// Load the given body and mesh node states in the TDPF data manager structures.
    /// This function converts FEA mesh states from the provided AOS records to the SOA layout used by the TDPF data
    /// manager. LoadSolidStates is always called once during initialization. If the TDPF fluid solver is paired with
    /// the generic FSI interface, LoadSolidStates is also called from ChFsiInterfaceGeneric::ExchangeSolidStates at
    /// each co-simulation data exchange. If using a custom TDPF FSI interface, MBS states are copied directly...
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                 const std::vector<FsiMeshState>& mesh1D_states,
                                 const std::vector<FsiMeshState>& mesh2D_states) override;

    /// Store the body and mesh node forces from the TDPF data manager to the given vectors.
    /// If the TDPF fluid solver is paired with the generic FSI interface, StoreSolidForces is called from
    /// ChFsiInterfaceGeneric::ExchangeSolidForces at each co-simulation data exchange. If using a custom TDPF FSI
    /// interface, MBS forces are copied directly...
    virtual void StoreSolidForces(std::vector<FsiBodyForce>& body_forces, std::vector<FsiMeshForce>& mesh1D_forces, std::vector<FsiMeshForce>& mesh2D_forces) override;
#endif

    /// Function to integrate the fluid system from `time` to `time + step`.
    virtual void OnDoStepDynamics(double time, double step) override;

    /// Additional actions taken before applying fluid forces to the solid phase.
    virtual void OnExchangeSolidForces() override;

    /// Additional actions taken after loading new solid phase states.
    virtual void OnExchangeSolidStates() override;

  private:
    // ----------

    /// TDPF solver-specific actions taken when a rigid solid is added as an FSI object.
    virtual void OnAddRigidBody(std::shared_ptr<FsiBody> fsi_body, bool check_embedded) override;

    /// Initialize the TDPF fluid system with FSI support.
    virtual void Initialize(const std::vector<FsiBodyState>& body_states) override;

#ifdef CHRONO_FEA
    /// TDPF solver-specific actions taken when a 1D deformable solid is added as an FSI object.
    virtual void OnAddFeaMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh, bool check_embedded) override;

    /// TDPF solver-specific actions taken when a 2D deformable solid is added as an FSI object.
    virtual void OnAddFeaMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh, bool check_embedded) override;

    /// Initialize the TDPF fluid system with FSI support.
    virtual void Initialize(const std::vector<FsiBodyState>& body_states, const std::vector<FsiMeshState>& mesh1D_states, const std::vector<FsiMeshState>& mesh2D_states) override;
#endif

    // ----------

    unsigned int m_num_rigid_bodies;     ///< number of rigid bodies
    unsigned int m_num_1D_meshes;        ///< number of 1-D meshes
    unsigned int m_num_flex1D_nodes;     ///< number of 1-D flexible nodes (across all meshes)
    unsigned int m_num_flex1D_elements;  ///< number of 1-D flexible segments (across all meshes)
    unsigned int m_num_2D_meshes;        ///< number of 2-D meshes
    unsigned int m_num_flex2D_nodes;     ///< number of 2-D flexible nodes (across all meshes)
    unsigned int m_num_flex2D_elements;  ///< number of 2-D flexible faces (across all meshes)

    std::string m_hydro_filename;                       ///< input hydro file name (HDF5 format)
    std::unique_ptr<ChFsiFluidSystemTDPF_impl> m_impl;  ///< private implementation

    friend class ChFsiSystemTDPF;
    friend class ChFsiInterfaceTDPF;
    friend class FSITDPFStatsVSG;
};

/// @} fsitdpf

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono

#endif
