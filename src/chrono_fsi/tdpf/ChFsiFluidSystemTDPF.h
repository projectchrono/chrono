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
// Implementation of an FSI-aware TDPF fluid solver.
//
// =============================================================================

#ifndef CH_FLUID_SYSTEM_TDPF_H
#define CH_FLUID_SYSTEM_TDPF_H

#include "chrono_fsi/ChFsiFluidSystem.h"

// HydroChrono
#include "hydroc/core/hydro_types.h"
#include "hydroc/core/system_state.h"
#include "hydroc/core/hydro_forces.h"

#include "hydro/force_components/excitation_component.h"
#include "hydro/force_components/hydrostatics_component.h"
#include "hydro/force_components/radiation_component.h"

#include "hydroc/io/h5_reader.h"
#include "hydroc/waves/regular_wave.h"
#include "hydroc/waves/irregular_wave.h"

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

    /// Set input file name with hydro data (HDF5 format).
    void SetHydroFilename(const std::string& filename);

    /// Set gravity for the FSI syatem.
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) override;

    /// Return gravitational acceleration.
    ChVector3d GetGravitationalAcceleration() const;

    void SetRadiationConvolutionMode(hydrochrono::hydro::RadiationConvolutionMode mode);
    void SetTaperedDirectOptions(const hydrochrono::hydro::TaperedDirectOptions& opts);

    /// Add no wave conditions.
    void AddWaves(const NoWaveParams& params);

    /// Add regular wave conditions.
    void AddWaves(const RegularWaveParams& params);

    /// Add irregular wave conditions.
    void AddWaves(const IrregularWaveParams& params);

    /// Get current wave elevation at specified position (in X-Y plane).
    double GetWaveElevation(const ChVector3d& pos);

    /// Get current wave velocity at specified position (in X-Y plane).
    ChVector3d GetWaveVelocity(const ChVector3d& pos);

    /// Get current wave velocity at specified position (in X-Y plane).
    ChVector3d GetWaveVelocity(const ChVector3d& pos, double elevation);

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
    unsigned int m_num_1D_meshes;        ///< number of 1-D meshes
    unsigned int m_num_flex1D_nodes;     ///< number of 1-D flexible nodes (across all meshes)
    unsigned int m_num_flex1D_elements;  ///< number of 1-D flexible segments (across all meshes)
    unsigned int m_num_2D_meshes;        ///< number of 2-D meshes
    unsigned int m_num_flex2D_nodes;     ///< number of 2-D flexible nodes (across all meshes)
    unsigned int m_num_flex2D_elements;  ///< number of 2-D flexible faces (across all meshes)

    std::string m_hydro_filename;  ///< input hydro file name (HDF5 format)
    HydroData m_hydro_data;        ///< data read from HDF5 file

    // Additional properties related to equilibrium and hydrodynamics
    std::vector<double> m_equilibrium;
    std::vector<double> m_cb_minus_cg;
    Eigen::VectorXd m_rirf_time_vector;
    Eigen::VectorXd m_rirf_width_vector;

    hydrochrono::hydro::RadiationConvolutionMode m_convolution_mode;  ///< default: baseline
    hydrochrono::hydro::TaperedDirectOptions m_tapered_opts;
    std::string m_diagnostics_output_dir;

    std::shared_ptr<WaveBase> m_waves;  ///< default: no wave

    // -----------

    hydrochrono::hydro::SystemState m_hc_state;
    hydrochrono::hydro::BodyForces m_hc_forces;
    std::unique_ptr<hydrochrono::hydro::HydroForces> m_hc_force_system;

    // ------------

    std::unique_ptr<hydrochrono::hydro::ExcitationComponent> CreateExcitationComponent() const;
    std::unique_ptr<hydrochrono::hydro::HydrostaticsComponent> CreateHydrostaticsComponent() const;
    std::unique_ptr<hydrochrono::hydro::RadiationComponent> CreateRadiationComponent() const;

    friend class ChFsiSystemTDPF;
    friend class ChFsiInterfaceTDPF;
    friend class FSITDPFStatsVSG;
};

/// @} fsitdpf

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono

#endif
