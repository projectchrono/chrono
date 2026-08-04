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
// Authors: Radu Serban
// =============================================================================

#ifndef CH_PRECICE_ADAPTER_TDPF_H
#define CH_PRECICE_ADAPTER_TDPF_H

#include "chrono_precice/ChPreciceAdapter.h"

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    #include "chrono_parsers/yaml/ChParserTdpfYAML.h"
#endif

namespace chrono {
namespace ch_precice {

/// @addtogroup precice_module
/// @{

class ChApiPrecice ChPreciceAdapterTdpf : public ChPreciceAdapter {
  public:
    /// Construct a Chrono TDPF preCICE participant for the specified Chrono::FSI-TDPF system.
    /// No preCICE interfaces (coupling bodies and FEA meshes) are defined.
    ChPreciceAdapterTdpf(const std::string& precice_config_filename, std::shared_ptr<fsi::tdpf::ChFsiFluidSystemTDPF> sysTDPF, double time_step, bool verbose = false);

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    /// Construct a Chrono TDPF preCICE participant configured from the specified YAML file.
    /// The provided YAML file must be of type `TDPF` and include a member `precice_adapter_configuration`.
    /// The preCICE interfaces (coupling bodies and FEA meshes, the corresponding geometry, and their
    /// associated coupling meshes and mesh data) are read from the YAML specification file.
    ChPreciceAdapterTdpf(const std::string& precice_config_filename, const std::string& input_filename, bool verbose = false);
#endif

    ~ChPreciceAdapterTdpf() {}

    /// Add the body with given name and initial pose as a preCICE interface object.
    /// Notes:
    /// - if the TDPF preCICE participant is created from a YAML specification file, calls to this function are made automatically.
    void AddCouplingBody(const std::string& name, const ChFramed& frame);

  public:
    // Implementation of base class virtual methods
    virtual void InitializeParticipant() override;
    virtual void WriteCheckpoint(double time) override;
    virtual void ReadCheckpoint(double time) override;
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void WriteData() override;
    virtual void WriteOutput(int frame, double time) override;

  private:
    struct CouplingBody {
        int index;                       ///< index of coupling body
        std::vector<ChVector3d> points;  ///< BCE markers on body expressed in local frame
        ChFramed init_body_frame;        ///< initial body reference frame (absolute)
    };

#ifdef CHRONO_FEA
    struct CouplingFEAMesh {
        int index;
    };
#endif

    // Data exchange at the body-level
    void ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    // Data exchange at the BCE-level
    void ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    std::shared_ptr<fsi::tdpf::ChFsiSystemTDPF> m_sysFSI;        ///< underlying FSI-TDPF system
    std::shared_ptr<fsi::tdpf::ChFsiFluidSystemTDPF> m_sysTDPF;  ///< underlying Chrono TDPF system

    // Chrono physics items in coupling interface
    std::vector<std::shared_ptr<CouplingBody>> m_coupling_bodies;  ///< coupling rigid bodies
#ifdef CHRONO_FEA
    std::vector<std::shared_ptr<CouplingFEAMesh>> m_coupling_fea;  ///< coupling FEA meshes
#endif

#ifdef CHRONO_VSG
    fsi::tdpf::ChTdpfVisualizationVSG::Settings m_visTDPF_settings;  ///< TDPF visualization settings
#endif
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
