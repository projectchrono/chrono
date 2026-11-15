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

#ifndef CH_PRECICE_ADAPTER_SPH_H
#define CH_PRECICE_ADAPTER_SPH_H

#include "chrono_precice/ChPreciceAdapter.h"

#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#ifdef CHRONO_FEA
    #include "chrono/core/ChVector2.h"
    #include "chrono/fea/ChMesh.h"
    #include "chrono/fea/ChNodeFEAxyzD.h"
#endif

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    #include "chrono_parsers/yaml/ChParserSphYAML.h"
#endif

namespace chrono {
namespace ch_precice {

/// @addtogroup precice_module
/// @{

class ChApiPrecice ChPreciceAdapterSph : public ChPreciceAdapter {
  public:
    /// Construct a Chrono SPH preCICE participant for the specified Chrono::FSI-SPH system.
    /// No preCICE interfaces (coupling bodies and FEA meshes) are defined.
    ChPreciceAdapterSph(const std::string& precice_config_filename, std::shared_ptr<fsi::sph::ChFsiFluidSystemSPH> sysSPH, double time_step, bool verbose = false);

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    /// Construct a Chrono SPH preCICE participant configured from the specified YAML file.
    /// The provided YAML file must be of type `SPH` and include a member `precice_adapter_configuration`.
    /// The preCICE interfaces (coupling bodies and FEA meshes, the corresponding geometry, and their
    /// associated coupling meshes and mesh data) are read from the YAML specification file.
    ChPreciceAdapterSph(const std::string& precice_config_filename, const std::string& input_filename, bool verbose = false);
#endif

    ~ChPreciceAdapterSph() {}

    /// Add the body with given name and initial pose as a preCICE interface object.
    /// Notes:
    /// - BCE markers are created for each shape in the specified ChBodyGeometry object.
    /// - if the SPH preCICE participant is created from a YAML specification file, calls to this function are made automatically.
    void AddCouplingBody(const std::string& name, const ChFramed& frame, std::shared_ptr<ChBodyGeometry> geometry);

    /// Add the body with given name and initial pose as a preCICE interface object.
    /// Notes:
    /// - BCE markers are set from the specified points.
    /// - if the SPH preCICE participant is created from a YAML specification file, calls to this function are made automatically.
    void AddCouplingBody(const std::string& name, const ChFramed& frame, const std::vector<ChVector3d> bce);

#ifdef CHRONO_FEA
    /// Add a 1-D FEA mesh (e.g., a cable) with given name as a preCICE interface object.
    /// The mesh is specified by its initial node positions and its segments (pairs of 0-based node indices). The
    /// mesh nodes are exchanged through coupling meshes of type FEA_MESH_NODES, in the order in which the meshes were
    /// added and, within each mesh, in the order of the specified nodes.
    /// Notes:
    /// - a proxy FEA mesh is created only to generate the BCE markers that represent the mesh to the SPH solver; its
    ///   nodes follow the states received through preCICE.
    /// - if the SPH preCICE participant is created from a YAML specification file, calls to this function are made automatically.
    void AddCouplingFEAMesh(const std::string& name,                                                    ///< mesh name
                            const std::vector<ChVector3d>& nodes,                                       ///< initial node positions
                            const std::vector<ChVector2i>& segments,                                    ///< segment node indices
                            fsi::sph::BcePatternMesh1D bce_pattern = fsi::sph::BcePatternMesh1D::FULL,  ///< BCE pattern
                            bool remove_center = false,                                                 ///< remove BCE markers on center line
                            bool check_embedded = true                                                  ///< remove overlapping SPH particles
    );
#endif

  public:
    // Implementation of base class virtual methods
    virtual size_t GetNumFsiBodies() const override;
    virtual void InitializeParticipant() override;
    virtual void OnReadData() override;
    virtual void OnWriteData() override;
    virtual void OnReadCheckpoint(double time) override;
    virtual void OnWriteCheckpoint(double time) override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void OnWriteOutput(int frame, double time) override;

  private:
    struct CouplingBody {
        int index;                       ///< index of coupling body
        std::vector<ChVector3d> points;  ///< BCE markers on body expressed in local frame
        ChFramed init_body_frame;        ///< initial body reference frame (absolute)
    };

#ifdef CHRONO_FEA
    /// Coupling FEA mesh data.
    struct CouplingFEAMesh {
        int index;                                               ///< index of coupling FEA mesh
        std::string name;                                        ///< mesh name
        std::shared_ptr<fea::ChMesh> mesh;                       ///< proxy FEA mesh
        std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> nodes;  ///< proxy mesh nodes (in specified order)
        std::shared_ptr<fsi::FsiMesh1D> fsi_mesh;                ///< associated FSI 1-D mesh
        std::vector<int> fsi_node_index;                         ///< index of each node in the FSI mesh
    };
#endif

    // Data exchange at the body-level
    void ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    // Data exchange at the BCE-level
    void ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

#ifdef CHRONO_FEA
    // Data exchange at the FEA node level
    void ReadFeaNodeData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteFeaNodeData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);
#endif

    //// TODO - get rid of using a ChFsiProblemSPH!!!
    //// OK to use it when initializing the preCICE adapter from a YAML file (because the parser creates it anyway)

    std::shared_ptr<fsi::sph::ChFsiProblemSPH> m_fsi_problem;  ///< underlying FSI-SPH problem
    std::shared_ptr<fsi::sph::ChFsiFluidSystemSPH> m_sysSPH;   ///< underlying Chrono SPH system
    double m_time_step;                                        ///< integration step size

    // Chrono physics items in coupling interface
    std::vector<std::shared_ptr<CouplingBody>> m_coupling_bodies;  ///< coupling rigid bodies
#ifdef CHRONO_FEA
    std::vector<std::shared_ptr<CouplingFEAMesh>> m_coupling_fea;  ///< coupling FEA meshes
#endif

    // Solid states and forces exchanged with the SPH solver (states persist across reads, so that data not
    // received through preCICE keeps its last value)
    std::vector<fsi::FsiBodyState> m_body_states;  ///< states of coupling bodies
    std::vector<fsi::FsiBodyForce> m_body_forces;  ///< forces on coupling bodies
#ifdef CHRONO_FEA
    std::vector<fsi::FsiMeshState> m_mesh1D_states;  ///< states of FSI 1-D mesh nodes
    std::vector<fsi::FsiMeshState> m_mesh2D_states;  ///< states of FSI 2-D mesh nodes
    std::vector<fsi::FsiMeshForce> m_mesh1D_forces;  ///< forces on FSI 1-D mesh nodes
    std::vector<fsi::FsiMeshForce> m_mesh2D_forces;  ///< forces on FSI 2-D mesh nodes
#endif

#ifdef CHRONO_VSG
    fsi::sph::ChSphVisualizationVSG::Settings m_visSPH_settings;  ///< SPH visualization settings
#endif
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
