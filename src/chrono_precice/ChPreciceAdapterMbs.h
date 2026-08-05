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

#ifndef CH_PRECICE_ADAPTER_MBS_H
#define CH_PRECICE_ADAPTER_MBS_H

#include "chrono_precice/ChPreciceAdapter.h"

#include "chrono/functions/ChFunction.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/input_output/ChOutput.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLoadHydrodynamics.h"
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChJoint.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkRSDA.h"
#include "chrono/physics/ChLinkMotorLinear.h"
#include "chrono/physics/ChLinkMotorRotation.h"

#ifdef CHRONO_FEA
    #include "chrono/fea/ChMesh.h"
#endif

#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    #include "chrono_parsers/yaml/ChParserMbsYAML.h"
#endif

namespace chrono {
namespace ch_precice {

/// @addtogroup precice_module
/// @{

/// preCICE adapter for Chrono MBS simulation.
class ChApiPrecice ChPreciceAdapterMbs : public ChPreciceAdapter {
  public:
    /// Construct a Chrono MBS preCICE participant for the specified Chrono system.
    /// No preCICE interfaces (coupling bodies and FEA meshes) are defined.
    /// If enabled and if added mass blocks are defined, a hydrodynamics load object will be created during initialization.
    ChPreciceAdapterMbs(const std::string& precice_config_filename, std::shared_ptr<ChSystem> sys, double time_step, bool verbose = false);

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    /// Construct a Chrono MBS preCICE participant configured from the specified YAML file.
    /// The provided YAML file must be of type `MBS` and include a member `precice_adapter_configuration`.
    /// The preCICE interfaces (coupling bodies and FEA meshes and their associated coupling meshes and mesh data)
    /// are read from the YAML specification file.
    ChPreciceAdapterMbs(const std::string& precice_config_filename, const std::string& input_filename, bool verbose = false);
#endif

    ~ChPreciceAdapterMbs() {}

    /// Enable/disable soft real-time for MBS simulation (default: false).
    void EnforceRealtime(bool realtime) { m_enforce_realtime = realtime; }

    /// Add the specified body as a preCICE interface object.
    /// Notes:
    /// - if the MBS preCICE participant is created from a YAML specification file, calls to this function are made automatically.
    void AddCouplingBody(std::shared_ptr<ChBodyAuxRef> body, const std::vector<ChVector3d>& points);

#ifdef CHRONO_FEA
    /// Add the specified FEA mesh as a preCICE interface object.
    /// Notes:
    /// - if the MBS preCICE participant is created from a YAML specification file, calls to this function are made automatically.
    void AddCouplingFEAMesh(std::shared_ptr<fea::ChMesh> fea_mesh);
#endif

    /// Set added mass blocks from specified HDF5 hydrodynamics file.
    /// Notes:
    /// - if the MBS preCICE participant is created from a YAML specification file, this information is read from that file if present.
    void SetAddedMassBlocks(const std::string& h5_filename);

    /// Set added mass blocks (simultaneously for all interface objects).
    /// Notes:
    /// - if the MBS preCICE participant is created from a YAML specification file, this information is read from that file if present.
    void SetAddedMassBlocks(const std::vector<ChMatrixDynamic<>> blocks);

    /// Get underlying Chrono multibody system.
    ChSystem& GetSystem() { return *m_sys; }

    /// Class to be used as a callback interface for user-defined actions to be performed before advancing MBS dynamics.
    /// The `OnStepDynamics` is called at each step, before the call to `DoStepDynamics`.
    class BeforeStepDynamicsCallback {
      public:
        virtual ~BeforeStepDynamicsCallback() {}
        virtual void OnStepDynamics(double time, double step) = 0;
    };

    /// Class to be used as a callback interface for user-defined actions to be performed after advancing MBS dynamics.
    /// The `OnStepDynamics` is called at each step, after the call to `DoStepDynamics`.
    class AfterStepDynamicsCallback {
      public:
        virtual ~AfterStepDynamicsCallback() {}
        virtual void OnStepDynamics(double time, double step) = 0;
    };

    /// Register a callback for operations to be performed before advancing dynamics at each step.
    void RegisterBeforeStepDynamicsCallback(std::shared_ptr<BeforeStepDynamicsCallback> callback) { m_beforestep_callback = callback; }

    /// Register a callback for operations to be performed after advancing dynamics at each step.
    void RegisterAfterStepDynamicsCallback(std::shared_ptr<AfterStepDynamicsCallback> callback) { m_afterstep_callback = callback; }

  private:
    // Implementation of base class virtual methods
    virtual size_t GetNumFsiBodies() const override;
    virtual void InitializeParticipant() override;
    virtual void OnReadData() override;
    virtual void OnWriteData() override;
    virtual void OnReadDataAM(const std::vector<ChMatrix66d>& blocks) override;
    virtual void OnReadCheckpoint(double time) override;
    virtual void OnWriteCheckpoint(double time) override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void OnWriteOutput(int frame, double time) override;

  private:
    /// Checkpoint data.
    struct Checkpoint {
        double time;     ///< checkpointing time
        ChState x;       ///< generalized positions
        ChStateDelta v;  ///< generalized velocities
    };

    /// Coupling rigid body data.
    struct CouplingBody {
        int index;                           ///< index of coupling body
        std::shared_ptr<ChBodyAuxRef> body;  ///< coupling body
        std::vector<ChVector3d> points;      ///< points on body expressed in local frame
        ChFramed init_body_frame;            ///< initial body reference frame (absolute)
        unsigned int accumulator_index;      ///< index of associated force accumulator
    };

#ifdef CHRONO_FEA
    /// Coupling FEA mesh data.
    struct CouplingFEAMesh {
        int index;                          ///< index of coupling FEA mesh
        std::shared_ptr<fea::ChMesh> mesh;  ///< coupling FEA mesh
    };
#endif

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    void LoadBodiesYAML(const YAML::Node& bodies, const parsers::ChParserMbsYAML& parser);
    void LoadMeshesYAML(const YAML::Node& meshes, const parsers::ChParserMbsYAML& parser);
    void LoadAddedMassYAML(const YAML::Node& added_mass, const parsers::ChParserMbsYAML& parser);
#endif

    void ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    void ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    std::shared_ptr<ChSystem> m_sys;  ///< underlying Chrono system
    double m_time_step;               ///< integration step size
    bool m_enforce_realtime;          ///< flag indicating soft real-time
    ChRealtimeStepTimer m_rt_timer;   ///< timer for enforcing soft real-time

    // Checkpointing
    Checkpoint m_checkpoint;  ///< Chrono system checkpoint data

    // Chrono physics items in coupling interface
    std::vector<std::shared_ptr<CouplingBody>> m_coupling_bodies;  ///< coupling rigid bodies
#ifdef CHRONO_FEA
    std::vector<std::shared_ptr<CouplingFEAMesh>> m_coupling_fea;  ///< coupling FEA meshes
#endif

    bool m_has_added_mass;                               ///< participant provides added mass
    std::vector<ChMatrixDynamic<>> m_added_mass_blocks;  ///< added mass blocks
    std::shared_ptr<ChLoadHydrodynamics> m_hydro_load;   ///< hydrodynamic added mass loads

    // Dynamics callbacks
    std::shared_ptr<BeforeStepDynamicsCallback> m_beforestep_callback;  ///< operations performed before advancing dynamics
    std::shared_ptr<AfterStepDynamicsCallback> m_afterstep_callback;    ///< operations performed after advancing dynamics

    // Output data
    ChAssembly::Components m_output_data;  ///< output data for a Chrono MBS participant
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
