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
#include "chrono/physics/ChLoadContainer.h"
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

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
#endif

namespace chrono {
namespace ch_precice {

/// @addtogroup precice_module
/// @{

/// preCICE adapter for Chrono MBS simulation.
class ChApiPrecice ChPreciceAdapterMbs : public ChPreciceAdapter {
  public:
    ChPreciceAdapterMbs(std::shared_ptr<ChSystem> sys, double time_step, bool verbose = false);

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    ChPreciceAdapterMbs(const std::string& input_filename, bool verbose = false);
#endif

    ~ChPreciceAdapterMbs();

    /// Get underlying Chrono multibody system.
    ChSystem& GetSystem() { return *m_sys; }

    /// Set the Chrono MBS model name.
    /// Notes:
    /// - if the MBS preCICE participant is created from a YAML specification file, the model name is read from that file.
    void SetModelName(const std::string& name) { m_model_name = name; }

    /// Set Chrono MBS run-time visualization parameters.
    /// Notes:
    /// - run-time visualization requires the Chrono::VSG module.
    /// - if the MBS preCICE participant is created from a YAML specification file, visualization parameters are read from that file.
    void SetVisualizationParameters(double render_fps,                  ///< rendering frequency
                                    CameraVerticalDir camera_vertical,  ///< camera vertical direction (Y or Z)
                                    const ChVector3d& camera_location,  ///< initial camera location
                                    const ChVector3d& camera_target,    ///< initial camera look-at point
                                    bool enable_shadows                 ///< enable dynamic shadows
    );

    /// Set Chrono MBS simulation output parameters.
    /// Notes:
    /// - output is generated for all coupling bodies and FEA meshes.
    /// - if the MBS preCICE participant is created from a YAML specification file, output parameters are read from that file.
    void SetOutputParameters(ChOutput::Format format,  ///< output DB format
                             ChOutput::Mode mode,      ///< output mode
                             double output_fps         ///< output frequency
    );

    /// Get the name of the Chrono MBS model.
    const std::string& GetModelName() const { return m_model_name; }

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
        ChFramed init_body_frame;            ///< initial body frame
        unsigned int accumulator_index;      ///< index of associated force accumulator
    };

#ifdef CHRONO_FEA
    /// Coupling FEA mesh data.
    struct CouplingFEAMesh {
        int index;                          ///< index of coupling FEA mesh
        std::shared_ptr<fea::ChMesh> mesh;  ///< coupling FEA mesh
    };
#endif

    /// Chrono MBS simulation output parameters.
    struct OutputParameters {
        OutputParameters();
        ChOutput::Format format;
        ChOutput::Mode mode;
        double fps;
    };

    /// Chrono MBS simulation run-time visualization parameters.
    struct VisParameters {
        VisParameters();
        bool render;
        double render_fps;
        CameraVerticalDir camera_vertical;
        ChVector3d camera_location;
        ChVector3d camera_target;
        bool enable_shadows;
    };

    // Implementation of base class virtual methods
    virtual void InitializeParticipant() override;
    virtual void WriteCheckpoint(double time) override;
    virtual void ReadCheckpoint(double time) override;
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void WriteData() override;

    void ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    void ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    void WriteOutput(int frame, double time);

    std::string m_model_name;         ///< name of the Chrono MBS model
    std::shared_ptr<ChSystem> m_sys;  ///< underlying Chrono system
    double m_time_step;               ///< integration step size
    bool m_enforce_realtime;          ///< flag indicating soft real-time
    ChRealtimeStepTimer m_rt_timer;   ///< timer for enforcing soft real-time

    // Checkpointing
    Checkpoint m_checkpoint;  ///< Chrono system checkpoint data

    // Chrono physics items in coupling interface
    std::vector<std::shared_ptr<CouplingBody>> m_coupling_bodies;  ///< coupling rigid bodies
#ifdef CHRONO_FEA
    std::vector<std::shared_ptr<fea::ChMesh>> m_coupling_fea;  ///< coupling FEA meshes
#endif

    // Dynamics callbacks
    std::shared_ptr<BeforeStepDynamicsCallback> m_beforestep_callback;  ///< operations performed before advancing dynamics
    std::shared_ptr<AfterStepDynamicsCallback> m_afterstep_callback;    ///< operations performed after advancing dynamics

    // Run-time visualization
    VisParameters m_vis_params;  ///< visualization parameters
#ifdef CHRONO_VSG
    std::shared_ptr<vsg3d::ChVisualSystemVSG> m_vsg;  ///< run-time visualization system
#endif

    // Output
    OutputParameters m_output_params;       ///< output specification
    ChAssembly::Components m_output_data;   ///< output data
    std::unique_ptr<ChOutput> m_output_db;  ///< output database

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    ChYamlFileHandler m_file_handler;  ///< handler for data file paths
#endif
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
