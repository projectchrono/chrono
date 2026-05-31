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

    ChSystem& GetSystem() { return *m_sys; }

    bool EnableVisualization(double render_fps,                  ///< rendering frequency
                             CameraVerticalDir camera_vertical,  ///< camera vertical direction (Y or Z)
                             const ChVector3d& camera_location,  ///< initial camera location
                             const ChVector3d& camera_target,    ///< initial camera look-at point
                             bool enable_shadows                 ///< enable dynamic shadows
    );

    void EnforceRealtime(bool realtime) { m_enforce_realtime = realtime; }

    void AddCouplingBody(std::shared_ptr<ChBodyAuxRef> body, const std::vector<ChVector3d>& points);
#ifdef CHRONO_FEA
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
    struct Checkpoint {
        double time;
        ChState x;
        ChStateDelta v;
    };

    struct CouplingBody {
        int index;
        std::shared_ptr<ChBodyAuxRef> body;
        std::vector<ChVector3d> points;
        ChFramed init_body_frame;
        unsigned int accumulator_index;
    };

#ifdef CHRONO_FEA
    struct CouplingFEAMesh {
        int index;
        std::shared_ptr<fea::ChMesh> mesh;
    };
#endif

    void ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    void ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    // Implementation of base class virtual methods
    virtual void InitializeParticipant() override;
    virtual void WriteCheckpoint(double time) override;
    virtual void ReadCheckpoint(double time) override;
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void WriteData() override;

    std::shared_ptr<ChSystem> m_sys;
    double m_time_step;
    bool m_enforce_realtime;
    ChRealtimeStepTimer m_rt_timer;

    // System checkpoint data
    Checkpoint m_checkpoint;

    // Chrono physics items in coupling interface
    std::vector<std::shared_ptr<CouplingBody>> m_coupling_bodies;
#ifdef CHRONO_FEA
    std::vector<std::shared_ptr<fea::ChMesh>> m_coupling_fea;
#endif

    // Dynamics callbacks
    std::shared_ptr<BeforeStepDynamicsCallback> m_beforestep_callback;  ///< operations performed before advancing dynamics
    std::shared_ptr<AfterStepDynamicsCallback> m_afterstep_callback;    ///< operations performed after advancing dynamics

    // Run-time visualization
    struct VisParams {
        VisParams();
        bool render;
        double render_fps;
        CameraVerticalDir camera_vertical;
        ChVector3d camera_location;
        ChVector3d camera_target;
        bool enable_shadows;
    };
    VisParams m_vis;  ///< visualization parameters

#if defined(CHRONO_PARSERS) && defined(CHRONO_HAS_YAML)
    ChYamlFileHandler m_file_handler;  ///< handler for data file paths
#endif

#ifdef CHRONO_VSG
    std::shared_ptr<vsg3d::ChVisualSystemVSG> m_vsg;
#endif
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
