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
#include "chrono/utils/ChBodyGeometry.h"

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

    void AddCouplingBody(std::shared_ptr<ChBodyAuxRef> body);

    virtual void InitializeParticipant() override;
    virtual void WriteCheckpoint(double time) override;
    virtual void ReadCheckpoint(double time) override;
    virtual void ReadData() override;
    virtual double GetSolverTimeStep(double max_time_step) const override;
    virtual void AdvanceParticipant(double time, double time_step) override;
    virtual void WriteData() override;

  private:
    struct Checkpoint {
        double time;
        ChState x;
        ChStateDelta v;
    };

    struct CouplingBody {
        int index;
        std::shared_ptr<ChBodyAuxRef> body;
        unsigned int accumulator_index;
    };

    void ReadBodyRefData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyRefData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    void ReadBodyMeshData(const std::string& mesh_name, const CouplingMeshInfo& mesh_info);
    void WriteBodyMeshData(const std::string& mesh_name, CouplingMeshInfo& mesh_info);

    std::shared_ptr<ChSystem> m_sys;
    double m_time_step;
    bool m_enforce_realtime;
    ChRealtimeStepTimer m_rt_timer;

    // System checkpoint data
    Checkpoint m_checkpoint;

    // Chrono physics items in coupling interface
    std::vector<std::shared_ptr<CouplingBody>> m_coupling_bodies;

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

#ifdef CHRONO_VSG
    std::shared_ptr<vsg3d::ChVisualSystemVSG> m_vsg;
#endif
};

/// @} precice_module

}  // end namespace ch_precice
}  // namespace chrono

#endif
