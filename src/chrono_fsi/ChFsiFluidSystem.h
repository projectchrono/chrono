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
// Base class for an FSI-aware fluid solver
//
// =============================================================================

#ifndef CH_FLUID_SYSTEM_H
#define CH_FLUID_SYSTEM_H

#include <vector>
#include <stdexcept>

#include "chrono/ChConfig.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiDefinitions.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_base
/// @{

/// Base class for an FSI-aware fluid solver.
class CH_FSI_API ChFsiFluidSystem {
  public:
    virtual ~ChFsiFluidSystem();

    /// Enable/disable verbose terminal output (default: true).
    void SetVerbose(bool verbose);

    /// Set gravity for the fluid syatem.
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) = 0;

    /// Set integration step size.
    void SetStepSize(double step);

    /// Initialize the fluid system with no FSI support.
    virtual void Initialize();

    /// Function to integrate the FSI fluid system in time.
    void DoStepDynamics(double step);

    /// Get current simulation time.
    double GetSimTime() const { return m_time; }

    /// Get the integration step size.
    double GetStepSize() const { return m_step; }

    /// Get current estimated RTF (real time factor).
    double GetRtf() const { return m_RTF; }

    /// Return the time in seconds for fluid dynamics over the last step.
    double GetTimerStep() const { return m_timer_step(); }

    /// Function to integrate the FSI fluid system from `time` to `time + step`.
    virtual void OnDoStepDynamics(double time, double step) = 0;

    /// Additional actions taken before applying fluid forces to the solid phase.
    virtual void OnExchangeSolidForces() = 0;

    /// Additional actions taken after loading new solid phase states.
    virtual void OnExchangeSolidStates() = 0;

    /// Load FSI body and mesh node states from the given vectors.
    /// The functions LoadSolidStates and StoreSolidForces allow using a generic FSI interface.
    /// However, a concrete fluid system can be paired with a corresponding FSI interface, both of which work on the
    /// same data structures; in that case, the custom FSI interface need not use the mechanism provided by
    /// LoadSolidStates and StoreSolidForces (which incur the cost of additional data copies).
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                 const std::vector<FsiMeshState>& mesh1D_states,
                                 const std::vector<FsiMeshState>& mesh2D_states) = 0;

    /// Store the body and mesh node forces to the given vectors.
    /// The functions LoadSolidStates and StoreSolidForces allow using a generic FSI interface.
    /// However, a concrete fluid system can be paired with a corresponding FSI interface, both of which work on the
    /// same data structures; in that case, the custom FSI interface need not use the mechanism provided by
    /// LoadSolidStates and StoreSolidForces (which incur the cost of additional data copies).
    virtual void StoreSolidForces(std::vector<FsiBodyForce> body_forces,
                                  std::vector<FsiMeshForce> mesh1D_forces,
                                  std::vector<FsiMeshForce> mesh2D_forces) = 0;

  protected:
    ChFsiFluidSystem();

    /// Initialize the fluid system using initial states of solid FSI objects.
    /// A call to this function marks completion of the fluid system construction and can only be made from ChFsiSystem.
    /// The boolean `has_node_directions` indicates whether or not mesh states also contain node directions.
    virtual void Initialize(const std::vector<FsiBody>& fsi_bodies,
                            const std::vector<FsiMesh1D>& fsi_meshes1D,
                            const std::vector<FsiMesh2D>& fsi_meshes2D,
                            const std::vector<FsiBodyState>& body_states,
                            const std::vector<FsiMeshState>& mesh1D_states,
                            const std::vector<FsiMeshState>& mesh2D_states,
                            bool use_node_directions) = 0;

    bool m_verbose;        ///< enable/disable m_verbose terminal output
    std::string m_outdir;  ///< output directory

    bool m_is_initialized;  ///< set to true once the Initialize function is called

    double m_step;         ///< time step for fluid dynamics
    unsigned int m_frame;  ///< current simulation frame

  private:
    ChTimer m_timer_step;  ///< timer for integration step
    double m_RTF;          ///< real-time factor (simulation time / simulated time)
    double m_time;         ///< current simulation time

    friend class ChFsiSystem;
};

/// @} fsi_base

}  // end namespace fsi
}  // end namespace chrono

#endif
