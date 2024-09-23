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
// Base class for an FSI system, independent of the fluid solver
//
// =============================================================================

#ifndef CH_FSI_SYSTEM_H
#define CH_FSI_SYSTEM_H

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceSegmentSet.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiInterface.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for a system for fluid-solid interaction problems.
class CH_FSI_API ChFsiSystem {
  public:
    /// Output mode.
    enum class OutputMode {
        CSV,   ///< comma-separated value
        CHPF,  ///< binary
        NONE   ///< none
    };

    /// Destructor for the FSI system.
    virtual ~ChFsiSystem();

    /// Attach Chrono MBS system.
    void AttachSystem(ChSystem* sysMBS);

    /// Enable/disable verbose terminal output (default: true).
    /// The default implementation sets the verbose mode for the FSI system and the underlying FSI interface.
    void SetVerbose(bool verbose);

    /// Set gravity for the FSI syatem.
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) = 0;

    /// Set integration step size for fluid dynamics.
    void SetStepSizeCFD(double step);

    /// Set integration step size for multibody dynamics.
    /// If a value is not provided, the MBS system is integrated with the same step used for fluid dynamics.
    void SetStepsizeMBD(double step);

    /// Add a rigid body to the FSI system.
    /// Returns the index of the FSI body in the internal list.
    size_t AddFsiBody(std::shared_ptr<ChBody> body);

    /// Add an FEA mesh to the FSI system.
    /// Contact surfaces (of type segment_set or tri_mesh) already defined for the FEA mesh are used to generate the
    /// interface between the solid and fluid phases. If none are defined, one contact surface of each type is created
    /// (as needed), but these are not attached to the given FEA mesh.
    void AddFsiMesh(std::shared_ptr<fea::ChMesh> mesh);

    /// Initialize the FSI system.
    /// A call to this function marks the completion of system construction.
    /// The default implementation simply checks that an associated FSI interface was created and a value for
    /// integration step size was provided.
    virtual void Initialize();

    /// Function to advance the FSI system combined state.
    /// This implements an explicit force-displacement co-simulation step with given meta-step:
    /// - advance fluid dynamics to new data exchange point
    /// - apply fluid forces on solid objects
    /// - advance multibody dynamics to new data exchange point
    /// - extract new states for FSI solid objects
    void DoStepDynamics(double step); 
    
    /// Get current simulation time.
    double GetSimTime() const { return m_time; }

    /// Get the integration step size for multibody dynamics.
    double GetStepSizeMBD() const { return m_step_MBD; }

    /// Get the integration step size for fluid dynamics.
    double GetStepSizeCFD() const { return m_step_CFD; }

    /// Get current estimated RTF (real time factor).
    double GetRTF() const { return m_RTF; }

    /// Get ratio of simulation time spent in MBS integration.
    double GetRatioMBS() const { return m_ratio_MBS; }

    // ----------

    //// TODO: change these to take a shared_ptr to a ChBody

    /// Return the FSI applied force on the body with specified index (as returned by AddFsiBody).
    /// The force is applied at the body COM and is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyForce(size_t i) const;

    /// Return the FSI applied torque on the body with specified index (as returned by AddFsiBody).
    /// The torque is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyTorque(size_t i) const;

    //// TODO: add functions to get force on FEA nodes

  protected:
    ChFsiSystem(ChSystem* sysMBS = nullptr);

    /// Additional actions taken after adding a rigid body to the FSI system.
    virtual void OnAddFsiBody(ChFsiInterface::FsiBody& fsi_body) = 0;

    /// Additional actions taken after adding a 1-D flexible mesh to the FSI system.
    virtual void OnAddFsiMesh1D(ChFsiInterface::FsiMesh1D& fsi_mesh) = 0;

    /// Additional actions taken after adding a 2-D flexible mesh to the FSI system.
    virtual void OnAddFsiMesh2D(ChFsiInterface::FsiMesh2D& fsi_mesh) = 0;

    /// Function to integrate the FSI fluid system in time.
    virtual void AdvanceFluidDynamics(double time, double step) = 0;
    
    /// Additional actions taken before applying fluid forces to the solid phase.
    virtual void OnApplySolidForces() = 0;

    /// Additional actions taken after loading new solid phase states.
    virtual void OnLoadSolidStates() = 0;

    std::unique_ptr<ChFsiInterface> m_fsi_interface;  ///< FSI interface system

    bool m_verbose;           ///< enable/disable m_verbose terminal output
    std::string m_outdir;     ///< output directory
    OutputMode m_write_mode;  ///< output type

    bool m_is_initialized;  ///< set to true once the Initialize function is called

  private:
    /// Add a flexible solid with segment set contact to the FSI system.
    void AddFsiMesh1D(std::shared_ptr<fea::ChContactSurfaceSegmentSet> surface);

    /// Add a flexible solid with surface mesh contact to the FSI system.
    void AddFsiMesh2D(std::shared_ptr<fea::ChContactSurfaceMesh> surface);

    ChSystem* m_sysMBS;  ///< multibody system

    double m_step_MBD;  ///< time step for multibody dynamics
    double m_step_CFD;  ///< time step for fluid dynamics
    double m_time;      ///< current fluid dynamics simulation time

    ChTimer m_timer_step;  ///< timer for integration step
    ChTimer m_timer_CFD;   ///< timer for fluid dynamics integration
    ChTimer m_timer_MBS;   ///< timer for multibody dynamics integration
    ChTimer m_timer_FSI;   ///< timer for FSI data exchange
    double m_RTF;          ///< real-time factor (simulation time / simulated time)
    double m_ratio_MBS;    ///< fraction of step simulation time for MBS integration
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
