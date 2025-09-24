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
#include "chrono_fsi/ChFsiFluidSystem.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_base
/// @{

/// Base class for a system for fluid-solid interaction problems.
///
/// This class is used to represent fluid-solid interaction problems consisting of fluid dynamics and multibody system.
/// Each of the two underlying physics is an independent object owned and instantiated by this class.
/// A derived class must always set the FSI interface for coupling the two physics systems.
class CH_FSI_API ChFsiSystem {
  public:
    /// Destructor for the FSI system.
    virtual ~ChFsiSystem();

    /// Attach a fluid system.
    void AttachFluidSystem(ChFsiFluidSystem* sys);

    /// Attach a MBS system.
    void AttachMultibodySystem(ChSystem* sys);

    /// Access the associated fluid system.
    ChFsiFluidSystem& GetFluidSystem() const;

    /// Access the associated multibody system.
    ChSystem& GetMultibodySystem() const;

    /// Access the associated FSI interface.
    ChFsiInterface& GetFsiInterface() const;

    /// Enable/disable verbose terminal output (default: true).
    /// The default implementation sets the verbose mode for the FSI system and the underlying FSI interface.
    void SetVerbose(bool verbose);

    /// Set gravitational acceleration for the FSI syatem.
    /// This function sets gravity for both the fluid and multibody systems.
    void SetGravitationalAcceleration(const ChVector3d& gravity);

    /// Set integration step size for fluid dynamics.
    void SetStepSizeCFD(double step);

    /// Set integration step size for multibody dynamics.
    /// If a value is not provided, the MBS system is integrated with the same step used for fluid dynamics.
    void SetStepsizeMBD(double step);

    /// Add a rigid body to the FSI system.
    /// BCE markers are created based on the provided geometry.
    std::shared_ptr<FsiBody> AddFsiBody(std::shared_ptr<ChBody> body,
                                        std::shared_ptr<ChBodyGeometry> geometry,
                                        bool check_embedded);

    /// Add an FEA mesh to the FSI system.
    /// Any SegmentSet contact surfaces already defined for the FEA mesh are used to generate the interface between the
    /// solid and fluid phases. If none are defined, one contact surface is created, but it is not attached to the FEA
    /// mesh.
    std::shared_ptr<FsiMesh1D> AddFsiMesh1D(std::shared_ptr<fea::ChMesh> mesh, bool check_embedded);

    /// Add an FEA mesh to the FSI system.
    /// Any TriMesh contact surfaces already defined for the FEA mesh are used to generate the interface between the
    /// solid and fluid phases. If none are defined, one contact surface is created, but it is not attached to the FEA
    /// mesh.
    std::shared_ptr<FsiMesh2D> AddFsiMesh2D(std::shared_ptr<fea::ChMesh> mesh, bool check_embedded);

    /// Enable use and set method of obtaining FEA node directions.
    /// If provided, node direction vectors can be used to provide a more accurate interpolation of positions between
    /// nodes (piece-wise cubic as opposed to only piece-wise linear). Node directions can be provided by the FEA
    /// elements and communicated to the fluid solver (NodeDirectionsMode::EXACT), or else approximated by averaging
    /// direction over all elements incident to the node. The default is set by a concrete ChFsiSystem and the
    /// associated FSI interface.
    void UseNodeDirections(NodeDirectionsMode mode);

    /// Initialize the FSI system.
    /// A call to this function marks the completion of system construction.
    /// The default implementation performs the following operations:
    /// - check that an associated FSI interface was created and a value for integration step size was provided;
    /// - initialize the FSI interface;
    /// - get initial solid states from the FSI interface;
    /// - initialize the associated fluid solver, passing the initial solid states;
    virtual void Initialize();

    /// Class to specify step dynamics for the associated multibody system.
    class CH_FSI_API MBDCallback {
      public:
        virtual ~MBDCallback() {}
        /// Advance the state of the associated multibody system by the provided step. The MBS integration can take as
        /// many steps as necessary to reach the final step within the provided threshold.
        virtual void Advance(double step, double threshold) = 0;
    };

    /// Set a custom function for advancing the dynamics of the associated multibody system.
    /// If not provided, the MBS system is integrated with the specified step size (see SetStepsizeMBD) by calling
    /// ChSystem::DoStepDynamics.
    void RegisterMBDCallback(std::shared_ptr<MBDCallback> callback) { m_MBD_callback = callback; }

    /// Function to advance the FSI system combined state.
    /// This implements an explicit force-displacement co-simulation step:
    /// - advance fluid dynamics (CFD) to new data exchange point;
    /// - advance multibody dynamics (MBD) to new data exchange point;
    /// - extract (from fluid system) and apply (to MBS) fluid forces on FSI solid objects;
    /// - extract (from MBS) and apply (to fluid system) new states for FSI solid objects;
    /// Notes:
    /// - no data exchange is performed before the first step;
    ///   this assumes that FSI solid states and FSI solid forces are properly initialized
    /// - CFD advance calls ChFsiFluidSystem::DoStepDynamics multiple times (see SetStepsizeCFD);
    /// - MBD advance is executed in a separate, concurrent thread and does not block execution;
    /// - the caller can register a custom callback (of type ChFsiSystem::MBDCallback) to control MBD advance;
    /// - if MBDCallback not provided, MBD advance calls ChSystem::DoStepDynamics multiple times (see SetStepsizeMBD);
    void DoStepDynamics(double step);

    /// Get current simulation time.
    double GetSimTime() const { return m_time; }

    /// Get the integration step size for multibody dynamics.
    double GetStepSizeMBD() const { return m_step_MBD; }

    /// Get the integration step size for fluid dynamics.
    double GetStepSizeCFD() const { return m_step_CFD; }

    /// Get current estimated RTF (real time factor) for the coupled problem.
    double GetRtf() const { return m_RTF; }

    /// Set RTF value for the coupled problem.
    /// This is necessary for correct reporting if automatic integration of the multibody system dynamics is disabled
    /// and performed externally.
    void SetRtf(double rtf) { m_RTF = rtf; }

    /// Get current estimated RTF (real time factor) for the fluid system.
    double GetRtfCFD() const;

    /// Get current estimated RTF (real time factor) for the multibody system.
    double GetRtfMBD() const;

    /// Get ratio of simulation time spent in MBS integration.
    double GetRatioMBD() const { return m_ratio_MBD; }

    // ----------

    /// Return the time in seconds for for simulating the last step.
    double GetTimerStep() const { return m_timer_step(); }

    /// Return the time in seconds for fluid dynamics over the last step.
    double GetTimerCFD() const { return m_timer_CFD; }

    /// Return the time in seconds for multibody dynamics over the last step.
    double GetTimerMBD() const { return m_timer_MBD; }

    /// Return the time in seconds for data exchange between phases over the last step.
    double GetTimerFSI() const { return m_timer_FSI(); }

    // ----------

    /// Get a list of the FSI rigid bodies.
    const std::vector<std::shared_ptr<FsiBody>>& GetBodies() const { return m_fsi_interface->GetBodies(); }

    //// TODO: change these to take a shared_ptr to a ChBody

    /// Return the FSI applied force on the body with specified index (as returned by AddFsiBody).
    /// The force is applied at the body COM and is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyForce(size_t i) const;

    /// Return the FSI applied torque on the body with specified index (as returned by AddFsiBody).
    /// The torque is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyTorque(size_t i) const;

    /// Print the FSI statistics
    void PrintFSIStats() const;

    //// TODO: add functions to get force on FEA nodes

  protected:
    /// Construct an FSI system coupling the provided multibody and fluid systems.
    /// Derived classes must also construct and set the FSI interface (`m_fsi_interface`).
    ChFsiSystem(ChSystem* sysMBS, ChFsiFluidSystem* sysCFD);

    ChSystem* m_sysMBS;                               ///< multibody system
    ChFsiFluidSystem* m_sysCFD;                       ///< FSI fluid solver
    std::shared_ptr<ChFsiInterface> m_fsi_interface;  ///< FSI interface system

    bool m_verbose;         ///< enable/disable m_verbose terminal output
    bool m_is_initialized;  ///< set to true once the Initialize function is called

  private:
    void AdvanceCFD(double step, double threshold);
    void AdvanceMBS(double step, double threshold);

    double m_step_MBD;  ///< time step for multibody dynamics
    double m_step_CFD;  ///< time step for fluid dynamics
    double m_time;      ///< current fluid dynamics simulation time

    std::shared_ptr<MBDCallback> m_MBD_callback;  ///< callback for MBS dynamics

    ChTimer m_timer_step;  ///< timer for integration step
    ChTimer m_timer_FSI;   ///< timer for FSI data exchange
    double m_timer_CFD;    ///< timer for fluid dynamics integration
    double m_timer_MBD;    ///< timer for multibody dynamics integration
    double m_RTF;          ///< real-time factor (simulation time / simulated time)
    double m_ratio_MBD;    ///< fraction of step simulation time for MBS integration
};

/// @} fsi_base

}  // end namespace fsi
}  // end namespace chrono

#endif
