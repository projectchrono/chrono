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

    /// Get current estimated RTF (real time factor).
    double GetRTF() const { return m_RTF; }

    /// Get ratio of simulation time spent in MBS integration.
    double GetRatioMBS() const { return m_ratio_MBS; }

    /// Enable/disable verbose terminal output (default: true).
    /// The default implementation sets the verbose mode for the FSI system and the underlying FSI interface.
    virtual void SetVerbose(bool verbose);

    /// Set gravity for the FSI syatem.
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) = 0;

    /// Initialize the FSI system.
    /// A call to this function marks the completion of system construction.
    /// The default implementation simply checks that an associated FSI interface was created.
    virtual void Initialize();

    /// Function to integrate the FSI system in time.
    virtual void DoStepDynamics_FSI() = 0;

    // ----------- Functions for adding bodies and associated BCE markers for different shapes

    /// Add a rigid body to the FSI system.
    /// Returns the index of the FSI body in the internal list.
    virtual size_t AddFsiBody(std::shared_ptr<ChBody> body) = 0;

    /// Add an FEA mesh to the FSI system.
    virtual void AddFsiMesh(std::shared_ptr<fea::ChMesh> mesh) = 0;

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

    ChSystem* m_sysMBS;  ///< multibody system

    std::unique_ptr<ChFsiInterface> m_fsi_interface;  ///< FSI interface system

    bool m_verbose;           ///< enable/disable m_verbose terminal output
    std::string m_outdir;     ///< output directory
    OutputMode m_write_mode;  ///< output type

    ChTimer m_timer_step;  ///< timer for integration step
    ChTimer m_timer_MBS;   ///< timer for MBS integration
    double m_RTF;          ///< real-time factor (simulation time / simulated time)
    double m_ratio_MBS;    ///< fraction of step simulation time for MBS integration
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
