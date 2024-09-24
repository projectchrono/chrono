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

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChConfigFsi.h"
#include "chrono_fsi/ChFsiInterface.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class for an FSI-aware fluid solver.
class CH_FSI_API ChFluidSystem {
  public:
    /// Output mode.
    enum class OutputMode {
        CSV,   ///< comma-separated value
        CHPF,  ///< binary
        NONE   ///< none
    };

    /// Destructor for the FSI system.
    virtual ~ChFluidSystem();

    /// Enable/disable verbose terminal output (default: true).
    /// The default implementation sets the verbose mode for the FSI system and the underlying FSI interface.
    void SetVerbose(bool verbose);

    /// Set gravity for the FSI syatem.
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) = 0;

    /// Set integration step size.
    void SetStepSize(double step);

    /// Initialize the FSI system.
    /// A call to this function marks the completion of system construction.
    /// The default implementation simply checks that an associated FSI interface was created and a value for
    /// integration step size was provided.
    virtual void Initialize(unsigned int num_fsi_bodies,
                            unsigned int num_fsi_nodes1D,
                            unsigned int num_fsi_elements1D,
                            unsigned int num_fsi_nodes2D,
                            unsigned int num_fsi_elements2D);

    /// Get current simulation time.
    double GetSimTime() const { return m_time; }

    /// Get the integration step size.
    double GetStepSize() const { return m_step; }

    /// Additional actions taken after adding a rigid body to the FSI system.
    virtual void OnAddFsiBody(unsigned int index, ChFsiInterface::FsiBody& fsi_body) = 0;

    /// Additional actions taken after adding a 1-D flexible mesh to the FSI system.
    virtual void OnAddFsiMesh1D(unsigned int index, ChFsiInterface::FsiMesh1D& fsi_mesh) = 0;

    /// Additional actions taken after adding a 2-D flexible mesh to the FSI system.
    virtual void OnAddFsiMesh2D(unsigned int index, ChFsiInterface::FsiMesh2D& fsi_mesh) = 0;

    /// Function to integrate the FSI fluid system in time.
    virtual void DoStepDynamics(double step) = 0;

    /// Additional actions taken before applying fluid forces to the solid phase.
    virtual void OnApplySolidForces() = 0;

    /// Additional actions taken after loading new solid phase states.
    virtual void OnLoadSolidStates() = 0;

  protected:
    ChFluidSystem();

    bool m_verbose;           ///< enable/disable m_verbose terminal output
    std::string m_outdir;     ///< output directory
    OutputMode m_write_mode;  ///< output type

    bool m_is_initialized;  ///< set to true once the Initialize function is called

    double m_step;  ///< time step for fluid dynamics
    double m_time;  ///< current fluid dynamics simulation time
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
