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
// Author: Radu Serban
// =============================================================================
//
// Implementation of FSI system using a TDPF fluid solver.
//
// =============================================================================

#ifndef CH_FSI_SYSTEM_TDPF_H
#define CH_FSI_SYSTEM_TDPF_H

#include "chrono_fsi/ChFsiSystem.h"
#include "chrono/physics/ChLoadHydrodynamics.h"

#include "chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h"

namespace chrono {
namespace fsi {
namespace tdpf {

/// @addtogroup fsitdpf
/// @{

/// FSI system using a TDPF-based fluid solver.
class CH_FSI_API ChFsiSystemTDPF : public ChFsiSystem {
  public:
    /// Construct an FSI system to couple the specified Chrono MBS system and TDPF fluid system.
    ChFsiSystemTDPF(ChSystem* sysMBS, ChFsiFluidSystemTDPF* sysTDPF, bool use_generic_interface = false);
    ~ChFsiSystemTDPF();

    /// Access the associated TDPF fluid system.
    ChFsiFluidSystemTDPF& GetFluidSystemTDPF() const;

    /*
    // Allow using the AddRigidBody method from parent class (in case we want additional AddRigidBody functions here)
    using ChFsiSystem::AddRigidBody;
    */

    /// Set input file name with hydro data (HDF5 format).
    void SetHydroFilename(const std::string& filename);

    /// Set the points about which the hydro data coefficients are given (default: body reference frame origins).
    /// One point must be provided for each body in the hydro data file, in the order of the bodies in that file (the order in which the FSI bodies are added), expressed
    /// in the body reference frame. Hydro data files do not record the point about which their coefficients were computed,
    /// so these points must be provided if they are not the origins of the body reference frames. The infinite-frequency
    /// added mass is shifted from these points to the body reference frame origins at initialization (see
    /// ChLoadHydrodynamics). This function must be called before Initialize().
    void SetHydroReferencePoints(const std::vector<ChVector3d>& points);

    /// Modify the added mass blocks.
    /// This function can only be called after Initialize().
    void SetBodyAddedMassBlocks(const std::vector<ChMatrixDynamic<>>& blocks);

    /// Initialize the FSI system.
    /// A call to this function marks the completion of system construction.
    virtual void Initialize() override;

  private:
    ChFsiFluidSystemTDPF* m_sysTDPF;                    ///< cached TDPF fluid solver
    bool m_generic_fsi_interface;                       ///< use default FSI interface?
    std::shared_ptr<ChLoadHydrodynamics> m_hydro_load;  ///< hydrodynamics added mass
};

/// @} fsitdpf

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono

#endif
