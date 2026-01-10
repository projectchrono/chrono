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
    // Allow using the AddFsiBody method from parent class (in case we want additional AddFsiBody functions here)
    using ChFsiSystem::AddFsiBody;
    */

    /// Set input file name with hydro data (HDF5 format).
    void SetHydroFilename(const std::string& filename) { m_hydro_filename = filename; }

    /// Initialize the FSI system.
    /// A call to this function marks the completion of system construction.
    virtual void Initialize() override;

  private:
    ChFsiFluidSystemTDPF* m_sysTDPF;  ///< cached TDPF fluid solver
    std::string m_hydro_filename;     ///< input hydro file name (HDF5 format)
    bool m_generic_fsi_interface;
};

/// @} fsitdpf

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono

#endif
