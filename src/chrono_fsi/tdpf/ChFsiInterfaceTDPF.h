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
// Custom FSI interface for coupling a TDPF-based fluid system with a Chrono MBS
//
// =============================================================================

#ifndef CH_FSI_INTERFACE_TDPF_H
#define CH_FSI_INTERFACE_TDPF_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiInterface.h"

#include "chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h"

namespace chrono {
namespace fsi {
namespace tdpf {

/// @addtogroup fsitdpf
/// @{

/// Custom FSI interface between a Chrono multibody system and the TDPF-based fluid system.
class CH_FSI_API ChFsiInterfaceTDPF : public ChFsiInterface {
  public:
    ChFsiInterfaceTDPF(ChSystem* sysMBS, ChFsiFluidSystemTDPF* sysTDPF);
    ~ChFsiInterfaceTDPF();

    /// Exchange solid phase state information between the MBS and fluid system.
    virtual void ExchangeSolidStates() override;

    /// Exchange solid phase force information between the multibody and fluid systems.
    virtual void ExchangeSolidForces() override;

  private:
    ChFsiFluidSystemTDPF* m_sysTDPF;
};

/// @} fsitdpf

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono

#endif
