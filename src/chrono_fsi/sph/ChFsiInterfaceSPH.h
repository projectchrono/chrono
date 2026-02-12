// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
// =============================================================================
//
// Custom FSI interface for coupling the SPH-based fluid system with a Chrono MBS
// =============================================================================

#ifndef CH_FSI_INTERFACE_SPH_H
#define CH_FSI_INTERFACE_SPH_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiInterface.h"

#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

namespace chrono {
namespace fsi {
namespace sph {

struct FsiDataManager;

/// @addtogroup fsisph
/// @{

/// Custom FSI interface between a Chrono multibody system and the SPH-based fluid system.
/// This custom FSI interface is paired with ChFsiFluidSystemSPH and provides a more efficient coupling with a Chrono
/// MBS that a generic FSI interface does, because it works directly with the data structures of ChFsiFluidSystemSPH.
class CH_FSI_API ChFsiInterfaceSPH : public ChFsiInterface {
  public:
    ChFsiInterfaceSPH(ChSystem* sysMBS, ChFsiFluidSystemSPH* sysSPH);
    ~ChFsiInterfaceSPH();

    /// Exchange solid phase state information between the MBS and fluid system.
    /// Extract FSI body states and mesh node states from the MBS, copy to the SPH data manager on the host, and then
    /// transfer to GPU memory. Directly access SPH data manager.
    virtual void ExchangeSolidStates() override;

    /// Exchange solid phase force information between the multibody and fluid systems.
    /// Transfer rigid and flex forces from the SPH data manager on the GPU to the host, then apply fluid forces and
    /// torques as external loads in the MBS. Directly access SPH data manager.
    virtual void ExchangeSolidForces() override;

  private:
    FsiDataManager* m_data_mgr;  ///< FSI data manager
};

/// @} fsisph

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
