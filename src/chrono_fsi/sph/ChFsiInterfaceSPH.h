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
// Class for processing the interface between Chrono and the SPH-based FSI module
// =============================================================================

#ifndef CH_FSI_INTERFACE_SPH_H
#define CH_FSI_INTERFACE_SPH_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChFsiInterface.h"

#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Class for processing the interface between Chrono and the SPH-based FSI module.
class ChFsiInterfaceSPH : public ChFsiInterface {
  public:
    ChFsiInterfaceSPH(sph::FsiDataManager& data_mgr);
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
    sph::FsiDataManager& m_data_mgr;  ///< FSI data manager
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
