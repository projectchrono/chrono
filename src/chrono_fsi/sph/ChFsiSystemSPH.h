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
// Implementation of FSI system using an SPH fluid solver.
//
// =============================================================================

#ifndef CH_FSI_SYSTEM_SPH_H
#define CH_FSI_SYSTEM_SPH_H

#include "chrono_fsi/ChFsiSystem.h"

#include "chrono_fsi/sph/ChFluidSystemSPH.h"
#include "chrono_fsi/sph/ChFsiInterfaceSPH.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// FSI system using an SPH-based fluid solver.
class CH_FSI_API ChFsiSystemSPH : public ChFsiSystem {
  public:
    ChFsiSystemSPH(ChSystem* sysMBS);
    ~ChFsiSystemSPH();

    ChFluidSystemSPH& GetFluidSystemSPH() const;
    ChFsiInterfaceSPH& GetFsiInterface() const;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
