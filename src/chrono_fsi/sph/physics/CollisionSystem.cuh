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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Base class for processing proximity in an FSI system.
// =============================================================================

#ifndef CH_COLLISIONSYSTEM_FSI_H_
#define CH_COLLISIONSYSTEM_FSI_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/physics/FsiDataManager.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Base class for processing proximity computation in an FSI system.
class CollisionSystem {
  public:
    CollisionSystem(FsiDataManager& data_mgr);
    ~CollisionSystem();

    /// Encapsulate calcHash, findCellStartEndD, and reorderDataD
    void ArrangeData(std::shared_ptr<SphMarkerDataD> sphMarkersD);

    /// Complete construction.
    void Initialize();

  private:
    FsiDataManager& m_data_mgr;  ///< FSI data manager
    // Note: this is cached on every call to ArrangeData()
    std::shared_ptr<SphMarkerDataD> m_sphMarkersD;  ///< Information of the particles in the original array

    /// Smart resize that handles both initial allocation and subsequent resizes efficiently
    void ResizeArrays(uint numExtended);
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
