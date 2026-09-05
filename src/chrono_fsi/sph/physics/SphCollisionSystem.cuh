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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu, Radu Serban
// =============================================================================
//
// Base class for processing proximity in an FSI system.
// =============================================================================

#ifndef CH_SPH_COLLISION_SYSTEM_H
#define CH_SPH_COLLISION_SYSTEM_H

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/physics/SphDataManager.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Base class for processing proximity computation in an FSI system.
class SphCollisionSystem {
  public:
    SphCollisionSystem(FsiDataManager& data_mgr);
    ~SphCollisionSystem();

    // The object owns a device allocation (m_errflagD) freed in the destructor, so a copy would
    // alias it and free it twice. Chrono holds this class through a shared_ptr and never copies it;
    // deleting the operations makes that a compile-time fact rather than a convention.
    SphCollisionSystem(const SphCollisionSystem&) = delete;
    SphCollisionSystem& operator=(const SphCollisionSystem&) = delete;
    SphCollisionSystem(SphCollisionSystem&&) = delete;
    SphCollisionSystem& operator=(SphCollisionSystem&&) = delete;

    /// Complete construction.
    void Initialize();

    /// Sort particles based on their bins (broad phase proximity search).
    /// This function encapsulates calcHash, findCellStartEndD, and reorderDataD.
    void ArrangeData(std::shared_ptr<SphMarkerDataD> sphMarkersD, std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);

    /// Construct neighbor lists (narrow phase proximity search).
    void NeighborSearch(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);

  private:
    FsiDataManager& m_data_mgr;  ///< FSI data manager
    bool* m_errflagD = nullptr;  ///< device error flag for the proximity kernels, allocated once
    // Note: this is cached on every call to ArrangeData()
    std::shared_ptr<SphMarkerDataD> m_sphMarkersD;  ///< Information of the particles in the original array
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
