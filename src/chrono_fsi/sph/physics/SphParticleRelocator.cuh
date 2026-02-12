// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Device utilities for moving SPH particles and BCE markers external to the solver
//
// =============================================================================

#ifndef SPH_PARTICLE_RELOCATOR_CUH
#define SPH_PARTICLE_RELOCATOR_CUH

#include "chrono_fsi/sph/physics/SphDataManager.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

class SphParticleRelocator {
  public:
    struct DefaultProperties {
        Real rho0;
        Real mu0;
    };

    SphParticleRelocator(FsiDataManager& data_mgr, const DefaultProperties& props);
    ~SphParticleRelocator() {}

    /// Shift all particles of specified type by the given vector.
    /// Properties (density and pressure) of relocated particles are overwritten with the specified values.
    void Shift(MarkerType type, const Real3& shift);

    /// Move all particles of specified type that are currently inside the source AABB to the given AABB.
    /// The destination AABB is assumed to be given in integer grid coordinates. Properties (density and pressure) of
    /// relocated particles are overwritten with the specified values.
    void MoveAABB2AABB(MarkerType type, const RealAABB& aabb_src, const IntAABB& aabb_dest, Real spacing);

  private:
    FsiDataManager& m_data_mgr;  ///< FSI data manager
    DefaultProperties m_props;   ///< particle density and pressure after relocation
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
