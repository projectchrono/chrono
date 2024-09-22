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
// Author: Arman Pazouki, Wei Hu
// =============================================================================

#ifndef CH_FSI_FORCE_EXPLICITSPH_H_
#define CH_FSI_FORCE_EXPLICITSPH_H_

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/physics/ChFsiForce.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsi_physics
/// @{

/// Inter-particle force calculation for explicit schemes.
class ChFsiForceExplicitSPH : public ChFsiForce {
  public:
    /// Force class implemented using WCSPH with explicit integrator.
    /// Supports for both fluid and granular material dynamics.
    ChFsiForceExplicitSPH(FsiDataManager& data_mgr,  ///< FSI data manager
                          BceManager& bce_mgr,       ///< BCE manager
                          bool verbose               ///< verbose terminal output
    );

    ~ChFsiForceExplicitSPH();

    void Initialize() override;

  private:
    int density_initialization;

    /// Function to find neighbor particles and calculate the interactions between SPH particles
    void ForceSPH(std::shared_ptr<SphMarkerDataD> sortedSphMarkers_D, Real time, bool firstHalfStep) override;

    void neighborSearch();

    /// Function to calculate the XSPH velocity of the particles.
    /// XSPH velocity is a compromise between Eulerian and Lagrangian velocities, used
    /// to regularize the particles velocity and reduce noise.
    void CalculateXSPH_velocity();

    /// A wrapper around collide function.
    /// Calculates the force on particles, and copies the sorted XSPH velocities to the original.
    /// The latter is needed later for position update.
    void CollideWrapper(Real time, bool firstHalfStep);
};

/// @} fsi_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
