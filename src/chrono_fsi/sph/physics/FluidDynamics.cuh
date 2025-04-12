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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================
//
// Class for performing time integration in fluid system.
//
// =============================================================================

#ifndef CH_FLUIDDYNAMICS_H_
#define CH_FLUIDDYNAMICS_H_

#include "chrono_fsi/sph/physics/FsiDataManager.cuh"
#include "chrono_fsi/sph/physics/FsiForce.cuh"
#include "chrono_fsi/sph/physics/CollisionSystem.cuh"
#include "chrono_fsi/sph/utils/UtilsDevice.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph_physics
/// @{

/// Class to represent the fluid/granular dynamics system.
///
/// This class is used to represent a fluid/granular system and take care of the time integration of the fluid/granular
/// dynamics. This is a class designed for base SPH simulation. The class holds pointer to data, which is hold somewhere
/// else. It also include a forceSystem, which takes care of the computation of force between particles. The forceSystem
/// is owned by the class FsiForce.
class FluidDynamics {
  public:
    /// Constructor of the fluid/granular dynamics class.
    /// - Instantiate FsiForce, i.e. force system;
    /// - Copy the pointer to SPH particle data, parameters,
    ///   and number of objects to member variables.
    FluidDynamics(FsiDataManager& data_mgr,  ///< FSI data manager
                    BceManager& bce_mgr,       ///< BCE manager
                    bool verbose               ///< verbose output
    );

    /// Destructor of the fluid/granular dynamics class.
    ~FluidDynamics();

    /// Perform proximity search.
    /// Sort particles (broad-phase) and create neighbor lists (narrow-phase)
    void ProximitySearch();

    /// Integrate the SPH fluid system in time.
    /// In a explicit scheme, the force system calculates the forces between the particles which are then used to update
    /// the particles position, velocity, and density. The density is then used, through the equation of state, to
    /// update pressure. in In the implicit scheme, the pressures are updated instead of density.
    void IntegrateSPH(
        std::shared_ptr<SphMarkerDataD> sortedSphMarkers2_D,  ///< SPH particle information at the second half step
        std::shared_ptr<SphMarkerDataD> sortedSphMarkers1_D,  ///< SPH particle information at the first half step
        Real time,                                            ///< current simulation time
        Real step                                             ///< simulation stepsize
    );

    /// Copy markers in the specified group from sorted arrays to original-order arrays.
    void CopySortedToOriginal(MarkerGroup group,
                              std::shared_ptr<SphMarkerDataD> sortedSphMarkersD2,
                              std::shared_ptr<SphMarkerDataD> sphMarkersD);

    /// Function to perform Shepard filtering.
    /// It calculates the densities directly, not based on the derivative of the density. This function is used in
    /// addition to the density update in UpdateFluid.
    void DensityReinitialization();

    /// Synchronize the copy of the data between device (GPU) and host (CPU).
    /// Including the parameters and number of objects. This function needs to be called once the host data are modified.
    void Initialize();

    /// Return the FsiForce type used in the simulation.
    std::shared_ptr<FsiForce> GetForceSystem() { return forceSystem; }

    /// Update activity of SPH particles.
    /// SPH particles which are in an active domain (e.g., close to a solid) are set as active particles.
    void UpdateActivity(std::shared_ptr<SphMarkerDataD> sphMarkersD);

    /// Check if arrays must be resized due to change in particle activity.
    bool CheckActivityArrayResize();

  protected:
    FsiDataManager& m_data_mgr;                        ///< FSI data manager
    std::shared_ptr<FsiForce> forceSystem;             ///< force system object; calculates the force between particles
    std::shared_ptr<CollisionSystem> collisionSystem;  ///< collision system for building neighbors list

    bool m_verbose;

    /// Update SPH particles data for explicit integration.
    void UpdateFluid(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD, Real dT);

    /// Apply periodic boundary to the normal SPH particles.
    void ApplyBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);

    /// Modify the velocity of BCE particles.
    /// TODO (Huzaifa) - Might need to deprecated this function.
    void ApplyModifiedBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD);
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
