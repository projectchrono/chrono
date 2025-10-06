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
// Class for performing time integration in fluid system.
//
// =============================================================================

#ifndef CH_FLUIDDYNAMICS_H_
#define CH_FLUIDDYNAMICS_H_

#include "chrono_fsi/sph/physics/SphDataManager.cuh"
#include "chrono_fsi/sph/physics/SphForce.cuh"
#include "chrono_fsi/sph/physics/SphCollisionSystem.cuh"
#include "chrono_fsi/sph/utils/SphUtilsDevice.cuh"

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
/// is owned by the class SphForce.
class SphFluidDynamics {
  public:
    /// Constructor of the fluid/granular dynamics class.
    /// - Instantiate SphForce, i.e. force system;
    /// - Copy the pointer to SPH particle data, parameters,
    ///   and number of objects to member variables.
    SphFluidDynamics(FsiDataManager& data_mgr,  ///< FSI data manager
                  SphBceManager& bce_mgr,       ///< BCE manager
                  bool verbose,              ///< verbose output
                  bool check_errors          ///< check CUDA errors
    );

    /// Destructor of the fluid/granular dynamics class.
    ~SphFluidDynamics();

    /// Perform proximity search.
    /// Sort particles (broad-phase) and create neighbor lists (narrow-phase)
    void ProximitySearch();

    //// TODO: make private
    void CopySortedMarkers(const std::shared_ptr<SphMarkerDataD>& in, std::shared_ptr<SphMarkerDataD>& out);

    /// Advance the dynamics of the SPH fluid system.
    /// In a explicit scheme, the force system calculates the forces between the particles which are then used to update
    /// the particles position, velocity, and density; The density is then used, through the equation of state, to
    /// update pressure. For the implicit SPH scheme, the pressures are updated instead of density.
    void DoStepDynamics(std::shared_ptr<SphMarkerDataD> y,  ///< marker state (in/out)
                        Real t,                             ///< current simulation time
                        Real h,                             ///< simulation stepsize
                        IntegrationScheme scheme            ///< integration scheme
    );

    /// Copy markers in the specified group from sorted arrays to original-order arrays.
    void CopySortedToOriginal(MarkerGroup group,
                              std::shared_ptr<SphMarkerDataD> sortedSphMarkersD2,
                              std::shared_ptr<SphMarkerDataD> sphMarkersD);

    /// Synchronize the async copy stream (used for the copySortedToOriginal function)
    void SynchronizeCopyStream() {
        cudaStreamSynchronize(m_copy_stream);
    }

    /// Function to perform Shepard filtering.
    /// It calculates the densities directly, not based on the derivative of the density. This function is used in
    /// addition to the density update in UpdateFluid.
    //// TODO RADU: where is this used?!?  Obsololete?
    void DensityReinitialization();

    /// Initialize the force and colision systems.
    /// Synchronize data between device and host (parameters and counters).
    void Initialize();

    /// Return the SphForce type used in the simulation.
    std::shared_ptr<SphForce> GetForceSystem() { return forceSystem; }

    /// Update activity of SPH particles.
    /// SPH particles which are in an active domain (e.g., close to a solid) are set as active particles.
    void UpdateActivity(std::shared_ptr<SphMarkerDataD> sphMarkersD, double time);

    /// Check if arrays must be resized due to change in particle activity.
    bool CheckActivityArrayResize();

    /// Compute the time step.
    double computeTimeStep() const;

  private:
    FsiDataManager& m_data_mgr;                        ///< FSI data manager
    std::shared_ptr<SphForce> forceSystem;             ///< force system object; calculates the force between particles
    std::shared_ptr<SphCollisionSystem> collisionSystem;  ///< collision system for building neighbors list

    bool m_verbose;
    bool m_check_errors;

    /// Advance the state of the fluid system using an explicit Euler step.
    void EulerStep(std::shared_ptr<SphMarkerDataD> sortedMarkers, Real dT);

    /// Advance the state of the fluid system using an mid-point step.
    void MidpointStep(std::shared_ptr<SphMarkerDataD> sortedMarkers, Real dT);

    /// Apply boundary conditions on the sides of the computational domain.
    void ApplyBoundaryConditions(std::shared_ptr<SphMarkerDataD> sortedSphMarkersD);


    cudaStream_t m_copy_stream;  ///< stream for async copy operations
};

/// @} fsisph_physics

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
