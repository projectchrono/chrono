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

#include "chrono_fsi/physics/ChFsiForce.cuh"
#include "chrono_fsi/utils/ChUtilsDevice.cuh"
#include "chrono_fsi/physics/ChFsiForceExplicitSPH.cuh"
#include "chrono_fsi/physics/ChFsiForceI2SPH.cuh"
#include "chrono_fsi/physics/ChFsiForceIISPH.cuh"
#include "chrono_fsi/physics/ChSystemFsi_impl.cuh"

using chrono::fsi::TimeIntegrator;

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Class to represent the fluid/granular dynamics system.
///
/// This class is used to represent a fluid/granular system and take care of the time integration of the fluid/granular
/// dynamics. This is a class designed for base SPH simulation. The class holds pointer to data, which is hold somewhere
/// else. It also include a forceSystem, which takes care of the computation of force between particles. The forceSystem
/// is owned by the class ChFsiForce.
class ChFluidDynamics : public ChFsiBase {
  public:
    /// Constructor of the fluid/granular dynamics class.
    /// - Instantiate ChFsiForce, i.e. force system;
    /// - Copy the pointer to SPH particle data, parameters,
    ///   and number of objects to member variables.
    ChFluidDynamics(std::shared_ptr<ChBce> bce_manager,      ///< information on BCE particles
                    ChSystemFsi_impl& sysFSI,                ///< implementatin FSI system
                    std::shared_ptr<SimParams> params,       ///< simulation parameters
                    std::shared_ptr<ChCounters> numObjects,  ///< problem counters
                    TimeIntegrator otherIntegrator,          ///< integration type (only for ISPH)
                    bool verb                                ///< verbose output
    );

    /// Destructor of the fluid/granular dynamics class.
    ~ChFluidDynamics();

    /// Integrate the fluid/granular system in time.
    /// The underlying SPH method implementation goes inside this function.
    /// In a explicit scheme, to perform the integration, the force system
    /// calculates the forces between the particles. Then the forces are
    /// used to to update the particles position, velocity, and density in
    /// time, the latter is used to update the pressure from an equation of
    /// state. In the implicit scheme, the pressures are updated instead of density.
    void IntegrateSPH(
        std::shared_ptr<SphMarkerDataD> sphMarkers2_D,    ///< SPH particle information at the second half step
        std::shared_ptr<SphMarkerDataD> sphMarkers1_D,    ///< SPH particle information at the first half step
        std::shared_ptr<FsiBodyStateD> fsiBodyState_D,    ///< nformation on rigid bodies
        std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,  ///< information of 1-D flexible mesh
        std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D,  ///< information of 2-D flexible mesh
        Real dT,                                          ///< simulation stepsize
        Real time                                         ///< simulation time
    );

    /// Function to Shepard Filtering.
    /// It calculates the densities directly, not based on the derivative of the
    /// density. This function is used in addition to the density update in UpdateFluid.
    void DensityReinitialization();

    /// Synchronize the copy of the data between device (GPU) and host (CPU).
    /// Including the parameters and number of objects.
    /// This function needs to be called once the host data are modified.
    void Initialize();

    /// Return the integrator type used in the simulation.
    TimeIntegrator GetIntegratorType() { return integrator_type; }

    /// Return the ChFsiForce type used in the simulation.
    std::shared_ptr<ChFsiForce> GetForceSystem() { return forceSystem; }

  protected:
    ChSystemFsi_impl& fsiSystem;              ///< FSI data; values are maintained externally
    std::shared_ptr<ChFsiForce> forceSystem;  ///< force system object; calculates the force between particles
    TimeIntegrator integrator_type;           ///< integrator type

    bool verbose;

    /// Update activity of SPH particles.
    /// SPH particles which are in an active domain (e.g., close to a solid) are set as active particles.
    void UpdateActivity(std::shared_ptr<SphMarkerDataD> sphMarkers1_D,
                        std::shared_ptr<SphMarkerDataD> sphMarkers2_D,
                        std::shared_ptr<FsiBodyStateD> fsiBodyState_D,
                        std::shared_ptr<FsiMeshStateD> fsiMesh1DState_D,
                        std::shared_ptr<FsiMeshStateD> fsiMesh2DState_D,
                        Real time);

    /// Update SPH particles data for explicit integration.
    void UpdateFluid(std::shared_ptr<SphMarkerDataD> sphMarkersD, Real dT);

    /// Update SPH particles data for implicit integration.
    void UpdateFluid_Implicit(std::shared_ptr<SphMarkerDataD> sphMarkersD);

    /// Apply periodic boundary to the normal SPH particles.
    void ApplyBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD);

    /// Mpodify the velocity of BCE particles.
    void ApplyModifiedBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
