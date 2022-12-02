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

/// @brief Class to represent the fluid/granular dynamics system.
///
/// This class is used to represent a fluid/granular system and take
/// care of the time integration of the fluid/granular dynamics. This
/// is a class designed for base SPH simulation. The class holds pointer
/// to data, which is hold somewhere else. It also include a forceSystem,
/// which takes care of the computation of force between particles. The
/// forceSystem is owned by the class ChFsiForce.
class ChFluidDynamics : public ChFsiGeneral {
  public:
    /// Constructor of the fluid/granular dynamics class.
    /// - Instantiate ChFsiForce, i.e. force system;
    /// - Copy the pointer to SPH particle data, parameters,
    ///   and number of objects to member variables.
    ChFluidDynamics(std::shared_ptr<ChBce> otherBceWorker,             ///< Pointer to the information of BCE particles
                    ChSystemFsi_impl& otherFsiSystem,                  ///< Pointer to the FSI system
                    std::shared_ptr<SimParams> otherParamsH,           ///< Pointer to the simulation parameters
                    std::shared_ptr<ChCounters> otherNumObjects,       ///< Pointer to the number of objects
                    TimeIntegrator otherIntegrator,                    ///< Integration type (only for ISPH)
                    bool verb                                          ///< verbose terminal output
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
        std::shared_ptr<SphMarkerDataD> sphMarkersD2,  ///< Pointer SPH particle information at the second half step
        std::shared_ptr<SphMarkerDataD> sphMarkersD1,  ///< Pointer SPH particle information at the first half step
        std::shared_ptr<FsiBodiesDataD> fsiBodiesD,    ///< Pointer information of rigid bodies
        std::shared_ptr<FsiMeshDataD> fsiMeshD,        ///< Pointer information of flexible mesh
        Real dT,                                       ///< Simulation stepsize
        Real Time                                      ///< Simulation time
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
    std::shared_ptr<SimParams> paramsH;       ///< FSI parameters; values are mainained externally
    std::shared_ptr<ChCounters> numObjectsH;  ///< counters (fluid particles, number of rigids, boundaries)
    std::shared_ptr<ChFsiForce> forceSystem;  ///< Force system object; calculates the force between particles
    TimeIntegrator integrator_type;           ///< Integrator type

    bool verbose;

    /// Update activity of SPH particles.
    /// SPH particles which are in an active domain are set as active particles.
    /// For example, particles close to a rigid body.
    void UpdateActivity(std::shared_ptr<SphMarkerDataD> sphMarkersD1,
                        std::shared_ptr<SphMarkerDataD> sphMarkersD2,
                        std::shared_ptr<FsiBodiesDataD> fsiBodiesD,
                        std::shared_ptr<FsiMeshDataD> fsiMeshD,
                        Real Time);

    /// Update SPH particles data.
    /// In an explicit formulation, the function relies on the explicit integration scheme.
    void UpdateFluid(std::shared_ptr<SphMarkerDataD> sphMarkersD, Real dT);

    /// Update SPH particles data.
    /// In an implicit formulation, the function relies on the implicit integration scheme.
    void UpdateFluid_Implicit(std::shared_ptr<SphMarkerDataD> sphMarkersD);

    /// The function applies periodic boundary to the normal SPH particles.
    void ApplyBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD);

    /// The function modify the velocity of BCE particles.
    void ApplyModifiedBoundarySPH_Markers(std::shared_ptr<SphMarkerDataD> sphMarkersD);
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
