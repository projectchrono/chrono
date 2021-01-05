// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Contributors: Dan Negrut, Nic Olsen, Radu Serban
// =============================================================================

#pragma once

#include <vector>

#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"

namespace chrono {
namespace gpu {

/// @addtogroup gpu_physics
/// @{

// -----------------------------------------------------------------------------

/// Interface to a Chrono::Gpu system.
class CH_GPU_API ChSystemGpu {
  public:
    /// Construct system with given sphere radius, density, and big domain dimensions.
    ChSystemGpu(float sphere_rad, float density, float3 boxDims);

    virtual ~ChSystemGpu();

    ChSystemGpu_impl& getSystem() { return *m_sys; }

    /// Setr gravitational acceleration vector.
    void SetGravitationalAcceleration(const ChVector<float> g);

    /// Set particle positions.
    void SetParticlePositions(const std::vector<ChVector<float>>& points,
                              const std::vector<ChVector<float>>& vels = std::vector<ChVector<float>>(),
                              const std::vector<ChVector<float>>& ang_vels = std::vector<ChVector<float>>());

    /// Set the big domain to be fixed or not.
    /// If fixed, it will ignore any given position functions.
    void SetBDFixed(bool fixed);

    /// Set flags indicating whether or not a particle is fixed.
    /// MUST be called only once and MUST be called before Initialize.
    void SetParticleFixed(const std::vector<bool>& fixed);

    /// Set the output mode of the simulation.
    void SetOutputMode(CHGPU_OUTPUT_MODE mode);

    /// Set output settings bit flags by bitwise ORing settings in CHGPU_OUTPUT_FLAGS.
    void SetOutputFlags(unsigned char flags);

    /// Set timestep size.
    void SetFixedStepSize(float size_UU);

    /// Ensure that the deformation-based length unit is used.
    void DisableMinLength();

    /// Set the time integration scheme for the system.
    void SetTimeIntegrator(CHGPU_TIME_INTEGRATOR new_integrator);

    /// Set friction formulation.
    /// The frictionless setting uses a streamlined solver and avoids storing any physics information associated with
    /// friction
    void SetFrictionMode(CHGPU_FRICTION_MODE new_mode);

    /// Set rolling resistence formulation.
    /// NOTE: This requires friction to be active, otherwise this setting will be ignored.
    void SetRollingMode(CHGPU_ROLLING_MODE new_mode);

    /// Safety check velocity to ensure the simulation is still stable.
    void SetMaxSafeVelocity_SU(float max_vel);

    /// Set tuning psi factors for tuning the non-dimensionalization.
    void SetPsiFactors(unsigned int psi_T, unsigned int psi_L, float psi_R = 1.f);

    /// Enable/disable recording of contact info.
    void SetRecordingContactInfo(bool record);

    /// Set simualtion verbosity level.
    void SetVerbosity(CHGPU_VERBOSITY level);

    /// Create an axis-aligned sphere boundary condition.
    size_t CreateBCSphere(const ChVector<float>& center, float radius, bool outward_normal, bool track_forces);

    /// Create a Z-axis aligned cone boundary condition.
    size_t CreateBCConeZ(const ChVector<float>& tip,
                         float slope,
                         float hmax,
                         float hmin,
                         bool outward_normal,
                         bool track_forces);

    /// Create a plane boundary condition.
    size_t CreateBCPlane(const ChVector<float>& pos, const ChVector<float>& normal, bool track_forces);

    /// Create a Z-axis aligned cylinder boundary condition.
    size_t CreateBCCylinderZ(const ChVector<float>& center, float radius, bool outward_normal, bool track_forces);

    /// Disable a boundary condition by its ID, returns false if the BC does not exist.
    bool DisableBCbyID(size_t BC_id);

    /// Enable a boundary condition by its ID, returns false if the BC does not exist.
    bool EnableBCbyID(size_t BC_id);

    /// Return the total number of particles in the system
    size_t GetNumParticles() const;

    /// Return the maximum Z position over all particles.
    double GetMaxParticleZ() const;

    /// Return particle position.
    ChVector<float> GetParticlePosition(int nSphere) const;

    /// Return particle angular velocity.
    ChVector<float> GetParticleAngVelocity(int nSphere) const;

    /// Return particle linear velocity.
    ChVector<float> GetParticleVelocity(int nSphere) const;

    /// Return position of BC plane.
    ChVector<float> GetBCPlanePosition(size_t plane_id) const;

    /// Get the reaction forces on a boundary by ID, returns false if the forces are invalid (bad BC ID)
    bool GetBCReactionForces(size_t BC_id, ChVector<float>& force) const;

    /// Return number of particle-particle contacts.
    int GetNumContacts() const;

    /// Return number of subdomains in the big domain.
    unsigned int GetNumSDs() const;

    /// Initialize simulation so that it can be advanced.
    /// Must be called before AdvanceSimulation and after simulation parameters are set.
    virtual void Initialize();

    /// Advance simulation by duration in user units, return actual duration elapsed.
    /// Requires Initialize() to have been called.
    virtual double AdvanceSimulation(float duration);

    /// Write particle positions according to the system output mode.
    void WriteFile(std::string ofile) const;

    /// Write contact info file.
    void WriteContactInfoFile(std::string ofile) const;

    /// Roughly estimate of the total amount of memory used by the system.
    size_t EstimateMemUsage() const;

  protected:
    /// Protected default constructor.  Derived class must create m_sys.
    ChSystemGpu() : m_sys(nullptr) {}

    ChSystemGpu_impl* m_sys;  ///< underlying system implementation
};

// -----------------------------------------------------------------------------

/// Interface to a Chrono::Gpu mesh system.
class CH_GPU_API ChSystemGpuMesh : public ChSystemGpu {
  public:
    /// Construct system with given sphere radius, density, and big domain dimensions.
    ChSystemGpuMesh(float sphere_rad, float density, float3 boxDims);
    ~ChSystemGpuMesh();

    /// Load triangle meshes into granular system. MUST happen before initialize is called.
    void LoadMeshes(std::vector<std::string> objfilenames,
                    std::vector<ChMatrix33<float>> rotscale,
                    std::vector<float3> translations,
                    std::vector<float> masses);

    /// Enable/disable mesh collision (for all defined meshes).
    void EnableMeshCollision(bool val);

    /// Apply rigid body motion to specified mesh.
    void ApplyMeshMotion(unsigned int mesh,
                         const ChVector<>& pos,
                         const ChQuaternion<>& rot,
                         const ChVector<>& lin_vel,
                         const ChVector<>& ang_vel);

    /// Return the number of meshes in the system.
    unsigned int GetNumMeshes() const;

    ChSystemGpuMesh_impl& getSystemMesh() { return *(static_cast<ChSystemGpuMesh_impl*>(m_sys)); }

    /// Set verbosity level of mesh operations.
    void SetMeshVerbosity(CHGPU_MESH_VERBOSITY level);

    /// Initialize simulation so that it can be advanced.
    /// Must be called before AdvanceSimulation and after simulation parameters are set.
    /// This function initializes both the granular material and any existing trimeshes.
    virtual void Initialize() override;

    /// Advance simulation by duration in user units, return actual duration elapsed.
    /// Requires Initialize() to have been called.
    virtual double AdvanceSimulation(float duration) override;

    /// Collect contact forces exerted on all meshes by the granular system.
    void CollectMeshContactForces(std::vector<ChVector<>>& forces, std::vector<ChVector<>>& torques);

    /// Collect contact forces exerted on the specified meshe by the granular system.
    void CollectMeshContactForces(int mesh, ChVector<>& force, ChVector<>& torque);

    /// Write visualization files for triangle meshes with current positions.
    void WriteMeshes(std::string outfilename) const;

  private:
    /// Setup data structures associated with triangle mesh.
    void SetMeshes(const std::vector<geometry::ChTriangleMeshConnected>& all_meshes, std::vector<float> masses);

    CHGPU_MESH_VERBOSITY mesh_verbosity;  ///< mesh operations verbosity level
};

/// @} gpu_physics

}  // namespace gpu
}  // namespace chrono
