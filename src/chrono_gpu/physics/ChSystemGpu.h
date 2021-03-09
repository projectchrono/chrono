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

#include "chrono_gpu/ChApiGpu.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_gpu/ChGpuDefines.h"

namespace chrono {
namespace gpu {

/// @addtogroup gpu_physics
/// @{

// Forward declarations
class ChSystemGpu_impl;
class ChSystemGpuMesh_impl;

// -----------------------------------------------------------------------------

/// Interface to a Chrono::Gpu system.
class CH_GPU_API ChSystemGpu {
  public:
    /// Construct system with given sphere radius, density, and big domain dimensions.
    ChSystemGpu(float sphere_rad, float density, float3 boxDims);

    virtual ~ChSystemGpu();

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
    /// friction.
    void SetFrictionMode(CHGPU_FRICTION_MODE new_mode);

    /// Set rolling resistence formulation.
    /// NOTE: This requires friction to be active, otherwise this setting will be ignored.
    void SetRollingMode(CHGPU_ROLLING_MODE new_mode);

    /// Set sphere-to-sphere static friction coefficient.
    void SetStaticFrictionCoeff_SPH2SPH(float mu);
    /// Set sphere-to-wall static friction coefficient.
    void SetStaticFrictionCoeff_SPH2WALL(float mu);
    /// Set sphere-to-sphere rolling friction coefficient -- units and use vary by rolling friction mode.
    void SetRollingCoeff_SPH2SPH(float mu);
    /// Set sphere-to-wall rolling friction coefficient -- units and use vary by rolling friction mode.
    void SetRollingCoeff_SPH2WALL(float mu);

    /// Set sphere-to-sphere spinning friction coefficient -- units and use vary by spinning friction mode.
    void SetSpinningCoeff_SPH2SPH(float mu);
    /// Set sphere-to-wall spinning friction coefficient -- units and use vary by spinning friction mode.
    void SetSpinningCoeff_SPH2WALL(float mu);

    /// Set sphere-to-sphere normal contact stiffness
    void SetKn_SPH2SPH(double someValue);
    /// Set sphere-to-wall normal contact stiffness
    void SetKn_SPH2WALL(double someValue);

    /// Set sphere-to-sphere normal damping coefficient
    void SetGn_SPH2SPH(double someValue);
    /// Set sphere-to-wall normal damping coefficient
    void SetGn_SPH2WALL(double someValue);

    /// Set sphere-to-sphere tangential contact stiffness
    void SetKt_SPH2SPH(double someValue);
    /// Set sphere-to-sphere tangential damping coefficient
    void SetGt_SPH2SPH(double someValue);

    /// Set sphere-to-wall tangential contact stiffness
    void SetKt_SPH2WALL(double someValue);
    /// Set sphere-to-wall tangential damping coefficient
    void SetGt_SPH2WALL(double someValue);

    /// Set the ratio of cohesion to gravity for monodisperse spheres. Assumes a constant cohesion model
    void SetCohesionRatio(float someValue);

    /// Set the ratio of adhesion to gravity for sphere to wall. Assumes a constant cohesion model
    void SetAdhesionRatio_SPH2WALL(float someValue);

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

    /// Enable a boundary condition by its ID, returns false if the BC does not exist.
    bool SetBCOffsetFunction(size_t BC_id, const GranPositionFunction& offset_function);

    /// Prescribe the motion of the big domain, allows wavetank-style simulations.
    void setBDWallsMotionFunction(const GranPositionFunction& pos_fn);

    /// Return current simulation time.
    float GetSimTime() const;

    /// Return the total number of particles in the system
    size_t GetNumParticles() const;

    /// Return the radius of a spherical particle.
    float GetParticleRadius() const;

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

    /// Add a trimesh to the granular system.
    /// The return value is a mesh identifier which can be used during the simulation to apply rigid body motion to the
    /// mesh; see ApplyMeshMotion(). This function must be called before Initialize().
    unsigned int AddMesh(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh, float mass);

    /// Add a trimesh from the specified Wavefront OBJ file to the granular system.
    /// The return value is a mesh identifier which can be used during the simulation to apply rigid body motion to the
    /// mesh; see ApplyMeshMotion(). This function must be called before Initialize().
    unsigned int AddMesh(const std::string& filename,
                         const ChVector<float>& translation,
                         const ChMatrix33<float>& rotscale,
                         float mass);

    /// Add a set of trimeshes from Wavefront OBJ files into granular system.
    /// The return value is a vector of mesh identifiers which can be used during the simulation to apply rigid body
    /// motion to the mesh; see ApplyMeshMotion(). This function must be called before Initialize().
    std::vector<unsigned int> AddMeshes(const std::vector<std::string>& objfilenames,
                                        const std::vector<ChVector<float>>& translations,
                                        const std::vector<ChMatrix33<float>>& rotscales,
                                        const std::vector<float>& masses);

    /// Enable/disable mesh collision (for all defined meshes).
    void EnableMeshCollision(bool val);

    /// Apply rigid body motion to specified mesh.
    void ApplyMeshMotion(unsigned int mesh_id,
                         const ChVector<>& pos,
                         const ChQuaternion<>& rot,
                         const ChVector<>& lin_vel,
                         const ChVector<>& ang_vel);

    /// Return the number of meshes in the system.
    unsigned int GetNumMeshes() const;

    /// Return the specified mesh in the system.
    /// The mesh is assumed to have been added with one of the AddMesh() functions.
    std::shared_ptr<geometry::ChTriangleMeshConnected> GetMesh(unsigned int mesh_id) const { return m_meshes[mesh_id]; }

    /// Return the mass of the specified mesh.
    /// The mesh is assumed to have been added with one of the AddMesh() functions.
    float GetMeshMass(unsigned int mesh_id) const { return m_mesh_masses[mesh_id]; }

    /// Set sphere-to-mesh static friction coefficient.
    void SetStaticFrictionCoeff_SPH2MESH(float mu);
    /// Set sphere-to-mesh rolling friction coefficient.
    void SetRollingCoeff_SPH2MESH(float mu);
    /// Set sphere-to-mesh spinning friction coefficient.
    void SetSpinningCoeff_SPH2MESH(float mu);

    /// Set sphere-to-mesh normal contact stiffness
    void SetKn_SPH2MESH(double someValue);
    /// Set sphere-to-mesh normal damping coefficient
    void SetGn_SPH2MESH(double someValue);

    /// Set sphere-to-mesh tangential contact stiffness
    void SetKt_SPH2MESH(double someValue);
    /// Set sphere-to-mesh tangential damping coefficient
    void SetGt_SPH2MESH(double someValue);

    /// Set the ratio of adhesion force to sphere weight for sphere to mesh.
    void SetAdhesionRatio_SPH2MESH(float someValue);

    /// Set verbosity level of mesh operations.
    void SetMeshVerbosity(CHGPU_MESH_VERBOSITY level);

    /// Initialize simulation so that it can be advanced.
    /// Must be called before AdvanceSimulation and after simulation parameters are set.
    /// This function initializes both the granular material and any existing trimeshes.
    virtual void Initialize() override;

    /// Initialize only the trimeshes (assumes the granular material was already initialized).
    /// Must be called before AdvanceSimulation and after simulation parameters are set.
    void InitializeMeshes();

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
    /// Set triangle meshes in underlying GPU system.
    void SetMeshes();

    CHGPU_MESH_VERBOSITY mesh_verbosity;                                       ///< mesh operations verbosity level
    std::vector<std::shared_ptr<geometry::ChTriangleMeshConnected>> m_meshes;  ///< list of meshes used in cosimulation
    std::vector<float> m_mesh_masses;                                          ///< associated mesh masses
};

/// @} gpu_physics

}  // namespace gpu
}  // namespace chrono
