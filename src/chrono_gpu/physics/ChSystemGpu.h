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
// Contributors: Nic Olsen, Ruochun Zhang, Dan Negrut, Radu Serban
// =============================================================================

#pragma once

#include <vector>

#include "chrono_gpu/ChApiGpu.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/core/ChTimer.h"

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
    /// Construct system with given sphere radius, density, big domain dimensions and center.
    ChSystemGpu(float sphere_rad,
                float density,
                const ChVector<float>& boxDims,
                ChVector<float> O = ChVector<float>(0));

    /// Construct system with a checkpoint file.
    ChSystemGpu(const std::string& checkpoint);

    virtual ~ChSystemGpu();

    /// Set gravitational acceleration vector.
    void SetGravitationalAcceleration(const ChVector<float>& g);

    /// Set particle positions, velocities and angular velocities.
    void SetParticles(const std::vector<ChVector<float>>& points,
                      const std::vector<ChVector<float>>& vels = std::vector<ChVector<float>>(),
                      const std::vector<ChVector<float>>& ang_vels = std::vector<ChVector<float>>());

    /// Set particle positions, velocities and angular velocities from a file.
    void ReadParticleFile(const std::string& infilename);

    /// Set particle contact friction history from a file.
    void ReadContactHistoryFile(const std::string& infilename);

    // Read in a (Chrono::Gpu generated) checkpoint file to restart a simulation.
    void ReadCheckpointFile(const std::string& infilename, bool overwrite = false);

    /// Set the big domain to be fixed or not.
    /// If fixed, it will ignore any given position functions.
    void SetBDFixed(bool fixed);

    /// Set the center of the big box domain, relative to the origin of the coordinate system (default: [0,0,0]).
    /// Note that the domain is always axis-aligned. The user must make sure that all simulation information (particle
    /// locations, boundaries, meshes...) is consistent with this domain.
    void SetBDCenter(const ChVector<float>& O);

    /// Set flags indicating whether or not a particle is fixed.
    /// MUST be called only once and MUST be called before Initialize.
    void SetParticleFixed(const std::vector<bool>& fixed);

    /// Set the output mode of the simulation.
    void SetParticleOutputMode(CHGPU_OUTPUT_MODE mode);

    /// Set output settings bit flags by bitwise ORing settings in CHGPU_OUTPUT_FLAGS.
    void SetParticleOutputFlags(unsigned int flags);

    /// Set timestep size.
    void SetFixedStepSize(float size_UU);

    /// If yes, on Initialize(), particles will have their order re-arranged so that those in the same SD are close
    /// together. This is usually done if starting from scratch, and optional if this is a re-started simulation. Note
    /// this is by default on, unless the user loads simulation data from files, in which case it gets disabled.
    void SetDefragmentOnInitialize(bool defragment);

    /// Ensure that the deformation-based length unit is used.
    void EnableMinLength(bool useMinLen);
    void DisableMinLength() { EnableMinLength(false); }

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

    /// Set sphere-to-sphere normal contact stiffness.
    void SetKn_SPH2SPH(double someValue);
    /// Set sphere-to-wall normal contact stiffness.
    void SetKn_SPH2WALL(double someValue);

    /// Set sphere-to-sphere normal damping coefficient.
    void SetGn_SPH2SPH(double someValue);
    /// Set sphere-to-wall normal damping coefficient.
    void SetGn_SPH2WALL(double someValue);

    /// Set sphere-to-sphere tangential contact stiffness.
    void SetKt_SPH2SPH(double someValue);
    /// Set sphere-to-sphere tangential damping coefficient.
    void SetGt_SPH2SPH(double someValue);

    /// Set sphere-to-wall tangential contact stiffness.
    void SetKt_SPH2WALL(double someValue);
    /// Set sphere-to-wall tangential damping coefficient.
    void SetGt_SPH2WALL(double someValue);

    /// Set the ratio of cohesion to gravity for monodisperse spheres. Assumes a constant cohesion model.
    void SetCohesionRatio(float someValue);

    /// Set the ratio of adhesion to gravity for sphere to wall. Assumes a constant cohesion model.
    void SetAdhesionRatio_SPH2WALL(float someValue);

    void UseMaterialBasedModel(bool val);

    /// Set youngs modulus of spheres
    void SetYoungModulus_SPH(double someValue);
    /// Set youngs modulus of boundary
    void SetYoungModulus_WALL(double someValue);

    /// Set poisson ratio of sphere
    void SetPoissonRatio_SPH(double someValue);
    /// Set poisson ratio of boundary
    void SetPoissonRatio_WALL(double someValue);

    /// Set coefficient of restitution of spheres
    void SetRestitution_SPH(double someValue);
    /// Set coefficient of restitution of spheres
    void SetRestitution_WALL(double someValue);

    /// Safety check velocity to ensure the simulation is still stable.
    void SetMaxSafeVelocity_SU(float max_vel);

    /// Set tuning psi factors for tuning the non-dimensionalization.
    void SetPsiFactors(unsigned int psi_T, unsigned int psi_L, float psi_R = 1.f);
    void SetPsiT(unsigned int psi_T);
    void SetPsiL(unsigned int psi_L);
    void SetPsiR(float psi_R = 1.f);

    /// Enable/disable recording of contact info.
    void SetRecordingContactInfo(bool record);

    /// Manually set the simulation time (mainly used for restarted simulation).
    void SetSimTime(float time);

    /// Set simualtion verbosity level.
    void SetVerbosity(CHGPU_VERBOSITY level);

    /// Create an axis-aligned sphere boundary condition.
    size_t CreateBCSphere(const ChVector<float>& center,
                          float radius,
                          bool outward_normal,
                          bool track_forces,
                          float mass);

    // void UpdateBCSpherePosition(size_t sphere_bc_id, ChVector<double> position);

    /// Create a Z-axis aligned cone boundary condition.
    size_t CreateBCConeZ(const ChVector<float>& tip,
                         float slope,
                         float hmax,
                         float hmin,
                         bool outward_normal,
                         bool track_forces);

    /// Create a plane boundary condition.
    size_t CreateBCPlane(const ChVector<float>& pos, const ChVector<float>& normal, bool track_forces);

    /// create a plate boundary condition
    size_t CreateCustomizedPlate(const ChVector<float>& pos_center, const ChVector<float>& normal, float hdim_y);

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

    // -------------------------- A plethora of "Get" methods -------------------------------- //

    /// Return current simulation time.
    float GetSimTime() const;

    /// Return the total number of particles in the system
    size_t GetNumParticles() const;

    /// Return the maximum Z position over all particles.
    double GetMaxParticleZ() const;

    /// Return the minimum Z position over all particles.
    double GetMinParticleZ() const;

    /// Return the number of particles that are higher than a given Z coordinate
    unsigned int GetNumParticleAboveZ(float ZValue) const;

    /// Return the number of particles that are higher than a given X coordinate
    unsigned int GetNumParticleAboveX(float XValue) const;

    /// Return the radius of a spherical particle.
    float GetParticleRadius() const;

    /// Return particle position.
    ChVector<float> GetParticlePosition(int nSphere) const;

    /// Set particle position
    void SetParticlePosition(int nSphere, const ChVector<double> pos);

    /// Set particle density
    void SetParticleDensity(float density);

    /// Set particle radius
    void SetParticleRadius(float rad);

    /// Set particle velocity
    void SetParticleVelocity(int nSphere, const ChVector<double> velo);

    /// Return particle angular velocity.
    ChVector<float> GetParticleAngVelocity(int nSphere) const;

    /// return particle acc
    ChVector<float> GetParticleLinAcc(int nSphere) const;

    /// Return whether or not the particle is fixed
    bool IsFixed(int nSphere) const;

    /// Return particle linear velocity.
    ChVector<float> GetParticleVelocity(int nSphere) const;

    /// Return the total kinetic energy of all particles.
    float GetParticlesKineticEnergy() const;

    /// Return position of BC plane.
    ChVector<float> GetBCPlanePosition(size_t plane_id) const;

    /// Return position of BC sphere
    ChVector<float> GetBCSpherePosition(size_t sphere_id) const;

    /// Set position of BC spheres
    void SetBCSpherePosition(size_t sphere_bc_id, const ChVector<float>& pos);

    /// Return velocity of BC sphere
    ChVector<float> GetBCSphereVelocity(size_t sphere_id) const;

    /// Set velocity of BC spheres
    void SetBCSphereVelocity(size_t sphere_bc_id, const ChVector<float>& velo);

    /// Set BC plane rotation
    void SetBCPlaneRotation(size_t plane_id, ChVector<double> center, ChVector<double> omega);

    /// Get the reaction forces on a boundary by ID, returns false if the forces are invalid (bad BC ID)
    bool GetBCReactionForces(size_t BC_id, ChVector<float>& force) const;

    /// Return number of particle-particle contacts.
    int GetNumContacts() const;

    /// Return number of subdomains in the big domain.
    unsigned int GetNumSDs() const;

    // ------------------------------- End of "Get" methods -------------------------------//

    /// Initialize simulation so that it can be advanced.
    /// Must be called before AdvanceSimulation and after simulation parameters are set.
    virtual void Initialize();

    /// Advance simulation by duration in user units, return actual duration elapsed.
    /// Requires Initialize() to have been called.
    virtual double AdvanceSimulation(float duration);

    /// Write a one-stop checkpoint file for Chrono::Gpu.
    /// All information defining a simulation is in this file.
    void WriteCheckpointFile(const std::string& outfilename);

    /// Write particle positions according to the system output mode.
    void WriteParticleFile(const std::string& outfilename) const;

    /// Write contact pair history to a file.
    void WriteContactHistoryFile(const std::string& outfilename) const;

    /// Write contact force and torque to a file.
    void WriteContactInfoFile(const std::string& outfilename) const;

    /// Roughly estimate of the total amount of memory used by the system.
    size_t EstimateMemUsage() const;

    /// Get rolling friction torque between body i and j, return 0 if not in contact
    ChVector<float> getRollingFrictionTorque(unsigned int i, unsigned int j);

    /// Get tangential friction force between body i and j, return 0 if not in contact
    ChVector<float> getSlidingFrictionForce(unsigned int i, unsigned int j);

    /// Get normal friction force between body i and j, return 0 if not in contact
    ChVector<float> getNormalForce(unsigned int i, unsigned int j);

    /// Get v_rot for rolling friction
    ChVector<float> getRollingVrot(unsigned int i, unsigned int j);

    /// get contact char time
    float getRollingCharContactTime(unsigned int i, unsigned int j);

    /// get index list of neighbors
    void getNeighbors(unsigned int ID, std::vector<unsigned int>& neighborList);

    /// Get current estimated RTF (real time factor).
    float GetRTF() const { return m_RTF; }


  protected:
    /// Protected default constructor.  Derived class must create m_sys.
    ChSystemGpu() : m_sys(nullptr) {}

    ChSystemGpu_impl* m_sys;  ///< underlying system implementation

    /// Set particle positions, velocities and angular velocities from a CSV ifstream.
    /// Methods that read sphere position/velocity info from a file serve as its wrapper.
    void ReadCsvParticles(std::ifstream& ifile, unsigned int totRow = UINT_MAX);

    /// Set particle contact friction history from a hst ifstream.
    /// Methods that read history info from a file serve as its wrapper.
    void ReadHstHistory(std::ifstream& ifile, unsigned int totItem = UINT_MAX);

    /// Give a string identifier, set the corresponding simulation parameter, using a switch statement.
    /// ReadDatParams() is its wrapper.
    /// It must be virtual, because derived classes also use it (and may call it from a inherited method), and read some
    /// more data (thus built on top of it). We must ensure those derived classes call the correct version of it.
    virtual bool SetParamsFromIdentifier(const std::string& identifier, std::istringstream& iss1, bool overwrite);

    /// Set simulation params from a DAT checkpoint file stream. Returns the number of particles.
    /// If instructed to overwrite, then overwrite current simulation parameters with the values in the checkpoint file;
    /// else, when an inconsistency is found, throw an error.
    /// ReadCheckpointFile() is its wrapper.
    unsigned int ReadDatParams(std::ifstream& ifile, bool overwrite);

    /// Write simulation params to a stream. WriteCheckpointFile() is its wrapper.
    void WriteCheckpointParams(std::ofstream& cpFile) const;

    /// Write particle position, velocity and ang. vel. to a stream (of several possible formats).
    /// WriteCheckpointFile() and WriteParticleFile() are their wrappers.
    void WriteCsvParticles(std::ofstream& ptFile) const;
    void WriteRawParticles(std::ofstream& ptFile) const;
    void WriteChPFParticles(std::ofstream& ptFile) const;
#ifdef USE_HDF5
    void WriteH5Particles(H5::H5File& ptFile) const;
#endif

    /// Write contact pair/history to a stream.
    /// WriteCheckpointFile() and WriteContactHistoryFile() are its wrappers.
    void WriteHstHistory(std::ofstream& histFile) const;

    /// Set gravitational acceleration as a float3 vector.
    void SetGravitationalAcceleration(const float3 g);

    ChTimer m_timer; 
    float m_RTF;  // real-time factor
};

// -----------------------------------------------------------------------------

/// Interface to a Chrono::Gpu mesh system.
class CH_GPU_API ChSystemGpuMesh : public ChSystemGpu {
  public:
    /// Construct system with given sphere radius, density, big domain dimensions and center.
    ChSystemGpuMesh(float sphere_rad,
                    float density,
                    const ChVector<float>& boxDims,
                    ChVector<float> O = ChVector<float>(0));

    /// Construct system with a checkpoint file.
    ChSystemGpuMesh(const std::string& checkpoint);

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

    /// Enable/disable mesh normal-based orientation correction.
    void UseMeshNormals(bool val) { use_mesh_normals = val; }

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

    /// Set sphere-to-mesh normal contact stiffness.
    void SetKn_SPH2MESH(double someValue);
    /// Set sphere-to-mesh normal damping coefficient.
    void SetGn_SPH2MESH(double someValue);

    /// Set sphere-to-mesh tangential contact stiffness.
    void SetKt_SPH2MESH(double someValue);
    /// Set sphere-to-mesh tangential damping coefficient.
    void SetGt_SPH2MESH(double someValue);

    void UseMaterialBasedModel(bool val);

    /// Set material-based contact model parameters
    void SetYoungModulus_MESH(double someValue);

    void SetPoissonRatio_MESH(double someValue);

    void SetRestitution_MESH(double someValue);

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

    /// GpuMesh version of checkpoint loading from a file.
    void ReadCheckpointFile(const std::string& infilename, bool overwrite = false);

    /// GpuMesh version of checkpoint generating subroutine. Has a bit more content than parent.
    void WriteCheckpointFile(const std::string& outfilename);

    /// Write the i-th mesh cached in m_meshes, with the current position.
    void WriteMesh(const std::string& outfilename, unsigned int i) const;

    /// Write all the meshes cached in m_meshes into a combined file, with their current positions.
    void WriteMeshes(const std::string& outfilename) const;

  private:
    /// Set triangle meshes in underlying GPU system.
    void SetMeshes();

    CHGPU_MESH_VERBOSITY mesh_verbosity;                                       ///< mesh operations verbosity level
    std::vector<std::shared_ptr<geometry::ChTriangleMeshConnected>> m_meshes;  ///< list of meshes used in cosimulation
    std::vector<float> m_mesh_masses;                                          ///< associated mesh masses
    bool use_mesh_normals =
        false;  ///< true: use mesh normals in file to correct mesh orientation; false: do nothing, implicitly use RHR

    /// GpuMesh version of setting simulation params based on identifiers in the checkpoint file.
    virtual bool SetParamsFromIdentifier(const std::string& identifier,
                                         std::istringstream& iss1,
                                         bool overwrite) override;

    /// GpuMesh version of parameter writing subroutine
    void WriteCheckpointMeshParams(std::ofstream& cpFile) const;
};

/// @} gpu_physics

}  // namespace gpu
}  // namespace chrono
