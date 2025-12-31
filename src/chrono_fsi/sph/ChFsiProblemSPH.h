// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Utility class to set up an SPH-based Chrono::FSI problem.
//
// =============================================================================

#ifndef CH_FSI_PROBLEM_SPH_H
#define CH_FSI_PROBLEM_SPH_H

#include <cmath>
#include <unordered_set>
#include <unordered_map>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/functions/ChFunction.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiSplashsurfSPH.h"
#include "chrono_fsi/sph/physics/SphParticleRelocator.cuh"

namespace chrono {
namespace fsi {
namespace sph {

/// @addtogroup fsisph
/// @{

/// Base class to set up a Chrono::FSI problem.
class CH_FSI_API ChFsiProblemSPH {
  public:
    /// Enable verbose output during construction of ChFsiProblemSPH (default: false).
    void SetVerbose(bool verbose);

    /// Attach Chrono MBS system.
    void AttachMultibodySystem(ChSystem* sys);

    /// Access the underlying FSI system.
    std::shared_ptr<ChFsiSystemSPH> GetFsiSystemSPH() { return m_sysFSI; }

    /// Access the underlying SPH system.
    std::shared_ptr<ChFsiFluidSystemSPH> GetFluidSystemSPH() { return m_sysSPH; }

    /// Access the underlying MBS system.
    ChSystem& GetMultibodySystem() { return m_sysFSI->GetMultibodySystem(); }

    /// Enable solution of a CFD problem.
    void SetCfdSPH(const ChFsiFluidSystemSPH::FluidProperties& fluid_props);

    /// Enable solution of elastic SPH (for continuum representation of granular dynamics).
    /// By default, a ChSystemFSI solves an SPH fluid dynamics problem.
    void SetElasticSPH(const ChFsiFluidSystemSPH::ElasticMaterialProperties& mat_props);

    /// Set SPH method parameters.
    void SetSPHParameters(const ChFsiFluidSystemSPH::SPHParameters& sph_params);

    /// Set surface reconstruction parameters (`with splashsurf`).
    void SetSplashsurfParameters(const ChFsiFluidSystemSPH::SplashsurfParameters& params);

    /// Add a rigid body to the FSI problem.
    /// BCE markers are created for the provided geometry (which may or may not match the body collision geometry).
    /// By default, where applicable, BCE markers are created using polar coordinates (in layers starting from the shape
    /// surface). Generation of BCE markers on a uniform Cartesian grid can be enforced setting use_grid_bce=true.
    /// Creation of FSI bodies embedded in the fluid phase is allowed (SPH markers inside the body geometry volume are
    /// pruned). To check for possible overlap with SPH particles, set 'check_embedded=true'.
    /// This function must be called before Initialize().
    void AddRigidBody(std::shared_ptr<ChBody> body,
                      std::shared_ptr<utils::ChBodyGeometry> geometry,
                      bool check_embedded,
                      bool use_grid_bce = false);

    void AddRigidBodySphere(std::shared_ptr<ChBody> body,
                            const ChVector3d& pos,
                            double radius,
                            bool use_grid_bce = false);
    void AddRigidBodyBox(std::shared_ptr<ChBody> body, const ChFramed& pos, const ChVector3d& size);
    void AddRigidBodyCylinderX(std::shared_ptr<ChBody> body,
                               const ChFramed& pos,
                               double radius,
                               double length,
                               bool use_grid_bce = false);
    void AddRigidBodyMesh(std::shared_ptr<ChBody> body,
                          const ChFramed& pos,
                          const std::string& obj_file,
                          const ChVector3d& interior_point,
                          double scale);

    /// Return the number of BCE markers associated with the specified rigid body.
    size_t GetNumBCE(std::shared_ptr<ChBody> body) const;

    /// Enable use and set method of obtaining FEA node directions for generating FEA BCE marker location.
    /// By default, node directions are not used, resulting in linear interpolation between nodes.
    /// If enabled, exact node direction vectors are received from the FEA solver (NodeDirectionsMode::EXACT), or else
    /// approximated as averages of directions from elements incident to the node.
    void UseNodeDirections(NodeDirectionsMode mode);

    /// Set the BCE marker pattern for 1D flexible solids for subsequent calls to AddFeaMesh.
    /// By default, a full set of BCE markers is used across each section, including a central marker.
    void SetBcePattern1D(BcePatternMesh1D pattern,   ///< marker pattern in cross-section
                         bool remove_center = false  ///< eliminate markers on center line
    );

    /// Set the BCE marker pattern for 2D flexible solids for subsequent calls to AddFeaMesh.
    /// By default, BCE markers are created centered on the mesh surface, with a layer of BCEs on the surface.
    void SetBcePattern2D(BcePatternMesh2D pattern,   ///< pattern of marker locations along normal
                         bool remove_center = false  ///< eliminate markers on surface
    );

    /// Add an FEA mesh to the FSI problem.
    /// BCE markers are created based on the type of elements and the corresponding FEA collision surface.
    /// To check for possible overlap with SPH particles, set 'check_embedded=true'.
    /// This function must be called before Initialize().
    void AddFeaMesh(std::shared_ptr<fea::ChMesh> mesh, bool check_embedded);

    /// Interface for callback to set initial particle pressure, density, viscosity, and velocity.
    class CH_FSI_API ParticlePropertiesCallback {
      public:
        ParticlePropertiesCallback() : p0(0), rho0(0), mu0(0), v0(VNULL), pre_pressure_scale0(1.01) {}
        ParticlePropertiesCallback(const ParticlePropertiesCallback& other) = default;
        virtual ~ParticlePropertiesCallback() {}

        /// Set values for particle properties.
        /// The default implementation sets pressure and velocity to zero and constant density and viscosity.
        /// If an override is provided, it must set *all* particle properties.
        virtual void set(const ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) {
            p0 = 0;
            rho0 = sysSPH.GetDensity();
            mu0 = sysSPH.GetViscosity();
            v0 = VNULL;
            pre_pressure_scale0 = 1.01;
        }

        double p0;
        double rho0;
        double mu0;
        ChVector3d v0;
        double pre_pressure_scale0;
    };

    /// Register a callback for setting SPH particle initial properties.
    void RegisterParticlePropertiesCallback(std::shared_ptr<ParticlePropertiesCallback> callback) {
        m_props_cb = callback;
    }

    /// Set gravitational acceleration for both multibody and fluid systems.
    void SetGravitationalAcceleration(const ChVector3d& gravity) { m_sysFSI->SetGravitationalAcceleration(gravity); }

    /// Set integration step size for fluid dynamics.
    void SetStepSizeCFD(double step) { m_sysFSI->SetStepSizeCFD(step); }

    /// Set integration step size for multibody dynamics.
    /// If a value is not provided, the MBS system is integrated with the same step used for fluid dynamics.
    void SetStepsizeMBD(double step) { m_sysFSI->SetStepsizeMBD(step); }

    /// Explicitly set the computational domain limits.
    /// By default, this encompasses all SPH and BCE markers with no boundary conditions imposed in any direction.
    void SetComputationalDomain(const ChAABB& aabb,
                                BoundaryConditions bc_type = {BCType::NONE, BCType::NONE, BCType::NONE}) {
        m_domain_aabb = aabb;
        m_bc_type = bc_type;
    }

    /// Complete construction of the FSI problem and initialize the FSI system.
    /// After this call, no additional solid bodies should be added to the FSI problem.
    virtual void Initialize();

    /// Print the FSI statistics
    void PrintStats() const;
    void PrintTimeSteps(const std::string& path) const;

    /// Advance the dynamics of the underlying FSI system by the specified step.
    void DoStepDynamics(double step);

    /// Get the ground body.
    std::shared_ptr<ChBody> GetGroundBody() const { return m_ground; }

    /// Get number of SPH particles.
    size_t GetNumSPHParticles() const { return m_sph.size(); }

    /// Get number of boundary BCE markers.
    size_t GetNumBoundaryBCEMarkers() const { return m_bce.size(); }

    /// Get limits of computational domain.
    const ChAABB& GetComputationalDomain() const { return m_domain_aabb; }

    /// Get the boundary condition type for the three sides of the computational domain.
    const BoundaryConditions& GetBoundaryConditionTypes() const { return m_bc_type; }

    /// Get limits of SPH volume.
    const ChAABB& GetSPHBoundingBox() const { return m_sph_aabb; }

    /// Return the FSI applied force on the specified body (as returned by AddRigidBody).
    /// The force is applied at the body COM and is expressed in the absolute frame.
    /// An exception is thrown if the given body was not added through AddRigidBody.
    const ChVector3d& GetFsiBodyForce(std::shared_ptr<ChBody> body) const;

    /// Return the FSI applied torque on on the specified body (as returned by AddRigidBody).
    /// The torque is expressed in the absolute frame.
    /// An exception is thrown if the given body was not added through AddRigidBody.
    const ChVector3d& GetFsiBodyTorque(std::shared_ptr<ChBody> body) const;

    /// Get current estimated RTF (real time factor) for the fluid system.
    double GetRtfCFD() const { return m_sysSPH->GetRtf(); }

    /// Get current estimated RTF (real time factor) for the multibody system.
    double GetRtfMBD() const { return m_sysFSI->GetMultibodySystem().GetRTF(); }

    /// Set SPH simulation data output level (default: STATE_PRESSURE).
    /// Options:
    /// - STATE           marker state, velocity, and acceleration
    /// - STATE_PRESSURE  STATE plus density and pressure
    /// - CFD_FULL        STATE_PRESSURE plus various CFD parameters
    /// - CRM_FULL        STATE_PRESSURE plus normal and shear stress
    void SetOutputLevel(OutputLevel output_level);

    /// Save current SPH and solid data to files.
    /// This functions creates three CSV files (for SPH particles, boundary BCE markers, and solid BCE markers data) in
    /// the directory `sph_dir` and two CSV files (for force and torque on rigid bodies and flexible nodes) in the
    /// directory `fsi_dir`.
    void SaveOutputData(double time, const std::string& sph_dir, const std::string& fsi_dir);

    /// Save the set of initial SPH and BCE grid locations to files in the specified output directory.
    void SaveInitialMarkers(const std::string& out_dir) const;

    /// Reconstruct surface from the current SPH particle data cloud.
    /// This function invokes the external `splashsurf` tool to generate a Wavefront OBJ mesh reconstructed from the
    /// current positions of the SPH pareticles. If splashsurf was not found during configuration, this function is a
    /// no-op. The intermediate data file with SPH particle positions and the resulting mesh file are created in the
    /// specified directory and are named [name].json and [name].obj, respectively. If quiet=true, splashsurf console
    /// output is supressed. This is a blocking operation which can be computationally expensive for large problems.
    /// See ChFsiSplashsurfSPH.
    void WriteReconstructedSurface(const std::string& dir, const std::string& name, bool quiet = false);

    PhysicsProblem GetPhysicsProblem() const { return m_sysSPH->GetPhysicsProblem(); }
    std::string GetPhysicsProblemString() const { return m_sysSPH->GetPhysicsProblemString(); }
    std::string GetSphIntegrationSchemeString() const { return m_sysSPH->GetSphIntegrationSchemeString(); }

    void SetActiveDomain(const ChVector3d& box_dim) { m_sysSPH->SetActiveDomain(box_dim); }

  protected:
    /// Create a ChFsiProblemSPH object.
    /// No SPH parameters are set.
    ChFsiProblemSPH(double spacing, ChSystem* sys = nullptr);

    /// Hash function for a 3D integer grid coordinate.
    struct CoordHash {
        std::size_t operator()(const ChVector3i& p) const {
            size_t h1 = std::hash<int>()(p.x());
            size_t h2 = std::hash<int>()(p.y());
            size_t h3 = std::hash<int>()(p.z());
            return (h1 ^ (h2 << 1)) ^ h3;
        }
    };

    /// Grid points with integer coordinates.
    typedef std::unordered_set<ChVector3i, CoordHash> GridPoints;

    virtual ChVector3i Snap2Grid(const ChVector3d& point) = 0;
    virtual ChVector3d Grid2Point(const ChVector3i& p) = 0;

    /// Prune SPH markers that are inside the solid body volume.
    /// Treat separately primitive shapes (use explicit test for interior points) and mesh shapes (use ProcessBodyMesh).
    void ProcessBody(ChFsiFluidSystemSPH::FsiSphBody& b);

    /// Prune SPH markers that are inside a body mesh volume.
    /// Voxelize the body mesh (at the scaling resolution) and identify grid nodes inside the boundary
    /// defined by the body BCEs. Note that this assumes the BCE markers form a watertight boundary.
    int ProcessBodyMesh(ChFsiFluidSystemSPH::FsiSphBody& b,
                        ChTriangleMeshConnected trimesh,
                        const ChVector3d& interior_point);

    /// Prune SPH markers that overlap with the FEA mesh BCE markers.
    void ProcessFeaMesh1D(ChFsiFluidSystemSPH::FsiSphMesh1D& m);

    /// Prune SPH markers that overlap with the FEA mesh BCE markers.
    void ProcessFeaMesh2D(ChFsiFluidSystemSPH::FsiSphMesh2D& m);

    // Only derived classes can use the following particle and marker relocation functions

    void CreateParticleRelocator();
    void BCEShift(const ChVector3d& shift_dist);
    void SPHShift(const ChVector3d& shift_dist);
    void SPHMoveAABB2AABB(const ChAABB& aabb_src, const ChIntAABB& aabb_dest);
    void ForceProximitySearch();

    std::shared_ptr<ChFsiFluidSystemSPH> m_sysSPH;     ///< underlying Chrono SPH system
    std::shared_ptr<ChFsiSystemSPH> m_sysFSI;          ///< underlying Chrono FSI system
    ChSystem* m_sysMBS;                                ///< associated MBS system
    std::shared_ptr<ChFsiSplashsurfSPH> m_splashsurf;  ///< surface reconstructor
    double m_spacing;                                  ///< particle and marker spacing
    std::shared_ptr<ChBody> m_ground;                  ///< ground body
    GridPoints m_sph;                                  ///< SPH particle grid locations
    GridPoints m_bce;                                  ///< boundary BCE marker grid locations
    ChVector3d m_offset_sph;                           ///< SPH particles offset
    ChVector3d m_offset_bce;                           ///< boundary BCE particles offset
    ChAABB m_domain_aabb;                              ///< computational domain bounding box
    BoundaryConditions m_bc_type;                      ///< boundary conditions in each direction
    ChAABB m_sph_aabb;                                 ///< SPH volume bounding box

    std::unordered_map<std::shared_ptr<ChBody>, size_t>
        m_fsi_bodies;  ///< map from ChBody pointer to index in FSI body list

    std::shared_ptr<ParticlePropertiesCallback> m_props_cb;  ///< callback for particle properties

    std::unique_ptr<SphParticleRelocator> m_relocator;

    bool m_verbose;      ///< if true, write information to standard output
    bool m_initialized;  ///< if true, problem was initialized

    friend class SelectorFunctionWrapper;
};

// ----------------------------------------------------------------------------

/// Class to set up a Chrono::FSI problem using particles and markers on a Cartesian coordinates grid.
class CH_FSI_API ChFsiProblemCartesian : public ChFsiProblemSPH {
  public:
    /// Create a ChFsiProblemSPH object.
    /// No SPH parameters are set.
    ChFsiProblemCartesian(double spacing, ChSystem* sys = nullptr);

    /// Construct using information from the specified files.
    /// The SPH particle and BCE marker locations are assumed to be provided on an integer grid.
    /// Locations in real space are generated using the specified grid separation value and the
    /// patch translated to the specified position.
    void Construct(const std::string& sph_file,      ///< filename with SPH grid particle positions
                   const std::string& bce_file,      ///< filename with BCE grid marker positions
                   const ChVector3d& pos,            ///< reference position,
                   bool use_grid_coordinates = true  ///< if true, uses the data in the file as integer grid
                                                     ///< coordinates, otherwise uses physical positions
    );

    /// Construct SPH particles and optionally BCE markers in a box of given dimensions.
    /// The reference position is the center of the bottom face of the box; in other words, SPH particles are generated
    /// above this location and BCE markers for the bottom boundary are generated below this location.
    /// If created, the BCE markers for the top, bottom, and side walls are adjacent to the SPH domain; 'side_flags' are
    /// boolean combinations of BoxSide enums.
    void Construct(const ChVector3d& box_size,  ///< box dimensions
                   const ChVector3d& pos,       ///< reference position
                   int side_flags               ///< sides for which BCE markers are created
    );

    /// Construct SPH particles and optionally BCE markers from a given heightmap.
    /// The image file is read with STB, using the number of channels defined in the input file and reading
    /// the image as 16-bit (8-bit images are automatically converted). Supported image formats: JPEG, PNG,
    /// BMP, GIF, PSD, PIC, PNM.
    /// Create the SPH particle grid locations for a patch of specified X and Y dimensions with optional
    /// translation. The height at each grid point is obtained through bilinear interpolation from the gray values in
    /// the provided heightmap image (with pure black corresponding to the lower height range and pure white to the
    /// upper height range). SPH particle grid locations are generated to cover the specified depth under each grid
    /// point. If created, BCE marker layers are generated below the bottom-most layer of SPH particles and on the sides
    /// of the patch. 'side_flags' are boolean combinations of BoxSide enums.
    void Construct(const std::string& heightmap_file,  ///< filename for the heightmap image
                   double length,                      ///< patch length (X direction)
                   double width,                       ///< patch width (Y direction)
                   const ChVector2d& height_range,     ///< height range (black to white level)
                   double depth,                       ///< fluid phase depth
                   bool uniform_depth,                 ///< if true, bottom follows surface
                   const ChVector3d& pos,              ///< reference position
                   int side_flags                      ///< sides for which BCE markers are created
    );

    /// Add fixed BCE markers, representing a container for the computational domain.
    /// The specified 'box_size' represents the dimensions of the *interior* of the box.
    /// The reference position is the center of the bottom face of the box.
    /// Boundary BCE markers are created outside this volume, in layers, starting at a distance equal to the spacing.
    /// 'side_flags' are boolean combinations of BoxSide enums.
    /// It is the caller responsibility to ensure that the container BCE markers do not overlap with any SPH particles.
    size_t AddBoxContainer(const ChVector3d& box_size,  ///< box dimensions
                           const ChVector3d& pos,       ///< reference positions
                           int side_flags               ///< sides for which BCE markers are created
    );

  private:
    virtual ChVector3i Snap2Grid(const ChVector3d& point) override;
    virtual ChVector3d Grid2Point(const ChVector3i& p) override;
};

/// Class to construct a wavetank with a rigid piston or flap wavemaker mechanism.
/// The wavetank can include a beach for wave dissipation, by specifying a profile for the bottom.
class CH_FSI_API ChFsiProblemWavetank : public ChFsiProblemCartesian {
  public:
    /// Wavemaker mechanism types.
    enum class WavemakerType { PISTON, FLAP };

    /// Create a ChFsiProblemSPH object.
    /// No SPH parameters are set.
    ChFsiProblemWavetank(double spacing, ChSystem* sys = nullptr);

    /// Interface for callback to specify wave tank profile.
    class CH_FSI_API Profile {
      public:
        virtual ~Profile() {}

        /// Set bottom height at specified downstream location (must be non-negative).
        /// Default implementation corresponds to a tank with horizontal bottom.
        virtual double operator()(double x) = 0;
    };

    /// Set the callback for the bottom tank profile and indicate if an end-wall is constructed.
    /// By default, a tank with flat bottom and with end-wall is constructed
    void SetProfile(std::shared_ptr<Profile> profile, bool end_wall);

    /// Use periodic boundary conditions in lateral direction (default: false).
    /// If not set, side boundary conditions are enforced by constructing lateral walls.
    void SetLateralPeriodicBC(bool periodic_BC) { m_periodic_BC = periodic_BC; }

    /// Add a wave tank with a rigid-body wavemaker (piston-type or flap-type).
    std::shared_ptr<ChBody> ConstructWaveTank(WavemakerType type,                    ///< wave generator type
                                              const ChVector3d& pos,                 ///< reference position
                                              const ChVector3d& box_size,            ///< box dimensions
                                              double depth,                          ///< fluid depth
                                              std::shared_ptr<ChFunction> actuation  ///< actuation function
    );

    /// Complete construction of the FSI problem and initialize the FSI system.
    /// After this call, no additional solid bodies should be added to the FSI problem.
    virtual void Initialize() override;

  private:
    bool m_periodic_BC;
    bool m_end_wall;
    std::shared_ptr<Profile> m_profile;
    std::shared_ptr<ChBody> m_wavemaker_body;
    std::shared_ptr<ChLinkMotor> m_wavemaker_motor;
    ChVector3d m_wavemaker_size;
    ChVector3d m_wavemaker_pos;
};

// ----------------------------------------------------------------------------

/// Class to set up a Chrono::FSI problem using particles and markers on a cylindrical coordinates grid.
class CH_FSI_API ChFsiProblemCylindrical : public ChFsiProblemSPH {
  public:
    /// Create a ChFsiProblemSPH object.
    /// No SPH parameters are set.
    ChFsiProblemCylindrical(double spacing, ChSystem* sys = nullptr);

    /// Construct SPH particles and optionally BCE markers in a cylindrical annulus of given dimensions.
    /// Set inner radius to zero to create a cylindrical container.
    /// The reference position is the center of the bottom face of the cylinder; in other words, SPH particles are
    /// generated above this location and BCE markers for the bottom boundary are generated below this location.
    /// If created, the BCE markers for the bottom and side walls are adjacent to the SPH domain; 'side_flags' are
    /// boolean combinations of CylSide enums.
    void Construct(double radius_inner,    ///< inner radius
                   double radius_outer,    ///< outer radius
                   double height,          ///< height
                   const ChVector3d& pos,  ///< reference position,
                   int side_flags          ///< sides for which BCE markers are created
    );

    /// Add fixed BCE markers, representing a cylindrical annulus container for the computational domain.
    /// Set inner radius to zero to create a cylindrical container.
    /// The cylinder is constructed with its axis along the global Z axis.
    /// The specified dimensions refer to the *interior* of the cylindrical annulus.
    /// 'side_flags' are boolean combinations of CylSide enums.
    size_t AddCylindricalContainer(double radius_inner,    ///< inner radius
                                   double radius_outer,    ///< outer radius
                                   double height,          ///< height
                                   const ChVector3d& pos,  ///< reference position
                                   int side_flags          ///< sides for which BCE markers are created
    );

  private:
    virtual ChVector3i Snap2Grid(const ChVector3d& point) override;
    virtual ChVector3d Grid2Point(const ChVector3i& p) override;
};

// ----------------------------------------------------------------------------

/// Predefined SPH particle initial properties callback (depth-based pressure).
class CH_FSI_API DepthPressurePropertiesCallback : public ChFsiProblemSPH::ParticlePropertiesCallback {
  public:
    DepthPressurePropertiesCallback(double zero_height) : ParticlePropertiesCallback(), zero_height(zero_height) {}

    virtual void set(const ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) override {
        double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
        double c2 = sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed();
        p0 = sysSPH.GetDensity() * gz * (zero_height - pos.z());
        rho0 = sysSPH.GetDensity() + p0 / c2;
        mu0 = sysSPH.GetViscosity();
        v0 = VNULL;
    }

  private:
    double zero_height;
};

// ----------------------------------------------------------------------------

/// Predefined wave tank profile with a ramp beach.
/// Defines the tank bottom profile as:
/// <pre>
/// h = 0,                   if x < x_start
/// h = alpha * (x-x_start), if x > x_start
/// </pre>
class CH_FSI_API WaveTankRampBeach : public ChFsiProblemWavetank::Profile {
  public:
    WaveTankRampBeach(double x_start, double alpha) : x_start(x_start), alpha(alpha) {}

    virtual double operator()(double x) {
        if (x <= x_start)
            return 0;
        return alpha * (x - x_start);
    }

  private:
    double x_start;
    double alpha;
};

/// Predefined wave tank profile with a parabolic beach.
/// Defines the tank bottom profile as:
/// <pre>
/// h = 0,                       if x < x_start
/// h = alpha * sqrt(x-x_start), if x > x_start
/// </pre>
class CH_FSI_API WaveTankParabolicBeach : public ChFsiProblemWavetank::Profile {
  public:
    WaveTankParabolicBeach(double x_start, double alpha) : x_start(x_start), alpha(alpha) {}

    virtual double operator()(double x) {
        if (x <= x_start)
            return 0;
        double xx = x - x_start;
        return alpha * std::sqrt(xx);
    }

  private:
    double x_start;
    double alpha;
};

/// @} fsisph

}  // namespace sph
}  // namespace fsi
}  // namespace chrono

#endif
