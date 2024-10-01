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
// Utility class to set up a Chrono::FSI problem.
//
// =============================================================================

#ifndef CH_FSI_PROBLEM_H
#define CH_FSI_PROBLEM_H

#include <unordered_set>
#include <unordered_map>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChBodyGeometry.h"
#include "chrono/functions/ChFunction.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChSystemFsi.h"

namespace chrono {
namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// Base class to set up a Chrono::FSI problem.
class CH_FSI_API ChFsiProblem {
  public:
    /// Wave generator types.
    enum class WavemakerType { PISTON, FLAP };

    /// Enable verbose output during construction of ChFsiProblem (default: false).
    void SetVerbose(bool verbose);

    /// Access the underlying MBS system.
    ChSystem& GetSystyemMBS() { return m_sys; }

    /// Access the underlying FSI system.
    ChSystemFsi& GetSystemFSI() { return m_sysFSI; }

    /// Add a rigid body to the FSI problem.
    /// BCE markers are created for the provided geometry (which may or may not match the body collision geometry).
    /// By default, where applicable, BCE markers are created using polar coordinates (in layers starting from the shape
    /// surface). Generation of BCE markers on a uniform Cartesian grid can be enforced setting use_grid_bce=true.
    /// Creation of FSI bodies embedded in the fluid phase is allowed (SPH markers inside the body geometry volume are
    /// pruned). To check for possible overlap with SPH particles, set 'check_embedded=true'.
    /// This function must be called before Initialize().
    size_t AddRigidBody(std::shared_ptr<ChBody> body,
                        const chrono::utils::ChBodyGeometry& geometry,
                        bool check_embedded,
                        bool use_grid_bce = false);

    size_t AddRigidBodySphere(std::shared_ptr<ChBody> body,
                              const ChVector3d& pos,
                              double radius,
                              bool use_grid_bce = false);
    size_t AddRigidBodyBox(std::shared_ptr<ChBody> body, const ChFramed& pos, const ChVector3d& size);
    size_t AddRigidBodyCylinderX(std::shared_ptr<ChBody> body,
                                 const ChFramed& pos,
                                 double radius,
                                 double length,
                                 bool use_grid_bce = false);
    size_t AddRigidBodyMesh(std::shared_ptr<ChBody> body,
                            const ChVector3d& pos,
                            const std::string& obj_file,
                            const ChVector3d& interior_point,
                            double scale);

    /// Interface for callback to set initial particle pressure, density, viscosity, and velocity.
    class CH_FSI_API ParticlePropertiesCallback {
      public:
        ParticlePropertiesCallback(const ChSystemFsi& sysFSI) : sysFSI(sysFSI), p0(0), rho0(0), mu0(0), v0(VNULL) {}
        ParticlePropertiesCallback(const ParticlePropertiesCallback& other) = default;
        virtual ~ParticlePropertiesCallback() {}

        /// Set values for particle properties.
        /// The default implementation sets pressure and velocity to zero and constant density and viscosity.
        /// If an override is provided, it must set *all* particle properties.
        virtual void set(const ChVector3d& pos) {
            p0 = 0;
            rho0 = sysFSI.GetDensity();
            mu0 = sysFSI.GetViscosity();
            v0 = VNULL;
        }

        const ChSystemFsi& sysFSI;
        double p0;
        double rho0;
        double mu0;
        ChVector3d v0;
    };

    /// Register a callback for setting SPH particle initial properties.
    void RegisterParticlePropertiesCallback(std::shared_ptr<ParticlePropertiesCallback> callback) {
        m_props_cb = callback;
    }

    /// Explicitly set the computational domain limits.
    /// By default, this is set so that it encompasses all SPH particles and BCE markers.
    void SetComputationalDomainSize(ChAABB aabb) { m_domain_aabb = aabb; }

    /// Complete construction of the FSI problem and initialize the FSI system.
    /// After this call, no additional solid bodies should be added to the FSI problem.
    void Initialize();

    /// Get the ground body.
    std::shared_ptr<ChBody> GetGroundBody() const { return m_ground; }

    /// Get number of SPH particles.
    size_t GetNumSPHParticles() const { return m_sph.size(); }

    /// Get number of boundary BCE markers.
    size_t GetNumBoundaryBCEMarkers() const { return m_bce.size(); }

    /// Get limits of computational domain.
    const ChAABB& GetComputationalDomainSize() const { return m_domain_aabb; }

    /// Get limits of SPH volume
    const ChAABB& GetSPHBoundingBox() const { return m_sph_aabb; }

    /// Return the FSI applied force on the specified body (as returned by AddRigidBody).
    /// The force is applied at the body COM and is expressed in the absolute frame.
    /// An exception is thrown if the given body was not added through AddRigidBody.
    const ChVector3d& GetFsiBodyForce(std::shared_ptr<ChBody> body) const;

    /// Return the FSI applied torque on on the specified body (as returned by AddRigidBody).
    /// The torque is expressed in the absolute frame.
    /// An exception is thrown if the given body was not added through AddRigidBody.
    const ChVector3d& GetFsiBodyTorque(std::shared_ptr<ChBody> body) const;

    /// Save the set of initial SPH and BCE grid locations to files in the specified output directory.
    void SaveInitialMarkers(const std::string& out_dir) const;

  protected:
    /// Create a ChFsiProblem object.
    /// No SPH parameters are set.
    ChFsiProblem(ChSystem& sys, double spacing);

    /// Hash function for a 3D integer grid coordinate.
    struct CoordHash {
        std::size_t operator()(const ChVector3i& p) const {
            size_t h1 = std::hash<int>()(p.x());
            size_t h2 = std::hash<int>()(p.y());
            size_t h3 = std::hash<int>()(p.z());
            return (h1 ^ (h2 << 1)) ^ h3;
        }
    };

    virtual ChVector3i Snap2Grid(const ChVector3d& point) = 0;
    virtual ChVector3d Grid2Point(const ChVector3i& p) = 0;

    typedef std::unordered_set<ChVector3i, CoordHash> GridPoints;
    typedef std::vector<ChVector3d> RealPoints;

    /// Specification of an FSI rigid body.
    struct RigidBody {
        std::shared_ptr<ChBody> body;            ///< associated body
        chrono::utils::ChBodyGeometry geometry;  ///< geometry for body BCE
        bool check_embedded;                     ///< if true, check for overlapping SPH particles
        RealPoints bce;                          ///< body BCE marker locations
        ChVector3d oobb_center;                  ///< center of bounding box
        ChVector3d oobb_dims;                    ///< dimensions of bounding box
    };

    /// Prune SPH markers that are inside the solid body volume.
    /// Treat separately primitive shapes (use explicit test for interior points) and mesh shapes (use ProcessBodyMesh).
    void ProcessBody(RigidBody& b);

    /// Prune SPH markers that are inside a body mesh volume.
    /// Voxelize the body mesh (at the scaling resolution) and identify grid nodes inside the boundary
    /// defined by the body BCEs. Note that this assumes the BCE markers form a watertight boundary.
    int ProcessBodyMesh(RigidBody& b, ChTriangleMeshConnected trimesh, const ChVector3d& interior_point);

    ChSystemFsi m_sysFSI;              ///< underlying Chrono FSI system
    ChSystem& m_sys;                   ///< associated Chrono MBS system
    double m_spacing;                  ///< particle and marker spacing
    std::shared_ptr<ChBody> m_ground;  ///< ground body
    GridPoints m_sph;                  ///< SPH particle grid locations
    GridPoints m_bce;                  ///< boundary BCE marker grid locations
    ChVector3d m_offset_sph;           ///< SPH particles offset
    ChVector3d m_offset_bce;           ///< boundary BCE particles offset
    ChAABB m_domain_aabb;              ///< computational domain bounding box
    ChAABB m_sph_aabb;                 ///< SPH volume bounding box
    std::vector<RigidBody> m_bodies;   ///< list of FSI rigid bodies

    std::unordered_map<std::shared_ptr<ChBody>, size_t> m_fsi_bodies;

    std::shared_ptr<ParticlePropertiesCallback> m_props_cb;  ///< callback for particle properties

    bool m_verbose;      ///< if true, write information to standard output
    bool m_initialized;  ///< set to 'true' once terrain is initialized
};

// ----------------------------------------------------------------------------

/// Class to set up a Chrono::FSI problem using particles and markers on a Cartesian coordinates grid.
class CH_FSI_API ChFsiProblemCartesian : public ChFsiProblem {
  public:
    /// Create a ChFsiProblem object.
    /// No SPH parameters are set.
    ChFsiProblemCartesian(ChSystem& sys, double spacing);

    /// Construct using information from the specified files.
    /// The SPH particle and BCE marker locations are assumed to be provided on an integer grid.
    /// Locations in real space are generated using the specified grid separation value and the
    /// patch translated to the specified position.
    void Construct(const std::string& sph_file,  ///< filename with SPH grid particle positions
                   const std::string& bce_file,  ///< filename with BCE grid marker positions
                   const ChVector3d& pos         ///< reference position
    );

    /// Construct SPH particles and optionally BCE markers in a box of given dimensions.
    /// The reference position is the center of the bottom face of the box; in other words, SPH particles are generated
    /// above this location and BCE markers for the bottom boundary are generated below this location.
    /// If created, the BCE markers for the bottom and side walls are adjacent to the SPH domain.
    void Construct(const ChVector3d& box_size,  ///< box dimensions
                   const ChVector3d& pos,       ///< reference position
                   bool bottom_wall,            ///< create bottom boundary
                   bool side_walls              ///< create side boundaries
    );

    /// Construct SPH particles and optionally BCE markers from a given heightmap.
    /// The image file is read with STB, using the number of channels defined in the input file and reading
    /// the image as 16-bit (8-bit images are automatically converted). Supported image formats: JPEG, PNG,
    /// BMP, GIF, PSD, PIC, PNM.
    /// Create the SPH particle grid locations for a patch of specified X and Y dimensions with optional
    /// translation. The height at each grid point is obtained through bilinear interpolation from the gray values in
    /// the provided heightmap image (with pure black corresponding to the lower height range and pure white to the
    /// upper height range). SPH particle grid locations are generated to cover the specified depth under each grid
    /// point. BCE marker layers are created below the bottom-most layer of SPH particles and (optionally) on the sides
    /// of the patch.
    void Construct(const std::string& heightmap_file,  ///< filename for the heightmap image
                   double length,                      ///< patch length (X direction)
                   double width,                       ///< patch width (Y direction)
                   const ChVector2d& height_range,     ///< height range (black to white level)
                   double depth,                       ///< fluid phase depth
                   bool uniform_depth,                 ///< if true, bottom follows surface
                   const ChVector3d& pos,              ///< reference position
                   bool bottom_wall,                   ///< create bottom boundary
                   bool side_walls                     ///< create side boundaries
    );

    /// Add fixed BCE markers, representing a container for the computational domain.
    /// The specified 'box_size' represents the dimensions of the *interior* of the box.
    /// The reference position is the center of the bottom face of the box.
    /// Boundary BCE markers are created outside this volume, in layers, starting at a distance equal to the spacing.
    /// It is the caller responsibility to ensure that the container BCE markers do not overlap with any SPH particles.
    size_t AddBoxContainer(const ChVector3d& box_size,  ///< box dimensions
                           const ChVector3d& pos,       ///< reference position
                           bool bottom_wall,            ///< create bottom boundary
                           bool side_walls,             ///< create side boundaries
                           bool top_wall                ///< create top boundary
    );

    /// Add a rigid-body wavemaker (piston-type or flap-type).
    std::shared_ptr<ChBody> AddWaveMaker(WavemakerType type,                     ///< wave generator type
                                         const ChVector3d& box_size,             ///< box dimensions
                                         const ChVector3d& pos,                  ///< reference position
                                         std::shared_ptr<ChFunction> piston_fun  ///< piston actuation function
    );

  private:
    virtual ChVector3i Snap2Grid(const ChVector3d& point) override;
    virtual ChVector3d Grid2Point(const ChVector3i& p) override;
};

/// Class to set up a Chrono::FSI problem using particles and markers on a cylindrical coordinates grid.
class CH_FSI_API ChFsiProblemCylindrical : public ChFsiProblem {
  public:
    /// Create a ChFsiProblem object.
    /// No SPH parameters are set.
    ChFsiProblemCylindrical(ChSystem& sys, double spacing);

    /// Construct SPH particles and optionally BCE markers in a cylindrical annulus of given dimensions.
    /// Set inner radius to zero to create a cylindrical container.
    /// The reference position is the center of the bottom face of the cylinder; in other words, SPH particles are
    /// generated above this location and BCE markers for the bottom boundary are generated below this location. If
    /// created, the BCE markers for the bottom and side walls are adjacent to the SPH domain.
    void Construct(double radius_inner,
                   double radius_outer,
                   double height,
                   const ChVector3d& pos,
                   bool bottom_wall,
                   double side_walls);

    /// Add fixed BCE markers, representing a cylindrical annulus container for the computational domain.
    /// Set inner radius to zero to create a cylindrical container.
    /// The cylinder is constructed with its axis along the global Z axis.
    /// The specified dimensions refer to the *interior* of the cylindrical annulus.
    size_t AddCylindricalContainer(double radius_inner,    ///< inner radius
                                   double radius_outer,    ///< outer radius
                                   double height,          ///< height
                                   const ChVector3d& pos,  ///< reference position
                                   bool bottom_wall,       ///< create bottom boundary
                                   bool side_walls,        ///< create side boundaries
                                   bool top_wall           ///< create top boundary
    );

  private:
    virtual ChVector3i Snap2Grid(const ChVector3d& point) override;
    virtual ChVector3d Grid2Point(const ChVector3i& p) override;
};

// ----------------------------------------------------------------------------

/// Predefined SPH particle initial properties callback (depth-based pressure).
class CH_FSI_API DepthPressurePropertiesCallback : public ChFsiProblem::ParticlePropertiesCallback {
  public:
    DepthPressurePropertiesCallback(const ChSystemFsi& sysFSI, double zero_height)
        : ParticlePropertiesCallback(sysFSI), zero_height(zero_height) {
        gz = std::abs(sysFSI.GetGravitationalAcceleration().z());
        c2 = sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed();
    }

    virtual void set(const ChVector3d& pos) override {
        p0 = sysFSI.GetDensity() * gz * (zero_height - pos.z());
        rho0 = sysFSI.GetDensity() + p0 / c2;
        mu0 = sysFSI.GetViscosity();
        v0 = VNULL;
    }

  private:
    double zero_height;
    double gz;
    double c2;
};

/// @} fsi_physics

}  // namespace fsi
}  // namespace chrono

#endif
