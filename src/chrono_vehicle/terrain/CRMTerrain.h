// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Continuum representation (SPH-based) deformable terrain model.
//
// Reference frame is ISO (X forward, Y left, Z up).
// All units SI.
//
// =============================================================================

#ifndef CRM_SPH_TERRAIN_H
#define CRM_SPH_TERRAIN_H

#include <unordered_set>

#include "chrono/physics/ChSystem.h"
#include "chrono/geometry/ChGeometry.h"
#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChWorldFrame.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Continuum representation (CRM) deformable terrain model using SPH.
class CH_VEHICLE_API CRMTerrain : public ChTerrain {
  public:
    /// Create a CRM terrain object.
    /// No SPH parameters are set.
    CRMTerrain(ChSystem& sys, double spacing);

    /// Enable verbose output during construction of CRMTerrain (default: false).
    void SetVerbose(bool verbose);

    /// Access the underlying FSI system.
    fsi::ChSystemFsi& GetSystemFSI() { return m_sysFSI; }

    /// Add a rigid obstacle.
    /// A rigid body with visualization and collision geometry read from the Wavefront OBJ file is created
    /// at the specified position and with the specified orientation. BCE markers are created with the
    /// separation value specified at construction.
    /// Must be called before Initialize().
    void AddRigidObstacle(const std::string& obj_file,                      ///< Wavefront OBJ file name
                          double scale,                                     ///< scale factor
                          double density,                                   ///< material density
                          const ChContactMaterialData& cmat,                ///< contact material properties
                          const ChFrame<>& pos,                             ///< obstacle position in absolute frame
                          const ChVector3d& interior_point = ChVector3d(0)  ///< point inside obstacle volume
    );

    /// Construct a CRM terrain object using information from the specified files.
    /// The SPH particle and BCE marker locations are assumed to be provided on an integer grid.
    /// Locations in real space are generated using the specified grid separation value and the
    /// terrain patch translated and rotated aboutn the vertical.
    void Construct(const std::string& sph_file,            ///< filename with SPH grid particle positions
                   const std::string& bce_file,            ///< filename with BCE grid marker positions
                   const ChVector3d& pos = ChVector3d(0),  ///< patch center
                   double yaw_angle = 0                    ///< patch yaw rotation
    );

    /// Construct a rectangular patch granular CRM terrain object of given dimensions.
    void Construct(double length,                          ///< patch length (X direction)
                   double width,                           ///< patch width (Y direction)
                   double depth,                           ///< patch depth (Z direction)
                   int bce_layers = 3,                     ///< number of BCE layers
                   const ChVector3d& pos = ChVector3d(0),  ///< patch center
                   double yaw_angle = 0,                   ///< patch yaw rotation
                   bool side_walls = true                  ///< create side boundaries
    );

    /// Construct a CRM terrain object from a given heightmap.
    /// The image file is read with STB, using the number of channels defined in the input file and reading
    /// the image as 16-bit (8-bit images are automatically converted). Supported image formats: JPEG, PNG,
    /// BMP, GIF, PSD, PIC, PNM.
    /// Create the SPH particle grid locations for a terrain patch of specified X and Y dimensions with optional
    /// translation and rotation. The height at each grid point is obtained through bilinear interpolation from
    /// the gray values in the provided heightmap image (with pure black corresponding to the lower height range
    /// and pure white to the upper height range). SPH particle grid locations are generated to cover the specified
    /// depth under each grid point. BCE marker layers are created below the bottom-most layer of SPH particles
    /// and (optionally) on the sides of the terrain patch.
    void Construct(const std::string& heightmap_file,      ///< filename for the heightmap image
                   double length,                          ///< patch length (X direction)
                   double width,                           ///< patch width (Y direction)
                   const ChVector2d& height_range,         ///< height range (black to white level)
                   double depth,                           ///< soil depth
                   int bce_layers = 3,                     ///< number of BCE layers
                   const ChVector3d& pos = ChVector3d(0),  ///< patch center
                   double yaw_angle = 0,                   ///< patch yaw rotation
                   bool side_walls = true                  ///< create side boundaries
    );

    /// Initialize the terrain.
    /// After this call, no additional solid bodies should be added to the FSI problem.
    void Initialize();

    /// Get the ground body.
    std::shared_ptr<ChBody> GetGroundBody() const { return m_ground; }

    /// Get the AABB of terrain SPH particles.
    const ChAABB& GetBoundingBox() const { return m_aabb; }

    /// Save the set of SPH and BCE grid locations to the files in the specified output directory.
    void SaveMarkers(const std::string& out_dir) const;

    //// TODO - anything needed here?
    virtual void Synchronize(double time) override {}
    virtual double GetHeight(const ChVector3d& loc) const override { return 0.0; }
    virtual chrono::ChVector3d GetNormal(const ChVector3d& loc) const override { return ChWorldFrame::Vertical(); }
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override { return 0.0f; }

  private:
    /// Hash function for a 3D integer grid coordinate.
    struct CoordHash {
        std::size_t operator()(const ChVector3i& p) const {
            size_t h1 = std::hash<int>()(p.x());
            size_t h2 = std::hash<int>()(p.y());
            size_t h3 = std::hash<int>()(p.z());
            return (h1 ^ (h2 << 1)) ^ h3;
        }
    };

    typedef std::unordered_set<ChVector3i, CoordHash> Points;

    /// Specification of a rigid obstacle.
    struct RigidObstacle {
        std::shared_ptr<ChBody> body;                      ///< associated body
        double density;                                    ///< material density
        ChContactMaterialData cmat;                        ///< contact material properties
        std::shared_ptr<ChTriangleMeshConnected> trimesh;  ///< geometry
        std::vector<ChVector3d> point_cloud;               ///< point cloud for BCE markers
        ChVector3d point;                                  ///< location of an interior point
        Points bce;                                        ///< BCE marker grid locations
        ChVector3d oobb_center;                            ///< center of bounding box
        ChVector3d oobb_dims;                              ///< dimensions of bounding box
    };

    /// Complete construction of the CRMTerrain.
    /// - Prune SPH particle locations that overlap with obstacles
    /// - Convert grid locations to real coordinates
    /// - Create the SPH particles
    /// - Created the fixed BCE markers
    /// - Create the obstacle BCE markers
    void CompleteConstruct();

    /// Prune SPH markers that are inside the obstacle volume.
    /// Voxelize the obstacle mesh (at the scaling resolution) and identify grid nodes inside the boundary
    /// defined by the obstacle BCEs. Note that this assumes the BCE markers form a watertight boundary.
    void ProcessObstacleMesh(RigidObstacle& o);

    fsi::ChSystemFsi m_sysFSI;               ///< underlying Chrono FSI system
    double m_spacing;                        ///< (initial) particle and marker spacing
    ChSystem& m_sys;                         ///< associated Chrono MBS system
    std::shared_ptr<ChBody> m_ground;        ///< associated body
    bool m_initialized;                      ///< set to 'true' once terrain is initialized
    Points m_sph;                            ///< SPH particle grid locations
    Points m_bce;                            ///< BCE marker grid locations
    ChVector3d m_offset;                     ///< patch offset
    double m_angle;                          ///< patch rotation about vertical
    ChAABB m_aabb;                           ///< particle AABB
    std::vector<RigidObstacle> m_obstacles;  ///< list of rigid obstacles
    bool m_verbose;                          ///< if true, write information to standard output
};

/// @} vehicle_terrain

}  // namespace vehicle
}  // namespace chrono

#endif
