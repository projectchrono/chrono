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
// Authors: Radu Serban
// =============================================================================
//
// Rigid terrain
//
// =============================================================================

#ifndef RIGID_TERRAIN_H
#define RIGID_TERRAIN_H

#include <string>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Rigid terrain model.
/// This class implements a terrain modeled as a rigid shape which can interact
/// through contact and friction with any other bodies whose contact flag is
/// enabled. In particular, this type of terrain can be used in conjunction with
/// a ChRigidTire.
class CH_VEHICLE_API RigidTerrain : public ChTerrain {
  public:
    /// Patch type.
    enum class PatchType {
        BOX,        ///< rectangular box
        MESH,       ///< triangular mesh (from a Wavefront OBJ file)
        HEIGHT_MAP  ///< triangular mesh (generated from a gray-scale BMP height-map)
    };

    /// Definition of a patch in a rigid terrain model.
    class CH_VEHICLE_API Patch {
      public:
        virtual ~Patch() {}

        /// Set visualization color.
        void SetColor(const ChColor& color  ///< [in] color of the visualization material
        );

        /// Set texture properties.
        void SetTexture(const std::string& tex_file,  ///< [in] texture filename
                        float tex_scale_x = 1,        ///< [in] texture scale in X
                        float tex_scale_y = 1         ///< [in] texture scale in Y
        );

        /// Return a handle to the ground body.
        std::shared_ptr<ChBody> GetGroundBody() const { return m_body; }

      protected:
        virtual bool FindPoint(const ChVector<>& loc, double& height, ChVector<>& normal) const = 0;
        virtual void ExportMeshPovray(const std::string& out_dir, bool smoothed = false) {}
        virtual void ExportMeshWavefront(const std::string& out_dir) {}

        PatchType m_type;                ///< type of this patch
        std::shared_ptr<ChBody> m_body;  ///< associated body
        float m_friction;                ///< coefficient of friction
        double m_radius;                 ///< bounding sphere radius

        friend class RigidTerrain;
    };

    /// Construct a default RigidTerrain.
    /// The user is responsible for calling various Set methods before Initialize.
    RigidTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
    );

    /// Construct a RigidTerrain from a JSON specification file.
    RigidTerrain(ChSystem* system,            ///< [in] pointer to the containing multibody system
                 const std::string& filename  ///< [in] name of the JSON specification file
    );

    ~RigidTerrain();

    /// Add a terrain patch represented by a rigid box.
    /// If tiled = true, multiple side-by-side boxes are used.
    /// The "driving" surface is assumed to be the +z face of the specified box domain.
    std::shared_ptr<Patch> AddPatch(
        std::shared_ptr<ChMaterialSurface> material,  ///< [in] contact material
        const ChVector<>& location,                   ///< [in] center of top surface
        const ChVector<>& normal,                     ///< [in] normal to top surface
        double length,                                ///< [in] patch length
        double width,                                 ///< [in] patch width
        double thickness = 1,                         ///< [in] box thickness
        bool tiled = false,                           ///< [in] terrain created from multiple tiled boxes
        double max_tile_size = 1,                     ///< [in] maximum tile size
        bool visualization = true                     ///< [in] enable/disable construction of visualization assets
    );

    /// Add a terrain patch represented by a triangular mesh.
    /// The mesh is specified through a Wavefront file and is used for both contact and visualization.
    std::shared_ptr<Patch> AddPatch(
        std::shared_ptr<ChMaterialSurface> material,  ///< [in] contact material
        const ChCoordsys<>& position,                 ///< [in] patch location and orientation
        const std::string& mesh_file,                 ///< [in] filename of the input mesh (OBJ)
        double sweep_sphere_radius = 0,               ///< [in] radius of sweep sphere
        bool visualization = true                     ///< [in] enable/disable construction of visualization assets
    );

    /// Add a terrain patch represented by a height-field map.
    /// The height map is specified through a BMP gray-scale image.
    std::shared_ptr<Patch> AddPatch(
        std::shared_ptr<ChMaterialSurface> material,  ///< [in] contact material
        const ChCoordsys<>& position,                 ///< [in] patch location and orientation
        const std::string& heightmap_file,            ///< [in] filename for the height map (BMP)
        double length,                                ///< [in] patch length
        double width,                                 ///< [in] patch width
        double hMin,                                  ///< [in] minimum height (black level)
        double hMax,                                  ///< [in] maximum height (white level)
        double sweep_sphere_radius = 0,               ///< [in] radius of sweep sphere
        bool visualization = true                     ///< [in] enable/disable construction of visualization assets
    );

    /// Initialize all defined terrain patches.
    void Initialize();

    /// Get the terrain patches currently added to the rigid terrain system.
    const std::vector<std::shared_ptr<Patch>>& GetPatches() const { return m_patches; }

    /// Get the terrain height below the specified location.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    virtual ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Enable use of location-dependent coefficient of friction in terrain-solid contacts.
    /// This assumes that a non-trivial functor (of type ChTerrain::FrictionFunctor) was defined
    /// and registered with the terrain subsystem. Enable this only if simulating a system that
    /// has contacts with the terrain that must be resolved using the underlying Chrono contact
    /// mechanism (e.g., for rigid tires, FEA tires, tracked vehicles). Note that this feature
    /// requires a relatively expensive traversal of all contacts at each simulation step.
    /// By default, this option is disabled.  This function must be called before Initialize.
    void UseLocationDependentFriction(bool val) { m_use_friction_functor = val; }

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// For RigidTerrain, this function defers to the user-provided functor object of type
    /// ChTerrain::FrictionFunctor, if one was specified. Otherwise, it returns the constant
    /// value from the appropriate patch, as specified through SetContactFrictionCoefficient.
    /// Note that this function may be used by tire models to appropriately modify the tire
    /// characteristics, but it will have no effect on the interaction of the terrain with
    /// other objects (including tire models that do not explicitly use it).
    /// See UseLocationDependentFriction.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

    /// Export all patch meshes as macros in PovRay include files.
    void ExportMeshPovray(const std::string& out_dir, bool smoothed = false);

    /// Export all patch meshes as Wavefront files.
    void ExportMeshWavefront(const std::string& out_dir);

    /// Find the terrain height, normal, and coefficient of friction at the point below the specified location.
    /// The point on the terrain surface is obtained through ray casting into the terrain contact model.
    /// The return value is 'true' if the ray intersection succeeded and 'false' otherwise (in which case
    /// the output is set to heigh=0, normal=[0,0,1], and friction=0.8).
    bool FindPoint(const ChVector<> loc, double& height, ChVector<>& normal, float& friction) const;

    /// Set common collision family for patches.
    /// Used only if defining two or more patches. Default: 14.
    void SetCollisionFamily(int family) { m_collision_family = family; }

  private:
    /// Patch represented as a box domain.
    struct CH_VEHICLE_API BoxPatch : public Patch {
        ChVector<> m_location;  ///< center of top surface
        ChVector<> m_normal;    ///< outward normal of the top surface
        double m_hlength;       ///< patch half-length
        double m_hwidth;        ///< patch half-width
        virtual bool FindPoint(const ChVector<>& loc, double& height, ChVector<>& normal) const override;
    };

    /// Patch represented as a mesh.
    struct CH_VEHICLE_API MeshPatch : public Patch {
        std::shared_ptr<geometry::ChTriangleMeshConnected> m_trimesh;  ///< associated mesh
        std::string m_mesh_name;                                       ///< name of associated mesh
        virtual bool FindPoint(const ChVector<>& loc, double& height, ChVector<>& normal) const override;
        virtual void ExportMeshPovray(const std::string& out_dir, bool smoothed = false) override;
        virtual void ExportMeshWavefront(const std::string& out_dir) override;
    };

    ChSystem* m_system;
    int m_num_patches;
    std::vector<std::shared_ptr<Patch>> m_patches;
    bool m_use_friction_functor;
    std::shared_ptr<ChContactContainer::AddContactCallback> m_contact_callback;

    void AddPatch(std::shared_ptr<Patch> patch,
                  const ChCoordsys<>& position,
                  std::shared_ptr<ChMaterialSurface> material);
    void LoadPatch(const rapidjson::Value& a);

    int m_collision_family;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
