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
    enum Type { BOX, MESH, HEIGHT_MAP };

    class CH_VEHICLE_API Patch {
      public:
        /// Set coefficient of friction.
        /// The default value is 0.7
        void SetContactFrictionCoefficient(float friction_coefficient);

        /// Set coefficient of restitution.
        /// The default value is 0.1
        void SetContactRestitutionCoefficient(float restitution_coefficient);

        /// Set contact material properties.
        /// These values are used to calculate contact material coefficients (if the containing
        /// system is so configured and if the SMC contact method is being used).
        /// The default values are: Y = 2e5 and nu = 0.3
        void SetContactMaterialProperties(float young_modulus,  ///< [in] Young's modulus of elasticity
                                          float poisson_ratio   ///< [in] Poisson ratio
        );

        /// Set contact material coefficients.
        /// These values are used directly to compute contact forces (if the containing system
        /// is so configured and if the SMC contact method is being used).
        /// The default values are: kn=2e5, gn=40, kt=2e5, gt=20
        void SetContactMaterialCoefficients(float kn,  ///< [in] normal contact stiffness
                                            float gn,  ///< [in] normal contact damping
                                            float kt,  ///< [in] tangential contact stiffness
                                            float gt   ///< [in] tangential contact damping
        );

        /// Set visualization color.
        void SetColor(const ChColor& color  ///< [in] color of the visualization material
        );

        /// Set texture properties.
        void SetTexture(const std::string& tex_file,  ///< [in] texture filename
                        float tex_scale_x = 1,        ///< [in] texture scale in X
                        float tex_scale_y = 1         ///< [in] texture scale in Y
        );

        /// Export the patch mesh (if any) as a macro in a PovRay include file.
        void ExportMeshPovray(const std::string& out_dir  ///< [in] output directory
        );

        /// Return a handle to the ground body.
        std::shared_ptr<ChBody> GetGroundBody() const;

      private:
        Type m_type;
        std::shared_ptr<ChBody> m_body;
        std::shared_ptr<geometry::ChTriangleMeshConnected> m_trimesh;
        std::string m_mesh_name;
        float m_friction;

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
    std::shared_ptr<Patch> AddPatch(
        const ChCoordsys<>& position,  ///< [in] box location and orientation
        const ChVector<>& size,        ///< [in] box dimensions (x, y, z)
        bool tiled = false,            ///< [in] terrain created from multiple tiled boxes
        double max_tile_size = 1,      ///< [in] maximum tile size
        bool visualization = true      ///< [in] enable/disable construction of visualization assets
    );

    /// Add a terrain patch represented by a triangular mesh.
    /// The mesh is specified through a Wavefront file and is used for both contact and visualization.
    std::shared_ptr<Patch> AddPatch(
        const ChCoordsys<>& position,    ///< [in] patch location and orientation
        const std::string& mesh_file,    ///< [in] filename of the input mesh (OBJ)
        const std::string& mesh_name,    ///< [in] name of the mesh asset
        double sweep_sphere_radius = 0,  ///< [in] radius of sweep sphere
        bool visualization = true        ///< [in] enable/disable construction of visualization assets
    );

    /// Add a terrain patch represented by a height-field map.
    /// The height map is specified through a BMP gray-scale image.
    std::shared_ptr<Patch> AddPatch(
        const ChCoordsys<>& position,       ///< [in] patch location and orientation
        const std::string& heightmap_file,  ///< [in] filename for the height map (BMP)
        const std::string& mesh_name,       ///< [in] name of the mesh asset
        double sizeX,                       ///< [in] terrain dimension in the X direction
        double sizeY,                       ///< [in] terrain dimension in the Y direction
        double hMin,                        ///< [in] minimum height (black level)
        double hMax,                        ///< [in] maximum height (white level)
        bool visualization = true           ///< [in] enable/disable construction of visualization assets
    );

    /// Initialize all defined terrain patches.
    void Initialize();

    /// Get the terrain patches currently added to the rigid terrain system.
    const std::vector<std::shared_ptr<Patch>>& GetPatches() const { return m_patches; }

    /// Get the terrain height at the specified (x,y) location.
    virtual double GetHeight(double x, double y) const override;

    /// Get the terrain height at the specified (x,y) location.
    virtual ChVector<> GetNormal(double x, double y) const override;

    /// Enable use of location-dependent coefficient of friction in terrain-solid contacts.
    /// This assumes that a non-trivial functor (of type ChTerrain::FrictionFunctor) was defined
    /// and registered with the terrain subsystem. Enable this only if simulating a system that
    /// has contacts with the terrain that must be resolved using the underlying Chrono contact
    /// mechanism (e.g., for rigid tires, FEA tires, tracked vehicles). Note that this feature
    /// requires a relatively expensive traversal of all contacts at each simulation step.
    /// By default, this option is disabled.  This function must be called before Initialize.
    void UseLocationDependentFriction(bool val) { m_use_friction_functor = val; }

    /// Get the terrain coefficient of friction at the specified (x,y) location.
    /// For RigidTerrain, this function defers to the user-provided functor object of type
    /// ChTerrain::FrictionFunctor, if one was specified. Otherwise, it returns the constant
    /// value from the appropriate patch, as specified through SetContactFrictionCoefficient.
    /// Note that this function may be used by tire models to appropriately modify the tire
    /// characteristics, but it will have no effect on the interaction of the terrain with
    /// other objects (including tire models that do not explicitly use it).
    /// See UseLocationDependentFriction.
    virtual float GetCoefficientFriction(double x, double y) const override;

    /// Export all patch meshes as macros in PovRay include files.
    void ExportMeshPovray(const std::string& out_dir);

    /// Evaluate terrain height, normal, and coefficient of friction at the specified (x,y) location.
    /// The point on the terrain surface is obtained through ray casting into the terrain contact model.
    /// The return value is 'true' if the ray intersection succeeded and 'false' otherwise (in which case
    /// the output is set to heigh=0, normal=[0,0,1], and friction=0.8).
    bool FindPoint(double x, double y, double& height, ChVector<>& normal, float& friction) const;

  private:
    ChSystem* m_system;
    int m_num_patches;
    std::vector<std::shared_ptr<Patch>> m_patches;
    bool m_use_friction_functor;
    ChContactContainer::AddContactCallback* m_contact_callback;

    std::shared_ptr<Patch> AddPatch(const ChCoordsys<>& position);
    void LoadPatch(const rapidjson::Value& a);
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
