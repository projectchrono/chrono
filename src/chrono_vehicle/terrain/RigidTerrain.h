// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

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
    enum Type { FLAT, MESH, HEIGHT_MAP };

    /// Construct a default RigidTerrain.
    /// The user is responsible for calling various Set methods before Initialize.
    RigidTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
                 );

    /// Construct a RigidTerrain from a JSON specification file.
    RigidTerrain(ChSystem* system,            ///< [in] pointer to the containing multibody system
                 const std::string& filename  ///< [in] name of the JSON specification file
                 );

    ~RigidTerrain() {}

    /// Set coefficient of friction.
    /// The default value is 0.7
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Set coefficient of restitution.
    /// The default value is 0.1
    void SetContactRestitutionCoefficient(float restitution_coefficient) { m_restitution = restitution_coefficient; }

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

    /// Get coefficient of friction for contact material.
    float GetCoefficientFriction() const { return m_friction; }
    /// Get coefficient of restitution for contact material.
    float GetCoefficientRestitution() const { return m_restitution; }
    /// Get Young's modulus of elasticity for contact material.
    float GetYoungModulus() const { return m_young_modulus; }
    /// Get Poisson ratio for contact material.
    float GetPoissonRatio() const { return m_poisson_ratio; }
    /// Get normal stiffness coefficient for contact material.
    float GetKn() const { return m_kn; }
    /// Get tangential stiffness coefficient for contact material.
    float GetKt() const { return m_kt; }
    /// Get normal viscous damping coefficient for contact material.
    float GetGn() const { return m_gn; }
    /// Get tangential viscous damping coefficient for contact material.
    float GetGt() const { return m_gt; }

    /// Enable/disable terrain visualization (default: true).
    /// Note that this is ignored when constructing a terrain object from a JSON specification file.
    void EnableVisualization(bool val) { m_vis_enabled = val; }
    bool IsVisualizationEnabled() const { return m_vis_enabled; }

    /// Set visualization color.
    void SetColor(ChColor color  ///< [in] color of the visualization material
                  );

    /// Set texture properties.
    void SetTexture(const std::string tex_file,  ///< [in] texture filename
                    float tex_scale_x = 1,       ///< [in] texture scale in X
                    float tex_scale_y = 1        ///< [in] texture scale in Y
                    );

    /// Return a handle to the ground body.
    std::shared_ptr<ChBody> GetGroundBody() { return m_ground; }

    /// Initialize the terrain system (flat).
    /// This version uses a rigid box of specified dimensions and with specified
    /// material properties. If tiled = true, multiple side-by-side boxes are used.
    void Initialize(double height,            ///< [in] terrain height
                    double sizeX,             ///< [in] terrain dimension in the X direction
                    double sizeY,             ///< [in] terrain dimension in the Y direction
                    bool tiled = false,       ///< [in] terrain created from multiple tiled boxes
                    double max_tile_size = 1  ///< [in] maximum tile size
                    );

    /// Initialize the terrain system (mesh).
    /// this version uses the specified mesh, for both visualization and contact.
    void Initialize(const std::string& mesh_file,   ///< [in] filename of the input mesh (OBJ)
                    const std::string& mesh_name,   ///< [in] name of the mesh asset
                    double sweep_sphere_radius = 0  ///< [in] radius of sweep sphere
                    );

    /// Initialize the terrain system (height map).
    /// This version uses the specified BMP file as a height map to create a mesh for
    /// both contact and visualization.
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (BMP)
                    const std::string& mesh_name,       ///< [in] name of the mesh asset
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax                         ///< [in] maximum height (white level)
                    );

    /// Export the terrain mesh (if any) as a macro in a PovRay include file.
    void ExportMeshPovray(const std::string& out_dir  ///< [in] output directory
                          );

    /// Get the terrain height at the specified (x,y) location.
    virtual double GetHeight(double x, double y) const override;

    /// Get the terrain normal at the specified (x,y) location.
    virtual chrono::ChVector<> GetNormal(double x, double y) const override;

  private:
    Type m_type;
    bool m_vis_enabled;
    std::shared_ptr<ChBody> m_ground;
    std::shared_ptr<ChColorAsset> m_color;
    geometry::ChTriangleMeshConnected m_trimesh;
    std::string m_mesh_name;
    double m_height;

    float m_friction;       ///< contact coefficient of friction
    float m_restitution;    ///< contact coefficient of restitution
    float m_young_modulus;  ///< contact material Young modulus
    float m_poisson_ratio;  ///< contact material Poisson ratio
    float m_kn;             ///< normal contact stiffness
    float m_gn;             ///< normal contact damping
    float m_kt;             ///< tangential contact stiffness
    float m_gt;             ///< tangential contact damping

    void ApplyContactMaterial();
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
