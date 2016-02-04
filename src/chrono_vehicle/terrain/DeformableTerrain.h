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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Rigid terrain
//
// =============================================================================

#ifndef DEFORMABLE_TERRAIN_H
#define DEFORMABLE_TERRAIN_H

#include <string>

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChCTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Deformable terrain model.
/// This class implements a terrain with variable heightmap, but differently from
/// RigidTerrain, the vertical coordinates of this terrain mesh can be deformed
/// because of vertical interaction with wheeled vehicles

class CH_VEHICLE_API DeformableTerrain : public ChLoadContainer {
  public:

    /// Construct a default DeformableTerrain.
    /// The user is responsible for calling various Set methods before Initialize.
    DeformableTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
                 );

    ~DeformableTerrain() {}


    /// Set visualization color.
    void SetColor(ChColor color  ///< [in] color of the visualization material
                  );

    /// Set texture properties.
    void SetTexture(const std::string tex_file,  ///< [in] texture filename
                    float tex_scale_x = 1,       ///< [in] texture scale in X
                    float tex_scale_y = 1        ///< [in] texture scale in Y
                    );

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(double height,  ///< [in] terrain height
                    double sizeX,   ///< [in] terrain dimension in the X direction
                    double sizeY,   ///< [in] terrain dimension in the Y direction
                    int divX,       ///< [in] number of divisions in the X direction
                    int divY       ///< [in] number of divisions in the Y direction
                    );

    /// Initialize the terrain system (mesh).
    /// The initial undeformed mesh is provided via a Wavefront .obj file. 
    void Initialize(const std::string& mesh_file  ///< [in] filename of the input mesh (.OBJ file in Wavefront format)
                    );

    /// Initialize the terrain system (height map).
    /// The initial undeformed mesh is provided via the specified BMP file as 
    /// a height map
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (BMP)
                    const std::string& mesh_name,       ///< [in] name of the mesh asset
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax                         ///< [in] maximum height (white level)
                    );

    /// Get the terrain height at the specified (x,y) location.
    //virtual double GetHeight(double x, double y) const override;

    /// Get the terrain normal at the specified (x,y) location.
    //virtual chrono::ChVector<> GetNormal(double x, double y) const override;

    // Updates the forces and the geometry
    virtual void Update(double mytime, bool update_assets = true) override;

  private:
    ChSharedPtr<ChColorAsset> m_color;
    ChSharedPtr<ChTriangleMeshShape> m_trimesh_shape;
    double m_height;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
