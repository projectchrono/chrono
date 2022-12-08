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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Simple trench or mound shaped obstacle as needed for NRMM
//  The obstacle is defined by three parameters
//  - aa:        approach angle 180 deg = flat, aa < 180 deg = mound, aa > 180 deg = trench
//  - length:    obstacle length for mound, base length for trench
//  - obsheight: allways >= 0, obstacle height for mound, obstacle depth for trench
//
// =============================================================================

#ifndef OBSMOD_TERRAIN_H
#define OBSMOD_TERRAIN_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Concrete class for a flat horizontal terrain.
/// This class implements a terrain modeled as an infinite horizontal plane at a specified height.
/// This type of terrain can be used in conjunction with tire models that perform their own collision detection
/// (e.g., ChTMeasy ChPac89, ChPac02, ChFiala).
class CH_VEHICLE_API ObsModTerrain : public ChTerrain {
  public:
    enum class VisualisationType { NONE, MESH };
    ObsModTerrain(ChSystem* system,       ///< [in] pointer to the containing multibody system
                  double height = 0.0,    ///< [m] global terrain height
                  float friction = 0.8f,  ///< [] terrain coefficient of friction
                  double aa = 180.0,      ///< [deg] approach angle
                  double length = 10.0,   ///< [m] obstacle length for mound, base length for trench
                  double obsheight = 0.0  ///< [m] allways >= 0, obstacle height for mound, obstacle depth for trench
    );

    ~ObsModTerrain() {}

    /// Get the terrain height below the specified location.
    /// Returns the constant value passed at construction.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    /// Returns a constant unit vector along the vertical axis.
    virtual ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For ObsModTerrain, this function defers to the user-provided functor object
    /// of type ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified at construction.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

    void Initialize(ObsModTerrain::VisualisationType vType = ObsModTerrain::VisualisationType::MESH);

    double GetXObstacleEnd() { return m_xmax; }

    double GetEffLength();

    /// Enable creation of a collision mesh and enable collision (default: no collision mesh).
    /// Optionally (length > 0), create a flat lane of given length positioned before the uneven portion.
    /// The specified radius (default 0) is used as a "mesh thickness" to improve robustness of the collision detection.
    /// Note that this function must be called before Initialize().
    void EnableCollisionMesh(std::shared_ptr<ChMaterialSurface> material,
                             double length = 0,
                             double sweep_sphere_radius = 0);

    inline double BilinearInterpolation(double q11,
                                        double q12,
                                        double q21,
                                        double q22,
                                        double x1,
                                        double x2,
                                        double y1,
                                        double y2,
                                        double x,
                                        double y) const {
        double x2x1, y2y1, x2x, y2y, yy1, xx1;
        x2x1 = x2 - x1;
        y2y1 = y2 - y1;
        x2x = x2 - x;
        y2y = y2 - y;
        yy1 = y - y1;
        xx1 = x - x1;
        return 1.0 / (x2x1 * y2y1) * (q11 * x2x * y2y + q21 * xx1 * y2y + q12 * x2x * yy1 + q22 * xx1 * yy1);
    }

  private:
    double m_height;                   ///< terrain height
    float m_friction;                  ///< contact coefficient of friction
    double m_width;                    ///< [m] obstacle width
    double m_aa;                       ///< approach angle 180° flat, >180° trench, <180° mound, internally in [rad]
    double m_obslength;                ///< [m] obstacle length for mound, base length for trench
    double m_obsheight;                ///< [m] allways >= 0, obstacle height for mound, obstacle depth for trench
    size_t m_nx;                       ///< number of obstacle coordinates in x direction
    size_t m_ny;                       ///< numer of obstacle coordinates in y direction
    double m_xmin;                     ///< x coordinate, where the obstacle begins
    double m_xmax;                     ///< x coordinate, where the obstacle ends
    double m_ymin;                     ///< y coordinate, where the obstacle begins
    double m_ymax;                     ///< y coordinate, where the obstacle ends
    std::vector<double> m_x;           ///< hold the unequally spaced x values
    std::vector<double> m_y;           ///< hold the unequally spaced y values
    ChMatrixDynamic<> m_Q;             ///< matrix of uneven height values
    std::shared_ptr<ChBody> m_ground;  ///< ground body
    std::shared_ptr<geometry::ChTriangleMeshConnected> m_mesh;  ///< mesh for visualization/export

    std::shared_ptr<ChMaterialSurface> m_material;
    bool m_collision_mesh;
    double m_start_length;
    double m_sweep_sphere_radius;

    void GenerateMesh();
    void SetupCollision();
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
