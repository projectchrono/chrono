// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
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
// Terrain defined by an OpenCRG file (http://opencrg.org)
//
// OpenCRG® (up to v1.1.2) was managed by
//	VIRES Simulationstechnologie GmbH
//
// OpenCRG® (>= v1.2) is now managed by
// ASAM e.V.
// https://www.asam.net/standards/detail/opencrg/
//
// v1.1.2 is still available. Both versions work with chrono.
// =============================================================================
//
// Limits:	Options and modifiers are ignored
//
// Roads start at {0,0,0}, first driving direction is {1,0,0}
//
//==============================================================================

#ifndef CRGTERRAIN_H
#define CRGTERRAIN_H

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/core/ChBezierCurve.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Concrete class for a (rigid) road loaded from an OpenCRG file.
/// This type of terrain can be used in conjunction with tire models that perform their own collision detection
/// (e.g. ChPacejkaTire, ChFiala, and ChLugreTire).
class CH_VEHICLE_API CRGTerrain : public ChTerrain {
  public:
    /// Construct a default CRGTerrain.
    /// The user should call optional Set methods and then Initialize.
    CRGTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
    );

    /// Enable verbose messages from CRG (default : false).
    void EnableVerbose(bool val);

    /// Set the road visualization mode (mesh or boundary lines).
    /// Default: mesh.
    void UseMeshVisualization(bool val) { m_use_vis_mesh = val; }

    /// Set coefficient of friction.
    /// The default value is 0.8
    void SetContactFrictionCoefficient(float friction_coefficient) { m_friction = friction_coefficient; }

    /// Initialize the CRGTerrain from the specified OpenCRG file.
    void Initialize(const std::string& crg_file  ///< [in] OpenCRG road specification file
    );

    ~CRGTerrain();

    /// Get the terrain height below the specified location.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    virtual ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For CRGTerrain, this function defers to the user-provided functor object
    /// of type ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified at construction.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

    /// Get the road center line as a Bezier curve.
    std::shared_ptr<ChBezierCurve> GetRoadCenterLine();

    /// Get the road left boundary as a Bezier curve.
    std::shared_ptr<ChBezierCurve> GetRoadBoundaryLeft() const { return m_road_left; }

    /// Get the road right boundary as a Bezier curve.
    std::shared_ptr<ChBezierCurve> GetRoadBoundaryRight() const { return m_road_right; }

    /// Get the road mesh.
    std::shared_ptr<geometry::ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Is the road a round course (closed loop)?
    bool IsPathClosed() { return m_isClosed; }

    /// Get length of the road.
    double GetLength() { return m_uend - m_ubeg; }

    /// Get width of the road.
    double GetWidth() { return m_vend - m_vbeg; }

    /// Get start heading (in radians), can be different from zero for georeferenced roads.
    double GetStartHeading();

    /// Get the start position (location and orientation).
    /// This is the (x,y,z) road location at CRG parameters u=v=0.
    ChCoordsys<> GetStartPosition();

    /// Get ground body (carries visualization assets).
    std::shared_ptr<ChBody> GetGround() const { return m_ground; }

    /// Generate roadside posts left and right (optional)
    void SetRoadsidePostDistance(double dist) { m_post_distance = ChClamp(dist, 0.0, 100.0); };

    /// Use a texture file, if mesh representation is desired (optional, should not be used with Irrlicht)
    void SetRoadTextureFile(std::string texFile);

    /// Export road mesh to Wavefront file.
    void ExportMeshWavefront(const std::string& out_dir);

    /// Export the patch mesh as a macro in a POV-Ray include file.
    void ExportMeshPovray(const std::string& out_dir);

    /// Export the road boundary curves as a macro in a POV-Ray include file.
    /// This function does nothing if using mesh visualization for the road.
    void ExportCurvesPovray(const std::string& out_dir);

  private:
    /// Build the graphical representation.
    void SetupLineGraphics();
    void SetupMeshGraphics();
    void GenerateMesh();
    void GenerateCurves();
    void SetRoadsidePosts();

    double m_post_distance; // 0 means no posts
    std::string m_texture_filename;
    bool m_use_texture; // if set, use a textured mesh

    std::shared_ptr<ChBody> m_ground;  ///< ground body
    bool m_use_vis_mesh;               ///< mesh or boundary visual asset?
    float m_friction;                  ///< contact coefficient of friction

    std::string m_mesh_name;
    std::string m_curve_left_name;
    std::string m_curve_right_name;

    std::shared_ptr<geometry::ChTriangleMeshConnected> m_mesh;  ///< mesh for visualization/export
    std::shared_ptr<ChBezierCurve> m_road_left;                 ///< curve for left road boundary
    std::shared_ptr<ChBezierCurve> m_road_right;                ///< curve for right road boundary

    bool m_isClosed;  ///< closed road profile?

    int m_dataSetId;
    int m_cpId;

    double m_uinc, m_ubeg, m_uend;  // increment, begin , end of longitudinal road coordinates
    double m_vinc, m_vbeg, m_vend;  // increment, begin , end of lateral road coordinates

    std::vector<double> m_v;  // vector with distinct v values, if m_vinc <= 0.01 m
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
