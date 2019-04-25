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
// Terrain defined by an OpenCRG file (http://opencrg.org)
//
// OpenCRG® is managed by
//	VIRES Simulationstechnologie GmbH
//	Grassinger Strasse 8
//	83043 Bad Aibling
//	Germany
//	p: +49.8061.939093-0
//	f: +49.8061.939093-13
//	e: opencrg@opencrg.org
//
// =============================================================================

#ifndef CRGTERRAIN_H
#define CRGTERRAIN_H

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
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

    /// Get the terrain height at the specified (x,y) location.
    /// Returns the constant value passed at construction.
    virtual double GetHeight(double x, double y) const override;

    /// Get the terrain normal at the specified (x,y) location.
    /// Returns a constant unit vector along the Z axis.
    virtual ChVector<> GetNormal(double x, double y) const override;

    /// Get the terrain coefficient of friction at the specified (x,y) location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For CRGTerrain, this function defers to the user-provided functor object
    /// of type ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value specified at construction.
    virtual float GetCoefficientFriction(double x, double y) const override;

    /// Get the vehicle path as ChBezierCurve.
    /// This function returns a path along the road's midline.
    std::shared_ptr<ChBezierCurve> GetPath();

    /// Get the road mesh.
    std::shared_ptr<geometry::ChTriangleMeshConnected> GetMesh() const { return m_mesh; }

    /// Is the road a round course (closed loop)?
    bool IsPathClosed() { return m_isClosed; }

    /// Get length of the road.
    double GetLength() { return m_uend - m_ubeg; }

    /// Export road mesh to Wavefront file.
    void ExportMeshWavefront(const std::string& out_dir);

    /// Export the patch mesh as a macro in a PovRay include file.
    void ExportMeshPovray(const std::string& out_dir);

  private:
    /// Build the graphical representation.
    void SetupLineGraphics();
    void SetupMeshGraphics();
    void GenerateMesh();

    std::shared_ptr<ChBody> m_ground;  ///< ground body
    bool m_use_vis_mesh;               ///< mesh or boundary visual asset?
    float m_friction;                  ///< contact coefficient of friction

    std::shared_ptr<geometry::ChTriangleMeshConnected> m_mesh;  ///< mesh for visualization/export

    bool m_isClosed;  ///< closed road profile?

    int m_dataSetId;
    int m_cpId;

    double m_uinc, m_ubeg, m_uend;  // increment, begin , end of longitudinal road coordinates
    double m_vinc, m_vbeg, m_vend;  // increment, begin , end of lateral road coordinates

    std::vector<double> m_v;  // vector with distinct v values, if m_vinc <= 0.01 m

    static const std::string m_mesh_name;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
