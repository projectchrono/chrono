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
// Rainer Gericke, Radu Serban
// =============================================================================

#ifndef CH_SHAPE_BUILDER_H
#define CH_SHAPE_BUILDER_H

#include <iostream>
#include <string>

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono_vsg/ChApiVSG.h"

#include "chrono/assets/ChVisualMaterial.h"

#include "chrono/geometry/ChLineArc.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLineSegment.h"
#include "chrono/geometry/ChSurface.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {
namespace vsg3d {

// -----------------------------------------------------------------------------

class CH_VSG_API ShapeBuilder : public vsg::Inherit<vsg::Object, ShapeBuilder> {
  public:
    enum class ShapeType { BOX, DIE, SPHERE, CYLINDER, CAPSULE, CONE };

    ShapeBuilder(vsg::ref_ptr<vsg::Options> options, int num_divs = 24);

    vsg::ref_ptr<vsg::Group> CreatePbrShape(ShapeType shape_type,
                                            std::shared_ptr<ChVisualMaterial> material,
                                            vsg::ref_ptr<vsg::MatrixTransform> transform,
                                            bool wireframe);

    vsg::ref_ptr<vsg::Group> CreatePbrSurfaceShape(std::shared_ptr<ChSurface> geometry,
                                                   std::shared_ptr<ChVisualMaterial> material,
                                                   vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                   int resolution_u,
                                                   int resolution_v,
                                                   bool wireframe);

    vsg::ref_ptr<vsg::Group> CreateTrimeshPbrMatShape(std::shared_ptr<ChTriangleMeshConnected> mesh,
                                                      vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      const std::vector<ChVisualMaterialSharedPtr>& materials,
                                                      bool wireframe);

    /// Convert the specified mesh into a triangle soup with vertex colors.
    /// Vertex normals are calculated from each face normal, resulting in sharp edges.
    vsg::ref_ptr<vsg::Group> CreateTrimeshColShape(std::shared_ptr<ChTriangleMeshConnected> mesh,
                                                   vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                   const ChColor& default_color,
                                                   bool wireframe);

    /// Convert the specified mesh into a triangle mesh with vertex colors.
    /// Vertex normals are calculated by averaging the normals of incident faces, resulting in smoothed edges.
    vsg::ref_ptr<vsg::Group> CreateTrimeshColAvgShape(std::shared_ptr<ChTriangleMeshConnected> mesh,
                                                      vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      const ChColor& default_color,
                                                      bool wireframe);

    /// Create a symbol to represent a reference frame.
    /// These are 3 mututally orthogonal line segments, colored red, green, and blue.
    /// The color factor controls simultaneous darkening of the 3 colors (a value of 1 indicates pure colors).
    vsg::ref_ptr<vsg::Group> createFrameSymbol(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                               float color_factor,
                                               float line_width,
                                               bool skipZbuffer = false);

    vsg::ref_ptr<vsg::Group> CreateLineShape(std::shared_ptr<ChLine> geometry,
                                             std::shared_ptr<ChVisualMaterial> material,
                                             vsg::ref_ptr<vsg::MatrixTransform> transform,
                                             unsigned int num_points);

    vsg::ref_ptr<vsg::Group> CreatePathShape(std::shared_ptr<ChLinePath> geometry,
                                             std::shared_ptr<ChVisualMaterial> material,
                                             vsg::ref_ptr<vsg::MatrixTransform> transform,
                                             unsigned int num_points);

    vsg::ref_ptr<vsg::Group> CreateUnitSegment(std::shared_ptr<ChVisualMaterial> material,
                                               vsg::ref_ptr<vsg::MatrixTransform> transform,
                                               float line_width,
                                               bool skipZbuffer = false);

    vsg::ref_ptr<vsg::Group> CreateSpringShape(std::shared_ptr<ChVisualMaterial> material,
                                               vsg::ref_ptr<vsg::MatrixTransform> transform,
                                               size_t num_points,
                                               double turns,
                                               float line_width);

    vsg::ref_ptr<vsg::Group> CreateGrid(double ustep, double vstep, int nu, int nv, ChCoordsys<> pos, ChColor col);

    void assignCompileTraversal(vsg::ref_ptr<vsg::CompileTraversal> ct);

  private:
    struct ShapeData {
        vsg::ref_ptr<vsg::vec3Array> vertices;
        vsg::ref_ptr<vsg::vec3Array> normals;
        vsg::ref_ptr<vsg::vec2Array> texcoords;
        vsg::ref_ptr<vsg::ushortArray> indices;
    };

    struct BoxShapeData : public ShapeData {
        BoxShapeData();
    };

    struct DieShapeData : public ShapeData {
        DieShapeData();
    };

    struct SphereShapeData : public ShapeData {
        SphereShapeData(int num_divs);
    };

    struct CylinderShapeData : public ShapeData {
        CylinderShapeData(int num_divs);
    };

    struct ConeShapeData : public ShapeData {
        ConeShapeData(int num_divs);
    };

    struct CapsuleShapeData : public ShapeData {
        CapsuleShapeData(int num_divs);
    };

    vsg::ref_ptr<vsg::Group> CreatePbrShape(vsg::ref_ptr<vsg::vec3Array>& vertices,
                                            vsg::ref_ptr<vsg::vec3Array>& normals,
                                            vsg::ref_ptr<vsg::vec2Array>& texcoords,
                                            vsg::ref_ptr<vsg::ushortArray>& indices,
                                            std::shared_ptr<ChVisualMaterial> material,
                                            vsg::ref_ptr<vsg::MatrixTransform> transform,
                                            bool wireframe);

    vsg::ref_ptr<vsg::Options> m_options;
    vsg::ref_ptr<vsg::CompileTraversal> m_compileTraversal;

    std::unique_ptr<BoxShapeData> m_box_data;
    std::unique_ptr<DieShapeData> m_die_data;
    std::unique_ptr<SphereShapeData> m_sphere_data;
    std::unique_ptr<CylinderShapeData> m_cylinder_data;
    std::unique_ptr<ConeShapeData> m_cone_data;
    std::unique_ptr<CapsuleShapeData> m_capsule_data;
};

}  // namespace vsg3d
}  // namespace chrono
#endif
