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
// Rainer Gericke
// =============================================================================

#ifndef CH_SHAPE_BUILDER_H
#define CH_SHAPE_BUILDER_H

#include <iostream>
#include <string>
#include "chrono_vsg/ChApiVSG.h"

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/assets/ChSurfaceShape.h"
#include "chrono/assets/ChPointPointShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

namespace chrono {
namespace vsg3d {

class CH_VSG_API ShapeBuilder : public vsg::Inherit<vsg::Object, ShapeBuilder> {
  public:
    typedef enum {
        BOX_SHAPE,
        DIE_SHAPE,
        SPHERE_SHAPE,
        CYLINDER_SHAPE,
        CAPSULE_SHAPE,
        CONE_SHAPE,
        SURFACE_SHAPE
    } BasicShape;

    vsg::ref_ptr<vsg::Options> m_options;
    vsg::ref_ptr<vsg::SharedObjects> m_sharedObjects;
    float m_maxAnisotropy = 0.0f;

    vsg::ref_ptr<vsg::Group> createPhongShape(BasicShape theShape,
                                              std::shared_ptr<ChVisualMaterial> material,
                                              vsg::ref_ptr<vsg::MatrixTransform> transform,
                                              bool wireframe,
                                              std::shared_ptr<ChSurfaceShape> surface = nullptr);

    vsg::ref_ptr<vsg::Group> createPbrShape(BasicShape theShape,
                                            std::shared_ptr<ChVisualMaterial> material,
                                            vsg::ref_ptr<vsg::MatrixTransform> transform,
                                            bool wireframe,
                                            std::shared_ptr<ChSurfaceShape> surface = nullptr);

    vsg::ref_ptr<vsg::Group> createTrimeshPhongMatShape(std::shared_ptr<ChTriangleMeshShape> tms,
                                                        vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                        bool wireframe);

    vsg::ref_ptr<vsg::Group> createTrimeshPbrMatShape(std::shared_ptr<ChTriangleMeshShape> tms,
                                                      vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      bool wireframe);

    /// Convert the specified mesh into a triangle soup with vertex colors.
    /// Vertex normals are calculated from each face normal, resulting in sharp edges.
    vsg::ref_ptr<vsg::Group> createTrimeshColShape(std::shared_ptr<ChTriangleMeshShape> tms,
                                                   vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                   bool wireframe);

    /// Convert the specified mesh into a triangle mesh with vertex colors.
    /// Vertex normals are calculated by averaging the normals of incident faces, resulting in smoothed edges.
    vsg::ref_ptr<vsg::Group> createTrimeshColAvgShape(std::shared_ptr<ChTriangleMeshShape> tms,
                                                      vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      bool wireframe);

    /// Create a symbol to represent a refernce frame.
    /// These are 3 mututally orthogonal line segments, colored red, green, and blue.
    /// The color factor controls simultaneous darkening of the 3 colors (a value of 1 indicates pure colors).
    vsg::ref_ptr<vsg::Group> createFrameSymbol(vsg::ref_ptr<vsg::MatrixTransform> transform, float color_factor);

    vsg::ref_ptr<vsg::Group> createLineShape(ChVisualModel::ShapeInstance shapeInstance,
                                             std::shared_ptr<ChVisualMaterial> material,
                                             vsg::ref_ptr<vsg::MatrixTransform> transform,
                                             std::shared_ptr<ChLineShape> ls);
    vsg::ref_ptr<vsg::Group> createPathShape(ChVisualModel::ShapeInstance shapeInstance,
                                             std::shared_ptr<ChVisualMaterial> material,
                                             vsg::ref_ptr<vsg::MatrixTransform> transform,
                                             std::shared_ptr<ChPathShape> ps);

    vsg::ref_ptr<vsg::Group> createSpringShape(std::shared_ptr<ChLinkBase> link,
                                               ChVisualModel::ShapeInstance shapeInstance,
                                               std::shared_ptr<ChVisualMaterial> material,
                                               vsg::ref_ptr<vsg::MatrixTransform> transform,
                                               std::shared_ptr<ChSpringShape> ss);

    vsg::ref_ptr<vsg::Group> createUnitSegment(std::shared_ptr<ChLinkBase> link,
                                               ChVisualModel::ShapeInstance shapeInstance,
                                               std::shared_ptr<ChVisualMaterial> material,
                                               vsg::ref_ptr<vsg::MatrixTransform> transform);

    vsg::ref_ptr<vsg::Group> createDecoGrid(double ustep, double vstep, int nu, int nv, ChCoordsys<> pos, ChColor col);

    /// Create a ShaderSet for Phong shaded rendering with tiled textures.
    vsg::ref_ptr<vsg::ShaderSet> createTilingPhongShaderSet(vsg::ref_ptr<const vsg::Options> options = {});

    /// Create a ShaderSet for PBR shaded rendering with tiled textures.
    vsg::ref_ptr<vsg::ShaderSet> createTilingPbrShaderSet(vsg::ref_ptr<const vsg::Options> options = {});

    /// Assign compile traversal to enable compilation.
    void assignCompileTraversal(vsg::ref_ptr<vsg::CompileTraversal> ct);

    vsg::ref_ptr<vsg::CompileTraversal> compileTraversal;

  private:
    bool applyTexture(vsg::Path& path,
                      vsg::ref_ptr<vsg::GraphicsPipelineConfigurator> pipeConfig,
                      vsg::Descriptors& descriptors,
                      std::string& uniformName);

    bool applyMetalRoughnessTexture(vsg::Path& metalPath,
                                    vsg::Path& roughPath,
                                    vsg::ref_ptr<vsg::GraphicsPipelineConfigurator> pipeConfig,
                                    vsg::Descriptors& descriptors,
                                    std::string& uniformName);

    vsg::ref_ptr<vsg::PbrMaterialValue> createPbrMaterialFromChronoMaterial(
        std::shared_ptr<ChVisualMaterial> chronoMat);

    vsg::ref_ptr<vsg::PhongMaterialValue> createPhongMaterialFromChronoMaterial(
        std::shared_ptr<ChVisualMaterial> chronoMat);
};

}  // namespace vsg3d
}  // namespace chrono
#endif
