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
                                         bool drawMode,
                                         std::shared_ptr<ChSurfaceShape> surface = nullptr);
    
    vsg::ref_ptr<vsg::Group> createPbrShape(BasicShape theShape,
                                         std::shared_ptr<ChVisualMaterial> material,
                                         vsg::ref_ptr<vsg::MatrixTransform> transform,
                                         bool drawMode,
                                         std::shared_ptr<ChSurfaceShape> surface = nullptr);
    
   // variant for general use, mesh can have unknown structure and noncontiguous faces
    // face normals will be converted to vertex normals, it will be bloated by additional vertices
    vsg::ref_ptr<vsg::Group> createTrimeshColShape(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                   bool drawMode,
                                                   std::shared_ptr<ChTriangleMeshShape> tms = nullptr);
    
    // variant for SCMDeformableTerrain, regular mesh needed, no face normals allowed
    // no face colors allowed
    vsg::ref_ptr<vsg::Group> createTrimeshColShapeSCM(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      bool drawMode,
                                                      std::shared_ptr<ChTriangleMeshShape> tms = nullptr);
    
    vsg::ref_ptr<vsg::Group> createTrimeshPhongMatShape(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                        bool drawMode,
                                                        std::shared_ptr<ChTriangleMeshShape> tms = nullptr);
    
    vsg::ref_ptr<vsg::Group> createTrimeshPbrMatShape(vsg::ref_ptr<vsg::MatrixTransform> transform,
                                                      bool drawMode,
                                                      std::shared_ptr<ChTriangleMeshShape> tms = nullptr);
    
    vsg::ref_ptr<vsg::Group> createCoGSymbol(vsg::ref_ptr<vsg::MatrixTransform> transform);
    
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
    
    /// create a ShaderSet for Phong shaded rendering with tiled textures
    vsg::ref_ptr<vsg::ShaderSet> createTilingPhongShaderSet(vsg::ref_ptr<const vsg::Options> options = {});
    
    /// create a ShaderSet for PBR shaded rendering with tiled textures
    vsg::ref_ptr<vsg::ShaderSet> createTilingPbrShaderSet(vsg::ref_ptr<const vsg::Options> options = {});
    
    /// assign compile traversal to enable compilation.
    void assignCompileTraversal(vsg::ref_ptr<vsg::CompileTraversal> ct);
    
    vsg::ref_ptr<vsg::CompileTraversal> compileTraversal;
    
private:
    bool ApplyTexture(vsg::Path &path, vsg::ref_ptr<vsg::GraphicsPipelineConfigurator> pipeConfig, vsg::Descriptors &descriptors, std::string &uniformName);
    
    bool ApplyMetalRoughnessTexture(vsg::Path &metalPath, vsg::Path &roughPath,  vsg::ref_ptr<vsg::GraphicsPipelineConfigurator> pipeConfig, vsg::Descriptors &descriptors, std::string &uniformName);
    
    vsg::ref_ptr<vsg::PbrMaterialValue> createPbrMaterialFromChronoMaterial(std::shared_ptr<chrono::ChVisualMaterial> chronoMat);
    
    vsg::ref_ptr<vsg::PhongMaterialValue> createPhongMaterialFromChronoMaterial(std::shared_ptr<chrono::ChVisualMaterial> chronoMat);
};

}  // namespace vsg3d
}  // namespace chrono
#endif
