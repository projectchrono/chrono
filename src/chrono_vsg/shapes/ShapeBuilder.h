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
// Radu Serban, Rainer Gericke
// =============================================================================

#ifndef CH_SHAPE_BUILDER_H
#define CH_SHAPE_BUILDER_H

#include <iostream>
#include <string>
#include "chrono_vsg/core/ChApiVSG.h"

#include <vsg/all.h>
#include <vsgXchange/all.h>

#include "chrono/assets/ChVisualSystem.h"
#include "chrono/assets/ChVisualModel.h"

namespace chrono {
namespace vsg3d {
class CH_VSG_API ShapeBuilder : public vsg::Inherit<vsg::Object, ShapeBuilder> {
  public:
    typedef enum { BOX_SHAPE, SPHERE_SHAPE, CYLINDER_SHAPE, CAPSULE_SHAPE, CONE_SHAPE, TRIANGLE_MESH_SHAPE } BasicShape;
    vsg::ref_ptr<vsg::Options> m_options;
    vsg::ref_ptr<vsg::SharedObjects> m_sharedObjects;

    vsg::ref_ptr<vsg::Group> createShape(BasicShape theShape,
                                         std::shared_ptr<ChPhysicsItem> physItem,
                                         ChVisualModel::ShapeInstance shapeInstance,
                                         std::shared_ptr<ChVisualMaterial> material,
                                         vsg::ref_ptr<vsg::MatrixTransform> transform,
                                         bool drawMode,
                                         std::shared_ptr<ChTriangleMeshShape> tms = nullptr);

    /// assign compile traversal to enable compilation.
    void assignCompileTraversal(vsg::ref_ptr<vsg::CompileTraversal> ct);

    vsg::ref_ptr<vsg::CompileTraversal> compileTraversal;
};
}  // namespace vsg3d
}  // namespace chrono
#endif