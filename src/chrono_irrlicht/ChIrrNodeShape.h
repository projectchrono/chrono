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

#ifndef CH_IRR_NODE_SHAPE_H
#define CH_IRR_NODE_SHAPE_H

#include <irrlicht.h>

#include "chrono/assets/ChVisualShape.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChSurfaceShape.h"

#include "chrono_irrlicht/ChApiIrr.h"

#define ESNT_CHIRRNODE_SHAPE 1202

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Irrlicht scene node associated with a visual shape in a visual model.
/// Such a node is a children of a ChIrrNodeModel.
class ChApiIrr ChIrrNodeShape : public irr::scene::ISceneNode {
  public:
    ChIrrNodeShape(std::shared_ptr<ChVisualShape> shape,  ///< Chrono visualization shape
                   irr::scene::ISceneNode* parent         ///< parent node in Irrlicht hierarchy
    );

    ~ChIrrNodeShape() {}

    /// Get the associated visualization shape.
    std::shared_ptr<ChVisualShape>& GetVisualShape() { return m_shape; }

    /// Update to reflect possible changes in the associated visual shape.
    void Update();

  private:
    void UpdateTriangleMesh(std::shared_ptr<ChTriangleMeshShape> trianglemesh);
    void UpdateTriangleMesh_mat(std::shared_ptr<ChTriangleMeshShape> trianglemesh);
    void UpdateTriangleMesh_col(std::shared_ptr<ChTriangleMeshShape> trianglemesh);

    void UpdateTriangleMeshFixedConnectivity(std::shared_ptr<ChTriangleMeshShape> trianglemesh);

    void UpdateGlyphs(std::shared_ptr<ChGlyphs> glyphs);
    void UpdateSurface(std::shared_ptr<ChSurfaceShape> surface);
    void UpdateLine(std::shared_ptr<geometry::ChLine> line, unsigned int nvertexes);

    virtual irr::scene::ESCENE_NODE_TYPE getType() const override;
    virtual void render() override {}
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const override { return m_box; }
    virtual ISceneNode* clone(ISceneNode* newParent, irr::scene::ISceneManager* newManager) override;

    irr::core::aabbox3d<irr::f32> m_box;     ///< bounding box
    std::shared_ptr<ChVisualShape> m_shape;  ///< associated visualization shape
    bool m_initial_update;                   ///< flag forcing a first update
};

/// @} irrlicht_module

}  // namespace irrlicht
}  // end namespace chrono

#endif
