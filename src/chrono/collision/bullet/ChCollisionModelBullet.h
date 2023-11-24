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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CH_COLLISION_MODEL_BULLET_H
#define CH_COLLISION_MODEL_BULLET_H

#include <memory>
#include <vector>

#include "chrono/collision/ChCollisionModel.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/cbtCompoundShape.h"

// forward references
class cbtCollisionObject;

namespace chrono {

// forward references
class ChBody;
namespace fea {
class ChContactSurfaceMesh;
}

class ChConvexDecomposition;

/// @addtogroup collision_bullet
/// @{

/// Class defining the Bullet geometric model for collision detection.
class ChApi ChCollisionModelBullet : public ChCollisionModelImpl {
  public:
    ChCollisionModelBullet(ChCollisionModel* collision_model);
    virtual ~ChCollisionModelBullet();

    /// Returns the axis aligned bounding box (AABB) of the collision model.
    /// Note that SyncPosition() should be invoked before calling this.
    virtual geometry::ChAABB GetBoundingBox() const override;

    /// Return the outward safe margin.
    float GetEnvelope() { return model->GetEnvelope(); }

    /// Return the inward safe margin.
    float GetSafeMargin() { return model->GetSafeMargin(); }

    /// Sets the position and orientation of the collision
    /// model as the current position of the corresponding ChContactable
    virtual void SyncPosition() override;

    /// If the collision shape is a sphere, resize it and return true (if no
    /// sphere is found in this collision shape, return false).
    /// It can also change the outward envelope; the inward margin is automatically the radius of the sphere.
    bool SetSphereRadius(double coll_radius, double out_envelope);

  protected:
    /// Populate the collision system with the collision shapes defined in this model.
    void Populate();

    /// Additional operations to be performed on a change in collision family.
    virtual void OnFamilyChange(short int family_group, short int family_mask) override;

    void injectShape(std::shared_ptr<ChCollisionShape> shape,
                     std::shared_ptr<cbtCollisionShape> bt_shape,
                     const ChFrame<>& frame);

    void injectPath2D(std::shared_ptr<ChCollisionShapePath2D> shape_path, const ChFrame<>& frame);
    void injectConvexHull(std::shared_ptr<ChCollisionShapeConvexHull> shape_hull, const ChFrame<>& frame);
    void injectTriangleMesh(std::shared_ptr<ChCollisionShapeTriangleMesh> shape_trimesh, const ChFrame<>& frame);
    void injectTriangleProxy(std::shared_ptr<ChCollisionShapeMeshTriangle> shape_triangle);

    cbtCollisionObject* GetBulletObject() { return bt_collision_object.get(); }

    cbtScalar GetSuggestedFullMargin();

    std::unique_ptr<cbtCollisionObject> bt_collision_object;  ///< Bullet collision object containing Bullet geometries
    std::shared_ptr<cbtCompoundShape> bt_compound_shape;      ///< compound for models with more than one shape

    std::vector<std::shared_ptr<cbtCollisionShape>> m_bt_shapes;  ///< list of Bullet collision shapes in model
    std::vector<std::shared_ptr<ChCollisionShape>> m_shapes;      ///< extended list of collision shapes

    friend class ChCollisionSystemBullet;
    friend class ChCollisionSystemBulletMulticore;
    friend class chrono::fea::ChContactSurfaceMesh;
};

/// @} collision_bullet

CH_CLASS_VERSION(ChCollisionModelBullet, 0)

}  // end namespace chrono

#endif
