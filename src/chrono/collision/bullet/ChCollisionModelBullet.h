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

//// TODO
//// - implement ArchiveIn & ArchiveOut

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

namespace collision {

class ChConvexDecomposition;

/// @addtogroup collision_bullet
/// @{

/// Class defining the Bullet geometric model for collision detection.
class ChApi ChCollisionModelBullet : public ChCollisionModel {
  protected:
    std::unique_ptr<cbtCollisionObject> bt_collision_object;  ///< Bullet collision object containing Bullet geometries
    std::shared_ptr<cbtCompoundShape> bt_compound_shape;  ///< Compound for models with more than one collision shape

  public:
    ChCollisionModelBullet();
    virtual ~ChCollisionModelBullet();

    /// Return the type of this collision model.
    virtual ChCollisionSystemType GetType() const override { return ChCollisionSystemType::BULLET; }

    /// Add all shapes already contained in another model.
    /// The 'another' model must be of ChCollisionModelBullet subclass.
    virtual bool AddCopyOfAnotherModel(ChCollisionModel* another) override;

    virtual void SetFamily(int mfamily) override;
    virtual int GetFamily() override;
    virtual void SetFamilyMaskNoCollisionWithFamily(int mfamily) override;
    virtual void SetFamilyMaskDoCollisionWithFamily(int mfamily) override;
    virtual bool GetFamilyMaskDoesCollisionWithFamily(int mfamily) override;

    /// Set the collision family group of this model.
    /// This is an alternative way of specifying the collision family for this
    /// object.  The value family_group must have a single bit set (i.e. it must
    /// be a power of 2). The corresponding family is then the bit position.
    virtual void SetFamilyGroup(short int group) override;

    /// Set the collision mask for this model.
    /// Any set bit in the specified mask indicates that this model collides with
    /// all objects whose family is equal to the bit position.
    virtual void SetFamilyMask(short int mask) override;

    /// Returns the axis aligned bounding box (AABB) of the collision model,
    /// i.e. max-min along the x,y,z world axes. Remember that SyncPosition()
    /// should be invoked before calling this.
    virtual void GetAABB(ChVector<>& bbmin, ChVector<>& bbmax) const override;

    /// Sets the position and orientation of the collision
    /// model as the current position of the corresponding ChContactable
    virtual void SyncPosition() override;

    /// If the collision shape is a sphere, resize it and return true (if no
    /// sphere is found in this collision shape, return false).
    /// It can also change the outward envelope; the inward margin is automatically the radius of the sphere.
    bool SetSphereRadius(double coll_radius, double out_envelope);

    /// Return the position and orientation of the collision shape with specified index, relative to the model frame.
    virtual ChCoordsys<> GetShapePos(int index) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    /// Remove this model from the collision system (if applicable).
    virtual void Dissociate() override;

    /// Insert this model into the collision system (if applicable).
    virtual void Associate() override;

    /// Populate the collision system with the collision shapes defined in this model.
    virtual void Populate() override;

    void injectShape(std::shared_ptr<ChCollisionShape> shape, std::shared_ptr<cbtCollisionShape> bt_shape, const ChFrame<>& frame);

    void injectPath2D(std::shared_ptr<ChCollisionShapePath2D> shape_path, const ChFrame<>& frame);
    void injectConvexHull(std::shared_ptr<ChCollisionShapeConvexHull> shape_hull, const ChFrame<>& frame);
    void injectTriangleMesh(std::shared_ptr<ChCollisionShapeTriangleMesh> shape_trimesh, const ChFrame<>& frame);
    void injectTriangleProxy(std::shared_ptr<ChCollisionShapeTriangle> shape_triangle,
                             ChVector<>& ep1,
                             ChVector<>& ep2,
                             ChVector<>& ep3,
                             bool owns_vertex_1,
                             bool owns_vertex_2,
                             bool owns_vertex_3,
                             bool owns_edge_1,
                             bool owns_edge_2,
                             bool owns_edge_3,
                             double radius);
    void injectTriangleProxy(std::shared_ptr<ChCollisionShapeTriangle> shape_triangle,
                             bool owns_vertex_1,
                             bool owns_vertex_2,
                             bool owns_vertex_3,
                             bool owns_edge_1,
                             bool owns_edge_2,
                             bool owns_edge_3,
                             double radius);

    void onFamilyChange();

    cbtCollisionObject* GetBulletModel() { return bt_collision_object.get(); }

    std::vector<std::shared_ptr<cbtCollisionShape>> m_bt_shapes;  ///< list of Bullet collision shapes in model

    friend class ChCollisionSystemBullet;
    friend class ChCollisionSystemBulletMulticore;
    friend class chrono::fea::ChContactSurfaceMesh;
};

/// @} collision_bullet

}  // end namespace collision

CH_CLASS_VERSION(collision::ChCollisionModelBullet, 0)

}  // end namespace chrono

#endif
