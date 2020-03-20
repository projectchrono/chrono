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
// Authors: Radu Serban
// =============================================================================

#ifndef CH_COLLISION_SHAPE_BULLET
#define CH_COLLISION_SHAPE_BULLET

#include "chrono/collision/ChCollisionShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCollisionShape.h"

namespace chrono {
namespace collision {

class ChCollisionShapeBullet : public ChCollisionShape {
  public:
    ChCollisionShapeBullet(Type type, std::shared_ptr<ChMaterialSurface> material)
        : ChCollisionShape(type, material), m_bt_shape(nullptr) {}

    ~ChCollisionShapeBullet() { delete m_bt_shape; }

  private:
    btCollisionShape* m_bt_shape;

    friend class ChCollisionModelBullet;
};

}  // end namespace collision
}  // end namespace chrono

#endif
