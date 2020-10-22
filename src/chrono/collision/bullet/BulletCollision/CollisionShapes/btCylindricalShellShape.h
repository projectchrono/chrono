/*
***CHRONO***
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If
you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original
software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CYLINDRICAL_SHELL_H
#define BT_CYLINDRICAL_SHELL_H

#include "btBoxShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"  // for the types
#include "LinearMath/btVector3.h"

/// The btCylindricalShellShape class implements a cylindrical shell shape primitive, centered around the origin.
/// Its central axis aligned with the Y axis.
ATTRIBUTE_ALIGNED16(class)
btCylindricalShellShape : public btConvexInternalShape {
  public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    btVector3 getHalfExtentsWithMargin() const {
        btVector3 halfExtents = getHalfExtentsWithoutMargin();
        btVector3 margin(getMargin(), getMargin(), getMargin());
        halfExtents += margin;
        return halfExtents;
    }

    const btVector3& getHalfExtentsWithoutMargin() const {
        return m_implicitShapeDimensions;  // changed in Bullet 2.63: assume the scaling and margin are included
    }

    btCylindricalShellShape(btScalar radius, btScalar hlen);

    void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;

    virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const;

    virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& vec) const;

    virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,
                                                                   btVector3* supportVerticesOut, int numVectors) const;

    virtual void setMargin(btScalar collisionMargin) {
        // correct the m_implicitShapeDimensions for the margin
        btVector3 oldMargin(getMargin(), getMargin(), getMargin());
        btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

        btConvexInternalShape::setMargin(collisionMargin);
        btVector3 newMargin(getMargin(), getMargin(), getMargin());
        m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
    }

    virtual btVector3 localGetSupportingVertex(const btVector3& vec) const {
        btVector3 supVertex;
        supVertex = localGetSupportingVertexWithoutMargin(vec);

        if (getMargin() != btScalar(0.)) {
            btVector3 vecnorm = vec;
            if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON)) {
                vecnorm.setValue(btScalar(-1.), btScalar(-1.), btScalar(-1.));
            }
            vecnorm.normalize();
            supVertex += getMargin() * vecnorm;
        }
        return supVertex;
    }

    virtual btVector3 getAnisotropicRollingFrictionDirection() const {
        btVector3 aniDir(0, 0, 0);
        aniDir[1] = 1;
        return aniDir;
    }

    btScalar getRadius() const { return getHalfExtentsWithMargin().getX(); }
    btScalar getHalfLength() const { return getHalfExtentsWithMargin().getY(); }

    virtual void setLocalScaling(const btVector3& scaling) {
        btVector3 oldMargin(getMargin(), getMargin(), getMargin());
        btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;
        btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

        btConvexInternalShape::setLocalScaling(scaling);

        m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
    }

    // debugging
    virtual const char* getName() const { return "CylindricalShell"; }

    virtual int calculateSerializeBufferSize() const;

    /// fills the dataBuffer and returns the struct name (and 0 on failure)
    virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;
};

/// do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btCylindricalShellShapeData {
    btConvexInternalShapeData m_convexInternalShapeData;
    char m_padding[4];
};

SIMD_FORCE_INLINE int btCylindricalShellShape::calculateSerializeBufferSize() const {
    return sizeof(btCylindricalShellShapeData);
}

/// fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* btCylindricalShellShape::serialize(void* dataBuffer, btSerializer* serializer) const {
    btCylindricalShellShapeData* shapeData = (btCylindricalShellShapeData*)dataBuffer;

    btConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

    // Fill padding with zeros to appease msan.
    shapeData->m_padding[0] = 0;
    shapeData->m_padding[1] = 0;
    shapeData->m_padding[2] = 0;
    shapeData->m_padding[3] = 0;

    return "btCylindricalShellShapeData";
}

#endif  // BT_CYLINDER_MINKOWSKI_H
