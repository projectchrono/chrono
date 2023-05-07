/*
***CHRONO***
*/

#ifndef BT_CYLINDRICAL_SHELL_H
#define BT_CYLINDRICAL_SHELL_H

#include "cbtBoxShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types
#include "LinearMath/cbtVector3.h"

/// The cbtCylindricalShellShape class implements a cylindrical shell shape primitive, centered around the origin.
/// Its central axis aligned with the Y axis.
ATTRIBUTE_ALIGNED16(class)
cbtCylindricalShellShape : public cbtConvexInternalShape {
  public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    cbtVector3 getHalfExtentsWithMargin() const {
        cbtVector3 halfExtents = getHalfExtentsWithoutMargin();
        cbtVector3 margin(getMargin(), getMargin(), getMargin());
        halfExtents += margin;
        return halfExtents;
    }

    const cbtVector3& getHalfExtentsWithoutMargin() const {
        return m_implicitShapeDimensions;  // changed in Bullet 2.63: assume the scaling and margin are included
    }

    cbtCylindricalShellShape(cbtScalar radius, cbtScalar hheight);

    void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

    virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const;

    virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const;

    virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,
                                                                   cbtVector3* supportVerticesOut, int numVectors) const;

    virtual void setMargin(cbtScalar collisionMargin) {
        // correct the m_implicitShapeDimensions for the margin
        cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
        cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

        cbtConvexInternalShape::setMargin(collisionMargin);
        cbtVector3 newMargin(getMargin(), getMargin(), getMargin());
        m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
    }

    virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const {
        cbtVector3 supVertex;
        supVertex = localGetSupportingVertexWithoutMargin(vec);

        if (getMargin() != cbtScalar(0.)) {
            cbtVector3 vecnorm = vec;
            if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON)) {
                vecnorm.setValue(cbtScalar(-1.), cbtScalar(-1.), cbtScalar(-1.));
            }
            vecnorm.normalize();
            supVertex += getMargin() * vecnorm;
        }
        return supVertex;
    }

    virtual cbtVector3 getAnisotropicRollingFrictionDirection() const {
        cbtVector3 aniDir(0, 0, 0);
        aniDir[2] = 1;
        return aniDir;
    }

    cbtScalar getRadius() const { return getHalfExtentsWithMargin().getX(); }
    cbtScalar getHalfLength() const { return getHalfExtentsWithMargin().getY(); }

    virtual void setLocalScaling(const cbtVector3& scaling) {
        cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
        cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;
        cbtVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

        cbtConvexInternalShape::setLocalScaling(scaling);

        m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
    }

    // debugging
    virtual const char* getName() const { return "CylindricalShell"; }

    virtual int calculateSerializeBufferSize() const;

    /// fills the dataBuffer and returns the struct name (and 0 on failure)
    virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const;
};

/// do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct cbtCylindricalShellShapeData {
    cbtConvexInternalShapeData m_convexInternalShapeData;
    char m_padding[4];
};

SIMD_FORCE_INLINE int cbtCylindricalShellShape::calculateSerializeBufferSize() const {
    return sizeof(cbtCylindricalShellShapeData);
}

/// fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* cbtCylindricalShellShape::serialize(void* dataBuffer, cbtSerializer* serializer) const {
    cbtCylindricalShellShapeData* shapeData = (cbtCylindricalShellShapeData*)dataBuffer;

    cbtConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

    // Fill padding with zeros to appease msan.
    shapeData->m_padding[0] = 0;
    shapeData->m_padding[1] = 0;
    shapeData->m_padding[2] = 0;
    shapeData->m_padding[3] = 0;

    return "cbtCylindricalShellShapeData";
}

#endif  // BT_CYLINDER_MINKOWSKI_H
