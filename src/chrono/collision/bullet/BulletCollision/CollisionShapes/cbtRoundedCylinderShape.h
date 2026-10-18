/*
***CHRONO***
*/

#ifndef BT_ROUNDED_CYLINDER_H
#define BT_ROUNDED_CYLINDER_H

#include "cbtBoxShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"  // for the types
#include "LinearMath/cbtVector3.h"

/// The cbtRoundedCylinderShape class implements a cylinder with rounded edges shape primitive, centered around the origin.
/// Its central axis aligned with the Z axis.
ATTRIBUTE_ALIGNED16(class)
cbtRoundedCylinderShape : public cbtConvexInternalShape {
  public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    cbtRoundedCylinderShape(cbtScalar radius, cbtScalar hheight, cbtScalar sradius);

    cbtVector3 getHalfExtentsWithMargin() const {
        cbtVector3 halfExtents = getHalfExtentsWithoutMargin();
        cbtVector3 margin(getMargin(), getMargin(), getMargin());
        halfExtents += margin;
        return halfExtents;
    }

    const cbtVector3& getHalfExtentsWithoutMargin() const {
        return m_implicitShapeDimensions;  // changed in Bullet 2.63: assume the scaling and margin are included
    }

    cbtScalar getRadius() const {
        return getHalfExtentsWithMargin().getX();
    }

    cbtScalar getHalfLength() const {
        return getHalfExtentsWithMargin().getZ();
    }

    cbtScalar getSphereRadius() const {
        return s_radius;
    }

    virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const override;

    virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const override;

    virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const override;

    virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const override;

    virtual void setMargin(cbtScalar collisionMargin) override;

    virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const override;

    virtual cbtVector3 getAnisotropicRollingFrictionDirection() const override;

    virtual void setLocalScaling(const cbtVector3& scaling) override;

    // debugging
    virtual const char* getName() const override {
        return "RoundedCylinder";
    }

    virtual int calculateSerializeBufferSize() const override;

    /// fills the dataBuffer and returns the struct name (and 0 on failure)
    virtual const char* serialize(void* dataBuffer, cbtSerializer* serializer) const override;

  private:
    cbtScalar s_radius;
};

/// do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct cbtRoundedCylinderShapeData {
    cbtConvexInternalShapeData m_convexInternalShapeData;
    char m_padding[4];
};

SIMD_FORCE_INLINE int cbtRoundedCylinderShape::calculateSerializeBufferSize() const {
    return sizeof(cbtRoundedCylinderShapeData);
}

/// fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char* cbtRoundedCylinderShape::serialize(void* dataBuffer, cbtSerializer* serializer) const {
    cbtRoundedCylinderShapeData* shapeData = (cbtRoundedCylinderShapeData*)dataBuffer;

    cbtConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData, serializer);

    // Fill padding with zeros to appease msan.
    shapeData->m_padding[0] = 0;
    shapeData->m_padding[1] = 0;
    shapeData->m_padding[2] = 0;
    shapeData->m_padding[3] = 0;

    return "cbtRoundedCylinderShapeData";
}

#endif
