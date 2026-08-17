/*
***CHRONO***
*/

#ifndef BT_ROUNDED_BOX_H
#define BT_ROUNDED_BOX_H

#include "cbtPolyhedralConvexShape.h"
#include "cbtCollisionMargin.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h"
#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtMinMax.h"

/// The cbtRoundedBoxShape class implements a box with rounded edges shape primitive, centered around the origin.
/// Its central axis aligned with the Z axis.
ATTRIBUTE_ALIGNED16(class)
cbtRoundedBoxShape : public cbtConvexInternalShape {
  public:
    BT_DECLARE_ALIGNED_ALLOCATOR();

    cbtRoundedBoxShape(const cbtVector3& boxHalfExtents, cbtScalar sradius);

    cbtVector3 getHalfExtentsWithMargin() const {
        cbtVector3 halfExtents = getHalfExtentsWithoutMargin();
        cbtVector3 margin(getMargin(), getMargin(), getMargin());
        halfExtents += margin;
        return halfExtents;
    }

    const cbtVector3& getHalfExtentsWithoutMargin() const {
        return m_implicitShapeDimensions;  // changed in Bullet 2.63: assume the scaling and margin are included
    }

    cbtVector3 getHalfSize() const {
        return getHalfExtentsWithMargin();
    }

    cbtScalar getSphereRadius() const {
        return s_radius;
    }

    void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

    virtual void calculateLocalInertia(cbtScalar mass, cbtVector3 & inertia) const override;

    virtual cbtVector3 localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const override;

    virtual void batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const override;

    virtual void setMargin(cbtScalar collisionMargin) override;

    virtual cbtVector3 localGetSupportingVertex(const cbtVector3& vec) const override;

    virtual void setLocalScaling(const cbtVector3& scaling) override;

    // debugging
    virtual const char* getName() const override {
        return "RoundedBox";
    }

  private:
    cbtScalar s_radius;
};

#endif
