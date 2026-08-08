/*
***CHRONO***
*/

#include "cbtRoundedCylinderShape.h"

cbtRoundedCylinderShape::cbtRoundedCylinderShape(cbtScalar radius, cbtScalar hheight, cbtScalar sradius) : cbtConvexInternalShape(), s_radius(sradius) {
    cbtVector3 halfExtents(radius, radius, hheight);
    cbtVector3 margin(getMargin(), getMargin(), getMargin());
    m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;

    setSafeMargin(halfExtents);

    m_shapeType = ROUNDEDCYL_SHAPE_PROXYTYPE;
}

void cbtRoundedCylinderShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const {
    cbtTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
}

void cbtRoundedCylinderShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const {
    // Principal axis aligned along z by default, radius in x, y-value not used

    cbtScalar radius2;                                    // square of cylinder radius
    cbtScalar height2;                                    // square of cylinder height
    cbtVector3 halfExtents = getHalfExtentsWithMargin();  // get cylinder dimension
    cbtScalar div12 = mass / 12.f;
    cbtScalar div4 = mass / 4.f;
    cbtScalar div2 = mass / 2.f;

    // cylinder is aligned along z
    int idxRadius = 0;
    int idxHeight = 2;

    // calculate squares
    radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
    height2 = cbtScalar(4.) * halfExtents[idxHeight] * halfExtents[idxHeight];

    // calculate tensor terms (cylinder is aligned along z)
    cbtScalar t1 = div12 * height2 + div4 * radius2;
    cbtScalar t2 = div2 * radius2;
    inertia.setValue(t1, t1, t2);
}

static cbtVector3 RoundedcylLocalSupport(const cbtVector3& halfExtents, cbtScalar s_radius, const cbtVector3& v) {
    const int XX = 0;
    const int YY = 1;
    const int ZZ = 2;

    cbtScalar radius = halfExtents[XX];
    cbtScalar halfHeight = halfExtents[ZZ];

    cbtVector3 tmp;
    cbtScalar d;

    cbtScalar s = cbtSqrt(v[XX] * v[XX] + v[YY] * v[YY]);
    if (s != cbtScalar(0.0)) {
        d = radius / s;
        tmp[XX] = v[XX] * d;
        tmp[YY] = v[YY] * d;
        tmp[ZZ] = v[ZZ] < 0.0 ? -halfHeight : halfHeight;
    } else {
        tmp[XX] = radius;
        tmp[YY] = cbtScalar(0.0);
        tmp[ZZ] = v[ZZ] < 0.0 ? -halfHeight : halfHeight;
    }

    return tmp + s_radius * v;
}

cbtVector3 cbtRoundedCylinderShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const {
    return RoundedcylLocalSupport(getHalfExtentsWithoutMargin(), s_radius, vec);
}

void cbtRoundedCylinderShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const {
    for (int i = 0; i < numVectors; i++) {
        supportVerticesOut[i] = RoundedcylLocalSupport(getHalfExtentsWithoutMargin(), s_radius, vectors[i]);
    }
}

void cbtRoundedCylinderShape::setMargin(cbtScalar collisionMargin) {
    // correct the m_implicitShapeDimensions for the margin
    cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
    cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

    cbtConvexInternalShape::setMargin(collisionMargin);
    cbtVector3 newMargin(getMargin(), getMargin(), getMargin());
    m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
}

cbtVector3 cbtRoundedCylinderShape::localGetSupportingVertex(const cbtVector3& vec) const {
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

cbtVector3 cbtRoundedCylinderShape::getAnisotropicRollingFrictionDirection() const {
    cbtVector3 aniDir(0, 0, 0);
    aniDir[2] = 1;
    return aniDir;
}

void cbtRoundedCylinderShape::setLocalScaling(const cbtVector3& scaling) {
    cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
    cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;
    cbtVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

    cbtConvexInternalShape::setLocalScaling(scaling);

    m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
}
