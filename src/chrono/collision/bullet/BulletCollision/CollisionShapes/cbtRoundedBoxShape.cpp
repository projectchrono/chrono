/*
***CHRONO***
*/

#include "cbtRoundedBoxShape.h"

cbtRoundedBoxShape::cbtRoundedBoxShape(const cbtVector3& boxHalfExtents, cbtScalar sradius) : cbtConvexInternalShape(), s_radius(sradius) {
    cbtVector3 margin(getMargin(), getMargin(), getMargin());
    m_implicitShapeDimensions = (boxHalfExtents * m_localScaling) - margin;

    setSafeMargin(boxHalfExtents);

    m_shapeType = ROUNDEDBOX_SHAPE_PROXYTYPE;
}

void cbtRoundedBoxShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const {
    cbtTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
}

void cbtRoundedBoxShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const {
    // cbtScalar margin = cbtScalar(0.);
    cbtVector3 halfExtents = getHalfExtentsWithMargin();

    cbtScalar lx = cbtScalar(2.) * (halfExtents.x());
    cbtScalar ly = cbtScalar(2.) * (halfExtents.y());
    cbtScalar lz = cbtScalar(2.) * (halfExtents.z());

    inertia.setValue(mass / (cbtScalar(12.0)) * (ly * ly + lz * lz), mass / (cbtScalar(12.0)) * (lx * lx + lz * lz), mass / (cbtScalar(12.0)) * (lx * lx + ly * ly));
}

cbtVector3 cbtRoundedBoxShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const {
    const cbtVector3& halfExtents = getHalfExtentsWithoutMargin();

    cbtVector3 tmp(cbtFsels(vec.x(), halfExtents.x(), -halfExtents.x()),  //
                   cbtFsels(vec.y(), halfExtents.y(), -halfExtents.y()),  //
                   cbtFsels(vec.z(), halfExtents.z(), -halfExtents.z()));

    return tmp + s_radius * vec;
}

void cbtRoundedBoxShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors, cbtVector3* supportVerticesOut, int numVectors) const {
    for (int i = 0; i < numVectors; i++) {
        supportVerticesOut[i] = localGetSupportingVertexWithoutMargin(vectors[i]);
    }
}

void cbtRoundedBoxShape::setMargin(cbtScalar collisionMargin) {
    // correct the m_implicitShapeDimensions for the margin
    cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
    cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;

    cbtConvexInternalShape::setMargin(collisionMargin);
    cbtVector3 newMargin(getMargin(), getMargin(), getMargin());
    m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;
}

cbtVector3 cbtRoundedBoxShape::localGetSupportingVertex(const cbtVector3& vec) const {
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

void cbtRoundedBoxShape::setLocalScaling(const cbtVector3& scaling) {
    cbtVector3 oldMargin(getMargin(), getMargin(), getMargin());
    cbtVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions + oldMargin;
    cbtVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

    cbtConvexInternalShape::setLocalScaling(scaling);

    m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;
}
