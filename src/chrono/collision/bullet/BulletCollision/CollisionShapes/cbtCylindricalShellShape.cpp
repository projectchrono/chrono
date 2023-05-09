/*
***CHRONO***
*/

#include "cbtCylindricalShellShape.h"

cbtCylindricalShellShape::cbtCylindricalShellShape(cbtScalar radius, cbtScalar hheight)
    : cbtConvexInternalShape() {
    cbtVector3 halfExtents(radius, radius, hheight);
    cbtVector3 margin(getMargin(), getMargin(), getMargin());
    m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;

    setSafeMargin(halfExtents);

    m_shapeType = CYLSHELL_SHAPE_PROXYTYPE;
}

void cbtCylindricalShellShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const {
    cbtTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
}

void cbtCylindricalShellShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const {
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

inline cbtVector3 CylShellLocalSupport(const cbtVector3& halfExtents, const cbtVector3& v) {
    const int cylinderUpAxis = 2;
    const int XX = 0;
    const int YY = 1;
    const int ZZ = 2;

    cbtScalar radius = halfExtents[XX];
    cbtScalar halfHeight = halfExtents[cylinderUpAxis];

    cbtVector3 tmp;
    cbtScalar d;

    cbtScalar s = cbtSqrt(v[XX] * v[XX] + v[YY] * v[YY]);
    if (s != cbtScalar(0.0)) {
        d = radius / s;
        tmp[XX] = v[XX] * d;
        tmp[YY] = v[YY] * d;
        tmp[ZZ] = v[ZZ] < 0.0 ? -halfHeight : halfHeight;
        return tmp;
    } else {
        tmp[XX] = radius;
        tmp[YY] = cbtScalar(0.0);
        tmp[ZZ] = v[ZZ] < 0.0 ? -halfHeight : halfHeight;
        return tmp;
    }
}

cbtVector3 cbtCylindricalShellShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec) const {
    return CylShellLocalSupport(getHalfExtentsWithoutMargin(), vec);
}

void cbtCylindricalShellShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,
                                                                                cbtVector3* supportVerticesOut,
                                                                                int numVectors) const {
    for (int i = 0; i < numVectors; i++) {
        supportVerticesOut[i] = CylShellLocalSupport(getHalfExtentsWithoutMargin(), vectors[i]);
    }
}
