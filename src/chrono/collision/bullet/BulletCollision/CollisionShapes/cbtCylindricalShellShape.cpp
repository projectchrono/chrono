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

#include "cbtCylindricalShellShape.h"

cbtCylindricalShellShape::cbtCylindricalShellShape(cbtScalar radius, cbtScalar hlen)
    : cbtConvexInternalShape() {
    cbtVector3 halfExtents(radius, hlen, radius);
    cbtVector3 margin(getMargin(), getMargin(), getMargin());
    m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;

    setSafeMargin(halfExtents);

    m_shapeType = CYLSHELL_SHAPE_PROXYTYPE;
}

void cbtCylindricalShellShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const {
    cbtTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
}

void cbtCylindricalShellShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const {
    // Principal axis aligned along y by default, radius in x, z-value not used

    cbtScalar radius2;                                    // square of cylinder radius
    cbtScalar height2;                                    // square of cylinder height
    cbtVector3 halfExtents = getHalfExtentsWithMargin();  // get cylinder dimension
    cbtScalar div12 = mass / 12.f;
    cbtScalar div4 = mass / 4.f;
    cbtScalar div2 = mass / 2.f;

    // cylinder is aligned along y
    int idxRadius = 0;
    int idxHeight = 1;

    // calculate squares
    radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
    height2 = cbtScalar(4.) * halfExtents[idxHeight] * halfExtents[idxHeight];

    // calculate tensor terms (cylinder is aligned along y)
    cbtScalar t1 = div12 * height2 + div4 * radius2;
    cbtScalar t2 = div2 * radius2;
    inertia.setValue(t1, t2, t1);
}

inline cbtVector3 CylShellLocalSupport(const cbtVector3& halfExtents, const cbtVector3& v) {
    const int cylinderUpAxis = 1;
    const int XX = 0;
    const int YY = 1;
    const int ZZ = 2;

    cbtScalar radius = halfExtents[XX];
    cbtScalar halfHeight = halfExtents[cylinderUpAxis];

    cbtVector3 tmp;
    cbtScalar d;

    cbtScalar s = cbtSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != cbtScalar(0.0)) {
        d = radius / s;
        tmp[XX] = v[XX] * d;
        tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
        tmp[ZZ] = v[ZZ] * d;
        return tmp;
    } else {
        tmp[XX] = radius;
        tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
        tmp[ZZ] = cbtScalar(0.0);
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
