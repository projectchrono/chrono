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

#include "btCylindricalShellShape.h"

btCylindricalShellShape::btCylindricalShellShape(btScalar radius, btScalar hlen)
    : btConvexInternalShape() {
    btVector3 halfExtents(radius, hlen, radius);
    btVector3 margin(getMargin(), getMargin(), getMargin());
    m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;

    setSafeMargin(halfExtents);

    m_shapeType = CYLSHELL_SHAPE_PROXYTYPE;
}

void btCylindricalShellShape::getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const {
    btTransformAabb(getHalfExtentsWithoutMargin(), getMargin(), t, aabbMin, aabbMax);
}

void btCylindricalShellShape::calculateLocalInertia(btScalar mass, btVector3& inertia) const {
    // Principal axis aligned along y by default, radius in x, z-value not used

    btScalar radius2;                                    // square of cylinder radius
    btScalar height2;                                    // square of cylinder height
    btVector3 halfExtents = getHalfExtentsWithMargin();  // get cylinder dimension
    btScalar div12 = mass / 12.f;
    btScalar div4 = mass / 4.f;
    btScalar div2 = mass / 2.f;

    // cylinder is aligned along y
    int idxRadius = 0;
    int idxHeight = 1;

    // calculate squares
    radius2 = halfExtents[idxRadius] * halfExtents[idxRadius];
    height2 = btScalar(4.) * halfExtents[idxHeight] * halfExtents[idxHeight];

    // calculate tensor terms (cylinder is aligned along y)
    btScalar t1 = div12 * height2 + div4 * radius2;
    btScalar t2 = div2 * radius2;
    inertia.setValue(t1, t2, t1);
}

inline btVector3 CylShellLocalSupport(const btVector3& halfExtents, const btVector3& v) {
    const int cylinderUpAxis = 1;
    const int XX = 0;
    const int YY = 1;
    const int ZZ = 2;

    btScalar radius = halfExtents[XX];
    btScalar halfHeight = halfExtents[cylinderUpAxis];

    btVector3 tmp;
    btScalar d;

    btScalar s = btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != btScalar(0.0)) {
        d = radius / s;
        tmp[XX] = v[XX] * d;
        tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
        tmp[ZZ] = v[ZZ] * d;
        return tmp;
    } else {
        tmp[XX] = radius;
        tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
        tmp[ZZ] = btScalar(0.0);
        return tmp;
    }
}

btVector3 btCylindricalShellShape::localGetSupportingVertexWithoutMargin(const btVector3& vec) const {
    return CylShellLocalSupport(getHalfExtentsWithoutMargin(), vec);
}

void btCylindricalShellShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,
                                                                                btVector3* supportVerticesOut,
                                                                                int numVectors) const {
    for (int i = 0; i < numVectors; i++) {
        supportVerticesOut[i] = CylShellLocalSupport(getHalfExtentsWithoutMargin(), vectors[i]);
    }
}
