/*
***CHRONO***
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <cmath>

#include "cbtBarrelShape.h"

#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"
#include "LinearMath/cbtQuaternion.h"

cbtBarrelShape::cbtBarrelShape(cbtScalar sY_low, cbtScalar sY_high, cbtScalar sR_vert, cbtScalar sR_hor, cbtScalar sR_offset )
{
	Y_low = sY_low;
	Y_high = sY_high; 
	R_vert = sR_vert; 
	R_hor  = sR_hor; 
	R_offset = sR_offset;
	m_shapeType = BARREL_SHAPE_PROXYTYPE;
}

#include <stdio.h>
 cbtVector3	cbtBarrelShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0)const
{
	cbtVector3 supVec(0,0,0);
	cbtVector3 supVecD;

	// support point on the lathed ellipse?
	cbtScalar pY = vec0.y();
	cbtScalar pR = std::sqrt(vec0.z()*vec0.z() + vec0.x()*vec0.x());
	cbtScalar pH = pR;
	cbtScalar dX = vec0.x()/pR;
	cbtScalar dZ = vec0.z()/pR;
	cbtScalar tpar = std::atan((pY*R_vert)/(pH*R_hor));
	cbtScalar sY = R_vert * std::sin(tpar);
	cbtScalar sH = R_hor  * std::cos(tpar);
	cbtScalar sR = sH + R_offset;
	cbtScalar sX = dX * sR;
	cbtScalar sZ = dZ * sR;
	supVec.setValue(sX,sY,sZ);
	cbtScalar len = supVec.length();

	// support point on the top disc?
    if ((std::abs(Y_high) < R_vert) && (supVec.y() > Y_high)) {
        cbtScalar R_high_ellips = R_hor * std::sqrt(cbtScalar(1) - std::pow(Y_high / R_vert, cbtScalar(2)));
        cbtScalar R_high = R_high_ellips + R_offset;
        cbtScalar rad_ratio = pR / R_high;
        supVecD.setValue(vec0.x() / rad_ratio, Y_high, vec0.z() / rad_ratio);
        supVec = supVecD;
    }
    // support point on the bottom disc?
    if ((std::abs(Y_low) < R_vert) && (supVec.y() < Y_low)) {
        cbtScalar R_low_ellips = R_hor * std::sqrt(cbtScalar(1) - std::pow(Y_low / R_vert, cbtScalar(2)));
        cbtScalar R_low = R_low_ellips + R_offset;
        cbtScalar rad_ratio = pR / R_low;
        supVecD.setValue(vec0.x() / rad_ratio, Y_low, vec0.z() / rad_ratio);
        supVec = supVecD;
    }

	return supVec;
}

 void	cbtBarrelShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,cbtVector3* supportVerticesOut,int numVectors) const
{
	printf("NOT SUPPORTED!! \n");
}


void	cbtBarrelShape::calculateLocalInertia(cbtScalar mass,cbtVector3& inertia) const
{
	//// TODO 
	//as an approximation, take the inertia of the box that bounds the barrell

	cbtTransform ident;
	ident.setIdentity();

	cbtVector3 halfExtents;

	halfExtents.setValue((R_hor+R_offset), 
						 (R_vert),
						 (R_hor+R_offset));

	cbtScalar margin = CONVEX_DISTANCE_MARGIN;

	cbtScalar lx=cbtScalar(2.)*(halfExtents[0]+margin);
	cbtScalar ly=cbtScalar(2.)*(halfExtents[1]+margin);
	cbtScalar lz=cbtScalar(2.)*(halfExtents[2]+margin);
	const cbtScalar x2 = lx*lx;
	const cbtScalar y2 = ly*ly;
	const cbtScalar z2 = lz*lz;
	const cbtScalar scaledmass = mass * cbtScalar(.08333333);

	inertia[0] = scaledmass * (y2+z2);
	inertia[1] = scaledmass * (x2+z2);
	inertia[2] = scaledmass * (x2+y2);
}


void cbtBarrelShape::getAabb(const cbtTransform& t,cbtVector3& aabbMin,cbtVector3& aabbMax) const
{
	cbtVector3 halfExtents; 
	halfExtents.setValue((R_hor+R_offset), 
						 (R_vert),
						 (R_hor+R_offset));

	cbtMatrix3x3 abs_b = t.getBasis().absolute();  
	cbtVector3 center = t.getOrigin();
	cbtVector3 extent = cbtVector3(abs_b[0].dot(halfExtents),
		   abs_b[1].dot(halfExtents),
		  abs_b[2].dot(halfExtents));
	extent += cbtVector3(getMargin(),getMargin(),getMargin());

	aabbMin = center - extent;
	aabbMax = center + extent;

}


