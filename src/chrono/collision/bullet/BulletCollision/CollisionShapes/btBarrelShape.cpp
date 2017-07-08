/*
*** ALEX ***
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


#include "btBarrelShape.h"

#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "LinearMath/btQuaternion.h"

btBarrelShape::btBarrelShape(btScalar sY_low, btScalar sY_high, btScalar sR_vert, btScalar sR_hor, btScalar sR_offset )
{
	Y_low = sY_low;
	Y_high = sY_high; 
	R_vert = sR_vert; 
	R_hor  = sR_hor; 
	R_offset = sR_offset;
	m_shapeType = BARREL_SHAPE_PROXYTYPE;
}

#include <stdio.h>
 btVector3	btBarrelShape::localGetSupportingVertexWithoutMargin(const btVector3& vec0)const
{
	btVector3 supVec(0,0,0);
	btVector3 supVecD;

	// suppport point on the lathed ellipse?
	btScalar pY = vec0.y();
	btScalar pR = std::sqrt (vec0.z()*vec0.z() + vec0.x()*vec0.x() );
	btScalar pH = pR;
	btScalar dX = vec0.x()/pR;
	btScalar dZ = vec0.z()/pR;
	btScalar tpar = std::atan((pY*R_vert)/(pH*R_hor));
	btScalar sY = R_vert * sin(tpar);
	btScalar sH = R_hor  * cos(tpar);
	btScalar sR = sH + R_offset;
	btScalar sX = dX * sR;
	btScalar sZ = dZ * sR;
	supVec.setValue(sX,sY,sZ);
	btScalar len = supVec.length();

	// support point on the top disc?
	if ((fabs(Y_high) < R_vert)&(supVec.y()>Y_high))
	{
		btScalar R_high_ellips = R_hor * sqrt( 1- pow( Y_high/R_vert ,2) );
		btScalar R_high = R_high_ellips + R_offset;
		btScalar rad_ratio = pR/R_high;
		supVecD.setValue(vec0.x()/rad_ratio, Y_high, vec0.z()/rad_ratio);
		supVec = supVecD;
	}
	// support point on the bottom disc?
	if ((fabs(Y_low) < R_vert)&(supVec.y()<Y_low))
	{
		btScalar R_low_ellips = R_hor * sqrt( 1- pow( Y_low/R_vert ,2) );
		btScalar R_low = R_low_ellips + R_offset;
		btScalar rad_ratio = pR/R_low;
		supVecD.setValue(vec0.x()/rad_ratio, Y_low, vec0.z()/rad_ratio);
		supVec = supVecD;
	}

	return supVec;
}

 void	btBarrelShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const
{
	printf("NOT SUPPORTED!! \n");
}


void	btBarrelShape::calculateLocalInertia(btScalar mass,btVector3& inertia) const
{
	//***TO DO***
	//as an approximation, take the inertia of the box that bounds the barrell

	btTransform ident;
	ident.setIdentity();

	btVector3 halfExtents;

	halfExtents.setValue((R_hor+R_offset), 
						 (R_vert),
						 (R_hor+R_offset));

	btScalar margin = CONVEX_DISTANCE_MARGIN;

	btScalar lx=btScalar(2.)*(halfExtents[0]+margin);
	btScalar ly=btScalar(2.)*(halfExtents[1]+margin);
	btScalar lz=btScalar(2.)*(halfExtents[2]+margin);
	const btScalar x2 = lx*lx;
	const btScalar y2 = ly*ly;
	const btScalar z2 = lz*lz;
	const btScalar scaledmass = mass * btScalar(.08333333);

	inertia[0] = scaledmass * (y2+z2);
	inertia[1] = scaledmass * (x2+z2);
	inertia[2] = scaledmass * (x2+y2);
}


void btBarrelShape::getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
{
	btVector3 halfExtents; 
	halfExtents.setValue((R_hor+R_offset), 
						 (R_vert),
						 (R_hor+R_offset));

	btMatrix3x3 abs_b = t.getBasis().absolute();  
	btVector3 center = t.getOrigin();
	btVector3 extent = btVector3(abs_b[0].dot(halfExtents),
		   abs_b[1].dot(halfExtents),
		  abs_b[2].dot(halfExtents));
	extent += btVector3(getMargin(),getMargin(),getMargin());

	aabbMin = center - extent;
	aabbMax = center + extent;

}


