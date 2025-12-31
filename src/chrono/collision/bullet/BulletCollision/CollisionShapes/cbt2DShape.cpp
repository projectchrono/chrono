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

//#define NOMINMAX
#include <algorithm>

#include "cbt2DShape.h"

#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"
#include "LinearMath/cbtQuaternion.h"

cbt2DarcShape::cbt2DarcShape(cbtScalar mx, cbtScalar my, cbtScalar mradius, cbtScalar mangle1, cbtScalar mangle2, bool mcounterclock, cbtScalar mzthickness)
{
	x = mx;
	y = my;
	radius = mradius;
	angle1 = mangle1;
	angle2 = mangle2;
	m_shapeType = ARC2D_SHAPE_PROXYTYPE;
	counterclock = mcounterclock;
	zthickness = mzthickness;
}

#include <stdio.h>
 cbtVector3	cbt2DarcShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0)const
{
	cbtVector3 supVec(0,0,0);
	
	cbtVector3 O1(x, y, 0);
	
	cbtVector3 D( supVec-O1 );
	D.normalize();

	supVec = D * radius;

	return supVec;
}

 void	cbt2DarcShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,cbtVector3* supportVerticesOut,int numVectors) const
{
	printf("NOT SUPPORTED!! \n");
}


void	cbt2DarcShape::calculateLocalInertia(cbtScalar mass,cbtVector3& inertia) const
{
	//// TODO 
	//as an approximation, take the inertia of the box that bounds the barrell

	cbtTransform ident;
	ident.setIdentity();

	cbtVector3 halfExtents;

	halfExtents.setValue((radius), 
						 (radius),
						 (radius));

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


void cbt2DarcShape::getAabb(const cbtTransform& t,cbtVector3& aabbMin,cbtVector3& aabbMax) const
{
	cbtVector3 halfExtents; 
	halfExtents.setValue((radius), 
						 (radius),
						 (zthickness*0.5f));

	cbtMatrix3x3 abs_b = t.getBasis().absolute();  
	cbtVector3 center = t.getOrigin()+ t.getBasis()*cbtVector3(this->x, this->y, 0);
	cbtVector3 extent = cbtVector3(abs_b[0].dot(halfExtents),
		   abs_b[1].dot(halfExtents),
		  abs_b[2].dot(halfExtents));
	extent += cbtVector3(getMargin(),getMargin(),0);

	aabbMin = center - extent;
	aabbMax = center + extent;

}



////////////////////////////////////////////////

cbt2DsegmentShape::cbt2DsegmentShape(const cbtVector3& mP1, const cbtVector3& mP2, const cbtScalar mzthickness)
{
	P1 = mP1;
	P2 = mP2;
	m_shapeType = SEGMENT2D_SHAPE_PROXYTYPE;
	zthickness = mzthickness;
}

#include <stdio.h>
 cbtVector3	cbt2DsegmentShape::localGetSupportingVertexWithoutMargin(const cbtVector3& vec0)const
{
	cbtVector3 supVec(0,0,0);
	
	cbtScalar L1 = (vec0-P1).length();
	cbtScalar L2 = (vec0-P2).length();
	
	if(L1<L2)
	  return P1;

	return P2;
}

 void	cbt2DsegmentShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,cbtVector3* supportVerticesOut,int numVectors) const
{
	printf("NOT SUPPORTED!! \n");
}


void	cbt2DsegmentShape::calculateLocalInertia(cbtScalar mass,cbtVector3& inertia) const
{
	//// TODO 
	//as an approximation, take the inertia of the box that bounds the barrell

	cbtTransform ident;
	ident.setIdentity();

    double hlen = 0.5*(P2-P1).length();

	cbtVector3 halfExtents;

	halfExtents.setValue((cbtScalar)(hlen), 
						 (cbtScalar)(hlen),
						 (cbtScalar)(hlen));

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


void cbt2DsegmentShape::getAabb(const cbtTransform& t,cbtVector3& aabbMin,cbtVector3& aabbMax) const
{
	cbtVector3 P1w = t*P1;
	cbtVector3 P2w = t*P2;
	cbtVector3 vminabs (std::min(P1w.x(),P2w.x()), std::min(P1w.y(),P2w.y()),  P1w.z()-(zthickness*0.5f));
	cbtVector3 vmaxabs (std::max(P1w.x(),P2w.x()), std::max(P1w.y(),P2w.y()),  P1w.z()+(zthickness*0.5f));
	vminabs -= cbtVector3(getMargin(),getMargin(),0);
	vmaxabs += cbtVector3(getMargin(),getMargin(),0);
	aabbMin = vminabs;
	aabbMax = vmaxabs;
}

