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

//#define NOMINMAX
#include <algorithm>

#include "bt2DShape.h"

#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "LinearMath/btQuaternion.h"

bt2DarcShape::bt2DarcShape(btScalar mx, btScalar my, btScalar mradius, btScalar mangle1, btScalar mangle2, bool mcounterclock, btScalar mzthickness)
{
	x = mx;
	y = my;
	radius = mradius;
	angle1 = mangle1;
	angle2 = mangle2;
	m_shapeType = ARC_SHAPE_PROXYTYPE;
    counterclock = mcounterclock;
    zthickness = mzthickness;
}

#include <stdio.h>
 btVector3	bt2DarcShape::localGetSupportingVertexWithoutMargin(const btVector3& vec0)const
{
	btVector3 supVec(0,0,0);
	
    btVector3 O1(x,y,0);
	
    btVector3 D( supVec-O1 );
    D.normalize();

    supVec = D * radius;

	return supVec;
}

 void	bt2DarcShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const
{
	printf("NOT SUPPORTED!! \n");
}


void	bt2DarcShape::calculateLocalInertia(btScalar mass,btVector3& inertia) const
{
	//***TO DO***
	//as an approximation, take the inertia of the box that bounds the barrell

	btTransform ident;
	ident.setIdentity();

	btVector3 halfExtents;

	halfExtents.setValue((radius), 
						 (radius),
						 (radius));

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


void bt2DarcShape::getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
{
	btVector3 halfExtents; 
	halfExtents.setValue((radius), 
						 (radius),
						 (zthickness*0.5f));

	btMatrix3x3 abs_b = t.getBasis().absolute();  
	btVector3 center = t.getOrigin()+ t.getBasis()*btVector3(this->x, this->y, 0);
	btVector3 extent = btVector3(abs_b[0].dot(halfExtents),
		   abs_b[1].dot(halfExtents),
		  abs_b[2].dot(halfExtents));
	extent += btVector3(getMargin(),getMargin(),0);

	aabbMin = center - extent;
	aabbMax = center + extent;

}



////////////////////////////////////////////////

bt2DsegmentShape::bt2DsegmentShape(const btVector3& mP1, const btVector3& mP2, const btScalar mzthickness)
{
	P1 = mP1;
	P2 = mP2;
	m_shapeType = SEGMENT_SHAPE_PROXYTYPE;
    zthickness = mzthickness;
}

#include <stdio.h>
 btVector3	bt2DsegmentShape::localGetSupportingVertexWithoutMargin(const btVector3& vec0)const
{
	btVector3 supVec(0,0,0);
	
    btScalar L1 = (vec0-P1).length();
    btScalar L2 = (vec0-P2).length();
    
    if(L1<L2)
        return P1;
       
    return P2;
}

 void	bt2DsegmentShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const
{
	printf("NOT SUPPORTED!! \n");
}


void	bt2DsegmentShape::calculateLocalInertia(btScalar mass,btVector3& inertia) const
{
	//***TO DO***
	//as an approximation, take the inertia of the box that bounds the barrell

	btTransform ident;
	ident.setIdentity();

    double hlen = 0.5*(P2-P1).length();

	btVector3 halfExtents;

	halfExtents.setValue((btScalar)(hlen), 
						 (btScalar)(hlen),
						 (btScalar)(hlen));

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


void bt2DsegmentShape::getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
{
    btVector3 P1w = t*P1;
    btVector3 P2w = t*P2;
    btVector3 vminabs (std::min(P1w.x(),P2w.x()), std::min(P1w.y(),P2w.y()),  P1w.z()-(zthickness*0.5f));
    btVector3 vmaxabs (std::max(P1w.x(),P2w.x()), std::max(P1w.y(),P2w.y()),  P1w.z()+(zthickness*0.5f));
    vminabs -= btVector3(getMargin(),getMargin(),0);
    vmaxabs += btVector3(getMargin(),getMargin(),0);
    aabbMin = vminabs;
    aabbMax = vmaxabs;
}

