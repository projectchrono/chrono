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

#include "btCEtriangleShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "LinearMath/btQuaternion.h"
#include <stdio.h>

using namespace chrono;

btCEtriangleShape::btCEtriangleShape(ChVector<>* mp1,
                    ChVector<>* mp2,
                    ChVector<>* mp3,
	                bool mowns_vertex_1,
                    bool mowns_vertex_2,
                    bool mowns_vertex_3,
                    bool mowns_edge_1,
                    bool mowns_edge_2,
                    bool mowns_edge_3,
                    double msphereswept_rad)
{
    p1 = mp1;
    p2 = mp2;
    p3 = mp3;
    owns_vertex_1 = mowns_vertex_1;
    owns_vertex_2 = mowns_vertex_2;
    owns_vertex_3 = mowns_vertex_3;
    owns_edge_1 = mowns_edge_1;
    owns_edge_2 = mowns_edge_2;
    owns_edge_3 = mowns_edge_3;
    sphereswept_rad = msphereswept_rad;

    m_shapeType = CE_TRIANGLE_SHAPE_PROXYTYPE;
}



 btVector3	btCEtriangleShape::localGetSupportingVertexWithoutMargin(const btVector3& vec0)const
{
    btVector3 supVec(btScalar(0.),btScalar(0.),btScalar(0.));
	btScalar newDot,maxDot = btScalar(-BT_LARGE_FLOAT);
    btVector3 vtx;
	vtx = btVector3(this->p1->x,this->p1->y,this->p1->z);
	newDot = vec0.dot(vtx);
	if (newDot > maxDot){
			maxDot = newDot;
			supVec = vtx;
	}
    vtx = btVector3(this->p2->x,this->p2->y,this->p2->z);
	newDot = vec0.dot(vtx);
	if (newDot > maxDot){
			maxDot = newDot;
			supVec = vtx;
	}
    vtx = btVector3(this->p3->x,this->p3->y,this->p3->z);
	newDot = vec0.dot(vtx);
	if (newDot > maxDot){
			maxDot = newDot;
			supVec = vtx;
	}

	return supVec - vec0.normalized()*this->sphereswept_rad;
}

 void	btCEtriangleShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const
{
	printf("NOT SUPPORTED!! \n");
}


void	btCEtriangleShape::calculateLocalInertia(btScalar mass,btVector3& inertia) const
{
	//***TO DO***
	//as an approximation, take the inertia of an average radius sphere

	btTransform ident;
	ident.setIdentity();

	btVector3 halfExtents;
    double radius= ChMax((*p2-*p1).Length(), (*p3-*p1).Length());
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


void btCEtriangleShape::getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
{
	
	btVector3 p1_w = t.getOrigin()+ t.getBasis()*btVector3(this->p1->x, this->p1->y, this->p1->z);
    btVector3 p2_w = t.getOrigin()+ t.getBasis()*btVector3(this->p2->x, this->p2->y, this->p2->z);
    btVector3 p3_w = t.getOrigin()+ t.getBasis()*btVector3(this->p3->x, this->p3->y, this->p3->z);

	btVector3 vmargin (getMargin(),getMargin(),getMargin());
    btVector3 vsphereswept (this->sphereswept_rad,this->sphereswept_rad,this->sphereswept_rad);

	aabbMin = btVector3(ChMin(ChMin(p1_w.x(),p2_w.x()),p3_w.x()),
                        ChMin(ChMin(p1_w.y(),p2_w.y()),p3_w.y()),
                        ChMin(ChMin(p1_w.z(),p2_w.z()),p3_w.z())) - vmargin - vsphereswept;

    aabbMax = btVector3(ChMax(ChMax(p1_w.x(),p2_w.x()),p3_w.x()),
                        ChMax(ChMax(p1_w.y(),p2_w.y()),p3_w.y()),
                        ChMax(ChMax(p1_w.z(),p2_w.z()),p3_w.z())) + vmargin + vsphereswept;
}


