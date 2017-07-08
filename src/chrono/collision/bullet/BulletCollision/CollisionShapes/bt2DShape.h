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

#ifndef BT_2D_SHAPE_H
#define BT_2D_SHAPE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


/// bt2DarcShape represents an arc. This is meant to interact only 
/// with other 2D shapes such as bt2DarcShape or bt2DsegmentShape, for
/// "flat" collision shapes created as polylines on the same plane.
/// The 'clockwise' info tell if the arc is clockwise (=convex, inner part is solid)
/// or counterclockwise (=concave, outer part is solid, inner is hollow) because
/// the solid part is always 'to the right' of the increasing curvilinear abscyssa

class bt2DarcShape : public btConvexInternalShape
{
private:
	btScalar x;
	btScalar y;
	btScalar radius; 
	btScalar angle1; 
	btScalar angle2;
    bool     counterclock;
    btScalar zthickness;

public:
	bt2DarcShape(   btScalar mx, 
                    btScalar my, 
                    btScalar mradius, 
                    btScalar mangle1, 
                    btScalar mangle2, 
                    bool mcounterclock = false, 
                    btScalar mzthickness= 0.001);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const;

	/// btConvexShape Interface
	virtual btVector3	localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;

	virtual const char*	getName()const 
	{
		return "ArcShape";
	}

	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

	btScalar	get_X() const {return x;}
	btScalar	get_Y() const {return y;}
	btScalar	get_radius() const {return radius;}
	btScalar	get_angle1() const {return angle1;}
    btScalar	get_angle2() const {return angle2;}
	bool     	get_counterclock() const {return counterclock;}
    btScalar    get_zthickness() const {return zthickness;}


};


/// bt2DsegmentShape represents a segment. This is meant to interact only 
/// with other 2D shapes such as bt2DarcShape or bt2DsegmentShape, for
/// "flat" collision shapes created as polylines on the same plane.
/// The 'solid' part is on the right side when following the path from P1 to P2.

class bt2DsegmentShape : public btConvexInternalShape
{
private:
	btVector3 P1;
	btVector3 P2;
    btScalar zthickness;

public:
	bt2DsegmentShape(   const btVector3& mP1, 
                        const btVector3& mP2,
                        const btScalar mzthickness = 0.001);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const;

	/// btConvexShape Interface
	virtual btVector3	localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;

	virtual const char*	getName()const 
	{
		return "SegmentShape";
	}

	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

	btVector3	get_P1() const {return P1;}
	btVector3	get_P2() const {return P2;}
    btScalar    get_zthickness() const {return zthickness;}
};


////////////////////***TEST***


#include "btSphereShape.h"

/// Class for point-like nodes. 
/// These are like null radii spheres, but node-node collision is never processed at all.

class btPointShape : public btSphereShape {
public:
    btPointShape(btScalar mrad) 
        : btSphereShape(mrad) 
    { m_shapeType = POINT_SHAPE_PROXYTYPE; };

    virtual const char*	getName()const 
	{
		return "PointShape";
	}
};




#endif 
