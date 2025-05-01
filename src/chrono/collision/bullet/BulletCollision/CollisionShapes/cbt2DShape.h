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

#ifndef BT_2D_SHAPE_H
#define BT_2D_SHAPE_H

#include "cbtConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h" // for the types


/// cbt2DarcShape represents an arc. This is meant to interact only 
/// with other 2D shapes such as cbt2DarcShape or cbt2DsegmentShape, for
/// "flat" collision shapes created as polylines on the same plane.
/// The 'clockwise' info tell if the arc is clockwise (=convex, inner part is solid)
/// or counterclockwise (=concave, outer part is solid, inner is hollow) because
/// the solid part is always 'to the right' of the increasing curvilinear abscyssa

class cbt2DarcShape : public cbtConvexInternalShape
{
private:
	cbtScalar x;
	cbtScalar y;
	cbtScalar radius; 
	cbtScalar angle1; 
	cbtScalar angle2;
    bool     counterclock;
    cbtScalar zthickness;

public:
	cbt2DarcShape(   cbtScalar mx, 
                    cbtScalar my, 
                    cbtScalar mradius, 
                    cbtScalar mangle1, 
                    cbtScalar mangle2, 
                    bool mcounterclock = false, 
                    cbtScalar mzthickness= 0.001);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(cbtScalar mass,cbtVector3& inertia) const;

	/// cbtConvexShape Interface
	virtual cbtVector3	localGetSupportingVertexWithoutMargin(const cbtVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,cbtVector3* supportVerticesOut,int numVectors) const;

	virtual const char*	getName()const 
	{
		return "ArcShape";
	}

	virtual void getAabb(const cbtTransform& t,cbtVector3& aabbMin,cbtVector3& aabbMax) const;

	cbtScalar	get_X() const {return x;}
	cbtScalar	get_Y() const {return y;}
	cbtScalar	get_radius() const {return radius;}
	cbtScalar	get_angle1() const {return angle1;}
    cbtScalar	get_angle2() const {return angle2;}
	bool     	get_counterclock() const {return counterclock;}
    cbtScalar    get_zthickness() const {return zthickness;}


};


/// cbt2DsegmentShape represents a segment. This is meant to interact only 
/// with other 2D shapes such as cbt2DarcShape or cbt2DsegmentShape, for
/// "flat" collision shapes created as polylines on the same plane.
/// The 'solid' part is on the right side when following the path from P1 to P2.

class cbt2DsegmentShape : public cbtConvexInternalShape
{
private:
	cbtVector3 P1;
	cbtVector3 P2;
    cbtScalar zthickness;

public:
	cbt2DsegmentShape(   const cbtVector3& mP1, 
                        const cbtVector3& mP2,
                        const cbtScalar mzthickness = 0.001);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(cbtScalar mass,cbtVector3& inertia) const;

	/// cbtConvexShape Interface
	virtual cbtVector3	localGetSupportingVertexWithoutMargin(const cbtVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,cbtVector3* supportVerticesOut,int numVectors) const;

	virtual const char*	getName()const 
	{
		return "SegmentShape";
	}

	virtual void getAabb(const cbtTransform& t,cbtVector3& aabbMin,cbtVector3& aabbMax) const;

	cbtVector3	get_P1() const {return P1;}
	cbtVector3	get_P2() const {return P2;}
    cbtScalar    get_zthickness() const {return zthickness;}
};

#endif 
