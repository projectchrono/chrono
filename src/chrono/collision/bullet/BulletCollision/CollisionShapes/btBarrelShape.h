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

#ifndef BT_BARREL_SHAPE_H
#define BT_BARREL_SHAPE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


///btBarrelShape represents a barrel-like shape (with vertical axis)
///made by lathing an arc of an ellipse around the vertical Y axis.
///The center of the ellipse is on Y=0 level, and it is ofsetted by R_offset from 
///the Y axis in radial direction. The two radii of the ellipse are R_vert (for the 
///vertical direction, i.e. the axis parellel to Y) and R_hor (for the axis that
///is perpendicular to Y). Also, the solid is clamped with two discs on the top and
///the bottom, at levels Y_low and Y_high.
class btBarrelShape : public btConvexInternalShape
{
private:
	btScalar Y_low;
	btScalar Y_high; 
	btScalar R_vert; 
	btScalar R_hor; 
	btScalar R_offset;

public:
	btBarrelShape(btScalar sY_low, btScalar sY_high, btScalar sR_vert, btScalar sR_hor, btScalar sR_offset );

	///CollisionShape Interface
	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const;

	/// btConvexShape Interface
	virtual btVector3	localGetSupportingVertexWithoutMargin(const btVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const;
	
	//virtual int	getShapeType() const { return BARREL_SHAPE_PROXYTYPE; }

	virtual const char*	getName()const 
	{
		return "BarrelShape";
	}

	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const;

	btScalar	getY_low() const {return Y_low;}
	btScalar	getY_high() const {return Y_high;}
	btScalar	getR_vert() const {return R_vert;}
	btScalar	getR_hor() const {return R_hor;}
	btScalar	getR_offset() const {return R_offset;}


};



#endif //BT_CAPSULE_SHAPE_H
