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

#ifndef BT_BARREL_SHAPE_H
#define BT_BARREL_SHAPE_H

#include "cbtConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/cbtBroadphaseProxy.h" // for the types


///cbtBarrelShape represents a barrel-like shape (with vertical axis)
///made by lathing an arc of an ellipse around the vertical Y axis.
///The center of the ellipse is on Y=0 level, and it is ofsetted by R_offset from 
///the Y axis in radial direction. The two radii of the ellipse are R_vert (for the 
///vertical direction, i.e. the axis parellel to Y) and R_hor (for the axis that
///is perpendicular to Y). Also, the solid is clamped with two discs on the top and
///the bottom, at levels Y_low and Y_high.
class cbtBarrelShape : public cbtConvexInternalShape
{
private:
	cbtScalar Y_low;
	cbtScalar Y_high; 
	cbtScalar R_vert; 
	cbtScalar R_hor; 
	cbtScalar R_offset;

public:
	cbtBarrelShape(cbtScalar sY_low, cbtScalar sY_high, cbtScalar sR_vert, cbtScalar sR_hor, cbtScalar sR_offset );

	///CollisionShape Interface
	virtual void	calculateLocalInertia(cbtScalar mass,cbtVector3& inertia) const;

	/// cbtConvexShape Interface
	virtual cbtVector3	localGetSupportingVertexWithoutMargin(const cbtVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const cbtVector3* vectors,cbtVector3* supportVerticesOut,int numVectors) const;
	
	//virtual int	getShapeType() const { return BARREL_SHAPE_PROXYTYPE; }

	virtual const char*	getName()const 
	{
		return "BarrelShape";
	}

	virtual void getAabb(const cbtTransform& t,cbtVector3& aabbMin,cbtVector3& aabbMax) const;

	cbtScalar	getY_low() const {return Y_low;}
	cbtScalar	getY_high() const {return Y_high;}
	cbtScalar	getR_vert() const {return R_vert;}
	cbtScalar	getR_hor() const {return R_hor;}
	cbtScalar	getR_offset() const {return R_offset;}


};



#endif //BT_CAPSULE_SHAPE_H
