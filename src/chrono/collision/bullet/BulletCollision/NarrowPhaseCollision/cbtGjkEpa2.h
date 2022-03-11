/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
GJK-EPA collision solver by Nathanael Presson, 2008
*/
#ifndef BT_GJK_EPA2_H
#define BT_GJK_EPA2_H

#include "BulletCollision/CollisionShapes/cbtConvexShape.h"

///cbtGjkEpaSolver contributed under zlib by Nathanael Presson
struct cbtGjkEpaSolver2
{
	struct sResults
	{
		enum eStatus
		{
			Separated,   /* Shapes doesnt penetrate												*/
			Penetrating, /* Shapes are penetrating												*/
			GJK_Failed,  /* GJK phase fail, no big issue, shapes are probably just 'touching'	*/
			EPA_Failed   /* EPA phase fail, bigger problem, need to save parameters, and debug	*/
		} status;
		cbtVector3 witnesses[2];
		cbtVector3 normal;
		cbtScalar distance;
	};

	static int StackSizeRequirement();

	static bool Distance(const cbtConvexShape* shape0, const cbtTransform& wtrs0,
						 const cbtConvexShape* shape1, const cbtTransform& wtrs1,
						 const cbtVector3& guess,
						 sResults& results);

	static bool Penetration(const cbtConvexShape* shape0, const cbtTransform& wtrs0,
							const cbtConvexShape* shape1, const cbtTransform& wtrs1,
							const cbtVector3& guess,
							sResults& results,
							bool usemargins = true);
#ifndef __SPU__
	static cbtScalar SignedDistance(const cbtVector3& position,
								   cbtScalar margin,
								   const cbtConvexShape* shape,
								   const cbtTransform& wtrs,
								   sResults& results);

	static bool SignedDistance(const cbtConvexShape* shape0, const cbtTransform& wtrs0,
							   const cbtConvexShape* shape1, const cbtTransform& wtrs1,
							   const cbtVector3& guess,
							   sResults& results);
#endif  //__SPU__
};

#endif  //BT_GJK_EPA2_H
