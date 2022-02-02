/*
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

#ifndef BT_SPHERE_TRIANGLE_DETECTOR_H
#define BT_SPHERE_TRIANGLE_DETECTOR_H

#include "BulletCollision/NarrowPhaseCollision/cbtDiscreteCollisionDetectorInterface.h"

class cbtSphereShape;
class cbtTriangleShape;

/// sphere-triangle to match the cbtDiscreteCollisionDetectorInterface
struct SphereTriangleDetector : public cbtDiscreteCollisionDetectorInterface
{
	virtual void getClosestPoints(const ClosestPointInput& input, Result& output, class cbtIDebugDraw* debugDraw, bool swapResults = false);

	SphereTriangleDetector(cbtSphereShape* sphere, cbtTriangleShape* triangle, cbtScalar contactBreakingThreshold);

	virtual ~SphereTriangleDetector(){};

	bool collide(const cbtVector3& sphereCenter, cbtVector3& point, cbtVector3& resultNormal, cbtScalar& depth, cbtScalar& timeOfImpact, cbtScalar contactBreakingThreshold);

private:
	bool pointInTriangle(const cbtVector3 vertices[], const cbtVector3& normal, cbtVector3* p);
	bool facecontains(const cbtVector3& p, const cbtVector3* vertices, cbtVector3& normal);

	cbtSphereShape* m_sphere;
	cbtTriangleShape* m_triangle;
	cbtScalar m_contactBreakingThreshold;
};
#endif  //BT_SPHERE_TRIANGLE_DETECTOR_H
