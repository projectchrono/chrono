/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///This file was written by Erwin Coumans

#ifndef BT_POLYHEDRAL_CONTACT_CLIPPING_H
#define BT_POLYHEDRAL_CONTACT_CLIPPING_H

#include "LinearMath/cbtAlignedObjectArray.h"
#include "LinearMath/cbtTransform.h"
#include "cbtDiscreteCollisionDetectorInterface.h"

class cbtConvexPolyhedron;

typedef cbtAlignedObjectArray<cbtVector3> cbtVertexArray;

// Clips a face to the back of a plane
struct cbtPolyhedralContactClipping
{
	static void clipHullAgainstHull(const cbtVector3& separatingNormal1, const cbtConvexPolyhedron& hullA, const cbtConvexPolyhedron& hullB, const cbtTransform& transA, const cbtTransform& transB, const cbtScalar minDist, cbtScalar maxDist, cbtVertexArray& worldVertsB1, cbtVertexArray& worldVertsB2, cbtDiscreteCollisionDetectorInterface::Result& resultOut);

	static void clipFaceAgainstHull(const cbtVector3& separatingNormal, const cbtConvexPolyhedron& hullA, const cbtTransform& transA, cbtVertexArray& worldVertsB1, cbtVertexArray& worldVertsB2, const cbtScalar minDist, cbtScalar maxDist, cbtDiscreteCollisionDetectorInterface::Result& resultOut);

	static bool findSeparatingAxis(const cbtConvexPolyhedron& hullA, const cbtConvexPolyhedron& hullB, const cbtTransform& transA, const cbtTransform& transB, cbtVector3& sep, cbtDiscreteCollisionDetectorInterface::Result& resultOut);

	///the clipFace method is used internally
	static void clipFace(const cbtVertexArray& pVtxIn, cbtVertexArray& ppVtxOut, const cbtVector3& planeNormalWS, cbtScalar planeEqWS);
};

#endif  // BT_POLYHEDRAL_CONTACT_CLIPPING_H
