/*! \file cbtGImpactConvexDecompositionShape.h
\author Francisco León Nájera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef GIMPACT_CONVEX_DECOMPOSITION_SHAPE_H
#define GIMPACT_CONVEX_DECOMPOSITION_SHAPE_H


#include "chrono/collision/gimpact/GIMPACT/Bullet/cbtGImpactShape.h" // box tree class



//! This class creates a decomposition from a trimesh.
/*!

*/
class cbtGImpactConvexDecompositionShape	: public cbtGImpactCompoundShape
{
protected:
	cbtAlignedObjectArray<cbtGImpactMeshShapePart::TrimeshPrimitiveManager> m_trimeshInterfaces;

	class GIM_ConvexDecomposition*	m_decomposition;

	void buildConvexDecomposition(bool transformSubShapes);
public:

	cbtGImpactConvexDecompositionShape(
			cbtStridingMeshInterface * meshInterface,
			const cbtVector3 & mesh_scale,
			cbtScalar margin = cbtScalar(0.01),bool children_has_transform = true)
			:cbtGImpactCompoundShape(children_has_transform)
	{

		m_collisionMargin = margin;

		cbtGImpactMeshShapePart::TrimeshPrimitiveManager triInterface;
		triInterface.m_meshInterface = meshInterface;
		triInterface.m_scale = mesh_scale;
		triInterface.m_margin = cbtScalar(1.0);

		//add parts
		int part_count = meshInterface->getNumSubParts();
		for (int i=0;i< part_count;i++ )
		{
			triInterface.m_part = i;
			m_trimeshInterfaces.push_back(triInterface);
		}

		m_decomposition = 0;

		buildConvexDecomposition(children_has_transform);
	}

	virtual ~cbtGImpactConvexDecompositionShape();

	SIMD_FORCE_INLINE cbtGImpactMeshShapePart::TrimeshPrimitiveManager * getTrimeshInterface(int part)
	{
		return &m_trimeshInterfaces[part];
	}

	virtual void processAllTriangles(cbtTriangleCallback* callback,const cbtVector3& aabbMin,const cbtVector3& aabbMax) const;

};




#endif //GIMPACT_MESH_SHAPE_H
