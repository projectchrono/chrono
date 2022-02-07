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

#include "cbtGImpactShape.h"
#include "cbtGImpactMassUtil.h"

cbtGImpactMeshShapePart::cbtGImpactMeshShapePart(cbtStridingMeshInterface* meshInterface, int part)
{
	// moved from .h to .cpp because of conditional compilation
	// (The setting of BT_THREADSAFE may differ between various cpp files, so it is best to
	// avoid using it in h files)
	m_primitive_manager.m_meshInterface = meshInterface;
	m_primitive_manager.m_part = part;
	m_box_set.setPrimitiveManager(&m_primitive_manager);
#if BT_THREADSAFE
	// If threadsafe is requested, this object uses a different lock/unlock
	//  model with the cbtStridingMeshInterface -- lock once when the object is constructed
	//  and unlock once in the destructor.
	// The other way of locking and unlocking for each collision check in the narrowphase
	// is not threadsafe.  Note these are not thread-locks, they are calls to the meshInterface's
	// getLockedReadOnlyVertexIndexBase virtual function, which by default just returns a couple of
	// pointers.  In theory a client could override the lock function to do all sorts of
	// things like reading data from GPU memory, or decompressing data on the fly, but such things
	// do not seem all that likely or useful, given the performance cost.
	m_primitive_manager.lock();
#endif
}

cbtGImpactMeshShapePart::~cbtGImpactMeshShapePart()
{
	// moved from .h to .cpp because of conditional compilation
#if BT_THREADSAFE
	m_primitive_manager.unlock();
#endif
}

void cbtGImpactMeshShapePart::lockChildShapes() const
{
	// moved from .h to .cpp because of conditional compilation
#if !BT_THREADSAFE
	// called in the narrowphase -- not threadsafe!
	void* dummy = (void*)(m_box_set.getPrimitiveManager());
	TrimeshPrimitiveManager* dummymanager = static_cast<TrimeshPrimitiveManager*>(dummy);
	dummymanager->lock();
#endif
}

void cbtGImpactMeshShapePart::unlockChildShapes() const
{
	// moved from .h to .cpp because of conditional compilation
#if !BT_THREADSAFE
	// called in the narrowphase -- not threadsafe!
	void* dummy = (void*)(m_box_set.getPrimitiveManager());
	TrimeshPrimitiveManager* dummymanager = static_cast<TrimeshPrimitiveManager*>(dummy);
	dummymanager->unlock();
#endif
}

#define CALC_EXACT_INERTIA 1

void cbtGImpactCompoundShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	lockChildShapes();
#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f, 0.f, 0.f);

	int i = this->getNumChildShapes();
	cbtScalar shapemass = mass / cbtScalar(i);

	while (i--)
	{
		cbtVector3 temp_inertia;
		m_childShapes[i]->calculateLocalInertia(shapemass, temp_inertia);
		if (childrenHasTransform())
		{
			inertia = gim_inertia_add_transformed(inertia, temp_inertia, m_childTransforms[i]);
		}
		else
		{
			inertia = gim_inertia_add_transformed(inertia, temp_inertia, cbtTransform::getIdentity());
		}
	}

#else

	// Calc box inertia

	cbtScalar lx = m_localAABB.m_max[0] - m_localAABB.m_min[0];
	cbtScalar ly = m_localAABB.m_max[1] - m_localAABB.m_min[1];
	cbtScalar lz = m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const cbtScalar x2 = lx * lx;
	const cbtScalar y2 = ly * ly;
	const cbtScalar z2 = lz * lz;
	const cbtScalar scaledmass = mass * cbtScalar(0.08333333);

	inertia = scaledmass * (cbtVector3(y2 + z2, x2 + z2, x2 + y2));

#endif
	unlockChildShapes();
}

void cbtGImpactMeshShapePart::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	lockChildShapes();

#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f, 0.f, 0.f);

	int i = this->getVertexCount();
	cbtScalar pointmass = mass / cbtScalar(i);

	while (i--)
	{
		cbtVector3 pointintertia;
		this->getVertex(i, pointintertia);
		pointintertia = gim_get_point_inertia(pointintertia, pointmass);
		inertia += pointintertia;
	}

#else

	// Calc box inertia

	cbtScalar lx = m_localAABB.m_max[0] - m_localAABB.m_min[0];
	cbtScalar ly = m_localAABB.m_max[1] - m_localAABB.m_min[1];
	cbtScalar lz = m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const cbtScalar x2 = lx * lx;
	const cbtScalar y2 = ly * ly;
	const cbtScalar z2 = lz * lz;
	const cbtScalar scaledmass = mass * cbtScalar(0.08333333);

	inertia = scaledmass * (cbtVector3(y2 + z2, x2 + z2, x2 + y2));

#endif

	unlockChildShapes();
}

void cbtGImpactMeshShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f, 0.f, 0.f);

	int i = this->getMeshPartCount();
	cbtScalar partmass = mass / cbtScalar(i);

	while (i--)
	{
		cbtVector3 partinertia;
		getMeshPart(i)->calculateLocalInertia(partmass, partinertia);
		inertia += partinertia;
	}

#else

	// Calc box inertia

	cbtScalar lx = m_localAABB.m_max[0] - m_localAABB.m_min[0];
	cbtScalar ly = m_localAABB.m_max[1] - m_localAABB.m_min[1];
	cbtScalar lz = m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const cbtScalar x2 = lx * lx;
	const cbtScalar y2 = ly * ly;
	const cbtScalar z2 = lz * lz;
	const cbtScalar scaledmass = mass * cbtScalar(0.08333333);

	inertia = scaledmass * (cbtVector3(y2 + z2, x2 + z2, x2 + y2));

#endif
}

void cbtGImpactMeshShape::rayTest(const cbtVector3& rayFrom, const cbtVector3& rayTo, cbtCollisionWorld::RayResultCallback& resultCallback) const
{
}

void cbtGImpactMeshShapePart::processAllTrianglesRay(cbtTriangleCallback* callback, const cbtVector3& rayFrom, const cbtVector3& rayTo) const
{
	lockChildShapes();

	cbtAlignedObjectArray<int> collided;
	cbtVector3 rayDir(rayTo - rayFrom);
	rayDir.normalize();
	m_box_set.rayQuery(rayDir, rayFrom, collided);

	if (collided.size() == 0)
	{
		unlockChildShapes();
		return;
	}

	int part = (int)getPart();
	cbtPrimitiveTriangle triangle;
	int i = collided.size();
	while (i--)
	{
		getPrimitiveTriangle(collided[i], triangle);
		callback->processTriangle(triangle.m_vertices, part, collided[i]);
	}
	unlockChildShapes();
}

void cbtGImpactMeshShapePart::processAllTriangles(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const
{
	lockChildShapes();
	cbtAABB box;
	box.m_min = aabbMin;
	box.m_max = aabbMax;

	cbtAlignedObjectArray<int> collided;
	m_box_set.boxQuery(box, collided);

	if (collided.size() == 0)
	{
		unlockChildShapes();
		return;
	}

	int part = (int)getPart();
	cbtPrimitiveTriangle triangle;
	int i = collided.size();
	while (i--)
	{
		this->getPrimitiveTriangle(collided[i], triangle);
		callback->processTriangle(triangle.m_vertices, part, collided[i]);
	}
	unlockChildShapes();
}

void cbtGImpactMeshShape::processAllTriangles(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const
{
	int i = m_mesh_parts.size();
	while (i--)
	{
		m_mesh_parts[i]->processAllTriangles(callback, aabbMin, aabbMax);
	}
}

void cbtGImpactMeshShape::processAllTrianglesRay(cbtTriangleCallback* callback, const cbtVector3& rayFrom, const cbtVector3& rayTo) const
{
	int i = m_mesh_parts.size();
	while (i--)
	{
		m_mesh_parts[i]->processAllTrianglesRay(callback, rayFrom, rayTo);
	}
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
const char* cbtGImpactMeshShape::serialize(void* dataBuffer, cbtSerializer* serializer) const
{
	cbtGImpactMeshShapeData* trimeshData = (cbtGImpactMeshShapeData*)dataBuffer;

	cbtCollisionShape::serialize(&trimeshData->m_collisionShapeData, serializer);

	m_meshInterface->serialize(&trimeshData->m_meshInterface, serializer);

	trimeshData->m_collisionMargin = float(m_collisionMargin);

	localScaling.serializeFloat(trimeshData->m_localScaling);

	trimeshData->m_gimpactSubType = int(getGImpactShapeType());

	return "cbtGImpactMeshShapeData";
}
