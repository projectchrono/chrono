#ifndef BT_SDF_COLLISION_SHAPE_H
#define BT_SDF_COLLISION_SHAPE_H

#include "cbtConcaveShape.h"

class cbtSdfCollisionShape : public cbtConcaveShape
{
	struct cbtSdfCollisionShapeInternalData* m_data;

public:
	cbtSdfCollisionShape();
	virtual ~cbtSdfCollisionShape();

	bool initializeSDF(const char* sdfData, int sizeInBytes);

	virtual void getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const;
	virtual void setLocalScaling(const cbtVector3& scaling);
	virtual const cbtVector3& getLocalScaling() const;
	virtual void calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const;
	virtual const char* getName() const;
	virtual void setMargin(cbtScalar margin);
	virtual cbtScalar getMargin() const;

	virtual void processAllTriangles(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const;

	bool queryPoint(const cbtVector3& ptInSDF, cbtScalar& distOut, cbtVector3& normal);
};

#endif  //BT_SDF_COLLISION_SHAPE_H
