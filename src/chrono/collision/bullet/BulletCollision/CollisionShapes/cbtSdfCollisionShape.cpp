#include "cbtSdfCollisionShape.h"
#include "cbtMiniSDF.h"
#include "LinearMath/cbtAabbUtil2.h"

struct cbtSdfCollisionShapeInternalData
{
	cbtVector3 m_localScaling;
	cbtScalar m_margin;
	cbtMiniSDF m_sdf;

	cbtSdfCollisionShapeInternalData()
		: m_localScaling(1, 1, 1),
		  m_margin(0)
	{
	}
};

bool cbtSdfCollisionShape::initializeSDF(const char* sdfData, int sizeInBytes)
{
	bool valid = m_data->m_sdf.load(sdfData, sizeInBytes);
	return valid;
}
cbtSdfCollisionShape::cbtSdfCollisionShape()
{
	m_shapeType = SDF_SHAPE_PROXYTYPE;
	m_data = new cbtSdfCollisionShapeInternalData();

	//"E:/develop/bullet3/data/toys/ground_hole64_64_8.cdf");//ground_cube.cdf");
	/*unsigned int field_id=0;
	Eigen::Vector3d x (1,10,1);
	Eigen::Vector3d gradient;
	double dist = m_data->m_sdf.interpolate(field_id, x, &gradient);
	printf("dist=%g\n", dist);
	*/
}
cbtSdfCollisionShape::~cbtSdfCollisionShape()
{
	delete m_data;
}

void cbtSdfCollisionShape::getAabb(const cbtTransform& t, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	cbtAssert(m_data->m_sdf.isValid());
	cbtVector3 localAabbMin = m_data->m_sdf.m_domain.m_min;
	cbtVector3 localAabbMax = m_data->m_sdf.m_domain.m_max;
	cbtScalar margin(0);
	cbtTransformAabb(localAabbMin, localAabbMax, margin, t, aabbMin, aabbMax);
}

void cbtSdfCollisionShape::setLocalScaling(const cbtVector3& scaling)
{
	m_data->m_localScaling = scaling;
}
const cbtVector3& cbtSdfCollisionShape::getLocalScaling() const
{
	return m_data->m_localScaling;
}
void cbtSdfCollisionShape::calculateLocalInertia(cbtScalar mass, cbtVector3& inertia) const
{
	inertia.setValue(0, 0, 0);
}
const char* cbtSdfCollisionShape::getName() const
{
	return "cbtSdfCollisionShape";
}
void cbtSdfCollisionShape::setMargin(cbtScalar margin)
{
	m_data->m_margin = margin;
}
cbtScalar cbtSdfCollisionShape::getMargin() const
{
	return m_data->m_margin;
}

void cbtSdfCollisionShape::processAllTriangles(cbtTriangleCallback* callback, const cbtVector3& aabbMin, const cbtVector3& aabbMax) const
{
	//not yet
}

bool cbtSdfCollisionShape::queryPoint(const cbtVector3& ptInSDF, cbtScalar& distOut, cbtVector3& normal)
{
	int field = 0;
	cbtVector3 grad;
	double dist;
	bool hasResult = m_data->m_sdf.interpolate(field, dist, ptInSDF, &grad);
	if (hasResult)
	{
		normal.setValue(grad[0], grad[1], grad[2]);
		distOut = dist;
	}
	return hasResult;
}
