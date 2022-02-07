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

#ifndef BT_VORONOI_SIMPLEX_SOLVER_H
#define BT_VORONOI_SIMPLEX_SOLVER_H

#include "cbtSimplexSolverInterface.h"

#define VORONOI_SIMPLEX_MAX_VERTS 5

///disable next define, or use defaultCollisionConfiguration->getSimplexSolver()->setEqualVertexThreshold(0.f) to disable/configure
#define BT_USE_EQUAL_VERTEX_THRESHOLD

#ifdef BT_USE_DOUBLE_PRECISION
#define VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD 1e-12f
#else
#define VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD 0.0001f
#endif  //BT_USE_DOUBLE_PRECISION

struct cbtUsageBitfield
{
	cbtUsageBitfield()
	{
		reset();
	}

	void reset()
	{
		usedVertexA = false;
		usedVertexB = false;
		usedVertexC = false;
		usedVertexD = false;
	}
	unsigned short usedVertexA : 1;
	unsigned short usedVertexB : 1;
	unsigned short usedVertexC : 1;
	unsigned short usedVertexD : 1;
	unsigned short unused1 : 1;
	unsigned short unused2 : 1;
	unsigned short unused3 : 1;
	unsigned short unused4 : 1;
};

struct cbtSubSimplexClosestResult
{
	cbtVector3 m_closestPointOnSimplex;
	//MASK for m_usedVertices
	//stores the simplex vertex-usage, using the MASK,
	// if m_usedVertices & MASK then the related vertex is used
	cbtUsageBitfield m_usedVertices;
	cbtScalar m_barycentricCoords[4];
	bool m_degenerate;

	void reset()
	{
		m_degenerate = false;
		setBarycentricCoordinates();
		m_usedVertices.reset();
	}
	bool isValid()
	{
		bool valid = (m_barycentricCoords[0] >= cbtScalar(0.)) &&
					 (m_barycentricCoords[1] >= cbtScalar(0.)) &&
					 (m_barycentricCoords[2] >= cbtScalar(0.)) &&
					 (m_barycentricCoords[3] >= cbtScalar(0.));

		return valid;
	}
	void setBarycentricCoordinates(cbtScalar a = cbtScalar(0.), cbtScalar b = cbtScalar(0.), cbtScalar c = cbtScalar(0.), cbtScalar d = cbtScalar(0.))
	{
		m_barycentricCoords[0] = a;
		m_barycentricCoords[1] = b;
		m_barycentricCoords[2] = c;
		m_barycentricCoords[3] = d;
	}
};

/// cbtVoronoiSimplexSolver is an implementation of the closest point distance algorithm from a 1-4 points simplex to the origin.
/// Can be used with GJK, as an alternative to Johnson distance algorithm.
#ifdef NO_VIRTUAL_INTERFACE
ATTRIBUTE_ALIGNED16(class)
cbtVoronoiSimplexSolver
#else
ATTRIBUTE_ALIGNED16(class)
cbtVoronoiSimplexSolver : public cbtSimplexSolverInterface
#endif
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	int m_numVertices;

	cbtVector3 m_simplexVectorW[VORONOI_SIMPLEX_MAX_VERTS];
	cbtVector3 m_simplexPointsP[VORONOI_SIMPLEX_MAX_VERTS];
	cbtVector3 m_simplexPointsQ[VORONOI_SIMPLEX_MAX_VERTS];

	cbtVector3 m_cachedP1;
	cbtVector3 m_cachedP2;
	cbtVector3 m_cachedV;
	cbtVector3 m_lastW;

	cbtScalar m_equalVertexThreshold;
	bool m_cachedValidClosest;

	cbtSubSimplexClosestResult m_cachedBC;

	bool m_needsUpdate;

	void removeVertex(int index);
	void reduceVertices(const cbtUsageBitfield& usedVerts);
	bool updateClosestVectorAndPoints();

	bool closestPtPointTetrahedron(const cbtVector3& p, const cbtVector3& a, const cbtVector3& b, const cbtVector3& c, const cbtVector3& d, cbtSubSimplexClosestResult& finalResult);
	int pointOutsideOfPlane(const cbtVector3& p, const cbtVector3& a, const cbtVector3& b, const cbtVector3& c, const cbtVector3& d);
	bool closestPtPointTriangle(const cbtVector3& p, const cbtVector3& a, const cbtVector3& b, const cbtVector3& c, cbtSubSimplexClosestResult& result);

public:
	cbtVoronoiSimplexSolver()
		: m_equalVertexThreshold(VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD)
	{
	}
	void reset();

	void addVertex(const cbtVector3& w, const cbtVector3& p, const cbtVector3& q);

	void setEqualVertexThreshold(cbtScalar threshold)
	{
		m_equalVertexThreshold = threshold;
	}

	cbtScalar getEqualVertexThreshold() const
	{
		return m_equalVertexThreshold;
	}

	bool closest(cbtVector3 & v);

	cbtScalar maxVertex();

	bool fullSimplex() const
	{
		return (m_numVertices == 4);
	}

	int getSimplex(cbtVector3 * pBuf, cbtVector3 * qBuf, cbtVector3 * yBuf) const;

	bool inSimplex(const cbtVector3& w);

	void backup_closest(cbtVector3 & v);

	bool emptySimplex() const;

	void compute_points(cbtVector3 & p1, cbtVector3 & p2);

	int numVertices() const
	{
		return m_numVertices;
	}
};

#endif  //BT_VORONOI_SIMPLEX_SOLVER_H
