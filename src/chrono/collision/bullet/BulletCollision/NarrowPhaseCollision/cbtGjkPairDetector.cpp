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

#include "cbtGjkPairDetector.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/cbtSimplexSolverInterface.h"
#include "BulletCollision/NarrowPhaseCollision/cbtConvexPenetrationDepthSolver.h"

#if defined(DEBUG) || defined(_DEBUG)
//#define TEST_NON_VIRTUAL 1
#include <stdio.h>  //for debug printf
#ifdef __SPU__
#include <spu_printf.h>
#define printf spu_printf
#endif  //__SPU__
#endif

//must be above the machine epsilon
#ifdef BT_USE_DOUBLE_PRECISION
#define REL_ERROR2 cbtScalar(1.0e-12)
cbtScalar gGjkEpaPenetrationTolerance = 1.0e-12;
#else
#define REL_ERROR2 cbtScalar(1.0e-6)
cbtScalar gGjkEpaPenetrationTolerance = 0.001;
#endif


cbtGjkPairDetector::cbtGjkPairDetector(const cbtConvexShape *objectA, const cbtConvexShape *objectB, cbtSimplexSolverInterface *simplexSolver, cbtConvexPenetrationDepthSolver *penetrationDepthSolver)
	: m_cachedSeparatingAxis(cbtScalar(0.), cbtScalar(1.), cbtScalar(0.)),
	  m_penetrationDepthSolver(penetrationDepthSolver),
	  m_simplexSolver(simplexSolver),
	  m_minkowskiA(objectA),
	  m_minkowskiB(objectB),
	  m_shapeTypeA(objectA->getShapeType()),
	  m_shapeTypeB(objectB->getShapeType()),
	  m_marginA(objectA->getMargin()),
	  m_marginB(objectB->getMargin()),
	  m_ignoreMargin(false),
	  m_lastUsedMethod(-1),
	  m_catchDegeneracies(1),
	  m_fixContactNormalDirection(1)
{
}
cbtGjkPairDetector::cbtGjkPairDetector(const cbtConvexShape *objectA, const cbtConvexShape *objectB, int shapeTypeA, int shapeTypeB, cbtScalar marginA, cbtScalar marginB, cbtSimplexSolverInterface *simplexSolver, cbtConvexPenetrationDepthSolver *penetrationDepthSolver)
	: m_cachedSeparatingAxis(cbtScalar(0.), cbtScalar(1.), cbtScalar(0.)),
	  m_penetrationDepthSolver(penetrationDepthSolver),
	  m_simplexSolver(simplexSolver),
	  m_minkowskiA(objectA),
	  m_minkowskiB(objectB),
	  m_shapeTypeA(shapeTypeA),
	  m_shapeTypeB(shapeTypeB),
	  m_marginA(marginA),
	  m_marginB(marginB),
	  m_ignoreMargin(false),
	  m_lastUsedMethod(-1),
	  m_catchDegeneracies(1),
	  m_fixContactNormalDirection(1)
{
}

void cbtGjkPairDetector::getClosestPoints(const ClosestPointInput &input, Result &output, class cbtIDebugDraw *debugDraw, bool swapResults)
{
	(void)swapResults;

	getClosestPointsNonVirtual(input, output, debugDraw);
}

static void cbtComputeSupport(const cbtConvexShape *convexA, const cbtTransform &localTransA, const cbtConvexShape *convexB, const cbtTransform &localTransB, const cbtVector3 &dir, bool check2d, cbtVector3 &supAworld, cbtVector3 &supBworld, cbtVector3 &aMinb)
{
	cbtVector3 seperatingAxisInA = (dir)*localTransA.getBasis();
	cbtVector3 seperatingAxisInB = (-dir) * localTransB.getBasis();

	cbtVector3 pInANoMargin = convexA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
	cbtVector3 qInBNoMargin = convexB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);

	cbtVector3 pInA = pInANoMargin;
	cbtVector3 qInB = qInBNoMargin;

	supAworld = localTransA(pInA);
	supBworld = localTransB(qInB);

	if (check2d)
	{
		supAworld[2] = 0.f;
		supBworld[2] = 0.f;
	}

	aMinb = supAworld - supBworld;
}

struct cbtSupportVector
{
	cbtVector3 v;   //!< Support point in minkowski sum
	cbtVector3 v1;  //!< Support point in obj1
	cbtVector3 v2;  //!< Support point in obj2
};

struct cbtSimplex
{
	cbtSupportVector ps[4];
	int last;  //!< index of last added point
};

static cbtVector3 ccd_vec3_origin(0, 0, 0);

inline void cbtSimplexInit(cbtSimplex *s)
{
	s->last = -1;
}

inline int cbtSimplexSize(const cbtSimplex *s)
{
	return s->last + 1;
}

inline const cbtSupportVector *cbtSimplexPoint(const cbtSimplex *s, int idx)
{
	// here is no check on boundaries
	return &s->ps[idx];
}
inline void cbtSupportCopy(cbtSupportVector *d, const cbtSupportVector *s)
{
	*d = *s;
}

inline void cbtVec3Copy(cbtVector3 *v, const cbtVector3 *w)
{
	*v = *w;
}

inline void ccdVec3Add(cbtVector3 *v, const cbtVector3 *w)
{
	v->m_floats[0] += w->m_floats[0];
	v->m_floats[1] += w->m_floats[1];
	v->m_floats[2] += w->m_floats[2];
}

inline void ccdVec3Sub(cbtVector3 *v, const cbtVector3 *w)
{
	*v -= *w;
}
inline void cbtVec3Sub2(cbtVector3 *d, const cbtVector3 *v, const cbtVector3 *w)
{
	*d = (*v) - (*w);
}
inline cbtScalar cbtVec3Dot(const cbtVector3 *a, const cbtVector3 *b)
{
	cbtScalar dot;
	dot = a->dot(*b);

	return dot;
}

inline cbtScalar ccdVec3Dist2(const cbtVector3 *a, const cbtVector3 *b)
{
	cbtVector3 ab;
	cbtVec3Sub2(&ab, a, b);
	return cbtVec3Dot(&ab, &ab);
}

inline void cbtVec3Scale(cbtVector3 *d, cbtScalar k)
{
	d->m_floats[0] *= k;
	d->m_floats[1] *= k;
	d->m_floats[2] *= k;
}

inline void cbtVec3Cross(cbtVector3 *d, const cbtVector3 *a, const cbtVector3 *b)
{
	d->m_floats[0] = (a->m_floats[1] * b->m_floats[2]) - (a->m_floats[2] * b->m_floats[1]);
	d->m_floats[1] = (a->m_floats[2] * b->m_floats[0]) - (a->m_floats[0] * b->m_floats[2]);
	d->m_floats[2] = (a->m_floats[0] * b->m_floats[1]) - (a->m_floats[1] * b->m_floats[0]);
}

inline void cbtTripleCross(const cbtVector3 *a, const cbtVector3 *b,
						  const cbtVector3 *c, cbtVector3 *d)
{
	cbtVector3 e;
	cbtVec3Cross(&e, a, b);
	cbtVec3Cross(d, &e, c);
}

inline int ccdEq(cbtScalar _a, cbtScalar _b)
{
	cbtScalar ab;
	cbtScalar a, b;

	ab = cbtFabs(_a - _b);
	if (cbtFabs(ab) < SIMD_EPSILON)
		return 1;

	a = cbtFabs(_a);
	b = cbtFabs(_b);
	if (b > a)
	{
		return ab < SIMD_EPSILON * b;
	}
	else
	{
		return ab < SIMD_EPSILON * a;
	}
}

cbtScalar ccdVec3X(const cbtVector3 *v)
{
	return v->x();
}

cbtScalar ccdVec3Y(const cbtVector3 *v)
{
	return v->y();
}

cbtScalar ccdVec3Z(const cbtVector3 *v)
{
	return v->z();
}
inline int cbtVec3Eq(const cbtVector3 *a, const cbtVector3 *b)
{
	return ccdEq(ccdVec3X(a), ccdVec3X(b)) && ccdEq(ccdVec3Y(a), ccdVec3Y(b)) && ccdEq(ccdVec3Z(a), ccdVec3Z(b));
}

inline void cbtSimplexAdd(cbtSimplex *s, const cbtSupportVector *v)
{
	// here is no check on boundaries in sake of speed
	++s->last;
	cbtSupportCopy(s->ps + s->last, v);
}

inline void cbtSimplexSet(cbtSimplex *s, size_t pos, const cbtSupportVector *a)
{
	cbtSupportCopy(s->ps + pos, a);
}

inline void cbtSimplexSetSize(cbtSimplex *s, int size)
{
	s->last = size - 1;
}

inline const cbtSupportVector *ccdSimplexLast(const cbtSimplex *s)
{
	return cbtSimplexPoint(s, s->last);
}

inline int ccdSign(cbtScalar val)
{
	if (cbtFuzzyZero(val))
	{
		return 0;
	}
	else if (val < cbtScalar(0))
	{
		return -1;
	}
	return 1;
}

inline cbtScalar cbtVec3PointSegmentDist2(const cbtVector3 *P,
										const cbtVector3 *x0,
										const cbtVector3 *b,
										cbtVector3 *witness)
{
	// The computation comes from solving equation of segment:
	//      S(t) = x0 + t.d
	//          where - x0 is initial point of segment
	//                - d is direction of segment from x0 (|d| > 0)
	//                - t belongs to <0, 1> interval
	//
	// Than, distance from a segment to some point P can be expressed:
	//      D(t) = |x0 + t.d - P|^2
	//          which is distance from any point on segment. Minimization
	//          of this function brings distance from P to segment.
	// Minimization of D(t) leads to simple quadratic equation that's
	// solving is straightforward.
	//
	// Bonus of this method is witness point for free.

	cbtScalar dist, t;
	cbtVector3 d, a;

	// direction of segment
	cbtVec3Sub2(&d, b, x0);

	// precompute vector from P to x0
	cbtVec3Sub2(&a, x0, P);

	t = -cbtScalar(1.) * cbtVec3Dot(&a, &d);
	t /= cbtVec3Dot(&d, &d);

	if (t < cbtScalar(0) || cbtFuzzyZero(t))
	{
		dist = ccdVec3Dist2(x0, P);
		if (witness)
			cbtVec3Copy(witness, x0);
	}
	else if (t > cbtScalar(1) || ccdEq(t, cbtScalar(1)))
	{
		dist = ccdVec3Dist2(b, P);
		if (witness)
			cbtVec3Copy(witness, b);
	}
	else
	{
		if (witness)
		{
			cbtVec3Copy(witness, &d);
			cbtVec3Scale(witness, t);
			ccdVec3Add(witness, x0);
			dist = ccdVec3Dist2(witness, P);
		}
		else
		{
			// recycling variables
			cbtVec3Scale(&d, t);
			ccdVec3Add(&d, &a);
			dist = cbtVec3Dot(&d, &d);
		}
	}

	return dist;
}

cbtScalar cbtVec3PointTriDist2(const cbtVector3 *P,
							 const cbtVector3 *x0, const cbtVector3 *B,
							 const cbtVector3 *C,
							 cbtVector3 *witness)
{
	// Computation comes from analytic expression for triangle (x0, B, C)
	//      T(s, t) = x0 + s.d1 + t.d2, where d1 = B - x0 and d2 = C - x0 and
	// Then equation for distance is:
	//      D(s, t) = | T(s, t) - P |^2
	// This leads to minimization of quadratic function of two variables.
	// The solution from is taken only if s is between 0 and 1, t is
	// between 0 and 1 and t + s < 1, otherwise distance from segment is
	// computed.

	cbtVector3 d1, d2, a;
	double u, v, w, p, q, r;
	double s, t, dist, dist2;
	cbtVector3 witness2;

	cbtVec3Sub2(&d1, B, x0);
	cbtVec3Sub2(&d2, C, x0);
	cbtVec3Sub2(&a, x0, P);

	u = cbtVec3Dot(&a, &a);
	v = cbtVec3Dot(&d1, &d1);
	w = cbtVec3Dot(&d2, &d2);
	p = cbtVec3Dot(&a, &d1);
	q = cbtVec3Dot(&a, &d2);
	r = cbtVec3Dot(&d1, &d2);

	s = (q * r - w * p) / (w * v - r * r);
	t = (-s * r - q) / w;

	if ((cbtFuzzyZero(s) || s > cbtScalar(0)) && (ccdEq(s, cbtScalar(1)) || s < cbtScalar(1)) && (cbtFuzzyZero(t) || t > cbtScalar(0)) && (ccdEq(t, cbtScalar(1)) || t < cbtScalar(1)) && (ccdEq(t + s, cbtScalar(1)) || t + s < cbtScalar(1)))
	{
		if (witness)
		{
			cbtVec3Scale(&d1, s);
			cbtVec3Scale(&d2, t);
			cbtVec3Copy(witness, x0);
			ccdVec3Add(witness, &d1);
			ccdVec3Add(witness, &d2);

			dist = ccdVec3Dist2(witness, P);
		}
		else
		{
			dist = s * s * v;
			dist += t * t * w;
			dist += cbtScalar(2.) * s * t * r;
			dist += cbtScalar(2.) * s * p;
			dist += cbtScalar(2.) * t * q;
			dist += u;
		}
	}
	else
	{
		dist = cbtVec3PointSegmentDist2(P, x0, B, witness);

		dist2 = cbtVec3PointSegmentDist2(P, x0, C, &witness2);
		if (dist2 < dist)
		{
			dist = dist2;
			if (witness)
				cbtVec3Copy(witness, &witness2);
		}

		dist2 = cbtVec3PointSegmentDist2(P, B, C, &witness2);
		if (dist2 < dist)
		{
			dist = dist2;
			if (witness)
				cbtVec3Copy(witness, &witness2);
		}
	}

	return dist;
}

static int cbtDoSimplex2(cbtSimplex *simplex, cbtVector3 *dir)
{
	const cbtSupportVector *A, *B;
	cbtVector3 AB, AO, tmp;
	cbtScalar dot;

	// get last added as A
	A = ccdSimplexLast(simplex);
	// get the other point
	B = cbtSimplexPoint(simplex, 0);
	// compute AB oriented segment
	cbtVec3Sub2(&AB, &B->v, &A->v);
	// compute AO vector
	cbtVec3Copy(&AO, &A->v);
	cbtVec3Scale(&AO, -cbtScalar(1));

	// dot product AB . AO
	dot = cbtVec3Dot(&AB, &AO);

	// check if origin doesn't lie on AB segment
	cbtVec3Cross(&tmp, &AB, &AO);
	if (cbtFuzzyZero(cbtVec3Dot(&tmp, &tmp)) && dot > cbtScalar(0))
	{
		return 1;
	}

	// check if origin is in area where AB segment is
	if (cbtFuzzyZero(dot) || dot < cbtScalar(0))
	{
		// origin is in outside are of A
		cbtSimplexSet(simplex, 0, A);
		cbtSimplexSetSize(simplex, 1);
		cbtVec3Copy(dir, &AO);
	}
	else
	{
		// origin is in area where AB segment is

		// keep simplex untouched and set direction to
		// AB x AO x AB
		cbtTripleCross(&AB, &AO, &AB, dir);
	}

	return 0;
}

static int cbtDoSimplex3(cbtSimplex *simplex, cbtVector3 *dir)
{
	const cbtSupportVector *A, *B, *C;
	cbtVector3 AO, AB, AC, ABC, tmp;
	cbtScalar dot, dist;

	// get last added as A
	A = ccdSimplexLast(simplex);
	// get the other points
	B = cbtSimplexPoint(simplex, 1);
	C = cbtSimplexPoint(simplex, 0);

	// check touching contact
	dist = cbtVec3PointTriDist2(&ccd_vec3_origin, &A->v, &B->v, &C->v, 0);
	if (cbtFuzzyZero(dist))
	{
		return 1;
	}

	// check if triangle is really triangle (has area > 0)
	// if not simplex can't be expanded and thus no itersection is found
	if (cbtVec3Eq(&A->v, &B->v) || cbtVec3Eq(&A->v, &C->v))
	{
		return -1;
	}

	// compute AO vector
	cbtVec3Copy(&AO, &A->v);
	cbtVec3Scale(&AO, -cbtScalar(1));

	// compute AB and AC segments and ABC vector (perpendircular to triangle)
	cbtVec3Sub2(&AB, &B->v, &A->v);
	cbtVec3Sub2(&AC, &C->v, &A->v);
	cbtVec3Cross(&ABC, &AB, &AC);

	cbtVec3Cross(&tmp, &ABC, &AC);
	dot = cbtVec3Dot(&tmp, &AO);
	if (cbtFuzzyZero(dot) || dot > cbtScalar(0))
	{
		dot = cbtVec3Dot(&AC, &AO);
		if (cbtFuzzyZero(dot) || dot > cbtScalar(0))
		{
			// C is already in place
			cbtSimplexSet(simplex, 1, A);
			cbtSimplexSetSize(simplex, 2);
			cbtTripleCross(&AC, &AO, &AC, dir);
		}
		else
		{
			dot = cbtVec3Dot(&AB, &AO);
			if (cbtFuzzyZero(dot) || dot > cbtScalar(0))
			{
				cbtSimplexSet(simplex, 0, B);
				cbtSimplexSet(simplex, 1, A);
				cbtSimplexSetSize(simplex, 2);
				cbtTripleCross(&AB, &AO, &AB, dir);
			}
			else
			{
				cbtSimplexSet(simplex, 0, A);
				cbtSimplexSetSize(simplex, 1);
				cbtVec3Copy(dir, &AO);
			}
		}
	}
	else
	{
		cbtVec3Cross(&tmp, &AB, &ABC);
		dot = cbtVec3Dot(&tmp, &AO);
		if (cbtFuzzyZero(dot) || dot > cbtScalar(0))
		{
			dot = cbtVec3Dot(&AB, &AO);
			if (cbtFuzzyZero(dot) || dot > cbtScalar(0))
			{
				cbtSimplexSet(simplex, 0, B);
				cbtSimplexSet(simplex, 1, A);
				cbtSimplexSetSize(simplex, 2);
				cbtTripleCross(&AB, &AO, &AB, dir);
			}
			else
			{
				cbtSimplexSet(simplex, 0, A);
				cbtSimplexSetSize(simplex, 1);
				cbtVec3Copy(dir, &AO);
			}
		}
		else
		{
			dot = cbtVec3Dot(&ABC, &AO);
			if (cbtFuzzyZero(dot) || dot > cbtScalar(0))
			{
				cbtVec3Copy(dir, &ABC);
			}
			else
			{
				cbtSupportVector tmp;
				cbtSupportCopy(&tmp, C);
				cbtSimplexSet(simplex, 0, B);
				cbtSimplexSet(simplex, 1, &tmp);

				cbtVec3Copy(dir, &ABC);
				cbtVec3Scale(dir, -cbtScalar(1));
			}
		}
	}

	return 0;
}

static int cbtDoSimplex4(cbtSimplex *simplex, cbtVector3 *dir)
{
	const cbtSupportVector *A, *B, *C, *D;
	cbtVector3 AO, AB, AC, AD, ABC, ACD, ADB;
	int B_on_ACD, C_on_ADB, D_on_ABC;
	int AB_O, AC_O, AD_O;
	cbtScalar dist;

	// get last added as A
	A = ccdSimplexLast(simplex);
	// get the other points
	B = cbtSimplexPoint(simplex, 2);
	C = cbtSimplexPoint(simplex, 1);
	D = cbtSimplexPoint(simplex, 0);

	// check if tetrahedron is really tetrahedron (has volume > 0)
	// if it is not simplex can't be expanded and thus no intersection is
	// found
	dist = cbtVec3PointTriDist2(&A->v, &B->v, &C->v, &D->v, 0);
	if (cbtFuzzyZero(dist))
	{
		return -1;
	}

	// check if origin lies on some of tetrahedron's face - if so objects
	// intersect
	dist = cbtVec3PointTriDist2(&ccd_vec3_origin, &A->v, &B->v, &C->v, 0);
	if (cbtFuzzyZero(dist))
		return 1;
	dist = cbtVec3PointTriDist2(&ccd_vec3_origin, &A->v, &C->v, &D->v, 0);
	if (cbtFuzzyZero(dist))
		return 1;
	dist = cbtVec3PointTriDist2(&ccd_vec3_origin, &A->v, &B->v, &D->v, 0);
	if (cbtFuzzyZero(dist))
		return 1;
	dist = cbtVec3PointTriDist2(&ccd_vec3_origin, &B->v, &C->v, &D->v, 0);
	if (cbtFuzzyZero(dist))
		return 1;

	// compute AO, AB, AC, AD segments and ABC, ACD, ADB normal vectors
	cbtVec3Copy(&AO, &A->v);
	cbtVec3Scale(&AO, -cbtScalar(1));
	cbtVec3Sub2(&AB, &B->v, &A->v);
	cbtVec3Sub2(&AC, &C->v, &A->v);
	cbtVec3Sub2(&AD, &D->v, &A->v);
	cbtVec3Cross(&ABC, &AB, &AC);
	cbtVec3Cross(&ACD, &AC, &AD);
	cbtVec3Cross(&ADB, &AD, &AB);

	// side (positive or negative) of B, C, D relative to planes ACD, ADB
	// and ABC respectively
	B_on_ACD = ccdSign(cbtVec3Dot(&ACD, &AB));
	C_on_ADB = ccdSign(cbtVec3Dot(&ADB, &AC));
	D_on_ABC = ccdSign(cbtVec3Dot(&ABC, &AD));

	// whether origin is on same side of ACD, ADB, ABC as B, C, D
	// respectively
	AB_O = ccdSign(cbtVec3Dot(&ACD, &AO)) == B_on_ACD;
	AC_O = ccdSign(cbtVec3Dot(&ADB, &AO)) == C_on_ADB;
	AD_O = ccdSign(cbtVec3Dot(&ABC, &AO)) == D_on_ABC;

	if (AB_O && AC_O && AD_O)
	{
		// origin is in tetrahedron
		return 1;
		// rearrange simplex to triangle and call cbtDoSimplex3()
	}
	else if (!AB_O)
	{
		// B is farthest from the origin among all of the tetrahedron's
		// points, so remove it from the list and go on with the triangle
		// case

		// D and C are in place
		cbtSimplexSet(simplex, 2, A);
		cbtSimplexSetSize(simplex, 3);
	}
	else if (!AC_O)
	{
		// C is farthest
		cbtSimplexSet(simplex, 1, D);
		cbtSimplexSet(simplex, 0, B);
		cbtSimplexSet(simplex, 2, A);
		cbtSimplexSetSize(simplex, 3);
	}
	else
	{  // (!AD_O)
		cbtSimplexSet(simplex, 0, C);
		cbtSimplexSet(simplex, 1, B);
		cbtSimplexSet(simplex, 2, A);
		cbtSimplexSetSize(simplex, 3);
	}

	return cbtDoSimplex3(simplex, dir);
}

static int cbtDoSimplex(cbtSimplex *simplex, cbtVector3 *dir)
{
	if (cbtSimplexSize(simplex) == 2)
	{
		// simplex contains segment only one segment
		return cbtDoSimplex2(simplex, dir);
	}
	else if (cbtSimplexSize(simplex) == 3)
	{
		// simplex contains triangle
		return cbtDoSimplex3(simplex, dir);
	}
	else
	{  // cbtSimplexSize(simplex) == 4
		// tetrahedron - this is the only shape which can encapsule origin
		// so cbtDoSimplex4() also contains test on it
		return cbtDoSimplex4(simplex, dir);
	}
}

#ifdef __SPU__
void cbtGjkPairDetector::getClosestPointsNonVirtual(const ClosestPointInput &input, Result &output, class cbtIDebugDraw *debugDraw)
#else
void cbtGjkPairDetector::getClosestPointsNonVirtual(const ClosestPointInput &input, Result &output, class cbtIDebugDraw *debugDraw)
#endif
{
	m_cachedSeparatingDistance = 0.f;

	cbtScalar distance = cbtScalar(0.);
	cbtVector3 normalInB(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));

	cbtVector3 pointOnA, pointOnB;
	cbtTransform localTransA = input.m_transformA;
	cbtTransform localTransB = input.m_transformB;
	cbtVector3 positionOffset = (localTransA.getOrigin() + localTransB.getOrigin()) * cbtScalar(0.5);
	localTransA.getOrigin() -= positionOffset;
	localTransB.getOrigin() -= positionOffset;

	bool check2d = m_minkowskiA->isConvex2d() && m_minkowskiB->isConvex2d();

	cbtScalar marginA = m_marginA;
	cbtScalar marginB = m_marginB;


	//for CCD we don't use margins
	if (m_ignoreMargin)
	{
		marginA = cbtScalar(0.);
		marginB = cbtScalar(0.);
	}

	m_curIter = 0;
	int gGjkMaxIter = 1000;  //this is to catch invalid input, perhaps check for #NaN?
	m_cachedSeparatingAxis.setValue(0, 1, 0);

	bool isValid = false;
	bool checkSimplex = false;
	bool checkPenetration = true;
	m_degenerateSimplex = 0;

	m_lastUsedMethod = -1;
	int status = -2;
	cbtVector3 orgNormalInB(0, 0, 0);
	cbtScalar margin = marginA + marginB;

	//we add a separate implementation to check if the convex shapes intersect
	//See also "Real-time Collision Detection with Implicit Objects" by Leif Olvang
	//Todo: integrate the simplex penetration check directly inside the Bullet cbtVoronoiSimplexSolver
	//and remove this temporary code from libCCD
	//this fixes issue https://github.com/bulletphysics/bullet3/issues/1703
	//note, for large differences in shapes, use double precision build!
	{
		cbtScalar squaredDistance = BT_LARGE_FLOAT;
		cbtScalar delta = cbtScalar(0.);

		cbtSimplex simplex1;
		cbtSimplex *simplex = &simplex1;
		cbtSimplexInit(simplex);

		cbtVector3 dir(1, 0, 0);

		{
			cbtVector3 lastSupV;
			cbtVector3 supAworld;
			cbtVector3 supBworld;
			cbtComputeSupport(m_minkowskiA, localTransA, m_minkowskiB, localTransB, dir, check2d, supAworld, supBworld, lastSupV);

			cbtSupportVector last;
			last.v = lastSupV;
			last.v1 = supAworld;
			last.v2 = supBworld;

			cbtSimplexAdd(simplex, &last);

			dir = -lastSupV;

			// start iterations
			for (int iterations = 0; iterations < gGjkMaxIter; iterations++)
			{
				// obtain support point
				cbtComputeSupport(m_minkowskiA, localTransA, m_minkowskiB, localTransB, dir, check2d, supAworld, supBworld, lastSupV);

				// check if farthest point in Minkowski difference in direction dir
				// isn't somewhere before origin (the test on negative dot product)
				// - because if it is, objects are not intersecting at all.
				cbtScalar delta = lastSupV.dot(dir);
				if (delta < 0)
				{
					//no intersection, besides margin
					status = -1;
					break;
				}

				// add last support vector to simplex
				last.v = lastSupV;
				last.v1 = supAworld;
				last.v2 = supBworld;

				cbtSimplexAdd(simplex, &last);

				// if cbtDoSimplex returns 1 if objects intersect, -1 if objects don't
				// intersect and 0 if algorithm should continue

				cbtVector3 newDir;
				int do_simplex_res = cbtDoSimplex(simplex, &dir);

				if (do_simplex_res == 1)
				{
					status = 0;  // intersection found
					break;
				}
				else if (do_simplex_res == -1)
				{
					// intersection not found
					status = -1;
					break;
				}

				if (cbtFuzzyZero(cbtVec3Dot(&dir, &dir)))
				{
					// intersection not found
					status = -1;
				}

				if (dir.length2() < SIMD_EPSILON)
				{
					//no intersection, besides margin
					status = -1;
					break;
				}

				if (dir.fuzzyZero())
				{
					// intersection not found
					status = -1;
					break;
				}
			}
		}

		m_simplexSolver->reset();
		if (status == 0)
		{
			//status = 0;
			//printf("Intersect!\n");
		}

		if (status == -1)
		{
			//printf("not intersect\n");
		}
		//printf("dir=%f,%f,%f\n",dir[0],dir[1],dir[2]);
		if (1)
		{
			for (;;)
			//while (true)
			{
				cbtVector3 seperatingAxisInA = (-m_cachedSeparatingAxis) * localTransA.getBasis();
				cbtVector3 seperatingAxisInB = m_cachedSeparatingAxis * localTransB.getBasis();

				cbtVector3 pInA = m_minkowskiA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
				cbtVector3 qInB = m_minkowskiB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);

				cbtVector3 pWorld = localTransA(pInA);
				cbtVector3 qWorld = localTransB(qInB);

				if (check2d)
				{
					pWorld[2] = 0.f;
					qWorld[2] = 0.f;
				}

				cbtVector3 w = pWorld - qWorld;
				delta = m_cachedSeparatingAxis.dot(w);

				// potential exit, they don't overlap
				if ((delta > cbtScalar(0.0)) && (delta * delta > squaredDistance * input.m_maximumDistanceSquared))
				{
					m_degenerateSimplex = 10;
					checkSimplex = true;
					//checkPenetration = false;
					break;
				}

				//exit 0: the new point is already in the simplex, or we didn't come any closer
				if (m_simplexSolver->inSimplex(w))
				{
					m_degenerateSimplex = 1;
					checkSimplex = true;
					break;
				}
				// are we getting any closer ?
				cbtScalar f0 = squaredDistance - delta;
				cbtScalar f1 = squaredDistance * REL_ERROR2;

				if (f0 <= f1)
				{
					if (f0 <= cbtScalar(0.))
					{
						m_degenerateSimplex = 2;
					}
					else
					{
						m_degenerateSimplex = 11;
					}
					checkSimplex = true;
					break;
				}

				//add current vertex to simplex
				m_simplexSolver->addVertex(w, pWorld, qWorld);
				cbtVector3 newCachedSeparatingAxis;

				//calculate the closest point to the origin (update vector v)
				if (!m_simplexSolver->closest(newCachedSeparatingAxis))
				{
					m_degenerateSimplex = 3;
					checkSimplex = true;
					break;
				}

				if (newCachedSeparatingAxis.length2() < REL_ERROR2)
				{
					m_cachedSeparatingAxis = newCachedSeparatingAxis;
					m_degenerateSimplex = 6;
					checkSimplex = true;
					break;
				}

				cbtScalar previousSquaredDistance = squaredDistance;
				squaredDistance = newCachedSeparatingAxis.length2();
#if 0
				///warning: this termination condition leads to some problems in 2d test case see Bullet/Demos/Box2dDemo
				if (squaredDistance > previousSquaredDistance)
				{
					m_degenerateSimplex = 7;
					squaredDistance = previousSquaredDistance;
					checkSimplex = false;
					break;
				}
#endif  //

				//redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

				//are we getting any closer ?
				if (previousSquaredDistance - squaredDistance <= SIMD_EPSILON * previousSquaredDistance)
				{
					//				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
					checkSimplex = true;
					m_degenerateSimplex = 12;

					break;
				}

				m_cachedSeparatingAxis = newCachedSeparatingAxis;

				//degeneracy, this is typically due to invalid/uninitialized worldtransforms for a cbtCollisionObject
				if (m_curIter++ > gGjkMaxIter)
				{
#if defined(DEBUG) || defined(_DEBUG)

					printf("cbtGjkPairDetector maxIter exceeded:%i\n", m_curIter);
					printf("sepAxis=(%f,%f,%f), squaredDistance = %f, shapeTypeA=%i,shapeTypeB=%i\n",
						   m_cachedSeparatingAxis.getX(),
						   m_cachedSeparatingAxis.getY(),
						   m_cachedSeparatingAxis.getZ(),
						   squaredDistance,
						   m_minkowskiA->getShapeType(),
						   m_minkowskiB->getShapeType());

#endif
					break;
				}

				bool check = (!m_simplexSolver->fullSimplex());
				//bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver->maxVertex());

				if (!check)
				{
					//do we need this backup_closest here ?
					//				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
					m_degenerateSimplex = 13;
					break;
				}
			}

			if (checkSimplex)
			{
				m_simplexSolver->compute_points(pointOnA, pointOnB);
				normalInB = m_cachedSeparatingAxis;

				cbtScalar lenSqr = m_cachedSeparatingAxis.length2();

				//valid normal
				if (lenSqr < REL_ERROR2)
				{
					m_degenerateSimplex = 5;
				}
				if (lenSqr > SIMD_EPSILON * SIMD_EPSILON)
				{
					cbtScalar rlen = cbtScalar(1.) / cbtSqrt(lenSqr);
					normalInB *= rlen;  //normalize

					cbtScalar s = cbtSqrt(squaredDistance);

					cbtAssert(s > cbtScalar(0.0));
					pointOnA -= m_cachedSeparatingAxis * (marginA / s);
					pointOnB += m_cachedSeparatingAxis * (marginB / s);
					distance = ((cbtScalar(1.) / rlen) - margin);
					isValid = true;
					orgNormalInB = normalInB;

					m_lastUsedMethod = 1;
				}
				else
				{
					m_lastUsedMethod = 2;
				}
			}
		}

		bool catchDegeneratePenetrationCase =
			(m_catchDegeneracies && m_penetrationDepthSolver && m_degenerateSimplex && ((distance + margin) < gGjkEpaPenetrationTolerance));

		//if (checkPenetration && !isValid)
		if ((checkPenetration && (!isValid || catchDegeneratePenetrationCase)) || (status == 0))
		{
			//penetration case

			//if there is no way to handle penetrations, bail out
			if (m_penetrationDepthSolver)
			{
				// Penetration depth case.
				cbtVector3 tmpPointOnA, tmpPointOnB;

				m_cachedSeparatingAxis.setZero();

				bool isValid2 = m_penetrationDepthSolver->calcPenDepth(
					*m_simplexSolver,
					m_minkowskiA, m_minkowskiB,
					localTransA, localTransB,
					m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB,
					debugDraw);

				if (m_cachedSeparatingAxis.length2())
				{
					if (isValid2)
					{
						cbtVector3 tmpNormalInB = tmpPointOnB - tmpPointOnA;
						cbtScalar lenSqr = tmpNormalInB.length2();
						if (lenSqr <= (SIMD_EPSILON * SIMD_EPSILON))
						{
							tmpNormalInB = m_cachedSeparatingAxis;
							lenSqr = m_cachedSeparatingAxis.length2();
						}

						if (lenSqr > (SIMD_EPSILON * SIMD_EPSILON))
						{
							tmpNormalInB /= cbtSqrt(lenSqr);
							cbtScalar distance2 = -(tmpPointOnA - tmpPointOnB).length();
							m_lastUsedMethod = 3;
							//only replace valid penetrations when the result is deeper (check)
							if (!isValid || (distance2 < distance))
							{
								distance = distance2;
								pointOnA = tmpPointOnA;
								pointOnB = tmpPointOnB;
								normalInB = tmpNormalInB;
								isValid = true;
							}
							else
							{
								m_lastUsedMethod = 8;
							}
						}
						else
						{
							m_lastUsedMethod = 9;
						}
					}
					else

					{
						///this is another degenerate case, where the initial GJK calculation reports a degenerate case
						///EPA reports no penetration, and the second GJK (using the supporting vector without margin)
						///reports a valid positive distance. Use the results of the second GJK instead of failing.
						///thanks to Jacob.Langford for the reproduction case
						///http://code.google.com/p/bullet/issues/detail?id=250

						if (m_cachedSeparatingAxis.length2() > cbtScalar(0.))
						{
							cbtScalar distance2 = (tmpPointOnA - tmpPointOnB).length() - margin;
							//only replace valid distances when the distance is less
							if (!isValid || (distance2 < distance))
							{
								distance = distance2;
								pointOnA = tmpPointOnA;
								pointOnB = tmpPointOnB;
								pointOnA -= m_cachedSeparatingAxis * marginA;
								pointOnB += m_cachedSeparatingAxis * marginB;
								normalInB = m_cachedSeparatingAxis;
								normalInB.normalize();

								isValid = true;
								m_lastUsedMethod = 6;
							}
							else
							{
								m_lastUsedMethod = 5;
							}
						}
					}
				}
				else
				{
					//printf("EPA didn't return a valid value\n");
				}
			}
		}
	}

	if (isValid && ((distance < 0) || (distance * distance < input.m_maximumDistanceSquared)))
	{
		m_cachedSeparatingAxis = normalInB;
		m_cachedSeparatingDistance = distance;
		if (1)
		{
			///todo: need to track down this EPA penetration solver degeneracy
			///the penetration solver reports penetration but the contact normal
			///connecting the contact points is pointing in the opposite direction
			///until then, detect the issue and revert the normal

			cbtScalar d2 = 0.f;
			{
				cbtVector3 seperatingAxisInA = (-orgNormalInB) * localTransA.getBasis();
				cbtVector3 seperatingAxisInB = orgNormalInB * localTransB.getBasis();

				cbtVector3 pInA = m_minkowskiA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
				cbtVector3 qInB = m_minkowskiB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);

				cbtVector3 pWorld = localTransA(pInA);
				cbtVector3 qWorld = localTransB(qInB);
				cbtVector3 w = pWorld - qWorld;
				d2 = orgNormalInB.dot(w) - margin;
			}

			cbtScalar d1 = 0;
			{
				cbtVector3 seperatingAxisInA = (normalInB)*localTransA.getBasis();
				cbtVector3 seperatingAxisInB = -normalInB * localTransB.getBasis();

				cbtVector3 pInA = m_minkowskiA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
				cbtVector3 qInB = m_minkowskiB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);

				cbtVector3 pWorld = localTransA(pInA);
				cbtVector3 qWorld = localTransB(qInB);
				cbtVector3 w = pWorld - qWorld;
				d1 = (-normalInB).dot(w) - margin;
			}
			cbtScalar d0 = 0.f;
			{
				cbtVector3 seperatingAxisInA = (-normalInB) * input.m_transformA.getBasis();
				cbtVector3 seperatingAxisInB = normalInB * input.m_transformB.getBasis();

				cbtVector3 pInA = m_minkowskiA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
				cbtVector3 qInB = m_minkowskiB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);

				cbtVector3 pWorld = localTransA(pInA);
				cbtVector3 qWorld = localTransB(qInB);
				cbtVector3 w = pWorld - qWorld;
				d0 = normalInB.dot(w) - margin;
			}

			if (d1 > d0)
			{
				m_lastUsedMethod = 10;
				normalInB *= -1;
			}

			if (orgNormalInB.length2())
			{
				if (d2 > d0 && d2 > d1 && d2 > distance)
				{
					normalInB = orgNormalInB;
					distance = d2;
				}
			}
		}

		output.addContactPoint(
			normalInB,
			pointOnB + positionOffset,
			distance);
	}
	else
	{
		//printf("invalid gjk query\n");
	}
}
