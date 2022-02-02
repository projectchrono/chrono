
/***
 * ---------------------------------
 * Copyright (c)2012 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file was ported from mpr.c file, part of libccd.
 *  The Minkoski Portal Refinement implementation was ported 
 *  to OpenCL by Erwin Coumans for the Bullet 3 Physics library.
 *  The original MPR idea and implementation is by Gary Snethen
 *  in XenoCollide, see http://github.com/erwincoumans/xenocollide
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see <http://www.opensource.org/licenses/bsd-license.php>.
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

///2014 Oct, Erwin Coumans, Use templates to avoid void* casts

#ifndef BT_MPR_PENETRATION_H
#define BT_MPR_PENETRATION_H

#define BT_DEBUG_MPR1

#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtAlignedObjectArray.h"

//#define MPR_AVERAGE_CONTACT_POSITIONS

struct cbtMprCollisionDescription
{
	cbtVector3 m_firstDir;
	int m_maxGjkIterations;
	cbtScalar m_maximumDistanceSquared;
	cbtScalar m_gjkRelError2;

	cbtMprCollisionDescription()
		: m_firstDir(0, 1, 0),
		  m_maxGjkIterations(1000),
		  m_maximumDistanceSquared(1e30f),
		  m_gjkRelError2(1.0e-6)
	{
	}
	virtual ~cbtMprCollisionDescription()
	{
	}
};

struct cbtMprDistanceInfo
{
	cbtVector3 m_pointOnA;
	cbtVector3 m_pointOnB;
	cbtVector3 m_normalBtoA;
	cbtScalar m_distance;
};

#ifdef __cplusplus
#define BT_MPR_SQRT sqrtf
#else
#define BT_MPR_SQRT sqrt
#endif
#define BT_MPR_FMIN(x, y) ((x) < (y) ? (x) : (y))
#define BT_MPR_FABS fabs

#define BT_MPR_TOLERANCE 1E-6f
#define BT_MPR_MAX_ITERATIONS 1000

struct _cbtMprSupport_t
{
	cbtVector3 v;   //!< Support point in minkowski sum
	cbtVector3 v1;  //!< Support point in obj1
	cbtVector3 v2;  //!< Support point in obj2
};
typedef struct _cbtMprSupport_t cbtMprSupport_t;

struct _cbtMprSimplex_t
{
	cbtMprSupport_t ps[4];
	int last;  //!< index of last added point
};
typedef struct _cbtMprSimplex_t cbtMprSimplex_t;

inline cbtMprSupport_t *cbtMprSimplexPointW(cbtMprSimplex_t *s, int idx)
{
	return &s->ps[idx];
}

inline void cbtMprSimplexSetSize(cbtMprSimplex_t *s, int size)
{
	s->last = size - 1;
}

#ifdef DEBUG_MPR
inline void cbtPrintPortalVertex(_cbtMprSimplex_t *portal, int index)
{
	printf("portal[%d].v = %f,%f,%f, v1=%f,%f,%f, v2=%f,%f,%f\n", index, portal->ps[index].v.x(), portal->ps[index].v.y(), portal->ps[index].v.z(),
		   portal->ps[index].v1.x(), portal->ps[index].v1.y(), portal->ps[index].v1.z(),
		   portal->ps[index].v2.x(), portal->ps[index].v2.y(), portal->ps[index].v2.z());
}
#endif  //DEBUG_MPR

inline int cbtMprSimplexSize(const cbtMprSimplex_t *s)
{
	return s->last + 1;
}

inline const cbtMprSupport_t *cbtMprSimplexPoint(const cbtMprSimplex_t *s, int idx)
{
	// here is no check on boundaries
	return &s->ps[idx];
}

inline void cbtMprSupportCopy(cbtMprSupport_t *d, const cbtMprSupport_t *s)
{
	*d = *s;
}

inline void cbtMprSimplexSet(cbtMprSimplex_t *s, size_t pos, const cbtMprSupport_t *a)
{
	cbtMprSupportCopy(s->ps + pos, a);
}

inline void cbtMprSimplexSwap(cbtMprSimplex_t *s, size_t pos1, size_t pos2)
{
	cbtMprSupport_t supp;

	cbtMprSupportCopy(&supp, &s->ps[pos1]);
	cbtMprSupportCopy(&s->ps[pos1], &s->ps[pos2]);
	cbtMprSupportCopy(&s->ps[pos2], &supp);
}

inline int cbtMprIsZero(float val)
{
	return BT_MPR_FABS(val) < FLT_EPSILON;
}

inline int cbtMprEq(float _a, float _b)
{
	float ab;
	float a, b;

	ab = BT_MPR_FABS(_a - _b);
	if (BT_MPR_FABS(ab) < FLT_EPSILON)
		return 1;

	a = BT_MPR_FABS(_a);
	b = BT_MPR_FABS(_b);
	if (b > a)
	{
		return ab < FLT_EPSILON * b;
	}
	else
	{
		return ab < FLT_EPSILON * a;
	}
}

inline int cbtMprVec3Eq(const cbtVector3 *a, const cbtVector3 *b)
{
	return cbtMprEq((*a).x(), (*b).x()) && cbtMprEq((*a).y(), (*b).y()) && cbtMprEq((*a).z(), (*b).z());
}

template <typename cbtConvexTemplate>
inline void cbtFindOrigin(const cbtConvexTemplate &a, const cbtConvexTemplate &b, const cbtMprCollisionDescription &colDesc, cbtMprSupport_t *center)
{
	center->v1 = a.getObjectCenterInWorld();
	center->v2 = b.getObjectCenterInWorld();
	center->v = center->v1 - center->v2;
}

inline void cbtMprVec3Set(cbtVector3 *v, float x, float y, float z)
{
	v->setValue(x, y, z);
}

inline void cbtMprVec3Add(cbtVector3 *v, const cbtVector3 *w)
{
	*v += *w;
}

inline void cbtMprVec3Copy(cbtVector3 *v, const cbtVector3 *w)
{
	*v = *w;
}

inline void cbtMprVec3Scale(cbtVector3 *d, float k)
{
	*d *= k;
}

inline float cbtMprVec3Dot(const cbtVector3 *a, const cbtVector3 *b)
{
	float dot;

	dot = cbtDot(*a, *b);
	return dot;
}

inline float cbtMprVec3Len2(const cbtVector3 *v)
{
	return cbtMprVec3Dot(v, v);
}

inline void cbtMprVec3Normalize(cbtVector3 *d)
{
	float k = 1.f / BT_MPR_SQRT(cbtMprVec3Len2(d));
	cbtMprVec3Scale(d, k);
}

inline void cbtMprVec3Cross(cbtVector3 *d, const cbtVector3 *a, const cbtVector3 *b)
{
	*d = cbtCross(*a, *b);
}

inline void cbtMprVec3Sub2(cbtVector3 *d, const cbtVector3 *v, const cbtVector3 *w)
{
	*d = *v - *w;
}

inline void cbtPortalDir(const cbtMprSimplex_t *portal, cbtVector3 *dir)
{
	cbtVector3 v2v1, v3v1;

	cbtMprVec3Sub2(&v2v1, &cbtMprSimplexPoint(portal, 2)->v,
				  &cbtMprSimplexPoint(portal, 1)->v);
	cbtMprVec3Sub2(&v3v1, &cbtMprSimplexPoint(portal, 3)->v,
				  &cbtMprSimplexPoint(portal, 1)->v);
	cbtMprVec3Cross(dir, &v2v1, &v3v1);
	cbtMprVec3Normalize(dir);
}

inline int portalEncapsulesOrigin(const cbtMprSimplex_t *portal,
								  const cbtVector3 *dir)
{
	float dot;
	dot = cbtMprVec3Dot(dir, &cbtMprSimplexPoint(portal, 1)->v);
	return cbtMprIsZero(dot) || dot > 0.f;
}

inline int portalReachTolerance(const cbtMprSimplex_t *portal,
								const cbtMprSupport_t *v4,
								const cbtVector3 *dir)
{
	float dv1, dv2, dv3, dv4;
	float dot1, dot2, dot3;

	// find the smallest dot product of dir and {v1-v4, v2-v4, v3-v4}

	dv1 = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 1)->v, dir);
	dv2 = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 2)->v, dir);
	dv3 = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 3)->v, dir);
	dv4 = cbtMprVec3Dot(&v4->v, dir);

	dot1 = dv4 - dv1;
	dot2 = dv4 - dv2;
	dot3 = dv4 - dv3;

	dot1 = BT_MPR_FMIN(dot1, dot2);
	dot1 = BT_MPR_FMIN(dot1, dot3);

	return cbtMprEq(dot1, BT_MPR_TOLERANCE) || dot1 < BT_MPR_TOLERANCE;
}

inline int portalCanEncapsuleOrigin(const cbtMprSimplex_t *portal,
									const cbtMprSupport_t *v4,
									const cbtVector3 *dir)
{
	float dot;
	dot = cbtMprVec3Dot(&v4->v, dir);
	return cbtMprIsZero(dot) || dot > 0.f;
}

inline void cbtExpandPortal(cbtMprSimplex_t *portal,
						   const cbtMprSupport_t *v4)
{
	float dot;
	cbtVector3 v4v0;

	cbtMprVec3Cross(&v4v0, &v4->v, &cbtMprSimplexPoint(portal, 0)->v);
	dot = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 1)->v, &v4v0);
	if (dot > 0.f)
	{
		dot = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 2)->v, &v4v0);
		if (dot > 0.f)
		{
			cbtMprSimplexSet(portal, 1, v4);
		}
		else
		{
			cbtMprSimplexSet(portal, 3, v4);
		}
	}
	else
	{
		dot = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 3)->v, &v4v0);
		if (dot > 0.f)
		{
			cbtMprSimplexSet(portal, 2, v4);
		}
		else
		{
			cbtMprSimplexSet(portal, 1, v4);
		}
	}
}
template <typename cbtConvexTemplate>
inline void cbtMprSupport(const cbtConvexTemplate &a, const cbtConvexTemplate &b,
						 const cbtMprCollisionDescription &colDesc,
						 const cbtVector3 &dir, cbtMprSupport_t *supp)
{
	cbtVector3 seperatingAxisInA = dir * a.getWorldTransform().getBasis();
	cbtVector3 seperatingAxisInB = -dir * b.getWorldTransform().getBasis();

	cbtVector3 pInA = a.getLocalSupportWithMargin(seperatingAxisInA);
	cbtVector3 qInB = b.getLocalSupportWithMargin(seperatingAxisInB);

	supp->v1 = a.getWorldTransform()(pInA);
	supp->v2 = b.getWorldTransform()(qInB);
	supp->v = supp->v1 - supp->v2;
}

template <typename cbtConvexTemplate>
static int cbtDiscoverPortal(const cbtConvexTemplate &a, const cbtConvexTemplate &b,
							const cbtMprCollisionDescription &colDesc,
							cbtMprSimplex_t *portal)
{
	cbtVector3 dir, va, vb;
	float dot;
	int cont;

	// vertex 0 is center of portal
	cbtFindOrigin(a, b, colDesc, cbtMprSimplexPointW(portal, 0));

	// vertex 0 is center of portal
	cbtMprSimplexSetSize(portal, 1);

	cbtVector3 zero = cbtVector3(0, 0, 0);
	cbtVector3 *org = &zero;

	if (cbtMprVec3Eq(&cbtMprSimplexPoint(portal, 0)->v, org))
	{
		// Portal's center lies on origin (0,0,0) => we know that objects
		// intersect but we would need to know penetration info.
		// So move center little bit...
		cbtMprVec3Set(&va, FLT_EPSILON * 10.f, 0.f, 0.f);
		cbtMprVec3Add(&cbtMprSimplexPointW(portal, 0)->v, &va);
	}

	// vertex 1 = support in direction of origin
	cbtMprVec3Copy(&dir, &cbtMprSimplexPoint(portal, 0)->v);
	cbtMprVec3Scale(&dir, -1.f);
	cbtMprVec3Normalize(&dir);

	cbtMprSupport(a, b, colDesc, dir, cbtMprSimplexPointW(portal, 1));

	cbtMprSimplexSetSize(portal, 2);

	// test if origin isn't outside of v1
	dot = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 1)->v, &dir);

	if (cbtMprIsZero(dot) || dot < 0.f)
		return -1;

	// vertex 2
	cbtMprVec3Cross(&dir, &cbtMprSimplexPoint(portal, 0)->v,
				   &cbtMprSimplexPoint(portal, 1)->v);
	if (cbtMprIsZero(cbtMprVec3Len2(&dir)))
	{
		if (cbtMprVec3Eq(&cbtMprSimplexPoint(portal, 1)->v, org))
		{
			// origin lies on v1
			return 1;
		}
		else
		{
			// origin lies on v0-v1 segment
			return 2;
		}
	}

	cbtMprVec3Normalize(&dir);
	cbtMprSupport(a, b, colDesc, dir, cbtMprSimplexPointW(portal, 2));

	dot = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 2)->v, &dir);
	if (cbtMprIsZero(dot) || dot < 0.f)
		return -1;

	cbtMprSimplexSetSize(portal, 3);

	// vertex 3 direction
	cbtMprVec3Sub2(&va, &cbtMprSimplexPoint(portal, 1)->v,
				  &cbtMprSimplexPoint(portal, 0)->v);
	cbtMprVec3Sub2(&vb, &cbtMprSimplexPoint(portal, 2)->v,
				  &cbtMprSimplexPoint(portal, 0)->v);
	cbtMprVec3Cross(&dir, &va, &vb);
	cbtMprVec3Normalize(&dir);

	// it is better to form portal faces to be oriented "outside" origin
	dot = cbtMprVec3Dot(&dir, &cbtMprSimplexPoint(portal, 0)->v);
	if (dot > 0.f)
	{
		cbtMprSimplexSwap(portal, 1, 2);
		cbtMprVec3Scale(&dir, -1.f);
	}

	while (cbtMprSimplexSize(portal) < 4)
	{
		cbtMprSupport(a, b, colDesc, dir, cbtMprSimplexPointW(portal, 3));

		dot = cbtMprVec3Dot(&cbtMprSimplexPoint(portal, 3)->v, &dir);
		if (cbtMprIsZero(dot) || dot < 0.f)
			return -1;

		cont = 0;

		// test if origin is outside (v1, v0, v3) - set v2 as v3 and
		// continue
		cbtMprVec3Cross(&va, &cbtMprSimplexPoint(portal, 1)->v,
					   &cbtMprSimplexPoint(portal, 3)->v);
		dot = cbtMprVec3Dot(&va, &cbtMprSimplexPoint(portal, 0)->v);
		if (dot < 0.f && !cbtMprIsZero(dot))
		{
			cbtMprSimplexSet(portal, 2, cbtMprSimplexPoint(portal, 3));
			cont = 1;
		}

		if (!cont)
		{
			// test if origin is outside (v3, v0, v2) - set v1 as v3 and
			// continue
			cbtMprVec3Cross(&va, &cbtMprSimplexPoint(portal, 3)->v,
						   &cbtMprSimplexPoint(portal, 2)->v);
			dot = cbtMprVec3Dot(&va, &cbtMprSimplexPoint(portal, 0)->v);
			if (dot < 0.f && !cbtMprIsZero(dot))
			{
				cbtMprSimplexSet(portal, 1, cbtMprSimplexPoint(portal, 3));
				cont = 1;
			}
		}

		if (cont)
		{
			cbtMprVec3Sub2(&va, &cbtMprSimplexPoint(portal, 1)->v,
						  &cbtMprSimplexPoint(portal, 0)->v);
			cbtMprVec3Sub2(&vb, &cbtMprSimplexPoint(portal, 2)->v,
						  &cbtMprSimplexPoint(portal, 0)->v);
			cbtMprVec3Cross(&dir, &va, &vb);
			cbtMprVec3Normalize(&dir);
		}
		else
		{
			cbtMprSimplexSetSize(portal, 4);
		}
	}

	return 0;
}

template <typename cbtConvexTemplate>
static int cbtRefinePortal(const cbtConvexTemplate &a, const cbtConvexTemplate &b, const cbtMprCollisionDescription &colDesc,
						  cbtMprSimplex_t *portal)
{
	cbtVector3 dir;
	cbtMprSupport_t v4;

	for (int i = 0; i < BT_MPR_MAX_ITERATIONS; i++)
	//while (1)
	{
		// compute direction outside the portal (from v0 throught v1,v2,v3
		// face)
		cbtPortalDir(portal, &dir);

		// test if origin is inside the portal
		if (portalEncapsulesOrigin(portal, &dir))
			return 0;

		// get next support point

		cbtMprSupport(a, b, colDesc, dir, &v4);

		// test if v4 can expand portal to contain origin and if portal
		// expanding doesn't reach given tolerance
		if (!portalCanEncapsuleOrigin(portal, &v4, &dir) || portalReachTolerance(portal, &v4, &dir))
		{
			return -1;
		}

		// v1-v2-v3 triangle must be rearranged to face outside Minkowski
		// difference (direction from v0).
		cbtExpandPortal(portal, &v4);
	}

	return -1;
}

static void cbtFindPos(const cbtMprSimplex_t *portal, cbtVector3 *pos)
{
	cbtVector3 zero = cbtVector3(0, 0, 0);
	cbtVector3 *origin = &zero;

	cbtVector3 dir;
	size_t i;
	float b[4], sum, inv;
	cbtVector3 vec, p1, p2;

	cbtPortalDir(portal, &dir);

	// use barycentric coordinates of tetrahedron to find origin
	cbtMprVec3Cross(&vec, &cbtMprSimplexPoint(portal, 1)->v,
				   &cbtMprSimplexPoint(portal, 2)->v);
	b[0] = cbtMprVec3Dot(&vec, &cbtMprSimplexPoint(portal, 3)->v);

	cbtMprVec3Cross(&vec, &cbtMprSimplexPoint(portal, 3)->v,
				   &cbtMprSimplexPoint(portal, 2)->v);
	b[1] = cbtMprVec3Dot(&vec, &cbtMprSimplexPoint(portal, 0)->v);

	cbtMprVec3Cross(&vec, &cbtMprSimplexPoint(portal, 0)->v,
				   &cbtMprSimplexPoint(portal, 1)->v);
	b[2] = cbtMprVec3Dot(&vec, &cbtMprSimplexPoint(portal, 3)->v);

	cbtMprVec3Cross(&vec, &cbtMprSimplexPoint(portal, 2)->v,
				   &cbtMprSimplexPoint(portal, 1)->v);
	b[3] = cbtMprVec3Dot(&vec, &cbtMprSimplexPoint(portal, 0)->v);

	sum = b[0] + b[1] + b[2] + b[3];

	if (cbtMprIsZero(sum) || sum < 0.f)
	{
		b[0] = 0.f;

		cbtMprVec3Cross(&vec, &cbtMprSimplexPoint(portal, 2)->v,
					   &cbtMprSimplexPoint(portal, 3)->v);
		b[1] = cbtMprVec3Dot(&vec, &dir);
		cbtMprVec3Cross(&vec, &cbtMprSimplexPoint(portal, 3)->v,
					   &cbtMprSimplexPoint(portal, 1)->v);
		b[2] = cbtMprVec3Dot(&vec, &dir);
		cbtMprVec3Cross(&vec, &cbtMprSimplexPoint(portal, 1)->v,
					   &cbtMprSimplexPoint(portal, 2)->v);
		b[3] = cbtMprVec3Dot(&vec, &dir);

		sum = b[1] + b[2] + b[3];
	}

	inv = 1.f / sum;

	cbtMprVec3Copy(&p1, origin);
	cbtMprVec3Copy(&p2, origin);
	for (i = 0; i < 4; i++)
	{
		cbtMprVec3Copy(&vec, &cbtMprSimplexPoint(portal, i)->v1);
		cbtMprVec3Scale(&vec, b[i]);
		cbtMprVec3Add(&p1, &vec);

		cbtMprVec3Copy(&vec, &cbtMprSimplexPoint(portal, i)->v2);
		cbtMprVec3Scale(&vec, b[i]);
		cbtMprVec3Add(&p2, &vec);
	}
	cbtMprVec3Scale(&p1, inv);
	cbtMprVec3Scale(&p2, inv);
#ifdef MPR_AVERAGE_CONTACT_POSITIONS
	cbtMprVec3Copy(pos, &p1);
	cbtMprVec3Add(pos, &p2);
	cbtMprVec3Scale(pos, 0.5);
#else
	cbtMprVec3Copy(pos, &p2);
#endif  //MPR_AVERAGE_CONTACT_POSITIONS
}

inline float cbtMprVec3Dist2(const cbtVector3 *a, const cbtVector3 *b)
{
	cbtVector3 ab;
	cbtMprVec3Sub2(&ab, a, b);
	return cbtMprVec3Len2(&ab);
}

inline float _cbtMprVec3PointSegmentDist2(const cbtVector3 *P,
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

	float dist, t;
	cbtVector3 d, a;

	// direction of segment
	cbtMprVec3Sub2(&d, b, x0);

	// precompute vector from P to x0
	cbtMprVec3Sub2(&a, x0, P);

	t = -1.f * cbtMprVec3Dot(&a, &d);
	t /= cbtMprVec3Len2(&d);

	if (t < 0.f || cbtMprIsZero(t))
	{
		dist = cbtMprVec3Dist2(x0, P);
		if (witness)
			cbtMprVec3Copy(witness, x0);
	}
	else if (t > 1.f || cbtMprEq(t, 1.f))
	{
		dist = cbtMprVec3Dist2(b, P);
		if (witness)
			cbtMprVec3Copy(witness, b);
	}
	else
	{
		if (witness)
		{
			cbtMprVec3Copy(witness, &d);
			cbtMprVec3Scale(witness, t);
			cbtMprVec3Add(witness, x0);
			dist = cbtMprVec3Dist2(witness, P);
		}
		else
		{
			// recycling variables
			cbtMprVec3Scale(&d, t);
			cbtMprVec3Add(&d, &a);
			dist = cbtMprVec3Len2(&d);
		}
	}

	return dist;
}

inline float cbtMprVec3PointTriDist2(const cbtVector3 *P,
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
	float u, v, w, p, q, r;
	float s, t, dist, dist2;
	cbtVector3 witness2;

	cbtMprVec3Sub2(&d1, B, x0);
	cbtMprVec3Sub2(&d2, C, x0);
	cbtMprVec3Sub2(&a, x0, P);

	u = cbtMprVec3Dot(&a, &a);
	v = cbtMprVec3Dot(&d1, &d1);
	w = cbtMprVec3Dot(&d2, &d2);
	p = cbtMprVec3Dot(&a, &d1);
	q = cbtMprVec3Dot(&a, &d2);
	r = cbtMprVec3Dot(&d1, &d2);

	cbtScalar div = (w * v - r * r);
	if (cbtMprIsZero(div))
	{
		s = -1;
	}
	else
	{
		s = (q * r - w * p) / div;
		t = (-s * r - q) / w;
	}

	if ((cbtMprIsZero(s) || s > 0.f) && (cbtMprEq(s, 1.f) || s < 1.f) && (cbtMprIsZero(t) || t > 0.f) && (cbtMprEq(t, 1.f) || t < 1.f) && (cbtMprEq(t + s, 1.f) || t + s < 1.f))
	{
		if (witness)
		{
			cbtMprVec3Scale(&d1, s);
			cbtMprVec3Scale(&d2, t);
			cbtMprVec3Copy(witness, x0);
			cbtMprVec3Add(witness, &d1);
			cbtMprVec3Add(witness, &d2);

			dist = cbtMprVec3Dist2(witness, P);
		}
		else
		{
			dist = s * s * v;
			dist += t * t * w;
			dist += 2.f * s * t * r;
			dist += 2.f * s * p;
			dist += 2.f * t * q;
			dist += u;
		}
	}
	else
	{
		dist = _cbtMprVec3PointSegmentDist2(P, x0, B, witness);

		dist2 = _cbtMprVec3PointSegmentDist2(P, x0, C, &witness2);
		if (dist2 < dist)
		{
			dist = dist2;
			if (witness)
				cbtMprVec3Copy(witness, &witness2);
		}

		dist2 = _cbtMprVec3PointSegmentDist2(P, B, C, &witness2);
		if (dist2 < dist)
		{
			dist = dist2;
			if (witness)
				cbtMprVec3Copy(witness, &witness2);
		}
	}

	return dist;
}

template <typename cbtConvexTemplate>
static void cbtFindPenetr(const cbtConvexTemplate &a, const cbtConvexTemplate &b,
						 const cbtMprCollisionDescription &colDesc,
						 cbtMprSimplex_t *portal,
						 float *depth, cbtVector3 *pdir, cbtVector3 *pos)
{
	cbtVector3 dir;
	cbtMprSupport_t v4;
	unsigned long iterations;

	cbtVector3 zero = cbtVector3(0, 0, 0);
	cbtVector3 *origin = &zero;

	iterations = 1UL;
	for (int i = 0; i < BT_MPR_MAX_ITERATIONS; i++)
	//while (1)
	{
		// compute portal direction and obtain next support point
		cbtPortalDir(portal, &dir);

		cbtMprSupport(a, b, colDesc, dir, &v4);

		// reached tolerance -> find penetration info
		if (portalReachTolerance(portal, &v4, &dir) || iterations == BT_MPR_MAX_ITERATIONS)
		{
			*depth = cbtMprVec3PointTriDist2(origin, &cbtMprSimplexPoint(portal, 1)->v, &cbtMprSimplexPoint(portal, 2)->v, &cbtMprSimplexPoint(portal, 3)->v, pdir);
			*depth = BT_MPR_SQRT(*depth);

			if (cbtMprIsZero((*pdir).x()) && cbtMprIsZero((*pdir).y()) && cbtMprIsZero((*pdir).z()))
			{
				*pdir = dir;
			}
			cbtMprVec3Normalize(pdir);

			// barycentric coordinates:
			cbtFindPos(portal, pos);

			return;
		}

		cbtExpandPortal(portal, &v4);

		iterations++;
	}
}

static void cbtFindPenetrTouch(cbtMprSimplex_t *portal, float *depth, cbtVector3 *dir, cbtVector3 *pos)
{
	// Touching contact on portal's v1 - so depth is zero and direction
	// is unimportant and pos can be guessed
	*depth = 0.f;
	cbtVector3 zero = cbtVector3(0, 0, 0);
	cbtVector3 *origin = &zero;

	cbtMprVec3Copy(dir, origin);
#ifdef MPR_AVERAGE_CONTACT_POSITIONS
	cbtMprVec3Copy(pos, &cbtMprSimplexPoint(portal, 1)->v1);
	cbtMprVec3Add(pos, &cbtMprSimplexPoint(portal, 1)->v2);
	cbtMprVec3Scale(pos, 0.5);
#else
	cbtMprVec3Copy(pos, &cbtMprSimplexPoint(portal, 1)->v2);
#endif
}

static void cbtFindPenetrSegment(cbtMprSimplex_t *portal,
								float *depth, cbtVector3 *dir, cbtVector3 *pos)
{
	// Origin lies on v0-v1 segment.
	// Depth is distance to v1, direction also and position must be
	// computed
#ifdef MPR_AVERAGE_CONTACT_POSITIONS
	cbtMprVec3Copy(pos, &cbtMprSimplexPoint(portal, 1)->v1);
	cbtMprVec3Add(pos, &cbtMprSimplexPoint(portal, 1)->v2);
	cbtMprVec3Scale(pos, 0.5f);
#else
	cbtMprVec3Copy(pos, &cbtMprSimplexPoint(portal, 1)->v2);
#endif  //MPR_AVERAGE_CONTACT_POSITIONS

	cbtMprVec3Copy(dir, &cbtMprSimplexPoint(portal, 1)->v);
	*depth = BT_MPR_SQRT(cbtMprVec3Len2(dir));
	cbtMprVec3Normalize(dir);
}

template <typename cbtConvexTemplate>
inline int cbtMprPenetration(const cbtConvexTemplate &a, const cbtConvexTemplate &b,
							const cbtMprCollisionDescription &colDesc,
							float *depthOut, cbtVector3 *dirOut, cbtVector3 *posOut)
{
	cbtMprSimplex_t portal;

	// Phase 1: Portal discovery
	int result = cbtDiscoverPortal(a, b, colDesc, &portal);

	//sepAxis[pairIndex] = *pdir;//or -dir?

	switch (result)
	{
		case 0:
		{
			// Phase 2: Portal refinement

			result = cbtRefinePortal(a, b, colDesc, &portal);
			if (result < 0)
				return -1;

			// Phase 3. Penetration info
			cbtFindPenetr(a, b, colDesc, &portal, depthOut, dirOut, posOut);

			break;
		}
		case 1:
		{
			// Touching contact on portal's v1.
			cbtFindPenetrTouch(&portal, depthOut, dirOut, posOut);
			result = 0;
			break;
		}
		case 2:
		{
			cbtFindPenetrSegment(&portal, depthOut, dirOut, posOut);
			result = 0;
			break;
		}
		default:
		{
			//if (res < 0)
			//{
			// Origin isn't inside portal - no collision.
			result = -1;
			//}
		}
	};

	return result;
};

template <typename cbtConvexTemplate, typename cbtMprDistanceTemplate>
inline int cbtComputeMprPenetration(const cbtConvexTemplate &a, const cbtConvexTemplate &b, const cbtMprCollisionDescription &colDesc, cbtMprDistanceTemplate *distInfo)
{
	cbtVector3 dir, pos;
	float depth;

	int res = cbtMprPenetration(a, b, colDesc, &depth, &dir, &pos);
	if (res == 0)
	{
		distInfo->m_distance = -depth;
		distInfo->m_pointOnB = pos;
		distInfo->m_normalBtoA = -dir;
		distInfo->m_pointOnA = pos - distInfo->m_distance * dir;
		return 0;
	}

	return -1;
}

#endif  //BT_MPR_PENETRATION_H
