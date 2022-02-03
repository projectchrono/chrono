/*
Stan Melax Convex Hull Computation
Copyright (c) 2003-2006 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <string.h>

#include "cbtConvexHull.h"
#include "cbtAlignedObjectArray.h"
#include "cbtMinMax.h"
#include "cbtVector3.h"

//----------------------------------

// ***CHRONO*** rename class from 'int3' to 'cbtInt3' for consistency with renaming of 'int4'
class cbtInt3
{
public:
	int x, y, z;
    cbtInt3(){};
    cbtInt3(int _x, int _y, int _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}
	const int &operator[](int i) const { return (&x)[i]; }
	int &operator[](int i) { return (&x)[i]; }
};

//------- cbtPlane ----------

inline cbtPlane PlaneFlip(const cbtPlane &plane) { return cbtPlane(-plane.normal, -plane.dist); }
inline int operator==(const cbtPlane &a, const cbtPlane &b) { return (a.normal == b.normal && a.dist == b.dist); }
inline int coplanar(const cbtPlane &a, const cbtPlane &b) { return (a == b || a == PlaneFlip(b)); }

//--------- Utility Functions ------

cbtVector3 PlaneLineIntersection(const cbtPlane &plane, const cbtVector3 &p0, const cbtVector3 &p1);
cbtVector3 PlaneProject(const cbtPlane &plane, const cbtVector3 &point);

cbtVector3 ThreePlaneIntersection(const cbtPlane &p0, const cbtPlane &p1, const cbtPlane &p2);
cbtVector3 ThreePlaneIntersection(const cbtPlane &p0, const cbtPlane &p1, const cbtPlane &p2)
{
	cbtVector3 N1 = p0.normal;
	cbtVector3 N2 = p1.normal;
	cbtVector3 N3 = p2.normal;

	cbtVector3 n2n3;
	n2n3 = N2.cross(N3);
	cbtVector3 n3n1;
	n3n1 = N3.cross(N1);
	cbtVector3 n1n2;
	n1n2 = N1.cross(N2);

	cbtScalar quotient = (N1.dot(n2n3));

	cbtAssert(cbtFabs(quotient) > cbtScalar(0.000001));

	quotient = cbtScalar(-1.) / quotient;
	n2n3 *= p0.dist;
	n3n1 *= p1.dist;
	n1n2 *= p2.dist;
	cbtVector3 potentialVertex = n2n3;
	potentialVertex += n3n1;
	potentialVertex += n1n2;
	potentialVertex *= quotient;

	cbtVector3 result(potentialVertex.getX(), potentialVertex.getY(), potentialVertex.getZ());
	return result;
}

cbtScalar DistanceBetweenLines(const cbtVector3 &ustart, const cbtVector3 &udir, const cbtVector3 &vstart, const cbtVector3 &vdir, cbtVector3 *upoint = NULL, cbtVector3 *vpoint = NULL);
cbtVector3 TriNormal(const cbtVector3 &v0, const cbtVector3 &v1, const cbtVector3 &v2);
cbtVector3 NormalOf(const cbtVector3 *vert, const int n);

cbtVector3 PlaneLineIntersection(const cbtPlane &plane, const cbtVector3 &p0, const cbtVector3 &p1)
{
	// returns the point where the line p0-p1 intersects the plane n&d
	cbtVector3 dif;
	dif = p1 - p0;
	cbtScalar dn = cbtDot(plane.normal, dif);
	cbtScalar t = -(plane.dist + cbtDot(plane.normal, p0)) / dn;
	return p0 + (dif * t);
}

cbtVector3 PlaneProject(const cbtPlane &plane, const cbtVector3 &point)
{
	return point - plane.normal * (cbtDot(point, plane.normal) + plane.dist);
}

cbtVector3 TriNormal(const cbtVector3 &v0, const cbtVector3 &v1, const cbtVector3 &v2)
{
	// return the normal of the triangle
	// inscribed by v0, v1, and v2
	cbtVector3 cp = cbtCross(v1 - v0, v2 - v1);
	cbtScalar m = cp.length();
	if (m == 0) return cbtVector3(1, 0, 0);
	return cp * (cbtScalar(1.0) / m);
}

cbtScalar DistanceBetweenLines(const cbtVector3 &ustart, const cbtVector3 &udir, const cbtVector3 &vstart, const cbtVector3 &vdir, cbtVector3 *upoint, cbtVector3 *vpoint)
{
	cbtVector3 cp;
	cp = cbtCross(udir, vdir).normalized();

	cbtScalar distu = -cbtDot(cp, ustart);
	cbtScalar distv = -cbtDot(cp, vstart);
	cbtScalar dist = (cbtScalar)fabs(distu - distv);
	if (upoint)
	{
		cbtPlane plane;
		plane.normal = cbtCross(vdir, cp).normalized();
		plane.dist = -cbtDot(plane.normal, vstart);
		*upoint = PlaneLineIntersection(plane, ustart, ustart + udir);
	}
	if (vpoint)
	{
		cbtPlane plane;
		plane.normal = cbtCross(udir, cp).normalized();
		plane.dist = -cbtDot(plane.normal, ustart);
		*vpoint = PlaneLineIntersection(plane, vstart, vstart + vdir);
	}
	return dist;
}

#define COPLANAR (0)
#define UNDER (1)
#define OVER (2)
#define SPLIT (OVER | UNDER)
#define PAPERWIDTH (cbtScalar(0.001))

cbtScalar planetestepsilon = PAPERWIDTH;

typedef ConvexH::HalfEdge HalfEdge;

ConvexH::ConvexH(int vertices_size, int edges_size, int facets_size)
{
	vertices.resize(vertices_size);
	edges.resize(edges_size);
	facets.resize(facets_size);
}

int PlaneTest(const cbtPlane &p, const cbtVector3 &v);
int PlaneTest(const cbtPlane &p, const cbtVector3 &v)
{
	cbtScalar a = cbtDot(v, p.normal) + p.dist;
	int flag = (a > planetestepsilon) ? OVER : ((a < -planetestepsilon) ? UNDER : COPLANAR);
	return flag;
}

int SplitTest(ConvexH &convex, const cbtPlane &plane);
int SplitTest(ConvexH &convex, const cbtPlane &plane)
{
	int flag = 0;
	for (int i = 0; i < convex.vertices.size(); i++)
	{
		flag |= PlaneTest(plane, convex.vertices[i]);
	}
	return flag;
}

class VertFlag
{
public:
	unsigned char planetest;
	unsigned char junk;
	unsigned char undermap;
	unsigned char overmap;
};
class EdgeFlag
{
public:
	unsigned char planetest;
	unsigned char fixes;
	short undermap;
	short overmap;
};
class PlaneFlag
{
public:
	unsigned char undermap;
	unsigned char overmap;
};
class Coplanar
{
public:
	unsigned short ea;
	unsigned char v0;
	unsigned char v1;
};

template <class T>
int maxdirfiltered(const T *p, int count, const T &dir, cbtAlignedObjectArray<int> &allow)
{
	cbtAssert(count);
	int m = -1;
	for (int i = 0; i < count; i++)
		if (allow[i])
		{
			if (m == -1 || cbtDot(p[i], dir) > cbtDot(p[m], dir))
				m = i;
		}
	cbtAssert(m != -1);
	return m;
}

cbtVector3 orth(const cbtVector3 &v);
cbtVector3 orth(const cbtVector3 &v)
{
	cbtVector3 a = cbtCross(v, cbtVector3(0, 0, 1));
	cbtVector3 b = cbtCross(v, cbtVector3(0, 1, 0));
	if (a.length() > b.length())
	{
		return a.normalized();
	}
	else
	{
		return b.normalized();
	}
}

template <class T>
int maxdirsterid(const T *p, int count, const T &dir, cbtAlignedObjectArray<int> &allow)
{
	int m = -1;
	while (m == -1)
	{
		m = maxdirfiltered(p, count, dir, allow);
		if (allow[m] == 3) return m;
		T u = orth(dir);
		T v = cbtCross(u, dir);
		int ma = -1;
		for (cbtScalar x = cbtScalar(0.0); x <= cbtScalar(360.0); x += cbtScalar(45.0))
		{
			cbtScalar s = cbtSin(SIMD_RADS_PER_DEG * (x));
			cbtScalar c = cbtCos(SIMD_RADS_PER_DEG * (x));
			int mb = maxdirfiltered(p, count, dir + (u * s + v * c) * cbtScalar(0.025), allow);
			if (ma == m && mb == m)
			{
				allow[m] = 3;
				return m;
			}
			if (ma != -1 && ma != mb)  // Yuck - this is really ugly
			{
				int mc = ma;
				for (cbtScalar xx = x - cbtScalar(40.0); xx <= x; xx += cbtScalar(5.0))
				{
					cbtScalar s = cbtSin(SIMD_RADS_PER_DEG * (xx));
					cbtScalar c = cbtCos(SIMD_RADS_PER_DEG * (xx));
					int md = maxdirfiltered(p, count, dir + (u * s + v * c) * cbtScalar(0.025), allow);
					if (mc == m && md == m)
					{
						allow[m] = 3;
						return m;
					}
					mc = md;
				}
			}
			ma = mb;
		}
		allow[m] = 0;
		m = -1;
	}
	cbtAssert(0);
	return m;
}

int operator==(const cbtInt3& a, const cbtInt3& b);
int operator==(const cbtInt3 &a, const cbtInt3 &b)
{
	for (int i = 0; i < 3; i++)
	{
		if (a[i] != b[i]) return 0;
	}
	return 1;
}

int above(cbtVector3* vertices, const cbtInt3& t, const cbtVector3& p, cbtScalar epsilon);
int above(cbtVector3 *vertices, const cbtInt3 &t, const cbtVector3 &p, cbtScalar epsilon)
{
	cbtVector3 n = TriNormal(vertices[t[0]], vertices[t[1]], vertices[t[2]]);
	return (cbtDot(n, p - vertices[t[0]]) > epsilon);  // EPSILON???
}
int hasedge(const cbtInt3& t, int a, int b);
int hasedge(const cbtInt3 &t, int a, int b)
{
	for (int i = 0; i < 3; i++)
	{
		int i1 = (i + 1) % 3;
		if (t[i] == a && t[i1] == b) return 1;
	}
	return 0;
}
int hasvert(const cbtInt3& t, int v);
int hasvert(const cbtInt3 &t, int v)
{
	return (t[0] == v || t[1] == v || t[2] == v);
}
int shareedge(const cbtInt3& a, const cbtInt3& b);
int shareedge(const cbtInt3 &a, const cbtInt3 &b)
{
	int i;
	for (i = 0; i < 3; i++)
	{
		int i1 = (i + 1) % 3;
		if (hasedge(a, b[i1], b[i])) return 1;
	}
	return 0;
}

class cbtHullTriangle;

class cbtHullTriangle : public cbtInt3
{
public:
    cbtInt3 n;
	int id;
	int vmax;
	cbtScalar rise;
    cbtHullTriangle(int a, int b, int c) : cbtInt3(a, b, c), n(-1, -1, -1)
	{
		vmax = -1;
		rise = cbtScalar(0.0);
	}
	~cbtHullTriangle()
	{
	}
	int &neib(int a, int b);
};

int &cbtHullTriangle::neib(int a, int b)
{
	static int er = -1;
	int i;
	for (i = 0; i < 3; i++)
	{
		int i1 = (i + 1) % 3;
		int i2 = (i + 2) % 3;
		if ((*this)[i] == a && (*this)[i1] == b) return n[i2];
		if ((*this)[i] == b && (*this)[i1] == a) return n[i2];
	}
	cbtAssert(0);
	return er;
}
void HullLibrary::b2bfix(cbtHullTriangle *s, cbtHullTriangle *t)
{
	int i;
	for (i = 0; i < 3; i++)
	{
		int i1 = (i + 1) % 3;
		int i2 = (i + 2) % 3;
		int a = (*s)[i1];
		int b = (*s)[i2];
		cbtAssert(m_tris[s->neib(a, b)]->neib(b, a) == s->id);
		cbtAssert(m_tris[t->neib(a, b)]->neib(b, a) == t->id);
		m_tris[s->neib(a, b)]->neib(b, a) = t->neib(b, a);
		m_tris[t->neib(b, a)]->neib(a, b) = s->neib(a, b);
	}
}

void HullLibrary::removeb2b(cbtHullTriangle *s, cbtHullTriangle *t)
{
	b2bfix(s, t);
	deAllocateTriangle(s);

	deAllocateTriangle(t);
}

void HullLibrary::checkit(cbtHullTriangle *t)
{
	(void)t;

	int i;
	cbtAssert(m_tris[t->id] == t);
	for (i = 0; i < 3; i++)
	{
		int i1 = (i + 1) % 3;
		int i2 = (i + 2) % 3;
		int a = (*t)[i1];
		int b = (*t)[i2];

		// release compile fix
		(void)i1;
		(void)i2;
		(void)a;
		(void)b;

		cbtAssert(a != b);
		cbtAssert(m_tris[t->n[i]]->neib(b, a) == t->id);
	}
}

cbtHullTriangle *HullLibrary::allocateTriangle(int a, int b, int c)
{
	void *mem = cbtAlignedAlloc(sizeof(cbtHullTriangle), 16);
	cbtHullTriangle *tr = new (mem) cbtHullTriangle(a, b, c);
	tr->id = m_tris.size();
	m_tris.push_back(tr);

	return tr;
}

void HullLibrary::deAllocateTriangle(cbtHullTriangle *tri)
{
	cbtAssert(m_tris[tri->id] == tri);
	m_tris[tri->id] = NULL;
	tri->~cbtHullTriangle();
	cbtAlignedFree(tri);
}

void HullLibrary::extrude(cbtHullTriangle *t0, int v)
{
    cbtInt3 t = *t0;
	int n = m_tris.size();
	cbtHullTriangle *ta = allocateTriangle(v, t[1], t[2]);
    ta->n = cbtInt3(t0->n[0], n + 1, n + 2);
	m_tris[t0->n[0]]->neib(t[1], t[2]) = n + 0;
	cbtHullTriangle *tb = allocateTriangle(v, t[2], t[0]);
    tb->n = cbtInt3(t0->n[1], n + 2, n + 0);
	m_tris[t0->n[1]]->neib(t[2], t[0]) = n + 1;
	cbtHullTriangle *tc = allocateTriangle(v, t[0], t[1]);
    tc->n = cbtInt3(t0->n[2], n + 0, n + 1);
	m_tris[t0->n[2]]->neib(t[0], t[1]) = n + 2;
	checkit(ta);
	checkit(tb);
	checkit(tc);
	if (hasvert(*m_tris[ta->n[0]], v)) removeb2b(ta, m_tris[ta->n[0]]);
	if (hasvert(*m_tris[tb->n[0]], v)) removeb2b(tb, m_tris[tb->n[0]]);
	if (hasvert(*m_tris[tc->n[0]], v)) removeb2b(tc, m_tris[tc->n[0]]);
	deAllocateTriangle(t0);
}

cbtHullTriangle *HullLibrary::extrudable(cbtScalar epsilon)
{
	int i;
	cbtHullTriangle *t = NULL;
	for (i = 0; i < m_tris.size(); i++)
	{
		if (!t || (m_tris[i] && t->rise < m_tris[i]->rise))
		{
			t = m_tris[i];
		}
	}
	return (t->rise > epsilon) ? t : NULL;
}

// ***CHRONO*** rename class from 'int4' to 'cbtInt4' to prevent possible clash with CUDA 'int4' struct
cbtInt4 HullLibrary::FindSimplex(cbtVector3* verts, int verts_count, cbtAlignedObjectArray<int>& allow) {
	cbtVector3 basis[3];
	basis[0] = cbtVector3(cbtScalar(0.01), cbtScalar(0.02), cbtScalar(1.0));
	int p0 = maxdirsterid(verts, verts_count, basis[0], allow);
	int p1 = maxdirsterid(verts, verts_count, -basis[0], allow);
	basis[0] = verts[p0] - verts[p1];
	if (p0 == p1 || basis[0] == cbtVector3(0, 0, 0))
        return cbtInt4(-1, -1, -1, -1);
	basis[1] = cbtCross(cbtVector3(cbtScalar(1), cbtScalar(0.02), cbtScalar(0)), basis[0]);
	basis[2] = cbtCross(cbtVector3(cbtScalar(-0.02), cbtScalar(1), cbtScalar(0)), basis[0]);
	if (basis[1].length() > basis[2].length())
	{
		basis[1].normalize();
	}
	else
	{
		basis[1] = basis[2];
		basis[1].normalize();
	}
	int p2 = maxdirsterid(verts, verts_count, basis[1], allow);
	if (p2 == p0 || p2 == p1)
	{
		p2 = maxdirsterid(verts, verts_count, -basis[1], allow);
	}
	if (p2 == p0 || p2 == p1)
        return cbtInt4(-1, -1, -1, -1);
	basis[1] = verts[p2] - verts[p0];
	basis[2] = cbtCross(basis[1], basis[0]).normalized();
	int p3 = maxdirsterid(verts, verts_count, basis[2], allow);
	if (p3 == p0 || p3 == p1 || p3 == p2) p3 = maxdirsterid(verts, verts_count, -basis[2], allow);
	if (p3 == p0 || p3 == p1 || p3 == p2)
        return cbtInt4(-1, -1, -1, -1);
	cbtAssert(!(p0 == p1 || p0 == p2 || p0 == p3 || p1 == p2 || p1 == p3 || p2 == p3));
	if (cbtDot(verts[p3] - verts[p0], cbtCross(verts[p1] - verts[p0], verts[p2] - verts[p0])) < 0)
	{
		cbtSwap(p2, p3);
	}
    return cbtInt4(p0, p1, p2, p3);
}

int HullLibrary::calchullgen(cbtVector3 *verts, int verts_count, int vlimit)
{
	if (verts_count < 4) return 0;
	if (vlimit == 0) vlimit = 1000000000;
	int j;
	cbtVector3 bmin(*verts), bmax(*verts);
	cbtAlignedObjectArray<int> isextreme;
	isextreme.reserve(verts_count);
	cbtAlignedObjectArray<int> allow;
	allow.reserve(verts_count);

	for (j = 0; j < verts_count; j++)
	{
		allow.push_back(1);
		isextreme.push_back(0);
		bmin.setMin(verts[j]);
		bmax.setMax(verts[j]);
	}
	cbtScalar epsilon = (bmax - bmin).length() * cbtScalar(0.001);
	cbtAssert(epsilon != 0.0);

	cbtInt4 p = FindSimplex(verts, verts_count, allow);
	if (p.x == -1) return 0;  // simplex failed

	cbtVector3 center = (verts[p[0]] + verts[p[1]] + verts[p[2]] + verts[p[3]]) / cbtScalar(4.0);  // a valid interior point
	cbtHullTriangle *t0 = allocateTriangle(p[2], p[3], p[1]);
    t0->n = cbtInt3(2, 3, 1);
	cbtHullTriangle *t1 = allocateTriangle(p[3], p[2], p[0]);
    t1->n = cbtInt3(3, 2, 0);
	cbtHullTriangle *t2 = allocateTriangle(p[0], p[1], p[3]);
    t2->n = cbtInt3(0, 1, 3);
	cbtHullTriangle *t3 = allocateTriangle(p[1], p[0], p[2]);
    t3->n = cbtInt3(1, 0, 2);
	isextreme[p[0]] = isextreme[p[1]] = isextreme[p[2]] = isextreme[p[3]] = 1;
	checkit(t0);
	checkit(t1);
	checkit(t2);
	checkit(t3);

	for (j = 0; j < m_tris.size(); j++)
	{
		cbtHullTriangle *t = m_tris[j];
		cbtAssert(t);
		cbtAssert(t->vmax < 0);
		cbtVector3 n = TriNormal(verts[(*t)[0]], verts[(*t)[1]], verts[(*t)[2]]);
		t->vmax = maxdirsterid(verts, verts_count, n, allow);
		t->rise = cbtDot(n, verts[t->vmax] - verts[(*t)[0]]);
	}
	cbtHullTriangle *te;
	vlimit -= 4;
	while (vlimit > 0 && ((te = extrudable(epsilon)) != 0))
	{
		//int3 ti=*te;
		int v = te->vmax;
		cbtAssert(v != -1);
		cbtAssert(!isextreme[v]);  // wtf we've already done this vertex
		isextreme[v] = 1;
		//if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
		j = m_tris.size();
		while (j--)
		{
			if (!m_tris[j]) continue;
            cbtInt3 t = *m_tris[j];
			if (above(verts, t, verts[v], cbtScalar(0.01) * epsilon))
			{
				extrude(m_tris[j], v);
			}
		}
		// now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
		j = m_tris.size();
		while (j--)
		{
			if (!m_tris[j]) continue;
			if (!hasvert(*m_tris[j], v)) break;
            cbtInt3 nt = *m_tris[j];
			if (above(verts, nt, center, cbtScalar(0.01) * epsilon) || cbtCross(verts[nt[1]] - verts[nt[0]], verts[nt[2]] - verts[nt[1]]).length() < epsilon * epsilon * cbtScalar(0.1))
			{
				cbtHullTriangle *nb = m_tris[m_tris[j]->n[0]];
				cbtAssert(nb);
				cbtAssert(!hasvert(*nb, v));
				cbtAssert(nb->id < j);
				extrude(nb, v);
				j = m_tris.size();
			}
		}
		j = m_tris.size();
		while (j--)
		{
			cbtHullTriangle *t = m_tris[j];
			if (!t) continue;
			if (t->vmax >= 0) break;
			cbtVector3 n = TriNormal(verts[(*t)[0]], verts[(*t)[1]], verts[(*t)[2]]);
			t->vmax = maxdirsterid(verts, verts_count, n, allow);
			if (isextreme[t->vmax])
			{
				t->vmax = -1;  // already done that vertex - algorithm needs to be able to terminate.
			}
			else
			{
				t->rise = cbtDot(n, verts[t->vmax] - verts[(*t)[0]]);
			}
		}
		vlimit--;
	}
	return 1;
}

int HullLibrary::calchull(cbtVector3 *verts, int verts_count, TUIntArray &tris_out, int &tris_count, int vlimit)
{
	int rc = calchullgen(verts, verts_count, vlimit);
	if (!rc) return 0;
	cbtAlignedObjectArray<int> ts;
	int i;

	for (i = 0; i < m_tris.size(); i++)
	{
		if (m_tris[i])
		{
			for (int j = 0; j < 3; j++)
				ts.push_back((*m_tris[i])[j]);
			deAllocateTriangle(m_tris[i]);
		}
	}
	tris_count = ts.size() / 3;
	tris_out.resize(ts.size());

	for (i = 0; i < ts.size(); i++)
	{
		tris_out[i] = static_cast<unsigned int>(ts[i]);
	}
	m_tris.resize(0);

	return 1;
}

bool HullLibrary::ComputeHull(unsigned int vcount, const cbtVector3 *vertices, PHullResult &result, unsigned int vlimit)
{
	int tris_count;
	int ret = calchull((cbtVector3 *)vertices, (int)vcount, result.m_Indices, tris_count, static_cast<int>(vlimit));
	if (!ret) return false;
	result.mIndexCount = (unsigned int)(tris_count * 3);
	result.mFaceCount = (unsigned int)tris_count;
	result.mVertices = (cbtVector3 *)vertices;
	result.mVcount = (unsigned int)vcount;
	return true;
}

void ReleaseHull(PHullResult &result);
void ReleaseHull(PHullResult &result)
{
	if (result.m_Indices.size())
	{
		result.m_Indices.clear();
	}

	result.mVcount = 0;
	result.mIndexCount = 0;
	result.mVertices = 0;
}

//*********************************************************************
//*********************************************************************
//********  HullLib header
//*********************************************************************
//*********************************************************************

//*********************************************************************
//*********************************************************************
//********  HullLib implementation
//*********************************************************************
//*********************************************************************

HullError HullLibrary::CreateConvexHull(const HullDesc &desc,  // describes the input request
										HullResult &result)    // contains the resulst
{
	HullError ret = QE_FAIL;

	PHullResult hr;

	unsigned int vcount = desc.mVcount;
	if (vcount < 8) vcount = 8;

	cbtAlignedObjectArray<cbtVector3> vertexSource;
	vertexSource.resize(static_cast<int>(vcount));

	cbtVector3 scale;

	unsigned int ovcount;

	bool ok = CleanupVertices(desc.mVcount, desc.mVertices, desc.mVertexStride, ovcount, &vertexSource[0], desc.mNormalEpsilon, scale);  // normalize point cloud, remove duplicates!

	if (ok)
	{
		//		if ( 1 ) // scale vertices back to their original size.
		{
			for (unsigned int i = 0; i < ovcount; i++)
			{
				cbtVector3 &v = vertexSource[static_cast<int>(i)];
				v[0] *= scale[0];
				v[1] *= scale[1];
				v[2] *= scale[2];
			}
		}

		ok = ComputeHull(ovcount, &vertexSource[0], hr, desc.mMaxVertices);

		if (ok)
		{
			// re-index triangle mesh so it refers to only used vertices, rebuild a new vertex table.
			cbtAlignedObjectArray<cbtVector3> vertexScratch;
			vertexScratch.resize(static_cast<int>(hr.mVcount));

			BringOutYourDead(hr.mVertices, hr.mVcount, &vertexScratch[0], ovcount, &hr.m_Indices[0], hr.mIndexCount);

			ret = QE_OK;

			if (desc.HasHullFlag(QF_TRIANGLES))  // if he wants the results as triangle!
			{
				result.mPolygons = false;
				result.mNumOutputVertices = ovcount;
				result.m_OutputVertices.resize(static_cast<int>(ovcount));
				result.mNumFaces = hr.mFaceCount;
				result.mNumIndices = hr.mIndexCount;

				result.m_Indices.resize(static_cast<int>(hr.mIndexCount));

				memcpy(&result.m_OutputVertices[0], &vertexScratch[0], sizeof(cbtVector3) * ovcount);

				if (desc.HasHullFlag(QF_REVERSE_ORDER))
				{
					const unsigned int *source = &hr.m_Indices[0];
					unsigned int *dest = &result.m_Indices[0];

					for (unsigned int i = 0; i < hr.mFaceCount; i++)
					{
						dest[0] = source[2];
						dest[1] = source[1];
						dest[2] = source[0];
						dest += 3;
						source += 3;
					}
				}
				else
				{
					memcpy(&result.m_Indices[0], &hr.m_Indices[0], sizeof(unsigned int) * hr.mIndexCount);
				}
			}
			else
			{
				result.mPolygons = true;
				result.mNumOutputVertices = ovcount;
				result.m_OutputVertices.resize(static_cast<int>(ovcount));
				result.mNumFaces = hr.mFaceCount;
				result.mNumIndices = hr.mIndexCount + hr.mFaceCount;
				result.m_Indices.resize(static_cast<int>(result.mNumIndices));
				memcpy(&result.m_OutputVertices[0], &vertexScratch[0], sizeof(cbtVector3) * ovcount);

				//				if ( 1 )
				{
					const unsigned int *source = &hr.m_Indices[0];
					unsigned int *dest = &result.m_Indices[0];
					for (unsigned int i = 0; i < hr.mFaceCount; i++)
					{
						dest[0] = 3;
						if (desc.HasHullFlag(QF_REVERSE_ORDER))
						{
							dest[1] = source[2];
							dest[2] = source[1];
							dest[3] = source[0];
						}
						else
						{
							dest[1] = source[0];
							dest[2] = source[1];
							dest[3] = source[2];
						}

						dest += 4;
						source += 3;
					}
				}
			}
			ReleaseHull(hr);
		}
	}

	return ret;
}

HullError HullLibrary::ReleaseResult(HullResult &result)  // release memory allocated for this result, we are done with it.
{
	if (result.m_OutputVertices.size())
	{
		result.mNumOutputVertices = 0;
		result.m_OutputVertices.clear();
	}
	if (result.m_Indices.size())
	{
		result.mNumIndices = 0;
		result.m_Indices.clear();
	}
	return QE_OK;
}

static void addPoint(unsigned int &vcount, cbtVector3 *p, cbtScalar x, cbtScalar y, cbtScalar z)
{
	// XXX, might be broken
	cbtVector3 &dest = p[vcount];
	dest[0] = x;
	dest[1] = y;
	dest[2] = z;
	vcount++;
}

cbtScalar GetDist(cbtScalar px, cbtScalar py, cbtScalar pz, const cbtScalar *p2);
cbtScalar GetDist(cbtScalar px, cbtScalar py, cbtScalar pz, const cbtScalar *p2)
{
	cbtScalar dx = px - p2[0];
	cbtScalar dy = py - p2[1];
	cbtScalar dz = pz - p2[2];

	return dx * dx + dy * dy + dz * dz;
}

bool HullLibrary::CleanupVertices(unsigned int svcount,
								  const cbtVector3 *svertices,
								  unsigned int stride,
								  unsigned int &vcount,  // output number of vertices
								  cbtVector3 *vertices,   // location to store the results.
								  cbtScalar normalepsilon,
								  cbtVector3 &scale)
{
	if (svcount == 0) return false;

	m_vertexIndexMapping.resize(0);

#define EPSILON cbtScalar(0.000001) /* close enough to consider two cbtScalaring point numbers to be 'the same'. */

	vcount = 0;

	cbtScalar recip[3] = {0.f, 0.f, 0.f};

	if (scale)
	{
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;
	}

	cbtScalar bmin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
	cbtScalar bmax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

	const char *vtx = (const char *)svertices;

	//	if ( 1 )
	{
		for (unsigned int i = 0; i < svcount; i++)
		{
			const cbtScalar *p = (const cbtScalar *)vtx;

			vtx += stride;

			for (int j = 0; j < 3; j++)
			{
				if (p[j] < bmin[j]) bmin[j] = p[j];
				if (p[j] > bmax[j]) bmax[j] = p[j];
			}
		}
	}

	cbtScalar dx = bmax[0] - bmin[0];
	cbtScalar dy = bmax[1] - bmin[1];
	cbtScalar dz = bmax[2] - bmin[2];

	cbtVector3 center;

	center[0] = dx * cbtScalar(0.5) + bmin[0];
	center[1] = dy * cbtScalar(0.5) + bmin[1];
	center[2] = dz * cbtScalar(0.5) + bmin[2];

	if (dx < EPSILON || dy < EPSILON || dz < EPSILON || svcount < 3)
	{
		cbtScalar len = FLT_MAX;

		if (dx > EPSILON && dx < len) len = dx;
		if (dy > EPSILON && dy < len) len = dy;
		if (dz > EPSILON && dz < len) len = dz;

		if (len == FLT_MAX)
		{
			dx = dy = dz = cbtScalar(0.01);  // one centimeter
		}
		else
		{
			if (dx < EPSILON) dx = len * cbtScalar(0.05);  // 1/5th the shortest non-zero edge.
			if (dy < EPSILON) dy = len * cbtScalar(0.05);
			if (dz < EPSILON) dz = len * cbtScalar(0.05);
		}

		cbtScalar x1 = center[0] - dx;
		cbtScalar x2 = center[0] + dx;

		cbtScalar y1 = center[1] - dy;
		cbtScalar y2 = center[1] + dy;

		cbtScalar z1 = center[2] - dz;
		cbtScalar z2 = center[2] + dz;

		addPoint(vcount, vertices, x1, y1, z1);
		addPoint(vcount, vertices, x2, y1, z1);
		addPoint(vcount, vertices, x2, y2, z1);
		addPoint(vcount, vertices, x1, y2, z1);
		addPoint(vcount, vertices, x1, y1, z2);
		addPoint(vcount, vertices, x2, y1, z2);
		addPoint(vcount, vertices, x2, y2, z2);
		addPoint(vcount, vertices, x1, y2, z2);

		return true;  // return cube
	}
	else
	{
		if (scale)
		{
			scale[0] = dx;
			scale[1] = dy;
			scale[2] = dz;

			recip[0] = 1 / dx;
			recip[1] = 1 / dy;
			recip[2] = 1 / dz;

			center[0] *= recip[0];
			center[1] *= recip[1];
			center[2] *= recip[2];
		}
	}

	vtx = (const char *)svertices;

	for (unsigned int i = 0; i < svcount; i++)
	{
		const cbtVector3 *p = (const cbtVector3 *)vtx;
		vtx += stride;

		cbtScalar px = p->getX();
		cbtScalar py = p->getY();
		cbtScalar pz = p->getZ();

		if (scale)
		{
			px = px * recip[0];  // normalize
			py = py * recip[1];  // normalize
			pz = pz * recip[2];  // normalize
		}

		//		if ( 1 )
		{
			unsigned int j;

			for (j = 0; j < vcount; j++)
			{
				/// XXX might be broken
				cbtVector3 &v = vertices[j];

				cbtScalar x = v[0];
				cbtScalar y = v[1];
				cbtScalar z = v[2];

				cbtScalar dx = cbtFabs(x - px);
				cbtScalar dy = cbtFabs(y - py);
				cbtScalar dz = cbtFabs(z - pz);

				if (dx < normalepsilon && dy < normalepsilon && dz < normalepsilon)
				{
					// ok, it is close enough to the old one
					// now let us see if it is further from the center of the point cloud than the one we already recorded.
					// in which case we keep this one instead.

					cbtScalar dist1 = GetDist(px, py, pz, center);
					cbtScalar dist2 = GetDist(v[0], v[1], v[2], center);

					if (dist1 > dist2)
					{
						v[0] = px;
						v[1] = py;
						v[2] = pz;
					}

					break;
				}
			}

			if (j == vcount)
			{
				cbtVector3 &dest = vertices[vcount];
				dest[0] = px;
				dest[1] = py;
				dest[2] = pz;
				vcount++;
			}
			m_vertexIndexMapping.push_back(j);
		}
	}

	// ok..now make sure we didn't prune so many vertices it is now invalid.
	//	if ( 1 )
	{
		cbtScalar bmin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
		cbtScalar bmax[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

		for (unsigned int i = 0; i < vcount; i++)
		{
			const cbtVector3 &p = vertices[i];
			for (int j = 0; j < 3; j++)
			{
				if (p[j] < bmin[j]) bmin[j] = p[j];
				if (p[j] > bmax[j]) bmax[j] = p[j];
			}
		}

		cbtScalar dx = bmax[0] - bmin[0];
		cbtScalar dy = bmax[1] - bmin[1];
		cbtScalar dz = bmax[2] - bmin[2];

		if (dx < EPSILON || dy < EPSILON || dz < EPSILON || vcount < 3)
		{
			cbtScalar cx = dx * cbtScalar(0.5) + bmin[0];
			cbtScalar cy = dy * cbtScalar(0.5) + bmin[1];
			cbtScalar cz = dz * cbtScalar(0.5) + bmin[2];

			cbtScalar len = FLT_MAX;

			if (dx >= EPSILON && dx < len) len = dx;
			if (dy >= EPSILON && dy < len) len = dy;
			if (dz >= EPSILON && dz < len) len = dz;

			if (len == FLT_MAX)
			{
				dx = dy = dz = cbtScalar(0.01);  // one centimeter
			}
			else
			{
				if (dx < EPSILON) dx = len * cbtScalar(0.05);  // 1/5th the shortest non-zero edge.
				if (dy < EPSILON) dy = len * cbtScalar(0.05);
				if (dz < EPSILON) dz = len * cbtScalar(0.05);
			}

			cbtScalar x1 = cx - dx;
			cbtScalar x2 = cx + dx;

			cbtScalar y1 = cy - dy;
			cbtScalar y2 = cy + dy;

			cbtScalar z1 = cz - dz;
			cbtScalar z2 = cz + dz;

			vcount = 0;  // add box

			addPoint(vcount, vertices, x1, y1, z1);
			addPoint(vcount, vertices, x2, y1, z1);
			addPoint(vcount, vertices, x2, y2, z1);
			addPoint(vcount, vertices, x1, y2, z1);
			addPoint(vcount, vertices, x1, y1, z2);
			addPoint(vcount, vertices, x2, y1, z2);
			addPoint(vcount, vertices, x2, y2, z2);
			addPoint(vcount, vertices, x1, y2, z2);

			return true;
		}
	}

	return true;
}

void HullLibrary::BringOutYourDead(const cbtVector3 *verts, unsigned int vcount, cbtVector3 *overts, unsigned int &ocount, unsigned int *indices, unsigned indexcount)
{
	cbtAlignedObjectArray<int> tmpIndices;
	tmpIndices.resize(m_vertexIndexMapping.size());
	int i;

	for (i = 0; i < m_vertexIndexMapping.size(); i++)
	{
		tmpIndices[i] = m_vertexIndexMapping[i];
	}

	TUIntArray usedIndices;
	usedIndices.resize(static_cast<int>(vcount));
	memset(&usedIndices[0], 0, sizeof(unsigned int) * vcount);

	ocount = 0;

	for (i = 0; i < int(indexcount); i++)
	{
		unsigned int v = indices[i];  // original array index

		cbtAssert(v >= 0 && v < vcount);

		if (usedIndices[static_cast<int>(v)])  // if already remapped
		{
			indices[i] = usedIndices[static_cast<int>(v)] - 1;  // index to new array
		}
		else
		{
			indices[i] = ocount;  // new index mapping

			overts[ocount][0] = verts[v][0];  // copy old vert to new vert array
			overts[ocount][1] = verts[v][1];
			overts[ocount][2] = verts[v][2];

			for (int k = 0; k < m_vertexIndexMapping.size(); k++)
			{
				if (tmpIndices[k] == int(v))
					m_vertexIndexMapping[k] = ocount;
			}

			ocount++;  // increment output vert count

			cbtAssert(ocount >= 0 && ocount <= vcount);

			usedIndices[static_cast<int>(v)] = ocount;  // assign new index remapping
		}
	}
}
