/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If
you use this software in a product, an acknowledgment in the product
documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original
software.
3. This notice may not be removed or altered from any source distribution.

   Elsevier CDROM license agreements grants nonexclusive license to use the software
   for any purpose, commercial or non-commercial as long as the following credit is included
   identifying the original source of the software:

   Parts of the source are "from the book Real-Time Collision Detection by
   Christer Ericson, published by Morgan Kaufmann Publishers,
   (c) 2005 Elsevier Inc."

*/

#include <algorithm>

#include "collision/ChCCollisionModel.h"

#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/collision/ChCNarrowphaseVoronoiSimplex.h"
using namespace chrono;
using namespace chrono::collision;

#define VERTA 0
#define VERTB 1
#define VERTC 2
#define VERTD 3

#define CATCH_DEGENERATE_TETRAHEDRON 1
void ChCNarrowphaseVoronoiSimplex::removeVertex(int index) {
  m_numVertices--;
  m_simplexVectorW[index] = m_simplexVectorW[m_numVertices];
  m_simplexPointsP[index] = m_simplexPointsP[m_numVertices];
  m_simplexPointsQ[index] = m_simplexPointsQ[m_numVertices];
}

void ChCNarrowphaseVoronoiSimplex::reduceVertices(const btUsageBitfield& usedVerts) {
  if ((numVertices() >= 4) && (!usedVerts.usedVertexD))
    removeVertex(3);

  if ((numVertices() >= 3) && (!usedVerts.usedVertexC))
    removeVertex(2);

  if ((numVertices() >= 2) && (!usedVerts.usedVertexB))
    removeVertex(1);

  if ((numVertices() >= 1) && (!usedVerts.usedVertexA))
    removeVertex(0);
}

// clear the simplex, remove all the vertices
void ChCNarrowphaseVoronoiSimplex::reset() {
  m_cachedValidClosest = false;
  m_numVertices = 0;
  m_needsUpdate = true;
  m_lastW = real3(real(LARGE_REAL), real(LARGE_REAL), real(LARGE_REAL));
  m_cachedBC.reset();
}

// add a vertex
void ChCNarrowphaseVoronoiSimplex::addVertex(const real3& w, const real3& p, const real3& q) {
  m_lastW = w;
  m_needsUpdate = true;

  m_simplexVectorW[m_numVertices] = w;
  m_simplexPointsP[m_numVertices] = p;
  m_simplexPointsQ[m_numVertices] = q;

  m_numVertices++;
}

bool ChCNarrowphaseVoronoiSimplex::updateClosestVectorAndPoints() {
  if (m_needsUpdate) {
    m_cachedBC.reset();

    m_needsUpdate = false;

    switch (numVertices()) {
      case 0:
        m_cachedValidClosest = false;
        break;
      case 1: {
        m_cachedP1 = m_simplexPointsP[0];
        m_cachedP2 = m_simplexPointsQ[0];
        m_cachedV = m_cachedP1 - m_cachedP2;  //== m_simplexVectorW[0]
        m_cachedBC.reset();
        m_cachedBC.setBarycentricCoordinates(real(1.), real(0.), real(0.), real(0.));
        m_cachedValidClosest = m_cachedBC.isValid();
        break;
      }

      case 2: {
        // closest point origin from line segment
        const real3& from = m_simplexVectorW[0];
        const real3& to = m_simplexVectorW[1];
        real3 nearest;

        real3 p(real(0.), real(0.), real(0.));
        real3 diff = p - from;
        real3 v = to - from;
        real t = v.dot(diff);

        if (t > 0) {
          real dotVV = v.dot(v);
          if (t < dotVV) {
            t /= dotVV;
            diff -= t * v;
            m_cachedBC.m_usedVertices.usedVertexA = true;
            m_cachedBC.m_usedVertices.usedVertexB = true;
          } else {
            t = 1;
            diff -= v;
            // reduce to 1 point
            m_cachedBC.m_usedVertices.usedVertexB = true;
          }
        } else {
          t = 0;
          // reduce to 1 point
          m_cachedBC.m_usedVertices.usedVertexA = true;
        }
        m_cachedBC.setBarycentricCoordinates(1 - t, t);
        nearest = from + t * v;

        m_cachedP1 = m_simplexPointsP[0] + t * (m_simplexPointsP[1] - m_simplexPointsP[0]);
        m_cachedP2 = m_simplexPointsQ[0] + t * (m_simplexPointsQ[1] - m_simplexPointsQ[0]);
        m_cachedV = m_cachedP1 - m_cachedP2;

        reduceVertices(m_cachedBC.m_usedVertices);

        m_cachedValidClosest = m_cachedBC.isValid();
        break;
      }
      case 3: {
        // closest point origin from triangle
        real3 p(real(0.), real(0.), real(0.));

        const real3& a = m_simplexVectorW[0];
        const real3& b = m_simplexVectorW[1];
        const real3& c = m_simplexVectorW[2];

        closestPtPointTriangle(p, a, b, c, m_cachedBC);
        m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
                     m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
                     m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2];

        m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
                     m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
                     m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2];

        m_cachedV = m_cachedP1 - m_cachedP2;

        reduceVertices(m_cachedBC.m_usedVertices);
        m_cachedValidClosest = m_cachedBC.isValid();

        break;
      }
      case 4: {
        real3 p(real(0.), real(0.), real(0.));

        const real3& a = m_simplexVectorW[0];
        const real3& b = m_simplexVectorW[1];
        const real3& c = m_simplexVectorW[2];
        const real3& d = m_simplexVectorW[3];

        bool hasSeperation = closestPtPointTetrahedron(p, a, b, c, d, m_cachedBC);

        if (hasSeperation) {
          m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
                       m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
                       m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2] +
                       m_simplexPointsP[3] * m_cachedBC.m_barycentricCoords[3];

          m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
                       m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
                       m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2] +
                       m_simplexPointsQ[3] * m_cachedBC.m_barycentricCoords[3];

          m_cachedV = m_cachedP1 - m_cachedP2;
          reduceVertices(m_cachedBC.m_usedVertices);
        } else {
          //             printf("sub distance got penetration\n");

          if (m_cachedBC.m_degenerate) {
            m_cachedValidClosest = false;
          } else {
            m_cachedValidClosest = true;
            // degenerate case == false, penetration = true + zero
            m_cachedV = R3(real(0.), real(0.), real(0.));
          }
          break;
        }

        m_cachedValidClosest = m_cachedBC.isValid();

        // closest point origin from tetrahedron
        break;
      }
      default: { m_cachedValidClosest = false; }
    };
  }

  return m_cachedValidClosest;
}

// return/calculate the closest vertex
bool ChCNarrowphaseVoronoiSimplex::closest(real3& v) {
  bool succes = updateClosestVectorAndPoints();
  v = m_cachedV;
  return succes;
}

real ChCNarrowphaseVoronoiSimplex::maxVertex() {
  int i, numverts = numVertices();
  real maxV = real(0.);
  for (i = 0; i < numverts; i++) {
    real curLen2 = m_simplexVectorW[i].length2();
    if (maxV < curLen2)
      maxV = curLen2;
  }
  return maxV;
}

// return the current simplex
int ChCNarrowphaseVoronoiSimplex::getSimplex(real3* pBuf, real3* qBuf, real3* yBuf) const {
  int i;
  for (i = 0; i < numVertices(); i++) {
    yBuf[i] = m_simplexVectorW[i];
    pBuf[i] = m_simplexPointsP[i];
    qBuf[i] = m_simplexPointsQ[i];
  }
  return numVertices();
}

bool ChCNarrowphaseVoronoiSimplex::inSimplex(const real3& w) {
  bool found = false;
  int i, numverts = numVertices();
  // real maxV = real(0.);

  // w is in the current (reduced) simplex
  for (i = 0; i < numverts; i++) {
#ifdef BT_USE_EQUAL_VERTEX_THRESHOLD
    if (m_simplexVectorW[i].distance2(w) <= m_equalVertexThreshold)
#else
    if (m_simplexVectorW[i] == w)
#endif
      found = true;
  }

  // check in case lastW is already removed
  if (w == m_lastW)
    return true;

  return found;
}

void ChCNarrowphaseVoronoiSimplex::backup_closest(real3& v) {
  v = m_cachedV;
}

bool ChCNarrowphaseVoronoiSimplex::emptySimplex() const {
  return (numVertices() == 0);
}

void ChCNarrowphaseVoronoiSimplex::compute_points(real3& p1, real3& p2) {
  updateClosestVectorAndPoints();
  p1 = m_cachedP1;
  p2 = m_cachedP2;
}

bool ChCNarrowphaseVoronoiSimplex::closestPtPointTriangle(const real3& p,
                                                          const real3& a,
                                                          const real3& b,
                                                          const real3& c,
                                                          btSubSimplexClosestResult& result) {
  result.m_usedVertices.reset();

  // Check if P in vertex region outside A
  real3 ab = b - a;
  real3 ac = c - a;
  real3 ap = p - a;
  real d1 = ab.dot(ap);
  real d2 = ac.dot(ap);
  if (d1 <= real(0.0) && d2 <= real(0.0)) {
    result.m_closestPointOnSimplex = a;
    result.m_usedVertices.usedVertexA = true;
    result.setBarycentricCoordinates(1, 0, 0);
    return true;  // a; // barycentric coordinates (1,0,0)
  }

  // Check if P in vertex region outside B
  real3 bp = p - b;
  real d3 = ab.dot(bp);
  real d4 = ac.dot(bp);
  if (d3 >= real(0.0) && d4 <= d3) {
    result.m_closestPointOnSimplex = b;
    result.m_usedVertices.usedVertexB = true;
    result.setBarycentricCoordinates(0, 1, 0);

    return true;  // b; // barycentric coordinates (0,1,0)
  }
  // Check if P in edge region of AB, if so return projection of P onto AB
  real vc = d1 * d4 - d3 * d2;
  if (vc <= real(0.0) && d1 >= real(0.0) && d3 <= real(0.0)) {
    real v = d1 / (d1 - d3);
    result.m_closestPointOnSimplex = a + v * ab;
    result.m_usedVertices.usedVertexA = true;
    result.m_usedVertices.usedVertexB = true;
    result.setBarycentricCoordinates(1 - v, v, 0);
    return true;
    // return a + v * ab; // barycentric coordinates (1-v,v,0)
  }

  // Check if P in vertex region outside C
  real3 cp = p - c;
  real d5 = ab.dot(cp);
  real d6 = ac.dot(cp);
  if (d6 >= real(0.0) && d5 <= d6) {
    result.m_closestPointOnSimplex = c;
    result.m_usedVertices.usedVertexC = true;
    result.setBarycentricCoordinates(0, 0, 1);
    return true;  // c; // barycentric coordinates (0,0,1)
  }

  // Check if P in edge region of AC, if so return projection of P onto AC
  real vb = d5 * d2 - d1 * d6;
  if (vb <= real(0.0) && d2 >= real(0.0) && d6 <= real(0.0)) {
    real w = d2 / (d2 - d6);
    result.m_closestPointOnSimplex = a + w * ac;
    result.m_usedVertices.usedVertexA = true;
    result.m_usedVertices.usedVertexC = true;
    result.setBarycentricCoordinates(1 - w, 0, w);
    return true;
    // return a + w * ac; // barycentric coordinates (1-w,0,w)
  }

  // Check if P in edge region of BC, if so return projection of P onto BC
  real va = d3 * d6 - d5 * d4;
  if (va <= real(0.0) && (d4 - d3) >= real(0.0) && (d5 - d6) >= real(0.0)) {
    real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

    result.m_closestPointOnSimplex = b + w * (c - b);
    result.m_usedVertices.usedVertexB = true;
    result.m_usedVertices.usedVertexC = true;
    result.setBarycentricCoordinates(0, 1 - w, w);
    return true;
    // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
  }

  // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
  real denom = real(1.0) / (va + vb + vc);
  real v = vb * denom;
  real w = vc * denom;

  result.m_closestPointOnSimplex = a + ab * v + ac * w;
  result.m_usedVertices.usedVertexA = true;
  result.m_usedVertices.usedVertexB = true;
  result.m_usedVertices.usedVertexC = true;
  result.setBarycentricCoordinates(1 - v - w, v, w);

  return true;
  // return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = real(1.0) - v - w
}

/// Test if point p and d lie on opposite sides of plane through abc
int ChCNarrowphaseVoronoiSimplex::pointOutsideOfPlane(const real3& p,
                                                      const real3& a,
                                                      const real3& b,
                                                      const real3& c,
                                                      const real3& d) {
  real3 normal = (b - a).cross(c - a);

  real signp = (p - a).dot(normal);  // [AP AB AC]
  real signd = (d - a).dot(normal);  // [AD AB AC]

#ifdef CATCH_DEGENERATE_TETRAHEDRON
#ifdef BT_USE_DOUBLE_PRECISION
  if (signd * signd < (real(1e-8) * real(1e-8))) {
    return -1;
  }
#else
  if (signd * signd < (real(1e-4) * real(1e-4))) {
    //    printf("affine dependent/degenerate\n");//
    return -1;
  }
#endif

#endif
  // Points on opposite sides if expression signs are opposite
  return signp * signd < real(0.);
}

bool ChCNarrowphaseVoronoiSimplex::closestPtPointTetrahedron(const real3& p,
                                                             const real3& a,
                                                             const real3& b,
                                                             const real3& c,
                                                             const real3& d,
                                                             btSubSimplexClosestResult& finalResult) {
  btSubSimplexClosestResult tempResult;

  // Start out assuming point inside all halfspaces, so closest to itself
  finalResult.m_closestPointOnSimplex = p;
  finalResult.m_usedVertices.reset();
  finalResult.m_usedVertices.usedVertexA = true;
  finalResult.m_usedVertices.usedVertexB = true;
  finalResult.m_usedVertices.usedVertexC = true;
  finalResult.m_usedVertices.usedVertexD = true;

  int pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
  int pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
  int pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
  int pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);

  if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0) {
    finalResult.m_degenerate = true;
    return false;
  }

  if (!pointOutsideABC && !pointOutsideACD && !pointOutsideADB && !pointOutsideBDC) {
    return false;
  }

  real bestSqDist = FLT_MAX;
  // If point outside face abc then compute closest point on abc
  if (pointOutsideABC) {
    closestPtPointTriangle(p, a, b, c, tempResult);
    real3 q = tempResult.m_closestPointOnSimplex;

    real sqDist = (q - p).dot(q - p);
    // Update best closest point if (squared) distance is less than current best
    if (sqDist < bestSqDist) {
      bestSqDist = sqDist;
      finalResult.m_closestPointOnSimplex = q;
      // convert result bitmask!
      finalResult.m_usedVertices.reset();
      finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
      finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexB;
      finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
      finalResult.setBarycentricCoordinates(tempResult.m_barycentricCoords[VERTA],
                                            tempResult.m_barycentricCoords[VERTB],
                                            tempResult.m_barycentricCoords[VERTC],
                                            0);
    }
  }

  // Repeat test for face acd
  if (pointOutsideACD) {
    closestPtPointTriangle(p, a, c, d, tempResult);
    real3 q = tempResult.m_closestPointOnSimplex;
    // convert result bitmask!

    real sqDist = (q - p).dot(q - p);
    if (sqDist < bestSqDist) {
      bestSqDist = sqDist;
      finalResult.m_closestPointOnSimplex = q;
      finalResult.m_usedVertices.reset();
      finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;

      finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexB;
      finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexC;
      finalResult.setBarycentricCoordinates(tempResult.m_barycentricCoords[VERTA],
                                            0,
                                            tempResult.m_barycentricCoords[VERTB],
                                            tempResult.m_barycentricCoords[VERTC]);
    }
  }
  // Repeat test for face adb

  if (pointOutsideADB) {
    closestPtPointTriangle(p, a, d, b, tempResult);
    real3 q = tempResult.m_closestPointOnSimplex;
    // convert result bitmask!

    real sqDist = (q - p).dot(q - p);
    if (sqDist < bestSqDist) {
      bestSqDist = sqDist;
      finalResult.m_closestPointOnSimplex = q;
      finalResult.m_usedVertices.reset();
      finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
      finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexC;

      finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;
      finalResult.setBarycentricCoordinates(tempResult.m_barycentricCoords[VERTA],
                                            tempResult.m_barycentricCoords[VERTC],
                                            0,
                                            tempResult.m_barycentricCoords[VERTB]);
    }
  }
  // Repeat test for face bdc

  if (pointOutsideBDC) {
    closestPtPointTriangle(p, b, d, c, tempResult);
    real3 q = tempResult.m_closestPointOnSimplex;
    // convert result bitmask!
    real sqDist = (q - p).dot(q - p);
    if (sqDist < bestSqDist) {
      bestSqDist = sqDist;
      finalResult.m_closestPointOnSimplex = q;
      finalResult.m_usedVertices.reset();
      //
      finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexA;
      finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
      finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;

      finalResult.setBarycentricCoordinates(0,
                                            tempResult.m_barycentricCoords[VERTA],
                                            tempResult.m_barycentricCoords[VERTC],
                                            tempResult.m_barycentricCoords[VERTB]);
    }
  }

  // help! we ended up full !

  if (finalResult.m_usedVertices.usedVertexA && finalResult.m_usedVertices.usedVertexB &&
      finalResult.m_usedVertices.usedVertexC && finalResult.m_usedVertices.usedVertexD) {
    return true;
  }

  return true;
}
