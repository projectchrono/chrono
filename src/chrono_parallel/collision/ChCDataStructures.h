#pragma once

#include "chrono_parallel/math/ChParallelMath.h"

namespace chrono {
namespace collision {

struct ConvexShape {
  shape_type type;  // type of shape
  real3 A;  // location
  real3 B;  // dimensions
  real3 C;  // extra
  quaternion R;  // rotation
  real3* convex;  // pointer to convex data;
  real margin;
};

struct ContactPoint {
  real3 pointA, pointB, normal;
  real depth;
  ContactPoint() {}
  ContactPoint(const real3 &pa, const real3 &pb, const real3 &norm, const real d) {
    pointA = pa;
    pointB = pb;
    normal = norm;
    depth = d;
  }
};

#define MANIFOLD_SIZE 3

struct ContactManifold {
  ContactPoint points[MANIFOLD_SIZE];

  unsigned int num_contact_points;
  ContactManifold() { num_contact_points = 0; }

  int getCacheEntry(ContactPoint& newPoint) {
    real shortestDist = ZERO_EPSILON * 2;  // TODO: SHOULD THIS BE SOMETHIGN ELSE
    int size = num_contact_points;
    int nearestPoint = -1;
    for (int i = 0; i < size; i++) {
      const ContactPoint& mp = points[i];

      real3 diffA = mp.pointA - newPoint.pointA;
      const real distToManiPoint = diffA.dot(diffA);
      if (distToManiPoint < shortestDist) {
        shortestDist = distToManiPoint;
        nearestPoint = i;
      }
    }
    return nearestPoint;
  }

  void replaceContactPoint(ContactPoint& newPt, int& insertIndex) { points[insertIndex] = newPt; }
  int addManifoldPoint(ContactPoint& newPt) {
    if (num_contact_points == MANIFOLD_SIZE) {
      points[0] = newPt;
      return 0;
    } else {
      points[num_contact_points] = newPt;
      num_contact_points++;
      return num_contact_points - 1;
    }
  }

  void addContactPoint(const ConvexShape& shapeA,
                       const ConvexShape& shapeB,
                       const real3& normalOnBInWorld,
                       const real3& pointInWorld,
                       const real& depth) {
    real3 pointA = pointInWorld + normalOnBInWorld * depth;
    real3 localA = TransformParentToLocal(shapeA.A, shapeA.R, pointA);
    real3 localB = TransformParentToLocal(shapeB.A, shapeB.R, pointInWorld);

    ContactPoint newPt(localA, localB, normalOnBInWorld, depth);
    newPt.pointA = pointA;
    newPt.pointB = pointInWorld;
    int insertIndex = getCacheEntry(newPt);
    if (insertIndex >= 0) {
      replaceContactPoint(newPt, insertIndex);
    } else {
      insertIndex = addManifoldPoint(newPt);
    }
  }
};

}  // end namespace collision
}  // end namespace chrono
