// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Simple 2D convex hull class
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>

#include "chrono/utils/ChConvexHull.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------

// Return twice the signed area of the triangle {p1,p2,p3}
double SignedArea(const ChVector2<>& p1, const ChVector2<>& p2, const ChVector2<>& p3) {
    return (p2.y() - p1.y()) * (p3.x() - p2.x()) - (p2.x() - p1.x()) * (p3.y() - p2.y());
}

// Utility function to decide relative orientation of 3 points.
// Returns 0 for colinear points, +1 for clock-wise, -1 for counterclock-wise.
int Orientation(const ChVector2<>& p1, const ChVector2<>& p2, const ChVector2<>& p3) {
    static double eps = 1e-10;
    double val = SignedArea(p1, p2, p3);

    if (val == 0)
        return 0;               // (nearly) colinear
    return (val > 0) ? 1 : -1;  // clock- or counterclock-wise
}

// Return true if point p2 is between points p1 and p3 (the 3 points are assumed collinear).
bool InBetween(const ChVector2<>& p1, const ChVector2<>& p2, const ChVector2<>& p3) {
    bool a = p2.x() >= p1.x() && p2.x() <= p3.x() || p2.x() <= p1.x() && p2.x() >= p3.x();
    bool b = p2.y() >= p1.y() && p2.y() <= p3.y() || p2.y() <= p1.y() && p2.y() >= p3.y();
    return a && b;
}

// -----------------------------------------------------------------------------

ChConvexHull2D::ChConvexHull2D(std::vector<ChVector2<>>& points, Method method) : m_perimeter(0), m_area(0) {
    size_t n = points.size();

    // Special cases (low number of points)
    if (n == 1) {
        m_area = 0;
        m_perimeter = 0;
        m_hull.push_back(points[0]);
        return;
    } else if (n == 2) {
        m_area = 0;
        m_perimeter = (points[1] - points[0]).Length();
        m_hull.push_back(points[0]);
        m_hull.push_back(points[1]);
        return;
    } else if (n == 3) {
        m_area = 0.5 * std::abs(SignedArea(points[0], points[1], points[2]));
        m_perimeter =
            (points[1] - points[0]).Length() + (points[2] - points[1]).Length() + (points[0] - points[2]).Length();
        m_hull.push_back(points[0]);
        m_hull.push_back(points[1]);
        m_hull.push_back(points[2]);
        return;
    }

    switch (method) {
        case JARVIS:
            ComputeJarvis(points, n);
            break;
        case GRAHAM:
            ////ComputeGraham(points, n);
            break;
    }
}

// -----------------------------------------------------------------------------

void ChConvexHull2D::ComputeJarvis(const std::vector<ChVector2<>>& points, size_t n) {
    // Find point with lowest x. Ties are broken for lowest y.
    size_t first = 0;
    for (size_t i = 1; i < points.size(); i++) {
        double dx = points[i].x() - points[first].x();
        if (dx < 0)
            first = i;
        else if (dx == 0 && points[i].y() < points[first].y())
            first = i;
    }
    m_hull.push_back(points[first]);

    size_t crt = first;
    do {
        // Initialize next candidate.
        // Attention: important to consider all points, so must start at 0, but skip crt.
        size_t next = 0;
        while (next == crt)
            next = (next + 1) % n;

        // Find the next point on convex hull.
        for (size_t i = 0; i < n; i++) {
            if (Orientation(points[crt], points[i], points[next]) == -1) {
                next = i;
            }
        }

        // Before adding next to the convex hull, include any other points on the line from
        // crt to next.
        for (size_t i = 0; i < n; i++) {
            if (i != crt && i != next && Orientation(points[crt], points[i], points[next]) == 0 &&
                InBetween(points[crt], points[i], points[next])) {
                m_hull.push_back(points[i]);
            }
        }

        // Add next to the convex hull and update perimeter and area.
        // Note that it is important to sum the signed area (we don't know where the origin falls).
        // Also, we must disregard any intermediate points on the last edge, else we double count.
        m_hull.push_back(points[next]);
        m_perimeter += (points[next] - points[crt]).Length();
        m_area += SignedArea(points[next], points[crt], ChVector2<>(0, 0));

        // Safety check to prevent inifinite loop.
        //// TODO: still some corner cases that are not properly treated!
        if (m_hull.size() > n + 1) {
            std::cout << "\n\nERROR in ChConvexHull2D::ComputeJarvis: infinite loop\n\n" << std::endl;
            return;
        }

        crt = next;
    } while (crt != first);
}

// -----------------------------------------------------------------------------

// Utility struct for comparing points according to polar order.
struct PolarOrder {
    PolarOrder(const ChVector2<> pivot) : m_pivot(pivot) {}
    bool operator()(ChVector2<> a, ChVector2<> b) {
        int order = Orientation(m_pivot, a, b);
        if (order == 0)
            return (a - m_pivot).Length2() < (b - m_pivot).Length2();
        return (order == -1);
    }
    ChVector2<> m_pivot;
};

void ChConvexHull2D::ComputeGraham(std::vector<ChVector2<>>& points, size_t n) {
    // Find point with lowest y. Ties are broken for lowest x.
    size_t first = 0;
    for (size_t i = 1; i < points.size(); i++) {
        double dy = points[i].y() - points[first].y();
        if (dy < 0)
            first = i;
        else if (dy == 0 && points[i].x() < points[first].x())
            first = i;
    }

    // Swap this point in first position
    ChVector2<> tmp = points[0];
    points[0] = points[first];
    points[first] = tmp;

    // Sort the remaining points by polar order about the pivot.
    size_t pivot = 0;
    std::sort(std::next(points.begin()), points.end(), PolarOrder(points[pivot]));

    // Graham scan

    //// TODO
}

}  // end namespace utils
}  // end namespace chrono
