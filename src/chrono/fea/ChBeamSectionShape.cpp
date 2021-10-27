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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/fea/ChBeamSectionShape.h"

namespace chrono {
namespace fea {

/// Returns the axis-aligned bounding box (assuming axes of local reference of the section)
/// This functions has many uses, ex.for drawing, optimizations, collisions.
/// We provide a fallback default implementation that iterates over all points thanks to GetPoints(),
/// but one could override this if a more efficient implementaiton is possible (ex for circular beams, etc.)

void ChBeamSectionShape::GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const {
    ymin = 1e30;
    ymax = -1e30;
    zmin = 1e30;
    zmax = -1e30;
    for (int nl = 0; nl < GetNofLines(); ++nl) {
        std::vector<ChVector<>> mpoints(GetNofPoints(nl));
        GetPoints(nl, mpoints);
        for (int np = 0; np < GetNofPoints(nl); ++nl) {
            if (mpoints[np].y() < ymin)
                ymin = mpoints[np].y();
            if (mpoints[np].y() > ymax)
                ymax = mpoints[np].y();
            if (mpoints[np].z() < zmin)
                zmin = mpoints[np].z();
            if (mpoints[np].z() > zmax)
                zmax = mpoints[np].z();
        }
    }
}

/// Returns the axis-aligned bounding box (assuming axes of local reference of the section)
/// This functions has many uses, ex.for drawing, optimizations, collisions.

void ChBeamSectionShapeCircular::GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const {
    ymin = -radius;
    ymax = radius;
    zmin = -radius;
    zmax = radius;
}

// internal: update internal precomputed vertex arrays

void ChBeamSectionShapeCircular::UpdateProfile() {
    points.resize(resolution + 1);
    normals.resize(resolution + 1);
    for (size_t is = 0; is < points.size(); ++is) {
        double sangle = CH_C_2PI * ((double)is / (double)resolution);
        points[is] = ChVector<>(0, cos(sangle) * radius, sin(sangle) * radius);
        normals[is] = ChVector<>(0, cos(sangle), sin(sangle));
    }
}

/// Returns the axis-aligned bounding box (assuming axes of local reference of the section)

void ChBeamSectionShapeRectangular::GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const {
    ymin = -y_thick * 0.5;
    ymax = y_thick * 0.5;
    zmin = -z_thick * 0.5;
    zmax = z_thick * 0.5;
}

// internal: update internal precomputed vertex arrays

void ChBeamSectionShapeRectangular::UpdateProfile() {
    ml_points.resize(4);
    ml_normals.resize(4);

    double y_thick_half = 0.5 * y_thick;
    double z_thick_half = 0.5 * z_thick;

    ml_points[0].resize(2);
    ml_points[0][0].Set(0, -y_thick_half, -z_thick_half);
    ml_points[0][1].Set(0, y_thick_half, -z_thick_half);

    ml_points[1].resize(2);
    ml_points[1][0].Set(0, y_thick_half, -z_thick_half);
    ml_points[1][1].Set(0, y_thick_half, z_thick_half);

    ml_points[2].resize(2);
    ml_points[2][0].Set(0, y_thick_half, z_thick_half);
    ml_points[2][1].Set(0, -y_thick_half, z_thick_half);

    ml_points[3].resize(2);
    ml_points[3][0].Set(0, -y_thick_half, z_thick_half);
    ml_points[3][1].Set(0, -y_thick_half, -z_thick_half);

    ml_normals[0].resize(2);
    ml_normals[0][0].Set(0, 0, -1);
    ml_normals[0][1].Set(0, 0, -1);

    ml_normals[1].resize(2);
    ml_normals[1][0].Set(0, 1, 0);
    ml_normals[1][1].Set(0, 1, 0);

    ml_normals[2].resize(2);
    ml_normals[2][0].Set(0, 0, 1);
    ml_normals[2][1].Set(0, 0, 1);

    ml_normals[3].resize(2);
    ml_normals[3][0].Set(0, -1, 0);
    ml_normals[3][1].Set(0, -1, 0);
}

// internal: update internal precomputed vertex arrays, computing normals by smoothing segments

void ChBeamSectionShapePolyline::UpdateProfile() {
    ml_normals.resize(ml_points.size());

    for (int il = 0; il < ml_points.size(); ++il) {
        ml_normals[il].resize(ml_points[il].size());
        double dy, dz, len;
        for (int ip = 1; ip < ml_points[il].size() - 1; ++ip) {
            dy = ml_points[il][ip + 1].y() - ml_points[il][ip - 1].y();
            dz = ml_points[il][ip + 1].z() - ml_points[il][ip - 1].z();
            len = sqrt(dy * dy + dz * dz);
            ml_normals[il][ip].y() = -dz / len;
            ml_normals[il][ip].z() = dy / len;
        }
        dy = ml_points[il][1].y() - ml_points[il][0].y();
        dz = ml_points[il][1].z() - ml_points[il][0].z();
        len = sqrt(dy * dy + dz * dz);
        ml_normals[il][0].y() = -dz / len;
        ml_normals[il][0].z() = dy / len;
        dy = ml_points[il][ml_points[il].size() - 1].y() - ml_points[il][ml_points[il].size() - 2].y();
        dz = ml_points[il][ml_points[il].size() - 1].z() - ml_points[il][ml_points[il].size() - 2].z();
        len = sqrt(dy * dy + dz * dz);
        ml_normals[il][ml_points[il].size() - 1].y() = -dz / len;
        ml_normals[il][ml_points[il].size() - 1].z() = dy / len;
    }
}

}  // end namespace fea
}  // end namespace chrono
