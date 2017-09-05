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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cstdio>

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace geometry {

// Register into the object factory, to enable run-time dynamic creation and persistence
//CH_FACTORY_REGISTER(ChGeometry)  // NO! Abstract class!

void ChGeometry::InflateBoundingBox(double& xmin,
                                    double& xmax,
                                    double& ymin,
                                    double& ymax,
                                    double& zmin,
                                    double& zmax,
                                    ChMatrix33<>* Rot) const {
    double bxmin, bxmax, bymin, bymax, bzmin, bzmax;
    GetBoundingBox(bxmin, bxmax, bymin, bymax, bzmin, bzmax, Rot);
    if (xmin > bxmin)
        xmin = bxmin;
    if (ymin > bymin)
        ymin = bymin;
    if (zmin > bzmin)
        zmin = bzmin;
    if (xmax < bxmax)
        xmax = bxmax;
    if (ymax < bymax)
        ymax = bymax;
    if (zmax < bzmax)
        zmax = bzmax;
}

double ChGeometry::Size() const {
    double bxmin, bxmax, bymin, bymax, bzmin, bzmax;
    GetBoundingBox(bxmin, bxmax, bymin, bymax, bzmin, bzmax);
    return sqrt(pow((0.5 * (bxmax - bxmin)), 2) + pow((0.5 * (bymax - bymin)), 2) + pow((0.5 * (bzmax - bzmin)), 2));
}



}  // end namespace geometry
}  // end namespace chrono
