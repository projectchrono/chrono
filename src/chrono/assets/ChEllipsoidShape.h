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

#ifndef CHOBJELLIPSOIDSHAPE_H
#define CHOBJELLIPSOIDSHAPE_H

#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChEllipsoid.h"

namespace chrono {

/// Class for referencing an ellipsoid shape that can be visualized in some way.
class ChApi ChEllipsoidShape : public ChVisualization {
  protected:
    geometry::ChEllipsoid gellipsoid;

  public:
    ChEllipsoidShape() {}
    ChEllipsoidShape(const geometry::ChEllipsoid& mellipsoid) : gellipsoid(mellipsoid) {}

    virtual ~ChEllipsoidShape(){};

    // Access the sphere geometry
    geometry::ChEllipsoid& GetEllipsoidGeometry() { return gellipsoid; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChEllipsoidShape, 0)

}  // end namespace chrono

#endif
