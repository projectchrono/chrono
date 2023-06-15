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

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChEllipsoid.h"

namespace chrono {

/// Class for referencing an ellipsoid shape that can be visualized in some way.
class ChApi ChEllipsoidShape : public ChVisualShape {
  public:
    ChEllipsoidShape();
    ChEllipsoidShape(double axis_x, double axis_y, double axis_z);
    ChEllipsoidShape(const ChVector<>& axes);
    ChEllipsoidShape(const geometry::ChEllipsoid& ellipsoid);

    ~ChEllipsoidShape(){};

    // Access the ellipsoid geometry.
    geometry::ChEllipsoid& GetGeometry() { return gellipsoid; }

    /// Get the ellipsoid semiaxes.
    const ChVector<>& GetSemiaxes() const { return gellipsoid.GetSemiaxes(); }

    /// Get the ellipsoid axes.
    ChVector<> GetAxes() const { return gellipsoid.GetAxes(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChEllipsoid gellipsoid;
};

CH_CLASS_VERSION(ChEllipsoidShape, 0)

}  // end namespace chrono

#endif
