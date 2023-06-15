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

#ifndef CHOBJCONESHAPE_H
#define CHOBJCONESHAPE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChCone.h"

namespace chrono {

/// Class for referencing a cone shape that can be visualized in some way.
class ChApi ChConeShape : public ChVisualShape {
  public:
    ChConeShape();
    ChConeShape(double radius, double height);
    ChConeShape(const geometry::ChCone& cone);

    ~ChConeShape(){};

    /// Access the cone geometry.
    geometry::ChCone& GetGeometry() { return gcone; }

    /// Get the cone radius.
    double GetRadius() const { return gcone.GetRadius(); }

    /// Get the cone height.
    double GetHeight() const { return gcone.GetHeight(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    geometry::ChCone gcone;
};

CH_CLASS_VERSION(ChConeShape, 0)

}  // end namespace chrono

#endif
