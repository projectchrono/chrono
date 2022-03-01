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
    ChConeShape(const geometry::ChCone& cone);

    ~ChConeShape(){};

    // Access the cone geometry.
    geometry::ChCone& GetConeGeometry() { return gcone; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    geometry::ChCone gcone;
};

CH_CLASS_VERSION(ChConeShape, 0)

}  // end namespace chrono

#endif
