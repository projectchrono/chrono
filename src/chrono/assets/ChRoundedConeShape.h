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

#ifndef CHROUNDEDCONESHAPE_H
#define CHROUNDEDCONESHAPE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChRoundedCone.h"

namespace chrono {

/// Class for referencing a rounded cone shape that can be visualized in some way.
class ChApi ChRoundedConeShape : public ChVisualShape {
  public:
    ChRoundedConeShape();
    ChRoundedConeShape(const geometry::ChRoundedCone& cone);

    ~ChRoundedConeShape() {}

    // Access the rounded cone geometry.
    geometry::ChRoundedCone& GetRoundedConeGeometry() { return groundedcone; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    geometry::ChRoundedCone groundedcone;
};

CH_CLASS_VERSION(ChRoundedConeShape, 0)

}  // end namespace chrono

#endif
