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

#ifndef CHROUNDEDBOXSHAPE_H
#define CHROUNDEDBOXSHAPE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChRoundedBox.h"

namespace chrono {

/// Class for referencing a rounded box shape that can be visualized in some way.
class ChApi ChRoundedBoxShape : public ChVisualShape {
  public:
    ChRoundedBoxShape();
    ChRoundedBoxShape(const geometry::ChRoundedBox& box);

    ~ChRoundedBoxShape() {}

    // Access the rounded box geometry
    geometry::ChRoundedBox& GetRoundedBoxGeometry() { return groundedbox; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    geometry::ChRoundedBox groundedbox;
};

CH_CLASS_VERSION(ChRoundedBoxShape, 0)

}  // end namespace chrono

#endif
