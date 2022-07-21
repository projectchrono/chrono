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

#ifndef CHCAPSULESHAPE_H
#define CHCAPSULESHAPE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChCapsule.h"

namespace chrono {

/// Class for referencing a capsule shape that can be visualized in some way.
class ChApi ChCapsuleShape : public ChVisualShape {
  public:
    ChCapsuleShape();
    ChCapsuleShape(const geometry::ChCapsule& cap);

    ~ChCapsuleShape() {}

    // Access the capsule geometry.
    geometry::ChCapsule& GetCapsuleGeometry() { return gcapsule; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    geometry::ChCapsule gcapsule;
};

CH_CLASS_VERSION(ChCapsuleShape, 0)

}  // end namespace chrono

#endif
