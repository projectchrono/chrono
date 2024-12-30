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

#ifndef CHOBJSBOXSHAPE_H
#define CHOBJSBOXSHAPE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChBox.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for a box shape that can be visualized in some way.
class ChApi ChVisualShapeBox : public ChVisualShape {
  public:
    ChVisualShapeBox();
    ChVisualShapeBox(double length_x, double length_y, double length_z);
    ChVisualShapeBox(const ChVector3d& lengths);
    ChVisualShapeBox(const ChBox& box);

    ~ChVisualShapeBox() {}

    /// Access the box geometry.
    ChBox& GetGeometry() { return gbox; }

    /// Get the box half-lengths.
    const ChVector3d& GetHalflengths() const { return gbox.GetHalflengths(); }

    /// Get the box dimensions.
    ChVector3d GetLengths() const { return gbox.GetLengths(); }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return gbox.GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    ChBox gbox;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeBox, 0)

}  // end namespace chrono

#endif
