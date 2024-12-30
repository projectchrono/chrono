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

#ifndef CH_VISUAL_SHAPE_PATH_H
#define CH_VISUAL_SHAPE_PATH_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChLineArc.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a ChLinePath that can be visualized in some way.
class ChApi ChVisualShapePath : public ChVisualShape {
  public:
    ChVisualShapePath();
    ChVisualShapePath(std::shared_ptr<ChLinePath>& mpath);
    ~ChVisualShapePath() {}

    /// Access the underlying path geometry.
    std::shared_ptr<ChLinePath> GetPathGeometry() { return gpath; }

    unsigned int GetNumRenderPoints() const { return npoints; }
    void SetNumRenderPoints(unsigned int n) { npoints = n; }

    double GetThickness() const { return thickness; }
    void SetThickness(double mt) { thickness = mt; }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return gpath->GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    std::shared_ptr<ChLinePath> gpath;  ///< underlying path geometry
    unsigned int npoints;               ///< number of points evaluated when rendering
    double thickness;                   ///< thickness of line when rendering (for rendering engines that support it)
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapePath, 0)

}  // end namespace chrono

#endif
