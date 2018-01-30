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

#ifndef CHPATHSHAPE_H
#define CHPATHSHAPE_H

#include "chrono/assets/ChVisualization.h"
#include "chrono/geometry/ChLineArc.h"
#include "chrono/geometry/ChLinePath.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {

/// Class for referencing a ChLinePath that can be visualized in some way.
class ChApi ChPathShape : public ChVisualization {
  public:
    ChPathShape();
    ChPathShape(std::shared_ptr<geometry::ChLinePath>& mpath);
    virtual ~ChPathShape() {}

    /// Access the underlying path geometry.
    std::shared_ptr<geometry::ChLinePath> GetPathGeometry() { return gpath; }

    unsigned int GetNumRenderPoints() const { return npoints; }
    void SetNumRenderPoints(unsigned int n) { npoints = n; }

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    std::shared_ptr<geometry::ChLinePath> gpath;  ///< underlying path geometry
    unsigned int npoints;                         ///< number of points evaluated when rendering
};

CH_CLASS_VERSION(ChPathShape, 0)

}  // end namespace chrono

#endif
