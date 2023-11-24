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

#ifndef CH_VISUAL_SHAPE_LINE_H
#define CH_VISUAL_SHAPE_LINE_H

#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChLineSegment.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Class for referencing a ChLine that can be visualized in some way.
class ChApi ChVisualShapeLine : public ChVisualShape {
  public:
    ChVisualShapeLine();
    ChVisualShapeLine(std::shared_ptr<geometry::ChLine>& mline);
    virtual ~ChVisualShapeLine() {}

    /// Access the line geometry.
    std::shared_ptr<geometry::ChLine> GetLineGeometry() { return gline; }

    /// Set the line geometry.
    void SetLineGeometry(std::shared_ptr<geometry::ChLine> mline) { gline = mline; }

    const std::string& GetName() const { return name; }
    void SetName(const std::string& mname) { name = mname; }

    unsigned int GetNumRenderPoints() const { return npoints; }
    void SetNumRenderPoints(unsigned int n) { npoints = n; }

    double GetThickness() const { return thickness; }
    void SetThickness(double mt) { thickness = mt; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  protected:
    std::shared_ptr<geometry::ChLine> gline;  ///< underlying line geometry
    std::string name;                         ///< asset name
    unsigned int npoints;                     ///< number of points evaluated when rendering
    double thickness;                         ///< thickness of line when rendering (for rendering engines that support it)
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeLine, 0)

}  // end namespace chrono

#endif
