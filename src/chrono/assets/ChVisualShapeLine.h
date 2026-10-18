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
    ChVisualShapeLine(std::shared_ptr<ChLine> line);
    virtual ~ChVisualShapeLine() {}

    /// Set the line geometry.
    void SetLineGeometry(std::shared_ptr<ChLine> line) { m_line = line; }

    /// Access the line geometry.
    std::shared_ptr<ChLine> GetLineGeometry() const { return m_line; }

    /// Set line name.
    void SetName(const std::string& name) { m_name = name; }

    /// Get line name.
    const std::string& GetName() const { return m_name; }

    /// Set resolution for line rendering.
    void SetNumRenderPoints(unsigned int n) { m_num_render_points = n; }

    /// Get resolution for line rendering.
    unsigned int GetNumRenderPoints() const { return m_num_render_points; }

    /// Get thickness of line used when rendering (for engines supporting it).
    double GetThickness() const { return m_thickness; }

    /// Set thickness of line used when rendering (for engines supporting it).
    void SetThickness(double thickness) { m_thickness = thickness; }

    /// Get the shape bounding box.
    virtual ChAABB GetBoundingBox() const override { return m_line->GetBoundingBox(); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    std::shared_ptr<ChLine> m_line;    ///< underlying line geometry
    std::string m_name;                ///< asset name
    unsigned int m_num_render_points;  ///< number of points evaluated when rendering
    double m_thickness;                ///< thickness of line when rendering (for rendering engines that support it)
};

/// @} chrono_assets

CH_CLASS_VERSION(ChVisualShapeLine, 0)

}  // end namespace chrono

#endif
